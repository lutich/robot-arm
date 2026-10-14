"""Named cameras, offscreen rendering, the fit of the real camera's view, and robot-arm's
CameraSource port so the sandbox serves /api/camera/... exactly like the Pi.
"""
import io
import json
import math
import queue
import threading
import time
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image

from armsim.robotarm import install

install()
from roboter_arm.control.domain.camera import CameraSample, capabilities, defaults, validate_changes  # noqa: E402

MM = 0.001


# ---- fitting the real camera -------------------------------------------------------------
def load_calibration(path=None):
    path = Path(path) if path else install() / 'config' / 'camera-table-xy' / 'calibration.json'
    return json.loads(path.read_text())


def marker_points(calibration):
    """Marker centres in table millimetres and their pixels through the saved homography."""
    H = np.array(calibration['H_table_to_image'], dtype=float)
    table = np.array([m['center_xy_mm'] for m in calibration['table_layout']['markers']], dtype=float)
    pixels = homography_project(H, table)
    return table, pixels


def homography_project(H, table_xy):
    ones = np.ones((len(table_xy), 1))
    p = (H @ np.hstack([table_xy, ones]).T).T
    return p[:, :2] / p[:, 2:3]


def pose_from_homography(H, f, cx, cy):
    """Camera rotation (world->camera, OpenCV axes) and translation in mm for a pinhole with focal f."""
    K_inv = np.array([[1 / f, 0, -cx / f], [0, 1 / f, -cy / f], [0, 0, 1]])
    M = K_inv @ H
    scale = 2 / (np.linalg.norm(M[:, 0]) + np.linalg.norm(M[:, 1]))
    r1, r2, t = M[:, 0] * scale, M[:, 1] * scale, M[:, 2] * scale
    if t[2] < 0:
        r1, r2, t = -r1, -r2, -t
    R = np.column_stack([r1, r2, np.cross(r1, r2)])
    U, _, Vt = np.linalg.svd(R)
    R = U @ Vt
    if np.linalg.det(R) < 0:
        U[:, -1] *= -1
        R = U @ Vt
    return R, t


def pinhole_project(R, t, f, cx, cy, points_mm):
    cam = (R @ np.asarray(points_mm, dtype=float).T).T + t
    return np.column_stack([f * cam[:, 0] / cam[:, 2] + cx, f * cam[:, 1] / cam[:, 2] + cy])


def fit_oak_camera(calibration=None, width=1280, height=720):
    """Fit position, orientation and focal length so the markers land on their calibrated pixels."""
    calibration = calibration or load_calibration()
    H = np.array(calibration['H_table_to_image'], dtype=float)
    table, pixels = marker_points(calibration)
    points = np.column_stack([table, np.zeros(len(table))])
    cx, cy = width / 2, height / 2

    def error(f):
        R, t = pose_from_homography(H, f, cx, cy)
        return float(np.sqrt(np.mean(np.sum((pinhole_project(R, t, f, cx, cy, points) - pixels) ** 2, axis=1))))

    best = min(range(300, 4000, 10), key=error)
    best = min(range(best - 10, best + 11), key=error)
    R, t = pose_from_homography(H, best, cx, cy)
    position = (-R.T @ t) * MM
    xyaxes = np.concatenate([R[0, :], -R[1, :]])      # MuJoCo: x right, y up, looks along -z
    return dict(pos=position.tolist(), xyaxes=xyaxes.tolist(), fovy=math.degrees(2 * math.atan(height / (2 * best))),
                focal_px=best, error_px=error(best), width=width, height=height)


def camera_pixel(spec, point, width=None, height=None):
    """Pixel of a world point (metres) for a fitted camera spec, for tests."""
    width, height = width or spec['width'], height or spec['height']
    pos = np.array(spec['pos'])
    x, y = np.array(spec['xyaxes'][:3]), np.array(spec['xyaxes'][3:])
    z = np.cross(x, y)                     # camera looks along -z
    d = np.asarray(point) - pos
    cam = np.array([d @ x, d @ y, d @ z])
    f = (height / 2) / math.tan(math.radians(spec['fovy']) / 2)
    return np.array([width / 2 + f * cam[0] / -cam[2], height / 2 - f * cam[1] / -cam[2]])


def scene_cameras(scene, calibration=None, fit=True):
    """The scene's named cameras, with 'oak' replaced by the fitted real camera when possible."""
    cameras = dict(scene.get('cameras', {}))
    if fit:
        try:
            spec = fit_oak_camera(calibration, scene['render']['width'], scene['render']['height'])
            cameras['oak'] = dict(pos=spec['pos'], xyaxes=spec['xyaxes'], fovy=spec['fovy'])
            cameras['oak_fit'] = spec
        except (OSError, RuntimeError, ValueError, KeyError):
            pass
    return cameras


# ---- rendering ---------------------------------------------------------------------------
class Renderer:
    """Offscreen rendering on one dedicated thread: the GL context is only ever used there."""

    def __init__(self, physics, width=None, height=None):
        self.physics = physics
        render = physics.scene.get('render', {})
        self.width, self.height = width or render.get('width', 1280), height or render.get('height', 720)
        self.jobs = queue.Queue()
        self.ready = threading.Event()
        self.failure = None
        self.thread = threading.Thread(target=self._run, name='renderer', daemon=True)
        self.thread.start()
        self.ready.wait(30)
        if self.failure is not None:
            raise self.failure

    def _run(self):
        try:
            renderer = mujoco.Renderer(self.physics.model, height=self.height, width=self.width)
        except Exception as error:      # surfaced to the constructor
            self.failure = error
            self.ready.set()
            return
        self.ready.set()
        while True:
            job = self.jobs.get()
            if job is None:
                break
            camera, done, result = job
            try:
                with self.physics.lock:
                    renderer.update_scene(self.physics.data, camera=camera)
                result['image'] = renderer.render().copy()
            except Exception as error:
                result['error'] = error
            done.set()
        renderer.close()

    def names(self):
        return [self.physics.model.camera(i).name for i in range(self.physics.model.ncam)]

    def render(self, camera='oak'):
        if camera not in self.names():
            raise ValueError(f'Unknown camera {camera!r}; choose from {self.names()}')
        if not self.thread.is_alive():
            raise RuntimeError('Renderer is closed')
        done, result = threading.Event(), {}
        self.jobs.put((camera, done, result))
        if not done.wait(30):
            raise RuntimeError('Rendering timed out')
        if 'error' in result:
            raise result['error']
        return result['image']

    def image(self, camera='oak'):
        return Image.fromarray(self.render(camera))

    def png(self, camera='oak'):
        buffer = io.BytesIO()
        self.image(camera).save(buffer, format='PNG')
        return buffer.getvalue()

    def jpeg(self, camera='oak', quality=85):
        buffer = io.BytesIO()
        self.image(camera).save(buffer, format='JPEG', quality=quality)
        return buffer.getvalue()

    def snapshot(self, path, camera='oak', sidecar=None):
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        self.image(camera).save(path)
        if sidecar is not None:
            path.with_suffix('.json').write_text(json.dumps(sidecar, indent=2) + '\n')
        return path

    def close(self):
        if self.thread.is_alive():
            self.jobs.put(None)
            self.thread.join(5)


class MujocoCameraSource:
    """robot-arm's CameraSource port: the sandbox renders the fitted 'oak' view as JPEG frames."""
    kind = 'mujoco'

    def __init__(self, renderer, camera='oak', fps=5.0, clock=time.monotonic):
        self.renderer, self.camera, self.fps, self.clock = renderer, camera, fps, clock
        self.caps = capabilities(has_autofocus=False)
        self.requested = defaults(self.caps)
        self.sequence = 0
        self.next_frame = 0.0
        self.opened = False

    def open(self):
        self.opened = True
        self.sequence = 0
        self.next_frame = self.clock()
        return self.caps

    def apply(self, changes):
        validate_changes(changes, self.caps)
        self.requested.update(changes)

    def read(self):
        if not self.opened:
            raise RuntimeError('Camera source is closed')
        wait = self.next_frame - self.clock()
        if wait > 0:
            time.sleep(min(wait, 1 / self.fps))
        self.next_frame = self.clock() + 1 / self.fps
        jpeg = self.renderer.jpeg(self.camera)
        self.sequence += 1
        exposure, balance = self.requested.get('exposure', {}), self.requested.get('white_balance', {})
        observed = dict(width=self.renderer.width, height=self.renderer.height, sensor_fps=self.fps,
                        source_sequence=self.sequence, camera=self.camera, source=self.kind,
                        time_us=exposure.get('time_us'), iso=exposure.get('iso'),
                        temperature_k=balance.get('temperature_k'), lens_position=None)
        return CameraSample(jpeg=jpeg, observed=observed, age=0.0)

    def close(self):
        self.opened = False

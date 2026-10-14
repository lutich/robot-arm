"""Sandbox launcher: robot-arm's Session and HTTP server with the MuJoCo driver and camera.

    python -m armsim.server [--port 8765] [--host 127.0.0.1] [--no-camera-fit] [--viewer]

The robot-arm web page, its /simulation viewer and any API client then drive the physics.
"""
import argparse
import signal
import threading
import time

from pydantic import Field

from armsim.cameras import MujocoCameraSource, Renderer, scene_cameras
from armsim.config import ROOT, load_robot, load_scene
from armsim.driver import MujocoDriver
from armsim.client import ArmError, SessionArmClient
from armsim.mover import Mover
from armsim.physics import Physics, RealtimeStepper, reset_blocks
from armsim.robotarm import PARK, commit, install
from armsim.session import sandbox_session
from armsim.tasks import PhysicsWorld

install()
from fastapi import Request  # noqa: E402
from fastapi.responses import JSONResponse, Response  # noqa: E402
from roboter_arm.control.application.camera import CameraService  # noqa: E402
from roboter_arm.control.presentation.api_models import Body  # noqa: E402
from roboter_arm.control.presentation.http_api import Server, action  # noqa: E402


def _static(path, media_type):
    def endpoint():
        return Response(path.read_bytes(), media_type=media_type)
    return endpoint


class MovePose(Body):
    counts: list[int] = Field(min_length=6, max_length=6)
    speed: int | None = None


class MoveTo(Body):
    x: float
    y: float
    z: float = 0.01325
    yaw: float = 0.0
    lower: bool = True
    dry: bool = False
    path: str = 'direct'



class Sandbox:
    """Everything the sandbox server needs, built once; `serve` runs it, tests use the parts."""

    def __init__(self, *, fit_camera=True, artifacts=None, robot=None, scene=None):
        self.robot, self.scene = robot or load_robot(), scene or load_scene()
        self.cameras = scene_cameras(self.scene, fit=fit_camera)
        self.physics = Physics(self.robot, self.scene, {k: v for k, v in self.cameras.items() if k != 'oak_fit'})
        self.physics.place_arm(PARK)          # the real arm is confirmed in PARK before power on
        self.driver = MujocoDriver(self.physics)
        artifacts = artifacts or ROOT / 'artifacts'
        self.session = sandbox_session(self.driver, directory=artifacts / 'poses', demo_directory=artifacts / 'demos')
        self.renderer = Renderer(self.physics)
        self.world = PhysicsWorld(self.physics, self.scene)
        self.mover = Mover(SessionArmClient(self.session), self.robot, self.scene)
        self.stepper = RealtimeStepper(self.physics)
        self.server = None

    def build_server(self, port=8765, host='127.0.0.1'):
        camera = CameraService(self.session, MujocoCameraSource(self.renderer))
        self.server = Server(self.session, port, host, camera=camera)
        self.register_routes(self.server.app)
        return self.server

    def register_routes(self, app):
        app.add_api_route('/api/move_pose', action('move_pose', MovePose), methods=['POST'], tags=['Motion'],
                          summary='Sandbox extension: move all six joints to a pose at the demo speed')
        sandbox = self

        def blocks(request: Request):
            return sandbox.world.blocks()

        def sim_state(request: Request):
            physics = sandbox.physics
            return dict(time=physics.time, joint_angles=physics.joint_angles(), grasp_point=physics.grasp_point().tolist(),
                        pad_gap=physics.pad_gap(), held=sandbox.held_block(), cameras=sandbox.renderer.names(),
                        oak_fit=sandbox.cameras.get('oak_fit'), robot_arm_commit=commit())

        def reset(request: Request, args: Body):
            reset_blocks(sandbox.physics)
            return {'ok': True, 'result': sandbox.world.blocks()}

        def snapshot(request: Request, camera: str = 'oak'):
            try:
                return Response(sandbox.renderer.png(camera), media_type='image/png')
            except ValueError as error:
                return JSONResponse({'error': str(error)}, 400)

        web = ROOT / 'web'
        for route, name, media in (('/arm3d', 'arm3d.html', 'text/html; charset=utf-8'),
                                   ('/arm3d.js', 'arm3d.js', 'text/javascript; charset=utf-8'),
                                   ('/arm3d-core.js', 'arm3d-core.js', 'text/javascript; charset=utf-8'),
                                   ('/arm3d-config.js', 'arm3d-config.js', 'text/javascript; charset=utf-8'),
                                   ('/vendor/three.module.js', 'vendor/three.module.js', 'text/javascript; charset=utf-8'),
                                   ('/vendor/OrbitControls.js', 'vendor/OrbitControls.js', 'text/javascript; charset=utf-8'),
                                   ('/vendor/RoomEnvironment.js', 'vendor/RoomEnvironment.js', 'text/javascript; charset=utf-8')):
            app.add_api_route(route, _static(web / name, media), methods=['GET'], include_in_schema=False)
        def move_to(request: Request, args: MoveTo):
            try:
                if args.dry:
                    return {'ok': True, 'result': dict(steps=sandbox.mover.plan(args.x, args.y, args.z, args.yaw, args.lower, args.path))}
                sandbox.mover.start(args.x, args.y, args.z, args.yaw, args.lower, args.path)
            except ArmError as error:
                return JSONResponse({'error': str(error)}, 400)
            return {'ok': True, 'result': sandbox.mover.status()}

        def move_to_status(request: Request):
            return sandbox.mover.status()

        def move_to_stop(request: Request, args: Body):
            sandbox.mover.stop()
            return {'ok': True, 'result': sandbox.mover.status()}

        app.add_api_route('/move_to', move_to, methods=['POST'], tags=['Sandbox'], summary='Go above x, y (table frame, metres) and lower the open gripper to z')
        app.add_api_route('/move_to/status', move_to_status, methods=['GET'], tags=['Sandbox'], summary='Progress of the running move')
        app.add_api_route('/move_to/stop', move_to_stop, methods=['POST'], tags=['Sandbox'], summary='Stop the arm and the move')
        app.add_api_route('/sim/blocks', blocks, methods=['GET'], tags=['Sandbox'], summary='Block positions in the table frame')
        app.add_api_route('/sim/state', sim_state, methods=['GET'], tags=['Sandbox'], summary='Physics state: joint angles, grasp point, held block')
        app.add_api_route('/sim/reset', reset, methods=['POST'], tags=['Sandbox'], summary='Blocks back to their start slots')
        app.add_api_route('/sim/snapshot.png', snapshot, methods=['GET'], tags=['Sandbox'], summary='Render a named camera as PNG')

    def held_block(self):
        touching = self.physics.contacts('pad_left') & self.physics.contacts('pad_right')
        held = [name for name in self.physics.blocks if name in touching]
        return held[0] if held else None

    def start(self, port=8765, host='127.0.0.1'):
        server = self.build_server(port, host)
        self.stepper.start()
        server.app.state.camera.start()
        server.start()
        return server

    def stop(self):
        if self.server is not None:
            self.server.app.state.camera.close()
            self.server.stop()
        self.stepper.stop()
        self.session.close()
        self.renderer.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=8765)
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--no-camera-fit', action='store_true', help='Use the fallback oak camera instead of the fitted one')
    parser.add_argument('--viewer', action='store_true', help='Open the MuJoCo viewer (macOS: run with mjpython)')
    args = parser.parse_args()
    sandbox = Sandbox(fit_camera=not args.no_camera_fit)
    sandbox.build_server(args.port, args.host)
    sandbox.stepper.start()
    sandbox.server.app.state.camera.start()
    fit = sandbox.cameras.get('oak_fit')
    print(f'SANDBOX (MuJoCo) against robot-arm {commit()}: http://{args.host}:{args.port}', flush=True)
    print('Web page: /  3D view: /arm3d  browser sim: /simulation  API docs: /docs  sandbox routes: /sim/blocks /sim/state /sim/snapshot.png', flush=True)
    if fit:
        print(f'oak camera fitted from robot-arm calibration: {fit["error_px"]:.1f} px RMS over the markers', flush=True)
    finished = threading.Event()

    def watchdog():
        while not finished.wait(.25):
            sandbox.session.watchdog()
    threading.Thread(target=watchdog, daemon=True).start()
    try:
        if args.viewer:
            import mujoco.viewer
            sandbox.server.start()
            with mujoco.viewer.launch_passive(sandbox.physics.model, sandbox.physics.data) as viewer:
                while viewer.is_running():
                    with sandbox.physics.lock:
                        viewer.sync()
                    time.sleep(1 / 30)
            # Closing the viewer window must not take the API down; keep serving until Ctrl-C.
            print('Viewer closed; the API keeps running. Ctrl-C to stop.', flush=True)
            while True:
                time.sleep(1)
        else:
            signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
            sandbox.server.run()
    except KeyboardInterrupt:
        pass
    finally:
        finished.set()
        sandbox.stop()


if __name__ == '__main__':
    main()

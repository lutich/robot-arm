"""Camera HTTP contracts and responsiveness without a camera or servo hardware."""
from dataclasses import dataclass
import base64
import http.client
import json
from pathlib import Path
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from roboter_arm.control.domain.camera import CameraConflict, CameraUnavailable, DepthFrame
from roboter_arm.control.presentation import http_api as app
from roboter_arm.control.presentation.camera_api import CAMERA_FAILURE


@dataclass(frozen=True)
class Frame:
    sequence: int
    jpeg: bytes = b'\xff\xd8camera-frame\xff\xd9'
    depth: DepthFrame | None = None

    def metadata(self):
        result = dict(run_id='run-a', sequence=self.sequence, settings_revision=2,
                    captured_at='2026-10-06T12:00:00+00:00', received_at='2026-10-06T12:00:00+00:00',
                    age_ms=10, width=1280, height=720, time_us=8000, iso=200,
                    lens_position=130, temperature_k=4500, settings_verified=False,
                    measurement_ready=False)
        if self.depth is not None:
            result['depth'] = self.depth.metadata(self.depth.acquired_monotonic + .01)
        return result


def paired_frame(sequence):
    depth = DepthFrame(png=b'\x89PNG\r\n\x1a\n' + f'metric-{sequence}'.encode(),
                       preview_png=b'\x89PNG\r\n\x1a\n' + f'preview-{sequence}'.encode(),
                       captured_at='2026-10-06T12:00:00+00:00', acquired_monotonic=10,
                       observed=dict(unit='mm', invalid_value=0, aligned_to='rgb', width=1280,
                                     height=720, source_sequence=sequence, sync_delta_ms=4,
                                     valid_fraction=.75))
    return Frame(sequence, b'\xff\xd8' + f'rgb-{sequence}'.encode() + b'\xff\xd9', depth)


class Camera:
    def __init__(self):
        self.current = Frame(1)
        self.error = None
        self.waiting = threading.Event()
        self.requests = []

    def status(self):
        return dict(enabled=True, available=True, run_id='run-a', settings_revision=2,
                    application_status='sent', pending=False, locked=False,
                    latest_frame=self.current.metadata(), measurement_ready=False)

    def settings(self, **values):
        self.requests.append(values)
        return dict(run_id='run-a', settings_revision=3, application_status='queued',
                    requested={key:value for key, value in values.items() if not key.startswith('expected_')})

    def frame(self, after_run_id=None, after_sequence=None):
        if self.error:
            raise self.error
        if after_run_id is not None and after_run_id != 'run-a':
            raise CameraConflict('Camera run changed')
        if after_sequence is not None and self.current.sequence <= after_sequence:
            self.waiting.set()
            return None
        return self.current


class CameraAPITests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory(prefix='robot-camera-api-')
        root = Path(self.directory.name)
        self.driver = app.PreviewDriver()
        self.session = app.session_for(self.driver, list(range(6)), directory=root / 'poses',
                                       demo_directory=root / 'demos')
        self.camera = Camera()
        self.server = app.Server(self.session, 0, camera=self.camera)
        self.server.start()
        self.port = self.server.server_address[1]

    def tearDown(self):
        self.server.stop()
        self.session.close()
        self.directory.cleanup()

    def request(self, method, path, values=None, *, authorized=True):
        connection = http.client.HTTPConnection('127.0.0.1', self.port, timeout=3)
        headers = {}
        if method == 'POST':
            headers['Content-Type'] = 'application/json'
            if authorized:
                headers['Origin'] = f'http://127.0.0.1:{self.port}'
        connection.request(method, path, None if values is None else json.dumps(values), headers)
        response = connection.getresponse()
        data = response.read()
        result = json.loads(data) if response.getheader('Content-Type') == 'application/json' else data
        status, received_headers = response.status, {name.lower():value for name, value in response.getheaders()}
        connection.close()
        return status, result, received_headers

    def settings(self, **changes):
        return dict(expected_run_id='run-a', expected_revision=2, **changes)

    def stream_part(self, response):
        self.assertEqual(response.readline(), b'--frame\r\n')
        headers = {}
        while (line := response.readline()) != b'\r\n':
            name, value = line.decode().strip().split(':', 1)
            headers[name] = value.strip()
        data = response.read(int(headers['Content-Length']))
        self.assertEqual(response.read(2), b'\r\n')
        return data, headers

    def test_camera_reads_and_queued_settings_never_prepare_servos(self):
        with patch.object(self.driver, 'prepare', wraps=self.driver.prepare) as prepare:
            self.assertEqual(self.request('GET', '/api/camera/status')[0], 200)
            status, jpeg, headers = self.request('GET', '/api/camera/snapshot.jpg')
            self.assertEqual(status, 200)
            self.assertEqual(jpeg, self.camera.current.jpeg)
            self.assertEqual(headers['content-type'], 'image/jpeg')
            self.assertEqual(json.loads(headers['x-camera-frame']), self.camera.current.metadata())
            self.assertEqual(headers['cache-control'], 'no-store')
            changes = self.settings(exposure={'mode':'manual', 'time_us':8000, 'iso':200})
            status, accepted, _ = self.request('POST', '/api/camera/settings', changes)
            self.assertEqual(status, 200)
            self.assertTrue(accepted['ok'])
            self.assertEqual(accepted['result']['application_status'], 'queued')
            self.assertEqual(self.camera.requests, [changes])
            prepare.assert_not_called()
        self.assertFalse(self.session.prepared)

    def test_settings_origin_strict_types_nulls_and_complete_groups(self):
        path = '/api/camera/settings'
        self.assertEqual(self.request('POST', path, self.settings(anti_banding='50hz'), authorized=False)[0], 403)
        invalid = [
            self.settings(exposure=None), self.settings(focus=None),
            self.settings(white_balance=None), self.settings(anti_banding=None),
            self.settings(extra=True), self.settings(exposure={'mode':'manual', 'time_us':8000}),
            self.settings(exposure={'mode':'auto', 'iso':200}),
            self.settings(focus={'mode':'manual'}), self.settings(focus={'mode':'once', 'lens_position':130}),
            self.settings(white_balance={'mode':'manual'}),
            self.settings(white_balance={'mode':'auto', 'temperature_k':4500}),
            self.settings(exposure={'mode':'manual', 'time_us':'8000', 'iso':200}),
            self.settings(exposure={'mode':'manual', 'time_us':8000, 'iso':True}),
            self.settings(focus={'mode':'manual', 'lens_position':130.0}),
            self.settings(anti_banding='invalid'),
        ]
        invalid.extend(self.settings(anti_banding='50hz') | {'expected_revision':value}
                       for value in (-1, True, '2', 2.0))
        invalid.extend(self.settings(anti_banding='50hz') | {'expected_run_id':value}
                       for value in ('', ' ', None, 123))
        for values in invalid:
            with self.subTest(values=values):
                status, result, _ = self.request('POST', path, values)
                self.assertEqual(status, 400, result)
                self.assertIn('error', result)
        self.assertEqual(self.camera.requests, [])

    def test_camera_errors_preserve_status_and_never_stop_session(self):
        for error, expected in ((ValueError('Unsupported control'), 400),
                                (CameraConflict('Camera is locked'), 409),
                                (CameraUnavailable('Camera disconnected'), 503),
                                (RuntimeError('SDK failed'), 500)):
            with self.subTest(error=error), patch.object(self.camera, 'settings', side_effect=error), \
                    patch.object(self.session, 'stop', wraps=self.session.stop) as stop:
                status, result, _ = self.request('POST', '/api/camera/settings', self.settings(anti_banding='50hz'))
                self.assertEqual(status, expected)
                self.assertEqual(result['error'], CAMERA_FAILURE if expected == 500 else str(error))
                stop.assert_not_called()
        with patch.object(self.camera, 'status', side_effect=RuntimeError('SDK failed')), \
                patch.object(self.session, 'stop', wraps=self.session.stop) as stop:
            self.assertEqual(self.request('GET', '/api/camera/status')[:2], (500, {'error':CAMERA_FAILURE}))
            stop.assert_not_called()

    def test_snapshot_cursor_validation_timeout_and_run_change(self):
        invalid = ('after_run_id=run-a', 'after_sequence=1', 'after_run_id=&after_sequence=1',
                   'wait_ms=-1', 'wait_ms=2001', 'wait_ms=1.0', 'wait_ms=true',
                   'after_run_id=run-a&after_sequence=-1', 'after_run_id=run-a&after_sequence=1.0')
        for query in invalid:
            with self.subTest(query=query):
                self.assertEqual(self.request('GET', '/api/camera/snapshot.jpg?' + query)[0], 400)
        self.assertEqual(self.request('GET', '/api/camera/snapshot.jpg?after_run_id=old&after_sequence=1')[0], 409)
        started = time.monotonic()
        self.assertEqual(self.request('GET', '/api/camera/snapshot.jpg?after_run_id=run-a&after_sequence=1&wait_ms=0')[0], 503)
        self.assertLess(time.monotonic() - started, .5)
        self.camera.error = CameraUnavailable('Camera frame is stale')
        self.assertEqual(self.request('GET', '/api/camera/snapshot.jpg')[0], 503)
        self.assertEqual(self.request('GET', '/api/camera/stream.mjpg')[0], 503)

    def test_waiting_capture_returns_new_frame_and_does_not_delay_stop(self):
        captured = []
        capture = threading.Thread(target=lambda:captured.append(self.request(
            'GET', '/api/camera/snapshot.jpg?after_run_id=run-a&after_sequence=1&wait_ms=2000')))
        capture.start()
        self.assertTrue(self.camera.waiting.wait(1))
        try:
            started = time.monotonic()
            self.assertEqual(self.request('POST', '/api/stop', {})[0], 200)
            self.assertLess(time.monotonic() - started, .5)
            self.camera.current = Frame(2, b'\xff\xd8new-frame\xff\xd9')
            capture.join(timeout=2)
            self.assertFalse(capture.is_alive())
            status, jpeg, headers = captured[0]
            self.assertEqual(status, 200)
            self.assertEqual(jpeg, self.camera.current.jpeg)
            self.assertEqual(json.loads(headers['x-camera-frame'])['sequence'], 2)
        finally:
            self.camera.current = Frame(3)
            capture.join(timeout=3)

    def test_mjpeg_matching_part_metadata_and_stop_while_viewing(self):
        connection = http.client.HTTPConnection('127.0.0.1', self.port, timeout=2)
        connection.request('GET', '/api/camera/stream.mjpg')
        response = connection.getresponse()
        try:
            self.assertEqual(response.status, 200)
            self.assertEqual(response.getheader('Content-Type'), 'multipart/x-mixed-replace; boundary=frame')
            self.assertEqual(response.readline(), b'--frame\r\n')
            headers = {}
            while (line := response.readline()) != b'\r\n':
                name, value = line.decode().strip().split(':', 1)
                headers[name] = value.strip()
            self.assertEqual(headers['Content-Type'], 'image/jpeg')
            self.assertEqual(json.loads(headers['X-Camera-Frame']), self.camera.current.metadata())
            self.assertEqual(response.read(int(headers['Content-Length'])), self.camera.current.jpeg)
            self.assertEqual(response.read(2), b'\r\n')
            self.assertEqual(self.request('POST', '/api/stop', {})[0], 200)
            self.camera.error = CameraUnavailable('Camera disconnected')
            self.assertEqual(response.read(), b'--frame--\r\n')
        finally:
            response.close()
            connection.close()

    def test_capture_uses_one_matched_pair_when_cache_changes(self):
        original, replacement = paired_frame(1), paired_frame(2)
        self.camera.current = original

        def replace_cache(*cursor):
            self.camera.current = replacement
            return original

        with patch.object(self.camera, 'frame', side_effect=replace_cache) as lookup:
            status, captured, headers = self.request('GET', '/api/camera/capture')
            lookup.assert_called_once_with(None, None)
        self.assertEqual(status, 200)
        self.assertEqual(set(captured), {'metadata', 'rgb_jpeg', 'depth_png', 'depth_preview_png'})
        self.assertEqual(captured['metadata'], original.metadata())
        for field, expected in (('rgb_jpeg', original.jpeg), ('depth_png', original.depth.png),
                                ('depth_preview_png', original.depth.preview_png)):
            with self.subTest(field=field):
                self.assertEqual(base64.b64decode(captured[field], validate=True), expected)
        self.assertEqual(headers['cache-control'], 'no-store')
        self.assertFalse(captured['metadata']['settings_verified'])
        self.assertFalse(captured['metadata']['measurement_ready'])

    def test_depth_png_and_preview_keep_their_bytes_and_matching_metadata(self):
        self.camera.current = paired_frame(1)
        with patch.object(self.driver, 'prepare') as prepare:
            for path, expected in (('snapshot.png', self.camera.current.depth.png),
                                    ('preview.png', self.camera.current.depth.preview_png)):
                with self.subTest(path=path):
                    status, png, headers = self.request('GET', '/api/camera/depth/' + path)
                    self.assertEqual(status, 200)
                    self.assertEqual(png, expected)
                    self.assertEqual(headers['content-type'], 'image/png')
                    self.assertEqual(json.loads(headers['x-camera-frame']), self.camera.current.metadata())
                    self.assertEqual(headers['cache-control'], 'no-store')
            prepare.assert_not_called()

    def test_missing_depth_refuses_depth_routes_without_stopping_rgb(self):
        with patch.object(self.session, 'stop', wraps=self.session.stop) as stop:
            for path in ('snapshot.png', 'preview.png', 'stream'):
                with self.subTest(path=path):
                    status, result, _ = self.request('GET', '/api/camera/depth/' + path)
                    self.assertEqual((status, result), (503, {'error':'Stereo depth is unavailable'}))
            status, captured, _ = self.request('GET', '/api/camera/capture')
            self.assertEqual(status, 200)
            self.assertEqual(base64.b64decode(captured['rgb_jpeg']), self.camera.current.jpeg)
            self.assertIsNone(captured['depth_png'])
            self.assertIsNone(captured['depth_preview_png'])
            self.assertNotIn('depth', captured['metadata'])
            self.assertEqual(self.request('GET', '/api/camera/snapshot.jpg')[:2],
                             (200, self.camera.current.jpeg))
            stop.assert_not_called()

    def test_paired_capture_routes_validate_cursors_and_preserve_errors(self):
        self.camera.current = paired_frame(1)
        paths = ('/api/camera/capture', '/api/camera/depth/snapshot.png', '/api/camera/depth/preview.png')
        invalid = ('after_run_id=run-a', 'after_sequence=1', 'after_run_id=&after_sequence=1',
                   'after_run_id=run-a&after_sequence=-1', 'wait_ms=2001', 'wait_ms=1.0')
        with patch.object(self.session, 'stop', wraps=self.session.stop) as stop:
            for path in paths:
                for query in invalid:
                    with self.subTest(path=path, query=query):
                        self.assertEqual(self.request('GET', path + '?' + query)[0], 400)
                self.assertEqual(self.request('GET', path + '?after_run_id=old&after_sequence=1')[0], 409)
                self.assertEqual(self.request('GET', path + '?after_run_id=run-a&after_sequence=1&wait_ms=0')[0], 503)
                for error, expected in ((CameraUnavailable('Camera frame is stale'), 503),
                                        (RuntimeError('SDK failed'), 500)):
                    with self.subTest(path=path, error=error), patch.object(self.camera, 'frame', side_effect=error):
                        status, result, _ = self.request('GET', path)
                        self.assertEqual(status, expected)
                        self.assertEqual(result['error'], CAMERA_FAILURE if expected == 500 else str(error))
            stop.assert_not_called()

    def test_waiting_pair_capture_returns_all_new_images_without_delaying_stop(self):
        self.camera.current = paired_frame(1)
        captured = []
        capture = threading.Thread(target=lambda:captured.append(self.request(
            'GET', '/api/camera/capture?after_run_id=run-a&after_sequence=1&wait_ms=2000')))
        capture.start()
        self.assertTrue(self.camera.waiting.wait(1))
        try:
            started = time.monotonic()
            self.assertEqual(self.request('POST', '/api/stop', {})[0], 200)
            self.assertLess(time.monotonic() - started, .5)
            current = self.camera.current = paired_frame(2)
            capture.join(timeout=2)
            self.assertFalse(capture.is_alive())
            status, result, _ = captured[0]
            self.assertEqual(status, 200)
            self.assertEqual(result['metadata'], current.metadata())
            for field, expected in (('rgb_jpeg', current.jpeg), ('depth_png', current.depth.png),
                                    ('depth_preview_png', current.depth.preview_png)):
                self.assertEqual(base64.b64decode(result[field]), expected)
        finally:
            self.camera.current = paired_frame(3)
            capture.join(timeout=3)

    def test_depth_and_rgb_streams_share_frames_without_delaying_stop_or_snapshots(self):
        current = self.camera.current = paired_frame(1)
        connections, responses = [], []
        try:
            for path, media_type, expected in (('depth/stream', 'image/png', current.depth.preview_png),
                                               ('stream.mjpg', 'image/jpeg', current.jpeg)):
                connection = http.client.HTTPConnection('127.0.0.1', self.port, timeout=2)
                connections.append(connection)
                connection.request('GET', '/api/camera/' + path)
                response = connection.getresponse()
                responses.append(response)
                self.assertEqual(response.status, 200)
                self.assertEqual(response.getheader('Content-Type'), 'multipart/x-mixed-replace; boundary=frame')
                data, headers = self.stream_part(response)
                self.assertEqual(data, expected)
                self.assertEqual(headers['Content-Type'], media_type)
                self.assertEqual(json.loads(headers['X-Camera-Frame']), current.metadata())
            started = time.monotonic()
            self.assertEqual(self.request('POST', '/api/stop', {})[0], 200)
            self.assertLess(time.monotonic() - started, .5)
            self.assertEqual(self.request('GET', '/api/camera/depth/snapshot.png')[:2], (200, current.depth.png))
            self.assertEqual(self.request('GET', '/api/camera/snapshot.jpg')[:2], (200, current.jpeg))
            current = self.camera.current = paired_frame(2)
            for response, expected in zip(responses, (current.depth.preview_png, current.jpeg)):
                data, headers = self.stream_part(response)
                self.assertEqual(data, expected)
                self.assertEqual(json.loads(headers['X-Camera-Frame']), current.metadata())
            self.camera.error = CameraUnavailable('Camera disconnected')
            for response in responses:
                self.assertEqual(response.read(), b'--frame--\r\n')
        finally:
            for response in responses:
                response.close()
            for connection in connections:
                connection.close()

    def test_default_camera_stays_disabled_without_optional_sdk(self):
        self.server.stop()
        with patch.dict(sys.modules, {'depthai':None}), patch.object(self.driver, 'prepare') as prepare:
            self.server = app.Server(self.session, 0)
            self.server.start()
            self.port = self.server.server_address[1]
            status, values, _ = self.request('GET', '/api/camera/status')
            self.assertEqual(status, 200)
            self.assertFalse(values['enabled'])
            self.assertFalse(values['available'])
            self.assertFalse(values['depth_available'])
            self.assertFalse(values['capabilities'].get('has_depth', False))
            for path in ('snapshot.jpg', 'stream.mjpg', 'capture', 'depth/snapshot.png',
                         'depth/preview.png', 'depth/stream'):
                with self.subTest(path=path):
                    self.assertEqual(self.request('GET', '/api/camera/' + path)[0], 503)
            prepare.assert_not_called()


if __name__ == '__main__':
    unittest.main()

"""Real spawned workers: paired transport, native aborts and hangs; no camera or I2C."""
from functools import partial
import http.client
import json
import os
from pathlib import Path
import resource
import signal
import socket
import struct
import sys
import tempfile
import time
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from roboter_arm.control.application.camera import CameraService
from roboter_arm.control.domain.camera import CameraSample, CameraUnavailable, DepthSample, capabilities
from roboter_arm.control.infrastructure.camera_process import CameraProcess, _receive
from roboter_arm.control.presentation.http_api import PreviewDriver, Server, session_for

JPEG = b'\xff\xd8paired-image\xff\xd9'


class ProcessSource:
    def __init__(self, *, cleanup=None, hang=None):
        self.cleanup, self.hang = cleanup, hang
        self.settings = {}

    def open(self):
        # Stand-ins for native imports must remain confined to the child.
        resource.setrlimit(resource.RLIMIT_CORE, (0, 0))
        sys.modules['depthai'] = object()
        if self.hang == 'open':
            time.sleep(60)
        return dict(capabilities(True, has_depth=True), pid=os.getpid())

    def apply(self, changes):
        self.settings.update(changes)
        if changes.get('crash'):
            resource.setrlimit(resource.RLIMIT_CORE, (0, 0))
            os.abort()
        if self.hang == 'apply' and changes.get('anti_banding') == '60hz':
            time.sleep(60)

    def read(self):
        if self.hang == 'read':
            time.sleep(60)
        return CameraSample(JPEG, dict(pid=os.getpid(), settings=self.settings), .02,
            DepthSample(b'metric-png', b'preview-png', dict(sync_delta_ms=2), .022))

    def close(self):
        if self.hang == 'close':
            signal.signal(signal.SIGTERM, signal.SIG_IGN)
            time.sleep(60)
        if self.cleanup:
            Path(self.cleanup).write_text(str(os.getpid()))


class CameraProcessTests(unittest.TestCase):
    def source(self, **options):
        source = CameraProcess(partial(ProcessSource, **options), close_timeout=.3)
        self.addCleanup(source.close)
        return source

    def test_paired_payload_and_settings_stay_in_a_separate_interpreter(self):
        source = self.source()
        was_loaded = 'depthai' in sys.modules
        caps = source.open()
        self.assertNotEqual(caps['pid'], os.getpid())
        self.assertEqual(source.process._start_method, 'spawn')
        source.apply({'anti_banding':'60hz'})
        sample = source.read()
        self.assertEqual(sample.observed['settings'], {'anti_banding':'60hz'})
        self.assertEqual(sample.observed['pid'], caps['pid'])
        self.assertEqual((sample.jpeg, sample.depth.png, sample.depth.preview_png),
                         (JPEG, b'metric-png', b'preview-png'))
        self.assertGreater(sample.age, .02)
        self.assertAlmostEqual(sample.depth.age-sample.age, .002)
        self.assertEqual('depthai' in sys.modules, was_loaded)

    def test_transport_time_cannot_make_an_old_pair_appear_fresh(self):
        source = self.source()
        source.open()
        receive = _receive
        def delayed(*args):
            value = receive(*args)
            time.sleep(.08)
            return value
        with patch('roboter_arm.control.infrastructure.camera_process._receive', side_effect=delayed):
            sample = source.read()
        self.assertGreaterEqual(sample.age, .1)
        self.assertGreaterEqual(sample.depth.age, .102)

    def test_close_finishes_child_cleanup_and_is_idempotent(self):
        with tempfile.TemporaryDirectory() as directory:
            cleanup = Path(directory)/'closed'
            source = self.source(cleanup=str(cleanup))
            pid = source.open()['pid']
            source.close()
            self.assertEqual(cleanup.read_text(), str(pid))
            self.assertIsNone(source.process)
            source.close()

    def test_native_abort_is_reported_and_a_new_worker_can_open(self):
        source = self.source()
        old_pid = source.open()['pid']
        with self.assertRaisesRegex(CameraUnavailable, 'connection lost.*apply'):
            source.apply({'crash':True})
        source.process.join(1)
        self.assertEqual(source.process.exitcode, -signal.SIGABRT)
        self.assertNotEqual(source.open()['pid'], old_pid)
        self.assertEqual(source.read().jpeg, JPEG)

    def test_hung_open_read_and_apply_have_bounded_deadlines(self):
        for method in ('open', 'read', 'apply'):
            with self.subTest(method=method):
                source = self.source(hang=method)
                if method != 'open':
                    source.open()
                source.timeout = .3 if method != 'open' else 1
                started = time.monotonic()
                with self.assertRaisesRegex(CameraUnavailable, f'deadline exceeded during {method}'):
                    getattr(source, method)(*([{'anti_banding':'60hz'}] if method == 'apply' else []))
                source.close()
                self.assertLess(time.monotonic()-started, 3)
                self.assertIsNone(source.process)

    def test_hung_cleanup_is_killed_when_it_ignores_terminate(self):
        source = self.source(hang='close')
        source.open()
        process = source.process
        with patch.object(process, 'kill', wraps=process.kill) as kill:
            started = time.monotonic()
            source.close()
        kill.assert_called_once_with()
        self.assertLess(time.monotonic()-started, 2)
        self.assertIsNone(source.process)

    def test_partial_reply_header_or_payload_cannot_block_the_parent(self):
        for payload in (b'\0', struct.pack('!I', 100)+b'partial'):
            with self.subTest(payload=payload):
                reader, writer = socket.socketpair()
                with reader, writer:
                    writer.sendall(payload)
                    started = time.monotonic()
                    with self.assertRaises(TimeoutError):
                        _receive(reader, started+.1)
                    self.assertLess(time.monotonic()-started, .5)


class CameraProcessAPITests(unittest.TestCase):
    def wait_for(self, condition, timeout=5):
        deadline = time.monotonic()+timeout
        while not condition():
            if time.monotonic() >= deadline:
                self.fail('Camera condition timed out')
            time.sleep(.01)

    def request(self, server, method, path, values=None):
        host = f'127.0.0.1:{server.server_address[1]}'
        connection = http.client.HTTPConnection(host, timeout=2)
        try:
            connection.request(method, path, json.dumps(values) if values is not None else None,
                               {'Content-Type':'application/json', 'Origin':f'http://{host}'})
            response = connection.getresponse()
            return response.status, json.loads(response.read())
        finally:
            connection.close()

    def test_abort_clears_pending_without_losing_http_stop_or_session_state(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            session = session_for(PreviewDriver(), list(range(6)), directory=root/'poses',
                                  demo_directory=root/'demos')
            source = CameraProcess(partial(ProcessSource, hang='apply'), close_timeout=.3)
            camera = CameraService(session, source)
            server = Server(session, 0, camera=camera)
            camera.start()
            server.start()
            try:
                self.wait_for(lambda:camera.status()['available'])
                status = camera.status()
                # Crash the native owner while a valid settings request is in flight.
                camera.settings(expected_run_id=status['run_id'], expected_revision=0, anti_banding='60hz')
                self.assertTrue(session.camera_pending)
                started = time.monotonic()
                self.assertEqual(self.request(server, 'GET', '/api/state')[0], 200)
                self.assertEqual(self.request(server, 'POST', '/api/stop', {})[0], 200)
                self.assertLess(time.monotonic()-started, .5)
                self.assertTrue(session.camera_pending)
                # Abort only the child; the parent continues serving the controller API.
                os.kill(source.process.pid, signal.SIGABRT)
                self.wait_for(lambda:camera.status()['application_status'] == 'failed')
                self.wait_for(lambda:not session.camera_pending)
                self.assertFalse(camera.status()['available'])
                self.assertIsNone(camera.latest)
                self.assertEqual(self.request(server, 'GET', '/api/state')[0], 200)
                started = time.monotonic()
                self.assertEqual(self.request(server, 'POST', '/api/stop', {})[0], 200)
                self.assertLess(time.monotonic()-started, .5)
                self.assertFalse(session.prepared)
                self.assertFalse(session.armed)
                self.assertEqual(session.state()['moves'], 0)
                self.wait_for(lambda:camera.status()['available'])
                self.assertNotEqual(camera.status()['run_id'], status['run_id'])
                self.assertEqual(camera.status()['settings_revision'], 0)
            finally:
                server.stop()
                camera.close()
                session.close()


if __name__ == '__main__':
    unittest.main()

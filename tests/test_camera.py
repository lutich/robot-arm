"""Camera worker/admission checks use fake images and never open a device."""
from pathlib import Path
import sys
import tempfile
import threading
import time
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from roboter_arm.control.application.camera import CameraService
from roboter_arm.control.domain.camera import (
    CameraConflict, CameraFrame, CameraSample, CameraUnavailable, DepthFrame, DepthSample,
    capabilities, validate_changes)
from roboter_arm.control.infrastructure.oak_camera import OakCamera
from roboter_arm.control.presentation.http_api import PreviewDriver, session_for

JPEG = b'\xff\xd8fake-frame\xff\xd9'


class FakeSource:
    kind = 'oak'

    def __init__(self, autofocus=True):
        self.autofocus = autofocus
        self.sent = []
        self.opens = 0
        self.paused = False
        self.fail = False
        self.read_entered = threading.Event()
        self.read_release = threading.Event()
        self.block_read = False
        self.apply_entered = threading.Event()
        self.apply_release = threading.Event()
        self.block_apply = False
        self.depth = None

    def open(self):
        self.opens += 1
        return capabilities(self.autofocus, has_depth=self.depth is not None)

    def apply(self, changes):
        self.sent.append(changes)
        if self.block_apply:
            self.apply_entered.set()
            self.apply_release.wait(2)

    def read(self):
        if self.fail:
            self.fail = False
            raise RuntimeError('Disconnected fake camera')
        if self.block_read:
            self.read_entered.set()
            self.read_release.wait(2)
            self.block_read = False
        if self.paused:
            return None
        return CameraSample(JPEG, dict(width=1280, height=720, time_us=8000, iso=200,
                          lens_position=130 if self.autofocus else None,
                          temperature_k=4500, sensor_fps=5), 0, self.depth)

    def close(self):
        pass


class CameraTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory(prefix='robot-camera-')
        root = Path(self.directory.name)
        self.session = session_for(PreviewDriver(), list(range(6)), directory=root/'poses',
                                   demo_directory=root/'demos')
        self.source = FakeSource()
        self.camera = CameraService(self.session, self.source)
        self.camera.start()
        self.wait_for(lambda:self.camera.status()['available'])

    def tearDown(self):
        self.source.apply_release.set()
        self.source.read_release.set()
        self.camera.close()
        self.session.close()
        self.directory.cleanup()

    def wait_for(self, condition, timeout=3):
        deadline = time.monotonic()+timeout
        while not condition():
            if time.monotonic() >= deadline:
                self.fail('Camera condition timed out')
            time.sleep(.01)

    def change(self, **changes):
        status = self.camera.status()
        return self.camera.settings(expected_run_id=status['run_id'],
                                    expected_revision=status['settings_revision'], **changes)

    def test_disabled_service_and_reads_never_prepare_servos(self):
        disabled = CameraService(self.session)
        disabled.start()
        self.assertFalse(disabled.status()['enabled'])
        with self.assertRaises(CameraUnavailable):
            disabled.frame()
        self.assertFalse(self.session.prepared)
        self.assertFalse(self.camera.status()['measurement_ready'])
        self.assertFalse(self.camera.frame().metadata()['settings_verified'])

    def test_close_waits_for_worker_cleanup_before_returning(self):
        cleanup_entered, cleanup_release, close_returned = (threading.Event() for _ in range(3))
        def cleanup():
            cleanup_entered.set()
            cleanup_release.wait(5)
        def close():
            self.camera.close()
            close_returned.set()
        self.source.close = cleanup
        caller = threading.Thread(target=close)
        caller.start()
        try:
            self.assertTrue(cleanup_entered.wait(1))
            self.assertFalse(close_returned.wait(2.1))
            self.assertFalse(self.camera.thread.daemon)
            self.assertTrue(self.camera.thread.is_alive())
        finally:
            cleanup_release.set()
            caller.join(timeout=3)
        self.assertTrue(close_returned.is_set())
        self.assertFalse(self.camera.thread.is_alive())

    def test_oak_cleanup_waits_for_pipeline_before_releasing_device(self):
        calls = []
        class Pipeline:
            def stop(self):
                calls.append('stop')
            def wait(self):
                calls.append('wait')
        class Device:
            def close(self):
                calls.append('close')
        source = OakCamera()
        source.pipeline, source.device = Pipeline(), Device()
        source.frames, source.controls = object(), object()
        source.close()
        self.assertEqual(calls, ['stop', 'wait', 'close'])
        self.assertIsNone(source.pipeline)
        self.assertIsNone(source.device)
        self.assertIsNone(source.frames)
        self.assertIsNone(source.controls)
        source.close()
        self.assertEqual(calls, ['stop', 'wait', 'close'])

    def test_pair_freshness_uses_older_depth_and_keeps_its_timestamp(self):
        depth = DepthFrame(b'depth', b'preview', 'depth-time', 9.99,
                           dict(unit='mm', invalid_value=0, sync_delta_ms=10))
        frame = CameraFrame(JPEG, 'pair-run', 1, 0, 'rgb-time', 'received', 10, {}, depth)
        camera = CameraService(self.session, clock=lambda:10.995)
        camera.connected, camera.latest = True, frame
        metadata = frame.metadata(10.995)
        self.assertEqual(metadata['captured_at'], 'rgb-time')
        self.assertEqual(metadata['depth']['captured_at'], 'depth-time')
        self.assertEqual(metadata['depth']['age_ms'], 1005)
        self.assertFalse(camera.status()['available'])
        self.assertFalse(camera.status()['depth_available'])
        with self.assertRaises(CameraUnavailable):
            camera.frame()

    def test_depth_before_settings_dispatch_does_not_unlock_pair(self):
        observed = dict(width=1280, height=720, unit='mm', invalid_value=0,
                        aligned_to='rgb', sync_delta_ms=10, source_sequence=123)
        self.source.depth = DepthSample(b'depth', b'preview', observed, 0)
        self.wait_for(lambda:self.camera.status()['depth_available'])
        self.source.depth = DepthSample(b'old-depth', b'old-preview', observed, .5)
        self.change(anti_banding='60hz')
        time.sleep(.05)
        self.assertTrue(self.camera.status()['pending'])
        self.assertIsNone(self.camera.frame())
        self.source.depth = DepthSample(b'new-depth', b'new-preview', observed, 0)
        self.wait_for(lambda:not self.camera.status()['pending'])
        frame = self.camera.frame()
        self.assertEqual(frame.settings_revision, 1)
        self.assertEqual(frame.depth.png, b'new-depth')
        self.assertEqual(frame.metadata()['depth']['source_sequence'], 123)

    def test_excessive_depth_skew_is_not_published_or_sent_to_servos(self):
        self.source.depth = DepthSample(b'depth', b'preview', dict(sync_delta_ms=21), 0)
        self.wait_for(lambda:self.camera.status()['application_status'] == 'failed')
        self.assertIn('acquisition times do not match', self.camera.status()['last_error'])
        self.assertFalse(self.camera.status()['depth_available'])
        self.assertFalse(self.session.prepared)

    def test_group_policy_rejects_partial_manual_auto_extras_and_clamping(self):
        invalid = ({}, {'focus':dict(mode='manual', lens_position=256)},
                   {'exposure':dict(mode='manual', time_us=8000)},
                   {'exposure':dict(mode='auto', iso=200)},
                   {'exposure':dict(mode='manual', time_us=8000, iso=True)},
                   {'white_balance':dict(mode='manual', temperature_k=999)},
                   {'profile':dict(width=640)}, {'anti_banding':'unknown'})
        for changes in invalid:
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                self.change(**changes)
        self.assertEqual(self.camera.status()['settings_revision'], 0)
        with self.assertRaises(ValueError):
            validate_changes({'focus':dict(mode='once')}, capabilities(False))

    def test_groups_replace_and_readbacks_are_separate_from_requested_values(self):
        result = self.change(exposure=dict(mode='manual', time_us=9000, iso=300))
        self.assertEqual(result['application_status'], 'queued')
        self.wait_for(lambda:not self.camera.status()['pending'])
        status = self.camera.status()
        self.assertEqual(status['requested']['exposure']['time_us'], 9000)
        self.assertEqual(status['latest_frame']['time_us'], 8000)
        self.assertFalse(status['latest_frame']['settings_verified'])
        self.change(exposure=dict(mode='auto'))
        self.wait_for(lambda:not self.camera.status()['pending'])
        self.assertEqual(self.camera.status()['requested']['exposure'], {'mode':'auto'})
        self.assertEqual(self.source.sent[-1], {'exposure':{'mode':'auto'}})

    def test_move_and_camera_admission_are_mutually_exclusive_and_stop_remains_available(self):
        self.session.busy = True
        with self.assertRaises(CameraConflict):
            self.change(anti_banding='60hz')
        self.session.busy = False
        self.source.block_apply = True
        self.change(anti_banding='60hz')
        self.assertTrue(self.source.apply_entered.wait(1))
        with self.assertRaises(ValueError):
            self.session.power_on(park_confirmed=True)
        self.assertFalse(self.session.prepared)
        before = time.monotonic()
        self.session.stop()
        self.assertLess(time.monotonic()-before, .2)
        with self.assertRaises(CameraConflict):
            self.change(anti_banding='50hz')
        self.source.apply_release.set()
        self.wait_for(lambda:not self.session.camera_pending)

    def test_request_during_read_cannot_relabel_an_old_frame_or_unlock_motion(self):
        self.source.block_read = True
        self.assertTrue(self.source.read_entered.wait(1))
        self.source.block_apply = True
        self.change(focus=dict(mode='manual', lens_position=130))
        self.source.read_release.set()
        self.assertTrue(self.source.apply_entered.wait(1))
        self.assertIsNone(self.camera.frame())
        self.assertTrue(self.session.camera_pending)
        self.source.apply_release.set()
        self.wait_for(lambda:self.camera.status()['available'])
        self.assertEqual(self.camera.frame().settings_revision, 1)

    def test_stale_frame_refused_and_reconnect_invalidates_run_and_revision(self):
        old_run = self.camera.status()['run_id']
        self.source.paused = True
        self.wait_for(lambda:not self.camera.status()['available'], timeout=2)
        with self.assertRaises(CameraUnavailable):
            self.camera.frame()
        with self.assertRaises(CameraUnavailable):
            self.change(anti_banding='60hz')
        self.source.fail = True
        self.wait_for(lambda:self.camera.status()['application_status'] == 'failed')
        self.source.paused = False
        self.wait_for(lambda:self.camera.status()['available'])
        self.assertNotEqual(self.camera.status()['run_id'], old_run)
        with self.assertRaises(CameraConflict):
            self.camera.settings(expected_run_id=old_run, expected_revision=0, anti_banding='60hz')
        with self.assertRaises(CameraConflict):
            self.camera.frame(after_run_id=old_run, after_sequence=1)


if __name__ == '__main__':
    unittest.main()

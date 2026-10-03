"""Synthetic startup/stop/pose gates; physical calibration is never assumed."""
import json
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from roboter_arm.calibration.infrastructure import evidence_store
from roboter_arm.control.application.control import Control
from roboter_arm.control.domain.motion import JointLimit
from roboter_arm.control.infrastructure.evidence_limits import load_limits
from roboter_arm.shared.paths import ROOT
from test_motion import Driver, FakeTime


class ControlTests(unittest.TestCase):
    def setUp(self):
        self.time = FakeTime()
        self.driver = Driver(self.time)
        self.limits = {4:JointLimit(450, 510, 15, 'synthetic only', 'synthetic')}
        self.control = Control(self.driver, self.limits, {
            'PARK':dict(counts={4:480}, observed=True, provenance='synthetic supported fixture'),
            'HOME':dict(counts={4:490}, observed=True, provenance='synthetic clear fixture')})

    def timing(self, **overrides):
        return dict(timeout=5, clock=self.time.clock, sleep=self.time.sleep) | overrides

    def enable(self):
        self.control.enable({4:480}, supported=True, established=True,
                            stop_disable_agreed=True, **self.timing())

    def test_construction_and_unknown_start_never_write(self):
        for action in (lambda: self.control.move({4:490}, **self.timing()),
                       lambda: self.control.park(**self.timing()),
                       lambda: self.control.home(**self.timing())):
            with self.assertRaises(RuntimeError):
                action()
        self.assertEqual(self.driver.writes, [])

    def test_enable_requires_every_physical_condition_and_valid_counts(self):
        for flags in ({'supported':False}, {'established':False}, {'stop_disable_agreed':False}):
            with self.assertRaises(ValueError):
                self.control.enable({4:480}, **(dict(supported=True, established=True, stop_disable_agreed=True) | flags), **self.timing())
        with self.assertRaises(ValueError):
            self.control.enable({4:600}, supported=True, established=True, stop_disable_agreed=True, **self.timing())
        self.assertEqual(self.driver.writes, [])

    def test_normal_park_holds_then_supported_disable_invalidates_start(self):
        self.enable()
        self.control.home(**self.timing())
        self.control.park(**self.timing())
        self.assertEqual(self.driver.writes[-1][1:], (4, 480))
        self.assertEqual(self.driver.disabled, [])
        with self.assertRaises(ValueError):
            self.control.disable(supported=False)
        self.assertTrue(self.control.enabled)
        self.control.disable(supported=True)
        self.assertEqual(set(self.driver.disabled), {4})
        with self.assertRaises(RuntimeError):
            self.control.move({4:490}, **self.timing())
        self.assertEqual(self.control.runner.last_command, {4:480})

    def test_cancel_never_parks_and_invalidates_start(self):
        self.enable()
        with self.assertRaises(InterruptedError):
            self.control.move({4:500}, **self.timing(cancelled=lambda: self.time.now >= .05))
        self.assertEqual(set(self.driver.disabled), {4})
        self.assertFalse(self.control.enabled)
        self.assertTrue(all(count != 500 for _, _, count in self.driver.writes))
        with self.assertRaises(RuntimeError):
            self.control.park(**self.timing())

    def test_invalid_target_preserves_active_hold(self):
        self.enable()
        previous = list(self.driver.writes)
        with self.assertRaises(ValueError):
            self.control.move({4:600}, **self.timing())
        self.assertTrue(self.control.enabled)
        self.assertEqual(self.driver.writes, previous)
        self.assertEqual(self.driver.disabled, [])
        with self.assertRaises(ValueError):
            self.control.move({4:490}, **self.timing(timeout=.1))
        self.assertTrue(self.control.enabled)
        self.assertEqual(self.driver.writes, previous)

    def test_precancel_stops_outputs_held_from_enable(self):
        self.enable()
        with self.assertRaises(InterruptedError):
            self.control.move({4:490}, **self.timing(cancelled=lambda: True))
        self.assertEqual(self.driver.disabled, [4])
        self.assertFalse(self.control.enabled)

    def test_direct_disable_cancels_active_move_before_more_nonzero_commands(self):
        self.enable()
        def stop_during_sleep(seconds):
            self.time.sleep(seconds)
            self.control.disable(supported=True)
        with self.assertRaises(InterruptedError):
            self.control.move({4:490}, **self.timing(sleep=stop_during_sleep))
        self.assertFalse(self.control.enabled)
        self.assertTrue(all(count == 480 for _, _, count in self.driver.writes))

    def test_second_lifecycle_owner_rejected(self):
        with self.assertRaises(RuntimeError):
            Control(self.driver, self.limits, {})

    def test_commissioning_reference_limits_cannot_enter_general_control(self):
        self.driver.scope = 'commissioning'
        limits = {4:JointLimit(90,660,20,'unvalidated historical reference','commissioning')}
        with self.assertRaises(ValueError):
            Control(self.driver, limits, {})
        self.assertEqual(self.driver.writes, [])

    def test_conflicting_request_does_not_disable_active_hold(self):
        self.enable()
        attempted = False
        def write(channel, count):
            nonlocal attempted
            if not attempted:
                attempted = True
                with self.assertRaises(RuntimeError):
                    self.control.move({4:500}, **self.timing())
            self.driver.writes.append((self.time.now, channel, count))
        self.driver.write = write
        self.control.home(**self.timing())
        self.assertEqual(self.driver.disabled, [])
        self.assertTrue(self.control.enabled)

    def test_unobserved_or_out_of_bounds_poses_rejected_without_writes(self):
        for pose in (dict(counts={4:480}, observed=False, provenance='fixture'),
                     dict(counts={4:480}, observed=True, provenance=''),
                     dict(counts={4:600}, observed=True, provenance='fixture')):
            with self.assertRaises(ValueError):
                Control(self.driver, self.limits, {'PARK':pose})
        self.assertEqual(self.driver.writes, [])

    def test_disable_failure_invalidates_start_and_attempts_all_channels(self):
        self.driver = Driver(self.time)
        control = Control(self.driver, self.limits | {3:self.limits[4]}, {})
        def fail(channel):
            self.driver.disabled.append(channel)
            if channel == 4:
                raise OSError('fault')
        self.driver.disable = fail
        with self.assertRaises(RuntimeError):
            control.disable(supported=True)
        self.assertEqual(self.driver.disabled, [4, 3])
        self.assertFalse(control.enabled)


class EvidenceTests(unittest.TestCase):
    def test_only_explicit_validated_records_enable_general_limits(self):
        root = ROOT
        for name in ('inherited.json', 'gripper-observation.json'):
            with self.assertRaises(ValueError):
                load_limits([root / 'config/reference-arm' / name], {5:10})
        directory = Path(tempfile.mkdtemp(prefix='robot-limits-test-'))
        path = evidence_store.record(directory, channel=4, joint='synthetic wrist', low=450, high=510,
            kind='validated_limits', provenance='synthetic test, not physical evidence',
            direction='synthetic increasing', zero_count=480, zero_reference='synthetic reference')
        limits = load_limits([path], {4:15})
        self.assertEqual(limits[4].scope, 'validated')
        for paths, speeds in (([path, path], {4:15}), ([path], {}), ([path], {4:0})):
            with self.assertRaises(ValueError):
                load_limits(paths, speeds)
        data = json.loads(path.read_text())
        data['zero_reference'] = None
        corrupt = directory / 'invalid.json'
        corrupt.write_text(json.dumps(data))
        with self.assertRaises(ValueError):
            load_limits([corrupt], {4:15})


if __name__ == '__main__':
    unittest.main()

"""Deterministic synthetic trajectory, ownership, cancellation and fault checks."""
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from roboter_arm.control.application.motion_runner import MotionRunner
from roboter_arm.control.domain.motion import JointLimit, Trajectory


class FakeTime:
    def __init__(self):
        self.now = 0.0
    def clock(self):
        return self.now
    def sleep(self, seconds):
        self.now += seconds


class Driver:
    scope = 'synthetic'
    def __init__(self, time):
        self.time = time
        self.writes = []
        self.disabled = []
    def write(self, channel, count):
        self.writes.append((self.time.now, channel, count))
    def disable(self, channel):
        self.disabled.append(channel)


class MotionTests(unittest.TestCase):
    def setUp(self):
        self.time = FakeTime()
        self.driver = Driver(self.time)
        self.runner = MotionRunner(self.driver)
        self.limits = {i: JointLimit(100, 600, 30, 'synthetic fixture only', 'synthetic') for i in (0, 1)}
        self.start, self.target = {0:200, 1:400}, {0:260, 1:370}

    def run_motion(self, **kwargs):
        return self.runner.execute(self.start, self.target, self.limits, timeout=4,
            clock=self.time.clock, sleep=self.time.sleep, **kwargs)

    def test_duration_bounds_continuous_speed_and_exact_coordinated_endpoint(self):
        trajectory = Trajectory(self.start, self.target, self.limits)
        self.assertEqual(trajectory.duration, 3)
        self.assertEqual(trajectory.sample(0), self.start)
        self.assertEqual(trajectory.sample(3), self.target)
        self.assertEqual(trajectory.sample(9), self.target)
        # Analytical cubic derivative: zero at ends, maximum at the midpoint.
        for fraction in (0, .01, .25, .5, .75, .99, 1):
            counts = trajectory.sample(3 * fraction)
            for channel, count in counts.items():
                self.assertTrue(min(self.start[channel], self.target[channel]) <= count <= max(self.start[channel], self.target[channel]))
                derivative = abs(self.target[channel]-self.start[channel]) / 3 * 6 * fraction * (1-fraction)
                self.assertLessEqual(derivative, self.limits[channel].speed)
                if fraction in (0, 1):
                    self.assertEqual(derivative, 0)
        self.assertEqual(self.run_motion(), self.target)
        self.assertEqual(self.driver.writes[-2:], [(3.0, 0, 260), (3.0, 1, 370)])
        self.assertEqual(self.driver.disabled, [])

    def test_uses_elapsed_time_after_delayed_tick(self):
        def delayed_sleep(seconds):
            self.time.now += seconds + .3
        self.runner.execute(self.start, self.target, self.limits, timeout=4,
            clock=self.time.clock, sleep=delayed_sleep)
        curve = Trajectory(self.start, self.target, self.limits)
        for elapsed, channel, count in self.driver.writes:
            self.assertEqual(count, curve.sample(elapsed)[channel])
        self.assertEqual(self.runner.last_command, self.target)

    def test_invalid_requests_do_not_write_or_disable(self):
        for target in ({0:601, 1:370}, {0:260}, {0:260.5, 1:370}, {0:True, 1:370}):
            with self.assertRaises(ValueError):
                self.runner.execute(self.start, target, self.limits, timeout=4)
        for speed in (0, float('inf'), float('nan'), True, 1e-320):
            limits = self.limits | {0:JointLimit(100, 600, speed, 'fixture', 'synthetic')}
            with self.assertRaises(ValueError):
                self.runner.execute(self.start, self.target, limits, timeout=4)
        for changes in ({'timeout':2}, {'timeout':float('nan')}, {'interval':0}):
            with self.assertRaises(ValueError):
                self.runner.execute(self.start, self.target, self.limits, **({'timeout':4} | changes))
        self.driver.scope = 'validated'
        with self.assertRaises(ValueError):
            self.run_motion()
        self.assertEqual((self.driver.writes, self.driver.disabled), ([], []))

    def test_cancel_disables_without_recovery_and_releases_owner(self):
        with self.assertRaises(InterruptedError):
            self.run_motion(cancelled=lambda: self.time.now >= .05)
        self.assertEqual(self.driver.disabled, [0, 1])
        self.assertNotEqual(self.runner.last_command, self.target)
        self.assertEqual(self.run_motion(), self.target)

    def test_precancel_has_no_hardware_side_effect(self):
        with self.assertRaises(InterruptedError):
            self.run_motion(cancelled=lambda: True)
        self.assertEqual((self.driver.writes, self.driver.disabled), ([], []))

    def test_deadline_after_delayed_sleep_disables(self):
        def late_sleep(_):
            self.time.now += 5
        with self.assertRaises(TimeoutError):
            self.runner.execute(self.start, self.target, self.limits, timeout=4,
                clock=self.time.clock, sleep=late_sleep)
        self.assertEqual(self.driver.disabled, [0, 1])
        self.assertEqual(len(self.driver.writes), 2)

    def test_write_failure_stops_all_selected_even_if_one_disable_fails(self):
        def fail_write(channel, count):
            if channel == 1:
                raise OSError('write fault')
            self.driver.writes.append((self.time.now, channel, count))
        def fail_disable(channel):
            self.driver.disabled.append(channel)
            if channel == 0:
                raise OSError('disable fault')
        self.driver.write, self.driver.disable = fail_write, fail_disable
        with self.assertRaises(OSError) as caught:
            self.run_motion()
        self.assertEqual(self.driver.disabled, [0, 1])
        self.assertEqual(self.runner.last_command, {0:200})
        self.assertIn('disable failed', caught.exception.__notes__[0])

    def test_driver_overrun_is_detected_before_another_channel_write(self):
        def slow_write(channel, count):
            self.driver.writes.append((self.time.now, channel, count))
            self.time.now += 5
        self.driver.write = slow_write
        with self.assertRaises(TimeoutError):
            self.run_motion()
        self.assertEqual(len(self.driver.writes), 1)
        self.assertEqual(self.driver.disabled, [0, 1])
        self.assertEqual(self.runner.last_command, {0:200})

    def test_interrupt_or_backward_clock_after_start_disables(self):
        for failure in ('interrupt', 'backward'):
            self.setUp()
            def fail_sleep(_):
                if failure == 'interrupt':
                    raise KeyboardInterrupt()
                self.time.now -= 1
            with self.assertRaises(KeyboardInterrupt if failure == 'interrupt' else RuntimeError):
                self.runner.execute(self.start, self.target, self.limits, timeout=4,
                    clock=self.time.clock, sleep=fail_sleep)
            self.assertEqual(self.driver.disabled, [0, 1])

    def test_owner_rejects_nested_request_without_disturbing_active_motion(self):
        attempted = False
        second_runner = MotionRunner(self.driver)
        def write(channel, count):
            nonlocal attempted
            if not attempted:
                attempted = True
                with self.assertRaises(RuntimeError):
                    second_runner.execute(self.start, self.target, self.limits, timeout=4)
            self.driver.writes.append((self.time.now, channel, count))
        self.driver.write = write
        self.run_motion()
        self.assertEqual(self.driver.disabled, [])

    def test_unchanged_target_writes_exactly_once(self):
        self.target = dict(self.start)
        self.run_motion()
        self.assertEqual(len(self.driver.writes), 2)
        self.assertEqual(self.time.now, 0)


if __name__ == '__main__':
    unittest.main()

"""Hardware-free checks for the bounded gripper test and cleanup paths."""
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from roboter_arm.control.presentation import gripper_check as check


class Output:
    def __init__(self):
        self.writes = []

    @property
    def duty_cycle(self):
        return self.writes[-1]

    @duty_cycle.setter
    def duty_cycle(self, value):
        self.writes.append(value)


class GripperCheckTests(unittest.TestCase):
    def setUp(self):
        self.pwm = type('FakeController', (), {})()
        self.pwm.channels = [Output() for _ in range(16)]

    def test_bounded_gripper_only_and_return_then_off(self):
        waits = []
        check.run_motion(self.pwm, waits.append)
        writes = self.pwm.channels[5].writes
        counts = [value >> 4 for value in writes[:-1]]
        self.assertEqual(counts[0], 600)
        self.assertEqual(counts[-1], 600)
        self.assertEqual((min(counts), max(counts)), (595, 605))
        self.assertTrue(all(abs(a - b) == 1 for a, b in zip(counts, counts[1:])))
        self.assertEqual(writes[-1], 0)
        self.assertAlmostEqual(sum(waits), 2.0)
        self.assertTrue(all(not c.writes for i, c in enumerate(self.pwm.channels) if i != 5))

    def test_failure_and_interruption_disable_without_parking(self):
        for exception in (RuntimeError('simulated failure'), KeyboardInterrupt()):
            with self.subTest(exception=type(exception).__name__):
                self.setUp()
                def fail(_seconds):
                    raise exception
                with self.assertRaises(type(exception)):
                    check.run_motion(self.pwm, fail)
                self.assertEqual(self.pwm.channels[5].writes, [600 << 4, 0])


if __name__ == '__main__':
    unittest.main()

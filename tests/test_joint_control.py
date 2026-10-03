"""Hardware-free commissioning, driver initialization and evidence checks."""
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import types
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from roboter_arm.calibration.infrastructure import evidence_store
from roboter_arm.control.infrastructure import pca9685
from roboter_arm.control.presentation import joint_check as check
from roboter_arm.shared.paths import ROOT


class Output:
    def __init__(self):
        self.writes = []
        self.value = 0

    @property
    def duty_cycle(self):
        return self.value

    @duty_cycle.setter
    def duty_cycle(self, value):
        self.writes.append(value)
        self.value = value


class JointTests(unittest.TestCase):
    def setUp(self):
        self.pwm = types.SimpleNamespace(channels=[Output() for _ in range(16)])

    def test_each_selected_channel_bounds_timing_endpoint_and_off(self):
        for channel, start in enumerate((330, 540, 580, 250, 480, 600)):
            with self.subTest(channel=channel):
                self.setUp()
                waits = []
                check.run_motion(self.pwm, channel, start, sleep=waits.append)
                writes = self.pwm.channels[channel].writes
                counts = [value >> 4 for value in writes[:-1]]
                self.assertEqual((min(counts), max(counts)), (start - 5, start + 5))
                self.assertEqual((counts[0], counts[-1], writes[-1]), (start, start, 0))
                self.assertTrue(all(abs(a-b) == 1 for a, b in zip(counts, counts[1:])))
                self.assertAlmostEqual(sum(waits), 2)
                self.assertTrue(all(not output.writes for i, output in enumerate(self.pwm.channels) if i != channel))

    def test_invalid_requests_write_nothing(self):
        for args in ((6, 480, 5), (True, 480, 5), (4, 480.5, 5),
                     (4, 480, 0), (4, 480, 6), (4, 90, 5), (5, 660, 1)):
            with self.subTest(args=args), self.assertRaises(ValueError):
                check.run_motion(self.pwm, *args)
        self.assertTrue(all(not output.writes for output in self.pwm.channels))

    def test_failure_interrupt_and_failed_write_do_not_park(self):
        for error in (RuntimeError('fault'), KeyboardInterrupt()):
            self.setUp()
            def fail(_):
                raise error
            with self.assertRaises(type(error)):
                check.run_motion(self.pwm, 4, 480, sleep=fail)
            self.assertEqual(self.pwm.channels[4].writes, [480 << 4, 0])
        output = self.pwm.channels[4]
        class FailingOutput(Output):
            @Output.duty_cycle.setter
            def duty_cycle(self, value):
                if value:
                    raise OSError('write failure')
                Output.duty_cycle.fset(self, value)
        self.pwm.channels[4] = FailingOutput()
        with self.assertRaises(OSError):
            check.run_motion(self.pwm, 4, 480)
        self.assertEqual(self.pwm.channels[4].writes, [0])

    def test_output_off_readback_detects_active_channel(self):
        self.pwm.channels[2].value = 100
        with self.assertRaises(RuntimeError):
            check.verify_off(self.pwm)

    def test_bounded_position_hold_and_fault_cleanup_without_return_move(self):
        waits = []
        check.run_position(self.pwm, 4, 480, 5, sleep=waits.append)
        self.assertEqual(self.pwm.channels[4].writes, [480 << 4, 0])
        self.assertEqual(waits, [5])
        self.assertTrue(all(not c.writes for i, c in enumerate(self.pwm.channels) if i != 4))
        for seconds in (0, 6, float('nan'), float('inf'), True):
            with self.assertRaises(ValueError):
                check.run_position(self.pwm, 4, 480, seconds)
        self.setUp()
        def fail(_):
            raise KeyboardInterrupt()
        with self.assertRaises(KeyboardInterrupt):
            check.run_position(self.pwm, 4, 480, 5, sleep=fail)
        self.assertEqual(self.pwm.channels[4].writes, [480 << 4, 0])

    def test_full_off_precedes_wake_and_initialization_failure_closes_bus(self):
        events = []
        bus = types.SimpleNamespace(deinit=lambda: events.append('closed'))
        class Device:
            def __init__(self, *_):
                pass
            def __enter__(self):
                return self
            def __exit__(self, *_):
                pass
            def write(self, data):
                events.append(data)
        def wake(*args, **kwargs):
            events.append('wake')
            self.assertEqual(events[:16], [bytes((9 + 4*i, 16)) for i in range(16)])
            return self.pwm
        modules = {
            'board': types.SimpleNamespace(I2C=lambda: bus),
            'adafruit_bus_device.i2c_device': types.SimpleNamespace(I2CDevice=Device),
            'adafruit_pca9685': types.SimpleNamespace(PCA9685=wake),
        }
        with patch.dict(sys.modules, modules):
            with check.controller(4):
                check.run_motion(self.pwm, 4, 480, sleep=lambda _: None)
        self.assertEqual(events[-1], 'closed')
        self.assertEqual(self.pwm.frequency, 60)
        self.assertTrue(all(c.value == 0 for c in self.pwm.channels))
        modules['adafruit_pca9685'] = types.SimpleNamespace(PCA9685=lambda *a, **kw: (_ for _ in ()).throw(OSError()))
        events.clear()
        with patch.dict(sys.modules, modules), self.assertRaises(OSError):
            with pca9685.controller():
                self.fail('Initialization should fail')
        self.assertEqual(events[-1], 'closed')

    def test_preview_imports_no_hardware_and_missing_readiness_cannot_initialize(self):
        script = ROOT / 'scripts/joint_check.py'
        for extra, expected in (([], 0), (['--execute'], 2), (['--prepare'], 2),
                                (['--hold-seconds', '5'], 0),
                                (['--hold-seconds', '5', '--execute'], 2),
                                (['--hold-seconds', '6'], 2),
                                (['--hold-seconds', '5', '--excursion', '2'], 2)):
            result = subprocess.run([sys.executable, '-c',
                "import sys, runpy; sys.modules['board']=None; "
                "sys.modules['adafruit_pca9685']=None; "
                f"sys.path.insert(0, {str(script.parent)!r}); "
                f"sys.argv=[{str(script)!r}, '--channel','4','--start','480']+{extra!r}; "
                f"runpy.run_path({str(script)!r}, run_name='__main__')"], capture_output=True)
            self.assertEqual(result.returncode, expected, result.stderr.decode())
        self.assertIn('Preview only', subprocess.check_output(
            [sys.executable, str(script), '--channel', '4', '--start', '480'], text=True))


class CalibrationTests(unittest.TestCase):
    def setUp(self):
        self.directory = Path(tempfile.mkdtemp(prefix='robot-calibration-test-'))
        self.args = dict(channel=5, joint='gripper', low=595, high=605,
                         kind='observation', provenance='synthetic test evidence')

    def test_append_preserves_old_observation_without_promoting_limits(self):
        first = evidence_store.record(self.directory, **self.args)
        old = first.read_bytes()
        second = evidence_store.record(self.directory, **self.args)
        self.assertNotEqual(first, second)
        self.assertEqual(first.read_bytes(), old)
        data = json.loads(old)
        self.assertEqual(data['kind'], 'observation')
        self.assertIsNone(data['zero_count'])
        self.assertFalse(data['position_feedback'])
        self.assertEqual(data['units'], 'PCA9685_counts')

    def test_reject_missing_reference_and_invalid_bounds_before_persistence(self):
        for change in ({'kind':'validated_limits'}, {'channel':True}, {'low':490.5},
                       {'high':661}, {'low':606}, {'provenance':''},
                       {'zero_count':600}, {'direction':''}):
            with self.subTest(change=change), self.assertRaises(ValueError):
                evidence_store.record(self.directory, **(self.args | change))
        self.assertEqual(list(self.directory.iterdir()), [])

    def test_validated_record_retains_direction_zero_and_provenance(self):
        file = evidence_store.record(self.directory, **(self.args | dict(kind='validated_limits',
            direction='synthetic positive direction', zero_count=600, zero_reference='synthetic fixture')))
        self.assertEqual(json.loads(file.read_text())['zero_reference'], 'synthetic fixture')


if __name__ == '__main__':
    unittest.main()

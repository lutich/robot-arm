"""GPIO claims, edge validation, timing and cleanup using a fake lgpio module."""
from pathlib import Path
import sys
import threading
import time
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from roboter_arm.control.infrastructure.hcsr04 import HCSR04


class Clock:
    def __init__(self):
        self.now = 10.0

    def __call__(self):
        return self.now

    def ns(self):
        return int(self.now*1_000_000_000)

    def sleep(self, seconds):
        self.now += seconds


class Cancellation:
    def __init__(self, clock):
        self.clock = clock
        self.cancelled = False

    def is_set(self):
        return self.cancelled

    def wait(self, seconds):
        self.clock.sleep(seconds)
        return self.cancelled


class GPIO:
    BOTH_EDGES = 3

    def __init__(self, source, fail=None):
        self.source = source
        self.fail = fail
        self.log = []
        self.events = [(1, 200_000), (0, 400_000)]
        self.echo = 0
        self.callback_handle = Mock()
        self.on_trigger = None
        self.edge = None

    def gpiochip_open(self, chip):
        self.log.append(('open', chip))
        if self.fail == 'open':
            raise RuntimeError('open failed')
        return 7

    def gpio_claim_output(self, handle, gpio, level):
        self.log.append(('output', handle, gpio, level))
        if self.fail == 'output':
            raise RuntimeError('output failed')

    def gpio_claim_alert(self, handle, gpio, edges):
        self.log.append(('alert', handle, gpio, edges))
        if self.fail == 'alert':
            raise RuntimeError('alert failed')

    def callback(self, handle, gpio, edges, callback):
        self.log.append(('callback', handle, gpio, edges))
        if self.fail == 'callback':
            raise RuntimeError('callback failed')
        self.edge = callback
        return self.callback_handle

    def gpio_read(self, handle, gpio):
        self.log.append(('read', handle, gpio))
        return self.echo

    def gpio_write(self, handle, gpio, level):
        self.log.append(('write', handle, gpio, level, self.source.clock()))
        if level:
            if self.fail == 'write':
                raise RuntimeError('write failed')
            if self.on_trigger:
                self.on_trigger()
            for edge, offset in self.events:
                self.edge(0, 24, edge, self.source._start_ns+offset)

    def gpio_free(self, handle, gpio):
        self.log.append(('free', handle, gpio))
        if self.fail == 'free':
            raise RuntimeError('free failed')

    def gpiochip_close(self, handle):
        self.log.append(('close', handle))


class HCSR04Tests(unittest.TestCase):
    def source(self, *, fake_clock=False, fail=None):
        clock = Clock() if fake_clock else None
        source = HCSR04(clock=clock, clock_ns=clock.ns) if clock else HCSR04()
        gpio = GPIO(source, fail)
        with patch.dict(sys.modules, lgpio=gpio):
            source.start()
        self.addCleanup(source.close)
        return source, gpio, clock

    def acquire(self, source, clock=None, seconds=5):
        cancelled = Cancellation(clock) if clock else threading.Event()
        if clock:
            with patch('roboter_arm.control.infrastructure.hcsr04.time.sleep', side_effect=clock.sleep):
                return source.read(cancelled=cancelled, deadline=clock()+seconds)
        return source.read(cancelled=cancelled, deadline=time.monotonic()+seconds)

    def test_construction_does_not_import_lgpio_or_access_hardware(self):
        with patch.dict(sys.modules, lgpio=None):
            source = HCSR04()
        self.assertIsNone(source._gpio)
        self.assertIsNone(source._handle)

    def test_claims_only_configured_gpio_and_releases_in_reverse_order(self):
        source, gpio, clock = self.source(fake_clock=True)
        sample = self.acquire(source, clock)
        self.assertEqual(sample.pulse_us, 200)
        self.assertIsNone(sample.error)
        self.assertIn('+00:00', sample.captured_at)
        source.close()
        source.close()
        self.assertEqual(gpio.log[:4], [('open', 0), ('output', 7, 23, 0),
            ('alert', 7, 24, 3), ('callback', 7, 24, 3)])
        self.assertEqual([entry for entry in gpio.log if entry[0] == 'free'],
                         [('free', 7, 24), ('free', 7, 23)])
        self.assertEqual(gpio.log[-1], ('close', 7))
        gpio.callback_handle.cancel.assert_called_once_with()
        writes = [entry[3] for entry in gpio.log if entry[0] == 'write']
        self.assertEqual(writes, [1, 0, 0, 0])

    def test_partial_startup_failure_frees_only_successfully_claimed_pins(self):
        for fail, freed in (('open', []), ('output', []), ('alert', [23]), ('callback', [24, 23])):
            with self.subTest(fail=fail):
                source = HCSR04()
                gpio = GPIO(source, fail)
                with patch.dict(sys.modules, lgpio=gpio), self.assertRaises(RuntimeError):
                    source.start()
                self.assertEqual([entry[2] for entry in gpio.log if entry[0] == 'free'], freed)
                source.close()
                self.assertEqual(len([entry for entry in gpio.log if entry[0] == 'close']), int(fail != 'open'))
                with self.assertRaisesRegex(RuntimeError, 'restart required'):
                    source.start()

    def test_stale_and_future_notifications_are_ignored_for_current_trigger(self):
        source, gpio, clock = self.source(fake_clock=True)
        gpio.events = [(1, -400_000), (0, -200_000), (1, 101_000_000), (0, 102_000_000),
                       (1, 200_000), (0, 400_000)]
        sample = self.acquire(source, clock)
        self.assertEqual((sample.pulse_us, sample.error), (200, None))

    def test_wrong_chip_or_pin_notifications_are_ignored(self):
        source, gpio, clock = self.source(fake_clock=True)
        def wrong_edges():
            gpio.edge(1, 24, 0, source._start_ns+100_000)
            gpio.edge(0, 23, 0, source._start_ns+100_000)
        gpio.on_trigger = wrong_edges
        sample = self.acquire(source, clock)
        self.assertEqual((sample.pulse_us, sample.error), (200, None))

    def test_repeated_unordered_and_nonincreasing_edges_are_rejected(self):
        malformed = [[(0, 200_000)], [(1, 100_000), (1, 200_000), (0, 400_000)],
                     [(1, 200_000), (0, 100_000)], [(1, 200_000), (0, 200_000)], [(2, 100_000)]]
        for events in malformed:
            with self.subTest(events=events):
                source, gpio, clock = self.source(fake_clock=True)
                gpio.events = events
                sample = self.acquire(source, clock)
                self.assertIsNone(sample.pulse_us)
                self.assertEqual(sample.error, 'Malformed echo edges')

    def test_trigger_start_spacing_is_preserved_across_separate_reads(self):
        source, gpio, clock = self.source(fake_clock=True)
        for _ in range(3):
            self.acquire(source, clock)
        starts = [entry[4] for entry in gpio.log if entry[0] == 'write' and entry[3] == 1]
        self.assertEqual(len(starts), 3)
        self.assertAlmostEqual(starts[1]-starts[0], .2)
        self.assertAlmostEqual(starts[2]-starts[1], .2)
        low = [entry[4] for entry in gpio.log if entry[0] == 'write' and entry[3] == 0]
        self.assertAlmostEqual(low[0]-starts[0], .0001)

    def test_batch_deadline_during_cooldown_does_not_trigger(self):
        source, gpio, clock = self.source(fake_clock=True)
        self.acquire(source, clock)
        sample = self.acquire(source, clock, seconds=.05)
        self.assertEqual(sample.error, 'Batch deadline exceeded')
        self.assertEqual(len([entry for entry in gpio.log if entry[0] == 'write' and entry[3] == 1]), 1)

    def test_idle_high_echo_does_not_trigger(self):
        source, gpio, clock = self.source(fake_clock=True)
        gpio.echo = 1
        sample = self.acquire(source, clock)
        self.assertEqual(sample.error, 'ECHO already HIGH')
        self.assertFalse(any(entry[0] == 'write' for entry in gpio.log))

    def test_missing_and_incomplete_echo_time_out_and_trigger_returns_low(self):
        for events in ([], [(1, 200_000)]):
            with self.subTest(events=events):
                source, gpio, _ = self.source()
                gpio.events = events
                started = time.monotonic()
                sample = self.acquire(source)
                self.assertEqual(sample.error, 'No complete echo within 100 ms')
                self.assertGreaterEqual(time.monotonic()-started, .09)
                self.assertLess(time.monotonic()-started, .2)
                self.assertEqual([entry[3] for entry in gpio.log if entry[0] == 'write'], [1, 0, 0])

    def test_echo_wait_uses_remaining_batch_deadline(self):
        source, gpio, _ = self.source()
        gpio.events = []
        started = time.monotonic()
        sample = self.acquire(source, seconds=.02)
        self.assertEqual(sample.error, 'Batch deadline exceeded')
        self.assertLess(time.monotonic()-started, .08)

    def test_trigger_write_failure_still_attempts_low_and_later_cleanup(self):
        source, gpio, _ = self.source()
        gpio.fail = 'write'
        with self.assertRaisesRegex(RuntimeError, 'write failed'):
            self.acquire(source)
        self.assertEqual([entry[3] for entry in gpio.log if entry[0] == 'write'], [1, 0])
        self.assertFalse(source._active)
        source.close()
        self.assertEqual(gpio.log[-1], ('close', 7))

    def test_cancelled_echo_and_close_wake_before_free_and_callbacks_after_close_are_harmless(self):
        source, gpio, _ = self.source()
        gpio.events = []
        entered = threading.Event()
        gpio.on_trigger = entered.set
        cancelled = threading.Event()
        results = []
        worker = threading.Thread(target=lambda: results.append(source.read(
            cancelled=cancelled, deadline=time.monotonic()+5)))
        worker.start()
        self.assertTrue(entered.wait(1))
        started = time.monotonic()
        source.close()
        worker.join(1)
        self.assertFalse(worker.is_alive())
        self.assertLess(time.monotonic()-started, .1)
        self.assertEqual(results[0].error, 'Measurement cancelled')
        after = list(gpio.log)
        gpio.edge(0, 24, 1, time.monotonic_ns())
        gpio.edge(0, 24, 0, time.monotonic_ns()+200_000)
        self.assertEqual(gpio.log, after)
        self.assertEqual(gpio.log[-1], ('close', 7))

    def test_cancelled_cooldown_wakes_without_starting_another_trigger(self):
        source, gpio, _ = self.source()
        self.acquire(source)
        cancelled = threading.Event()
        results = []
        worker = threading.Thread(target=lambda: results.append(source.read(
            cancelled=cancelled, deadline=time.monotonic()+5)))
        worker.start()
        time.sleep(.01)
        cancelled.set()
        worker.join(.1)
        self.assertFalse(worker.is_alive())
        self.assertEqual(results[0].error, 'Measurement cancelled')
        self.assertEqual(len([entry for entry in gpio.log if entry[0] == 'write' and entry[3] == 1]), 1)

    def test_close_during_cooldown_wakes_without_external_cancellation(self):
        source, gpio, _ = self.source()
        self.acquire(source)
        results = []
        worker = threading.Thread(target=lambda: results.append(source.read(
            cancelled=threading.Event(), deadline=time.monotonic()+5)))
        worker.start()
        time.sleep(.01)
        started = time.monotonic()
        source.close()
        worker.join(.1)
        self.assertFalse(worker.is_alive())
        self.assertLess(time.monotonic()-started, .1)
        self.assertEqual(results[0].error, 'Measurement cancelled')
        self.assertEqual(len([entry for entry in gpio.log if entry[0] == 'write' and entry[3] == 1]), 1)

    def test_cleanup_continues_when_free_fails(self):
        source, gpio, _ = self.source()
        gpio.fail = 'free'
        with self.assertRaisesRegex(RuntimeError, 'cleanup failed'):
            source.close()
        self.assertEqual([entry[2] for entry in gpio.log if entry[0] == 'free'], [24, 23])
        self.assertEqual(gpio.log[-1], ('close', 7))
        source.close()


if __name__ == '__main__':
    unittest.main()

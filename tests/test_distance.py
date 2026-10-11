"""Distance aggregation, admission and shutdown without GPIO or servo hardware."""
from pathlib import Path
import sys
import threading
import time
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from roboter_arm.control.application.distance import DistanceService
from roboter_arm.control.domain.distance import (
    DistanceConflict, DistanceReading, DistanceUnavailable)


class FakeSource:
    def __init__(self, readings=()):
        self.readings = iter(readings)
        self.started = self.closed = 0
        self.calls = []
        self.failure = None

    def start(self):
        self.started += 1
        if self.failure:
            raise self.failure

    def read(self, **kwargs):
        self.calls.append(kwargs)
        return next(self.readings)

    def close(self):
        self.closed += 1


def reading(distance=None, error=None):
    return DistanceReading('2026-10-08T12:00:00+00:00',
                           distance*2000/343 if distance is not None else None, error)


class DistanceTests(unittest.TestCase):
    def service(self, readings, **kwargs):
        source = FakeSource(readings)
        service = DistanceService(source, **kwargs)
        service.start()
        self.addCleanup(service.close)
        return service, source

    def test_disabled_and_construction_have_no_source_calls(self):
        source = FakeSource()
        self.assertFalse(DistanceService(source).status()['ready'])
        self.assertEqual((source.started, source.closed, source.calls), (0, 0, []))
        service = DistanceService()
        service.start()
        self.assertFalse(service.status()['enabled'])
        with self.assertRaises(DistanceUnavailable):
            service.read()
        service.close()

    def test_defaults_are_copied_and_status_never_measures(self):
        service, source = self.service([])
        status = service.status()
        self.assertEqual(set(status), {'enabled', 'ready', 'busy', 'last_error', 'defaults'})
        self.assertEqual(status['defaults']['aggregation'], 'median')
        status['defaults']['default_samples'] = 17
        self.assertEqual(service.status()['defaults']['default_samples'], 5)
        self.assertEqual(source.calls, [])

    def test_default_batch_keeps_each_sample_and_uses_population_spread(self):
        service, source = self.service([reading(value) for value in (20, 30, 40, 50, 4000)])
        result = service.read()
        self.assertEqual((result['attempts'], result['valid'], result['distance_mm']), (5, 5, 40))
        self.assertEqual((result['min_mm'], result['max_mm']), (20, 4000))
        self.assertEqual(result['mean_mm'], 828)
        self.assertGreater(result['stdev_mm'], 1500)
        self.assertEqual(result['sound_speed_m_s'], 343)
        self.assertGreaterEqual(result['elapsed_ms'], 0)
        self.assertEqual(len(result['readings']), 5)
        self.assertEqual(result['readings'][0], dict(captured_at='2026-10-08T12:00:00+00:00',
            pulse_us=20*2000/343, distance_mm=20, error=None))
        self.assertEqual(len({call['deadline'] for call in source.calls}), 1)

    def test_partial_batch_preserves_invalid_pulses_and_failure_reason(self):
        service, _ = self.service([reading(100), reading(error='No echo'), reading(10), reading(5000), reading(200)])
        result = service.read()
        self.assertEqual((result['valid'], result['distance_mm'], result['stdev_mm']), (2, 150, 50))
        self.assertEqual([value['distance_mm'] for value in result['readings']], [100, None, None, None, 200])
        self.assertEqual(result['readings'][1]['error'], 'No echo')
        self.assertIsNotNone(result['readings'][2]['pulse_us'])
        self.assertIn('outside nominal', service.status()['last_error'])
        self.assertTrue(service.status()['ready'])

    def test_all_invalid_batch_is_unavailable_with_evidence_and_next_read_can_work(self):
        service, _ = self.service([reading(error='Timeout'), reading(36)])
        with self.assertRaises(DistanceUnavailable) as failure:
            service.read(1)
        self.assertEqual(failure.exception.result['valid'], 0)
        self.assertIsNone(failure.exception.result['distance_mm'])
        self.assertEqual(service.read(1)['distance_mm'], 36)
        self.assertIsNone(service.status()['last_error'])

    def test_nonfinite_zero_and_missing_pulse_are_invalid_and_json_safe(self):
        values = [DistanceReading('now', pulse) for pulse in (float('nan'), float('inf'), 0, -1, None)]
        service, _ = self.service(values)
        with self.assertRaises(DistanceUnavailable) as failure:
            service.read()
        result = failure.exception.result
        self.assertEqual(result['valid'], 0)
        self.assertEqual([value['pulse_us'] for value in result['readings']], [None, None, 0, -1, None])

    def test_strict_sample_validation_does_not_acquire_or_touch_source(self):
        service, source = self.service([])
        for value in (True, False, 0, 21, 1.0, '5', None):
            with self.subTest(value=value), self.assertRaises(ValueError):
                service.read(value)
        self.assertEqual(source.calls, [])
        self.assertTrue(service.status()['ready'])

    def test_startup_failure_is_recorded_cleaned_and_never_retried(self):
        source = FakeSource()
        source.failure = RuntimeError('GPIO busy')
        service = DistanceService(source)
        service.start()
        service.start()
        self.assertEqual((source.started, source.closed), (1, 1))
        self.assertFalse(service.status()['ready'])
        self.assertIn('GPIO busy', service.status()['last_error'])
        with self.assertRaises(DistanceUnavailable):
            service.read(1)
        service.close()
        service.close()
        self.assertEqual(source.closed, 1)

    def test_unexpected_failure_propagates_and_marks_sensor_unavailable(self):
        service, _ = self.service([])
        with self.assertRaises(StopIteration):
            service.read(1)
        self.assertIn('StopIteration', service.status()['last_error'])
        self.assertFalse(service.status()['ready'])
        self.assertFalse(service.status()['busy'])
        with self.assertRaises(DistanceUnavailable):
            service.read(1)

    def test_batch_deadline_counts_acquisition_and_stops_more_attempts(self):
        clock_value = [10.0]
        service, source = self.service([], clock=lambda: clock_value[0])
        def acquire(**kwargs):
            self.assertEqual(kwargs['deadline'], 15.0)
            clock_value[0] = min(kwargs['deadline'], clock_value[0]+3)
            return reading(36)
        source.read = acquire
        result = service.read(20)
        self.assertEqual(result['attempts'], 2)
        self.assertEqual(result['elapsed_ms'], 5000)

    def test_concurrent_request_rejects_immediately_and_close_waits_for_owner(self):
        entered, released = threading.Event(), threading.Event()
        service, source = self.service([])
        outcomes = []
        def acquire(*, cancelled, deadline):
            entered.set()
            cancelled.wait(1)
            self.assertEqual(source.closed, 0)
            released.wait(1)
            return reading(error='Measurement cancelled')
        source.read = acquire
        def read():
            try:
                service.read(1)
            except Exception as error:
                outcomes.append(error)
        worker = threading.Thread(target=read)
        worker.start()
        self.assertTrue(entered.wait(1))
        self.assertTrue(service.status()['busy'])
        start = time.monotonic()
        with self.assertRaises(DistanceConflict):
            service.read(1)
        self.assertLess(time.monotonic()-start, .1)
        service.begin_close()
        self.assertFalse(service.status()['ready'])
        closer = threading.Thread(target=service.close)
        closer.start()
        self.assertEqual(source.closed, 0)
        released.set()
        worker.join(1)
        closer.join(1)
        self.assertFalse(worker.is_alive())
        self.assertFalse(closer.is_alive())
        self.assertEqual(source.closed, 1)
        self.assertIsInstance(outcomes[0], DistanceUnavailable)
        service.close()
        self.assertEqual(source.closed, 1)


if __name__ == '__main__':
    unittest.main()

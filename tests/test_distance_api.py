"""Distance HTTP contracts and responsiveness without GPIO or servo hardware."""
import http.client
import json
from pathlib import Path
import signal
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from roboter_arm.control.domain.distance import DistanceConflict, DistanceUnavailable
from roboter_arm.control.presentation import http_api as app
from roboter_arm.control.presentation.distance_api import DISTANCE_FAILURE, DISTANCE_PATHS


class Distance:
    def __init__(self):
        self.requests = []
        self.owner = threading.Lock()
        self.waiting = threading.Event()
        self.release = threading.Event()
        self.release.set()
        self.closing = False
        self.closed = False
        self.result = dict(distance_mm=36.0, mean_mm=36.0, min_mm=35.0, max_mm=37.0,
                           stdev_mm=0.816496580927726, attempts=5, valid=3, sound_speed_m_s=343,
                           elapsed_ms=800.0, readings=[
                               dict(captured_at='2026-10-08T12:00:00+00:00', pulse_us=204.08,
                                    distance_mm=35.0, error=None),
                               dict(captured_at='2026-10-08T12:00:00.2+00:00', pulse_us=None,
                                    distance_mm=None, error='Echo timeout'),
                               dict(captured_at='2026-10-08T12:00:00.4+00:00', pulse_us=209.91,
                                    distance_mm=36.0, error=None),
                               dict(captured_at='2026-10-08T12:00:00.6+00:00', pulse_us=215.74,
                                    distance_mm=37.0, error=None),
                               dict(captured_at='2026-10-08T12:00:00.8+00:00', pulse_us=10.0,
                                    distance_mm=None, error='Distance outside sensor range')])

    def status(self):
        return dict(enabled=True, ready=not self.closing, busy=self.owner.locked(),
                    last_error=None, defaults=dict(default_samples=5, max_samples=20,
                    sample_interval_ms=200, echo_timeout_ms=100, trigger_pulse_us=100,
                    sound_speed_m_s=343, min_mm=20, max_mm=4000, gpiochip=0,
                    trig_gpio=23, echo_gpio=24, aggregation='median'))

    def read(self, samples=5):
        if not self.owner.acquire(blocking=False):
            raise DistanceConflict('Distance measurement is busy')
        try:
            self.requests.append(samples)
            self.waiting.set()
            if not self.release.wait(3):
                raise RuntimeError('Fake distance measurement was not released')
            if self.closing:
                raise DistanceUnavailable('Distance sensor is closing')
            return self.result
        finally:
            self.owner.release()

    def begin_close(self):
        self.closing = True
        self.release.set()

    def close(self):
        if self.owner.locked():
            raise AssertionError('Sensor closed before the acquisition finished')
        self.closed = True


class DistanceAPITests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory(prefix='robot-distance-api-')
        root = Path(self.directory.name)
        self.driver = app.PreviewDriver()
        self.session = app.session_for(self.driver, list(range(6)), directory=root / 'poses',
                                       demo_directory=root / 'demos')
        self.distance = Distance()
        self.server = app.Server(self.session, 0, distance=self.distance)
        self.server.start()
        self.port = self.server.server_address[1]

    def tearDown(self):
        self.distance.release.set()
        self.server.stop()
        self.session.close()
        self.directory.cleanup()

    def request(self, method, path, values=None):
        connection = http.client.HTTPConnection('127.0.0.1', self.port, timeout=3)
        headers = {}
        if method == 'POST':
            headers = {'Origin':f'http://127.0.0.1:{self.port}', 'Content-Type':'application/json'}
        connection.request(method, path, None if values is None else json.dumps(values), headers)
        response = connection.getresponse()
        data = json.loads(response.read())
        status = response.status
        received_headers = {name.lower():value for name, value in response.getheaders()}
        connection.close()
        return status, data, received_headers

    def test_reads_return_statistics_and_individual_failures_without_servo_preparation(self):
        with patch.object(self.driver, 'prepare', wraps=self.driver.prepare) as prepare, \
                patch.object(self.session, 'stop', wraps=self.session.stop) as stop:
            status, values, headers = self.request('GET', '/api/distance/status')
            self.assertEqual(status, 200)
            self.assertTrue(values['enabled'])
            self.assertTrue(values['ready'])
            self.assertFalse(values['busy'])
            self.assertEqual(self.distance.requests, [])
            self.assertEqual(headers['cache-control'], 'no-store')
            for query, expected in (('', 5), ('?samples=1', 1), ('?samples=20', 20)):
                with self.subTest(query=query):
                    status, result, headers = self.request('GET', '/api/distance/read' + query)
                    self.assertEqual(status, 200)
                    self.assertEqual(result, self.distance.result)
                    self.assertEqual(self.distance.requests[-1], expected)
                    self.assertEqual(headers['cache-control'], 'no-store')
            prepare.assert_not_called()
            stop.assert_not_called()
        self.assertFalse(self.session.prepared)

    def test_query_validation_is_strict_and_does_not_acquire_sensor(self):
        invalid = ('samples=', 'samples=1.0', 'samples=true', 'samples=-1', 'samples=0',
                   'samples=21', 'samples=%2B1', 'samples=%201', 'samples=1%20',
                   'samples=%D9%A1', 'samples=1&samples=2', 'sample=5', 'samples=1&extra=1')
        for query in invalid:
            with self.subTest(query=query):
                status, result, headers = self.request('GET', '/api/distance/read?' + query)
                self.assertEqual(status, 400, result)
                self.assertEqual(set(result), {'error'})
                self.assertEqual(headers['cache-control'], 'no-store')
        self.assertEqual(self.distance.requests, [])

    def test_distance_failures_keep_error_shape_and_never_stop_servos(self):
        errors = ((DistanceConflict('Already measuring'), 409),
                  (DistanceUnavailable('No valid readings'), 503),
                  (ValueError('GPIO returned malformed data'), 500),
                  (RuntimeError('GPIO failed'), 500))
        for error, expected in errors:
            with self.subTest(error=error), patch.object(self.distance, 'read', side_effect=error), \
                    patch.object(self.session, 'stop', wraps=self.session.stop) as stop:
                status, result, headers = self.request('GET', '/api/distance/read')
                self.assertEqual(status, expected)
                self.assertEqual(result, {'error':DISTANCE_FAILURE if expected == 500 else str(error)})
                self.assertEqual(headers['cache-control'], 'no-store')
                stop.assert_not_called()
        with patch.object(self.distance, 'status', side_effect=RuntimeError('GPIO failed')), \
                patch.object(self.session, 'stop', wraps=self.session.stop) as stop:
            status, result, headers = self.request('GET', '/api/distance/status')
            self.assertEqual((status, result), (500, {'error':DISTANCE_FAILURE}))
            self.assertEqual(headers['cache-control'], 'no-store')
            stop.assert_not_called()

    def test_malformed_internal_responses_are_isolated_server_failures(self):
        for path, method in (('/api/distance/status', 'status'), ('/api/distance/read', 'read')):
            with self.subTest(path=path), patch.object(self.distance, method, return_value={}), \
                    patch.object(self.session, 'stop', wraps=self.session.stop) as stop:
                status, result, headers = self.request('GET', path)
                self.assertEqual((status, result), (500, {'error':DISTANCE_FAILURE}))
                self.assertEqual(headers['cache-control'], 'no-store')
                stop.assert_not_called()

    def test_concurrent_clients_conflict_without_delaying_status_or_stop(self):
        self.distance.release.clear()
        captured = []
        reader = threading.Thread(target=lambda:captured.append(
            self.request('GET', '/api/distance/read?samples=20')))
        reader.start()
        self.assertTrue(self.distance.waiting.wait(1))
        try:
            started = time.monotonic()
            losers = []
            clients = [threading.Thread(target=lambda:losers.append(
                self.request('GET', '/api/distance/read'))) for _ in range(8)]
            for client in clients:
                client.start()
            for client in clients:
                client.join(timeout=1)
                self.assertFalse(client.is_alive())
            self.assertEqual(len(losers), 8)
            self.assertTrue(all(status == 409 for status, _, _ in losers), losers)
            self.assertLess(time.monotonic() - started, 1)
            started = time.monotonic()
            status, values, _ = self.request('GET', '/api/distance/status')
            self.assertEqual(status, 200)
            self.assertTrue(values['busy'])
            self.assertEqual(self.request('POST', '/api/stop', {})[0], 200)
            self.assertLess(time.monotonic() - started, 1)
            self.assertEqual(self.distance.requests, [20])
            self.assertFalse(self.distance.closing)
        finally:
            self.distance.release.set()
            reader.join(timeout=3)
        self.assertFalse(reader.is_alive())
        self.assertEqual(captured[0][0], 200)

    def test_signal_disables_servos_before_cancelling_measurements(self):
        order = []
        stop = self.session.stop
        begin_close = self.distance.begin_close

        def stop_first():
            order.append('stop')
            return stop()

        def cancel_second():
            order.append('cancel')
            begin_close()

        with patch.object(self.session, 'stop', side_effect=stop_first), \
                patch.object(self.distance, 'begin_close', side_effect=cancel_second):
            self.server.uvicorn.handle_exit(signal.SIGTERM, None)
        self.assertEqual(order, ['stop', 'cancel'])
        self.assertTrue(self.server.uvicorn.should_exit)

    def test_server_stop_cancels_active_read_before_releasing_sensor_resources(self):
        self.distance.release.clear()
        captured = []
        reader = threading.Thread(target=lambda:captured.append(
            self.request('GET', '/api/distance/read')))
        reader.start()
        self.assertTrue(self.distance.waiting.wait(1))
        self.server.stop()
        reader.join(timeout=3)
        self.assertFalse(reader.is_alive())
        self.assertEqual(captured[0][0], 503)
        self.assertTrue(self.distance.closing)
        self.assertTrue(self.distance.closed)

    def test_default_sensor_is_disabled_without_importing_gpio(self):
        self.server.stop()
        with patch.dict(sys.modules, {'lgpio':None}), patch.object(self.driver, 'prepare') as prepare:
            self.server = app.Server(self.session, 0)
            self.server.start()
            self.port = self.server.server_address[1]
            status, values, headers = self.request('GET', '/api/distance/status')
            self.assertEqual(status, 200)
            self.assertFalse(values['enabled'])
            self.assertFalse(values['ready'])
            self.assertFalse(values['busy'])
            self.assertEqual(headers['cache-control'], 'no-store')
            status, values, headers = self.request('GET', '/api/distance/read')
            self.assertEqual(status, 503)
            self.assertEqual(set(values), {'error'})
            self.assertEqual(headers['cache-control'], 'no-store')
            prepare.assert_not_called()

    def test_swagger_describes_distance_endpoints_and_refusals(self):
        status, spec, _ = self.request('GET', '/openapi.json')
        self.assertEqual(status, 200)
        for path in DISTANCE_PATHS:
            operation = spec['paths'][path]['get']
            self.assertEqual(operation['tags'], ['Distance'])
            self.assertTrue({'200', '500'} <= set(operation['responses']))
        operation = spec['paths']['/api/distance/read']['get']
        self.assertTrue({'400', '409', '503'} <= set(operation['responses']))
        samples = next(parameter for parameter in operation['parameters'] if parameter['name'] == 'samples')
        self.assertEqual(samples['in'], 'query')
        self.assertFalse(samples['required'])
        self.assertEqual(samples['schema']['default'], 5)
        self.assertEqual(samples['schema']['type'], 'integer')
        self.assertEqual(samples['schema']['minimum'], 1)
        self.assertEqual(samples['schema']['maximum'], 20)


if __name__ == '__main__':
    unittest.main()

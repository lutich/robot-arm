"""Offline HTTP workflow for the commissioning frontend and authoritative API."""
import http.client
from html.parser import HTMLParser
import inspect
import json
from pathlib import Path
import signal
import socket
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import patch
import urllib.request

from fastapi.routing import APIRoute

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from roboter_arm.control.presentation import http_api as app
from test_motion import FakeTime


class ManualAPITests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory(prefix='robot-api-')
        self.time = FakeTime()
        self.driver = app.PreviewDriver()
        root = Path(self.directory.name)
        self.session = app.session_for(self.driver, list(range(6)), directory=root / 'poses',
                                   demo_directory=root / 'demos',
                                   clock=self.time.clock, sleep=self.time.sleep)
        self.server = app.Server(self.session, 0)
        self.server.start()
        self.port = self.server.server_address[1]

    def tearDown(self):
        self.server.stop()
        self.session.close()
        self.directory.cleanup()

    def request(self, method, path, data=None, *, authorized=True):
        headers = {}
        if authorized and method == 'POST':
            headers = {'Origin':f'http://127.0.0.1:{self.port}', 'Content-Type':'application/json'}
        return self.send(method, path, json.dumps(data).encode() if data is not None else None, headers)

    def send(self, method, path, body=None, headers=None):
        connection = http.client.HTTPConnection('127.0.0.1', self.port, timeout=2)
        connection.request(method, path, body, headers or {})
        response = connection.getresponse()
        body = response.read()
        content_type = response.getheader('Content-Type')
        connection.close()
        return response.status, json.loads(body) if content_type == 'application/json' else body

    def action(self, path, args=None):
        status, data = self.request('POST', '/api/' + path, args or {})
        self.assertEqual(status, 200, data)
        if self.session.worker:
            self.session.worker.join(timeout=2)
        return data['result']

    def test_static_and_status_reads_do_not_initialize_hardware(self):
        with patch.object(self.driver, 'prepare', wraps=self.driver.prepare) as prepare:
            for path in ('/', '/manual_control.css', '/manual_control.js', '/api/state'):
                self.assertEqual(self.request('GET', path)[0], 200)
            prepare.assert_not_called()
        self.assertEqual(self.request('GET', '/../../.env')[0], 404)

    def test_all_new_mutations_require_a_same_origin_page(self):
        paths = ('go_pose', 'limits', 'settings', 'jog', 'demo/name', 'demo/add', 'demo/replace',
                 'demo/rename', 'demo/reorder', 'demo/remove', 'demo/next', 'demo/run',
                 'demo/pause', 'demo/resume', 'demo/restart', 'demo/save', 'demo/load')
        for path in paths:
            with self.subTest(path=path):
                self.assertEqual(self.request('POST', '/api/' + path, {}, authorized=False)[0], 403)
        self.assertFalse(self.session.prepared)

    def test_settings_and_relative_jog_use_authoritative_backend_values(self):
        self.action('settings', {'step_size':3, 'demo_speed':5})
        state = self.request('GET', '/api/state')[1]
        self.assertEqual(state['settings'], dict(step_size=3, demo_speed=5, demo_speed_max=20))
        self.assertFalse(state['prepared'])
        self.assertEqual(self.request('POST', '/api/settings', {'step_size':4, 'demo_speed':21})[0], 400)
        self.assertEqual(self.request('POST', '/api/jog', {'channel':4, 'direction':1})[0], 400)
        self.action('power_on', {'park_confirmed':True})
        self.action('jog', {'channel':4, 'direction':1})
        self.assertEqual(self.session.commands[4], 483)
        self.action('jog', {'channel':4, 'direction':-1})
        self.assertEqual(self.session.commands, app.HOME)
        self.assertEqual(self.session.budget.moves, 2)

    def test_joint_buttons_submit_the_intended_count_direction(self):
        class JogButtons(HTMLParser):
            def __init__(self):
                super().__init__()
                self.buttons = {}
                self.symbols = {}
                self.order = []
                self.current = None

            def handle_starttag(self, tag, attributes):
                attributes = dict(attributes)
                if tag == 'button' and 'data-jog-channel' in attributes:
                    self.current = (int(attributes['data-jog-channel']), int(attributes['data-direction']))
                    self.order.append(self.current)
                if tag == 'use' and self.current:
                    self.symbols[self.current[0], attributes['href']] = self.current[1]

            def handle_data(self, text):
                if self.current and text.strip() in ('↑', '↓', '+', '−'):
                    self.buttons[self.current[0], text.strip()] = self.current[1]

            def handle_endtag(self, tag):
                if tag == 'button':
                    self.current = None

        parser = JogButtons()
        parser.feed(self.request('GET', '/')[1].decode())
        self.action('power_on', {'park_confirmed':True})
        for channel, up in ((1,1), (2,-1), (3,1)):
            start = self.session.commands[channel]
            self.assertEqual(parser.buttons[channel, '↑'], up)
            self.assertEqual(parser.buttons[channel, '↓'], -up)
            self.action('jog', {'channel':channel, 'direction':parser.buttons[channel, '↑']})
            self.assertEqual(self.session.commands[channel], start + up*self.session.step_size)
            self.action('jog', {'channel':channel, 'direction':parser.buttons[channel, '↓']})
            self.assertEqual(self.session.commands[channel], start)
        for channel in (0,4):
            start = self.session.commands[channel]
            self.assertEqual(parser.symbols[channel, '#rotate-clockwise'], 1)
            self.assertEqual(parser.symbols[channel, '#rotate-counterclockwise'], -1)
            for symbol, target in (('#rotate-clockwise',start+self.session.step_size),
                                   ('#rotate-counterclockwise',start)):
                self.action('jog', {'channel':channel, 'direction':parser.symbols[channel, symbol]})
                self.assertEqual(self.session.commands[channel], target)
        self.assertEqual([direction for channel,direction in parser.order if channel==5], [-1,1])
        self.assertEqual(parser.buttons[5, '−'], -1)
        self.assertEqual(parser.buttons[5, '+'], 1)
        start = self.session.commands[5]
        for label, target in (('−',start-self.session.step_size), ('+',start)):
            self.action('jog', {'channel':5, 'direction':parser.buttons[5, label]})
            self.assertEqual(self.session.commands[5], target)

    def test_save_updates_home_and_park_in_status_and_subsequent_moves(self):
        self.action('power_on', {'park_confirmed':True})
        for name, target in (('HOME',481), ('PARK',482)):
            self.action('move', {'channel':4, 'target':target, 'first_clear':False})
            before = dict(self.driver.outputs)
            filename = self.action('save', {'name':name, 'observed':True, 'provenance':'API preview test'})
            self.assertTrue((Path(self.directory.name) / 'poses' / 'synthetic' / filename).is_file())
            self.assertEqual(self.driver.outputs, before)
            state = self.request('GET', '/api/state')[1]
            self.assertEqual(state['joints'][4][name.lower()], target)
        for name, target in (('HOME',481), ('PARK',482)):
            self.action('go_pose', {'name':name})
            self.assertEqual(self.session.commands[4], target)

    def test_whole_arm_limits_capture_edit_save_load_and_step_execution(self):
        self.action('power_on', {'park_confirmed':True})
        self.action('go_pose', {'name':'PARK'})
        self.assertEqual(self.session.commands, app.PARK)
        self.assertTrue(self.session.armed)
        self.action('go_pose', {'name':'HOME'})
        before = dict(self.driver.outputs)
        self.action('limits', {'channel':2, 'low':390, 'high':600})
        self.assertEqual(self.driver.outputs, before)
        self.action('demo/name', {'name':'API rehearsal'})
        self.action('demo/add')
        self.action('move', {'channel':4, 'target':481, 'first_clear':False})
        self.action('demo/add', {'name':'Turned wrist'})
        self.action('demo/rename', {'index':0, 'name':'Ready'})
        self.action('demo/reorder', {'index':1, 'direction':-1})
        self.action('demo/reorder', {'index':0, 'direction':1})
        self.action('demo/replace', {'index':1})
        filename = self.action('demo/save')
        saved = Path(self.directory.name) / 'demos' / 'synthetic' / filename
        content = saved.read_bytes()
        self.action('demo/remove', {'index':1})
        self.action('demo/load', {'filename':filename})
        demo = self.request('GET', '/api/state')[1]['demo']
        self.assertEqual(demo['name'], 'API rehearsal')
        self.assertEqual(len(demo['positions']), 2)
        self.assertEqual(demo['cursor'], 0)
        self.action('demo/next')
        self.assertEqual(self.session.commands, app.HOME)
        self.assertEqual(self.request('GET', '/api/state')[1]['demo']['cursor'], 1)
        self.action('demo/next')
        self.assertEqual(self.session.commands[4], 481)
        self.assertTrue(self.session.armed)
        self.assertEqual(saved.read_bytes(), content)
        self.action('demo/restart')
        self.action('demo/run')
        self.assertEqual(self.request('GET', '/api/state')[1]['demo']['cursor'], 2)
        self.action('stop')
        state = self.request('GET', '/api/state')[1]
        self.assertFalse(state['armed'])
        self.assertTrue(state['outputs_off'])
        self.assertEqual(state['demo']['cursor'], 0)
        self.assertTrue(all(value is None for value in state['commands'].values()))

    def test_bad_requests_do_not_move(self):
        self.action('power_on', {'park_confirmed':True})
        before = dict(self.driver.outputs)
        for path, args in (('limits', {'channel':2, 'low':500, 'high':600}),
                           ('go_pose', {'name':'unknown'}),
                           ('demo/load', {'filename':'../../.env'}),
                           ('demo/rename', {'index':False, 'name':'bad'})):
            with self.subTest(path=path):
                self.assertEqual(self.request('POST', '/api/' + path, args)[0], 400)
                self.assertEqual(self.driver.outputs, before)

    def test_strict_json_types_never_power_on_or_move(self):
        for args, field in (({'park_confirmed':'yes'}, 'park_confirmed'), ({'park_confirmed':1}, 'park_confirmed'),
                            ({'park_confirmed':True, 'extra':1}, 'extra')):
            with self.subTest(args=args):
                status, data = self.request('POST', '/api/power_on', args)
                self.assertEqual(status, 400)
                self.assertTrue(data['error'].startswith(field + ': '), data)
        self.assertFalse(self.session.armed)
        self.action('power_on', {'park_confirmed':True})
        before = dict(self.driver.outputs)
        for channel in ('4', True, 4.0):
            with self.subTest(channel=channel):
                status, data = self.request('POST', '/api/move', {'channel':channel, 'target':410, 'first_clear':False})
                self.assertEqual(status, 400)
                self.assertTrue(data['error'].startswith('channel: '), data)
        for junk in ([], {'index':0}, 'next'):
            with self.subTest(junk=junk):
                self.assertEqual(self.request('POST', '/api/demo/next', junk)[0], 400)
        self.assertEqual(self.driver.outputs, before)

    def test_refusals_and_unknown_paths_keep_the_error_shape(self):
        origin = {'Origin':f'http://127.0.0.1:{self.port}'}
        json_type = origin | {'Content-Type':'application/json'}
        for method, path, body, headers, status, error in (
                ('POST', '/api/unknown', b'{}', json_type, 404, 'Unknown path'),
                ('GET', '/api/move', None, {}, 405, 'Method Not Allowed'),
                ('POST', '/', b'{}', json_type, 405, 'Method Not Allowed'),
                ('POST', '/api/stop', b'{}', origin | {'Content-Type':'text/plain'}, 400, 'Content-Type must be application/json'),
                ('POST', '/api/stop', b'{}', origin, 400, 'Content-Type must be application/json'),
                ('POST', '/api/stop', b'', json_type, 400, 'Request too large or empty'),
                ('POST', '/api/demo/name', json.dumps({'name':'x' * 8200}).encode(), json_type, 400, 'Request too large or empty'),
                ('POST', '/api/stop', b'{not json', json_type, 400, 'JSON decode error')):
            with self.subTest(method=method, path=path, headers=headers, body=body[:20] if body else body):
                self.assertEqual(self.send(method, path, body, headers), (status, {'error':error}))
        self.assertFalse(self.session.armed)

    def test_missing_host_is_refused(self):
        # HTTP/1.1 without Host never reaches the app (h11 answers 400); HTTP/1.0 may omit it.
        with socket.create_connection(('127.0.0.1', self.port), timeout=2) as connection:
            connection.sendall(b'GET /api/state HTTP/1.0\r\n\r\n')
            response = b''
            while chunk := connection.recv(4096):
                response += chunk
        head, _, content = response.partition(b'\r\n\r\n')
        self.assertTrue(head.startswith(b'HTTP/1.1 403'), head)
        self.assertEqual(json.loads(content), {'error':'Use an IP address or local host name'})

    def test_a_shutdown_signal_stops_the_session_before_requests_drain(self):
        with patch.object(self.session, 'stop', wraps=self.session.stop) as stop:
            self.server.uvicorn.handle_exit(signal.SIGTERM, None)
        stop.assert_called_once_with()
        self.assertTrue(self.server.uvicorn.should_exit)

    def test_unexpected_failure_stops_the_session(self):
        with patch.object(self.session, 'demo_restart', side_effect=RuntimeError('driver gone')), \
                patch.object(self.session, 'stop', wraps=self.session.stop) as stop:
            self.assertEqual(self.request('POST', '/api/demo/restart', {}), (500, {'error':app.FAILURE}))
        stop.assert_called_once_with(app.FAILURE)

    def test_docs_and_openapi_describe_exactly_the_api(self):
        status, page = self.request('GET', '/docs')
        self.assertEqual(status, 200)
        self.assertIn(b'swagger-ui', page)
        status, spec = self.request('GET', '/openapi.json')
        self.assertEqual(status, 200)
        self.assertEqual(set(spec['paths']), {'/api/state', '/api/stop', *(f'/api/{path}' for path in app.ACTIONS)})
        self.assertEqual(len(spec['paths']), 25)
        power_on = spec['components']['schemas']['PowerOn']
        self.assertEqual(power_on['properties']['park_confirmed']['type'], 'boolean')
        self.assertFalse(power_on['additionalProperties'])
        tags = {path: operation['tags'] for path, item in spec['paths'].items() for operation in item.values()}
        self.assertEqual([tag['name'] for tag in spec['tags']], [tag['name'] for tag in app.TAGS])
        self.assertTrue(all(len(names) == 1 and names[0] in {tag['name'] for tag in app.TAGS} for names in tags.values()), tags)
        self.assertEqual({path for path, names in tags.items() if names == ['Bounded tests']}, {'/api/prepare', '/api/arm'})
        self.assertEqual(self.send('GET', '/docs', None, {'Host':'evil.example'})[0], 403)

    def test_only_stop_is_async_and_it_never_waits_for_the_shared_threads(self):
        for route in app.create_app(self.session).routes:
            if isinstance(route, APIRoute):
                with self.subTest(path=route.path):
                    self.assertEqual(inspect.iscoroutinefunction(route.endpoint), route.path == '/api/stop')
        release = threading.Event()
        state = self.session.state
        def blocked():
            release.wait(5)
            return state()
        with patch.object(self.session, 'state', side_effect=blocked):
            readers = [threading.Thread(target=self.request, args=('GET', '/api/state')) for _ in range(45)]
            for reader in readers:
                reader.start()
            time.sleep(.3)
            started = time.monotonic()
            status = self.request('POST', '/api/stop', {})[0]
            elapsed = time.monotonic() - started
            release.set()
            for reader in readers:
                reader.join()
        self.assertEqual(status, 200)
        self.assertLess(elapsed, 1)


class ProcessTests(unittest.TestCase):
    def test_signals_shut_the_app_down_cleanly(self):
        root = Path(__file__).resolve().parents[1]
        opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
        for sig in (signal.SIGTERM, signal.SIGHUP):
            with self.subTest(signal=sig.name):
                with socket.socket() as probe:
                    probe.bind(('127.0.0.1', 0))
                    port = probe.getsockname()[1]
                process = subprocess.Popen([sys.executable, 'scripts/manual_control.py', '--port', str(port)], cwd=root,
                                           stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
                try:
                    deadline = time.monotonic() + 15
                    while True:
                        try:
                            with opener.open(f'http://127.0.0.1:{port}/api/state', timeout=.5) as response:
                                self.assertEqual(json.load(response)['mode'], 'preview')
                            break
                        except OSError:
                            if process.poll() is not None or time.monotonic() > deadline:
                                self.fail('Preview did not start: ' + process.stderr.read())
                            time.sleep(.05)
                    started = time.monotonic()
                    process.send_signal(sig)
                    process.wait(timeout=5)
                    self.assertLess(time.monotonic() - started, 5)
                finally:
                    if process.poll() is None:
                        process.kill()
                    stdout, stderr = process.communicate()
                self.assertEqual(process.returncode, 0, stderr)
                self.assertIn('PREVIEW, no hardware', stdout)


if __name__ == '__main__':
    unittest.main()

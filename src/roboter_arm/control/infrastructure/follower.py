"""The mock controller following a real one (manual_control.py --follow http://<pi>:8765).

Reads the real controller's /api/state and feeds its commands into the simulation, and forwards
the page's API requests to it with the real controller's own Host and Origin, so its same-origin
check accepts them. Nothing here decides about motion; the real controller does.
"""
import http.client
import json
import threading
from urllib.parse import urlsplit

TIMEOUT = 5
CHUNK = 65536


class Follower:
    def __init__(self, url, arm, *, interval=0.1):
        parts = urlsplit(url)
        if parts.scheme != 'http' or not parts.hostname or parts.path not in ('', '/') or parts.query:
            raise ValueError('Follow a controller by its base URL, e.g. http://raspberrypi.local:8765')
        self.host, self.port = parts.hostname, parts.port or 80
        self.origin = f'http://{parts.netloc}'
        self.url = self.origin
        self.arm, self.interval = arm, interval
        self.stopped = threading.Event()
        self.thread = None
        arm.following = self.url

    def connection(self):
        return http.client.HTTPConnection(self.host, self.port, timeout=TIMEOUT)

    # ---------------- state into the simulation ----------------

    def poll(self):
        connection = self.connection()
        try:
            connection.request('GET', '/api/state', headers={'Host': self.origin[7:]})
            response = connection.getresponse()
            body = response.read()
            if response.status != 200:
                raise RuntimeError(f'/api/state answered {response.status}')
            state = json.loads(body)
        finally:
            connection.close()
        commands, park = state['commands'], {j['channel']: j.get('park') for j in state.get('joints', [])}
        self.arm.observe([commands.get(str(c), commands.get(c)) for c in range(6)], state.get('outputs_off') is not False,
                         park=[park.get(c) for c in range(6)])
        self.arm.follow_error = None

    def run(self):
        while not self.stopped.wait(self.interval):
            try:
                self.poll()
            except Exception as error:
                self.arm.follow_error = f'{type(error).__name__}: {error}'

    def start(self):
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def stop(self):
        self.stopped.set()
        if self.thread is not None:
            self.thread.join(timeout=TIMEOUT)

    # ---------------- requests to the real controller ----------------

    def forward(self, method, target, body, content_type):
        """Send one request on; returns (status, content type, connection, response) to stream from."""
        headers = {'Host': self.origin[7:]}
        if method == 'POST':
            headers |= {'Origin': self.origin, 'Content-Type': content_type or 'application/json'}
        connection = self.connection()
        try:
            connection.request(method, target, body=body or None, headers=headers)
            response = connection.getresponse()
        except BaseException:
            connection.close()
            raise
        return response.status, response.getheader('Content-Type'), connection, response

"""Clients that speak the robot-arm API: over HTTP, or directly to a Session in tests.

Both expose the same methods, so the task layer and the text interface do not care whether
they talk to the sandbox, the real arm, or an in-process session.
"""
import json
import threading
import time
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

CHANNELS = range(6)
PITCH_FIRST = (3, 2, 1, 0, 4, 5)   # single-joint fallback: pitch joints before the base


class ArmError(Exception):
    """The API refused a request."""


class _Base:
    def wait_idle(self, timeout=120.0):
        raise NotImplementedError

    def commands(self):
        return {int(k): v for k, v in self.state()['commands'].items()}

    def move_pose(self, counts, speed=None, wait=True):
        """All six joints together; falls back to single-joint moves when the server lacks move_pose."""
        pose = {c: int(counts[c]) for c in CHANNELS}
        try:
            self._move_pose(pose, speed)
        except ArmError as error:
            if 'Unknown path' not in str(error):
                raise
            self._move_pose_fallback(pose)
            return
        if wait:
            self.wait_idle()

    def _move_pose_fallback(self, pose):
        current = self.commands()
        if any(current.get(c) is None for c in CHANNELS):
            raise ArmError('All six commands must be known before moving a pose')
        # Lifting: move the pitch joints before the base; lowering: base first.
        order = PITCH_FIRST if pose[1] <= current[1] else tuple(reversed(PITCH_FIRST[:4])) + PITCH_FIRST[4:]
        for channel in order:
            if pose[channel] != current[channel]:
                self.move(channel, pose[channel])
                self.wait_idle()


class HttpArmClient(_Base):
    """Same-origin JSON requests against the robot-arm HTTP API, sandbox or real arm."""

    def __init__(self, base_url='http://127.0.0.1:8765', timeout=10.0):
        self.base, self.timeout = base_url.rstrip('/'), timeout

    def _request(self, path, body=None):
        url = f'{self.base}{path}'
        if body is None:
            request = Request(url)
        else:
            request = Request(url, data=json.dumps(body).encode(), method='POST',
                              headers={'Content-Type': 'application/json', 'Origin': self.base})
        try:
            with urlopen(request, timeout=self.timeout) as response:
                return json.load(response)
        except HTTPError as error:
            try:
                message = json.load(error).get('error', str(error))
            except ValueError:
                message = str(error)
            raise ArmError(message) from None
        except URLError as error:
            raise ArmError(f'No server at {self.base}: {error.reason}') from None

    def state(self):
        return self._request('/api/state')

    def action(self, path, **body):
        result = self._request(f'/api/{path}', body)
        if not result.get('ok'):
            raise ArmError(result.get('error', 'refused'))
        return result.get('result')

    def power_on(self):
        return self.action('power_on', park_confirmed=True)

    def power_off(self):
        return self.action('power_off')

    def stop(self):
        return self.action('stop')

    def move(self, channel, target, first_clear=False):
        return self.action('move', channel=int(channel), target=int(target), first_clear=first_clear)

    def jog(self, channel, direction):
        return self.action('jog', channel=int(channel), direction=int(direction))

    def go_pose(self, name):
        return self.action('go_pose', name=name)

    def settings(self, step_size=None, demo_speed=None):
        current = self.state()['settings']
        return self.action('settings', step_size=int(step_size or current['step_size']),
                           demo_speed=int(demo_speed or current['demo_speed']))

    def _move_pose(self, pose, speed):
        body = dict(counts=[pose[c] for c in CHANNELS])
        if speed is not None:
            body['speed'] = int(speed)
        return self.action('move_pose', **body)

    def demo(self, action, **body):
        return self.action(f'demo/{action}', **body)

    def sim(self, path, body=None):
        return self._request(f'/sim/{path}', body)

    def wait_idle(self, timeout=120.0):
        deadline = time.monotonic() + timeout
        while True:
            state = self.state()
            if not state['busy']:
                if state.get('error'):
                    raise ArmError(state['error'])
                return state
            if time.monotonic() > deadline:
                raise ArmError('Timed out waiting for the arm')
            time.sleep(0.05)


class SessionArmClient(_Base):
    """Direct calls into a Session; waiting joins the worker thread, so a sim clock works."""

    def __init__(self, session):
        self.session = session

    def state(self):
        return self.session.state()

    def _call(self, method, **kwargs):
        try:
            return getattr(self.session, method)(**kwargs)
        except (ValueError, TypeError, KeyError) as error:
            raise ArmError(str(error)) from None
        except AttributeError:
            raise ArmError('Unknown path') from None

    def power_on(self):
        return self._call('power_on', park_confirmed=True)

    def power_off(self):
        return self._call('power_off')

    def stop(self):
        return self._call('stop')

    def move(self, channel, target, first_clear=False):
        return self._call('move', channel=channel, target=target, first_clear=first_clear)

    def jog(self, channel, direction):
        return self._call('jog', channel=channel, direction=direction)

    def go_pose(self, name):
        return self._call('go_pose', name=name)

    def settings(self, step_size=None, demo_speed=None):
        return self._call('set_settings', step_size=step_size or self.session.step_size,
                          demo_speed=demo_speed or self.session.demo_speed)

    def _move_pose(self, pose, speed):
        if not hasattr(self.session, 'move_pose'):
            raise ArmError('Unknown path')
        return self._call('move_pose', counts=pose, speed=speed)

    def demo(self, action, **body):
        return self._call(f'demo_{action}', **body)

    def wait_idle(self, timeout=120.0):
        worker = self.session.worker
        if worker is not None and worker is not threading.current_thread():
            worker.join(timeout)
        state = self.session.state()
        if state['busy']:
            raise ArmError('Timed out waiting for the arm')
        if state.get('error'):
            raise ArmError(state['error'])
        return state

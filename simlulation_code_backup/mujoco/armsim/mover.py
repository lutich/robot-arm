"""Mover: "go to x, y on the table and lower the gripper there", as a background job with status.

Used by the sandbox server (in-process session) and by the 3D-view proxy in front of the real
arm (HTTP client). The poses come from the inverse kinematics and are sent as move_pose steps,
so both targets receive the same counts.
"""
import threading
import time

from armsim.client import ArmError
from armsim.kinematics import Kinematics
from armsim.robotarm import BOUNDS

OPEN_GAP = 0.045


class Mover:
    def __init__(self, client, robot, scene):
        self.client, self.scene = client, scene
        self.kin = Kinematics(robot, scene, BOUNDS)
        self.task = scene['task']
        self.lock = threading.Lock()
        self.state = dict(running=False, step=None, error=None, done=None, target=None, log=[])

    def status(self):
        with self.lock:
            return dict(self.state, log=list(self.state['log']))

    def poses(self, x, y, z, yaw=0.0):
        """The two poses of a move: above the point at safe height, then lowered to z."""
        above = self.kin.inverse_counts((x, y, z + self.task['safe_height']), OPEN_GAP, yaw)
        low = self.kin.inverse_counts((x, y, z), OPEN_GAP, yaw)
        return above, low

    def plan(self, x, y, z=0.01325, yaw=0.0, lower=True, path='direct'):
        """The poses a move would send, without sending them (for the view's virtual mode).

        path 'direct': one pose straight to the target (lowered, or above it when lower=False).
        path 'safe': rise to safe height first, travel above the target, then lower.
        """
        if not 0 <= x <= self.scene['table']['depth'] or abs(y) > self.scene['table']['width'] / 2:
            raise ArmError(f'({x * 1000:.0f}, {y * 1000:.0f}) mm is outside the table')
        if z < 0.005:
            raise ArmError('z must stay at least 5 mm above the table')
        try:
            above, low = self.poses(x, y, z, yaw)
        except ValueError as error:
            raise ArmError(f'({x * 1000:.0f}, {y * 1000:.0f}, {z * 1000:.0f}) mm is not reachable: {error}') from None
        if path not in ('direct', 'safe'):
            raise ArmError("path must be 'direct' or 'safe'")
        pose = lambda label, counts, speed: dict(label=label, counts=[int(counts[c]) for c in range(6)], speed=speed)
        if path == 'direct':
            return [pose('go to target' if lower else 'go above target', low if lower else above, self.task['travel_speed'])]
        steps = []
        rise = self.rise_pose()
        if rise is not None:
            steps.append(pose('rise', rise, self.task['approach_speed']))
        steps.append(pose('travel above target', above, self.task['travel_speed']))
        if lower:
            steps.append(pose('lower', low, self.task['approach_speed']))
        return steps

    def rise_pose(self):
        """Straight up from the current pose to safe height, or None if unknown, unneeded or unreachable."""
        try:
            current = self.client.commands()
        except ArmError:
            return None
        if any(current.get(c) is None for c in range(6)):
            return None
        fk = self.kin.forward_counts(current)
        rise_z = max(fk['grasp'][2], 0.01325 + self.task['safe_height'])
        try:
            rise = self.kin.inverse_counts((fk['grasp'][0], fk['grasp'][1], rise_z), OPEN_GAP, 0.0)
        except ValueError:
            return None
        rise[4], rise[5] = current[4], current[5]
        return None if rise == current else rise

    def start(self, x, y, z=0.01325, yaw=0.0, lower=True, path='direct'):
        with self.lock:
            if self.state['running']:
                raise ArmError('A move is already running')
        steps = self.plan(x, y, z, yaw, lower, path)
        with self.lock:
            self.state.update(running=True, step='starting', error=None, done=None, target=[x, y, z], log=[])
        threading.Thread(target=self._run, args=(steps,), daemon=True).start()

    def _go(self, label, counts, speed):
        with self.lock:
            self.state['step'] = label
            self.state['log'].append(dict(label=label, counts=[int(counts[c]) for c in range(6)], speed=speed))
        self.client.move_pose(counts, speed=speed)

    def _run(self, steps):
        try:
            current = self.client.commands()
            if any(current.get(c) is None for c in range(6)):
                raise ArmError('Power on first: the joint commands are unknown')
            for step in steps:
                self._go(step['label'], dict(enumerate(step['counts'])), step['speed'])
            with self.lock:
                self.state.update(done=True, step='done')
        except Exception as error:
            with self.lock:
                self.state.update(done=False, error=f'{type(error).__name__}: {error}', step='failed')
        finally:
            with self.lock:
                self.state['running'] = False

    def stop(self):
        try:
            self.client.stop()
        except ArmError:
            pass

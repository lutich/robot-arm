"""The mock controller's servo driver: every command goes into the arm simulation (roboter_arm.sim).

Preview (blocking): the synthetic engine. A write that would bring a new contact (arm into
the board, itself or a cube) is refused before it reaches the driver; MotionRunner then
disables the outputs, like a real emergency stop.
Following (observing): the Follower feeds the real controller's commands in through observe();
contacts are only reported, because the model's servo calibration is not measured.
"""
import threading

from roboter_arm.sim.domain.arm_model import MODEL, REST_ZONE
from roboter_arm.sim.domain.collision import arm_shapes
from roboter_arm.sim.domain.kinematics import angles_from_counts, tool_yaw_and_tilt
from roboter_arm.sim.domain.world import World

# MotionRunner writes one channel at a time, so a contact can drop out for a write or two while
# the joints catch up. A contact is new only if it was clear for this many writes (2 samples).
RECENT_WRITES = 12


class SimulatedCollision(RuntimeError):
    pass


def describe(contacts):
    return ', '.join(f'{k.a} × {k.b} {-k.sep * 10:.1f} mm' for k in contacts)


class SimulatedArm:
    def __init__(self, driver, *, blocking, rest):
        self.driver, self.blocking = driver, blocking
        self.rest = [rest[c] for c in range(6)]
        self.counts = list(self.rest)
        self.world = World()
        self.lock = threading.Lock()
        self.outputs_off = True
        self.stopped_by = None
        self.following = None     # URL of the real controller while the Follower feeds observe()
        self.follow_error = None  # why the last read of the real controller failed
        self.writes = 0
        self.recent = {}          # contact pair -> last write it was seen in
        self.world.step(self.counts)
        self._remember(self.world.contacts)

    @property
    def scope(self):
        return self.driver.scope

    def prepare(self):
        self.driver.prepare()

    def resting(self, counts):
        return all(abs(c - r) <= REST_ZONE for c, r in zip(counts, self.rest))

    def _remember(self, contacts):
        for k in contacts:
            self.recent[(k.a, k.b)] = self.writes

    def _apply(self, counts, *, blocking):
        """Step the world to counts (lock held); refuse a new hard contact when blocking."""
        before = self.world.copy()
        self.writes += 1
        fresh = [k for k in self.world.step(counts)
                 if self.recent.get((k.a, k.b), -RECENT_WRITES - 1) < self.writes - RECENT_WRITES]
        hard = [k for k in fresh if not k.pressing]
        if hard and not self.resting(counts):
            if blocking:
                self.world = before
                self.stopped_by = describe(hard)
                self.world.event('collision', f'stopped: {self.stopped_by}')
                raise SimulatedCollision(self.stopped_by)
            self.world.event('warn', f'model contact (real arm not stopped): {describe(hard)}')
        if pressing := [k for k in fresh if k.pressing]:
            self.world.event('press', f'pressing: {describe(pressing)} (the servo stops short of its command)')
        self._remember(self.world.contacts)
        self.counts, self.stopped_by = counts, None

    def write(self, channel, count):
        if not 0 <= channel <= 5:
            return self.driver.write(channel, count)
        if not self.blocking:
            self.driver.write(channel, count)
        with self.lock:
            counts = list(self.counts)
            counts[channel] = count
            self._apply(counts, blocking=self.blocking)
            self.outputs_off = False
        if self.blocking:
            self.driver.write(channel, count)

    def observe(self, commands, outputs_off, park=None):
        """The real arm's commands (None keeps the last known count) and PARK; never refuses."""
        with self.lock:
            if park and None not in park:
                self.rest = list(park)
            counts = [self.counts[c] if commands[c] is None else commands[c] for c in range(6)]
            if counts != self.counts:
                self._apply(counts, blocking=False)
            self.outputs_off = outputs_off

    def disable(self, channel):
        self.driver.disable(channel)
        if 0 <= channel <= 5:
            with self.lock:
                self.outputs_off = True

    def verify_off(self):
        self.driver.verify_off()

    def close(self):
        self.driver.close()

    # ---------------- simulation API ----------------

    @staticmethod
    def model():
        return MODEL

    def snapshot(self):
        with self.lock:
            w = self.world
            yaw, tilt = tool_yaw_and_tilt(w.frames.hand)
            return {
                'mode': 'following' if self.following else 'synthetic' if self.blocking else 'observing',
                'following': self.following,
                'follow_error': self.follow_error,
                'counts': list(self.counts),
                'angles': list(angles_from_counts(self.counts)),
                'gap': w.gap,
                'grasp': list(w.frames.grasp),
                'tool': {'yaw': yaw, 'tilt': tilt},
                'cubes': [c.json() for c in w.cubes],
                'holding': w.holding.id if w.holding else None,
                'contacts': [{'a': k.a, 'b': k.b, 'sep': k.sep, 'pressing': k.pressing} for k in w.contacts],
                'resting': self.resting(self.counts),
                'shapes': [s.json() for s in arm_shapes(w.frames, w.gap)],
                'events': list(w.events[-50:]),
                'outputs_off': self.outputs_off,
                'stopped_by': self.stopped_by,
            }

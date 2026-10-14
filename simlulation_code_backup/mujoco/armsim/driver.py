"""MujocoDriver: robot-arm's ServoDriver port backed by MuJoCo physics.

Construction never starts anything. Commands are counts, exactly as the PCA9685 driver takes
them; there is no position feedback, matching the real arm.
"""
from armsim.physics import Physics

CHANNELS = range(6)


class MujocoDriver:
    scope = 'synthetic'

    def __init__(self, physics=None):
        self.physics = physics if physics is not None else Physics()
        self.outputs = {c: 0 for c in range(16)}
        self.prepared = False
        self.writes = []   # (sim time, channel, count), for tests and logs

    def prepare(self):
        self.outputs = {c: 0 for c in range(16)}
        for c in CHANNELS:
            self.physics.enable(c, False)
        self.prepared = True

    def write(self, channel, count):
        if type(channel) is not int or channel not in CHANNELS:
            raise ValueError('Channel must be 0–5')
        if type(count) is not int or not 1 <= count <= 4095:
            raise ValueError('Count must be an integer PCA9685 count')
        self.outputs[channel] = count
        self.physics.set_target(channel, count)
        self.physics.enable(channel, True)
        self.writes.append((self.physics.time, channel, count))

    def disable(self, channel):
        self.outputs[channel] = 0
        if channel in CHANNELS:
            self.physics.enable(channel, False)

    def verify_off(self):
        if any(self.outputs.values()):
            raise RuntimeError('Output-off verification failed')

    def close(self):
        for c in CHANNELS:
            self.physics.enable(c, False)
        self.prepared = False

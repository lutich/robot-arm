"""Counts <-> joint angles. Counts are PCA9685 counts at nominal 60 Hz, as in robot-arm.

angle = direction * (count - home_count) / counts_per_degree + home_angle
The gripper channel maps linearly between its open and closed counts.
"""
import math

CHANNELS = range(6)
GRIPPER = 5


class Calibration:
    def __init__(self, robot):
        self.channels = {}
        for channel in CHANNELS:
            spec = robot['channels'][channel]
            if channel == GRIPPER:
                continue
            for key in ('home_count', 'counts_per_degree', 'direction', 'home_angle'):
                if key not in spec:
                    raise ValueError(f'channel {channel} needs {key}')
            if spec['direction'] not in (-1, 1) or spec['counts_per_degree'] <= 0:
                raise ValueError(f'channel {channel}: direction must be ±1 and counts_per_degree positive')
            self.channels[channel] = dict(spec)
        gripper = robot['gripper']
        self.gripper = dict(open_count=int(gripper['open_count']), closed_count=int(gripper['closed_count']),
                            open_angle=float(gripper['open_angle']), closed_angle=float(gripper['closed_angle']))
        if self.gripper['open_count'] == self.gripper['closed_count']:
            raise ValueError('Gripper open and closed counts must differ')
        self.home_counts = {c: int(robot['channels'][c]['home_count']) for c in self.channels}
        self.home_counts[GRIPPER] = self.gripper['open_count']

    def angle(self, channel, count):
        """Joint angle in degrees for a count; the gripper returns its link angle."""
        if channel == GRIPPER:
            g = self.gripper
            fraction = (count - g['closed_count']) / (g['open_count'] - g['closed_count'])
            return g['closed_angle'] + fraction * (g['open_angle'] - g['closed_angle'])
        spec = self.channels[channel]
        return spec['direction'] * (count - spec['home_count']) / spec['counts_per_degree'] + spec['home_angle']

    def count(self, channel, angle):
        """Nearest integer count for a joint angle in degrees."""
        if channel == GRIPPER:
            g = self.gripper
            fraction = (angle - g['closed_angle']) / (g['open_angle'] - g['closed_angle'])
            return round(g['closed_count'] + fraction * (g['open_count'] - g['closed_count']))
        spec = self.channels[channel]
        return round(spec['home_count'] + spec['direction'] * (angle - spec['home_angle']) * spec['counts_per_degree'])

    def angles(self, counts):
        return {c: self.angle(c, counts[c]) for c in counts}

    def radians(self, channel, count):
        return math.radians(self.angle(channel, count))

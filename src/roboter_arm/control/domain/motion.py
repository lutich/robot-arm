"""Hardware-independent elapsed-time motion. No real-driver adapter is provided.

Counts and counts/second at nominal 60 Hz. Start commands are caller-established
conditions, never position feedback. Synthetic limits are only for fake drivers.
"""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class JointLimit:
    low: int
    high: int
    speed: float
    provenance: str
    scope: str  # 'synthetic', 'commissioning', or 'validated' evidence.


def finite(value):
    return type(value) in (int, float) and math.isfinite(value)


class Trajectory:
    def __init__(self, start, target, limits):
        if not start or start.keys() != target.keys():
            raise ValueError('Start and target must select the same nonempty joint set')
        self.start, self.target = dict(start), dict(target)
        self.duration = 0.0
        self.scopes = set()
        for channel in self.start:
            if type(channel) is not int or not 0 <= channel <= 5 or channel not in limits:
                raise ValueError('Every selected channel needs explicit limits')
            limit = limits[channel]
            if (type(limit.low) is not int or type(limit.high) is not int
                    or not 1 <= limit.low <= limit.high <= 4095
                    or not finite(limit.speed) or limit.speed <= 0
                    or not isinstance(limit.provenance, str) or not limit.provenance.strip()
                    or limit.scope not in ('synthetic', 'commissioning', 'validated')):
                raise ValueError('Limits, speed, provenance and calibration scope are required')
            self.scopes.add(limit.scope)
            for count in (self.start[channel], self.target[channel]):
                if type(count) is not int or not limit.low <= count <= limit.high:
                    raise ValueError('Start and target must be integer counts inside limits')
            # Cubic smoothstep peaks at 1.5 times the average speed.
            self.duration = max(self.duration, 1.5 * abs(self.target[channel] - self.start[channel]) / limit.speed)
        if not math.isfinite(self.duration):
            raise ValueError('Requested speed produces a nonfinite duration')

    def sample(self, elapsed):
        if not finite(elapsed) or elapsed < 0:
            raise ValueError('Elapsed time must be finite and nonnegative')
        if elapsed >= self.duration:
            return dict(self.target)
        fraction = elapsed / self.duration
        progress = fraction * fraction * (3 - 2 * fraction)
        return {channel: round(count + (self.target[channel] - count) * progress)
                for channel, count in self.start.items()}

    def validate_timing(self, timeout, interval):
        if not finite(interval) or interval <= 0 or not finite(timeout) or timeout <= 0:
            raise ValueError('Interval and timeout must be positive and finite')
        if self.duration > timeout:
            raise ValueError('Trajectory cannot finish within timeout')

"""Distance acquisition values and port; no GPIO or HTTP dependencies."""
from dataclasses import dataclass
from threading import Event
from typing import Protocol


class DistanceConflict(Exception):
    """Another request already owns the sensor."""


class DistanceUnavailable(Exception):
    """The sensor or a usable distance measurement is unavailable."""

    def __init__(self, message, *, result=None):
        super().__init__(message)
        self.result = result


DEFAULTS = dict(default_samples=5, max_samples=20, sample_interval_ms=200,
                echo_timeout_ms=100, trigger_pulse_us=100, sound_speed_m_s=343,
                min_mm=20, max_mm=4000, gpiochip=0, trig_gpio=23, echo_gpio=24,
                aggregation='median')
BATCH_SECONDS = 5


@dataclass(frozen=True)
class DistanceReading:
    captured_at: str
    pulse_us: float | None
    error: str | None = None


class DistanceSource(Protocol):
    """One owner; read observes cancellation/deadline and spaces trigger starts."""

    def start(self) -> None: ...
    def read(self, *, cancelled: Event, deadline: float) -> DistanceReading: ...
    def close(self) -> None: ...

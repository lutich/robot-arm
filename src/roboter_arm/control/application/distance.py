"""On-demand measurement admission and aggregation, independent of servo control."""
import math
import statistics
import threading
import time

from roboter_arm.control.domain.distance import (
    BATCH_SECONDS, DEFAULTS, DistanceConflict, DistanceSource, DistanceUnavailable)


class DistanceService:
    def __init__(self, source: DistanceSource | None = None, *, clock=time.monotonic):
        self.source, self.clock = source, clock
        self._state = threading.RLock()
        self._acquisition = threading.Lock()
        self._cancelled = threading.Event()
        self._started = self._source_closed = False
        self._ready = self._busy = False
        self._last_error = None

    def start(self):
        with self._acquisition:
            with self._state:
                if self._started or self._cancelled.is_set():
                    return
                self._started = True
            if self.source is None:
                return
            try:
                self.source.start()
            except Exception as error:
                with self._state:
                    self._last_error = f'{type(error).__name__}: {error}'
                try:
                    self._close_source()
                except Exception as cleanup_error:
                    with self._state:
                        self._last_error += f'; cleanup: {type(cleanup_error).__name__}: {cleanup_error}'
            else:
                with self._state:
                    self._ready = not self._cancelled.is_set()

    def status(self):
        with self._state:
            return dict(enabled=self.source is not None, ready=self._ready,
                        busy=self._busy, last_error=self._last_error, defaults=dict(DEFAULTS))

    def read(self, samples=5):
        if type(samples) is not int or not 1 <= samples <= DEFAULTS['max_samples']:
            raise ValueError('samples must be an integer from 1 to 20')
        if not self._acquisition.acquire(blocking=False):
            raise DistanceConflict('Distance sensor already has an active request')
        try:
            with self._state:
                if not self._ready or self._cancelled.is_set():
                    raise DistanceUnavailable(self._last_error or 'Distance sensor is disabled or unavailable')
                self._busy = True
            started = self.clock()
            deadline = started + BATCH_SECONDS
            readings = []
            for _ in range(samples):
                if self._cancelled.is_set() or self.clock() >= deadline:
                    break
                sample = self.source.read(cancelled=self._cancelled, deadline=deadline)
                pulse, error = sample.pulse_us, sample.error
                distance = None
                if error is None:
                    if pulse is None or not math.isfinite(pulse) or pulse <= 0:
                        error = 'Malformed echo pulse'
                        if pulse is not None and not math.isfinite(pulse):
                            pulse = None
                    else:
                        distance = pulse * DEFAULTS['sound_speed_m_s'] / 2000
                        if not DEFAULTS['min_mm'] <= distance <= DEFAULTS['max_mm']:
                            error, distance = 'Distance outside nominal 20–4000 mm range', None
                readings.append(dict(captured_at=sample.captured_at, pulse_us=pulse,
                                     distance_mm=distance, error=error))
            values = [reading['distance_mm'] for reading in readings if reading['error'] is None]
            result = dict(distance_mm=statistics.median(values) if values else None,
                mean_mm=statistics.mean(values) if values else None,
                min_mm=min(values) if values else None, max_mm=max(values) if values else None,
                stdev_mm=statistics.pstdev(values) if values else None,
                attempts=len(readings), valid=len(values), readings=readings,
                sound_speed_m_s=DEFAULTS['sound_speed_m_s'],
                elapsed_ms=max(0, self.clock()-started)*1000)
            if self._cancelled.is_set():
                raise DistanceUnavailable('Distance sensor is closing', result=result)
            with self._state:
                self._last_error = next((reading['error'] for reading in reversed(readings)
                                         if reading['error']), None)
                if not values and self._last_error is None:
                    self._last_error = 'Batch deadline exceeded'
            if not values:
                raise DistanceUnavailable(self._last_error or 'Batch deadline exceeded', result=result)
            return result
        except DistanceUnavailable:
            raise
        except Exception as error:
            with self._state:
                self._ready = False
                self._last_error = f'{type(error).__name__}: {error}'
            raise
        finally:
            with self._state:
                self._busy = False
            self._acquisition.release()

    def begin_close(self):
        self._cancelled.set()
        with self._state:
            self._ready = False

    def _close_source(self):
        if self.source is not None and not self._source_closed:
            self.source.close()
            self._source_closed = True

    def close(self):
        self.begin_close()
        with self._acquisition:
            self._close_source()

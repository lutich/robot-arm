"""HC-SR04 on GPIO23/24; construction and import never access GPIO."""
from datetime import datetime, timezone
import threading
import time

from roboter_arm.control.domain.distance import DEFAULTS, DistanceReading


class HCSR04:
    def __init__(self, *, clock=time.monotonic, clock_ns=time.monotonic_ns):
        self.clock, self.clock_ns = clock, clock_ns
        self._gpio = self._handle = self._callback = None
        self._claimed = []
        self._lock = threading.Lock()
        self._acquisition = threading.Lock()
        self._completed = threading.Event()
        self._stopping = threading.Event()
        self._active = False
        self._start_ns = self._end_ns = self._rise_ns = self._last_tick = None
        self._pulse_us = self._error = None
        self._last_trigger = None

    def start(self):
        with self._acquisition:
            if self._stopping.is_set():
                raise RuntimeError('Distance sensor is closed; restart required')
            if self._handle is not None:
                return
            import lgpio
            self._gpio = lgpio
            try:
                self._handle = lgpio.gpiochip_open(DEFAULTS['gpiochip'])
                lgpio.gpio_claim_output(self._handle, DEFAULTS['trig_gpio'], 0)
                self._claimed.append(DEFAULTS['trig_gpio'])
                lgpio.gpio_claim_alert(self._handle, DEFAULTS['echo_gpio'], lgpio.BOTH_EDGES)
                self._claimed.append(DEFAULTS['echo_gpio'])
                self._callback = lgpio.callback(self._handle, DEFAULTS['echo_gpio'],
                                                lgpio.BOTH_EDGES, self._edge)
            except BaseException:
                self._stopping.set()
                self._cleanup()
                raise

    def _edge(self, chip, gpio, level, tick):
        # lgpio event ticks and monotonic_ns use Linux CLOCK_MONOTONIC nanoseconds.
        with self._lock:
            if (not self._active or self._stopping.is_set()
                    or chip != DEFAULTS['gpiochip'] or gpio != DEFAULTS['echo_gpio']
                    or tick < self._start_ns or tick > self._end_ns):
                return
            if ((self._last_tick is not None and tick <= self._last_tick)
                    or level not in (0, 1)
                    or (level == 1 and self._rise_ns is not None)
                    or (level == 0 and self._rise_ns is None)):
                self._error = 'Malformed echo edges'
            elif level == 1:
                self._rise_ns, self._last_tick = tick, tick
                return
            else:
                self._pulse_us = (tick-self._rise_ns)/1000
            self._active = False
            self._completed.set()

    def read(self, *, cancelled, deadline):
        with self._acquisition:
            captured_at = datetime.now(timezone.utc).isoformat()
            if self._handle is None or self._stopping.is_set():
                raise RuntimeError('Distance sensor is not started')
            if self._last_trigger is not None:
                remaining = self._last_trigger + DEFAULTS['sample_interval_ms']/1000 - self.clock()
                while remaining > 0 and self.clock() < deadline:
                    if cancelled.is_set() or self._stopping.is_set():
                        break
                    cancelled.wait(min(.01, remaining, max(0, deadline-self.clock())))
                    remaining = self._last_trigger + DEFAULTS['sample_interval_ms']/1000 - self.clock()
            if cancelled.is_set() or self._stopping.is_set():
                return DistanceReading(captured_at, None, 'Measurement cancelled')
            if self.clock() >= deadline:
                return DistanceReading(captured_at, None, 'Batch deadline exceeded')
            if self._gpio.gpio_read(self._handle, DEFAULTS['echo_gpio']):
                return DistanceReading(captured_at, None, 'ECHO already HIGH')
            self._completed.clear()
            captured_at = datetime.now(timezone.utc).isoformat()
            with self._lock:
                self._last_trigger = self.clock()
                start_ns = self.clock_ns()
                echo_deadline = min(deadline, self._last_trigger+DEFAULTS['echo_timeout_ms']/1000)
                self._active = True
                self._start_ns, self._end_ns = start_ns, int(echo_deadline*1_000_000_000)
                self._rise_ns = self._last_tick = self._pulse_us = self._error = None
            try:
                self._gpio.gpio_write(self._handle, DEFAULTS['trig_gpio'], 1)
                time.sleep(DEFAULTS['trigger_pulse_us']/1_000_000)
                self._gpio.gpio_write(self._handle, DEFAULTS['trig_gpio'], 0)
                while not self._completed.is_set():
                    remaining = echo_deadline-self.clock()
                    if cancelled.is_set() or self._stopping.is_set() or remaining <= 0:
                        break
                    self._completed.wait(min(.01, remaining))
                with self._lock:
                    if cancelled.is_set() or self._stopping.is_set():
                        error = 'Measurement cancelled'
                    elif not self._completed.is_set():
                        error = 'No complete echo within 100 ms' if echo_deadline < deadline else 'Batch deadline exceeded'
                    else:
                        error = self._error
                    return DistanceReading(captured_at, self._pulse_us, error)
            finally:
                with self._lock:
                    self._active = False
                self._gpio.gpio_write(self._handle, DEFAULTS['trig_gpio'], 0)

    def _cleanup(self):
        with self._lock:
            self._active = False
        callback, self._callback = self._callback, None
        failures = []
        if callback is not None:
            try:
                callback.cancel()
            except BaseException as error:
                failures.append(error)
        if self._handle is not None:
            if DEFAULTS['trig_gpio'] in self._claimed:
                try:
                    self._gpio.gpio_write(self._handle, DEFAULTS['trig_gpio'], 0)
                except BaseException as error:
                    failures.append(error)
            for gpio in reversed(self._claimed):
                try:
                    self._gpio.gpio_free(self._handle, gpio)
                except BaseException as error:
                    failures.append(error)
            self._claimed.clear()
            try:
                self._gpio.gpiochip_close(self._handle)
            except BaseException as error:
                failures.append(error)
            finally:
                self._handle = None
        if failures:
            raise RuntimeError('Distance sensor GPIO cleanup failed') from failures[0]

    def close(self):
        self._stopping.set()
        self._completed.set()
        with self._acquisition:
            self._cleanup()

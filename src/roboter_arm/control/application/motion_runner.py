"""Single-owner trajectory execution through a driver; failure directly disables."""
import threading
import time

from roboter_arm.control.domain.motion import Trajectory, finite


class MotionRunner:
    """One controller owner; completion retains output, failure directly disables.

    Driver protocol: write(channel, count), disable(channel), scope string.
    Hardware use awaits calibrated startup/stop procedures and an explicit adapter.
    Quantized PWM steps may exceed the continuous curve's instantaneous speed;
    the curve bounds counts/second before rounding, not mechanical velocity.
    """
    _creation_lock = threading.Lock()

    def __init__(self, driver):
        self.driver = driver
        self.last_command = {}  # Commands only; actual pose remains unknown.
        with self._creation_lock:
            if not hasattr(driver, '_motion_owner'):
                driver._motion_owner = threading.Lock()
            self._owner = driver._motion_owner

    def execute(self, start, target, limits, *, timeout, cancelled=lambda: False,
                clock=time.monotonic, sleep=time.sleep, interval=0.02):
        if not self._owner.acquire(blocking=False):
            raise RuntimeError('Controller already has a motion owner')
        begun = False
        try:
            trajectory = Trajectory(start, target, limits)
            trajectory.validate_timing(timeout, interval)
            if trajectory.scopes != {self.driver.scope}:
                raise ValueError('Driver and calibration scopes must match')
            origin = clock()
            previous = 0.0
            def checkpoint():
                nonlocal previous
                elapsed = clock() - origin
                if not finite(elapsed) or elapsed < previous:
                    raise RuntimeError('Clock must be finite and monotonic')
                previous = elapsed
                if cancelled():
                    raise InterruptedError('Motion cancelled')
                if elapsed > timeout:
                    raise TimeoutError('Motion deadline exceeded')
                return elapsed

            while True:
                elapsed = checkpoint()
                # Avoid a zero-progress sleep when the remaining float interval
                # is smaller than the clock's resolution near the endpoint.
                if trajectory.duration - elapsed <= 1e-9:
                    elapsed = trajectory.duration
                values = trajectory.sample(elapsed)
                for channel, count in values.items():
                    checkpoint()
                    begun = True  # Even a failed write may have reached the hardware.
                    self.driver.write(channel, count)
                    self.last_command[channel] = count
                    checkpoint()
                if elapsed >= trajectory.duration:
                    return dict(self.last_command)
                sleep(min(interval, trajectory.duration - elapsed, timeout - elapsed))
        except BaseException as error:
            if begun:
                failures = []
                for channel in start:
                    try:
                        self.driver.disable(channel)
                    except BaseException as cleanup_error:
                        failures.append(f'{channel}: {type(cleanup_error).__name__}')
                if failures:
                    error.add_note('Best-effort disable failed for channels ' + ', '.join(failures))
            raise
        finally:
            self._owner.release()

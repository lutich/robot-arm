"""Explicit startup/stop gates; hardware-independent until measured poses exist."""
from functools import wraps
import threading

from roboter_arm.control.application.motion_runner import MotionRunner
from roboter_arm.control.domain.motion import Trajectory


def exclusive(method):
    @wraps(method)
    def guarded(self, *args, **kwargs):
        if not self._request.acquire(blocking=False):
            raise RuntimeError('Controller already has an active request')
        try:
            return method(self, *args, **kwargs)
        finally:
            self._request.release()
    return guarded


class Control:
    """Commands only, no position feedback. Construction performs no driver call.

    One instance is the lifecycle owner for a driver. Normal PARK holds; supported
    disable is a separate operation. Exceptions invalidate the starting condition.
    No real adapter/CLI or saved startup pose is supplied by this module.
    """
    def __init__(self, driver, limits, poses):
        self.runner = MotionRunner(driver)
        self.limits = dict(limits)
        low = {c:lim.low for c, lim in self.limits.items()}
        trajectory = Trajectory(low, low, self.limits)
        if trajectory.scopes != {driver.scope} or driver.scope not in ('synthetic', 'validated'):
            raise ValueError('Driver and calibration scopes must match')
        self.poses = {}
        for name, pose in poses.items():
            if name not in ('PARK', 'HOME') or pose.get('observed') is not True or not pose.get('provenance'):
                raise ValueError('PARK/HOME need explicit observed clearance/support provenance')
            counts = dict(pose['counts'])
            if counts.keys() != self.limits.keys():
                raise ValueError('Pose must cover every configured channel')
            Trajectory(counts, counts, self.limits)
            self.poses[name] = counts
        self.enabled = False
        self._command = None
        self._stop = threading.Event()
        self._request = threading.Lock()
        with MotionRunner._creation_lock:
            if hasattr(driver, '_control_owner'):
                raise RuntimeError('Driver already has a lifecycle owner')
            driver._control_owner = self

    @exclusive
    def enable(self, start, *, supported, established, stop_disable_agreed, **timing):
        if self.enabled:
            raise RuntimeError('Controller already enabled')
        if not all(flag is True for flag in (supported, established, stop_disable_agreed)):
            raise ValueError('Supported physical start and emergency disable agreement are required')
        if start.keys() != self.limits.keys():
            raise ValueError('Physical start must cover every configured channel')
        self._stop.clear()
        try:
            self.runner.execute(start, start, self.limits, **timing)
        except BaseException:
            self._command = None
            raise
        self._command = dict(start)
        self.enabled = True

    @exclusive
    def move(self, target, **timing):
        if not self.enabled or self._command is None:
            raise RuntimeError('Starting condition unknown; establish supported startup first')
        # Validate before beginning; invalid requests leave the current hold intact.
        trajectory = Trajectory(self._command, target, self.limits)
        trajectory.validate_timing(timing.get('timeout'), timing.get('interval', .02))
        cancelled = timing.pop('cancelled', lambda: False)
        try:
            self.runner.execute(self._command, target, self.limits,
                                cancelled=lambda: self._stop.is_set() or cancelled(), **timing)
        except BaseException as error:
            # Startup includes explicit emergency-disable agreement. Even a
            # pre-cancelled request must stop outputs held by a previous move.
            try:
                self.disable(supported=True)
            except BaseException as cleanup_error:
                error.add_note(f'Controller disable failed: {type(cleanup_error).__name__}')
            raise
        self._command = dict(target)

    def park(self, **timing):
        if 'PARK' not in self.poses:
            raise ValueError('Observed PARK is not configured')
        self.move(self.poses['PARK'], **timing)

    def home(self, **timing):
        if 'HOME' not in self.poses:
            raise ValueError('Observed HOME is not configured')
        self.move(self.poses['HOME'], **timing)

    def disable(self, *, supported):
        if supported is not True:
            raise ValueError('Support is required before removing holding torque')
        self._stop.set()
        failures = []
        try:
            for channel in self.limits:
                try:
                    self.runner.driver.disable(channel)
                except BaseException as error:
                    failures.append(error)
        finally:
            self.enabled = False
            self._command = None
        if failures:
            raise RuntimeError('Best-effort disable failed; use servo power switch') from failures[0]

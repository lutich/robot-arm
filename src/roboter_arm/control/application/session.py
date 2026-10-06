"""Authoritative attended control actions, independent of HTTP and frontend."""
import threading
import time

from roboter_arm.control.application.motion_runner import MotionRunner
from roboter_arm.control.domain.budget import Budget
from roboter_arm.control.domain.demo import Demo
from roboter_arm.control.domain.motion import JointLimit, Trajectory
from roboter_arm.shared.joints import JOINTS

STARTUP = (330, 540, 580, 195, 480, 650)
OLD_HOME = (90, 440, 320, 250, 480, 600)
# Observed full PARK: config/reference-arm/park.json.
# These are commissioning commands, never measured or assumed startup positions.
PARK = {0:330, 1:540, 2:580, 3:195, 4:480, 5:650}
# User-observed HOME: config/reference-arm/home.json.
HOME = {0:320, 1:427, 2:403, 3:220, 4:480, 5:650}
INACTIVITY_WARNING_SECONDS = 600
SPEED = 20  # Command counts/second; not measured physical velocity.
DEMO_SPEED_MAX = 100  # Demo playback cap; manual moves and PARK/HOME keep SPEED.


class Session:
    """Attended, single-owner commissioning; optional finite test caps."""
    def __init__(self, driver, channels, *, pose_store, demo_store,
                 clock=time.monotonic, sleep=time.sleep, ranges=None, seconds=None, max_moves=None):
        if not channels or len(set(channels)) != len(channels) or any(type(c) is not int or c not in JOINTS for c in channels):
            raise ValueError('Select explicit channels 0–5 without duplicates')
        if driver.scope not in ('synthetic', 'commissioning'):
            raise ValueError('Manual app requires preview or commissioning driver scope')
        if pose_store.scope != driver.scope or demo_store.scope != driver.scope:
            raise ValueError('Pose and demo records must use the driver scope')
        self.budget = Budget(seconds, max_moves)
        self.ranges = {c:(JOINTS[c][1], JOINTS[c][2]) for c in channels}
        for channel, bounds in (ranges or {}).items():
            if type(channel) is not int or channel not in channels or len(bounds) != 2:
                raise ValueError('Range must select a permitted channel')
            low, high = bounds
            if type(low) is not int or type(high) is not int or not JOINTS[channel][1] <= low <= high <= JOINTS[channel][2]:
                raise ValueError('Session range must stay inside historical reference bounds')
            self.ranges[channel] = (low, high)
        self.driver, self.channels = driver, tuple(channels)
        self.pose_store, self.demo_store = pose_store, demo_store
        self.clock, self.sleep = clock, sleep
        self.lock = threading.RLock()
        self.io = threading.RLock()
        self.cancel = threading.Event()
        self.stop_lock = threading.Lock()
        self.stop_generation = 0
        self.prepared = self.armed = self.busy = False
        self.camera_pending = False
        self.phase = None
        self.outputs_off = None
        self.commands = {c:None for c in JOINTS}
        self.last_command = {}
        self.error = None
        self.last_activity = 0
        self.worker = None
        self.runner = MotionRunner(self)
        self.scope = driver.scope
        self.poses = self._load_poses()
        self.demo = Demo()
        self.step_size, self.demo_speed = 10, SPEED

    def _load_poses(self):
        poses = {'HOME':dict(HOME), 'PARK':dict(PARK)}
        for name in poses:
            counts = self.pose_store.load(name)
            if counts is not None:
                poses[name] = counts
        return poses

    def state(self):
        with self.lock:
            return dict(mode='preview' if self.scope == 'synthetic' else 'hardware',
                prepared=self.prepared, armed=self.armed, busy=self.busy, phase=self.phase,
                outputs_off=self.outputs_off, error=self.error, commands=dict(self.commands),
                last_command=dict(self.last_command), moves=self.budget.moves,
                remaining_moves=self.budget.remaining_moves(),
                session_seconds=self.budget.seconds, max_moves=self.budget.max_moves,
                remaining_seconds=self.budget.remaining_seconds(self.clock()) if self.armed else None,
                inactivity_warning=self.armed and self.clock()-self.last_activity >= INACTIVITY_WARNING_SECONDS,
                inactivity_warning_seconds=INACTIVITY_WARNING_SECONDS,
                joints=[dict(channel=c, name=name, low=self.ranges.get(c,(low,high))[0],
                    high=self.ranges.get(c,(low,high))[1], historical_low=low, historical_high=high, startup=STARTUP[c],
                    old_home=OLD_HOME[c], park=self.poses['PARK'][c], home=self.poses['HOME'][c], allowed=c in self.channels) for c, (name, low, high) in JOINTS.items()],
                speed=SPEED, demo=self._demo_state(),
                settings=dict(step_size=self.step_size, demo_speed=self.demo_speed, demo_speed_max=DEMO_SPEED_MAX))

    def _demo_state(self):
        positions = []
        for position in self.demo.positions:
            valid, error = self._position_valid(position['counts'])
            positions.append(dict(name=position['name'], counts=dict(position['counts']), valid=valid, error=error))
        return dict(name=self.demo.title, positions=positions, cursor=self.demo.cursor,
            status=self.demo.status, mode=self.demo.mode, pause_requested=self.demo.pause_requested, saved=self.demo_store.saved())

    def _position_valid(self, counts):
        if set(self.channels) != set(JOINTS):
            return False, 'Select all six joints to execute recorded positions'
        if (not isinstance(counts, dict) or set(counts) != set(JOINTS)
                or any(type(v) is not int for v in counts.values())):
            return False, 'A recorded position needs all six integer commands'
        for channel, count in counts.items():
            if not self.ranges[channel][0] <= count <= self.ranges[channel][1]:
                return False, f'{JOINTS[channel][0]} is outside the applied limits'
        return True, None

    def _idle(self):
        if self.camera_pending:
            raise ValueError('Wait for the pending camera change before changing controls')
        if self.busy:
            raise ValueError('Wait for the current movement before changing controls')

    def _full_commands(self):
        self._check_session()
        self._idle()
        valid, error = self._position_valid(self.commands)
        if not valid:
            raise ValueError(error)
        return dict(self.commands)

    def _limits(self, speed=SPEED):
        return {c:JointLimit(*self.ranges[c], speed, 'Historical commissioning guards', self.scope) for c in JOINTS}

    def _check_batch(self, start, targets, *, speed=SPEED):
        limits = self._limits(speed)
        duration = 0
        for target in targets:
            valid, error = self._position_valid(target)
            if not valid:
                raise ValueError(error)
            duration += Trajectory(start, target, limits).duration
            start = target
        if not self.budget.allows(len(targets)):
            raise ValueError('Not enough bounded-test moves for this request')
        if self.budget.reached(self.clock() + duration):
            raise ValueError('Not enough bounded-test time for this request')
        return limits

    def set_settings(self, *, step_size, demo_speed):
        with self.lock:
            self._idle()
            if type(step_size) is not int or step_size < 1:
                raise ValueError('Step size must be a positive integer count')
            if type(demo_speed) is not int or not 1 <= demo_speed <= DEMO_SPEED_MAX:
                raise ValueError(f'Demo speed must be integer counts/s from 1 to {DEMO_SPEED_MAX}')
            self.step_size, self.demo_speed = step_size, demo_speed

    def jog(self, *, channel, direction):
        with self.lock:
            self._check_session()
            self._idle()
            if type(channel) is not int or channel not in self.channels:
                raise ValueError('Channel is outside this attended session')
            if type(direction) is not int or direction not in (-1, 1):
                raise ValueError('Jog direction must be +1 or -1')
            start = self.commands[channel]
            if start is None:
                raise ValueError('Initialize the joint before using its arrows')
            self.move(channel=channel, target=start + direction*self.step_size, first_clear=False)

    def set_limits(self, *, channel, low, high):
        with self.lock:
            self._idle()
            if type(channel) is not int or channel not in self.channels:
                raise ValueError('Channel is outside this attended session')
            if type(low) is not int or type(high) is not int or not JOINTS[channel][1] <= low <= high <= JOINTS[channel][2]:
                raise ValueError('Applied limits must stay inside the historical outer bounds')
            required = [self.poses['PARK'][channel], self.poses['HOME'][channel]]
            if self.commands[channel] is not None:
                required.append(self.commands[channel])
            if any(not low <= value <= high for value in required):
                raise ValueError('Applied limits must retain PARK, HOME and the held command')
            self.ranges[channel] = (low, high)
            self.demo.reset()
            self.last_activity = self.clock()

    def go_pose(self, *, name):
        with self.lock:
            start = self._full_commands()
            if name not in ('HOME', 'PARK'):
                raise ValueError('Select HOME or PARK')
            target = dict(self.poses[name])
            limits = self._check_batch(start, [target])
            self.demo.reset()
            self.budget.accept()
            self.last_activity = self.clock()
            self.busy, self.phase, self.error = True, f'Going to {name}', None
            self.worker = threading.Thread(target=self._go_pose, args=(start,target,limits), daemon=True)
            self.worker.start()

    def _go_pose(self, start, target, limits):
        try:
            self.runner.execute(start, target, limits, timeout=Trajectory(start,target,limits).duration + 5,
                cancelled=self._cancelled, clock=self.clock, sleep=self.sleep)
        except BaseException as error:
            self.stop(f'Pose move stopped: {type(error).__name__}; use power switch for unexpected behavior')
        finally:
            with self.lock:
                self.busy = False
                self.phase = None

    def _cancelled(self, generation=None):
        return (not self.armed or self.cancel.is_set()
                or generation is not None and generation != self.stop_generation
                or self.budget.reached(self.clock()))

    def _admit_start(self, generation):
        if generation != self.stop_generation:
            raise InterruptedError('Stopped while starting the session')
        self.cancel.clear()
        if generation != self.stop_generation:
            self.cancel.set()
            raise InterruptedError('Stopped while starting the session')

    def demo_name(self, *, name):
        with self.lock:
            self._idle()
            self.demo.rename(name)

    def demo_add(self, *, name=None):
        with self.lock:
            counts = self._full_commands()
            self.demo.add(counts, name)
            self.last_activity = self.clock()

    def demo_replace(self, *, index):
        with self.lock:
            self._idle()
            self.demo.position(index)
            self.demo.replace(index, self._full_commands())
            self.last_activity = self.clock()

    def demo_rename(self, *, index, name):
        with self.lock:
            self._idle()
            self.demo.rename_position(index, name)

    def demo_reorder(self, *, index, direction):
        with self.lock:
            self._idle()
            self.demo.reorder(index, direction)

    def demo_remove(self, *, index):
        with self.lock:
            self._idle()
            self.demo.remove(index)

    def _start_demo(self, *, start, all_positions):
        current = self._full_commands()
        targets = self.demo.targets(start, all_positions, self._position_valid)
        limits = self._check_batch(current, targets, speed=self.demo_speed)
        self.demo.begin(start, all_positions)
        self.busy, self.phase, self.error = True, f'Demo position {start+1} of {len(self.demo.positions)}', None
        self.last_activity = self.clock()
        self.worker = threading.Thread(target=self._run_demo, args=(current,targets,limits,start,all_positions), daemon=True)
        self.worker.start()

    def demo_next(self):
        with self.lock:
            if self.demo.status == 'paused':
                raise ValueError('Resume or restart the paused demo')
            self._start_demo(start=self.demo.cursor, all_positions=False)

    def demo_run(self):
        with self.lock:
            self._start_demo(start=0, all_positions=True)

    def demo_resume(self):
        with self.lock:
            if self.demo.status != 'paused':
                raise ValueError('There is no paused demo to resume')
            self._start_demo(start=self.demo.cursor, all_positions=True)

    def demo_pause(self):
        with self.lock:
            self._check_session()
            self.demo.request_pause(self.busy)

    def demo_restart(self):
        with self.lock:
            self._idle()
            self.demo.reset()

    def _run_demo(self, current, targets, limits, first, all_positions):
        try:
            for offset, target in enumerate(targets):
                with self.lock:
                    self._check_session()
                    self.phase = f'Demo position {first+offset+1} of {len(self.demo.positions)}'
                    self.budget.accept()
                    self.last_activity = self.clock()
                self.runner.execute(current, target, limits,
                    timeout=Trajectory(current,target,limits).duration + 5,
                    cancelled=self._cancelled, clock=self.clock, sleep=self.sleep)
                current = target
                with self.lock:
                    self._check_session()
                    if self.demo.reached(first+offset, all_positions):
                        break
        except BaseException as error:
            self.stop(f'Demo stopped: {type(error).__name__}; use power switch for unexpected behavior')
        finally:
            with self.lock:
                self.busy = False
                self.phase = None
                self.demo.finish()

    def demo_save(self):
        with self.lock:
            self._idle()
            filename = self.demo_store.save(self.demo.title, self.demo.positions)
            self.last_activity = self.clock()
            return filename

    def demo_load(self, *, filename):
        with self.lock:
            self._idle()
            record = self.demo_store.load(filename)
            self.demo.load(record['name'], record['positions'])

    def power_on(self, *, park_confirmed):
        """Explicit supported, powered PARK acknowledgment; no supply switching."""
        generation = self.stop_generation
        with self.lock:
            if park_confirmed is not True:
                raise ValueError('Confirm the arm is physically in PARK, supported and powered-ready')
            self._idle()
            if self.busy or self.armed:
                raise ValueError('Power OFF before starting another session')
            park, home = dict(self.poses['PARK']), dict(self.poses['HOME'])
            if set(self.channels) != set(JOINTS) or any(
                    not self.ranges[c][0] <= v <= self.ranges[c][1]
                    for pose in (park, home) for c,v in pose.items()):
                raise ValueError('Power ON requires all six PARK and HOME targets inside session ranges')
            if self.prepared and self.outputs_off is not True:
                raise ValueError('Outputs OFF must be verified; switch servo power OFF')
            self._admit_start(generation)
            try:
                with self.io:
                    if not self.prepared:
                        self.driver.prepare()
                        self.prepared = True
                    self.driver.verify_off()
                    self.outputs_off = True
                if self.cancel.is_set():
                    raise InterruptedError('Stopped during PARK preparation')
                self.armed = True
                self.demo.reset()
                self.budget.start(self.clock())
                self.last_activity = self.clock()
                self.error = None
            except BaseException:
                self.stop('Power ON failed; switch servo power OFF')
                raise
            limits = {c:JointLimit(*self.ranges[c], SPEED, 'Observed PARK; commissioning only', self.scope) for c in park}
            self.busy, self.phase = True, 'Going to HOME'
            self.worker = threading.Thread(target=self._power_on, args=(park,home,limits,generation), daemon=True)
            self.worker.start()

    def _power_on(self, park, home, limits, generation):
        try:
            self.runner.execute(park, park, limits, timeout=min(10, self.budget.seconds) if self.budget.seconds is not None else 10,
                cancelled=lambda:self._cancelled(generation),
                clock=self.clock, sleep=self.sleep)
            trajectory = Trajectory(park, home, limits)
            self.runner.execute(park, home, limits, timeout=trajectory.duration + 5,
                cancelled=lambda:self._cancelled(generation),
                clock=self.clock, sleep=self.sleep)
        except BaseException as error:
            self.stop(f'HOME transition stopped: {type(error).__name__}; use power switch for unexpected behavior')
        finally:
            with self.lock:
                self.busy = False
                self.phase = None

    def power_off(self):
        """Return held commands to PARK, then directly disable PWM."""
        with self.lock:
            self._check_session()
            self._idle()
            park = dict(self.poses['PARK'])
            if self.busy or any(self.commands[c] is None for c in park):
                raise ValueError('Wait for movement and known joint commands before PARK')
            limits = {c:JointLimit(*self.ranges[c], SPEED, 'Observed PARK/HOME; commissioning only', self.scope) for c in park}
            start = {c:self.commands[c] for c in park}
            trajectory = Trajectory(start, park, limits)
            if self.budget.passed(self.clock() + trajectory.duration):
                raise ValueError('Not enough bounded-test time to return to PARK')
            self.demo.reset()
            self.busy, self.phase = True, 'Returning to PARK'
            self.worker = threading.Thread(target=self._power_off, args=(start, park, limits, trajectory.duration+5), daemon=True)
            self.worker.start()

    def _power_off(self, start, park, limits, timeout):
        try:
            self.runner.execute(start, park, limits, timeout=timeout,
                cancelled=lambda:self.cancel.is_set() or self.budget.reached(self.clock()),
                clock=self.clock, sleep=self.sleep)
            self.stop()
        except BaseException as error:
            self.stop(f'PARK return stopped: {type(error).__name__}; use power switch for unexpected behavior')
        finally:
            with self.lock:
                self.busy = False
                self.phase = None

    def prepare(self, *, supported, power_off):
        with self.lock:
            if supported is not True or power_off is not True:
                raise ValueError('Current support and servo power OFF are required')
            if self.busy or self.armed or self.prepared:
                raise ValueError('Stop and close the previous session before preparation')
            with self.io:
                self.driver.prepare()
                self.driver.verify_off()
            self.prepared, self.outputs_off, self.error = True, True, None

    def arm(self, *, ready, supported, switch_ready):
        generation = self.stop_generation
        with self.lock:
            self._idle()
            if not self.prepared or self.outputs_off is not True or self.armed or self.busy:
                raise ValueError('Prepare outputs with power OFF before arming')
            if not all(flag is True for flag in (ready, supported, switch_ready)):
                raise ValueError('Current attended powered readiness, support and power switch access required')
            self._admit_start(generation)
            self.armed = True
            self.demo.reset()
            self.budget.start(self.clock())
            self.last_activity = self.clock()

    def write(self, channel, count):
        with self.io:
            if self.cancel.is_set():
                raise InterruptedError('Stopped')
            self.driver.write(channel, count)
            self.commands[channel] = count
            self.last_command[channel] = count
            self.outputs_off = False

    def disable(self, channel):
        with self.io:
            self.driver.disable(channel)

    def move(self, *, channel, target, first_clear):
        with self.lock:
            self._check_session()
            if self.camera_pending:
                raise ValueError('Wait for the pending camera change before moving')
            if self.busy:
                raise ValueError('A move is already running')
            if not self.budget.allows(1):
                raise ValueError('Move budget finished; save the pose or stop')
            if type(channel) is not int or channel not in self.channels:
                raise ValueError('Channel is outside this attended session')
            low, high = self.ranges[channel]
            if type(target) is not int or not low <= target <= high:
                raise ValueError('Target must be integer counts inside the historical reference range')
            start = self.commands[channel]
            if start is None and first_clear is not True:
                raise ValueError('Unknown first position needs explicit support/clearance confirmation')
            limit = JointLimit(low, high, SPEED, 'Historical reference; commissioning only', self.scope)
            trajectory = Trajectory({channel:target if start is None else start}, {channel:target}, {channel:limit})
            if self.budget.passed(self.clock() + trajectory.duration):
                raise ValueError('Not enough session time for this move')
            self.demo.reset()
            self.budget.accept(manual=True)
            self.last_activity = self.clock()
            self.busy, self.phase, self.error = True, 'Moving selected joint', None
            self.worker = threading.Thread(target=self._move, args=(channel, trajectory.start[channel], target, limit), daemon=True)
            self.worker.start()

    def _move(self, channel, start, target, limit):
        try:
            self.runner.execute({channel:start}, {channel:target}, {channel:limit}, timeout=60,
                cancelled=self.cancel.is_set, clock=self.clock, sleep=self.sleep)
        except BaseException as error:
            self.stop(f'Move stopped: {type(error).__name__}; use power switch for unexpected behavior')
        finally:
            with self.lock:
                self.busy = False
                self.phase = None

    def _check_session(self):
        if not self.armed or self.cancel.is_set():
            raise ValueError('Session is not armed')
        if self.budget.reached(self.clock()):
            raise ValueError('Finite session finished; stop and obtain fresh readiness')

    def stop(self, reason=None):
        return self._stop(reason)

    def _stop(self, reason=None, *, expected_generation=None):
        # No driver/session lock is held here: admission cannot erase this stop.
        with self.stop_lock:
            if expected_generation is not None and expected_generation != self.stop_generation:
                return None
            self.stop_generation += 1
            self.cancel.set()
        failures = []
        with self.io:
            if self.prepared:
                for channel in range(16):
                    try:
                        self.driver.disable(channel)
                    except BaseException as error:
                        failures.append(type(error).__name__)
                try:
                    self.driver.verify_off()
                except BaseException as error:
                    failures.append(type(error).__name__)
        with self.lock:
            self.armed = False
            self.phase = None
            self.commands = {c:None for c in JOINTS}
            self.outputs_off = not failures if self.prepared else None
            self.error = 'Disable/readback failed; switch servo power OFF' if failures else reason
            self.demo.reset()
        return not failures

    def watchdog(self):
        with self.lock:
            generation = self.stop_generation
            reason = None
            if self.armed and self.budget.reached(self.clock()):
                reason = 'Bounded test session expired; PWM OFF; starting position unknown'
        if reason:
            self._stop(reason, expected_generation=generation)

    def save_pose(self, *, name, observed, provenance):
        with self.lock:
            self._check_session()
            if self.busy or any(count is None for count in self.commands.values()):
                raise ValueError('Position all six joints and wait for motion before saving a full pose')
            if name not in ('PARK', 'HOME') or observed is not True or not isinstance(provenance, str) or not provenance.strip():
                raise ValueError('Pose needs name, observed clearance/support and provenance')
            counts = self._full_commands()
            filename = self.pose_store.save(name, counts, provenance=provenance)
            self.poses[name] = counts
            self.last_activity = self.clock()
            return filename

    def close(self):
        stopped = self.stop()
        if self.worker is not None:
            self.worker.join(timeout=2)
        with self.io:
            self.driver.close()
        return stopped

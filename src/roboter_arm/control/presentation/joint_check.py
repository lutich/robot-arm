"""Explicit single-joint commissioning check; preview is hardware-free."""
import argparse
from contextlib import contextmanager
import math
import signal
import time

from roboter_arm.control.infrastructure.pca9685 import controller as gripper_controller, verify_off
from roboter_arm.control.presentation.gripper_check import stop
from roboter_arm.shared.joints import JOINTS


def trajectory(channel, start, excursion=5):
    if type(channel) is not int or channel not in JOINTS:
        raise ValueError('Select exactly one channel from 0 through 5')
    if type(start) is not int or type(excursion) is not int or not 1 <= excursion <= 5:
        raise ValueError('Start must be an integer; excursion must be 1–5 counts')
    _, low, high = JOINTS[channel]
    if not low <= start - excursion <= start + excursion <= high:
        raise ValueError('Entire check must stay inside the historical reference range')
    return ([start] + list(range(start + 1, start + excursion + 1))
            + list(range(start + excursion - 1, start - excursion - 1, -1))
            + list(range(start - excursion + 1, start + 1)))


def position(channel, count, seconds):
    if type(channel) is not int or channel not in JOINTS:
        raise ValueError('Select exactly one channel from 0 through 5')
    _, low, high = JOINTS[channel]
    if type(count) is not int or not low <= count <= high:
        raise ValueError('Position must be integer counts inside the historical range')
    if type(seconds) not in (int, float) or not math.isfinite(seconds) or not 0 < seconds <= 5:
        raise ValueError('Commissioning hold must be finite and between 0 and 5 seconds')


def run_position(pwm, channel, count, seconds, sleep=time.sleep):
    position(channel, count, seconds)
    try:
        verify_off(pwm)
        pwm.channels[channel].duty_cycle = count << 4
        sleep(seconds)
    finally:
        pwm.channels[channel].duty_cycle = 0


@contextmanager
def controller(channel):
    # Reuse the reviewed FULL_OFF-before-wake initialization. The legacy gripper
    # helper also clears channel 5; no nonzero pulse is written by initialization.
    with gripper_controller() as pwm:
        try:
            yield pwm
        finally:
            pwm.channels[channel].duty_cycle = 0
            verify_off(pwm)


def run_motion(pwm, channel, start, excursion=5, sleep=time.sleep):
    counts = trajectory(channel, start, excursion)  # Validate before any write.
    try:
        verify_off(pwm)
        for index, count in enumerate(counts):
            pwm.channels[channel].duty_cycle = count << 4
            sleep(1.0 if index == 0 else 0.05)
    finally:
        pwm.channels[channel].duty_cycle = 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--channel', type=int, required=True)
    parser.add_argument('--start', type=int, required=True, help='Explicit first target, in counts')
    movement = parser.add_mutually_exclusive_group()
    movement.add_argument('--excursion', type=int, default=5)
    movement.add_argument('--hold-seconds', type=float, help='Hold only the explicit start target, 0–5 s, then OFF')
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument('--prepare', action='store_true')
    modes.add_argument('--execute', action='store_true')
    parser.add_argument('--supported', action='store_true', help='Current arm support confirmed')
    parser.add_argument('--power-off', action='store_true', help='Current servo power OFF confirmed')
    parser.add_argument('--ready', action='store_true', help='Current attended readiness for this check')
    args = parser.parse_args()
    try:
        if args.hold_seconds is None:
            counts = trajectory(args.channel, args.start, args.excursion)
        else:
            position(args.channel, args.start, args.hold_seconds)
    except ValueError as error:
        parser.error(str(error))
    if args.hold_seconds is None:
        print(f'Channel {args.channel} ({JOINTS[args.channel][0]}); nominal 60 Hz; '
              f'{args.start} -> {max(counts)} -> {min(counts)} -> {args.start} -> OFF.', flush=True)
        duration = 1 + .05 * (len(counts) - 1)
    else:
        print(f'Channel {args.channel} ({JOINTS[args.channel][0]}); nominal 60 Hz; '
              f'hold {args.start} counts -> OFF.', flush=True)
        duration = args.hold_seconds
    print(f'Duration {duration:.2f} s; first move is from unknown position.', flush=True)
    if not (args.prepare or args.execute):
        print('Preview only; no hardware access.')
        return
    if not args.supported or (args.prepare and not args.power_off) or (args.execute and not args.ready):
        parser.error('Hardware action requires --supported and --power-off (prepare) or --ready (execute)')
    handlers = {sig: signal.signal(sig, stop) for sig in
                (signal.SIGINT, signal.SIGTERM, signal.SIGHUP, signal.SIGALRM)}
    signal.alarm(10)
    try:
        with controller(args.channel) as pwm:
            if args.execute:
                if args.hold_seconds is None:
                    run_motion(pwm, args.channel, args.start, args.excursion)
                else:
                    run_position(pwm, args.channel, args.start, args.hold_seconds)
            verify_off(pwm)
        print('Completed; all output registers read back off.', flush=True)
    finally:
        signal.alarm(0)
        for sig, handler in handlers.items():
            signal.signal(sig, handler)

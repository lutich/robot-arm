"""Single gripper check. Default is a hardware-free preview; see confirmation-test.md."""
import argparse
import signal
import time

from roboter_arm.control.infrastructure.pca9685 import CHANNEL, controller

START = 600  # Preserved S6 home_value, not measured current position.


def trajectory():
    """One count per 50 ms: 600 -> 605 -> 595 -> 600, then off."""
    return [START] + list(range(601, 606)) + list(range(604, 594, -1)) + list(range(596, 601))


def stop(_signum, _frame):
    raise KeyboardInterrupt('Stopped; attempting to disable gripper output')


def run_motion(pwm, sleep=time.sleep):
    try:
        for index, count in enumerate(trajectory()):
            pwm.channels[CHANNEL].duty_cycle = count << 4
            sleep(1.0 if index == 0 else 0.05)
    finally:
        pwm.channels[CHANNEL].duty_cycle = 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument('--prepare', action='store_true', help='Servo power OFF: disable all channels')
    modes.add_argument('--execute', action='store_true', help='Attended powered gripper check')
    args = parser.parse_args()
    print('Channel 5; nominal 60 Hz; counts 600 -> 605 -> 595 -> 600 -> OFF.', flush=True)
    print('First positioning can be larger. Support arm; keep servo power switch reachable.', flush=True)
    if not (args.prepare or args.execute):
        print('Preview only. Hardware was not imported or accessed.')
        return
    handlers = {sig: signal.signal(sig, stop) for sig in
                (signal.SIGINT, signal.SIGTERM, signal.SIGHUP, signal.SIGALRM)}
    signal.alarm(10)
    try:
        with controller() as pwm:
            if args.execute:
                run_motion(pwm)
            for channel in pwm.channels:
                if channel.duty_cycle != 0:
                    raise RuntimeError('Output-off verification failed; disconnect servo power')
        print('Completed; outputs off.', flush=True)
    finally:
        signal.alarm(0)
        for sig, handler in handlers.items():
            signal.signal(sig, handler)

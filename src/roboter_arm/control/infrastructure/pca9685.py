"""PCA9685 ownership and FULL_OFF-first initialization; imports never touch hardware."""
from contextlib import contextmanager

from roboter_arm.shared.paths import ROOT

CHANNEL = 5  # Gripper; _controller also clears it on exit, as the legacy gripper helper did.
FREQUENCY = 60


@contextmanager
def controller():
    import fcntl
    lock_path = ROOT / 'artifacts/controller.lock'
    lock_path.parent.mkdir(parents=True, exist_ok=True)
    with lock_path.open('a') as owner:
        try:
            fcntl.flock(owner, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise RuntimeError('Another process owns the controller') from None
        with _controller() as pwm:
            yield pwm


@contextmanager
def _controller():
    # Imports alone must never enable hardware. Stop every channel before reset/wake.
    import board
    from adafruit_bus_device.i2c_device import I2CDevice
    from adafruit_pca9685 import PCA9685

    bus = board.I2C()
    pwm = None
    try:
        device = I2CDevice(bus, 0x40)
        with device:
            for channel in range(16):
                # LEDn_OFF_H FULL_OFF; single-register writes do not require AI.
                device.write(bytes((0x09 + 4 * channel, 0x10)))
        pwm = PCA9685(bus, address=0x40)
        pwm.frequency = FREQUENCY
        for channel in pwm.channels:
            channel.duty_cycle = 0
        yield pwm
    finally:
        try:
            if pwm is not None:
                pwm.channels[CHANNEL].duty_cycle = 0
        finally:
            bus.deinit()


def verify_off(pwm):
    if any(channel.duty_cycle != 0 for channel in pwm.channels):
        raise RuntimeError('Output-off verification failed; disconnect servo power')

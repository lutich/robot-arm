"""Preview and explicit PCA9685 adapters; construction never accesses hardware."""
from roboter_arm.control.infrastructure.pca9685 import controller, verify_off


class PreviewDriver:
    scope = 'synthetic'

    def __init__(self):
        self.outputs = {c:0 for c in range(16)}

    def prepare(self):
        self.outputs = {c:0 for c in range(16)}

    def write(self, channel, count):
        self.outputs[channel] = count

    def disable(self, channel):
        self.outputs[channel] = 0

    def verify_off(self):
        if any(self.outputs.values()):
            raise RuntimeError('Output-off verification failed')

    def close(self):
        pass


class HardwareDriver:
    scope = 'commissioning'

    def __init__(self):
        self.context = None
        self.pwm = None  # Construction has no hardware action.

    def prepare(self):
        if self.context is not None:
            raise RuntimeError('Already prepared')
        context = controller()
        self.pwm = context.__enter__()
        self.context = context
        try:
            verify_off(self.pwm)
        except BaseException:
            self.close()
            raise

    def write(self, channel, count):
        self.pwm.channels[channel].duty_cycle = count << 4

    def disable(self, channel):
        self.pwm.channels[channel].duty_cycle = 0

    def verify_off(self):
        verify_off(self.pwm)

    def close(self):
        if self.context is not None:
            try:
                self.context.__exit__(None, None, None)
            finally:
                self.context, self.pwm = None, None

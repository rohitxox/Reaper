import board
from adafruit_pca9685 import PCA9685

__version__ = "0.0.0-auto.0"
__repo__ = "https://github.com/adafruit/Adafruit_CircuitPython_ServoKit.git"


class ServoKit:
    """Class representing an Adafruit PWM/Servo FeatherWing, Shield or Pi HAT and Bonnet kits.

    Automatically uses the I2C bus on a Feather, Metro or Raspberry Pi.

    Initialise the PCA9685 chip at ``address``.

    The internal reference clock is 25MHz but may vary slightly with environmental conditions and
    manufacturing variances. Providing a more precise ``reference_clock_speed`` can improve the
    accuracy of the frequency and duty_cycle computations. See the ``calibration.py`` example in
    the `PCA9685 GitHub repo <https://github.com/adafruit/Adafruit_CircuitPython_PCA9685>`_ for
    how to derive this value by measuring the resulting pulse widths.

    :param int channels: The number of servo channels available. Must be 8 or 16. The FeatherWing
                         has 8 channels. The Shield, HAT, and Bonnet have 16 channels.
    :param int address: The I2C address of the PCA9685. Default address is ``0x40``.
    :param int reference_clock_speed: The frequency of the internal reference clock in Hertz.
                                      Default reference clock speed is ``25000000``.
    :param int frequency: The overall PWM frequency of the PCA9685 in Hertz.
                                      Default frequency is ``50``.

    """

    def __init__(
        self,
        *,
        channels,
        i2c=None,
        address=0x41,
        reference_clock_speed=25000000,
        frequency=50
    ):
        if channels not in [8, 16]:
            raise ValueError("servo_channels must be 8 or 16!")
        self._items = [None] * channels
        self._channels = channels
        if i2c is None:
            i2c = board.I2C()
        self._pca = PCA9685(
            i2c, address=address, reference_clock_speed=reference_clock_speed
        )
        self._pca.frequency = frequency

        self._servo = _Servo(self)
        self._continuous_servo = _ContinuousServo(self)

    @property
    def servo(self):
        """:class:`~adafruit_motor.servo.Servo` controls for standard servos.

        This FeatherWing example rotates a servo on channel ``0`` to ``180`` degrees for one second,
        and then returns it to ``0`` degrees.

        .. code-block:: python

            import time
            from adafruit_servokit import ServoKit

            kit = ServoKit(channels=8)

            kit.servo[0].angle = 180
            time.sleep(1)
            kit.servo[0].angle = 0

        """
        return self._servo

    @property
    def continuous_servo(self):
        """:class:`~adafruit_motor.servo.ContinuousServo` controls for continuous rotation
        servos.

        This FeatherWing example rotates a continuous rotation servo on channel ``0`` forward for
        one second, then backward for one second, and then stops the rotation.

        .. code-block:: python

            import time
            from adafruit_servokit import ServoKit

            kit = ServoKit(channels=8)

            kit.continuous_servo[0].throttle = 1
            time.sleep(1)
            kit.continuous_servo[0].throttle = -1
            time.sleep(1)
            kit.continuous_servo[0].throttle = 0

        """
        return self._continuous_servo


class _Servo:
    # pylint: disable=protected-access
    def __init__(self, kit):
        self.kit = kit

    def __getitem__(self, servo_channel):
        import adafruit_motor.servo  # pylint: disable=import-outside-toplevel

        num_channels = self.kit._channels
        if servo_channel >= num_channels or servo_channel < 0:
            raise ValueError("servo must be 0-{}!".format(num_channels - 1))
        servo = self.kit._items[servo_channel]
        if servo is None:
            servo = adafruit_motor.servo.Servo(self.kit._pca.channels[servo_channel])
            self.kit._items[servo_channel] = servo
            return servo
        if isinstance(self.kit._items[servo_channel], adafruit_motor.servo.Servo):
            return servo
        raise ValueError("Channel {} is already in use.".format(servo_channel))

    def __len__(self):
        return len(self.kit._items)


class _ContinuousServo:
    # pylint: disable=protected-access
    def __init__(self, kit):
        self.kit = kit

    def __getitem__(self, servo_channel):
        import adafruit_motor.servo  # pylint: disable=import-outside-toplevel

        num_channels = self.kit._channels
        if servo_channel >= num_channels or servo_channel < 0:
            raise ValueError("servo must be 0-{}!".format(num_channels - 1))
        servo = self.kit._items[servo_channel]
        if servo is None:
            servo = adafruit_motor.servo.ContinuousServo(
                self.kit._pca.channels[servo_channel]
            )
            self.kit._items[servo_channel] = servo
            return servo
        if isinstance(
            self.kit._items[servo_channel], adafruit_motor.servo.ContinuousServo
        ):
            return servo
        raise ValueError("Channel {} is already in use.".format(servo_channel))

    def __len__(self):
        return len(self.kit._items)
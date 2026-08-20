"""I2C driver for the Yourfun NeZha bus driver board.

Protocol notes are derived from the vendor SDK in `vendor/yourfun-nezha/sdk/`.
See `docs/hardware/nezha-i2c-protocol.md` for the reconstructed command reference, and
`docs/adr/0004-nezha-python-driver-port.md` for why the vendor C SDK was reimplemented here.

The board is an I2C slave at 7-bit address `0x40`. All commands are written to command register
`0x00` first. Commands with arguments then send a second data frame, matching the vendor behavior.
"""

from __future__ import annotations

import time
from typing import TYPE_CHECKING, Self

if TYPE_CHECKING:
    from collections.abc import Callable

    from smbus2 import SMBus

DEFAULT_ADDRESS = 0x40
DEFAULT_BUS = 1

# Command register: every command is written here first
REG_COMMAND = 0x00

CMD_MOTOR_INIT = 0x01
CMD_MOTOR_SET = (0x05, 0x09, 0x0D, 0x11)
CMD_ENCODER_INIT = (0x15, 0x18, 0x1B, 0x1E)
CMD_ENCODER_READ = (0x16, 0x19, 0x1C, 0x1F)
CMD_SERVO_INIT = (0x21, 0x24, 0x27, 0x2A)
CMD_SERVO_SET = (0x22, 0x25, 0x28, 0x2B)
CMD_RESET = 0xFF

CMD_LED = {
    # name: (on, off, toggle)
    "head": (0x2D, 0x2E, 0x2F),
    "tail_left": (0x30, 0x31, 0x32),
    "tail_right": (0x33, 0x34, 0x35),
    "ambient": (0x36, 0x37, 0x38),
}

MAX_SPEED = 1000
SERVO_PWM_MIN = 50
SERVO_PWM_MAX = 250

# Power-up delays marked as required by the vendor implementation
RESET_DELAY_S = 0.1
INIT_DELAY_S = 0.5

# A write to this board sets state rather than advancing it — re-sending a motor speed, an init,
# or a reset lands on the same result — so a transient bus error is safe to retry. Errno 121
# (Remote I/O error) was observed on 2026-08-19 during a line-follow run with all four motors
# driving; without a retry it killed the run and the stop path with it.
WRITE_RETRIES = 2
WRITE_RETRY_DELAY_S = 0.002

# The vendor manual and source comments disagree on whether motor_a or motor_b is "forward".
# Flip this constant only after verifying against real hardware.
FORWARD_IS_MOTOR_A = True


class NeZhaError(RuntimeError):
    """Raised when communication with the driver board fails."""


class NeZha:
    """NeZha board abstraction for 4 motors, 4 servos, 4 LEDs, and 4 encoder channels.

    Motor and servo channel numbers are **1-4**, matching the silkscreen labels `M1`-`M4`
    and `S1`-`S4` on the board.

    >>> with NeZha() as board:
    ...     board.motor(1, 300)
    ...     time.sleep(1)
    ...     board.stop()
    """

    def __init__(
        self,
        bus: int | SMBus = DEFAULT_BUS,
        address: int = DEFAULT_ADDRESS,
        *,
        init_motors: bool = True,
    ) -> None:
        if isinstance(bus, int):
            try:
                from smbus2 import SMBus
            except ImportError as exc:  # pragma: no cover - only triggered off-device
                raise NeZhaError(
                    "smbus2 is required. Run `uv sync` on the Raspberry Pi or pass an SMBus object."
                ) from exc
            self._bus = SMBus(bus)
            self._owns_bus = True
        else:
            self._bus = bus
            self._owns_bus = False

        self.address = address
        #: Transient bus writes that needed a retry. A run that ends with this well above zero is
        #: reporting a marginal bus, even though nothing raised.
        self.write_retries = 0
        time.sleep(INIT_DELAY_S)
        self.reset()
        if init_motors:
            self.init_motors()

    # ---------------------------------------------------------------- Low level
    def _retry_write(self, write: Callable[[], None]) -> None:
        """Run a bus write, retrying transient errors. Raises the last `OSError` if all fail."""
        for attempt in range(WRITE_RETRIES + 1):
            try:
                write()
            except OSError:
                if attempt == WRITE_RETRIES:
                    raise
                self.write_retries += 1
                time.sleep(WRITE_RETRY_DELAY_S)
            else:
                return

    def _command(self, command: int) -> None:
        """Write the command register. Mirrors vendor `NeZha_WriteCommand()`."""
        try:
            self._retry_write(lambda: self._bus.write_byte_data(self.address, REG_COMMAND, command))
        except OSError as exc:
            raise NeZhaError(
                f"Failed to write command 0x{command:02X} to address 0x{self.address:02X} "
                f"after {WRITE_RETRIES + 1} attempts. "
                "Check wiring, power, and whether the I2C clock exceeds 200kHz."
            ) from exc

    def _command_with_data(self, command: int, data: list[int]) -> None:
        """Send the command register write, then the data frame in a second transfer."""
        self._command(command)
        try:
            self._retry_write(lambda: self._bus.write_i2c_block_data(self.address, command, data))
        except OSError as exc:
            raise NeZhaError(f"Failed to send data for command 0x{command:02X}.") from exc

    @staticmethod
    def _split_u16(value: int) -> list[int]:
        return [(value >> 8) & 0xFF, value & 0xFF]

    @staticmethod
    def _check_channel(n: int) -> int:
        if n not in (1, 2, 3, 4):
            raise ValueError(f"Channel must be in the range 1-4, got {n}")
        return n - 1

    # ---------------------------------------------------------------- Basics
    def reset(self) -> None:
        self._command(CMD_RESET)
        time.sleep(RESET_DELAY_S)

    def init_motors(self) -> None:
        self._command(CMD_MOTOR_INIT)

    def close(self, *, stop_motors: bool = True) -> None:
        """Close the bus, optionally stopping all wheel motors first."""
        try:
            if stop_motors:
                self.stop()
        finally:
            if self._owns_bus:
                self._bus.close()

    def __enter__(self) -> Self:
        return self

    def __exit__(self, *exc_info: object) -> None:
        self.close()

    # ---------------------------------------------------------------- Motors
    def motor(self, n: int, speed: int) -> None:
        """Set the speed of motor `Mn`.

        `speed` ranges from -1000 to 1000. Positive values mean forward, negative values mean
        reverse, and 0 stops the motor. The absolute value is the PWM duty cycle in thousandths.
        """
        idx = self._check_channel(n)
        if not -MAX_SPEED <= speed <= MAX_SPEED:
            raise ValueError(f"Speed must be between -{MAX_SPEED} and {MAX_SPEED}, got {speed}")

        # The vendor manual marks non-zero motor_a and motor_b at the same time as invalid.
        forward, reverse = (abs(speed), 0) if speed >= 0 else (0, abs(speed))
        a, b = (forward, reverse) if FORWARD_IS_MOTOR_A else (reverse, forward)
        self._command_with_data(CMD_MOTOR_SET[idx], self._split_u16(a) + self._split_u16(b))

    def motors(self, m1: int, m2: int, m3: int, m4: int) -> None:
        """Set all four motors in sequence."""
        for n, speed in enumerate((m1, m2, m3, m4), start=1):
            self.motor(n, speed)

    def stop(self) -> None:
        """Stop all four motors."""
        self.motors(0, 0, 0, 0)

    # -------------------------------------------------------------- Encoders
    def init_encoder(self, n: int) -> None:
        self._command(CMD_ENCODER_INIT[self._check_channel(n)])

    def encoder(self, n: int) -> int:
        """Read encoder speed for motor `Mn`.

        Returns a signed 16-bit value. Sampling period is fixed at 20ms in firmware. Call
        `init_encoder(n)` before reading.
        """
        idx = self._check_channel(n)
        try:
            hi, lo = self._bus.read_i2c_block_data(self.address, CMD_ENCODER_READ[idx], 2)
        except OSError as exc:
            raise NeZhaError(f"Failed to read encoder for M{n}.") from exc
        return int.from_bytes(bytes((hi, lo)), "big", signed=True)

    # ---------------------------------------------------------------- Servos
    def init_servo(self, n: int) -> None:
        """Initialize servo `Sn`. Each servo port must be initialized before use."""
        self._command(CMD_SERVO_INIT[self._check_channel(n)])

    def servo_pwm(self, n: int, pwm: int) -> None:
        """Set the raw PWM value for servo `Sn`, in the range 50-250."""
        idx = self._check_channel(n)
        if not SERVO_PWM_MIN <= pwm <= SERVO_PWM_MAX:
            raise ValueError(
                f"Servo PWM must be between {SERVO_PWM_MIN} and {SERVO_PWM_MAX}, got {pwm}"
            )
        self._command_with_data(CMD_SERVO_SET[idx], self._split_u16(pwm))

    def servo(self, n: int, angle: float) -> None:
        """Move servo `Sn` to a target angle in the range 0-180 degrees."""
        if not 0 <= angle <= 180:
            raise ValueError(f"Angle must be between 0 and 180, got {angle}")
        span = SERVO_PWM_MAX - SERVO_PWM_MIN
        self.servo_pwm(n, round(SERVO_PWM_MIN + angle / 180 * span))

    # -------------------------------------------------------------------- LEDs
    def led(self, name: str, state: bool | None) -> None:
        """Control an onboard LED.

        `state=None` toggles the current state. Valid names are `head`, `tail_left`,
        `tail_right`, and `ambient`.
        """
        if name not in CMD_LED:
            raise ValueError(f"Unknown LED name {name!r}; valid names: {', '.join(CMD_LED)}")
        on, off, toggle = CMD_LED[name]
        self._command(toggle if state is None else (on if state else off))

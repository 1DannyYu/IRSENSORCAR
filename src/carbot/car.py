"""Differential-drive control for the four-wheel chassis.

This layer wraps the per-motor `NeZha` API into left/right drive commands and applies the wheel
mapping and inversion rules from `config.py`.

    from carbot import Car

    with Car() as car:
        car.forward(300)
        time.sleep(1)
        car.stop()

The current `config.WHEEL_TO_MOTOR` mapping and inversion settings were verified on real hardware.
If motor wiring changes, lift the car and rerun `examples/02_motor_check.py`.
"""

from __future__ import annotations

import time
import warnings
from collections.abc import Callable
from typing import TYPE_CHECKING, Self

from carbot import config
from carbot.nezha import MAX_SPEED, NeZha, NeZhaError

if TYPE_CHECKING:
    from smbus2 import SMBus

LEFT_WHEELS = ("front_left", "rear_left")
RIGHT_WHEELS = ("front_right", "rear_right")

# Attempts per motor on the best-effort stop path, on top of the driver's own write retries.
STOP_ATTEMPTS = 3
STOP_RETRY_DELAY_S = 0.01
# Reassert a timed movement periodically.  The NeZha command is state-setting, so this does not
# change the requested speed; it only recovers quickly if another one-shot write or a transient
# board reset replaces the active command during a multi-second open-loop movement.
MOVE_REFRESH_S = 0.01

WHEELS_MAY_BE_TURNING = (
    "Could not confirm every motor stopped — the wheels may still be turning. Cut power."
)


class Car:
    """Four-wheel differential-drive chassis controller.

    Speeds always use the range -1000 to 1000. Positive values mean forward, negative values mean
    reverse, and the absolute value represents PWM duty cycle in thousandths.
    """

    def __init__(self, board: NeZha | int | SMBus | None = None) -> None:
        self._board = (
            board if isinstance(board, NeZha) else NeZha(board if board is not None else 1)
        )
        self._owns_board = not isinstance(board, NeZha)
        self._left = [config.WHEEL_TO_MOTOR[w] for w in LEFT_WHEELS]
        self._right = [config.WHEEL_TO_MOTOR[w] for w in RIGHT_WHEELS]

    @property
    def board(self) -> NeZha:
        """Expose the low-level board object for direct servo or LED control."""
        return self._board

    # ---------------------------------------------------------------- Core
    def drive(self, left: int, right: int) -> None:
        """Set left and right side speeds. All movement funnels through this method."""
        for speed, motors in ((left, self._left), (right, self._right)):
            speed = max(-MAX_SPEED, min(MAX_SPEED, speed))
            for n in motors:
                self._board.motor(n, -speed if n in config.INVERTED_MOTORS else speed)

    def stop(self, *, best_effort: bool = False) -> bool:
        """Set both sides to zero. Returns whether every motor accepted the command.

        With `best_effort`, keep trying the remaining motors after one fails and return `False`
        instead of raising. A cleanup path must not abandon three motors because the first one
        would not answer, and must not mask the exception that sent it there — but a caller that
        gets `False` back has wheels that may still be turning, and must say so.
        """
        if not best_effort:
            self.drive(0, 0)
            return True

        stopped = True
        for n in self._left + self._right:
            for attempt in range(STOP_ATTEMPTS):
                try:
                    self._board.motor(n, 0)
                except NeZhaError:
                    if attempt == STOP_ATTEMPTS - 1:
                        stopped = False
                    time.sleep(STOP_RETRY_DELAY_S)
                else:
                    break
        return stopped

    # ------------------------------------------------------------- Movements
    def forward(self, speed: int = config.SAFE_TEST_SPEED) -> None:
        self.drive(abs(speed), abs(speed))

    def backward(self, speed: int = config.SAFE_TEST_SPEED) -> None:
        self.drive(-abs(speed), -abs(speed))

    def turn_left(self, speed: int = config.SAFE_TEST_SPEED, *, ratio: float = 0.3) -> None:
        """Drive forward while turning left. `ratio` scales the inside wheel speed."""
        self.drive(round(abs(speed) * ratio), abs(speed))

    def turn_right(self, speed: int = config.SAFE_TEST_SPEED, *, ratio: float = 0.3) -> None:
        self.drive(abs(speed), round(abs(speed) * ratio))

    def spin_left(self, speed: int = config.SAFE_TEST_SPEED) -> None:
        """Rotate left in place."""
        self.drive(-abs(speed), abs(speed))

    def spin_right(self, speed: int = config.SAFE_TEST_SPEED) -> None:
        self.drive(abs(speed), -abs(speed))

    def move_for(
        self,
        seconds: float,
        left: int,
        right: int,
        *,
        on_command: Callable[[int, float, int, int], None] | None = None,
    ) -> int:
        """Drive for a fixed duration, refreshing the command, then stop safely."""
        if seconds < 0:
            raise ValueError("seconds must be non-negative")
        started = time.monotonic()
        deadline = started + seconds
        writes = 0
        try:
            while True:
                self.drive(left, right)
                writes += 1
                now = time.monotonic()
                if on_command is not None:
                    on_command(writes, now - started, left, right)
                remaining = deadline - now
                if remaining <= 0:
                    break
                time.sleep(min(MOVE_REFRESH_S, remaining))
        finally:
            if not self.stop(best_effort=True):
                warnings.warn(WHEELS_MAY_BE_TURNING, RuntimeWarning, stacklevel=2)
        return writes

    # --------------------------------------------------------------- Cleanup
    def close(self) -> None:
        try:
            if not self.stop(best_effort=True):
                warnings.warn(WHEELS_MAY_BE_TURNING, RuntimeWarning, stacklevel=2)
        finally:
            if self._owns_board:
                # The board's own close() would stop the motors again through the same failing
                # bus; we have already tried that as hard as we are going to.
                self._board.close(stop_motors=False)

    def __enter__(self) -> Self:
        return self

    def __exit__(self, *exc_info: object) -> None:
        self.close()

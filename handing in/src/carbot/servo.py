"""Guarded servo check for the arm's three servos (S2, S3, S4).

The sequence and its safety gate used to live inside
``examples/04_servo_check.py``. That put unit-testable behaviour — the
exact-``yes`` clearance gate, the one-channel-at-a-time ordering, and the
guarantee that the board is closed with ``stop_motors=False`` on every exit
path — in a file that cannot be imported, because ``04_servo_check`` is not a
valid Python identifier. ``tests/test_servo_check.py`` had to load it through
``importlib`` to reach it. The logic lives here instead so the tests can
import it like any other module; the example is now argparse-free wiring.

Movement is deliberately small: each servo is stepped around its 90° centre
by ±10° only, one channel at a time, with an operator confirmation before
every single move. Servos hold torque after the run, so the operator is told
to cut main power once results are recorded.
"""

from __future__ import annotations

from collections.abc import Callable
from typing import Protocol

from carbot.nezha import NeZhaError

#: Arm servo channels in the order they are exercised.
SERVO_CHANNELS = (2, 3, 4)

#: Angles each channel is stepped through: centre, -10°, +10°, back to centre.
TEST_ANGLES = (90, 80, 100, 90)

#: The only accepted answer to the clearance question. Anything else — "y",
#: "YES ", an empty line — refuses to move, because a mistyped or reflexive
#: confirmation must not energise an arm that someone still has a hand in.
CLEARANCE_ANSWER = "yes"

EXIT_OK = 0
EXIT_REFUSED = 1
EXIT_INTERRUPTED = 130


class ServoBoard(Protocol):
    """The board surface :func:`run_check` needs."""

    def init_servo(self, channel: int) -> None: ...

    def servo(self, channel: int, angle: float) -> None: ...


class ManagedServoBoard(ServoBoard, Protocol):
    """A :class:`ServoBoard` the session is also responsible for closing."""

    def close(self, *, stop_motors: bool) -> None: ...


def clearance_confirmed(answer: str) -> bool:
    """True only for an exact ``yes``. See :data:`CLEARANCE_ANSWER`."""
    return answer == CLEARANCE_ANSWER


def run_check(
    board: ServoBoard,
    prompt: Callable[[str], str] = input,
    output: Callable[[str], None] = print,
) -> None:
    """Step S2, S3, S4 through :data:`TEST_ANGLES`, one channel at a time.

    Every individual move waits on ``prompt`` first, so the operator can stop
    between any two commands rather than only between channels.
    """
    for channel in SERVO_CHANNELS:
        output(f"\n=== S{channel} ===")
        board.init_servo(channel)

        for angle in TEST_ANGLES:
            prompt(
                f"Confirm that hands are clear and the joint is free, then press Enter "
                f"to move S{channel} to {angle}°. Cut main power immediately if anything "
                "looks or sounds wrong."
            )
            board.servo(channel, angle)

        output(
            f"S{channel} complete: record the joint it controls, its direction, "
            "and any unusual noise."
        )


def run_session(
    open_board: Callable[[], ManagedServoBoard],
    prompt: Callable[[str], str] = input,
    output: Callable[[str], None] = print,
) -> int:
    """Gate on operator clearance, run the check, and always close the board.

    ``open_board`` is a factory rather than a board so that nothing touches
    I2C until the clearance answer is an exact ``yes`` — a refused run must
    not have constructed a board at all.

    The board is closed with ``stop_motors=False`` on every exit path: this
    check never started the motors, and stopping them here would issue motor
    commands the operator did not ask for.

    Returns :data:`EXIT_OK`, :data:`EXIT_REFUSED`, or :data:`EXIT_INTERRUPTED`.
    """
    output("This test checks S2, S3, and S4 in order using small moves around 90°.")
    output("Cut main power immediately if a joint binds, jitters, buzzes, or nears its limit.")

    if not clearance_confirmed(
        prompt("Is the arm area clear and the main power switch within reach? (yes/no) ")
    ):
        output("Exact confirmation 'yes' was not received; no servo commands were sent.")
        return EXIT_REFUSED

    board: ManagedServoBoard | None = None
    try:
        board = open_board()
        run_check(board, prompt=prompt, output=output)
    except KeyboardInterrupt:
        output("\nTest interrupted. Servos may still hold torque; switch off main power.")
        return EXIT_INTERRUPTED
    except NeZhaError as exc:
        output(f"\nCommunication failed: {exc}")
        output("Switch off main power, then inspect the wiring and I2C connection.")
        return EXIT_REFUSED
    finally:
        if board is not None:
            board.close(stop_motors=False)

    output("\nAll three ports are complete. Servos may still hold torque; record results and switch off main power.")
    return EXIT_OK

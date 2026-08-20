from __future__ import annotations

import pytest

from carbot.nezha import NeZhaError
from carbot.servo import (
    EXIT_INTERRUPTED,
    EXIT_OK,
    EXIT_REFUSED,
    clearance_confirmed,
    run_check,
    run_session,
)


class FakeBoard:
    def __init__(self) -> None:
        self.calls: list[tuple[object, ...]] = []

    def init_servo(self, channel: int) -> None:
        self.calls.append(("init_servo", channel))

    def servo(self, channel: int, angle: int) -> None:
        self.calls.append(("servo", channel, angle))

    def close(self, *, stop_motors: bool) -> None:
        self.calls.append(("close", stop_motors))


def test_run_check_initializes_and_moves_one_channel_at_a_time():
    board = FakeBoard()
    prompts: list[str] = []

    run_check(
        board,
        prompt=lambda message: prompts.append(message) or "",
        output=lambda _message: None,
    )

    assert board.calls == [
        ("init_servo", 2),
        ("servo", 2, 90),
        ("servo", 2, 80),
        ("servo", 2, 100),
        ("servo", 2, 90),
        ("init_servo", 3),
        ("servo", 3, 90),
        ("servo", 3, 80),
        ("servo", 3, 100),
        ("servo", 3, 90),
        ("init_servo", 4),
        ("servo", 4, 90),
        ("servo", 4, 80),
        ("servo", 4, 100),
        ("servo", 4, 90),
    ]
    assert len(prompts) == 12


@pytest.mark.parametrize("answer", ["", "no", "YES ", "y", " yes", "Yes"])
def test_clearance_requires_exact_yes(answer: str):
    assert clearance_confirmed(answer) is False


def test_clearance_accepts_exact_yes():
    assert clearance_confirmed("yes") is True


@pytest.mark.parametrize("answer", ["", "no", "YES ", "y"])
def test_session_rejects_any_confirmation_except_exact_yes(answer: str):
    opened = False

    def forbidden_factory():
        nonlocal opened
        opened = True
        raise AssertionError("the board must not be opened without clearance")

    result = run_session(
        forbidden_factory,
        prompt=lambda _message: answer,
        output=lambda _message: None,
    )

    assert result == EXIT_REFUSED
    assert opened is False


def test_session_runs_check_and_closes_without_motor_stop():
    board = FakeBoard()

    result = run_session(
        lambda: board,
        prompt=lambda _message: "yes",
        output=lambda _message: None,
    )

    assert result == EXIT_OK
    assert board.calls[0] == ("init_servo", 2)
    assert board.calls[-1] == ("close", False)


def test_session_handles_keyboard_interrupt_and_closes():
    closed: list[bool] = []

    class Board(FakeBoard):
        def init_servo(self, channel: int) -> None:
            raise KeyboardInterrupt

        def close(self, *, stop_motors: bool) -> None:
            closed.append(stop_motors)

    result = run_session(
        Board,
        prompt=lambda _message: "yes",
        output=lambda _message: None,
    )

    assert result == EXIT_INTERRUPTED
    assert closed == [False]


def test_session_handles_communication_error_and_closes():
    closed: list[bool] = []

    class Board(FakeBoard):
        def init_servo(self, channel: int) -> None:
            raise NeZhaError("I2C failed")

        def close(self, *, stop_motors: bool) -> None:
            closed.append(stop_motors)

    result = run_session(
        Board,
        prompt=lambda _message: "yes",
        output=lambda _message: None,
    )

    assert result == EXIT_REFUSED
    assert closed == [False]


def test_session_closes_even_when_the_board_fails_to_open():
    def failing_factory():
        raise NeZhaError("no board on the bus")

    result = run_session(
        failing_factory,
        prompt=lambda _message: "yes",
        output=lambda _message: None,
    )

    assert result == EXIT_REFUSED

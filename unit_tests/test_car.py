"""Cover the differential-drive layer, and above all the stop path.

A 2026-08-19 line-follow run died on an I2C write and then died again inside `car.stop()`,
so the wheels kept turning. Everything here that looks paranoid is that failure.
"""

from __future__ import annotations

import time
import warnings

import pytest

from carbot import config
from carbot.car import WHEELS_MAY_BE_TURNING, Car
from carbot.nezha import NeZhaError


class RecordingBoard:
    """Stands in for `NeZha`, optionally refusing to talk to some motors."""

    def __init__(self, *, dead_motors: set[int] | None = None) -> None:
        self.commands: list[tuple[int, int]] = []
        self.dead = dead_motors or set()
        self.closed_with_stop: bool | None = None
        self.write_retries = 0

    def motor(self, n: int, speed: int) -> None:
        if n in self.dead:
            raise NeZhaError(f"M{n} is not answering")
        self.commands.append((n, speed))

    def close(self, *, stop_motors: bool = True) -> None:
        self.closed_with_stop = stop_motors


@pytest.fixture
def board() -> RecordingBoard:
    return RecordingBoard()


def _car(board: RecordingBoard) -> Car:
    car = Car.__new__(Car)  # bypass NeZha construction; no bus in the test environment
    car._board = board
    car._owns_board = True
    car._left = [1, 3]
    car._right = [2, 4]
    return car


def test_plain_stop_propagates_a_bus_error(board: RecordingBoard):
    """The normal path must still fail loudly — silence there hides a broken board."""
    car = _car(RecordingBoard(dead_motors={1}))
    with pytest.raises(NeZhaError):
        car.stop()


def test_best_effort_stop_reaches_every_motor_despite_one_failing():
    board = RecordingBoard(dead_motors={1})
    car = _car(board)
    assert car.stop(best_effort=True) is False
    assert sorted(n for n, _ in board.commands) == [2, 3, 4]
    assert all(speed == 0 for _, speed in board.commands)


def test_best_effort_stop_reports_success_when_all_motors_answer(board: RecordingBoard):
    car = _car(board)
    assert car.stop(best_effort=True) is True
    assert sorted(n for n, _ in board.commands) == [1, 2, 3, 4]


def test_close_warns_when_the_stop_could_not_be_confirmed():
    car = _car(RecordingBoard(dead_motors={4}))
    with pytest.warns(RuntimeWarning, match=WHEELS_MAY_BE_TURNING):
        car.close()


def test_close_does_not_ask_the_board_to_stop_again(board: RecordingBoard):
    """`Car.close` has already tried; a second attempt down the same bus buys nothing."""
    car = _car(board)
    with warnings.catch_warnings():
        warnings.simplefilter("error")
        car.close()
    assert board.closed_with_stop is False


def test_move_for_warns_rather_than_masking_a_failed_stop():
    """The drive error still surfaces, but not before the stop has been attempted."""
    car = _car(RecordingBoard(dead_motors={2}))
    with pytest.raises(NeZhaError), pytest.warns(RuntimeWarning, match=WHEELS_MAY_BE_TURNING):
        car.move_for(0.0, 100, 100)


def test_move_for_refreshes_the_active_command_until_deadline(monkeypatch):
    board = RecordingBoard()
    car = _car(board)
    clock = [0.0]
    sleeps: list[float] = []
    audit: list[tuple[int, float, int, int]] = []
    monkeypatch.setattr(time, "monotonic", lambda: clock[0])

    def advance(seconds: float) -> None:
        sleeps.append(seconds)
        clock[0] += seconds

    monkeypatch.setattr(time, "sleep", advance)

    writes = car.move_for(0.025, 100, 100, on_command=lambda *event: audit.append(event))

    nonzero = [(motor, speed) for motor, speed in board.commands if speed]
    assert writes == 4
    assert len(nonzero) == 16  # four wheels, initial command + three refreshes
    assert sleeps == pytest.approx([0.01, 0.01, 0.005])
    assert [event[0] for event in audit] == [1, 2, 3, 4]
    assert [event[1] for event in audit] == pytest.approx([0.0, 0.01, 0.02, 0.025])
    assert board.commands[-4:] == [(1, 0), (3, 0), (2, 0), (4, 0)]


def test_move_for_rejects_negative_duration(board: RecordingBoard):
    car = _car(board)
    with pytest.raises(ValueError, match="seconds"):
        car.move_for(-0.1, 100, 100)
    assert board.commands == []


def test_drive_clamps_to_the_board_range(board: RecordingBoard):
    car = _car(board)
    car.drive(5000, -5000)
    expected = {n: (-1000 if n in config.INVERTED_MOTORS else 1000) for n in (1, 3)} | {
        n: (1000 if n in config.INVERTED_MOTORS else -1000) for n in (2, 4)
    }
    assert dict(board.commands) == expected

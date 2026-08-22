from pathlib import Path
from runpy import run_path

import pytest

SCRIPT = Path(__file__).parents[2] / "examples" / "48_motor_one_second_check.py"
NAMESPACE = run_path(str(SCRIPT))
RUN_ONE_SECOND = NAMESPACE["run_one_second"]


class FakeCar:
    def __init__(self) -> None:
        self.calls: list[tuple[float, int, int]] = []

    def move_for(self, seconds: float, left: int, right: int) -> int:
        self.calls.append((seconds, left, right))
        return 101


def test_moves_both_sides_forward_for_exactly_one_second() -> None:
    car = FakeCar()

    writes = RUN_ONE_SECOND(car, speed=150)

    assert car.calls == [(1.0, 150, 150)]
    assert writes == 101


@pytest.mark.parametrize("speed", [0, 1001])
def test_rejects_speed_outside_the_board_range(speed: int) -> None:
    car = FakeCar()

    with pytest.raises(ValueError, match="speed"):
        RUN_ONE_SECOND(car, speed=speed)

    assert car.calls == []

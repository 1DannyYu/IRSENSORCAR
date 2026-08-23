#!/usr/bin/env python3
"""Verify motor port mapping and direction one motor at a time.

⚠️ Lift the car so that all four wheels are off the ground before running this script.

    uv run python examples/02_motor_check.py

Each motor runs forward for one second, stops, then runs in reverse for one second.
Record two things:

1. **Which wheel moved** so you can update `src/carbot/config.py::WHEEL_TO_MOTOR`
2. **Whether "forward" actually moved the wheel forward or backward**

If every motor is reversed, flip `FORWARD_IS_MOTOR_A` in `src/carbot/nezha.py`.
If only some motors are reversed, list them in `config.INVERTED_MOTORS`.
"""

from __future__ import annotations

import sys
import time

from carbot.config import SAFE_TEST_SPEED
from carbot.nezha import NeZha, NeZhaError

HOLD_S = 1.0


def main() -> int:
    answer = (
        input("Is the car lifted with all four wheels off the ground? (yes/no) ").strip().lower()
    )
    if answer != "yes":
        print("Lift the car before running this test.")
        return 1

    try:
        board = NeZha()
    except NeZhaError as exc:
        print(f"Connection failed: {exc}")
        print("Run `examples/01_i2c_probe.py` first to debug the link.")
        return 1

    with board:
        for n in (1, 2, 3, 4):
            print(f"\n=== M{n} ===")
            for label, speed in (("forward", SAFE_TEST_SPEED), ("reverse", -SAFE_TEST_SPEED)):
                print(f"  {label} {abs(speed)}...", flush=True)
                board.motor(n, speed)
                time.sleep(HOLD_S)
                board.motor(n, 0)
                time.sleep(HOLD_S)
            print(f"  Note which wheel M{n} moved and in which physical direction.")

    print("\nDone. Write the verified results into `src/carbot/config.py`.")
    return 0


if __name__ == "__main__":
    sys.exit(main())

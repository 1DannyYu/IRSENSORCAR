#!/usr/bin/env python3
"""Minimal driving test that moves the car.

⚠️ Lift the car so all wheels are off the ground before running this script.
Only place it on the floor after the configuration and directions are verified.

    uv run python examples/03_motor_drive.py
"""

from __future__ import annotations

import sys
import time

from carbot import Car, NeZhaError
from carbot.config import SAFE_TEST_SPEED

MOVES = [
    ("forward", lambda c: c.forward(SAFE_TEST_SPEED)),
    ("backward", lambda c: c.backward(SAFE_TEST_SPEED)),
    ("turn left", lambda c: c.turn_left(SAFE_TEST_SPEED)),
    ("turn right", lambda c: c.turn_right(SAFE_TEST_SPEED)),
    ("spin left", lambda c: c.spin_left(SAFE_TEST_SPEED)),
    ("spin right", lambda c: c.spin_right(SAFE_TEST_SPEED)),
]
HOLD_S = 1.0


def main() -> int:
    if (
        input("Is the car lifted with all four wheels off the ground? (yes/no) ").strip().lower()
        != "yes"
    ):
        print("Lift the car before running this test.")
        return 1

    try:
        car = Car()
    except NeZhaError as exc:
        print(f"Connection failed: {exc}")
        print("Run `examples/01_i2c_probe.py` first to debug the link.")
        return 1

    with car:
        for label, move in MOVES:
            print(f"  {label}...", flush=True)
            move(car)
            time.sleep(HOLD_S)
            car.stop()
            time.sleep(0.5)

    print("\nConfirm that every movement matched its label. If not, update `src/carbot/config.py`.")
    return 0


if __name__ == "__main__":
    sys.exit(main())

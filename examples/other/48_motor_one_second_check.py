#!/usr/bin/env python3
"""Move all four wheels forward for one second, then stop.

This is a deliberately small motor test. It uses ``Car.move_for`` so the active
command is refreshed at 100Hz and a best-effort stop is attempted even if an
exception interrupts the movement.

**Motor-moving. Lift the wheels or secure the chassis. The operator must stand
beside the car and be able to cut main power instantly.**

Run on the Raspberry Pi from the repository root::

    uv run python examples/47_motor_one_second_check.py

Preview without opening I2C or moving motors::

    uv run python examples/47_motor_one_second_check.py --dry-run
"""

from __future__ import annotations

import argparse
from typing import Protocol

from carbot.config import SAFE_TEST_SPEED

MOVE_SECONDS = 1.0


class TimedCar(Protocol):
    """Motor surface needed by this test."""

    def move_for(self, seconds: float, left: int, right: int) -> int: ...


def run_one_second(car: TimedCar, speed: int = SAFE_TEST_SPEED) -> int:
    """Drive both sides forward for exactly one second and return the write count."""
    if not 1 <= speed <= 1000:
        raise ValueError("speed must be in [1, 1000]")
    return car.move_for(MOVE_SECONDS, speed, speed)


def main() -> int:
    parser = argparse.ArgumentParser(description="Move all four wheels forward for one second")
    parser.add_argument(
        "--speed",
        type=int,
        default=SAFE_TEST_SPEED,
        help=f"forward motor PWM in [1, 1000] (default {SAFE_TEST_SPEED})",
    )
    parser.add_argument("--dry-run", action="store_true", help="print the plan; do not open I2C")
    args = parser.parse_args()

    if not 1 <= args.speed <= 1000:
        parser.error("--speed must be in [1, 1000]")

    print(f"Plan: all four wheels forward at PWM {args.speed} for {MOVE_SECONDS:.1f}s, then stop.")
    if args.dry_run:
        print("Dry run complete — no I2C connection opened and no motor command sent.")
        return 0

    answer = input(
        "Operator beside car, wheels lifted/secured, power ready to cut? (yes/no) "
    ).strip()
    if answer.lower() != "yes":
        print("No motor command sent. Re-run when the car is safely prepared.")
        return 1

    from carbot import Car, NeZhaError

    try:
        with Car() as car:
            print("MOVING FORWARD", flush=True)
            writes = run_one_second(car, args.speed)
    except NeZhaError as exc:
        print(f"Motor/I2C test failed: {exc}")
        print("Check that every wheel stopped. Cut main power immediately if unsure.")
        return 1

    print(f"STOPPED after {MOVE_SECONDS:.1f}s ({writes} refreshed drive commands).")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

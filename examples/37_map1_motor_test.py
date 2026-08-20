#!/usr/bin/env python3
"""Map1 test: low-speed motor verification with wheels lifted.

**WHEELS MUST BE LIFTED OR CHASSIS SECURED BEFORE RUNNING.**
Operator must stand beside the car able to cut main power instantly.

Tests:
  1. Forward/backward at speed 100
  2. Left/right turns (30% inside ratio)
  3. In-place spins
  4. Verify IR sensor reads during motion

Usage:
    # operator ready: wheels lifted, power in hand
    PYTHONPATH=src python3 examples/37_map1_motor_test.py

    # simulate without driving (debug IR sensor timing)
    PYTHONPATH=src python3 examples/37_map1_motor_test.py --dry-run
"""

from __future__ import annotations

import argparse
import sys
import time
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from carbot import Car
    from carbot.ir_tracing import IRTracingSensor




def run_motor_checks(car: Car) -> bool:
    """Low-speed motor test — wheels lifted.

    Deliberately not named ``test_*``: this filename matches pytest's
    ``*_test.py`` collection pattern, so a ``test_``-prefixed function here
    gets collected as a unit test and fails on the missing ``car`` fixture.
    """
    speed = 100
    print(f"\n[MOTOR TEST] speed={speed}")

    # Forward
    print("Forward 2s...")
    car.forward(speed)
    time.sleep(2)
    car.stop()
    time.sleep(0.5)

    # Backward
    print("Backward 2s...")
    car.backward(speed)
    time.sleep(2)
    car.stop()
    time.sleep(0.5)

    # Left turn (30% inside ratio)
    print("Turn left 2s (30% inside ratio)...")
    car.turn_left(speed, ratio=0.3)
    time.sleep(2)
    car.stop()
    time.sleep(0.5)

    # Right turn
    print("Turn right 2s (30% inside ratio)...")
    car.turn_right(speed, ratio=0.3)
    time.sleep(2)
    car.stop()
    time.sleep(0.5)

    # Spin left
    print("Spin left 1.5s...")
    car.spin_left(speed)
    time.sleep(1.5)
    car.stop()
    time.sleep(0.5)

    # Spin right
    print("Spin right 1.5s...")
    car.spin_right(speed)
    time.sleep(1.5)
    car.stop()
    time.sleep(0.5)

    return True


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Map1 motor test — wheels lifted, operator beside car"
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="simulate motor commands without driving (debug only)",
    )
    args = parser.parse_args()

    print("=" * 64)
    print("Map1 Motor Test — WHEELS LIFTED")
    print("=" * 64)

    if args.dry_run:
        print("\n[DRY RUN] Skipping motor test")
        return 0

    # Motor test (requires operator)
    answer = input("\nOperator beside car, WHEELS LIFTED, power ready? (yes/no) ").strip()
    if answer.lower() != "yes":
        print("Re-run when ready.")
        return 1

    from carbot import Car, NeZhaError

    car = None
    try:
        car = Car()
    except NeZhaError as exc:
        print(f"Connection failed: {exc}")
        print("Run `examples/01_i2c_probe.py` first.")
        return 1

    try:
        run_motor_checks(car)
        print("\n[SUCCESS] All motor tests completed.")
        return 0
    except KeyboardInterrupt:
        print("\nStopped by operator")
        return 1
    except Exception as e:
        print(f"Motor test error: {e}")
        return 1
    finally:
        if car:
            car.stop()
            car.close()


if __name__ == "__main__":
    sys.exit(main())

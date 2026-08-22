#!/usr/bin/env python3
"""Move forward 15cm, then turn 90 degrees in place, then stop. Nothing else.

Calibration (speed 150, Map1 paper, 2026-08-18 — docs/progress/2026-08-18-map1-turn-and-
travel-calibration.md and examples/44_map1_ir_phase_drive.py): 10.0 cm/s forward,
39.8 deg/s in-place spin. Durations below are derived from those two figures, not guessed.

**Motor-moving. Operator must stand beside the car with a hand on the power cut, wheels
lifted or chassis secured.**

Usage:
    PYTHONPATH=src python3 examples/45_forward_then_turn.py
    PYTHONPATH=src python3 examples/45_forward_then_turn.py --turn-direction left
    PYTHONPATH=src python3 examples/45_forward_then_turn.py --dry-run
"""

from __future__ import annotations

import argparse
import sys
import time

BASE_SPEED = 150  # motor units; both calibration figures below were measured at this speed
FORWARD_CM_S = 10.0  # measured on the Map1 paper, 2026-08-18
SPIN_DEG_S = 39.8  # in-place spin, same session, 6 s outlier excluded

FORWARD_CM = 15.0
TURN_DEG = 90.0


def main() -> int:
    parser = argparse.ArgumentParser(description="Forward 15cm, turn 90 degrees, stop")
    parser.add_argument("--dry-run", action="store_true", help="print the plan, no motor")
    parser.add_argument(
        "--turn-direction", choices=("right", "left"), default="right", help="default: right"
    )
    parser.add_argument("--speed", type=int, default=BASE_SPEED, help=f"default: {BASE_SPEED}")
    args = parser.parse_args()

    forward_s = FORWARD_CM / FORWARD_CM_S
    turn_s = TURN_DEG / SPIN_DEG_S

    print("=" * 60)
    print(f"Forward {FORWARD_CM}cm ({forward_s:.2f}s) then spin {args.turn_direction} "
          f"{TURN_DEG} deg ({turn_s:.2f}s), at speed {args.speed}")
    print("=" * 60)

    if args.dry_run:
        print("Dry run — no motor commands sent.")
        return 0

    if input("Operator beside car, wheels lifted/secured, power ready to cut? (yes/no) ").strip().lower() != "yes":
        print("Re-run when ready.")
        return 1

    from carbot import Car, NeZhaError

    try:
        car = Car()
    except NeZhaError as exc:
        print(f"Connection failed: {exc}")
        return 1

    with car:
        print("Forward...", flush=True)
        car.forward(args.speed)
        time.sleep(forward_s)
        car.stop()
        time.sleep(0.2)

        print(f"Turn {args.turn_direction}...", flush=True)
        if args.turn_direction == "right":
            car.spin_right(args.speed)
        else:
            car.spin_left(args.speed)
        time.sleep(turn_s)
        car.stop()

    print("Done.")
    return 0


if __name__ == "__main__":
    sys.exit(main())

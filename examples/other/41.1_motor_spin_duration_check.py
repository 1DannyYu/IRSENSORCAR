#!/usr/bin/env python3
"""Find the real spin duration for a target turn angle, on the real track surface.

Pure motor test — no camera, no IR reading required. Commands a single
in-place spin for a given duration; the operator visually judges the
resulting heading against a known reference (e.g. the Map1 T-junction's
printed right angle) and adjusts `--duration-s` on the next run until it
matches.

This is deliberately surface-specific: friction differs between the printed
map paper and other floors, so a duration measured elsewhere (e.g. with
`examples/ai_camera/23_cam_spin_rate_check.py` on a textured wall) would not transfer
reliably here. Iterate directly on the track.

**Motor-moving. Operator must stand beside the car able to cut main power instantly.**

Usage:
    # start with a rough guess, e.g. from spin_deg_per_s_at_200=53.5 -> ~2.24s for 90 deg at speed 150
    PYTHONPATH=src python3 examples/other/41.1_motor_spin_duration_check.py --speed 150 --duration-s 2.24 --direction right

    # too far / not far enough -> adjust and re-run
    PYTHONPATH=src python3 examples/other/41.1_motor_spin_duration_check.py --speed 150 --duration-s 1.80 --direction right
"""

from __future__ import annotations

import argparse
import sys


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Single scripted spin — visually verify the resulting angle"
    )
    parser.add_argument("--speed", type=int, default=150, help="spin speed 0-1000 (default 150)")
    parser.add_argument(
        "--duration-s", type=float, required=True, help="spin duration in seconds to test"
    )
    parser.add_argument(
        "--direction", choices=("right", "left"), default="right", help="spin direction"
    )
    args = parser.parse_args()

    print("=" * 60)
    print("Spin Duration Check — visual angle verification, no camera")
    print("=" * 60)
    print(f"Speed: {args.speed} | Duration: {args.duration_s:.2f}s | Direction: {args.direction}")
    print()

    answer = input("Operator beside car, ready to spin? (yes/no) ").strip()
    if answer.lower() != "yes":
        print("Re-run when ready.")
        return 1

    from carbot import Car, NeZhaError

    try:
        car = Car()
    except NeZhaError as exc:
        print(f"Connection failed: {exc}")
        return 1

    speed = abs(args.speed)
    left, right = (speed, -speed) if args.direction == "right" else (-speed, speed)

    try:
        print(f"Spinning {args.direction} for {args.duration_s:.2f}s...")
        car.move_for(args.duration_s, left, right)
        print("Done. Check the resulting heading against the reference angle.")
    finally:
        car.close()

    return 0


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""Sweep spin duration vs. actual turned angle, measured on the real track paper.

Pure motor test — no camera. For each duration in `--durations`, the car
spins in place for that long; the operator resets it to the reference heading
between spins and reports the observed angle by eye (e.g. against the Map1
T-junction's printed right angle, or any straightedge). Prints a
duration -> angle -> deg/s table at the end.

Deliberately measured on the actual paper surface the robot drives on:
friction differs between the Task-1 print and other floors, so a rate
measured elsewhere (`examples/23_cam_spin_rate_check.py`, done on a textured wall
for the camera feature-matcher) would not transfer reliably here.

**Motor-moving. Operator must stand beside the car able to cut main power instantly.**

Usage:
    PYTHONPATH=src python3 examples/41_motor_spin_angle_sweep.py --speed 150
    PYTHONPATH=src python3 examples/41_motor_spin_angle_sweep.py --speed 150 --durations 2,4,6,8,10
"""

from __future__ import annotations

import argparse
import sys


def parse_durations(text: str) -> list[float]:
    return [float(p) for p in text.split(",") if p.strip()]


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Spin duration -> angle sweep, measured on the real track paper"
    )
    parser.add_argument("--speed", type=int, default=150, help="spin speed 0-1000 (default 150)")
    parser.add_argument(
        "--durations",
        default="2,4,6,8,10",
        help="comma-separated spin durations in seconds (default 2,4,6,8,10)",
    )
    parser.add_argument(
        "--direction", choices=("right", "left"), default="right", help="spin direction"
    )
    args = parser.parse_args()
    durations = parse_durations(args.durations)
    if not durations:
        print("--durations must list at least one value")
        return 2

    print("=" * 64)
    print("Spin Angle Sweep — duration vs. real angle, on the track paper")
    print("=" * 64)
    print(f"Speed: {args.speed} | Direction: {args.direction} | Durations: {durations}")
    print()

    answer = input("Operator beside car, ready to start the sweep? (yes/no) ").strip()
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

    results: list[tuple[float, float]] = []
    try:
        for duration_s in durations:
            print()
            print("-" * 64)
            print(f"Next: spin {args.direction} for {duration_s:.2f}s at speed {speed}")
            ready = input("Car reset to the reference heading, ready? (yes/skip/stop) ").strip().lower()
            if ready == "stop":
                break
            if ready == "skip":
                continue
            car.move_for(duration_s, left, right)
            angle_str = input(
                "Observed angle turned, in degrees (e.g. 90; blank to discard this reading): "
            ).strip()
            if not angle_str:
                print("  discarded")
                continue
            try:
                angle_deg = float(angle_str)
            except ValueError:
                print("  could not parse, discarded")
                continue
            results.append((duration_s, angle_deg))
    finally:
        car.close()

    print()
    print("=" * 64)
    print("Results")
    print("=" * 64)
    print(f"{'duration_s':>10} | {'angle_deg':>9} | {'deg_per_s':>9}")
    for duration_s, angle_deg in results:
        rate = angle_deg / duration_s if duration_s > 0 else 0.0
        print(f"{duration_s:>10.2f} | {angle_deg:>9.1f} | {rate:>9.1f}")

    if results:
        avg_rate = sum(a / d for d, a in results if d > 0) / len(results)
        print()
        print(f"Average deg/s across all readings: {avg_rate:.1f}")
        print(
            "Note: short durations include the motor's startup dead time and read a "
            "lower deg/s than longer ones; weight the longer-duration rows more when "
            "picking a final number."
        )

    return 0


if __name__ == "__main__":
    sys.exit(main())

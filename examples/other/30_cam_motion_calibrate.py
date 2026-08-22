#!/usr/bin/env python3
"""Calibrate the time-based motion model for route-prediction navigation.

This chassis has no Hall encoders (``config.HAS_ENCODERS = False``), so route
prediction integrates wall-clock time against calibrated speeds. Measure
them on the real car:

    # 1) straight-line speed: car drives 1.0 s at --speed, operator measures
    #    how far it travelled (tape measure on the floor)
    PYTHONPATH=src python3 examples/other/30_cam_motion_calibrate.py --mode forward --seconds 1.0

    # 2) spin rate: car spins in place 1.0 s at --speed, operator measures
    #    the heading change (phone compass or a reference line on the floor)
    PYTHONPATH=src python3 examples/other/30_cam_motion_calibrate.py --mode spin

**Motor-moving. An operator must stand beside the car able to cut main power
instantly; the wheels must be able to move freely on a clear floor.**

Report the measured distance (cm) and angle (deg) back into the constants in
`carbot/motion.py`, or pass --forward-mps/--spin-degps to example 29.
"""

from __future__ import annotations

import argparse
import sys
import time

from carbot import Car, NeZhaError


def main() -> int:
    parser = argparse.ArgumentParser(description="Calibrate motion model speeds")
    parser.add_argument("--mode", choices=("forward", "spin"), default="forward")
    parser.add_argument(
        "--seconds", type=float, default=1.0, help="drive duration for the measurement"
    )
    parser.add_argument("--speed", type=int, default=200, help="motor speed for the measurement")
    parser.add_argument(
        "--direction", choices=("left", "right"), default="right", help="spin direction (spin mode)"
    )
    args = parser.parse_args()

    if not args.seconds or args.seconds <= 0:
        print("--seconds must be positive")
        return 1

    answer = input("Operator beside the car, clear floor, power ready to cut? (yes/no) ").strip()
    if answer.lower() != "yes":
        print("Re-run when an operator is ready beside the car.")
        return 1

    try:
        with Car() as car:
            print(
                f"\nDriving {args.mode} at speed {args.speed} for "
                f"{args.seconds:.2f}s — measure now..."
            )
            if args.mode == "forward":
                car.forward(args.speed)
            elif args.direction == "left":
                car.spin_left(args.speed)
            else:
                car.spin_right(args.speed)
            time.sleep(args.seconds)
            car.stop()
    except NeZhaError as exc:
        print(f"Connection failed: {exc}")
        print("Run `examples/other/01_i2c_probe.py` first to debug the link.")
        return 1

    print("\nMeasurement done. Report the result:")
    if args.mode == "forward":
        print(f"  distance travelled (cm) -> forward_mps = distance_cm / 100 / {args.seconds:.2f}")
    else:
        print(f"  heading change (deg) -> spin_degps = angle_deg / {args.seconds:.2f}")
    print(
        "Update the constants in src/carbot/motion.py or pass the values to "
        "examples/ai_camera/29_cam_route_nav_drive.py with --forward-mps / --spin-degps."
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())

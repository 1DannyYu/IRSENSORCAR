#!/usr/bin/env python3
"""Calibrate drive motion for the M3 mapping loop using the HC-SR04.

Two calibrations:

1. **Forward speed** (cm/s at a given --speed): place a flat board/wall in
   front of the car (~20-60 cm away). The script measures the distance, drives
   forward for ``--seconds``, measures again; the difference / time is the
   linear speed. Runs forward then backward and averages.

2. **Spin duration** (s per revolution at a given --spin-speed): the script
   spins the car right for ``--spin-seconds``. The verified spin360 for speed
   150 on this build is ~8.2 s, so ``--spin-speed 150 --spin-seconds 8.2`` is
   one full revolution. If you change the spin speed, re-measure the
   revolution time and pass it back — the 8.2 s value is *only* valid at
   speed 150.

Safety: the car is constructed only after the operator confirms; every exit
path (refused confirmation, connection error, exception, Ctrl-C, normal end)
stops the motors, closes the board and cleans up GPIO.

Run on the Raspberry Pi with the car LIFTED or on a clear floor, operator
beside it able to cut power:

    PYTHONPATH=src python3 examples/10_sonar_motion_calibrate.py --speed 200
"""

from __future__ import annotations

import argparse
import statistics
import sys
import time

from RPi import GPIO

from carbot.sonar import Sonar

TRIG_PIN = 17  # GPIO 17 (Pin 11)
ECHO_PIN = 27  # GPIO 27 (Pin 13)
VERIFIED_SPIN_SPEED = 150
VERIFIED_SPIN_360_S = 8.2  # seconds per revolution at VERIFIED_SPIN_SPEED on this build


def avg_distance(sonar: Sonar, n: int = 5) -> float:
    vals = [d for d in (sonar.measure() for _ in range(n)) if d is not None]
    if not vals:
        raise RuntimeError("HC-SR04 never responded — check wiring (Pin 2/9/11/13 + divider)")
    return statistics.mean(vals)


def main() -> int:
    parser = argparse.ArgumentParser(description="Calibrate drive motion with the HC-SR04")
    parser.add_argument("--speed", type=int, default=200, help="forward drive speed 0-255")
    parser.add_argument("--seconds", type=float, default=2.0, help="forward drive time per leg")
    parser.add_argument(
        "--spin-speed",
        type=int,
        default=VERIFIED_SPIN_SPEED,
        help=f"spin speed 0-255 (spin360 {VERIFIED_SPIN_360_S} s only verified at "
        f"{VERIFIED_SPIN_SPEED})",
    )
    parser.add_argument(
        "--spin-seconds",
        type=float,
        default=VERIFIED_SPIN_360_S,
        help="spin duration per leg (one full turn at the verified spin speed)",
    )
    parser.add_argument("--reps", type=int, default=2, help="forward/backward repetitions")
    args = parser.parse_args()

    GPIO.setmode(GPIO.BCM)
    GPIO.setup(TRIG_PIN, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(ECHO_PIN, GPIO.IN)
    sonar = Sonar(TRIG_PIN, ECHO_PIN, GPIO)

    answer = input(
        "Board in front of the car (20-60 cm), car lifted or clear floor? (yes/no) "
    ).strip()
    if answer.lower() != "yes":
        print("Place a flat board in front of the car and re-run.")
        GPIO.cleanup()
        return 1

    from carbot import Car, NeZhaError

    try:
        car = Car()
    except NeZhaError as exc:
        print(f"Connection failed: {exc}")
        print("Run `examples/01_i2c_probe.py` first.")
        GPIO.cleanup()
        return 1

    # --- forward speed: (distance before - after) / time, average over reps ---
    speeds: list[float] = []
    vb = 0.0
    try:
        for rep in range(1, args.reps + 1):
            d1 = avg_distance(sonar)
            car.forward(args.speed)
            time.sleep(args.seconds)
            car.stop()
            time.sleep(0.5)
            d2 = avg_distance(sonar)
            v = (d1 - d2) / args.seconds
            speeds.append(v)
            print(f"rep {rep}: d1={d1:.1f} cm -> d2={d2:.1f} cm, forward speed {v:.1f} cm/s")
        # backward leg to return near the start
        d3 = avg_distance(sonar)
        car.backward(args.speed)
        time.sleep(args.seconds)
        car.stop()
        time.sleep(0.5)
        d4 = avg_distance(sonar)
        vb = (d4 - d3) / args.seconds
        print(f"back: d3={d3:.1f} cm -> d4={d4:.1f} cm, backward speed {vb:.1f} cm/s")

        # --- spin duration: one timed spin at the chosen spin speed ---
        print(
            f"\nSpin calibration: spinning right for {args.spin_seconds:.1f} s at "
            f"speed {args.spin_speed}..."
        )
        car.spin_right(args.spin_speed)
        time.sleep(args.spin_seconds)
        car.stop()
        if args.spin_speed == VERIFIED_SPIN_SPEED:
            note = f"(one full turn at the verified {VERIFIED_SPIN_360_S} s/rev)"
        else:
            note = (
                f"NOTE: {VERIFIED_SPIN_360_S} s/rev was measured at speed "
                f"{VERIFIED_SPIN_SPEED}; time the actual revolution at speed "
                f"{args.spin_speed} before using it for angle conversion"
            )
        print(f"  spun for {args.spin_seconds:.1f} s {note}")

    finally:
        car.stop()
        car.close()
        GPIO.cleanup()

    fwd = statistics.mean(speeds)
    print("\n=== RESULTS ===")
    print(f"forward speed @ {args.speed}: {fwd:.1f} cm/s")
    print(f"backward speed @ {args.speed}: {vb:.1f} cm/s")
    print(f"spin360 @ {args.spin_speed}: {args.spin_seconds:.1f} s (as configured)")
    print(f"To use in code: MOVEMENT_CM_PER_S = {{{args.speed}: {fwd:.1f}}}  # from calibration")
    return 0


if __name__ == "__main__":
    sys.exit(main())

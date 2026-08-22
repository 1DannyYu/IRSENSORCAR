#!/usr/bin/env python3
"""Wall-following patrol + capture for Structure-from-Motion.

Follows a wall with a single forward HC-SR04 (the classic technique, e.g.
"Wall Following with a Single Ultrasonic Sensor", Springer 2010): the car
drives with its nose angled toward the wall, so the forward sonar reads the
*slant* distance ``d``. The perpendicular distance to the wall is
``d * sin(angle)``; the car steers to hold that at --target-cm and thereby
rides along the wall. When the slant distance opens up past --corner-cm the
car has reached a corner, so it turns ~90 deg to pick up the next wall.

This covers the room perimeter (exactly the walls SfM needs) and, because it
steers along the wall, does not drive into corners. An operator must stand
beside the car able to cut main power instantly (motor-moving).

Run on the Pi:

    PYTHONPATH=src python3 examples/other/18_sonar_wall_follow_capture.py --frames 150

Assumes the wall is on the car's **right** side; the nose is angled toward
the wall by --approach-angle-deg (default 45 deg).

Output: one frame-NNN.jpg per capture under --out-dir (default /tmp/room-sfm).
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

from RPi import GPIO

from carbot.sonar import Sonar

TRIG_PIN = 17  # GPIO 17 (Pin 11)
ECHO_PIN = 27  # GPIO 27 (Pin 13)
VERIFIED_SPIN_DEG_PER_S = 360.0 / 8.2  # ~43.9 deg/s at speed 150


def read_distance(sonar: Sonar, trials: int = 3) -> float | None:
    """Nearest of ``trials`` readings, or None when none return a value."""
    vals = [d for d in (sonar.measure() for _ in range(trials)) if d is not None]
    return min(vals) if vals else None


def spin_seconds(angle_deg: float, deg_per_s: float) -> float:
    return angle_deg / deg_per_s


def main() -> int:
    parser = argparse.ArgumentParser(description="Wall-follow patrol + capture for SfM")
    parser.add_argument("--frames", type=int, default=150, help="number of stills to capture")
    parser.add_argument("--speed", type=int, default=200, help="drive speed 0-255")
    parser.add_argument("--step-s", type=float, default=0.6, help="seconds per forward step")
    parser.add_argument(
        "--target-cm", type=float, default=25.0, help="perpendicular distance to hold from the wall"
    )
    parser.add_argument(
        "--approach-angle-deg",
        type=float,
        default=45.0,
        help="nose angle toward the wall while following",
    )
    parser.add_argument(
        "--corner-cm",
        type=float,
        default=80.0,
        help="slant distance above which a corner is assumed",
    )
    parser.add_argument(
        "--find-cm",
        type=float,
        default=40.0,
        help="slant distance at which the car has reached a wall",
    )
    parser.add_argument(
        "--deadband-cm",
        type=float,
        default=8.0,
        help="perpendicular error deadband before steering",
    )
    parser.add_argument(
        "--turn-ratio", type=float, default=0.5, help="inside-wheel ratio for arc steering (0..1)"
    )
    parser.add_argument(
        "--spin-deg-per-s",
        type=float,
        default=VERIFIED_SPIN_DEG_PER_S,
        help="spin rate (deg/s) at --speed",
    )
    parser.add_argument("--size", default="2028x1520", help="capture size WxH")
    parser.add_argument("--out-dir", type=Path, default=Path("/tmp/room-sfm"))
    args = parser.parse_args()

    width, height = (int(v) for v in args.size.split("x"))
    sin_angle = math.sin(math.radians(args.approach_angle_deg))

    GPIO.setmode(GPIO.BCM)
    GPIO.setup(TRIG_PIN, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(ECHO_PIN, GPIO.IN)
    sonar = Sonar(TRIG_PIN, ECHO_PIN, GPIO)

    answer = input("Operator beside the car, path clear, power ready to cut? (yes/no) ").strip()
    if answer.lower() != "yes":
        print("Re-run when an operator is ready beside the car.")
        GPIO.cleanup()
        return 1

    from carbot import Car, NeZhaError

    try:
        car = Car()
    except NeZhaError as exc:
        print(f"Connection failed: {exc}")
        print("Run `examples/other/01_i2c_probe.py` first to debug the link.")
        GPIO.cleanup()
        return 1

    try:
        from picamera2 import Picamera2
    except ImportError as exc:
        print(f"Picamera2 required: {exc}")
        car.close()
        GPIO.cleanup()
        return 1

    camera = Picamera2()
    try:
        config = camera.create_still_configuration(main={"size": (width, height)})
        camera.configure(config)
        camera.start()
        time.sleep(1.5)
    except Exception:
        camera.close()
        car.close()
        GPIO.cleanup()
        raise

    args.out_dir.mkdir(parents=True, exist_ok=True)
    print(
        f"Wall following: {args.frames} frames, target {args.target_cm:.0f} cm, "
        f"nose angle {args.approach_angle_deg:.0f} deg, corner > {args.corner_cm:.0f} cm. "
        f"Ctrl-C to stop."
    )

    n = 0
    try:
        # --- Initialization: drive forward until a wall is found, then angle
        # the nose toward it (wall assumed on the right side).
        d = read_distance(sonar)
        while d is None or d > args.find_cm:
            if n >= args.frames:
                break
            car.forward(args.speed)
            time.sleep(args.step_s)
            car.stop()
            time.sleep(0.5)
            d = read_distance(sonar)
            path = args.out_dir / f"frame-{n:03d}.jpg"
            camera.capture_file(str(path))
            print(f"[{n + 1}] approach: {path.name}")
            n += 1
        # nose toward the wall: turn right (wall on the right) by approach angle
        car.spin_right(args.speed)
        time.sleep(spin_seconds(args.approach_angle_deg, args.spin_deg_per_s))
        car.stop()
        time.sleep(0.5)
        print("  reached wall; nose angled toward it, beginning wall follow")

        while n < args.frames:
            d = read_distance(sonar)
            if d is None or d > args.corner_cm:
                # Corner / lost the wall: turn ~90 deg right to find the next wall.
                print(f"[{n + 1}] corner/open (d={d}) -> turn right 90 deg")
                car.spin_right(args.speed)
                time.sleep(spin_seconds(90.0, args.spin_deg_per_s))
                car.stop()
                time.sleep(0.4)
                # re-approach the next wall
                d = read_distance(sonar)
                while d is None or d > args.find_cm:
                    if n >= args.frames:
                        break
                    car.forward(args.speed)
                    time.sleep(args.step_s)
                    car.stop()
                    time.sleep(0.5)
                    d = read_distance(sonar)
                    path = args.out_dir / f"frame-{n:03d}.jpg"
                    camera.capture_file(str(path))
                    print(f"[{n + 1}] re-approach: {path.name}")
                    n += 1
                car.spin_right(args.speed)
                time.sleep(spin_seconds(args.approach_angle_deg, args.spin_deg_per_s))
                car.stop()
                time.sleep(0.4)
                continue

            # Follow the wall: hold the perpendicular distance.
            perp = d * sin_angle
            error = perp - args.target_cm
            if error > args.deadband_cm:
                car.turn_right(args.speed, ratio=args.turn_ratio)  # too far -> steer toward wall
            elif error < -args.deadband_cm:
                car.turn_left(args.speed, ratio=args.turn_ratio)  # too close -> steer away
            else:
                car.forward(args.speed)
            time.sleep(args.step_s)
            car.stop()
            time.sleep(0.8)  # settle so the still is sharp

            path = args.out_dir / f"frame-{n:03d}.jpg"
            camera.capture_file(str(path))
            print(f"[{n + 1}] d={d:.0f}cm perp={perp:.0f}cm -> {path.name}")
            n += 1
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        car.stop()
        camera.stop()
        camera.close()
        car.close()
        GPIO.cleanup()

    print(f"Captured {n} frames -> {args.out_dir}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

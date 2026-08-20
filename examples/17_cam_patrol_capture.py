#!/usr/bin/env python3
"""Autonomous patrol + capture for Structure-from-Motion (Roomba-style).

The car drives **itself** around the room using a random-bounce strategy
(the same one robot vacuums have proven for decades): drive forward until an
obstacle is near, then turn by a **random angle** (30-150 deg) in a random
direction and continue. Over a long run this probabilistically covers the
whole room, which is exactly what a photo sweep for SfM needs — no encoders,
no map, just overlapping viewpoints.

An operator must stand beside the car able to cut main power instantly
(motor-moving). Run on the Pi:

    PYTHONPATH=src python3 examples/17_cam_patrol_capture.py --frames 150

Loop per frame: sample the sonar (keep nearest of 3); if ``None`` (HC-SR04
<~20 cm blind zone or fault) or closer than --obstacle-cm, spin a random angle
in a random direction; otherwise drive forward a short step. A still is
captured after every move/stop. A missing reading is treated as unsafe, never
as "clear".

Spin timing uses --spin-deg-per-s (default = the verified 8.2 s/360 deg at
speed 150); at a higher --speed the car turns faster, so the actual turn
angle is approximate — fine for random coverage.

Output: one frame-NNN.jpg per capture under --out-dir (default /tmp/room-sfm).
"""

from __future__ import annotations

import argparse
import random
import sys
import time
from pathlib import Path

from RPi import GPIO

from carbot.sonar import Sonar

TRIG_PIN = 17  # GPIO 17 (Pin 11)
ECHO_PIN = 27  # GPIO 27 (Pin 13)
VERIFIED_SPIN_DEG_PER_S = 360.0 / 8.2  # ~43.9 deg/s at speed 150


def read_distance(sonar: Sonar, trials: int = 3) -> float | None:
    """Nearest of ``trials`` readings, or None when none return a value.

    A None result means "cannot confirm clear" (blind zone or fault); callers
    must treat it as an obstacle, never as free space.
    """
    vals = [d for d in (sonar.measure() for _ in range(trials)) if d is not None]
    return min(vals) if vals else None


def main() -> int:
    parser = argparse.ArgumentParser(description="Roomba-style patrol + capture for SfM")
    parser.add_argument("--frames", type=int, default=150, help="number of stills to capture")
    parser.add_argument("--step-s", type=float, default=1.0, help="seconds of forward per step")
    parser.add_argument("--speed", type=int, default=200, help="drive speed 0-255 (low)")
    parser.add_argument(
        "--backup-s",
        type=float,
        default=0.6,
        help="seconds to reverse before turning (frees the car from a corner)",
    )
    parser.add_argument(
        "--obstacle-cm",
        type=float,
        default=30.0,
        help="turn away when the sonar reads closer than this",
    )
    parser.add_argument(
        "--turn-min-deg", type=float, default=30.0, help="minimum random turn angle (deg)"
    )
    parser.add_argument(
        "--turn-max-deg", type=float, default=150.0, help="maximum random turn angle (deg)"
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
        print("Run `examples/01_i2c_probe.py` first to debug the link.")
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
        f"Roomba patrol: {args.frames} frames, step {args.step_s}s at speed {args.speed}, "
        f"turn below {args.obstacle_cm:.0f} cm by {args.turn_min_deg:.0f}-"
        f"{args.turn_max_deg:.0f} deg random angle. Ctrl-C to stop."
    )
    n = 0
    try:
        while n < args.frames:
            d = read_distance(sonar)
            if d is None or d < args.obstacle_cm:
                # Obstacle / blind zone: back up to leave the wall/corner, then
                # turn a random angle in a random direction.
                angle = random.uniform(args.turn_min_deg, args.turn_max_deg)
                spin_s = angle / args.spin_deg_per_s
                why = f"{d:.0f} cm" if d is not None else "no reading (blind zone)"
                direction = "left" if random.random() < 0.5 else "right"
                print(f"[{n + 1}] obstacle: {why} -> back up, then {direction} {angle:.0f} deg")
                car.backward(args.speed)
                time.sleep(args.backup_s)
                car.stop()
                time.sleep(0.3)
                if direction == "left":
                    car.spin_left(args.speed)
                else:
                    car.spin_right(args.speed)
                time.sleep(spin_s)
                car.stop()
                time.sleep(0.8)  # settle so the still is sharp
            else:
                # Clear: advance one short step, stop, then shoot.
                car.forward(args.speed)
                time.sleep(args.step_s)
                car.stop()
                time.sleep(1.0)  # settle so the still is sharp

            path = args.out_dir / f"frame-{n:03d}.jpg"
            camera.capture_file(str(path))
            print(f"[{n + 1}] {path.name}")
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

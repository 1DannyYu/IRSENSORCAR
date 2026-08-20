#!/usr/bin/env python3
"""Closed-loop obstacle avoidance: HC-SR04 distance drives the car.

⚠️ Requires an operator beside the car who can cut main power. By default the
script verifies the car is LIFTED (all wheels off the ground); pass --ground
for a real floor run after the wheel-mapping tests pass.

Run on the Raspberry Pi (system interpreter, not uv):

    PYTHONPATH=src python3 examples/07_sonar_avoidance_drive.py --dry-run
    PYTHONPATH=src python3 examples/07_sonar_avoidance_drive.py --duration 60
    PYTHONPATH=src python3 examples/07_sonar_avoidance_drive.py --ground --duration 60

Logic per loop:
  distance > threshold      -> drive forward at --speed
  distance <= threshold     -> stop, spin right 1.2 s, re-measure
  no echo (wiring problem)  -> stop and report

Wiring (verified in this build): VCC=Pin 2, GND=Pin 9, TRIG=Pin 11 (GPIO 17),
ECHO=Pin 13 (GPIO 27) via a 2.2k/1k divider. See
docs/hardware/hc-sr04-ultrasonic-sensor.md.
"""

from __future__ import annotations

import argparse
import sys
import time

TRIG_PIN = 17  # GPIO 17 (Pin 11)
ECHO_PIN = 27  # GPIO 27 (Pin 13)
SPEED_OF_SOUND = 34300.0  # cm/s
SPIN_S = 1.2  # seconds of spin when an obstacle is detected


def measure_once(timeout_s: float = 0.5) -> float | None:
    """One TRIG/ECHO cycle; returns distance in cm or None on timeout."""
    from RPi import GPIO

    GPIO.output(TRIG_PIN, GPIO.LOW)
    time.sleep(0.06)
    GPIO.output(TRIG_PIN, GPIO.HIGH)
    time.sleep(0.00001)
    GPIO.output(TRIG_PIN, GPIO.LOW)

    t0 = time.time()
    while GPIO.input(ECHO_PIN) == GPIO.LOW:
        if time.time() - t0 > timeout_s:
            return None
    pulse_start = time.time()
    while GPIO.input(ECHO_PIN) == GPIO.HIGH:
        if time.time() - pulse_start > timeout_s:
            return None
    pulse_end = time.time()
    return (pulse_end - pulse_start) * SPEED_OF_SOUND / 2.0


def main() -> int:
    parser = argparse.ArgumentParser(description="Closed-loop obstacle avoidance demo")
    parser.add_argument("--dry-run", action="store_true", help="sensor only — never drive motors")
    parser.add_argument(
        "--ground", action="store_true", help="car is on the floor (real avoidance run)"
    )
    parser.add_argument("--duration", type=float, default=30.0, help="run duration in seconds")
    parser.add_argument("--threshold", type=float, default=25.0, help="obstacle distance in cm")
    parser.add_argument("--speed", type=int, default=200, help="drive speed (0-255)")
    args = parser.parse_args()

    if not args.dry_run and not args.ground:
        answer = (
            input("Is the car LIFTED with all wheels off the ground? (yes/no) ").strip().lower()
        )
        if answer != "yes":
            print("Lift the car (or pass --ground) before running this test.")
            return 1

    from RPi import GPIO

    car = None
    if not args.dry_run:
        try:
            from carbot import Car, NeZhaError

            car = Car()
        except NeZhaError as exc:
            print(f"Connection failed: {exc}")
            print("Run `examples/01_i2c_probe.py` first to debug the link.")
            return 1

    GPIO.setmode(GPIO.BCM)
    GPIO.setup(TRIG_PIN, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(ECHO_PIN, GPIO.IN)

    mode = "DRY-RUN (sensor only)" if args.dry_run else ("GROUND" if args.ground else "LIFTED")
    print(
        f"Obstacle avoidance — mode: {mode}, threshold {args.threshold:.0f} cm, "
        f"duration {args.duration:.0f} s"
    )
    print("=" * 60)

    start = time.time()
    loop = 0
    try:
        while time.time() - start < args.duration:
            loop += 1
            d = measure_once()
            if d is None:
                print(f"[{loop}] NO ECHO — stop and check TRIG/ECHO wiring + divider")
                if car:
                    car.stop()
                break
            if d > args.threshold:
                print(f"[{loop}] {d:6.1f} cm — clear, forward")
                if car:
                    car.forward(args.speed)
            else:
                print(f"[{loop}] {d:6.1f} cm — OBSTACLE, stop + spin right")
                if car:
                    car.stop()
                    time.sleep(0.3)
                    car.spin_right(args.speed)
                    time.sleep(SPIN_S)
                    car.stop()
                else:
                    time.sleep(0.3)
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        GPIO.cleanup()
        if car:
            car.stop()
            car.close()

    print("-" * 60)
    print(
        f"Finished {loop} loop(s). "
        f"{'Dry-run OK — sensor loop responded.' if args.dry_run else 'Stop confirmed.'}"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())

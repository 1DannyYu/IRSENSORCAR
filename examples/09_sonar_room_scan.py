#!/usr/bin/env python3
"""Room spin-scan: log HC-SR04 distance vs. time while the car spins in place.

This is M1 of the room-mapping prototype — capture the polar distance profile
of the space around the car. Run with the car LIFTED (or on the floor with an
operator beside it, able to cut power):

    PYTHONPATH=src python3 examples/09_sonar_room_scan.py --scan 20 --interval 0.15

Output: /tmp/room_scan.csv with columns elapsed_s, distance_cm, spin360_s.

The scan is one frame of the incremental mapping loop in
``src/carbot/mapping.py`` (ICP + occupancy grid). The recorded ``spin360_s``
column is the *configured* revolution time and travels with the capture; angle
conversion later uses that column (``carbot.frames.scan_angle_rad``), never a
global constant. Spin rate is verified at ~8.2 s per full turn at speed 150 on
this build — pass ``--spin-360`` if you change the speed or drive, and
re-measure the revolution time at that speed first.

The car is constructed only after the operator confirms; the spin stops and
the board closes on every exit path.

Wiring: HC-SR04 VCC=Pin 2, GND=Pin 9, TRIG=Pin 11 (GPIO 17), ECHO=Pin 13
(GPIO 27) via a 2.2k/1k divider. See docs/hardware/hc-sr04-ultrasonic-sensor.md.
"""

from __future__ import annotations

import argparse
import csv
import sys
import time

from RPi import GPIO

from carbot.sonar import Sonar

TRIG_PIN = 17  # GPIO 17 (Pin 11)
ECHO_PIN = 27  # GPIO 27 (Pin 13)
VERIFIED_SPIN_SPEED = 150
VERIFIED_SPIN_360_S = 8.2  # seconds per revolution at VERIFIED_SPIN_SPEED on this build


def main() -> int:
    parser = argparse.ArgumentParser(description="Spin-scan the room with the HC-SR04")
    parser.add_argument("--scan", type=float, default=20.0, help="scan duration in seconds")
    parser.add_argument("--interval", type=float, default=0.15, help="seconds between readings")
    parser.add_argument(
        "--spin-360",
        type=float,
        default=VERIFIED_SPIN_360_S,
        help=f"estimated seconds per full spin at --speed (verified "
        f"{VERIFIED_SPIN_360_S} at speed {VERIFIED_SPIN_SPEED})",
    )
    parser.add_argument(
        "--speed",
        type=int,
        default=VERIFIED_SPIN_SPEED,
        help="spin speed 0-255 (revolution time must be re-measured if changed from the default)",
    )
    parser.add_argument("--out", default="/tmp/room_scan.csv", help="output CSV path")
    args = parser.parse_args()

    GPIO.setmode(GPIO.BCM)
    GPIO.setup(TRIG_PIN, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(ECHO_PIN, GPIO.IN)
    sonar = Sonar(TRIG_PIN, ECHO_PIN, GPIO)

    answer = input(
        "Operator beside the car, wheels lifted or floor clear, power ready to cut? (yes/no) "
    ).strip()
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

    rows: list[tuple[float, float]] = []
    print(f"Scanning {args.scan:.0f}s at {args.interval}s interval, spin speed {args.speed}...")
    car.spin_right(args.speed)
    t_start = time.monotonic()
    try:
        while time.monotonic() - t_start < args.scan:
            d = sonar.measure()
            elapsed = time.monotonic() - t_start
            if d is not None:
                rows.append((elapsed, d))
                print(f"  t={elapsed:5.1f}s  d={d:6.1f} cm")
            time.sleep(args.interval)
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        car.stop()
        car.close()
        GPIO.cleanup()

    with open(args.out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["elapsed_s", "distance_cm", "spin360_s"])
        for elapsed, d in rows:
            w.writerow([f"{elapsed:.2f}", f"{d:.1f}", args.spin_360])
    print(f"Saved {len(rows)} readings -> {args.out}")
    if rows:
        dists = [d for _, d in rows]
        print(
            f"min {min(dists):.1f} cm, max {max(dists):.1f} cm, "
            f"avg {sum(dists) / len(dists):.1f} cm"
        )
    return 0


if __name__ == "__main__":
    sys.exit(main())

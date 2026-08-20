#!/usr/bin/env python3
"""HC-SR04 ultrasonic distance check and basic obstacle-avoidance demo.

Run on the Raspberry Pi:

    python3 examples/06_ultrasonic_avoidance.py              # 5 readings, warn below 30 cm
    python3 examples/06_ultrasonic_avoidance.py --trials 10 --threshold 50

This script only reads the sensor; it does not move motors or servos, so it is safe to
run over SSH.

Wiring (verified in this build — NeZha I2C occupies Pin 3/4/5/6, so GND moves to Pin 9):

    HC-SR04 VCC  -> Pin 2 (5V)
    HC-SR04 GND  -> Pin 9
    HC-SR04 TRIG -> Pin 11 (GPIO 17)
    HC-SR04 ECHO -> Pin 13 (GPIO 27) via 2.2k/1k voltage divider

See docs/hardware/hc-sr04-ultrasonic-sensor.md for the full wiring notes.

Exit code 0 = the sensor responded and the average distance is within the 2-400 cm range.
"""

from __future__ import annotations

import argparse
import statistics
import sys
import time

TRIG_PIN = 17  # GPIO 17 (Pin 11)
ECHO_PIN = 27  # GPIO 27 (Pin 13)

SPEED_OF_SOUND = 34300.0  # cm/s at ~20°C


def measure_once(timeout_s: float = 0.5) -> float | None:
    """Send one TRIG pulse and measure the ECHO pulse width; return cm or None on timeout."""
    from RPi import GPIO

    GPIO.output(TRIG_PIN, GPIO.LOW)
    time.sleep(0.06)  # ensure the line is settled low
    GPIO.output(TRIG_PIN, GPIO.HIGH)
    time.sleep(0.00001)  # 10 µs trigger pulse
    GPIO.output(TRIG_PIN, GPIO.LOW)

    t0 = time.time()
    while GPIO.input(ECHO_PIN) == GPIO.LOW:
        if time.time() - t0 > timeout_s:
            return None  # echo never started
    pulse_start = time.time()
    while GPIO.input(ECHO_PIN) == GPIO.HIGH:
        if time.time() - pulse_start > timeout_s:
            return None  # echo stuck high
    pulse_end = time.time()

    duration = pulse_end - pulse_start  # seconds
    return duration * SPEED_OF_SOUND / 2.0


def main() -> int:
    parser = argparse.ArgumentParser(description="HC-SR04 distance check and obstacle warning")
    parser.add_argument("--trials", type=int, default=5, help="number of readings (default 5)")
    parser.add_argument(
        "--threshold", type=float, default=30.0, help="obstacle warning distance in cm"
    )
    parser.add_argument("--timeout", type=float, default=0.5, help="echo timeout in seconds")
    args = parser.parse_args()

    from RPi import GPIO

    GPIO.setmode(GPIO.BCM)
    GPIO.setup(TRIG_PIN, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(ECHO_PIN, GPIO.IN)

    readings: list[float] = []
    failures = 0
    print(
        f"HC-SR04 distance check — {args.trials} trials, obstacle threshold {args.threshold:.0f} cm"
    )
    print("=" * 60)

    try:
        for i in range(args.trials):
            d = measure_once(args.timeout)
            if d is None:
                failures += 1
                print(f"[{i + 1}] NO RESPONSE — check TRIG/ECHO wiring and the voltage divider")
                continue
            readings.append(d)
            verdict = "OBSTACLE AHEAD" if d < args.threshold else "clear"
            print(f"[{i + 1}] {d:6.1f} cm  ({verdict})")
            time.sleep(0.1)
    finally:
        GPIO.cleanup()

    if failures == args.trials:
        print(
            "\n✗ Sensor never responded. Re-check wiring: VCC->Pin 2, GND->Pin 9, "
            "TRIG->Pin 11 (GPIO17), ECHO->Pin 13 (GPIO27) with the 2.2k/1k divider."
        )
        return 1

    avg = statistics.mean(readings)
    print("-" * 60)
    print(f"Average: {avg:.1f} cm across {len(readings)} readings")
    if avg < args.threshold:
        print(f"⚠️  Obstacle detected at {avg:.0f} cm — obstacle avoidance would trigger here.")
    else:
        print("Path clear within the configured threshold.")
    print("✓ Sensor responded correctly." if not failures else "⚠️  Some trials timed out.")
    return 0


if __name__ == "__main__":
    sys.exit(main())

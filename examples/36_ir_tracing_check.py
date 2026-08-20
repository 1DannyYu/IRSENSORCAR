#!/usr/bin/env python3
"""Yahboom 4-channel IR tracing sensor check with a uniform 1=black / 0=white readout.

Stationary, **no motors** — safe to run over SSH with the car powered.

    # on the Pi, from the repo root
    PYTHONPATH=src python3 examples/36_ir_tracing_check.py
    PYTHONPATH=src python3 examples/36_ir_tracing_check.py --invert 0,3 --count 20

Hold the sensor 1-3 cm above the track (or put the car on a stand with the
sensor over the line) and watch the readout. The normalization is already
applied: every channel must report **1** over the black line and **0** over
white. If a channel shows 0 while over black (or 1 while over white), add its
channel index — Out1=0, Out2=1, Out3=2, Out4=3 — to ``--invert``.

Verified pins for this build: Out1-Out4 use GPIO 24/25/22/23 (Pins 18/22/15/16).
These avoid the HC-SR04 TRIG/ECHO pins (GPIO 17/27). Module powered from 3.3V.

See docs/hardware/ir-tracing-sensor.md for wiring and signal logic.
"""

from __future__ import annotations

import argparse
import sys
import time

from carbot.ir_tracing import IRTracingSensor

DEFAULT_PINS = (24, 25, 22, 23)  # Out1..Out4 — verified 2026-08-17


def parse_pins(text: str) -> tuple[int, ...]:
    """'17,27,22,23' -> (17, 27, 22, 23)."""
    return tuple(int(part) for part in text.split(",") if part.strip())


def parse_invert(text: str) -> set[int]:
    """'0,3' -> {0, 3}; empty string -> empty set."""
    if not text.strip():
        return set()
    return {int(part) for part in text.split(",") if part.strip()}


def main() -> int:
    parser = argparse.ArgumentParser(
        description="IR tracing sensor check — every channel reports 1=black, 0=white"
    )
    parser.add_argument(
        "--pins",
        default=",".join(str(p) for p in DEFAULT_PINS),
        help="BCM GPIOs for Out1..OutN, comma-separated (default: planned 17,27,22,23)",
    )
    parser.add_argument(
        "--invert",
        default="",
        help="channel indices whose raw polarity is opposite the default, e.g. 0,3",
    )
    parser.add_argument("--count", type=int, default=10, help="readings to take (default 10)")
    parser.add_argument("--interval", type=float, default=0.3, help="seconds between readings")
    args = parser.parse_args()

    pins = parse_pins(args.pins)
    invert = parse_invert(args.invert)
    if not pins:
        print("✗ --pins must list at least one GPIO number.")
        return 2

    from RPi import GPIO

    GPIO.setmode(GPIO.BCM)
    for pin in pins:
        GPIO.setup(pin, GPIO.IN)
    sensor = IRTracingSensor(pins, GPIO, invert=invert)

    print(
        "IR tracing check — 1=black, 0=white; Ctrl+C to stop. "
        "Channels: " + ", ".join(f"Out{i + 1}(GPIO{p})" for i, p in enumerate(pins))
    )
    print("=" * 64)
    try:
        for _ in range(args.count):
            raw = sensor.raw()
            norm = sensor.read()
            print(
                "raw   "
                + "  ".join(f"{r}" for r in raw)
                + "   |   normalized  "
                + "  ".join(f"{n}" for n in norm)
            )
            time.sleep(args.interval)
    except KeyboardInterrupt:
        pass
    finally:
        GPIO.cleanup()

    print("-" * 64)
    print(
        "Hold each channel over black and over white. Expected 1 on black / 0 on white.\n"
        "If any channel disagrees, re-run with its index added to --invert "
        f"(channels: Out1=0 ... Out{len(pins)}={len(pins) - 1})."
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())

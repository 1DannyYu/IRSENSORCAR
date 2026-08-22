#!/usr/bin/env python3
"""No-motion preflight: camera, I2C board, HC-SR04, power, and encoders.

This script never constructs :class:`carbot.Car` and never sends a motor or
servo command. It is safe to run over SSH with the robot powered. Run it
before any Gate C/D/E motion test:

    PYTHONPATH=src python3 examples/other/14_all_sensors_preflight_check.py

Checks:
  1. Camera — Picamera2 opens and closes (static capture path only).
  2. I2C / NeZha — board responds at 0x40 with ``init_motors=False``.
  3. HC-SR04 — a few distance readings over GPIO 17/27 (sensor only).
  4. Power — EXT5V_V and ``get_throttled`` via vcgencmd.
  5. Encoders — reports ``config.HAS_ENCODERS`` and reads channels when on.

Exit code 0 = everything OK; 1 = at least one check failed or warned.
"""

from __future__ import annotations

import subprocess
import sys
import time
from collections.abc import Callable

from carbot.config import HAS_ENCODERS
from carbot.nezha import DEFAULT_ADDRESS, DEFAULT_BUS, NeZhaError
from carbot.power import decode_throttled, parse_throttled_output

TRIG_PIN = 17  # GPIO 17 (Pin 11)
ECHO_PIN = 27  # GPIO 27 (Pin 13)
SONAR_TRIALS = 3
EXT5V_MIN = 4.8  # volts
TEMP_MAX = 80.0  # °C

Check = Callable[[], tuple[bool, str]]


def _check_camera() -> tuple[bool, str]:
    try:
        from picamera2 import Picamera2
    except ImportError as exc:
        return False, f"picamera2 not importable: {exc}"
    camera = Picamera2()
    try:
        config = camera.create_still_configuration(main={"size": (2028, 1520)})
        camera.configure(config)
        camera.start()
        time.sleep(1.0)  # allow the sensor to settle
    finally:
        camera.stop()
        camera.close()
    return True, "picamera2 opened and closed a still configuration"


def _check_i2c() -> tuple[bool, str]:
    from carbot.nezha import NeZha

    try:
        board = NeZha(init_motors=False)
    except NeZhaError as exc:
        return False, f"NeZha at 0x{DEFAULT_ADDRESS:02X} bus {DEFAULT_BUS} failed: {exc}"
    try:
        board.reset()
        return True, f"NeZha responded at 0x{DEFAULT_ADDRESS:02X} bus {DEFAULT_BUS} (reset OK)"
    except NeZhaError as exc:
        return False, f"NeZha reset command failed: {exc}"
    finally:
        board.close(stop_motors=False)


def _check_sonar() -> tuple[bool, str]:
    from RPi import GPIO

    from carbot.sonar import Sonar

    GPIO.setmode(GPIO.BCM)
    GPIO.setup(TRIG_PIN, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(ECHO_PIN, GPIO.IN)
    sonar = Sonar(TRIG_PIN, ECHO_PIN, GPIO)
    readings: list[float] = []
    try:
        for _ in range(SONAR_TRIALS):
            d = sonar.measure()
            if d is not None:
                readings.append(d)
            time.sleep(0.1)
    finally:
        GPIO.cleanup()
    if not readings:
        return False, "HC-SR04 never responded (check TRIG/ECHO wiring + divider)"
    avg = sum(readings) / len(readings)
    return True, f"HC-SR04 responded: {len(readings)}/{SONAR_TRIALS} readings, avg {avg:.1f} cm"


def _vcgencmd(*args: str) -> str:
    out = subprocess.run(
        ["vcgencmd", *args], capture_output=True, text=True, timeout=10, check=False
    )
    return out.stdout.strip()


def _check_power() -> tuple[bool, str]:
    problems = 0
    parts: list[str] = []
    raw = _vcgencmd("pmic_read_adc", "EXT5V_V")
    try:
        volt = float(raw.split("=")[1].replace("V", "").strip())
    except (IndexError, ValueError):
        parts.append(f"EXT5V_V unreadable ({raw!r})")
        problems += 1
    else:
        ok = volt >= EXT5V_MIN
        parts.append(f"EXT5V_V={volt:.3f} V ({'OK' if ok else 'LOW'})")
        problems += 0 if ok else 1
    throttled_raw = _vcgencmd("get_throttled")
    try:
        status = decode_throttled(parse_throttled_output(throttled_raw))
    except ValueError:
        parts.append(f"get_throttled unreadable ({throttled_raw!r})")
        problems += 1
    else:
        # Only the live half gates a motion test; the since-boot half stays set
        # until reboot and would block motor work forever after one power dip.
        parts.append(f"get_throttled={status.describe()}")
        problems += 1 if status.throttled_now else 0
    temp_raw = _vcgencmd("measure_temp")
    try:
        temp = float(temp_raw.split("=")[1].replace("'C", "").strip())
    except (IndexError, ValueError):
        parts.append(f"temperature unreadable ({temp_raw!r})")
        problems += 1
    else:
        ok = temp < TEMP_MAX
        parts.append(f"temp={temp:.1f} °C ({'OK' if ok else 'HOT'})")
        problems += 0 if ok else 1
    return problems == 0, "; ".join(parts)


def _check_encoders() -> tuple[bool, str]:
    if not HAS_ENCODERS:
        return True, "config.HAS_ENCODERS=False (two-wire motors; encoder reads expected 0)"
    from carbot.nezha import NeZha, NeZhaError

    try:
        board = NeZha(init_motors=False)
    except NeZhaError as exc:
        return False, f"cannot open board for encoder reads: {exc}"
    values: list[str] = []
    try:
        for n in (1, 2, 3, 4):
            board.init_encoder(n)
            try:
                values.append(f"M{n}={board.encoder(n)}")
            except NeZhaError as exc:
                values.append(f"M{n}=read-failed ({exc})")
    finally:
        board.close(stop_motors=False)
    return True, "encoders: " + ", ".join(values)


CHECKS: list[tuple[str, Check]] = [
    ("camera", _check_camera),
    ("i2c/nezha", _check_i2c),
    ("hc-sr04", _check_sonar),
    ("power", _check_power),
    ("encoders", _check_encoders),
]


def main() -> int:
    print("No-motion preflight — camera, I2C, HC-SR04, power, encoders")
    print("=" * 64)
    problems = 0
    for name, check in CHECKS:
        try:
            ok, message = check()
        except Exception as exc:  # noqa: BLE001 — report any check failure and continue
            ok, message = False, f"unexpected error: {exc!r}"
        mark = "OK" if ok else "FAIL"
        problems += 0 if ok else 1
        print(f"[{mark}] {name:12s} {message}")
    print("-" * 64)
    if problems:
        print(f"✗ {problems} check(s) failed — resolve before any motion test.")
        return 1
    print("✓ All preflight checks passed — safe to proceed to a supervised motion test.")
    return 0


if __name__ == "__main__":
    sys.exit(main())

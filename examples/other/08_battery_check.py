#!/usr/bin/env python3
"""Battery and power-health check for the Raspberry Pi 5 car build.

Run on the Raspberry Pi (safe over SSH, no moving parts):

    PYTHONPATH=src python3 examples/other/08_battery_check.py

Reads (all without sudo on Raspberry Pi OS Bookworm+):
  - EXT5V_V: the external 5V rail voltage (battery feeds the NeZha board, which
    feeds this rail via Pin 4). Expect >= 4.8V on this build.
  - get_throttled: live + since-boot undervoltage / throttling / soft-temp bits,
    decoded by :mod:`carbot.power`
  - measure_temp: SoC temperature

Exit code 0 = no active warning (EXT5V_V >= threshold and no live throttle bits set);
1 = at least one active problem detected. Since-boot flags print as [INFO], not a
failure: they stay set until reboot, so one power dip would otherwise fail this
check for the rest of the session.
"""

from __future__ import annotations

import subprocess
import sys

from carbot.power import decode_throttled, parse_throttled_output

EXT5V_MIN = 4.8  # volts; below this warn
TEMP_MAX = 80.0  # °C; soft limit is around 85°C on Pi 5


def vcgencmd(*args: str) -> str:
    out = subprocess.run(
        ["vcgencmd", *args], capture_output=True, text=True, timeout=10, check=False
    )
    return out.stdout.strip()


def main() -> int:
    problems = 0

    print("Battery / power-health check")
    print("=" * 40)

    # 1. External 5V rail (fed from the battery via the NeZha board)
    raw = vcgencmd("pmic_read_adc", "EXT5V_V")
    volt = None
    try:
        volt = float(raw.split("=")[1].replace("V", "").strip())
    except (IndexError, ValueError):
        print(f"[FAIL] could not parse EXT5V_V: {raw!r}")
        problems += 1
    else:
        ok = volt >= EXT5V_MIN
        mark = "OK" if ok else "LOW"
        print(f"[{mark}] EXT5V_V = {volt:.3f} V  (threshold {EXT5V_MIN} V)")
        if not ok:
            problems += 1

    # 2. Throttling / undervoltage status
    throttled_raw = vcgencmd("get_throttled")
    try:
        status = decode_throttled(parse_throttled_output(throttled_raw))
    except ValueError:
        print(f"[FAIL] could not parse get_throttled: {throttled_raw!r}")
        problems += 1
    else:
        for name in status.live:
            print(f"[WARN] {name} — happening now")
            problems += 1
        for name in status.since_boot:
            print(f"[INFO] {name} — occurred since boot (sticky until reboot)")
        if not status.live and not status.since_boot:
            print("[OK] get_throttled = 0x0 — no undervoltage or throttling recorded")
        elif not status.live:
            print(f"[OK] get_throttled = 0x{status.raw:05X} — nothing throttling now")

    # 3. Temperature
    temp_raw = vcgencmd("measure_temp")
    try:
        temp = float(temp_raw.split("=")[1].replace("'C", "").strip())
    except (IndexError, ValueError):
        print(f"[FAIL] could not parse temperature: {temp_raw!r}")
        problems += 1
    else:
        ok = temp < TEMP_MAX
        print(f"[{'OK' if ok else 'HOT'}] temperature = {temp:.1f} °C  (limit {TEMP_MAX:.0f} °C)")
        if not ok:
            problems += 1

    print("-" * 40)
    if problems:
        print(f"⚠️  {problems} problem(s) found — check the battery / power supply.")
        return 1
    print("✓ Power health OK.")
    return 0


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""Communication check that does not move motors or servos.

Run on the Raspberry Pi:

    uv run python examples/01_i2c_probe.py

This verifies that the I2C bus is alive, the driver board responds at `0x40`, the reset command
works, and encoder reads are possible when enabled. Run this first after powering the system.
"""

from __future__ import annotations

import sys

from carbot.config import HAS_ENCODERS
from carbot.nezha import DEFAULT_ADDRESS, DEFAULT_BUS, NeZha, NeZhaError


def main() -> int:
    print(f"Opening I2C bus {DEFAULT_BUS} at address 0x{DEFAULT_ADDRESS:02X}...")

    try:
        board = NeZha(init_motors=False)
    except NeZhaError as exc:
        print(f"\n✗ Connection failed: {exc}")
        print("\nCheck in this order:")
        print("  1. Run `sudo i2cdetect -y 1` and confirm that 0x40 appears")
        print("  2. Confirm SDA -> Pin 3, SCL -> Pin 5, and shared ground")
        print("  3. Confirm the board has 12V power and the firmware LED is on")
        print("  4. Check for another device at 0x40, such as a PCA9685-based HAT")
        return 1

    print("✓ Reset command sent successfully; the driver board responded")

    with board:
        if not HAS_ENCODERS:
            print("\nSkipping encoders because config.HAS_ENCODERS = False for this build")
        else:
            print("\nReading encoder values:")
            for n in (1, 2, 3, 4):
                board.init_encoder(n)
                try:
                    print(f"  M{n}: {board.encoder(n):>6}")
                except NeZhaError as exc:
                    print(f"  M{n}: read failed - {exc}")

        print("\nBlinking the head LED to confirm the command path...")
        board.led("head", True)
        board.led("head", False)

    print(
        "\n✓ Probe finished successfully. Next: `examples/02_motor_check.py` with the car lifted."
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())

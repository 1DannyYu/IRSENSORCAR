#!/usr/bin/env python3
"""Interactively check the robotic arm's three servos.

The physical configuration uses S2, S3, and S4. Move only one servo at a time,
within 10 degrees of the 90-degree centre position.

**This script moves hardware. An operator must remain beside the robot and be able
to cut main power immediately.**

The procedure and safety gate live in :mod:`carbot.servo`; this file only connects
them to the real NeZha board.

Usage:
    PYTHONPATH=src python3 examples/other/04_servo_check.py
"""

from __future__ import annotations

import sys

from carbot.nezha import NeZha
from carbot.servo import run_session


def main() -> int:
    return run_session(lambda: NeZha(init_motors=False))


if __name__ == "__main__":
    sys.exit(main())

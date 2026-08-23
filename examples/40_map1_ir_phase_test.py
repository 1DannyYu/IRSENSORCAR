#!/usr/bin/env python3
"""Run one bounded Map 1 phase using Example 39's shared production control path.

This is a motor-moving operator tool, not an automated test.  It deliberately delegates to
``39_map1_ir_line_follow.py`` so a phase cannot pass with a copied algorithm that later differs
from the integrated route.  Phases 2-10 require the operator to place the car at the documented
entry pose printed before the normal hardware safety prompt.

Examples::

    PYTHONPATH=src python3 examples/40_map1_ir_phase_test.py --phase 1
    PYTHONPATH=src python3 examples/40_map1_ir_phase_test.py --phase 3 --speed 150
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from runpy import run_path

from carbot.map1_phases import PROVISIONAL_FORWARD_SPEED_CM_S, map1_phase


def main() -> int:
    parser = argparse.ArgumentParser(description="Bounded Map 1 IR phase hardware test")
    parser.add_argument("--phase", type=int, choices=range(1, 11), required=True)
    parser.add_argument(
        "--duration",
        type=float,
        help="safety timeout; default is derived conservatively from the phase distance",
    )
    args, forwarded = parser.parse_known_args()

    phase = map1_phase(args.phase)
    duration = args.duration
    if duration is None:
        duration = (
            20.0
            if phase.number == 1
            else max(20.0, phase.distance_cm / PROVISIONAL_FORWARD_SPEED_CM_S * 3.0 + 10.0)
        )

    script = Path(__file__).with_name("39_map1_ir_line_follow.py")
    namespace = run_path(str(script))
    integrated_main = namespace["main"]
    sys.argv = [
        str(script),
        "--test-phase",
        str(phase.number),
        "--duration",
        str(duration),
        *forwarded,
    ]
    return int(integrated_main())


if __name__ == "__main__":
    raise SystemExit(main())

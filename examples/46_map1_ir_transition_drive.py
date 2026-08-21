#!/usr/bin/env python3
"""Drive the run-derived Map 1 Phase 2 -> 3 -> 4 transition.

This is the consolidated follow-up to the 2026-08-21 morning runs. It uses
Example 39's production controller with the settings supported by those runs:

- acquire centred ``P0110`` on the Phase 2 eastbound straight;
- detect ARC 1 from ``P0100 -> P0000 -> P1000`` (or direct left-pair evidence);
- use the bounded left-only Phase 3 controller with no command-distance stop;
- accept the observed 0.5s centred ARC-exit window; and
- prove ordinary Phase 4 following for 2.0s, then stop.

The 07:26 physical run used the older 0.8s exit gate and left the map. The
0.5s correction is software-tested but has not yet passed a physical run.

**Motor-moving. Place the car on a centred Phase 2 point facing east. The
operator must stand beside the car able to cut main power instantly. Flatten
and secure the paper and clear the chassis underside before running.**

Usage (operator beside car, timestamp the evidence on the Pi)::

    mkdir -p scratch/ir-sensor-tracking
    LOG="scratch/ir-sensor-tracking/$(date +%Y-%m-%d-%H%M%S)-phase02-03-04.log"
    PYTHONPATH=src python3 -u examples/46_map1_ir_transition_drive.py 2>&1 | tee "$LOG"

Pass criteria are all three messages below. A duration limit is a failure::

    Phase 3 ARC 1 detected
    Phase 3 ARC 1 sensor exit confirmed -> Phase 4 North straight
    Phase 4 proof complete
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from runpy import run_path

DEFAULT_DURATION_S = 20.0
DEFAULT_SPEED = 150
DEFAULT_ACQUIRE_TIMEOUT_S = 5.0
DEFAULT_LEAD_IN_TIMEOUT_S = 5.0
DEFAULT_EXIT_CONFIRM_S = 0.5
DEFAULT_PHASE4_PROOF_S = 2.0
DEFAULT_HEARTBEAT_S = 0.5
DEFAULT_LOG_INTERVAL_S = 0.1


def example39_arguments(
    *,
    duration_s: float = DEFAULT_DURATION_S,
    speed: int = DEFAULT_SPEED,
    dry_run: bool = False,
    extra: tuple[str, ...] = (),
) -> list[str]:
    """Build the explicit Example 39 arguments for the consolidated run."""
    script = Path(__file__).with_name("39_map1_ir_line_follow.py")
    arguments = [
        str(script),
        "--test-phase",
        "3",
        "--duration",
        str(duration_s),
        "--speed",
        str(speed),
        "--start-acquire-timeout-s",
        str(DEFAULT_ACQUIRE_TIMEOUT_S),
        "--phase3-lead-in-timeout-s",
        str(DEFAULT_LEAD_IN_TIMEOUT_S),
        "--phase3-exit-confirm-s",
        str(DEFAULT_EXIT_CONFIRM_S),
        "--phase4-proof-s",
        str(DEFAULT_PHASE4_PROOF_S),
        "--heartbeat-s",
        str(DEFAULT_HEARTBEAT_S),
        "--log-every",
        "--log-min-interval-s",
        str(DEFAULT_LOG_INTERVAL_S),
    ]
    if dry_run:
        arguments.append("--dry-run")
    arguments.extend(extra)
    return arguments


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Bounded Map 1 IR Phase 2 -> ARC 1 -> Phase 4 transition drive"
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=DEFAULT_DURATION_S,
        help=f"whole-run safety ceiling in seconds (default {DEFAULT_DURATION_S:.0f})",
    )
    parser.add_argument(
        "--speed",
        type=int,
        default=DEFAULT_SPEED,
        help=f"base motor PWM (default {DEFAULT_SPEED})",
    )
    parser.add_argument("--dry-run", action="store_true", help="read sensors but send no motors")
    args, extra = parser.parse_known_args()

    if args.duration <= 0:
        parser.error("--duration must be positive")
    if not 1 <= args.speed <= 1000:
        parser.error("--speed must be in [1, 1000]")

    reserved = ("--test-phase", "--phase1-only", "--start-on-loop", "--stop-after-phase")
    conflicting = [item for item in extra if item.split("=", 1)[0] in reserved]
    if conflicting:
        parser.error(
            "this example fixes the Phase 2 -> 3 -> 4 envelope; remove conflicting argument(s): "
            + ", ".join(conflicting)
        )

    script = Path(__file__).with_name("39_map1_ir_line_follow.py")
    integrated_main = run_path(str(script))["main"]
    sys.argv = example39_arguments(
        duration_s=args.duration,
        speed=args.speed,
        dry_run=args.dry_run,
        extra=tuple(extra),
    )
    return int(integrated_main())


if __name__ == "__main__":
    raise SystemExit(main())

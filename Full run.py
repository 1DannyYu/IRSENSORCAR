#!/usr/bin/env python3
"""Phase 1 -> Phase 2, then the old 16-state IR table.

Same run as example 49: blind forward 17 cm + 90 degree right turn, then
auto-trace with the original table (left and right corrections both allowed).
A P1001 reading held past ROUNDABOUT_P1001_HOLD_S triggers the exit move once
(forward 5 cm, right 50 degrees) and drops back into auto-tracing. Once we're
40s in, a P0111 reading held past END_MARKER_HOLD_S means we've reached the
end marker - drive forward 1.2s and stop.

Motor-moving. The operator must stand beside the car, secure the chassis or
lift the wheels, and be able to cut power instantly.
"""

from __future__ import annotations

import argparse
import time

from carbot.ir_line_nav import detect_ir_line
from carbot.ir_modes import (
    ROUNDABOUT_P1001_HOLD_S,
    auto_tracing_original_command,
    phase1_to_phase2_timing,
    roundabout_p1001_action_timing,
)

# After this point in the run, a P0111 reading means "end marker reached",
# not "correct right" as it would earlier in the run.
END_MARKER_AFTER_S = 40.0
END_MARKER_HOLD_S = 0.2
END_MARKER_FORWARD_S = 1.2


def main() -> int:
    parser = argparse.ArgumentParser(description="Phase 1 -> Phase 2 -> original IR auto tracing")
    parser.add_argument("--duration", type=float, default=120.0)
    parser.add_argument("--speed", type=int, default=150)
    parser.add_argument("--dry-run", action="store_true", help="print commands without motors")
    args = parser.parse_args()
    if args.duration <= 0 or not 1 <= args.speed <= 1000:
        parser.error("duration must be positive and speed must be in [1, 1000]")

    # Blind Phase 1 move timing (forward, then 90 degree turn) and the
    # roundabout exit move timing (forward, then 50 degree turn).
    forward_s, turn_s = phase1_to_phase2_timing()
    p1001_forward_s, p1001_turn_s = roundabout_p1001_action_timing()

    # Safety gate: this script drives the motors, so the operator must be
    # standing by and ready to cut power before anything moves.
    if not args.dry_run and input(
        "Operator beside car, chassis secured, power ready to cut? (yes/no) "
    ).strip().lower() != "yes":
        print("Re-run when ready.")
        return 1

    from RPi import GPIO

    from carbot import Car, NeZhaError
    from carbot.ir_tracing import IRTracingSensor

    # Four IR sensors on BCM pins 24/25/22/23, all read as digital inputs.
    GPIO.setmode(GPIO.BCM)
    pins = (24, 25, 22, 23)
    for pin in pins:
        GPIO.setup(pin, GPIO.IN)
    sensor = IRTracingSensor(pins, GPIO, invert={0, 1, 2, 3})
    car = None
    try:
        if not args.dry_run:
            car = Car()
            # Phase 1: blind forward move, then a blind 90 degree right
            # turn, to clear the start marker before line-following starts.
            car.move_for(forward_s, args.speed, args.speed)
            car.move_for(turn_s, args.speed, -args.speed)

        started = time.monotonic()
        previous_command = None
        previous_localising = None
        p1001_since = None
        end_marker_since = None
        exit_done = False  # the roundabout exit move only ever fires once

        # Phase 2: read the IR sensors on a ~10ms loop and drive according
        # to the original 16-state table, until duration runs out or one of
        # the end conditions below fires.
        while time.monotonic() - started < args.duration:
            now = time.monotonic()
            elapsed = now - started
            reading = detect_ir_line(sensor, speed=args.speed)

            # Track how long the roundabout pattern (P1001) has been held
            # continuously, so a brief flicker doesn't trigger the exit move.
            if not exit_done and reading.physical == (1, 0, 0, 1):
                if p1001_since is None:
                    p1001_since = now
            else:
                p1001_since = None
            if reading.state.kind.value in ("on_line", "drift"):
                previous_localising = reading.physical

            # After 40s, a P0111 reading is the end marker. Track how long
            # it's been held continuously, so a brief flicker doesn't
            # trigger the stop early - only a hold past END_MARKER_HOLD_S
            # counts as actually reaching the marker.
            if elapsed >= END_MARKER_AFTER_S and reading.physical == (0, 1, 1, 1):
                if end_marker_since is None:
                    end_marker_since = now
            else:
                end_marker_since = None

            if end_marker_since is not None and now - end_marker_since > END_MARKER_HOLD_S:
                if car:
                    car.move_for(END_MARKER_FORWARD_S, args.speed, args.speed)
                    car.stop(best_effort=True)
                return 0

            # P1001 held past the hold threshold: this is the roundabout
            # exit move (forward, then hard right), fired once per run.
            if p1001_since is not None and now - p1001_since > ROUNDABOUT_P1001_HOLD_S:
                if car:
                    car.move_for(p1001_forward_s, args.speed, args.speed)
                    car.move_for(p1001_turn_s, args.speed, -args.speed)
                exit_done = True
                p1001_since = None
                previous_command = None
                continue

            # Normal case: look up the drive command for this sensor
            # reading in the original 16-state table and apply it.
            command = auto_tracing_original_command(
                reading.state,
                speed=args.speed,
                previous_command=previous_command,
                previous_localising=previous_localising,
            )
            if car:
                car.drive(command.left, command.right)
            previous_command = command
            time.sleep(max(0.0, 0.01 - (time.monotonic() - now)))
    except KeyboardInterrupt:
        print("Stopped by operator")
    except NeZhaError as exc:
        print(f"Motor/I2C error: {exc}")
        return 1
    finally:
        # Always release the motors and GPIO, even on error or Ctrl-C.
        if car:
            car.stop(best_effort=True)
            car.close()
        GPIO.cleanup()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

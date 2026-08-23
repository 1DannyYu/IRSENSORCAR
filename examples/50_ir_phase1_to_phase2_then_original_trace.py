#!/usr/bin/env python3
"""Phase 1 -> Phase 2, then the old 16-state IR table.

Same run as example 49: blind forward 17 cm + 90 degree right turn, then
auto-trace with the original table (left and right corrections both allowed).
A P1001 reading held past ROUNDABOUT_P1001_HOLD_S triggers the exit move once
(forward 5 cm, right 50 degrees) and drops back into auto-tracing. Once we're
40s in, a P0111 reading means we've reached the end marker - drive forward
1.2s and stop.

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

# after this point in the run, P0111 means "we're done", not "correct right"
END_MARKER_AFTER_S = 40.0
END_MARKER_FORWARD_S = 1.2


def main() -> int:
    parser = argparse.ArgumentParser(description="Phase 1 -> Phase 2 -> original IR auto tracing")
    parser.add_argument("--duration", type=float, default=120.0)
    parser.add_argument("--speed", type=int, default=150)
    parser.add_argument("--dry-run", action="store_true", help="print commands without motors")
    args = parser.parse_args()
    if args.duration <= 0 or not 1 <= args.speed <= 1000:
        parser.error("duration must be positive and speed must be in [1, 1000]")

    forward_s, turn_s = phase1_to_phase2_timing()
    p1001_forward_s, p1001_turn_s = roundabout_p1001_action_timing()
    print(f"Phase 1 -> Phase 2: forward 17 cm for {forward_s:.2f}s, then right 90 degrees for {turn_s:.2f}s")
    print(
        f"Auto tracing: original 16-state table (left + right corrections). Sustained P1001 over "
        f"{ROUNDABOUT_P1001_HOLD_S:.1f}s -> exit move (forward 5 cm, right 50 degrees). "
        f"After {END_MARKER_AFTER_S:.0f}s, P0111 -> forward {END_MARKER_FORWARD_S:.1f}s and stop."
    )

    if not args.dry_run and input(
        "Operator beside car, chassis secured, power ready to cut? (yes/no) "
    ).strip().lower() != "yes":
        print("Re-run when ready.")
        return 1

    from RPi import GPIO

    from carbot import Car, NeZhaError
    from carbot.ir_tracing import IRTracingSensor

    GPIO.setmode(GPIO.BCM)
    pins = (24, 25, 22, 23)
    for pin in pins:
        GPIO.setup(pin, GPIO.IN)
    sensor = IRTracingSensor(pins, GPIO, invert={0, 1, 2, 3})
    car = None
    try:
        if not args.dry_run:
            car = Car()
            print("Phase 1 starting: forward 17 cm", flush=True)
            car.move_for(forward_s, args.speed, args.speed)
            print("Phase 1 complete: turning right 90 degrees", flush=True)
            car.move_for(turn_s, args.speed, -args.speed)
            print("Phase 2 starting: original auto tracing", flush=True)

        started = time.monotonic()
        last = started
        previous_command = None
        previous_localising = None
        p1001_since = None
        exit_done = False  # the roundabout exit move only ever fires once

        while time.monotonic() - started < args.duration:
            now = time.monotonic()
            elapsed = now - started
            reading = detect_ir_line(sensor, speed=args.speed)

            if not exit_done and reading.physical == (1, 0, 0, 1):
                if p1001_since is None:
                    p1001_since = now
            else:
                p1001_since = None
            if reading.state.kind.value in ("on_line", "drift"):
                previous_localising = reading.physical

            if elapsed >= END_MARKER_AFTER_S and reading.physical == (0, 1, 1, 1):
                print(
                    f"{elapsed:6.1f}s P0111 after {END_MARKER_AFTER_S:.0f}s - forward "
                    f"{END_MARKER_FORWARD_S:.1f}s then stop",
                    flush=True,
                )
                if car:
                    car.move_for(END_MARKER_FORWARD_S, args.speed, args.speed)
                    car.stop(best_effort=True)
                return 0

            if p1001_since is not None and now - p1001_since > ROUNDABOUT_P1001_HOLD_S:
                print(f"{elapsed:6.1f}s P1001 held - exit move (forward 5 cm, right 50 degrees)", flush=True)
                if car:
                    car.move_for(p1001_forward_s, args.speed, args.speed)
                    car.move_for(p1001_turn_s, args.speed, -args.speed)
                exit_done = True
                p1001_since = None
                previous_command = None
                last = time.monotonic()
                continue

            command = auto_tracing_original_command(
                reading.state,
                speed=args.speed,
                previous_command=previous_command,
                previous_localising=previous_localising,
            )
            if car:
                car.drive(command.left, command.right)
            bits = "".join(str(bit) for bit in reading.physical)
            print(f"{elapsed:6.1f}s P{bits} -> L{command.left} R{command.right}: {command.reason}", flush=True)
            previous_command = command
            last = now
            time.sleep(max(0.0, 0.01 - (time.monotonic() - last)))
    except KeyboardInterrupt:
        print("Stopped by operator")
    except NeZhaError as exc:
        print(f"Motor/I2C error: {exc}")
        return 1
    finally:
        if car:
            car.stop(best_effort=True)
            car.close()
        GPIO.cleanup()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

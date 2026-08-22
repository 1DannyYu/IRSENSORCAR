#!/usr/bin/env python3
"""Drive Phase 1 -> Phase 2, then follow the original IR 16-state table.

The script performs the calibrated Phase 1 forward movement and 90-degree
right turn without sensor control. It then uses the original auto-tracing
policy, including both left and right corrections. A sustained P1001 reading
enters the Example 47 exit mode once: drive forward 5 cm, then turn right 50
degrees, and continue with the original auto-tracing policy. From 45 seconds
onward, any P0111 reading drives forward for 1.2 seconds, stops, and ends the run.

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

P0111_STOP_START_S = 40.0
P0111_STOP_FORWARD_S = 1.2


def main() -> int:
    parser = argparse.ArgumentParser(description="Phase 1 -> Phase 2 -> original IR auto tracing")
    parser.add_argument("--duration", type=float, default=120.0)
    parser.add_argument("--speed", type=int, default=150)
    parser.add_argument("--dry-run", action="store_true", help="print commands without motors")
    args = parser.parse_args()
    if args.duration <= 0 or not 1 <= args.speed <= 1000:
        parser.error("duration must be positive and speed must be in [1, 1000]")

    forward_s, turn_s = phase1_to_phase2_timing()
    print(
        f"Phase 1 -> Phase 2: forward 17 cm for {forward_s:.2f}s, "
        f"then right 90 degrees for {turn_s:.2f}s"
    )
    p1001_forward_s, p1001_turn_s = roundabout_p1001_action_timing()
    print(
        f"Auto tracing mode: original 16-state table, including right corrections; "
        f"exit mode on sustained P1001 over {ROUNDABOUT_P1001_HOLD_S:.1f}s "
        "(forward 5 cm, right 50 degrees)"
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
        p1001_since: float | None = None
        exit_action_done = False
        while time.monotonic() - started < args.duration:
            now = time.monotonic()
            elapsed = now - started
            reading = detect_ir_line(sensor, speed=args.speed)
            if not exit_action_done and reading.physical == (1, 0, 0, 1):
                if p1001_since is None:
                    p1001_since = now
            else:
                p1001_since = None
            if reading.state.kind.value in ("on_line", "drift"):
                previous_localising = reading.physical
            if elapsed >= P0111_STOP_START_S and reading.physical == (0, 1, 1, 1):
                print(
                    f"{elapsed:6.1f}s P0111 detected after {P0111_STOP_START_S:.0f}s; "
                    f"driving forward for {P0111_STOP_FORWARD_S:.1f}s, then stopping",
                    flush=True,
                )
                if car:
                    car.move_for(P0111_STOP_FORWARD_S, args.speed, args.speed)
                    car.stop(best_effort=True)
                return 0
            if p1001_since is not None and now - p1001_since > ROUNDABOUT_P1001_HOLD_S:
                print(
                    f"{now - started:6.1f}s P1001 held for over "
                    f"{ROUNDABOUT_P1001_HOLD_S:.1f}s; entering exit mode",
                    flush=True,
                )
                if car:
                    car.move_for(p1001_forward_s, args.speed, args.speed)
                    car.move_for(p1001_turn_s, args.speed, -args.speed)
                exit_action_done = True
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
            print(
                f"{now - started:6.1f}s P{bits} -> "
                f"L{command.left} R{command.right}: {command.reason}",
                flush=True,
            )
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

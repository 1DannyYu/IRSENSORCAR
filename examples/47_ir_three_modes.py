#!/usr/bin/env python3
"""Run one of the three operator-selected IR driving modes.

Modes:
  auto-tracing       Follow the 16-state table; motion is forward or left correction only.
  phase1-to-phase2   Drive forward 17 cm, spin right 90 degrees, then auto-trace Phase 2.
  circle             After 25.6 seconds and P1110/P1111, turn into the roundabout, auto-trace inside, then exit on
                     the verified P0111 -> P0101 -> P0100 -> P0110.
  chained             Phase 1 -> Phase 2 -> auto-tracing -> split circle mode.

Motor-moving. The operator must stand beside the car, secure the chassis or lift the wheels,
and be able to cut power instantly.
"""

from __future__ import annotations

import argparse
import time
from collections import deque

from carbot.ir_geometry import Kind
from carbot.ir_line_nav import detect_ir_line
from carbot.ir_modes import (
    CIRCLE_MODE_START_S,
    SEARCH_REPLAY_S,
    SEARCH_SWEEP_ANGLES_DEG,
    CircleModeState,
    DriveMode,
    ModeCommand,
    auto_tracing_command,
    line_search_required,
    phase1_to_phase2_timing,
    roundabout_entry_turn_s,
    search_sweep_turn_s,
)


def main() -> int:
    parser = argparse.ArgumentParser(description="Three IR driving modes")
    parser.add_argument("--mode", choices=[mode.value for mode in DriveMode], required=True)
    parser.add_argument("--duration", type=float, default=120.0)
    parser.add_argument("--speed", type=int, default=150)
    parser.add_argument("--dry-run", action="store_true", help="print commands without motors")
    args = parser.parse_args()
    if args.duration <= 0 or not 1 <= args.speed <= 1000:
        parser.error("duration must be positive and speed must be in [1, 1000]")

    mode = DriveMode(args.mode)
    forward_s, turn_s = phase1_to_phase2_timing()
    entry_turn_s = roundabout_entry_turn_s()
    print(f"Mode: {mode.value}; duration: {args.duration:.1f}s; speed: {args.speed}")
    if mode in (DriveMode.PHASE1_TO_PHASE2, DriveMode.CHAINED):
        print(f"Phase 1: forward 17 cm for {forward_s:.2f}s, then right 90 degrees for {turn_s:.2f}s")
    if mode in (DriveMode.CIRCLE, DriveMode.CHAINED):
        print(
            f"Circle mode: after {CIRCLE_MODE_START_S:.0f}s and P1110/P1111 turn right about 42.5 degrees, "
            "auto-trace inside, "
            "then exit on P0111 -> P0101 -> P0100 -> P0110"
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
        chain_started = time.monotonic() if mode is DriveMode.CHAINED else None
        if not args.dry_run:
            car = Car()
            if mode in (DriveMode.PHASE1_TO_PHASE2, DriveMode.CHAINED):
                print("Phase 1 starting: forward 17 cm", flush=True)
                car.move_for(forward_s, args.speed, args.speed)
                print("Phase 1 complete: turning right 90 degrees", flush=True)
                car.move_for(turn_s, args.speed, -args.speed)
                print("Phase 2 starting: auto-tracing", flush=True)

        started = chain_started or time.monotonic()
        last = started
        previous_command: ModeCommand | None = None
        previous_localising = None
        circle_state = CircleModeState()
        command_history: deque[tuple[int, int, float]] = deque()
        command_history_s = 0.0

        def log_search(message: str) -> None:
            print(f"{time.monotonic() - started:6.1f}s {message}", flush=True)

        def search_mode_1() -> bool:
            """Sweep 5, 20, then 45 degrees in both directions until line reacquisition."""
            for angle_deg in SEARCH_SWEEP_ANGLES_DEG:
                duration = search_sweep_turn_s(angle_deg)
                for direction, label in ((-1, "left"), (1, "right")):
                    log_search(f"SEARCH MODE 1: sweeping {label} {angle_deg:.0f} degrees")
                    deadline = time.monotonic() + duration
                    while time.monotonic() < deadline:
                        reading = detect_ir_line(sensor, speed=args.speed)
                        if reading.state.kind in (Kind.ON_LINE, Kind.DRIFT):
                            if car:
                                car.stop(best_effort=True)
                            log_search(f"SEARCH MODE 1: line reacquired at P{''.join(map(str, reading.physical))}")
                            return True
                        if car:
                            car.drive(-args.speed, args.speed) if direction < 0 else car.drive(args.speed, -args.speed)
                        time.sleep(0.01)
                    if car:
                        car.stop(best_effort=True)
            log_search("SEARCH MODE 1: no line found after 5, 20, and 45 degree sweeps")
            return False

        def search_mode_2() -> None:
            """Replay the most recent two seconds in reverse, then return to Mode 1."""
            log_search(f"SEARCH MODE 2: reverse-replaying up to {SEARCH_REPLAY_S:.1f}s")
            remaining = SEARCH_REPLAY_S
            for left, right, segment_s in reversed(command_history):
                if remaining <= 0:
                    break
                duration = min(segment_s, remaining)
                if car:
                    car.move_for(duration, -left, -right)
                remaining -= duration
            if car:
                car.stop(best_effort=True)
            log_search("SEARCH MODE 2: replay complete; returning to SEARCH MODE 1")

        while time.monotonic() - started < args.duration:
            now = time.monotonic()
            dt = now - last
            elapsed = now - started
            if previous_command is not None and dt > 0:
                command_history.append((previous_command.left, previous_command.right, dt))
                command_history_s += dt
                while command_history_s > SEARCH_REPLAY_S and command_history:
                    _, _, old_dt = command_history.popleft()
                    command_history_s -= old_dt
            reading = detect_ir_line(sensor, speed=args.speed)
            if reading.state.kind in (Kind.ON_LINE, Kind.DRIFT):
                previous_localising = reading.physical

            circle_event = None
            if mode in (DriveMode.CIRCLE, DriveMode.CHAINED):
                circle_event = circle_state.observe(elapsed_s=elapsed, bits=reading.physical)
            if circle_event == "enter":
                print(
                    f"{elapsed:.1f}s: timed circle entry at {CIRCLE_MODE_START_S:.0f}s; "
                    "turning right into roundabout",
                    flush=True,
                )
                if car:
                    car.move_for(entry_turn_s, args.speed, -args.speed)
                previous_command = None
                last = time.monotonic()
                continue
            if circle_event == "exit":
                print(
                    f"{elapsed:.1f}s: roundabout exit sequence confirmed; turning right to exit",
                    flush=True,
                )
                if car:
                    car.move_for(turn_s, args.speed, -args.speed)
                previous_command = None
                last = time.monotonic()
                continue

            if line_search_required(reading.state, previous_localising):
                log_search("P0000 resolved as genuine line loss; entering SEARCH MODE 1")
                while not search_mode_1():
                    search_mode_2()
                previous_command = None
                previous_localising = None
                last = time.monotonic()
                continue

            command = auto_tracing_command(
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

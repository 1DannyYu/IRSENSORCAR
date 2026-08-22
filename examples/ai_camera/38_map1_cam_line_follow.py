#!/usr/bin/env python3
"""Map1 test: drive the circular track by downward camera only (no IR sensor).

The IR-sensor version of the same route is `39_map1_ir_line_follow.py`.

**Motor-moving. An operator must stand beside the car able to cut main power instantly.**

The Map1 track is a simple circular loop with a marked start zone. The car:
  1. Starts in the marked start box
  2. Follows the black line around the circle
  3. Completes the loop and exits when a second roundabout fork is detected

Usage (dry run — no motor, debug detection):
    PYTHONPATH=src python3 examples/ai_camera/38_map1_cam_line_follow.py --dry-run --duration 10

Usage (live run — on the track):
    # Operator beside car, track clear, wheels on start line
    PYTHONPATH=src python3 examples/ai_camera/38_map1_cam_line_follow.py --duration 120
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

from carbot.line_follow import LinePolicy, detect_line
from carbot.line_nav import LineNav, NavPolicy


def _open_camera():
    from picamera2 import Picamera2

    camera = Picamera2()
    camera.configure(camera.create_preview_configuration(main={"size": (2028, 1520)}))
    camera.start()
    time.sleep(1.5)
    return camera


def main() -> int:
    parser = argparse.ArgumentParser(description="Map1 circular track line-following")
    parser.add_argument("--dry-run", action="store_true", help="detection only, no motor")
    parser.add_argument("--duration", type=float, default=120.0, help="run duration (seconds)")
    parser.add_argument(
        "--speed", type=int, default=150, help="base drive speed (150-200 typical)"
    )
    parser.add_argument(
        "--threshold",
        type=int,
        default=100,
        help="line detection threshold (0-255; verified default 100)",
    )
    parser.add_argument(
        "--turn-gain",
        type=float,
        default=2.5,
        help="steering sensitivity (2.5 = strong)",
    )
    parser.add_argument(
        "--enable-roundabout",
        action="store_true",
        default=False,
        help="enable roundabout entry/exit (Map1 simple circle: usually False)",
    )
    args = parser.parse_args()

    line_policy = LinePolicy(
        dark_threshold=args.threshold,
        roi_top=0.10,
        roi_bottom=0.68,  # exclude chassis band
    )
    nav_policy = NavPolicy(
        speed=args.speed,
        turn_gain=args.turn_gain,
        enable_roundabout=args.enable_roundabout,
        blind_creep_s=1.0,  # creep when line lost (clear camera blind cone)
        search_sweep_deg=20.0,
        search_give_up_s=3.0,
    )
    nav = LineNav(nav_policy)

    print("=" * 70)
    print("Map1 Line-Following Test")
    print("=" * 70)
    print(f"Speed: {args.speed} | Threshold: {args.threshold}")
    print(f"Turn gain: {args.turn_gain} | Roundabout: {args.enable_roundabout}")
    print(f"Duration: {args.duration}s")
    print()

    if not args.dry_run:
        answer = input("Operator beside car, track clear, power ready to cut? (yes/no) ").strip()
        if answer.lower() != "yes":
            print("Re-run when operator is ready.")
            return 1

    import cv2

    from carbot import Car, NeZhaError

    car = None
    if not args.dry_run:
        try:
            car = Car()
        except NeZhaError as exc:
            print(f"Connection failed: {exc}")
            return 1

    try:
        camera = _open_camera()
    except Exception as exc:
        print(f"Camera failed: {exc}", file=sys.stderr)
        if car:
            car.close()
        return 1

    # Fixed exposure (prevents auto-exposure drift during motion)
    try:
        camera.set_controls(
            {"AeEnable": False, "ExposureTime": 50_000, "AnalogueGain": 4.5}
        )
        time.sleep(0.5)
    except Exception:  # noqa: S110
        pass

    start = time.monotonic()
    last = start
    frame_index = 0

    try:
        while True:
            now = time.monotonic()
            dt = now - last
            last = now
            frame_index += 1

            frame = camera.capture_array("main")
            reading = detect_line(frame, line_policy)
            command = nav.step(reading, dt)

            if car:
                car.drive(command.left, command.right)

            elapsed = now - start
            print(
                f"[{elapsed:6.1f}s] #{frame_index:4d} "
                f"{reading.summary:30s} -> "
                f"{command.state.value:10s}:{command.action:6s} "
                f"L{command.left:4d} R{command.right:4d} | "
                f"{command.reason[:50]:50s}"
            )

            if args.duration and elapsed >= args.duration:
                print(f"\nDuration limit reached ({args.duration}s)")
                break

    except KeyboardInterrupt:
        print("\nStopped by operator")
    finally:
        if car:
            car.stop()
            car.close()
        camera.close()
        elapsed = time.monotonic() - start
        print(f"\nTest completed: {elapsed:.1f}s, {frame_index} frames")

    return 0


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""Drive Task-1 with the AI-camera vision navigation, route plan as advisory.

**Motor-moving. An operator must stand beside the car able to cut main power
instantly.**

The controller is the vision-driven state machine in ``carbot.line_nav``
(the same one example 26 uses): follow the 15 mm line while it is visible,
spin at a detected horizontal crossing (T junction), lap the roundabout on a
persistent fork, and search when the line drops. The Task-1 route plan
(``carbot.route_plan``) is advisory only — it is printed as a phase tracker
so the operator can see which phase the car is expected to be in; the car
never *commands* turns from wheel timing because the camera decides.

    # dry run: camera + nav only, no motors
    PYTHONPATH=src python3 examples/29_cam_route_nav_drive.py --dry-run --duration 10

    # supervised run
    PYTHONPATH=src python3 examples/29_cam_route_nav_drive.py --duration 120 \\
        --ground-view /tmp/line-follow/ground-view.json
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

from carbot.line_follow import LinePolicy
from carbot.line_nav import LineNav, NavPolicy
from carbot.route_nav import RouteTracker
from carbot.route_plan import task1_route, total_distance_m


def _open_camera():
    from picamera2 import Picamera2

    camera = Picamera2()
    camera.configure(camera.create_preview_configuration(main={"size": (2028, 1520)}))
    camera.start()
    return camera


def detect_line(frame, policy, ground_view=None):
    if ground_view is not None:
        from carbot.ground_view import detect_line_on_ground

        return detect_line_on_ground(frame, ground_view, policy)
    from carbot.line_follow import detect_line as _detect

    return _detect(frame, policy)


def main() -> int:
    parser = argparse.ArgumentParser(description="Task-1 vision navigation drive")
    parser.add_argument(
        "--dry-run", action="store_true", help="camera + plan only; never drive motors"
    )
    parser.add_argument(
        "--duration", type=float, default=120.0, help="max seconds to run (0 = until stopped)"
    )
    parser.add_argument("--threshold", type=int, default=LinePolicy().dark_threshold)
    parser.add_argument("--roi-top", type=float, default=LinePolicy().roi_top)
    parser.add_argument("--roi-bottom", type=float, default=LinePolicy().roi_bottom)
    parser.add_argument("--speed", type=int, default=200)
    parser.add_argument("--turn-gain", type=float, default=2.5)
    parser.add_argument("--blind-creep-s", type=float, default=1.5)
    parser.add_argument(
        "--right-turn-after-s",
        type=float,
        default=0.2,
        help="how long a near T bar must persist before the "
        "right spin starts (default 0.2s: the bar is only "
        "visible briefly before the blind cone)",
    )
    parser.add_argument("--search-sweep-deg", type=float, default=20.0)
    parser.add_argument("--search-give-up-s", type=float, default=2.5)
    parser.add_argument("--roundabout", action="store_true", default=True)
    parser.add_argument("--exposure-time-us", type=int, default=50_000)
    parser.add_argument("--analogue-gain", type=float, default=4.5)
    parser.add_argument(
        "--sonar-stop-cm",
        type=float,
        default=15.0,
        help="HC-SR04 emergency stop distance (0 disables)",
    )
    parser.add_argument(
        "--ground-view", type=Path, default=None, help="bird's-eye homography JSON (recommended)"
    )
    parser.add_argument(
        "--line-width-mm",
        type=float,
        default=LinePolicy().line_width_m * 1000,
        help="physical width of the track line in mm (Task-1 reprint map: 15)",
    )
    parser.add_argument("--save-every", type=int, default=0)
    parser.add_argument("--log-dir", type=Path, default=Path("/tmp/route-nav"))
    args = parser.parse_args()

    line_policy = LinePolicy(
        dark_threshold=args.threshold,
        roi_top=args.roi_top,
        roi_bottom=args.roi_bottom,
        line_width_m=args.line_width_mm / 1000,
    )
    nav_policy = NavPolicy(
        speed=args.speed,
        turn_gain=args.turn_gain,
        blind_creep_s=args.blind_creep_s,
        search_sweep_deg=args.search_sweep_deg,
        search_give_up_s=args.search_give_up_s,
        enable_roundabout=args.roundabout,
        right_turn_after_s=args.right_turn_after_s,
    )
    plan = task1_route()
    tracker = RouteTracker(plan, LineNav(nav_policy))
    print(
        f"route: {plan.name} - {len(plan)} steps, total {total_distance_m(plan):.2f} m "
        f"(advisory; vision drives)"
    )

    from carbot.ground_view import load_optional_ground_view

    ground_view = load_optional_ground_view(args.ground_view)
    if ground_view is not None:
        print("using bird's-eye ground view for line detection")

    if not args.dry_run:
        answer = input("Operator beside the car, path clear, power ready to cut? (yes/no) ").strip()
        if answer.lower() != "yes":
            print("Re-run when an operator is ready beside the car.")
            return 1

    import cv2

    from carbot import Car, NeZhaError

    car = None
    if not args.dry_run:
        try:
            car = Car()
        except NeZhaError as exc:
            print(f"Connection failed: {exc}")
            print("Run `examples/01_i2c_probe.py` first to debug the link.")
            return 1

    try:
        camera = _open_camera()
    except Exception as exc:  # noqa: BLE001 - report any camera backend error
        print(f"camera failed: {exc}", file=sys.stderr)
        if car:
            car.close()
        return 1

    try:
        camera.set_controls(
            {
                "AeEnable": False,
                "ExposureTime": args.exposure_time_us,
                "AnalogueGain": args.analogue_gain,
            }
        )
        time.sleep(0.5)
    except Exception as exc:  # noqa: BLE001 - report control failure
        print(f"exposure controls failed: {exc}", file=sys.stderr)

    if args.save_every and args.save_every > 0:
        args.log_dir.mkdir(parents=True, exist_ok=True)

    start = time.monotonic()
    last = start
    frame_index = 0
    try:
        while True:
            now = time.monotonic()
            dt = min(now - last, 0.5)
            last = now
            frame_index += 1

            frame = camera.capture_array("main")
            reading = detect_line(frame, line_policy, ground_view=ground_view)
            status = tracker.step(reading, dt)
            if car:
                cmd = tracker.last_command
                car.drive(cmd.left, cmd.right)

            summary = reading.summary if reading is not None else "no reading"
            print(
                f"[{now - start:6.1f}s] #{frame_index:4d} step{status.step_index:02d} "
                f"[{status.step_label}] {status.message} | {summary}"
            )

            if args.save_every and frame_index % args.save_every == 0:
                annotated = frame.copy()
                cv2.putText(
                    annotated,
                    f"step{status.step_index} {status.step_label}",
                    (20, 60),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1.2,
                    (0, 255, 0),
                    3,
                )
                cv2.imwrite(str(args.log_dir / f"frame-{frame_index:05d}.jpg"), annotated)

            if args.duration and (now - start) >= args.duration:
                print("duration limit reached; stopping")
                break
    except KeyboardInterrupt:
        print("\nstopped by operator")
    finally:
        if car:
            car.stop()
            car.close()
        camera.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())

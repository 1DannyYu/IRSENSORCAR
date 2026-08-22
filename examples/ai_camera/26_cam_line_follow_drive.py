#!/usr/bin/env python3
"""Drive the black line on the track map by camera, closed-loop.

Downward camera frames feed :func:`carbot.line_follow.detect_line`; the
resulting readings drive :class:`carbot.line_nav.LineNav`, whose wheel commands
are applied through `carbot.Car` (``car.drive(left, right)``). The car follows
the line, searches when it disappears, and treats a persistent fork as a
roundabout entry with a time-confirmed lap.

**Motor-moving. An operator must stand beside the car able to cut main power
instantly.** Run `examples/other/14_all_sensors_preflight_check.py` first, lift the wheels for
the first smoke test, then place the car on the track map at the start zone.

    # stationary logic check — camera + detection + state machine, never drives
    PYTHONPATH=src python3 examples/ai_camera/26_cam_line_follow_drive.py --dry-run --duration 10

    # supervised run (prompts for operator confirmation)
    PYTHONPATH=src python3 examples/ai_camera/26_cam_line_follow_drive.py --duration 60

Run with the system python3: picamera2 and OpenCV are apt packages. The line
detection threshold and the navigation policy are tunable; defaults match the
verified 2026-08-15 downward still (map paper ~208 gray, line ~2.3 % of
pixels, threshold 100).
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

from carbot.ground_view import load_optional_ground_view
from carbot.line_follow import LinePolicy, detect_line
from carbot.line_nav import LineNav, NavPolicy

PREVIEW_SIZE = (2028, 1520)


def _open_camera():
    from picamera2 import Picamera2

    camera = Picamera2()
    camera.configure(camera.create_preview_configuration(main={"size": PREVIEW_SIZE}))
    camera.start()
    time.sleep(1.5)  # auto-exposure settle
    return camera


def main() -> int:
    parser = argparse.ArgumentParser(description="Camera line-following drive")
    parser.add_argument(
        "--dry-run", action="store_true", help="run detection and the state machine but never drive"
    )
    parser.add_argument(
        "--duration", type=float, default=60.0, help="seconds to run (0 = until Ctrl-C)"
    )
    parser.add_argument(
        "--threshold",
        type=int,
        default=LinePolicy().dark_threshold,
        help="gray value below which a pixel counts as line",
    )
    parser.add_argument("--roi-top", type=float, default=LinePolicy().roi_top)
    parser.add_argument("--roi-bottom", type=float, default=LinePolicy().roi_bottom)
    parser.add_argument(
        "--speed",
        type=int,
        default=200,
        help="base drive speed 0-1000; 200 is the calibrated spin rate",
    )
    parser.add_argument(
        "--search-sweep-deg",
        type=float,
        default=20.0,
        help="sweep angle in degrees per step during visual search (default: 20.0 deg)",
    )
    parser.add_argument(
        "--search-give-up-s",
        type=float,
        default=2.5,
        help="stop search after this many seconds if no line found (default: 2.5s)",
    )
    parser.add_argument(
        "--blind-creep-s",
        type=float,
        default=1.5,
        help="creep straight for this many seconds when line is lost "
        "(clears forward camera blind cone) before search (default: 1.5s)",
    )
    parser.add_argument(
        "--turn-gain",
        type=float,
        default=2.5,
        help="steering sensitivity; 2.5 = strong (188-200 speed spread for small error)",
    )
    parser.add_argument(
        "--roundabout-loop-min-s",
        type=float,
        default=6.5,
        help="minimum seconds inside a roundabout before an exit fork counts",
    )
    parser.add_argument(
        "--junction-width-factor",
        type=float,
        default=2.0,
        help="line must widen by this factor to count as junction; "
        "2.0 = stricter (rejects scattered dark structures)",
    )
    parser.add_argument(
        "--junction-min-branch-rows-fraction",
        type=float,
        default=0.10,
        help="branch must span this fraction of ROI height to count as fork; "
        "0.10 = stricter (needs 88+ rows on 1763-row ROI)",
    )
    parser.add_argument(
        "--expected-center",
        type=float,
        default=0.46,
        help="frame-width fraction treated as on heading; 0.46 "
        "because the camera sits right of the axle",
    )
    parser.add_argument(
        "--roundabout",
        action="store_true",
        default=True,
        help="enable roundabout entry/exit (default: enabled)",
    )
    parser.add_argument(
        "--start-turn-s",
        type=float,
        default=0.0,
        help="optional right turn at launch before line-following; "
        "0 (default) because the start-zone line already bends right",
    )
    parser.add_argument(
        "--exposure-time-us",
        type=int,
        default=50_000,
        help="fixed shutter in us; fixed exposure stops the auto-exposure "
        "drift that broke detection while the car moved",
    )
    parser.add_argument(
        "--analogue-gain",
        type=float,
        default=4.5,
        help="fixed analogue gain with --exposure-time-us",
    )
    parser.add_argument(
        "--sonar-stop-cm",
        type=float,
        default=15.0,
        help="stop immediately if HC-SR04 sonar detects wall/obstacle closer than this (default: 15.0 cm; 0 to disable)",
    )
    parser.add_argument(
        "--log-dir",
        type=Path,
        default=Path("/tmp/line-follow"),
        help="save every Nth annotated frame here (--save-every)",
    )
    parser.add_argument(
        "--save-every",
        type=int,
        default=0,
        help="save an annotated frame every N frames (0 = never)",
    )
    parser.add_argument(
        "--ground-view", type=Path, default=None, help="bird's-eye homography JSON from examples/27"
    )
    parser.add_argument(
        "--line-width-mm",
        type=float,
        default=LinePolicy().line_width_m * 1000,
        help="physical width of the track line in mm (Task-1 reprint map: 15)",
    )
    args = parser.parse_args()

    line_policy = LinePolicy(
        dark_threshold=args.threshold,
        roi_top=args.roi_top,
        roi_bottom=args.roi_bottom,
        min_branch_rows_fraction=args.junction_min_branch_rows_fraction,
        line_width_m=args.line_width_mm / 1000,
    )
    nav_policy = NavPolicy(
        speed=args.speed,
        turn_gain=args.turn_gain,
        roundabout_loop_min_s=args.roundabout_loop_min_s,
        junction_width_factor=args.junction_width_factor,
        junction_min_branch_rows_fraction=args.junction_min_branch_rows_fraction,
        expected_center_fraction=args.expected_center,
        enable_roundabout=args.roundabout,
        blind_creep_s=args.blind_creep_s,
        search_sweep_deg=args.search_sweep_deg,
        search_give_up_s=args.search_give_up_s,
    )
    nav = LineNav(nav_policy)
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
            print("Run `examples/other/01_i2c_probe.py` first to debug the link.")
            return 1

    try:
        camera = _open_camera()
    except Exception as exc:  # noqa: BLE001 - report any camera backend error
        print(f"camera failed: {exc}", file=sys.stderr)
        if car:
            car.close()
        return 1

    # Fixed exposure: auto-exposure drifted the frame darker while the car
    # moved, pushing the map background under the line threshold.
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

    # Departure: the track plan starts with a right turn out of the start
    # zone, before line-following takes over.
    if car and args.start_turn_s > 0:
        print(f"departure: turning right for {args.start_turn_s:.1f}s")
        car.turn_right(args.speed, ratio=0.5)
        time.sleep(args.start_turn_s)
        car.stop()

    sonar_enabled = False
    if car and args.sonar_stop_cm > 0:
        try:
            from RPi import GPIO

            GPIO.setmode(GPIO.BCM)
            GPIO.setup(17, GPIO.OUT)
            GPIO.setup(27, GPIO.IN)
            sonar_enabled = True
            print(
                f"HC-SR04 ultrasonic sonar active: emergency stop if obstacle < {args.sonar_stop_cm:.1f} cm"
            )
        except Exception as exc:  # noqa: BLE001
            print(f"sonar setup skipped: {exc}")

    if args.save_every and args.save_every > 0:
        args.log_dir.mkdir(parents=True, exist_ok=True)

    start = time.monotonic()
    last = start
    frame_index = 0
    try:
        while True:
            now = time.monotonic()
            dt = now - last
            last = now
            frame_index += 1

            # Ultrasonic safety check
            if sonar_enabled:
                try:
                    from RPi import GPIO

                    GPIO.output(17, GPIO.LOW)
                    time.sleep(0.001)
                    GPIO.output(17, GPIO.HIGH)
                    time.sleep(0.00001)
                    GPIO.output(17, GPIO.LOW)
                    t0 = time.time()
                    echo_ok = True
                    while GPIO.input(27) == GPIO.LOW:
                        if time.time() - t0 > 0.02:
                            echo_ok = False
                            break
                    if echo_ok:
                        p_start = time.time()
                        while GPIO.input(27) == GPIO.HIGH:
                            if time.time() - p_start > 0.02:
                                break
                        dist_cm = (time.time() - p_start) * 34300.0 / 2.0
                        if 0.5 < dist_cm < args.sonar_stop_cm:
                            print(
                                f"\n[EMERGENCY STOP] Sonar detected obstacle/wall at {dist_cm:.1f} cm! Stopping motors."
                            )
                            if car:
                                car.stop()
                            break
                except Exception:  # noqa: BLE001, S110 - camera controls are optional
                    pass

            frame = camera.capture_array("main")
            reading = detect_line(frame, line_policy, ground_view=ground_view)
            command = nav.step(reading, dt)
            if car:
                car.drive(command.left, command.right)

            print(
                f"[{now - start:6.1f}s] #{frame_index:4d} {reading.summary} -> "
                f"{command.state.value}:{command.action} L{command.left} "
                f"R{command.right} | {command.reason}"
            )

            if args.save_every and frame_index % args.save_every == 0:
                annotated = frame.copy()
                cv2.putText(
                    annotated,
                    f"{command.state.value} {command.action}",
                    (20, 60),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1.5,
                    (0, 255, 0),
                    3,
                )
                cv2.imwrite(str(args.log_dir / f"frame-{frame_index:05d}.jpg"), annotated)

            if args.duration and (now - start) >= args.duration:
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

#!/usr/bin/env python3
"""Calibrate a ground-plane bird's-eye view for line following.

The camera looks forward, so the paper is a trapezoid. Four correspondences
on the floor (a measured rectangle, or the ChArUco board laid flat) give a
homography; later captures are warped to a metric top-down patch before the
15 mm line is measured.

Stationary, **no motors**.

    # on the Pi, ChArUco board flat on the paper in front of the car
    PYTHONPATH=src python3 examples/ai_camera/27_cam_ground_view_calibrate.py \\
        --output /tmp/line-follow/ground-view.json

    # from a still, four image corners of a measured rectangle (TL TR BR BL)
    PYTHONPATH=src python3 examples/ai_camera/27_cam_ground_view_calibrate.py \\
        --image /tmp/line-follow/line-follow-raw.jpg \\
        --corners 400,500,1600,520,1580,900,380,880 \\
        --size-m 0.20,0.15 --near-m 0.20

Copy the JSON next to the line-follow scripts (default path below) so
examples 25 and 26 use the bird's-eye detector.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np

from carbot.ground_view import (
    auto_calibrate_ground_view,
    calibrate_ground_view,
    detect_line_on_ground,
    ground_view_from_charuco,
    save_ground_view,
)
from carbot.line_follow import LinePolicy

PREVIEW_SIZE = (2028, 1520)
DEFAULT_OUTPUT = Path("/tmp/line-follow/ground-view.json")


def _capture(size: tuple[int, int]) -> object:
    from picamera2 import Picamera2

    camera = Picamera2()
    camera.configure(camera.create_preview_configuration(main={"size": size}))
    camera.start()
    try:
        camera.set_controls(
            {
                "AeEnable": False,
                "ExposureTime": 50_000,
                "AnalogueGain": 4.5,
            }
        )
        time.sleep(0.5)
    except Exception:  # noqa: BLE001 - camera controls are optional on some builds
        time.sleep(1.5)
    frame = camera.capture_array("main")
    camera.close()
    return frame


def _parse_corners(text: str) -> np.ndarray:
    values = [float(part) for part in text.replace(" ", "").split(",") if part]
    if len(values) != 8:
        raise ValueError("corners must be eight numbers: x1,y1,x2,y2,x3,y3,x4,y4")
    return np.array(values, dtype=np.float64).reshape(4, 2)


def main() -> int:
    parser = argparse.ArgumentParser(description="Calibrate bird's-eye ground view")
    parser.add_argument("--image", type=Path, help="use this still instead of capturing")
    parser.add_argument(
        "--output",
        type=Path,
        default=DEFAULT_OUTPUT,
        help="JSON path for the homography",
    )
    parser.add_argument(
        "--corners",
        help="TL,TR,BR,BL pixel corners of a measured floor rectangle",
    )
    parser.add_argument(
        "--size-m",
        default="0.20,0.15",
        help="width,height in metres of that rectangle (x right, y forward)",
    )
    parser.add_argument(
        "--near-m",
        type=float,
        default=0.18,
        help="ground-forward distance of the rectangle's near edge",
    )
    parser.add_argument(
        "--charuco", action="store_true", help="fit from a ChArUco board lying on the paper"
    )
    parser.add_argument(
        "--auto",
        action="store_true",
        help="auto-detect the printed calibration target's corners "
        "(scripts/generate_ground_view_target.py) instead of --corners; "
        "use with --size-m/--near-m",
    )
    # The BEV world-y window. Defaults here are the 2026-08-16 verified
    # values (`calibrate_ground_view`'s own defaults, y_min_m=0.12/y_max_m=0.72,
    # excluded the real near-field line and produced "no line" even though
    # the homography itself was correct) — worth exposing as flags so the
    # next calibration does not need ad-hoc Python to widen the window.
    parser.add_argument("--x-min-m", type=float, default=-0.30)
    parser.add_argument("--x-max-m", type=float, default=0.30)
    parser.add_argument("--y-min-m", type=float, default=-0.10)
    parser.add_argument("--y-max-m", type=float, default=0.90)
    parser.add_argument(
        "--line-width-mm",
        type=float,
        default=LinePolicy().line_width_m * 1000,
        help="physical width of the track line in mm (Task-1 reprint map: 15)",
    )
    args = parser.parse_args()

    if args.image is not None:
        import cv2

        frame = cv2.imread(str(args.image), cv2.IMREAD_COLOR)
        if frame is None:
            print(f"could not read {args.image}", file=sys.stderr)
            return 1
    else:
        try:
            frame = _capture(PREVIEW_SIZE)
        except Exception as exc:  # noqa: BLE001
            print(f"camera failed: {exc}", file=sys.stderr)
            return 1

    window = {
        "x_min_m": args.x_min_m,
        "x_max_m": args.x_max_m,
        "y_min_m": args.y_min_m,
        "y_max_m": args.y_max_m,
        "line_width_m": args.line_width_mm / 1000,
    }
    try:
        if args.charuco:
            view = ground_view_from_charuco(frame, **window)
        elif args.auto:
            width_m, height_m = (float(part) for part in args.size_m.split(","))
            view = auto_calibrate_ground_view(
                frame,
                target_width_m=width_m,
                target_height_m=height_m,
                near_m=args.near_m,
                **window,
            )
        elif args.corners:
            width_m, height_m = (float(part) for part in args.size_m.split(","))
            image_points = _parse_corners(args.corners)
            far_m = args.near_m + height_m
            world_points = np.array(
                [
                    [-width_m / 2, far_m],
                    [width_m / 2, far_m],
                    [width_m / 2, args.near_m],
                    [-width_m / 2, args.near_m],
                ],
                dtype=np.float64,
            )
            view = calibrate_ground_view(image_points, world_points, **window)
        else:
            print(
                "Pass --charuco (board on the paper), --auto (detect the "
                "printed target), or --corners with --size-m.",
                file=sys.stderr,
            )
            return 1
    except (ValueError, RuntimeError) as exc:
        print(f"calibration failed: {exc}", file=sys.stderr)
        return 1

    reading = detect_line_on_ground(frame, view, LinePolicy())
    save_ground_view(args.output, view)
    print(f"saved {args.output}")
    print(
        f"bird's-eye {view.bev_width}x{view.bev_height}  "
        f"{view.metres_per_pixel * 1000:.1f} mm/px  "
        f"line {view.line_width_m * 1000:.0f} mm ≈ {view.expected_line_width_px:.0f} px"
    )
    print(f"line: {reading.summary}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""Static IMX500 AprilTag distance and pose check; this script never moves the car.

Run on the Raspberry Pi with its system Python:

    PYTHONPATH=src python3 examples/ai_camera/12_cam_apriltag_pose.py

For an existing image on a development machine:

    PYTHONPATH=src uv run --extra vision python examples/ai_camera/12_cam_apriltag_pose.py --input photo.jpg
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

from carbot.vision import (
    DEFAULT_TAG_SIZE_M,
    annotate_apriltag_poses,
    camera_world_pose_from_tag,
    detect_apriltag_poses,
    load_calibration,
    undistort_image,
    wall_tag_rotation_world_from_tag,
)

PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CALIBRATION = (
    PROJECT_ROOT
    / "assets/reference/camera-calibration/2026-08-14-imx500-4056x3040/calibration.json"
)
CAPTURE_PATH = Path("/tmp/apriltag-pose-capture.jpg")


def _cv2():
    try:
        import cv2
    except ImportError as exc:
        raise RuntimeError(
            "OpenCV with aruco support is required; use system python3 on the Raspberry Pi"
        ) from exc
    return cv2


def capture_image(path: Path, size: tuple[int, int]) -> None:
    try:
        from picamera2 import Picamera2
    except ImportError as exc:
        raise RuntimeError("Picamera2 is required when --input is not provided") from exc

    camera = Picamera2()
    try:
        config = camera.create_still_configuration(main={"size": size})
        camera.configure(config)
        camera.start()
        time.sleep(1.5)
        camera.capture_file(str(path))
    finally:
        camera.stop()
        camera.close()


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Capture or load one image and estimate AprilTag 36h11 metric poses."
    )
    parser.add_argument("--input", type=Path, help="existing image instead of the Pi camera")
    parser.add_argument("--calibration", type=Path, default=DEFAULT_CALIBRATION)
    parser.add_argument(
        "--tag-size-mm",
        type=float,
        default=DEFAULT_TAG_SIZE_M * 1000.0,
        help="measured outer black-square side length (default: 70 mm)",
    )
    parser.add_argument("--annotated-out", type=Path, default=Path("/tmp/apriltag-pose.jpg"))
    parser.add_argument(
        "--undistorted-out", type=Path, default=Path("/tmp/apriltag-undistorted.jpg")
    )
    parser.add_argument(
        "--world-anchor-id",
        type=int,
        help="tag ID whose known wall position anchors the room coordinate frame",
    )
    parser.add_argument(
        "--anchor-height-cm",
        type=float,
        help="anchor tag center height above the floor",
    )
    parser.add_argument(
        "--anchor-clockwise-deg",
        type=float,
        default=0.0,
        help="canonical marker rotation on the wall when viewed from the room",
    )
    args = parser.parse_args()
    if args.tag_size_mm <= 0:
        parser.error("--tag-size-mm must be positive")
    if (args.world_anchor_id is None) != (args.anchor_height_cm is None):
        parser.error("--world-anchor-id and --anchor-height-cm must be used together")
    if args.anchor_height_cm is not None and args.anchor_height_cm < 0:
        parser.error("--anchor-height-cm must not be negative")

    calibration = load_calibration(args.calibration)
    image_path = args.input or CAPTURE_PATH
    if args.input is None:
        print("Capturing a static 4056x3040 frame; motors are not accessed...")
        capture_image(image_path, (calibration.width, calibration.height))
    elif not image_path.is_file():
        parser.error(f"input image does not exist: {image_path}")

    cv2 = _cv2()
    image = cv2.imread(str(image_path))
    if image is None:
        print(f"Could not read image: {image_path}", file=sys.stderr)
        return 1

    tag_size_m = args.tag_size_mm / 1000.0
    poses = detect_apriltag_poses(image, calibration, tag_size_m)
    undistorted = undistort_image(image, calibration)
    annotated = annotate_apriltag_poses(image, poses, calibration, tag_size_m / 2.0)
    if not cv2.imwrite(str(args.annotated_out), annotated):
        print(f"Could not write {args.annotated_out}", file=sys.stderr)
        return 1
    if not cv2.imwrite(str(args.undistorted_out), undistorted):
        print(f"Could not write {args.undistorted_out}", file=sys.stderr)
        return 1

    print(f"Detected {len(poses)} AprilTag(s); measured tag size = {args.tag_size_mm:.1f} mm")
    for pose in poses:
        print(
            f"ID {pose.tag_id}: right={pose.right_m * 100:+.1f} cm  "
            f"down={pose.down_m * 100:+.1f} cm  forward={pose.forward_m * 100:.1f} cm  "
            f"range={pose.range_m * 100:.1f} cm  yaw={pose.yaw_deg:+.1f} deg  "
            f"pitch={pose.pitch_deg:+.1f} deg  roll={pose.roll_deg:+.1f} deg  "
            f"reprojection={pose.reprojection_error_px:.2f} px"
        )
    if args.world_anchor_id is not None:
        anchor_pose = next((pose for pose in poses if pose.tag_id == args.world_anchor_id), None)
        if anchor_pose is None:
            print(f"World anchor ID {args.world_anchor_id} was not detected")
        else:
            world_pose = camera_world_pose_from_tag(
                anchor_pose,
                [0.0, 0.0, args.anchor_height_cm / 100.0],
                wall_tag_rotation_world_from_tag(args.anchor_clockwise_deg),
            )
            print(
                "Camera in room frame (X=away from wall, Y=wall-right, Z=up): "
                f"wall_distance={world_pose.wall_distance_m * 100:.1f} cm  "
                f"wall_right={world_pose.wall_right_m * 100:+.1f} cm  "
                f"height={world_pose.height_m * 100:.1f} cm  "
                f"heading={world_pose.heading_deg:+.1f} deg  "
                f"elevation={world_pose.elevation_deg:+.1f} deg"
            )
    print(f"Annotated image: {args.annotated_out}")
    print(f"Undistorted image: {args.undistorted_out}")
    return 0 if poses else 2


if __name__ == "__main__":
    sys.exit(main())

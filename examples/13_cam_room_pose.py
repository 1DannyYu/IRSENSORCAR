#!/usr/bin/env python3
"""Estimate a stable room pose from a wall ChArUco board and AprilTag anchor.

This is a static camera-only test. It never imports or accesses the car motors.

Run on the Raspberry Pi with its system Python:

    PYTHONPATH=src python3 examples/13_cam_room_pose.py --anchor-height-cm 14.7
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from datetime import UTC, datetime
from pathlib import Path

import numpy as np

from carbot.vision import (
    DEFAULT_TAG_SIZE_M,
    CameraWorldPose,
    aggregate_camera_world_poses,
    anchor_tags,
    annotate_apriltag_poses,
    annotate_charuco_board_pose,
    camera_world_pose_from_wall_board_and_tag,
    detect_apriltag_poses,
    detect_charuco_board_pose,
    load_calibration,
    load_charuco_board_geometry,
    wall_board_rotation_world_from_board,
)

PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CALIBRATION = (
    PROJECT_ROOT
    / "assets/reference/camera-calibration/2026-08-14-imx500-4056x3040/calibration.json"
)


def _cv2():
    try:
        import cv2
    except ImportError as exc:
        raise RuntimeError(
            "OpenCV with aruco support is required; use system python3 on the Raspberry Pi"
        ) from exc
    return cv2


def capture_images(paths: list[Path], size: tuple[int, int], interval_s: float) -> None:
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
        for index, path in enumerate(paths):
            camera.capture_file(str(path))
            if index + 1 < len(paths):
                time.sleep(interval_s)
    finally:
        camera.stop()
        camera.close()


def _pose_json(pose: CameraWorldPose) -> dict[str, object]:
    return {
        "position_m": {
            "away_from_wall_x": float(pose.position_m[0]),
            "wall_right_y": float(pose.position_m[1]),
            "up_z": float(pose.position_m[2]),
        },
        "rotation_world_from_camera": pose.rotation_world_from_camera.tolist(),
        "heading_deg": pose.heading_deg,
        "elevation_deg": pose.elevation_deg,
    }


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Estimate camera room pose from an upright wall board and AprilTag anchor."
    )
    parser.add_argument(
        "--input",
        action="append",
        type=Path,
        help="existing image; repeat for multiple samples instead of capturing",
    )
    parser.add_argument("--calibration", type=Path, default=DEFAULT_CALIBRATION)
    parser.add_argument("--samples", type=int, default=5)
    parser.add_argument("--min-valid", type=int, default=3)
    parser.add_argument("--interval-ms", type=float, default=250.0)
    parser.add_argument("--anchor-id", type=int, default=0)
    parser.add_argument("--anchor-height-cm", type=float, required=True)
    parser.add_argument("--tag-size-mm", type=float, default=DEFAULT_TAG_SIZE_M * 1000.0)
    parser.add_argument("--board-clockwise-deg", type=float, default=0.0)
    parser.add_argument("--max-reprojection-px", type=float, default=3.0)
    parser.add_argument("--max-position-deviation-cm", type=float, default=3.0)
    parser.add_argument("--max-rotation-deviation-deg", type=float, default=3.0)
    parser.add_argument("--json-out", type=Path, default=Path("/tmp/room-pose.json"))
    parser.add_argument("--annotated-out", type=Path, default=Path("/tmp/room-pose.jpg"))
    args = parser.parse_args()

    positive_values = {
        "--samples": args.samples,
        "--min-valid": args.min_valid,
        "--tag-size-mm": args.tag_size_mm,
        "--max-reprojection-px": args.max_reprojection_px,
        "--max-position-deviation-cm": args.max_position_deviation_cm,
        "--max-rotation-deviation-deg": args.max_rotation_deviation_deg,
    }
    for name, value in positive_values.items():
        if not math.isfinite(value) or value <= 0:
            parser.error(f"{name} must be positive and finite")
    if not math.isfinite(args.anchor_height_cm) or args.anchor_height_cm < 0:
        parser.error("--anchor-height-cm must be non-negative and finite")
    if not math.isfinite(args.interval_ms) or args.interval_ms < 0:
        parser.error("--interval-ms must be non-negative and finite")

    calibration = load_calibration(args.calibration)
    geometry = load_charuco_board_geometry(args.calibration)
    if args.input:
        image_paths = args.input
        missing = [str(path) for path in image_paths if not path.is_file()]
        if missing:
            parser.error(f"input image does not exist: {', '.join(missing)}")
    else:
        image_paths = [
            Path(f"/tmp/room-pose-capture-{index:02d}.jpg") for index in range(1, args.samples + 1)
        ]
        print(f"Capturing {len(image_paths)} static frames; motors are not accessed...")
        capture_images(
            image_paths,
            (calibration.width, calibration.height),
            args.interval_ms / 1000.0,
        )
    if args.min_valid > len(image_paths):
        parser.error("--min-valid cannot exceed the number of input or captured images")

    cv2 = _cv2()
    tag_size_m = args.tag_size_mm / 1000.0
    tag_position = np.asarray([0.0, 0.0, args.anchor_height_cm / 100.0])
    world_from_board = wall_board_rotation_world_from_board(args.board_clockwise_deg)
    valid_poses: list[CameraWorldPose] = []
    records: list[dict[str, object]] = []
    valid_images: list[tuple[np.ndarray, object, object]] = []

    for sample_index, image_path in enumerate(image_paths, start=1):
        image = cv2.imread(str(image_path))
        if image is None:
            records.append(
                {"sample": sample_index, "path": str(image_path), "status": "unreadable"}
            )
            continue
        tags = detect_apriltag_poses(image, calibration, tag_size_m)
        anchor_list = anchor_tags(tags, args.anchor_id)
        board = detect_charuco_board_pose(image, calibration, geometry)
        if len(anchor_list) > 1:
            records.append(
                {
                    "sample": sample_index,
                    "path": str(image_path),
                    "status": "duplicate-anchor",
                    "anchor_detections": len(anchor_list),
                }
            )
            continue
        if not anchor_list or board is None:
            missing = "anchor" if not anchor_list else "charuco-board"
            records.append(
                {"sample": sample_index, "path": str(image_path), "status": f"missing-{missing}"}
            )
            continue
        tag = anchor_list[0]
        if max(tag.reprojection_error_px, board.reprojection_error_px) > args.max_reprojection_px:
            records.append(
                {
                    "sample": sample_index,
                    "path": str(image_path),
                    "status": "reprojection-rejected",
                    "tag_reprojection_error_px": tag.reprojection_error_px,
                    "board_reprojection_error_px": board.reprojection_error_px,
                }
            )
            continue
        world_pose = camera_world_pose_from_wall_board_and_tag(
            tag,
            board,
            tag_position,
            world_from_board,
        )
        valid_index = len(valid_poses)
        valid_poses.append(world_pose)
        valid_images.append((image, tag, board))
        records.append(
            {
                "sample": sample_index,
                "path": str(image_path),
                "status": "candidate",
                "candidate_index": valid_index,
                "tag_reprojection_error_px": tag.reprojection_error_px,
                "board_reprojection_error_px": board.reprojection_error_px,
                "charuco_corners": board.corner_count,
                **_pose_json(world_pose),
            }
        )

    if len(valid_poses) < args.min_valid:
        print(
            f"Only {len(valid_poses)} valid frame(s); need at least {args.min_valid}.",
            file=sys.stderr,
        )
        return 2
    aggregate, inlier_indices = aggregate_camera_world_poses(
        valid_poses,
        args.max_position_deviation_cm / 100.0,
        args.max_rotation_deviation_deg,
    )
    inlier_set = set(inlier_indices)
    for record in records:
        candidate_index = record.get("candidate_index")
        if candidate_index is not None:
            record["status"] = "inlier" if candidate_index in inlier_set else "pose-outlier"
            del record["candidate_index"]
    if len(inlier_indices) < args.min_valid:
        print(
            f"Only {len(inlier_indices)} inlier frame(s); need at least {args.min_valid}.",
            file=sys.stderr,
        )
        return 2

    inlier_positions = np.asarray([valid_poses[index].position_m for index in inlier_indices])
    position_range = np.ptp(inlier_positions, axis=0)
    result = {
        "schema_version": 1,
        "created_at": datetime.now(UTC).isoformat(),
        "coordinate_frame": {
            "handedness": "right-handed",
            "x": "away from anchor wall",
            "y": "along anchor wall to the right",
            "z": "up from floor",
            "origin": "floor directly below AprilTag anchor center",
        },
        "anchor": {
            "tag_id": args.anchor_id,
            "tag_size_m": tag_size_m,
            "center_height_m": args.anchor_height_cm / 100.0,
        },
        "board": {
            "squares": [geometry.squares_x, geometry.squares_y],
            "dictionary": geometry.dictionary_name,
            "square_pitch_xy_m": [geometry.square_pitch_x_m, geometry.square_pitch_y_m],
            "clockwise_rotation_deg": args.board_clockwise_deg,
        },
        "sampling": {
            "requested": len(image_paths),
            "valid_before_outlier_rejection": len(valid_poses),
            "inliers": len(inlier_indices),
            "position_range_m": position_range.tolist(),
        },
        "camera_pose": _pose_json(aggregate),
        "samples": records,
    }
    args.json_out.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")

    representative = inlier_indices[len(inlier_indices) // 2]
    image, tag, board = valid_images[representative]
    annotated = annotate_apriltag_poses(image, [tag], calibration, tag_size_m / 2.0)
    annotated = annotate_charuco_board_pose(annotated, board, calibration)
    if not cv2.imwrite(str(args.annotated_out), annotated):
        print(f"Could not write {args.annotated_out}", file=sys.stderr)
        return 1

    print(
        f"Room pose from {len(inlier_indices)}/{len(image_paths)} inlier frames: "
        f"wall_distance={aggregate.wall_distance_m * 100:.2f} cm  "
        f"wall_right={aggregate.wall_right_m * 100:+.2f} cm  "
        f"height={aggregate.height_m * 100:.2f} cm  "
        f"heading={aggregate.heading_deg:+.2f} deg  "
        f"elevation={aggregate.elevation_deg:+.2f} deg"
    )
    print(
        "Inlier position ranges: "
        f"X={position_range[0] * 100:.2f} cm  "
        f"Y={position_range[1] * 100:.2f} cm  "
        f"Z={position_range[2] * 100:.2f} cm"
    )
    print(f"Pose JSON: {args.json_out}")
    print(f"Annotated image: {args.annotated_out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

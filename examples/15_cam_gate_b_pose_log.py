#!/usr/bin/env python3
"""Gate B manual-reposition pose log: repeatability + displacement check.

Gate B (from docs/handoff-2026-08-14-vision-to-mapping.md) repositions the
**powered-off / stopped** car by hand to 3-5 marked floor locations while the
fixed wall anchor stays in view. This script captures static frames at each
location, aggregates the camera room pose (same logic as examples/13), and
prints:

- per-location inlier count and position spread (repeatability, Gate B
  acceptance: spread < 1 cm with the anchor fully visible);
- camera displacement between consecutive locations, to compare against the
  operator's tape measurement (axis/sign contract check);
- a reminder that the previous /tmp/room-pose*.json is stale once the car
  moved.

It never constructs :class:`carbot.Car` and never sends a motor command.

Live capture mode (operator moves the car between locations):

    PYTHONPATH=src python3 examples/15_cam_gate_b_pose_log.py --positions 5 --anchor-height-cm 14.65

Existing-images mode (each subdirectory = one location, frames inside):

    PYTHONPATH=src python3 examples/15_cam_gate_b_pose_log.py --input-dir /tmp/gate-b --anchor-height-cm 14.65

Output: /tmp/gate-b-poses.json
"""

from __future__ import annotations

import argparse
import itertools
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


def capture_frames(count: int, size: tuple[int, int], interval_s: float, prefix: str) -> list[Path]:
    try:
        from picamera2 import Picamera2
    except ImportError as exc:
        raise RuntimeError("Picamera2 is required (use live capture or --input-dir)") from exc

    camera = Picamera2()
    paths: list[Path] = []
    try:
        config = camera.create_still_configuration(main={"size": size})
        camera.configure(config)
        camera.start()
        time.sleep(1.5)
        for index in range(1, count + 1):
            path = Path(f"/tmp/gate-b-{prefix}-{index:02d}.jpg")
            camera.capture_file(str(path))
            paths.append(path)
            if index < count:
                time.sleep(interval_s)
    finally:
        camera.stop()
        camera.close()
    return paths


def process_location(
    image_paths: list[Path],
    args: argparse.Namespace,
    calibration,
    geometry,
    tag_size_m: float,
    tag_position: np.ndarray,
    world_from_board: np.ndarray,
) -> dict[str, object]:
    """Per-frame detection + aggregation; returns one location record."""
    cv2 = _cv2()
    valid_poses: list[CameraWorldPose] = []
    records: list[dict[str, object]] = []
    for image_path in image_paths:
        image = cv2.imread(str(image_path))
        if image is None:
            records.append({"path": str(image_path), "status": "unreadable"})
            continue
        tags = detect_apriltag_poses(image, calibration, tag_size_m)
        anchor_list = anchor_tags(tags, args.anchor_id)
        board = detect_charuco_board_pose(image, calibration, geometry)
        if len(anchor_list) > 1:
            records.append(
                {
                    "path": str(image_path),
                    "status": "duplicate-anchor",
                    "anchor_detections": len(anchor_list),
                }
            )
            continue
        if not anchor_list or board is None:
            missing = "anchor" if not anchor_list else "charuco-board"
            records.append({"path": str(image_path), "status": f"missing-{missing}"})
            continue
        if (
            max(anchor_list[0].reprojection_error_px, board.reprojection_error_px)
            > args.max_reprojection_px
        ):
            records.append(
                {
                    "path": str(image_path),
                    "status": "reprojection-rejected",
                    "tag_reprojection_error_px": anchor_list[0].reprojection_error_px,
                    "board_reprojection_error_px": board.reprojection_error_px,
                }
            )
            continue
        valid_poses.append(
            camera_world_pose_from_wall_board_and_tag(
                anchor_list[0],
                board,
                tag_position,
                world_from_board,
            )
        )
        records.append({"path": str(image_path), "status": "candidate"})

    record: dict[str, object] = {
        "frames": len(image_paths),
        "candidates": len(valid_poses),
        "records": records,
    }
    if len(valid_poses) < args.min_valid:
        record["status"] = "insufficient-valid"
        return record
    aggregate, inlier_indices = aggregate_camera_world_poses(
        valid_poses,
        args.max_position_deviation_cm / 100.0,
        args.max_rotation_deviation_deg,
    )
    inlier_positions = np.asarray([valid_poses[index].position_m for index in inlier_indices])
    spread_cm = np.ptp(inlier_positions, axis=0) * 100.0
    record.update(
        {
            "status": "ok",
            "inliers": len(inlier_indices),
            "spread_cm": spread_cm.tolist(),
            "wall_distance_cm": aggregate.wall_distance_m * 100.0,
            "wall_right_cm": aggregate.wall_right_m * 100.0,
            "height_cm": aggregate.height_m * 100.0,
            "heading_deg": aggregate.heading_deg,
            "elevation_deg": aggregate.elevation_deg,
        }
    )
    return record


def main() -> int:
    parser = argparse.ArgumentParser(description="Gate B manual-reposition pose log")
    parser.add_argument("--positions", type=int, default=5, help="number of marked floor locations")
    parser.add_argument("--frames-per-position", type=int, default=5)
    parser.add_argument(
        "--input-dir",
        type=Path,
        default=None,
        help="existing images: one subdirectory per location",
    )
    parser.add_argument("--calibration", type=Path, default=DEFAULT_CALIBRATION)
    parser.add_argument("--min-valid", type=int, default=3)
    parser.add_argument("--interval-ms", type=float, default=250.0)
    parser.add_argument("--anchor-id", type=int, default=0)
    parser.add_argument("--anchor-height-cm", type=float, required=True)
    parser.add_argument("--tag-size-mm", type=float, default=DEFAULT_TAG_SIZE_M * 1000.0)
    parser.add_argument("--board-clockwise-deg", type=float, default=0.0)
    parser.add_argument("--max-reprojection-px", type=float, default=3.0)
    parser.add_argument("--max-position-deviation-cm", type=float, default=3.0)
    parser.add_argument("--max-rotation-deviation-deg", type=float, default=3.0)
    parser.add_argument("--json-out", type=Path, default=Path("/tmp/gate-b-poses.json"))
    args = parser.parse_args()

    if args.positions < 2:
        parser.error("--positions must be >= 2 to compare displacements")
    positive = {
        "--frames-per-position": args.frames_per_position,
        "--min-valid": args.min_valid,
        "--tag-size-mm": args.tag_size_mm,
        "--max-reprojection-px": args.max_reprojection_px,
        "--max-position-deviation-cm": args.max_position_deviation_cm,
        "--max-rotation-deviation-deg": args.max_rotation_deviation_deg,
    }
    for name, value in positive.items():
        if not math.isfinite(value) or value <= 0:
            parser.error(f"{name} must be positive and finite")
    if args.anchor_height_cm < 0 or not math.isfinite(args.anchor_height_cm):
        parser.error("--anchor-height-cm must be non-negative and finite")

    calibration = load_calibration(args.calibration)
    geometry = load_charuco_board_geometry(args.calibration)
    tag_size_m = args.tag_size_mm / 1000.0
    tag_position = np.asarray([0.0, 0.0, args.anchor_height_cm / 100.0])
    world_from_board = wall_board_rotation_world_from_board(args.board_clockwise_deg)

    location_paths: list[list[Path]] = []
    if args.input_dir is not None:
        if not args.input_dir.is_dir():
            parser.error(f"--input-dir does not exist: {args.input_dir}")
        for position in range(1, args.positions + 1):
            sub = args.input_dir / f"pos{position}"
            if not sub.is_dir():
                parser.error(f"missing location directory: {sub}")
            frames = sorted(sub.glob("*.jpg")) + sorted(sub.glob("*.png"))
            if not frames:
                parser.error(f"no images in {sub}")
            location_paths.append(frames)
    else:
        for position in range(1, args.positions + 1):
            print(
                f"\nLocation {position}/{args.positions}: place the car (powered off / "
                f"stopped) at marked spot {position}, facing the anchor wall, then press "
                f"Enter to capture {args.frames_per_position} frames..."
            )
            input()
            location_paths.append(
                capture_frames(
                    args.frames_per_position,
                    (calibration.width, calibration.height),
                    args.interval_ms / 1000.0,
                    f"pos{position}",
                )
            )

    locations: list[dict[str, object]] = []
    for position, paths in enumerate(location_paths, start=1):
        print(f"\nProcessing location {position}/{len(location_paths)} ({len(paths)} frames)...")
        record = process_location(
            paths, args, calibration, geometry, tag_size_m, tag_position, world_from_board
        )
        locations.append({"position": position, **record})
        if record["status"] == "ok":
            print(
                f"  {record['inliers']}/{record['frames']} inliers, "
                f"spread X={record['spread_cm'][0]:.2f} Y={record['spread_cm'][1]:.2f} "
                f"Z={record['spread_cm'][2]:.2f} cm, "
                f"wall_distance={record['wall_distance_cm']:.2f} cm, "
                f"heading={record['heading_deg']:+.2f} deg"
            )
        else:
            print(f"  {record['status']} ({record['candidates']}/{record['frames']} candidates)")

    print("\n" + "=" * 60)
    print("Location-to-location camera displacement (world X away, Y right):")
    displacement_rows: list[dict[str, object]] = []
    ok_poses = [(loc["position"], loc) for loc in locations if loc["status"] == "ok"]
    for (p1, loc1), (p2, loc2) in itertools.pairwise(ok_poses):
        dx = loc2["wall_distance_cm"] - loc1["wall_distance_cm"]
        dy = loc2["wall_right_cm"] - loc1["wall_right_cm"]
        dist = math.hypot(dx, dy)
        displacement_rows.append(
            {
                "from": p1,
                "to": p2,
                "dX_cm": round(dx, 2),
                "dY_cm": round(dy, 2),
                "distance_cm": round(dist, 2),
            }
        )
        print(f"  pos{p1} -> pos{p2}: dX={dx:+6.2f} cm  dY={dy:+6.2f} cm  distance={dist:5.2f} cm")
    if len(ok_poses) >= 2:
        print(
            "\nCompare each dX/dY against your tape measurement between the marked "
            "floor spots (direction must match the axis signs printed above)."
        )

    result = {
        "schema_version": 1,
        "created_at": datetime.now(UTC).isoformat(),
        "coordinate_frame": {
            "handedness": "right-handed",
            "x": "away from anchor wall",
            "y": "along wall right",
            "z": "up from floor",
        },
        "parameters": {"anchor_height_cm": args.anchor_height_cm, "tag_size_mm": args.tag_size_mm},
        "locations": locations,
        "displacements": displacement_rows,
    }
    args.json_out.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(f"\nFull JSON: {args.json_out}")
    if len(ok_poses) < 2:
        print(
            "Note: fewer than 2 valid locations — the car moved, so any earlier "
            "/tmp/room-pose*.json is stale; do not reuse it as current pose."
        )
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())

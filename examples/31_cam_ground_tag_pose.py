#!/usr/bin/env python3
"""Stationary ground-tag pose check: locate the camera on the Task-1 map.

**No motors** — safe to run over SSH. The camera looks at AprilTags lying
flat on the map (printed into the new map, or taped on the current one).
Each mapped tag gives the camera a 6-DOF pose in the map frame, so this
script answers the Phase-0 questions: *can the real camera see the printed
tags at the real mount angles, and does the localization match where the
car actually is?*

    # on the Pi, from the repo root (system python3, OpenCV + picamera2)
    PYTHONPATH=src python3 examples/31_cam_ground_tag_pose.py \
        --tag-map scratch/landmarks/task1-tag-map.json

    # on the Mac, with an existing image
    PYTHONPATH=src uv run --extra vision python examples/31_cam_ground_tag_pose.py \
        --input /tmp/tags.jpg --tag-map scratch/landmarks/task1-tag-map.json

Captures use the **2028x1520 preview stream — the same stream the drive
loop uses** (the 4056x3040 still is unnecessary for pose estimation: a 2 cm
tag at 40 cm is 155 px at 4K vs 78 px at 2K, and both are far beyond the
~3-5 px/module detection floor, so precision is identical for our needs).

Pass --tag-size-mm when no tag map is given (raw detection only). The tag
map JSON format is produced by ``scripts/generate_apriltag_sheet.py
--tag-map-out``; fill in each tag's measured x_m/y_m after placing it. Map
frame: origin at the map's **south-west corner**, x east, y north, metres;
the NE corner is (1.00, 0.70). yaw 0 = tag's N arrow pointing map-north.
See ``src/carbot/landmarks.py``.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

from carbot.landmarks import LandmarkLocalization, load_tag_map, localize_camera
from carbot.vision import (
    annotate_apriltag_poses,
    detect_apriltag_poses,
    load_calibration,
)

PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CALIBRATION = (
    PROJECT_ROOT
    / "assets/reference/camera-calibration/2026-08-14-imx500-4056x3040/calibration.json"
)
DEFAULT_TAG_SIZE_M = 0.020
PREVIEW_SIZE = (2028, 1520)


def _capture(
    size: tuple[int, int],
    exposure_time_us: int = 50_000,
    analogue_gain: float = 4.5,
):
    """Capture one frame with the same fixed-exposure settings as the drive."""
    from picamera2 import Picamera2

    camera = Picamera2()
    camera.configure(camera.create_preview_configuration(main={"size": size}))
    camera.start()
    try:
        camera.set_controls(
            {
                "AeEnable": False,
                "ExposureTime": exposure_time_us,
                "AnalogueGain": analogue_gain,
            }
        )
        time.sleep(0.5)
    except Exception:  # noqa: BLE001 - controls are optional on some builds
        time.sleep(1.5)
    frame = camera.capture_array("main")
    camera.close()
    return frame


def _localization_json(localization: LandmarkLocalization | None) -> dict | None:
    if localization is None:
        return None
    return {
        "x_m": round(localization.x_m, 4),
        "y_m": round(localization.y_m, 4),
        "z_m": round(float(localization.position_m[2]), 4),
        "heading_deg": round(localization.heading_deg, 2),
        "visible_tag_ids": list(localization.visible_tag_ids),
        "inlier_ids": list(localization.inlier_ids),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--input", type=Path, default=None, help="existing image instead of the Pi camera"
    )
    parser.add_argument("--calibration", type=Path, default=DEFAULT_CALIBRATION)
    parser.add_argument(
        "--tag-map", type=Path, default=None, help="tag map JSON; enables map-frame localization"
    )
    parser.add_argument(
        "--tag-size-mm",
        type=float,
        default=DEFAULT_TAG_SIZE_M * 1000.0,
        help="detection size when no tag map is given",
    )
    parser.add_argument("--annotated-out", type=Path, default=Path("/tmp/ground-tag-pose.jpg"))
    parser.add_argument("--json-out", type=Path, default=Path("/tmp/ground-tag-pose.json"))
    args = parser.parse_args()

    if args.tag_size_mm <= 0:
        parser.error("--tag-size-mm must be positive")
    if args.input is not None and not args.input.is_file():
        parser.error(f"input image does not exist: {args.input}")

    tag_map = load_tag_map(args.tag_map) if args.tag_map is not None else None
    detection_size = (
        tag_map.entries[0].size_m
        if tag_map is not None and tag_map.entries
        else args.tag_size_mm / 1000.0
    )

    calibration = load_calibration(args.calibration)
    if args.input is None:
        size = PREVIEW_SIZE
        print(f"Capturing a {size[0]}x{size[1]} frame; motors are not accessed...")
        frame = _capture(size)
        if frame.shape[0] != size[1] or frame.shape[1] != size[0]:
            print(f"camera returned {frame.shape[1]}x{frame.shape[0]}, using that size")
        image = frame[:, :, ::-1]  # RGB -> BGR for the OpenCV pipeline
    else:
        import cv2

        image = cv2.imread(str(args.input))
        if image is None:
            print(f"could not read image: {args.input}", file=sys.stderr)
            return 1
    height, width = image.shape[:2]
    scaled_calibration = (
        calibration.scaled_to(width, height)
        if (width, height) != (calibration.width, calibration.height)
        else calibration
    )

    import cv2

    poses = detect_apriltag_poses(image, scaled_calibration, detection_size)
    localization = (
        localize_camera(poses, scaled_calibration, tag_map) if tag_map is not None else None
    )

    print(f"detected {len(poses)} tag(s): {[pose.tag_id for pose in poses] or 'none'}")
    for pose in poses:
        print(
            f"  tag {pose.tag_id}: range {pose.range_m:.3f} m, "
            f"reprojection {pose.reprojection_error_px:.2f} px"
        )
    if localization is not None:
        print(
            f"camera in map frame: x={localization.x_m:.3f} m  y={localization.y_m:.3f} m  "
            f"heading={localization.heading_deg:.1f} deg "
            f"(inliers {localization.inlier_ids})"
        )
    elif tag_map is not None:
        print("no mapped tag visible; camera pose unknown (check tag placement and focus)")

    annotated = annotate_apriltag_poses(image, poses, scaled_calibration, detection_size / 2.0)
    if localization is not None:
        label = (
            f"cam @ ({localization.x_m:.2f}, {localization.y_m:.2f}) m "
            f"h {localization.heading_deg:.0f} deg"
        )
        cv2.putText(annotated, label, (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 1.4, (0, 255, 0), 3)
    if not cv2.imwrite(str(args.annotated_out), annotated):
        print(f"could not write {args.annotated_out}", file=sys.stderr)
        return 1
    print(f"wrote annotated overlay {args.annotated_out}")

    result = {
        "image": str(args.input) if args.input else "camera capture",
        "resolution": [width, height],
        "tag_map": str(args.tag_map) if args.tag_map else None,
        "localization": _localization_json(localization),
        "observations": [
            {
                "tag_id": pose.tag_id,
                "range_m": round(pose.range_m, 4),
                "reprojection_error_px": round(pose.reprojection_error_px, 4),
            }
            for pose in poses
        ],
    }
    args.json_out.parent.mkdir(parents=True, exist_ok=True)
    args.json_out.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(f"wrote {args.json_out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

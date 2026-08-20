#!/usr/bin/env python3
"""Give a COLMAP reconstruction real size, using the 70 mm wall AprilTag.

COLMAP recovers shape but not scale: a reconstructed room is correctly
proportioned and could be any size. ADR 0002 chose to anchor that with a target
of known dimensions instead of measuring the robot by hand, and the wall tag is
already printed, fixed and measured.

This reads a sparse model, detects AprilTags in the images that registered, and
solves metres per reconstruction unit from the ratio of metric to reconstruction
distances between camera pairs that saw the same tag (see :mod:`carbot.scale`).

Usage:
    uv run --extra vision --extra mapping python scripts/anchor_sfm_scale.py \\
        <sparse-model-dir> <image-dir>

Outputs:
    <model-dir>/scale.json          the factor, its evidence, and metric extents
    <model-dir>/trajectory-m.csv    camera centres in metres

Runs on the workstation only — no robot, no motors.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

from carbot.scale import estimate_scale, scale_positions, tag_frame_positions, trajectory_extent_m
from carbot.vision import DEFAULT_TAG_SIZE_M, detect_apriltag_poses, load_calibration

DEFAULT_CALIBRATION = Path(
    "assets/reference/camera-calibration/2026-08-14-imx500-4056x3040/calibration.json"
)


def main() -> int:
    parser = argparse.ArgumentParser(description="Anchor an SfM model to metric scale")
    parser.add_argument("model_dir", type=Path, help="sparse model directory (contains images.bin)")
    parser.add_argument("image_dir", type=Path, help="folder holding the source images")
    parser.add_argument("--calibration", type=Path, default=DEFAULT_CALIBRATION)
    parser.add_argument(
        "--tag-size-m",
        type=float,
        default=DEFAULT_TAG_SIZE_M,
        help="printed tag's black square edge length in metres",
    )
    parser.add_argument(
        "--max-reprojection-px",
        type=float,
        default=1.0,
        help="discard tag detections whose pose fits this badly; on the first "
        "real model every outlier ratio came from a 2.02 px detection",
    )
    args = parser.parse_args()

    if not args.model_dir.is_dir():
        parser.error(f"model directory does not exist: {args.model_dir}")
    if not args.image_dir.is_dir():
        parser.error(f"image directory does not exist: {args.image_dir}")

    import cv2
    import pycolmap

    try:
        calibration = load_calibration(args.calibration)
    except (OSError, ValueError) as exc:
        print(f"Could not load the camera calibration: {exc}", file=sys.stderr)
        return 1

    reconstruction = pycolmap.Reconstruction(str(args.model_dir))
    centres = {
        image.name: np.asarray(image.projection_center(), dtype=np.float64)
        for image in reconstruction.images.values()
    }
    print(f"Model: {len(centres)} registered images, {reconstruction.num_points3D()} points")

    tag_positions: dict[str, dict[int, np.ndarray]] = {}
    detections_per_tag: dict[int, int] = {}
    discarded = 0
    for name in sorted(centres):
        path = args.image_dir / name
        image = cv2.imread(str(path))
        if image is None:
            print(f"  {name}: unreadable, skipped", file=sys.stderr)
            continue
        poses = detect_apriltag_poses(image, calibration, args.tag_size_m)
        usable = [p for p in poses if p.reprojection_error_px <= args.max_reprojection_px]
        discarded += len(poses) - len(usable)
        if not usable:
            continue
        positions = tag_frame_positions(usable)
        if positions:
            tag_positions[name] = positions
            for tag_id in positions:
                detections_per_tag[tag_id] = detections_per_tag.get(tag_id, 0) + 1

    if not tag_positions:
        print("\nNo usable AprilTag was detected in any registered image.", file=sys.stderr)
        print(
            "The sweep has to actually see the wall tag; re-run the patrol so the "
            "camera passes it, or lower --max-reprojection-px if poses are being "
            "discarded.",
            file=sys.stderr,
        )
        return 1

    print(
        f"Tags: {len(tag_positions)} images carry a usable detection "
        f"({discarded} poses discarded above {args.max_reprojection_px:.1f} px)"
    )
    for tag_id, count in sorted(detections_per_tag.items()):
        print(f"  tag {tag_id}: seen in {count} images")

    estimate = estimate_scale(centres, tag_positions)
    if estimate is None:
        print(
            "\nNo image pair shared a tag with a long enough baseline to solve scale.",
            file=sys.stderr,
        )
        print(
            "A tag needs to be visible from at least two clearly separated viewpoints.",
            file=sys.stderr,
        )
        return 1

    print(f"\nScale: {estimate.describe()}")
    metric = scale_positions(centres, estimate.metres_per_unit)
    extent = trajectory_extent_m(metric)
    print(f"Camera trajectory extent: {extent[0]:.2f} x {extent[1]:.2f} x {extent[2]:.2f} m")
    if not estimate.trustworthy:
        print(
            "\nThe estimate is not trustworthy — treat the extents above as indicative "
            "only. More viewpoints on the same tag is the fix.",
            file=sys.stderr,
        )

    scale_path = args.model_dir / "scale.json"
    scale_path.write_text(
        json.dumps(
            {
                "metres_per_unit": estimate.metres_per_unit,
                "pair_count": estimate.pair_count,
                "tag_ids": list(estimate.tag_ids),
                "relative_spread": estimate.relative_spread,
                "trustworthy": estimate.trustworthy,
                "tag_size_m": args.tag_size_m,
                "registered_images": len(centres),
                "images_with_tag": len(tag_positions),
                "trajectory_extent_m": extent.tolist(),
            },
            indent=2,
        )
        + "\n"
    )

    trajectory_path = args.model_dir / "trajectory-m.csv"
    lines = ["image,x_m,y_m,z_m"]
    lines += [f"{name},{p[0]:.4f},{p[1]:.4f},{p[2]:.4f}" for name, p in sorted(metric.items())]
    trajectory_path.write_text("\n".join(lines) + "\n")

    print(f"\nWrote {scale_path}")
    print(f"Wrote {trajectory_path}")
    return 0 if estimate.trustworthy else 1


if __name__ == "__main__":
    sys.exit(main())

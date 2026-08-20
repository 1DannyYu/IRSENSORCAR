#!/usr/bin/env python3
"""Run COLMAP Structure-from-Motion via pycolmap on a folder of images.

Usage:
    uv run python scripts/run_colmap_sfm.py <image-dir> <work-dir>

Equivalent to the CLI pipeline (feature extraction -> exhaustive matching ->
incremental mapping) but uses the pip-installed ``pycolmap`` wheel, so it runs
without a system ``colmap`` binary. Images must be one scene captured from
multiple, overlapping viewpoints.

Outputs:
    <work-dir>/database.db          COLMAP database
    <work-dir>/sparse/              sparse reconstruction(s)
    <work-dir>/summary.json         per-model camera/point counts
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser(description="Run COLMAP SfM via pycolmap")
    parser.add_argument("image_dir", type=Path, help="folder of input images")
    parser.add_argument("work_dir", type=Path, help="output working directory")
    parser.add_argument(
        "--camera-model", default="SIMPLE_RADIAL", help="COLMAP camera model for self-calibration"
    )
    args = parser.parse_args()

    if not args.image_dir.is_dir():
        parser.error(f"image directory does not exist: {args.image_dir}")
    images = sorted([*args.image_dir.glob("*.jpg"), *args.image_dir.glob("*.png")])
    if len(images) < 3:
        parser.error(f"need >= 3 images, found {len(images)} in {args.image_dir}")

    import pycolmap

    args.work_dir.mkdir(parents=True, exist_ok=True)
    db = args.work_dir / "database.db"
    output_dir = args.work_dir / "sparse"

    reader = pycolmap.ImageReaderOptions()
    reader.camera_model = args.camera_model

    print(f"== feature extraction ({len(images)} images, {args.camera_model}) ==")
    pycolmap.extract_features(
        db, args.image_dir, camera_mode=pycolmap.CameraMode.SINGLE, reader_options=reader
    )

    print("== exhaustive matching ==")
    pycolmap.match_exhaustive(db)

    print("== incremental mapping ==")
    output_dir.mkdir(parents=True, exist_ok=True)
    reconstructions = pycolmap.incremental_mapping(db, args.image_dir, output_dir)

    if not reconstructions:
        print("No sparse model produced (mapper failed or found no scene).", file=sys.stderr)
        return 1

    summary = []
    for rec_id, rec in sorted(reconstructions.items()):
        entry = {
            "model": rec_id,
            "registered_images": rec.num_reg_images(),
            "total_images": len(images),
            "points3D": rec.num_points3D(),
        }
        summary.append(entry)
        print(
            f"  model {rec_id}: {entry['registered_images']}/{entry['total_images']} "
            f"images registered, {entry['points3D']} points"
        )
    (args.work_dir / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(f"Sparse model: {output_dir}")
    print(f"Summary: {args.work_dir / 'summary.json'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

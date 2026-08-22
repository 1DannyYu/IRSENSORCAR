#!/usr/bin/env python3
"""Self-calibrate the camera intrinsics from the Task-1 map's AprilTags.

The 2026-08-14 intrinsics are unusable for tag localization (verified
2026-08-17: even with the camera height fixed to the measured 0.28 m, a
5-DOF joint fit of all visible tags leaves 532 px RMS — the old fx/distortion
do not match this lens). The map's AprilTags are a known metrology target:
their positions are exact to ±5 mm (operator-confirmed), so multi-view
observations of them calibrate the intrinsics with no separate board needed.

Workflow (Pi, no motors):
1. Place the car where several tags are visible.
2. Run this script — it captures ``--views N`` frames, printing guidance.
3. Between frames, ROTATE THE CAR (or pan the camera) ~30-60 deg so the
   views differ. Stationary identical views add no constraints.
4. The script writes a fresh ``calibration.json`` (same schema as the
   existing 2026-08-14 file) for validation and review.

Only tags listed in the tag map are used. Views that see fewer than two tags
are rejected (single-tag views do not constrain intrinsics).

    PYTHONPATH=src python3 examples/ai_camera/33_cam_tag_self_calibrate.py \\
        --views 5 --out /tmp/camera-calibration/2026-08-17-imx500-2028x1520/calibration.json
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np


def _open_camera():
    from picamera2 import Picamera2

    camera = Picamera2()
    camera.configure(camera.create_preview_configuration(main={"size": (2028, 1520)}))
    camera.start()
    time.sleep(1.0)
    return camera


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--views", type=int, default=5, help="frames to capture")
    parser.add_argument(
        "--tag-map",
        type=Path,
        default=Path("scratch/landmarks/task1-tag-map.json"),
        help="tag map JSON (positions are ground truth)",
    )
    parser.add_argument(
        "--out",
        type=Path,
        default=Path("/tmp/camera-calibration/2026-08-17-imx500-2028x1520/calibration.json"),
        help="output calibration JSON path",
    )
    args = parser.parse_args()

    import cv2

    from carbot.landmarks import load_tag_map

    tag_map = load_tag_map(args.tag_map)

    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    detector = cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())
    camera = _open_camera()

    object_points: list[np.ndarray] = []
    image_points: list[np.ndarray] = []
    view_notes: list[dict] = []
    w = h = 0
    try:
        for i in range(args.views):
            print(f"\n=== view {i + 1}/{args.views} ===")
            if i > 0:
                print("> ROTATE the car ~30-60 deg so the tags appear at new angles,")
                print("  then press Enter when ready...")
                input()
            frame = camera.capture_array("main")
            h, w = frame.shape[:2]
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, tag_ids, _ = detector.detectMarkers(gray)
            mapped = []
            if tag_ids is not None:
                for corners4, tag_id in zip(corners, tag_ids.ravel(), strict=True):
                    entry = tag_map.entry(int(tag_id))
                    if entry is not None:
                        mapped.append((corners4.reshape(4, 2), entry))
            if len(mapped) < 2:
                print(f"  only {len(mapped)} mapped tag(s) — need >=2; redo this view")
                i -= 1
                continue
            obj, img = [], []
            for corners4, entry in mapped:
                s = entry.size_m
                local = np.array(
                    [[-s / 2, -s / 2, 0], [s / 2, -s / 2, 0], [s / 2, s / 2, 0], [-s / 2, s / 2, 0]],
                    dtype=np.float64,
                )
                yaw = np.deg2rad(entry.yaw_deg)
                Rz = np.array(
                    [[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]],
                    dtype=np.float64,
                )
                world = (Rz @ local.T).T + np.array(entry.position_m)
                obj.append(world)
                img.append(corners4)
            object_points.append(np.vstack(obj).astype(np.float32))
            image_points.append(np.vstack(img).astype(np.float32))
            view_notes.append({"view": i + 1, "tags": [e.tag_id for _, e in mapped]})
            print(f"  captured {len(mapped)} tags: {[e.tag_id for _, e in mapped]}")
            cv2.imwrite(str(Path("/tmp") / f"selfcal-view-{i + 1}.jpg"), frame)
    finally:
        camera.close()

    if len(object_points) < 2:
        print("need at least 2 good views", file=sys.stderr)
        return 1

    flags = cv2.CALIB_ZERO_TANGENT_DIST
    K0 = np.array([[w, 0, w / 2], [0, w, h / 2], [0, 0, 1]], dtype=np.float64)
    rms, K, dist, _rvecs, _tvecs = cv2.calibrateCamera(
        object_points, image_points, (w, h), K0, np.zeros(5), flags=flags
    )
    print(f"\ncalibrated {len(object_points)} views, {sum(len(o) // 4 for o in object_points)} tags")
    print(f"RMS: {rms:.3f} px")
    print(f"fx={K[0, 0]:.1f} fy={K[1, 1]:.1f} cx={K[0, 2]:.1f} cy={K[1, 2]:.1f}")
    print(f"dist={dist.ravel()}")

    args.out.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "schema_version": 1,
        "camera": {
            "model": "imx500",
            "image_width_px": w,
            "image_height_px": h,
            "rotation_degrees": 0,
        },
        "lens_model": {
            "name": "imx500-lens-2028x1520-selfcal",
            "camera_matrix": K.tolist(),
            "distortion_coefficient_order": "k1,k2,p1,p2,k3",
            "distortion_coefficients": dist.ravel().tolist(),
        },
        "calibration": {
            "date": "2026-08-17",
            "opencv_version": cv2.__version__,
            "rms_reprojection_error_px": rms,
            "method": "map-tag self-calibration (operator-verified tag positions +/-5mm)",
            "tag_map": str(args.tag_map),
            "views": view_notes,
        },
        "usage_notes": [
            "Self-calibrated 2026-08-17 from Task-1 map AprilTags (positions exact +/-5 mm).",
            "Valid for the 2028x1520 preview stream with the camera at the 2026-08-17 tilt.",
            "Recalibrate if the lens focus, module, or capture mode changes.",
        ],
    }
    args.out.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    print(f"wrote {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

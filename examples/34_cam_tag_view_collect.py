#!/usr/bin/env python3
"""Continuously capture AprilTag corners for camera self-calibration.

Run this ON the Pi while the operator slowly rotates the car 360 degrees
(no motors). Every ``--interval`` seconds one frame is captured; each frame's
mapped tag corners are appended to a JSON file. The operator rotates the car
by hand; the script just records.

    PYTHONPATH=src python3 examples/34_cam_tag_view_collect.py \\
        --frames 80 --interval 0.35 --out /tmp/tag-views.json

The JSON is meant to be calibrated offline on the Mac with a shared-intrinsics
bundle adjustment (the intrinsics are a lens property, independent of how the
camera is mounted).
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--frames", type=int, default=80)
    parser.add_argument("--interval", type=float, default=0.35)
    parser.add_argument(
        "--tag-map",
        type=Path,
        default=Path("scratch/landmarks/task1-tag-map.json"),
    )
    parser.add_argument("--out", type=Path, default=Path("/tmp/tag-views.json"))
    args = parser.parse_args()

    import cv2
    from picamera2 import Picamera2

    from carbot.landmarks import load_tag_map

    tag_map = load_tag_map(args.tag_map)
    det = cv2.aruco.ArucoDetector(
        cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11),
        cv2.aruco.DetectorParameters(),
    )
    cam = Picamera2()
    cam.configure(cam.create_preview_configuration(main={"size": (2028, 1520)}))
    cam.start()
    time.sleep(1.0)

    views: list[dict] = []
    w = h = 0
    print("capturing — ROTATE THE CAR NOW (slowly, full 360 deg)")
    try:
        for i in range(args.frames):
            t0 = time.monotonic()
            frame = cam.capture_array("main")
            h, w = frame.shape[:2]
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = det.detectMarkers(gray)
            obs = []
            if ids is not None:
                for c, tag_id in zip(corners, ids.ravel(), strict=True):
                    entry = tag_map.entry(int(tag_id))
                    if entry is not None:
                        obs.append(
                            {
                                "tag_id": int(tag_id),
                                "corners": c.reshape(4, 2).round(2).tolist(),
                            }
                        )
            if obs:
                views.append({"frame": i, "tags": obs})
            if i % 10 == 0 or obs:
                print(f"  frame {i}: {len(obs)} tag(s) "
                      f"({sorted(o['tag_id'] for o in obs)})")
            dt = time.monotonic() - t0
            if dt < args.interval:
                time.sleep(args.interval - dt)
    finally:
        cam.close()

    args.out.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "image_size": [w, h],
        "views": views,
    }
    args.out.write_text(json.dumps(payload), encoding="utf-8")
    print(f"wrote {args.out}: {len(views)} frames with >=1 tag of "
          f"{args.frames} captured")
    return 0


if __name__ == "__main__":
    sys.exit(main())

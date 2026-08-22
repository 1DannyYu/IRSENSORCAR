#!/usr/bin/env python3
"""IMX500 object-detection check for visual obstacle avoidance (no motors).

Loads an on-sensor object detector (SSD mobilenetv2, COCO 80 classes) and prints
every detection plus an "OBSTACLE AHEAD" flag when a large object sits in the
central lower part of the frame — the car's path. This is the visual layer the
sonar cannot provide: a single forward HC-SR04 sees neither thin chair legs nor
an overhead tabletop.

The detection and fusion logic lives in :mod:`carbot.vision_avoid` so the patrol
shares exactly this verdict; this script is the hardware check around it.

Run on the Pi while the operator places/removes obstacles in front of the car:

    PYTHONPATH=src python3 examples/ai_camera/20_cam_detection_check.py --frames 30

Models available on this Pi:
    /usr/share/imx500-models/imx500_network_ssd_mobilenetv2_fpnlite_320x320_pp.rpk
    /usr/share/imx500-models/imx500_network_nanodet_plus_416x416_pp.rpk
"""

from __future__ import annotations

import argparse
import sys
import time

from carbot.vision_avoid import (
    ObstaclePolicy,
    blocking_detections,
    detections_from_metadata,
)

DEFAULT_MODEL = "/usr/share/imx500-models/imx500_network_ssd_mobilenetv2_fpnlite_320x320_pp.rpk"


def main() -> int:
    parser = argparse.ArgumentParser(description="IMX500 visual obstacle detection check")
    parser.add_argument("--model", default=DEFAULT_MODEL)
    parser.add_argument("--frames", type=int, default=30, help="frames to analyse")
    parser.add_argument("--interval", type=float, default=1.0, help="seconds between reads")
    parser.add_argument(
        "--threshold", type=float, default=0.30, help="detection confidence threshold"
    )
    parser.add_argument("--iou", type=float, default=0.65)
    parser.add_argument("--max-detections", type=int, default=10)
    parser.add_argument(
        "--center-x",
        type=float,
        default=0.35,
        help="obstacle if the box centre x is within this fraction of frame centre",
    )
    parser.add_argument(
        "--min-height-frac",
        type=float,
        default=0.45,
        help="obstacle if the box bottom is below this fraction of frame height",
    )
    parser.add_argument(
        "--min-area-frac",
        type=float,
        default=0.06,
        help="obstacle only if the box covers at least this fraction of the frame",
    )
    args = parser.parse_args()

    from picamera2 import Picamera2
    from picamera2.devices import IMX500
    from picamera2.devices.imx500 import NetworkIntrinsics

    policy = ObstaclePolicy(
        confidence_threshold=args.threshold,
        center_x_fraction=args.center_x,
        min_bottom_fraction=args.min_height_frac,
        min_area_fraction=args.min_area_frac,
    )

    imx500 = IMX500(args.model)
    intrinsics = imx500.network_intrinsics
    if not intrinsics:
        intrinsics = NetworkIntrinsics()
        intrinsics.task = "object detection"
    if intrinsics.task != "object detection":
        print(f"Model is not an object-detection network (task={intrinsics.task})", file=sys.stderr)
        return 1
    intrinsics.update_with_defaults()

    picam2 = Picamera2(imx500.camera_num)
    config = picam2.create_preview_configuration(
        controls={"FrameRate": intrinsics.inference_rate}, buffer_count=12
    )
    imx500.show_network_fw_progress_bar()
    picam2.start(config)
    frame_width, frame_height = picam2.camera_configuration()["main"]["size"]
    time.sleep(1.0)

    print(f"Visual detection check: {args.frames} reads, model={args.model.split('/')[-1]}")
    print("=" * 60)
    try:
        for i in range(args.frames):
            metadata = picam2.capture_metadata()
            detections = detections_from_metadata(
                metadata,
                imx500,
                intrinsics,
                picam2,
                policy,
                iou=args.iou,
                max_detections=args.max_detections,
            )
            blocking = blocking_detections(detections, frame_width, frame_height, policy)
            for detection in detections:
                area = detection.area_fraction(frame_width, frame_height)
                mark = "*" if detection in blocking else " "
                print(
                    f" {mark}{detection.label():14s} conf={detection.confidence:.2f} "
                    f"box=({detection.x},{detection.y},{detection.width},{detection.height}) "
                    f"centre_x={detection.center_x:.0f} bottom={detection.bottom} "
                    f"area={area:.3f}"
                )
            print(f"[{i + 1}] -> {'OBSTACLE AHEAD' if blocking else 'clear'}")
            time.sleep(args.interval)
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        picam2.stop()
        picam2.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())

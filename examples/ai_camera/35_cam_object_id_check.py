#!/usr/bin/env python3
"""Time and grade the IMX500 on-sensor object detector for a one-shot "what's on the
table" check (no motors).

Answers two questions the health check in `examples/ai_camera/05_ai_camera_check.py` does not:
how long does a single identification actually take, and how confident is it. The
120 s in that script is an arbitrary soak-test duration, not a per-shot latency
figure — this script measures the real thing by timing camera start, the one-time
IMX500 network firmware upload, and then each subsequent inference frame separately.

Detection and box logic is shared with the patrol via :mod:`carbot.vision_avoid`
(ported from `examples/ai_camera/20_cam_detection_check.py`); this script adds timing and a
summary instead of the obstacle-fusion verdict.

Run on the Pi:

    PYTHONPATH=src python3 examples/ai_camera/35_cam_object_id_check.py
    PYTHONPATH=src python3 examples/ai_camera/35_cam_object_id_check.py --frames 20

This Pi's `/usr/share/imx500-models/` has no YOLO `.rpk` (checked 2026-08-17 — only SSD
mobilenetv2-fpnlite and NanoDet Plus are present as detection models, the rest are
classification/segmentation/pose). Default model is therefore the SSD mobilenetv2
already verified working by `examples/20-22` — COCO-ish 90-index label space, off by
several entries from the standard 80-class list (see `Detection` docstring in
`carbot.vision_avoid`), which the printed labels already account for.
"""

from __future__ import annotations

import argparse
import statistics
import sys
import time
from dataclasses import dataclass

from carbot.vision_avoid import Detection, ObstaclePolicy, detections_from_metadata

DEFAULT_MODEL = "/usr/share/imx500-models/imx500_network_ssd_mobilenetv2_fpnlite_320x320_pp.rpk"


@dataclass
class FrameResult:
    index: int
    seconds: float
    detections: list[Detection]


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Time and grade one-shot IMX500 object identification"
    )
    parser.add_argument("--model", default=DEFAULT_MODEL)
    parser.add_argument(
        "--frames", type=int, default=10, help="timed frames after warmup (default 10)"
    )
    parser.add_argument(
        "--warmup",
        type=int,
        default=5,
        help="frames to discard before timing starts, waiting for a stable tensor (default 5)",
    )
    parser.add_argument("--threshold", type=float, default=0.30, help="confidence threshold")
    parser.add_argument("--iou", type=float, default=0.65)
    parser.add_argument("--max-detections", type=int, default=10)
    args = parser.parse_args()

    from picamera2 import Picamera2
    from picamera2.devices import IMX500
    from picamera2.devices.imx500 import NetworkIntrinsics

    policy = ObstaclePolicy(confidence_threshold=args.threshold)

    t_setup_start = time.monotonic()
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
    t_camera_ready = time.monotonic()

    print(f"Model: {args.model.split('/')[-1]}")
    print(f"Camera + firmware-upload cold start: {t_camera_ready - t_setup_start:.2f} s")
    print()

    try:
        # Warmup: the network needs a handful of frames after firmware upload
        # before metadata carries a valid tensor (examples/21, "no tensor yet").
        warmup_start = time.monotonic()
        for _ in range(args.warmup):
            metadata = picam2.capture_metadata()
            detections_from_metadata(
                metadata, imx500, intrinsics, picam2, policy, iou=args.iou,
                max_detections=args.max_detections,
            )
        warmup_seconds = time.monotonic() - warmup_start
        print(f"Warmup ({args.warmup} frames): {warmup_seconds:.2f} s")
        print()

        results: list[FrameResult] = []
        for i in range(args.frames):
            frame_start = time.monotonic()
            metadata = picam2.capture_metadata()
            detections = detections_from_metadata(
                metadata, imx500, intrinsics, picam2, policy, iou=args.iou,
                max_detections=args.max_detections,
            )
            frame_seconds = time.monotonic() - frame_start
            results.append(FrameResult(i + 1, frame_seconds, detections))

            labels = ", ".join(
                f"{d.label()}({d.confidence:.2f})" for d in detections
            ) or "(nothing above threshold)"
            print(f"[frame {i + 1:2d}] {frame_seconds * 1000:6.1f} ms  {labels}")
    except KeyboardInterrupt:
        print("\nInterrupted.")
        results = []
    finally:
        picam2.stop()
        picam2.close()

    if not results:
        return 0

    print()
    print("=" * 60)
    times = [r.seconds for r in results]
    print(
        f"Per-frame latency over {len(results)} frames: "
        f"mean {statistics.mean(times) * 1000:.1f} ms, "
        f"min {min(times) * 1000:.1f} ms, max {max(times) * 1000:.1f} ms "
        f"(~{1 / statistics.mean(times):.1f} fps)"
    )

    best_confidence: dict[str, float] = {}
    seen_counts: dict[str, int] = {}
    for r in results:
        for d in r.detections:
            label = d.label()
            best_confidence[label] = max(best_confidence.get(label, 0.0), d.confidence)
            seen_counts[label] = seen_counts.get(label, 0) + 1

    if best_confidence:
        print(f"Distinct labels seen (of {len(results)} frames, threshold {args.threshold}):")
        for label, conf in sorted(best_confidence.items(), key=lambda kv: -kv[1]):
            print(f"  {label:14s} best_conf={conf:.2f}  seen in {seen_counts[label]}/{len(results)} frames")
    else:
        print(f"No detections above threshold {args.threshold} in any frame.")
    print(f"Frame size: {frame_width}x{frame_height}")

    return 0


if __name__ == "__main__":
    sys.exit(main())

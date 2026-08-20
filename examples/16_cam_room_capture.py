#!/usr/bin/env python3
"""Capture a room sweep of stills for Structure-from-Motion (SfM).

Push the **powered-off / stopped** car slowly around the room; this script
captures a still every ``--interval`` seconds so neighbouring frames overlap.
It never accesses the motors.

Run on the Pi:

    PYTHONPATH=src python3 examples/16_cam_room_capture.py --duration 90 --interval 3

Walk one small step (30-50 cm) then pause briefly at each beat so most frames
are sharp; keep the wall AprilTag (ID 0, 70 mm) in view for as many frames as
you can — it will be the real-scale anchor later. Aim for 20-40 frames that
cover all four walls with textured surfaces (tiles, furniture edges), not
only blank walls.

Output: one frame-NNN.jpg per capture under --out-dir (default /tmp/room-sfm).
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser(description="Capture a room sweep for SfM")
    parser.add_argument("--interval", type=float, default=3.0, help="seconds between captures")
    parser.add_argument(
        "--duration", type=float, default=90.0, help="total capture duration in seconds"
    )
    parser.add_argument(
        "--count", type=int, default=0, help="stop after this many frames (0 = use --duration)"
    )
    parser.add_argument(
        "--size",
        default="2028x1520",
        help="capture size WxH (default 2028x1520 = half-scale 4:3, "
        "~1080p class; use 4056x3040 for full resolution)",
    )
    parser.add_argument("--out-dir", type=Path, default=Path("/tmp/room-sfm"))
    args = parser.parse_args()

    width, height = (int(v) for v in args.size.split("x"))

    try:
        from picamera2 import Picamera2
    except ImportError as exc:
        raise RuntimeError("Picamera2 is required; run on the Pi with system python3") from exc

    camera = Picamera2()
    try:
        config = camera.create_still_configuration(main={"size": (width, height)})
        camera.configure(config)
        camera.start()
        time.sleep(1.5)
    except Exception:
        camera.close()
        raise

    args.out_dir.mkdir(parents=True, exist_ok=True)
    print(
        f"Capturing every {args.interval}s into {args.out_dir} ("
        f"duration {args.duration}s, size {width}x{height})..."
    )
    print("Walk slowly; pause a beat at each capture so frames stay sharp. Ctrl-C to stop early.")
    n = 0
    t0 = time.monotonic()
    try:
        while True:
            if (args.count and n >= args.count) or time.monotonic() - t0 >= args.duration:
                break
            path = args.out_dir / f"frame-{n:03d}.jpg"
            camera.capture_file(str(path))
            print(f"[{n + 1}] {path.name}")
            n += 1
            time.sleep(args.interval)
    except KeyboardInterrupt:
        print("\nStopped early.")
    finally:
        camera.stop()
        camera.close()
    print(f"Captured {n} frames -> {args.out_dir}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

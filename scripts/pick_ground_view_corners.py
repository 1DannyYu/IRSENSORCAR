#!/usr/bin/env python3
"""Click four floor points on a still to build a ``--corners`` string.

``examples/ai_camera/27_cam_ground_view_calibrate.py --corners`` wants eight pixel numbers
(TL,TR,BR,BL of a rectangle measured on the floor). Reading those off an image
by eye is error-prone at 2028x1520+. This opens the still in an OpenCV window,
scaled to fit the screen, and turns four clicks (in TL, TR, BR, BL order) into
the exact ``--corners`` value at full image resolution.

Runs on the workstation only — no robot, no motors, no camera.

Usage:
    uv run --extra vision python scripts/pick_ground_view_corners.py \\
        scratch/line-follow-2026-08-15/line-follow-raw.jpg

Click TL, TR, BR, BL of a rectangle you have actually measured on the floor
(e.g. a taped rectangle, or two tick marks a known distance apart on the
track paper). Press 'r' to restart the four clicks, 'q' or Esc to quit
without printing anything.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import cv2
import numpy as np

CORNER_LABELS = ("TL", "TR", "BR", "BL")
MAX_DISPLAY_SIZE = 1200


def _pick(image: np.ndarray) -> list[tuple[float, float]] | None:
    height, width = image.shape[:2]
    scale = min(1.0, MAX_DISPLAY_SIZE / max(height, width))
    display = cv2.resize(image, (round(width * scale), round(height * scale)))
    clicks: list[tuple[float, float]] = []

    def on_click(event: int, x: int, y: int, _flags: int, _param: object) -> None:
        if event == cv2.EVENT_LBUTTONDOWN and len(clicks) < 4:
            clicks.append((x / scale, y / scale))

    window = "pick TL, TR, BR, BL - r to restart, q/Esc to quit"
    cv2.namedWindow(window)
    cv2.setMouseCallback(window, on_click)

    while True:
        frame = display.copy()
        for i, (x, y) in enumerate(clicks):
            point = (round(x * scale), round(y * scale))
            cv2.circle(frame, point, 6, (0, 255, 0), -1)
            cv2.putText(
                frame,
                CORNER_LABELS[i],
                (point[0] + 8, point[1] - 8),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2,
            )
        cv2.imshow(window, frame)
        key = cv2.waitKey(20) & 0xFF
        if key in (ord("q"), 27):
            cv2.destroyAllWindows()
            return None
        if key == ord("r"):
            clicks.clear()
        if len(clicks) == 4:
            cv2.imshow(window, frame)
            cv2.waitKey(300)
            cv2.destroyAllWindows()
            return clicks


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("image", type=Path, help="still to click corners on")
    args = parser.parse_args()

    image = cv2.imread(str(args.image), cv2.IMREAD_COLOR)
    if image is None:
        print(f"could not read {args.image}", file=sys.stderr)
        return 1

    print(f"{args.image}  ({image.shape[1]}x{image.shape[0]})")
    print("Click the floor rectangle's corners in order: TL, TR, BR, BL.")
    clicks = _pick(image)
    if clicks is None:
        print("cancelled", file=sys.stderr)
        return 1

    corners = ",".join(f"{x:.0f},{y:.0f}" for x, y in clicks)
    print(f"\n--corners {corners}")
    print(
        "\nFull command (fill in --size-m for your measured rectangle and "
        "--near-m for its near-edge distance from the front wheels):\n"
        f"  PYTHONPATH=src python3 examples/ai_camera/27_cam_ground_view_calibrate.py \\\n"
        f"    --image {args.image} --corners {corners} \\\n"
        "    --size-m WIDTH_M,HEIGHT_M --near-m NEAR_M"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())

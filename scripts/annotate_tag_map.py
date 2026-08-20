#!/usr/bin/env python3
"""Overlay the Task-1 tag placements onto the corrected 100x70 route map.

Reads a tag-map JSON (``x_m``/``y_m`` in metres from the map's WEST/SOUTH
edges, ``yaw_deg``) and the corrected orthophoto, then draws each tag's
footprint (20 mm tag + 5 mm quiet zone = 30 mm), its N arrow, and its ID so
an operator can see at a glance where each AprilTag is taped.

Map conventions (see docs/adr/0003-landmark-localization-task1.md):
  - orthophoto 1000x700 px, 10 px = 1 cm, NW origin, y down
  - map frame: X east, Y north, origin SW corner, NE = (1.00, 0.70)
  - px = x_m * 1000, py = (0.70 - y_m) * 1000

Run:
    uv run python3 scripts/annotate_tag_map.py \
        --map assets/reference/map-1/2026-08-21-ir-route-map.png \
        --tag-map scratch/landmarks/task1-tag-map-draft.json \
        --output scratch/landmarks/task1-tag-placement-map.png
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import numpy as np

MAP_NORTH_Y_M = 0.70  # NE corner y, metres
PX_PER_M = 1000.0  # 1000 px = 1.00 m = 100 cm


def map_to_px(x_m: float, y_m: float) -> tuple[int, int]:
    """Map frame (SW origin, x east / y north, metres) -> pixel (NW origin, y down)."""
    px = round(x_m * PX_PER_M)
    py = round((MAP_NORTH_Y_M - y_m) * PX_PER_M)
    return px, py


def draw_tag(img, x_m, y_m, yaw_deg, tag_id, size_m=0.02, quiet_m=0.005):
    """Draw one tag footprint + N arrow + ID label at its map position."""
    cx, cy = map_to_px(x_m, y_m)
    half_foot = round((size_m + 2 * quiet_m) * PX_PER_M / 2.0)  # 30 mm -> 15 px
    half_tag = round(size_m * PX_PER_M / 2.0)  # 20 mm -> 10 px

    MAGENTA = (203, 0, 255)
    DARK = (60, 0, 90)
    WHITE = (255, 255, 255)

    # Footprint (quiet-zone border, the 30 mm square to cut out).
    cv2.rectangle(
        img,
        (cx - half_foot, cy - half_foot),
        (cx + half_foot, cy + half_foot),
        MAGENTA,
        2,
        cv2.LINE_AA,
    )
    # The tag itself (20 mm).
    cv2.rectangle(
        img, (cx - half_tag, cy - half_tag), (cx + half_tag, cy + half_tag), MAGENTA, 1, cv2.LINE_AA
    )
    # Centre cross.
    cv2.line(img, (cx - 4, cy), (cx + 4, cy), MAGENTA, 1, cv2.LINE_AA)
    cv2.line(img, (cx, cy - 4), (cx, cy + 4), MAGENTA, 1, cv2.LINE_AA)

    # N arrow: yaw 0 = pointing map-north (up in pixels, y down).
    theta = np.deg2rad(yaw_deg)
    dx, dy = np.sin(theta), -np.cos(theta)
    tip = (round(cx + dx * 26), round(cy + dy * 26))
    cv2.arrowedLine(img, (cx, cy), tip, MAGENTA, 2, cv2.LINE_AA, tipLength=0.45)

    # ID label with a white background box.
    text = f"ID {tag_id}"
    font = cv2.FONT_HERSHEY_SIMPLEX
    scale = 0.5
    thick = 1
    (tw, th), baseline = cv2.getTextSize(text, font, scale, thick)
    lx = cx + half_foot + 4
    ly = cy - half_foot - 6
    # keep the label inside the image
    if lx + tw + 6 > img.shape[1]:
        lx = cx - half_foot - tw - 10
    if ly - th - 6 < 0:
        ly = cy + half_foot + th + 10
    cv2.rectangle(img, (lx - 3, ly - th - 4), (lx + tw + 3, ly + baseline + 4), WHITE, -1)
    cv2.rectangle(img, (lx - 3, ly - th - 4), (lx + tw + 3, ly + baseline + 4), DARK, 1)
    cv2.putText(img, text, (lx, ly), font, scale, DARK, thick, cv2.LINE_AA)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--map",
        type=Path,
        default=Path("assets/reference/map-1/2026-08-21-ir-route-map.png"),
    )
    parser.add_argument(
        "--tag-map", type=Path, default=Path("scratch/landmarks/task1-tag-map-draft.json")
    )
    parser.add_argument(
        "--output", type=Path, default=Path("scratch/landmarks/task1-tag-placement-map.png")
    )
    args = parser.parse_args()

    if not args.map.exists():
        parser.error(f"--map not found: {args.map}")
    if not args.tag_map.exists():
        parser.error(f"--tag-map not found: {args.tag_map}")

    data = json.loads(args.tag_map.read_text(encoding="utf-8"))
    img = cv2.imread(str(args.map))
    if img is None:
        parser.error(f"could not read image: {args.map}")

    for tag in data.get("tags", []):
        draw_tag(
            img,
            float(tag["x_m"]),
            float(tag["y_m"]),
            float(tag.get("yaw_deg", 0)),
            tag["id"],
            size_m=float(tag.get("size_m", 0.02)),
        )

    args.output.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(args.output), img)
    print(f"wrote {args.output} ({len(data.get('tags', []))} tags annotated)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
"""Render the two-lap variant of the Map1 Task-1 route diagram.

The canonical Map 1 image under ``tasks/ir-sensor-tracking/`` is kept as the
base image: the red route line is identical for both laps. Only the annotations
change, because the second lap re-uses the same track:

    lap 1  Phase 1 stem -> ... -> Phase 10 return
    join   at the T junction the car does NOT turn into the stem; driving
           straight on IS lap 2's Phase 2 (same heading, same line)
    lap 2  Phase 2 -> ... -> Phase 10 return -> STOP at the T junction

There is no Phase 11 return stem in this variant.

Run:
    uv run --with pillow python3 scripts/generate_map1_two_lap_diagram.py
"""

from __future__ import annotations

import argparse
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

BOLD_FONT = "/System/Library/Fonts/Supplemental/Arial Bold.ttf"
REGULAR_FONT = "/System/Library/Fonts/Supplemental/Arial Unicode.ttf"

RED = (200, 0, 0)
BLUE = (0, 60, 190)
GREEN = (0, 110, 0)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)

T_JUNCTION = (702, 405)  # measured from the red route line in the base image
TITLE_BOX = (248, 17, 631, 36)  # old "corrected route v2" caption
PHASE11_BOX = (617, 546, 780, 567)  # old "Phase 11: return stem 10 cm" caption

# 2026-08-20: operator re-measured these four straight segments directly on the physical
# track; the printed labels below (from the original map generation) were off. Box
# coordinates measured from the red-bordered caption's own pixel extent (see
# scripts/ scratch measurement), not eyeballed.
PHASE_CORRECTIONS = [
    # (cover_box, new_label_text)
    ((555, 38, 685, 57), "Phase 6: west 47.0 cm"),
    ((848, 313, 982, 340), "Phase 4: north 18.0 cm"),
    ((755, 427, 882, 444), "Phase 2: east 15.5 cm"),
    ((495, 432, 641, 449), "Phase 10: return 21.5 cm"),
    # All three arcs measured the same shape/length (~12cm centreline), only position differs.
    ((798, 363, 928, 388), "ARC 1 SE corner ~12 cm"),
    ((838, 108, 976, 133), "ARC 2 NE corner ~12 cm"),
    ((146, 76, 323, 108), "ARC 3 NW corner ~12 cm"),
]

LEGEND_LINES = [
    ("TWO-LAP ROUTE  —  lap 1 runs straight into lap 2 at the T junction; "
     "no Phase 11 return stem.", BLACK),
    ("LAP 1   Phase 1 stem 10.0 → T right 90° → Phase 2 east 15.5 → ARC 1 → "
     "Phase 4 north 18.0 → ARC 2 → Phase 6 west 47.0 → ARC 3 →", BLACK),
    ("            Phase 8 entry 7.5 → Phase 9 roundabout 78.9 (270°, 33.5cm inner dia.) → "
     "exit right 90° → Phase 10 return 21.5", BLACK),
    ("JOIN 1)  at the T junction do NOT turn into the stem — keep going straight; "
     "that is lap 2's Phase 2 east 15.5.", BLUE),
    ("LAP 2   Phase 2 → ARC 1 → Phase 4 → ARC 2 → Phase 6 → ARC 3 → Phase 8 → "
     "Phase 9 roundabout → exit right 90° → Phase 10 return 21.5", BLACK),
    ("FINISH 2)  stop at the T junction when lap 2's Phase 10 ends. "
     "The stem (Phase 1) is driven once, at the start only.", RED),
]


def load_font(path: str, size: int) -> ImageFont.FreeTypeFont:
    try:
        return ImageFont.truetype(path, size)
    except OSError:
        return ImageFont.load_default()


def label(
    draw: ImageDraw.ImageDraw,
    xy: tuple[int, int],
    text: str,
    font: ImageFont.FreeTypeFont,
    color=RED,
    pad: int = 4,
) -> tuple[int, int, int, int]:
    """White label box with a coloured border, top-left anchored at ``xy``."""
    x, y = xy
    l, t, r, b = draw.textbbox((0, 0), text, font=font)
    box = (x, y, x + (r - l) + 2 * pad, y + (b - t) + 2 * pad)
    draw.rectangle(box, fill=WHITE, outline=color, width=2)
    draw.text((x + pad - l, y + pad - t), text, font=font, fill=color)
    return box


def annotate(base: Image.Image) -> Image.Image:
    img = base.convert("RGB").copy()
    draw = ImageDraw.Draw(img)
    f_title = load_font(BOLD_FONT, 12)
    f_label = load_font(BOLD_FONT, 13)
    f_small = load_font(BOLD_FONT, 11)
    f_legend = load_font(BOLD_FONT, 11)
    f_footer = load_font(REGULAR_FONT, 12)

    # 1. Replace the single-lap title caption.
    draw.rectangle(TITLE_BOX, fill=WHITE)
    label(
        draw,
        (TITLE_BOX[0], TITLE_BOX[1]),
        "Map 100 cm (E-W) x 70 cm (N-S) | 10 px = 1 cm | TWO-LAP route",
        f_title,
        color=GREEN,
        pad=3,
    )

    # 1b. Overwrite the four re-measured phase-length captions.
    for box, text in PHASE_CORRECTIONS:
        draw.rectangle(box, fill=WHITE)
        label(draw, (box[0], box[1]), text, f_label, color=RED, pad=2)

    # 2. Kill the old Phase 11 caption.
    draw.rectangle(PHASE11_BOX, fill=WHITE)
    label(
        draw,
        (PHASE11_BOX[0] - 40, PHASE11_BOX[1]),
        "no Phase 11 — the car never returns to the stem",
        f_small,
        color=BLACK,
    )

    # 3. Mark the T junction: loop point for lap 1, finish point for lap 2.
    tx, ty = T_JUNCTION
    draw.ellipse((tx - 13, ty - 13, tx + 13, ty + 13), outline=BLUE, width=4)
    draw.ellipse((tx - 5, ty - 5, tx + 5, ty + 5), fill=BLUE)
    label(
        draw,
        (362, 370),
        "1) lap 1 → lap 2: straight through, no turn",
        f_label,
        color=BLUE,
    )
    label(draw, (740, 452), "2) STOP after lap 2", f_label, color=RED)

    # Straight-through arrow along the return/east line, over the T junction.
    draw.line((tx - 46, ty, tx + 54, ty), fill=BLUE, width=5)
    draw.polygon(
        [(tx + 66, ty), (tx + 50, ty - 9), (tx + 50, ty + 9)], fill=BLUE
    )

    # 4. Footer legend band.
    x0, y0, x1, y1 = 14, 578, 986, 696
    draw.rectangle((x0, y0, x1, y1), fill=WHITE, outline=BLACK, width=2)
    y = y0 + 8
    for text, color in LEGEND_LINES:
        draw.text((x0 + 10, y), text, font=f_legend, fill=color)
        y += 15
    draw.text(
        (x0 + 10, y + 4),
        "Lap 1 Phase 10 continues directly into lap 2 Phase 2; "
        "stop at the T junction after lap 2 Phase 10.",
        font=f_footer,
        fill=BLACK,
    )
    return img


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--base",
        type=Path,
        default=Path("assets/reference/map-1/2026-08-21-ir-route-map.png"),
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("tasks/ir-sensor-tracking/map-1-two-lap-route-diagram.png"),
    )
    args = parser.parse_args()

    img = annotate(Image.open(args.base))
    img.save(args.output)
    print(f"wrote {args.output} ({img.width}x{img.height})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

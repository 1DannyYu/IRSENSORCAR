#!/usr/bin/env python3
"""Generate a printable AprilTag 36h11 sheet for Task-1 ground landmarks.

The sheet prints at **exact millimetre scale** (A4 PDF): each tag carries a
100 mm scale bar so the print scale can be verified with a ruler before
cutting. A tag taped with its **N arrow pointing map-north** (ID upright
when viewed from the north) has ``yaw_deg = 0`` in the tag map; rotate it
clockwise on the map for yaw 90/180/270. See
``docs/adr/0003-landmark-localization-task1.md`` and
``src/carbot/landmarks.py`` for the conventions.

Run (reportlab is not a project dependency):

    uv run --with reportlab python3 scripts/generate_apriltag_sheet.py \
        --output scratch/landmarks/task1-tags-50mm.pdf \
        --tag-map-out scratch/landmarks/task1-tag-map-template.json

Then print at 100% / Actual Size, verify the 100 mm bar with a ruler, cut
the tags, and place them on the map at the positions recorded in the tag
map JSON.
"""

from __future__ import annotations

import argparse
import io
import json
from pathlib import Path

MM_PER_PT = 25.4 / 72.0


def _make_tag_image(tag_id: int, size_px: int) -> bytes:
    """Render one 36h11 tag at ``size_px`` and return PNG bytes."""
    import cv2

    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    marker = cv2.aruco.generateImageMarker(dictionary, tag_id, size_px)
    ok, encoded = cv2.imencode(".png", marker)
    if not ok:
        raise RuntimeError(f"could not encode tag {tag_id}")
    return encoded.tobytes()


def _draw_tag_cell(
    canvas,
    tag_id: int,
    size_mm: float,
    quiet_mm: float,
    x_mm: float,
    y_mm: float,
    tag_png: bytes,
) -> None:
    """Draw one tag + quiet zone + N arrow + id label, ``x_mm``/``y_mm`` from
    the bottom-left corner of the cell (in mm, converted to points)."""
    from reportlab.lib.utils import ImageReader

    def pt(value_mm: float) -> float:
        return value_mm / MM_PER_PT

    quiet_side = size_mm + 2 * quiet_mm
    cx = x_mm + quiet_side / 2.0
    # Orientation + ID label, centred above the footprint (one uncluttered line).
    canvas.setFont("Helvetica-Bold", 8)
    canvas.drawCentredString(pt(cx), pt(y_mm + quiet_side + 3.0), f"N \u2191   ID {tag_id}")
    # Dashed cut line around the quiet-zone footprint: cut along this border.
    canvas.setDash(2.0, 2.0)
    canvas.setLineWidth(pt(0.3))
    canvas.rect(pt(x_mm), pt(y_mm), pt(quiet_side), pt(quiet_side))
    canvas.setDash()
    # Corner crosses for alignment.
    canvas.setLineWidth(pt(0.3))
    for corner_x, corner_y in (
        (x_mm, y_mm),
        (x_mm + quiet_side, y_mm),
        (x_mm, y_mm + quiet_side),
        (x_mm + quiet_side, y_mm + quiet_side),
    ):
        canvas.line(pt(corner_x - 1.0), pt(corner_y), pt(corner_x + 1.0), pt(corner_y))
        canvas.line(pt(corner_x), pt(corner_y - 1.0), pt(corner_x), pt(corner_y + 1.0))
    # The tag itself inside the quiet zone.
    canvas.drawImage(
        ImageReader(io.BytesIO(tag_png)),
        pt(x_mm + quiet_mm),
        pt(y_mm + quiet_mm),
        width=pt(size_mm),
        height=pt(size_mm),
    )
    # Size label centred below the footprint.
    canvas.setFont("Helvetica", 6)
    canvas.drawCentredString(pt(cx), pt(y_mm - 3.5), f"{size_mm:.0f} mm")


def _draw_scale_bar(canvas, page_width_mm: float, y_mm: float, length_mm: float = 100.0) -> None:
    from reportlab.pdfgen import canvas as canvas_mod  # noqa: F401

    def pt(value_mm: float) -> float:
        return value_mm / MM_PER_PT

    left = (page_width_mm - length_mm) / 2.0
    canvas.setLineWidth(pt(0.5))
    canvas.line(pt(left), pt(y_mm), pt(left + length_mm), pt(y_mm))
    canvas.setLineWidth(pt(0.3))
    for cm in range(11):
        x = left + cm * 10.0
        tick = pt(2.0) if cm % 5 == 0 else pt(1.0)
        canvas.line(pt(x), pt(y_mm), pt(x), pt(y_mm - tick))
        if cm % 5 == 0:
            canvas.setFont("Helvetica", 6)
            canvas.drawString(pt(x - 1.0), pt(y_mm + 1.5), str(cm))
    canvas.setFont("Helvetica-Bold", 7)
    canvas.drawCentredString(
        pt(page_width_mm / 2.0),
        pt(y_mm + 5.5),
        "100 mm scale bar \u2014 print at 100% (Actual Size) and verify with a ruler",
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("scratch/landmarks/apriltag-sheet.pdf"))
    parser.add_argument(
        "--tag-map-out",
        type=Path,
        default=None,
        help="also write a tag-map JSON template (yaw 0, placeholder positions)",
    )
    parser.add_argument(
        "--ids",
        default="0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15",
        help="space-separated tag IDs (default: the Task-1 draft set 0..15)",
    )
    parser.add_argument(
        "--tag-size-mm",
        type=float,
        default=20.0,
        help="printed black-square side length (operator limit: 20 mm on the 100x70 map)",
    )
    parser.add_argument(
        "--quiet-zone-mm",
        type=float,
        default=5.0,
        help="white margin around each tag (default 5 mm; 20 mm tag + 5 mm = 30 mm footprint)",
    )
    parser.add_argument(
        "--columns", type=int, default=0, help="tags per row (default: fit the page width)"
    )
    args = parser.parse_args()

    if args.tag_size_mm <= 0 or args.quiet_zone_mm <= 0:
        parser.error("--tag-size-mm and --quiet-zone-mm must be positive")
    try:
        ids = [int(value) for value in args.ids.split()]
    except ValueError as exc:
        parser.error(f"invalid --ids: {exc}")
    if not ids:
        parser.error("--ids must contain at least one id")
    if len(set(ids)) != len(ids):
        parser.error("--ids must not repeat")

    from reportlab.lib.pagesizes import A4
    from reportlab.pdfgen import canvas

    args.output.parent.mkdir(parents=True, exist_ok=True)
    page_w_mm, page_h_mm = A4[0] * MM_PER_PT, A4[1] * MM_PER_PT
    pdf = canvas.Canvas(str(args.output), pagesize=A4)

    quiet_side = args.tag_size_mm + 2 * args.quiet_zone_mm
    cell_w = quiet_side + 10.0
    cell_h = quiet_side + 14.0
    margin = 15.0
    if args.columns <= 0:
        args.columns = max(1, int((page_w_mm - 2 * margin - 5.0) / cell_w))
    rows = (len(ids) + args.columns - 1) // args.columns
    x_origin = (page_w_mm - args.columns * cell_w) / 2.0  # centre the grid

    def pt(value_mm: float) -> float:
        return value_mm / MM_PER_PT

    size_px = int(args.tag_size_mm * 20.0)  # 20 px/mm render resolution
    for index, tag_id in enumerate(ids):
        column = index % args.columns
        row = rows - 1 - index // args.columns  # top row first
        x = x_origin + column * cell_w
        y = margin + 18.0 + row * cell_h
        if y + quiet_side + 6.0 > page_h_mm:
            raise RuntimeError("tags do not fit on one A4 page; reduce size or raise --columns")
        _draw_tag_cell(
            pdf,
            tag_id,
            args.tag_size_mm,
            args.quiet_zone_mm,
            x,
            y,
            _make_tag_image(tag_id, size_px),
        )
    _draw_scale_bar(pdf, page_w_mm, 8.0)
    pdf.showPage()
    pdf.save()
    print(
        f"wrote {args.output} ({len(ids)} tags, {args.tag_size_mm:.0f} mm + "
        f"{args.quiet_zone_mm:.0f} mm quiet zone)"
    )

    if args.tag_map_out is not None:
        template = {
            "name": "task1-map",
            "description": "Fill x_m/y_m with the measured/designed position of each tag "
            "after placement; yaw 0 = N arrow pointing map-north. "
            "Map frame: origin at the map's SOUTH-WEST corner, x east, "
            "y north, metres; NE corner = (1.00, 0.70). Measure x from "
            "the west edge and y from the south edge to the tag center.",
            "tags": [
                {
                    "id": tag_id,
                    "x_m": 0.0,
                    "y_m": 0.0,
                    "z_m": 0.0,
                    "yaw_deg": 0,
                    "size_m": args.tag_size_mm / 1000.0,
                }
                for tag_id in ids
            ],
        }
        args.tag_map_out.parent.mkdir(parents=True, exist_ok=True)
        args.tag_map_out.write_text(json.dumps(template, indent=2) + "\n", encoding="utf-8")
        print(f"wrote tag-map template {args.tag_map_out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

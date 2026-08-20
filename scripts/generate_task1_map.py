#!/usr/bin/env python3
"""Generate the printable Task-1 map: 8 A4 map pages + 3 front pages.

The map is drawn vector-style at exact millimetre scale and split into a
4x2 grid of A4 pages (each 210x297 mm) that tile into a 840x594 mm sheet
carrying a 840x588 mm map (the original 1000x700 mm Task-1 map scaled by
0.84). The 15 mm black route line and the 20 mm AprilTags are NOT scaled —
they keep their physical size so line-follow and tag pose estimation keep
working; only the route geometry and tag positions are scaled.

Units are millimetres throughout. Coordinate frame: map frame, SW origin,
x east, y north (matches src/carbot/landmarks.py conventions).

Page 1 = overview; page 2 = print/assemble guide; page 3 = AprilTag
coordinate table; pages 4..11 = the 8 map tiles.

Run:
    uv run --with reportlab python3 scripts/generate_task1_map.py \
        --output scratch/landmarks/task1-map.pdf \
        --tag-map-out scratch/landmarks/task1-tag-map.json
"""

from __future__ import annotations

import argparse
import io
import json
import math
from itertools import pairwise
from pathlib import Path

import cv2
from reportlab.lib.colors import black, white
from reportlab.lib.pagesizes import A4
from reportlab.lib.utils import ImageReader
from reportlab.pdfgen import canvas

MM_PER_PT = 25.4 / 72.0
A4_W_MM, A4_H_MM = 210.0, 297.0

# ---------------------------------------------------------------------------
# Map geometry (mm, SW origin: x east, y north). Original 1000x700 mm * 0.84.
# ---------------------------------------------------------------------------
SCALE = 0.84
MAP_W_MM = 840.0
MAP_H_MM = 588.0
LINE_W_MM = 20.0  # 2.0 cm black line, kept at physical size
TAG_SIZE_MM = 40.0  # AprilTag side, kept at physical size
MIN_TAG_CLEAR_MM = 5.0  # extra clearance beyond the tag's own half-width

ROUNDABOUT = (245.3, 241.9, 151.2)  # cx, cy, R (mm)

# Route centerlines as polylines (mm). Roundabout is drawn separately.
ROUTE = [
    [(589.7, 163.8), (589.7, 247.8)],  # stem up
    [(589.7, 247.8), (724.1, 247.8)],  # Phase 2 east
    [(724.1, 247.8), (737.5, 253.7), (747.6, 257.9), (751.8, 260.4)],  # ARC 1 SE
    [(751.8, 260.4), (772.8, 283.9), (783.7, 310.8)],  # east slant
    [(783.7, 310.8), (783.7, 411.6)],  # Line A up
    [(783.7, 411.6), (783.7, 446.9), (783.7, 478.8), (774.5, 502.3), (760.2, 525)],  # ARC 2 NE
    [(760.2, 525), (268.8, 525)],  # Phase 6 top
    [(268.8, 525), (261.2, 510.7), (251.2, 493.9), (243.6, 470.4)],  # ARC 3 NW
    [(243.6, 470.4), (243.6, 407.4), (245.3, 393.1)],  # entry down
    [(396.5, 241.9), (396.5, 247.8)],  # exit -> return
    [(396.5, 247.8), (589.7, 247.8)],  # Phase 10 return
]

# Designed tag positions (id, x_mm, y_mm). Originally one AprilTag at each
# corner of every tile (8 tiles x 4 corners = 32); at 40 mm the corner slots
# near the top-right route/edge pinch (5, 8, 9, 12) and the Line A / ARC2
# route column (15) no longer had room, so those plus 25 were moved into the
# open roundabout interior / lower map instead of their original tile corner.
# All positions are clear of the route, page seams, and start/scale boxes
# (see validate_tags — clearances scale with TAG_SIZE_MM).
TAGS = [
    # tile 1 (NW) — TL, TR, BL, BR
    (0, 42, 546), (1, 180, 546), (2, 42, 324), (3, 180, 324),
    # tile 2 (N) — TL, BL; TR/BR relocated below (crowded by Phase-6 top line)
    (4, 240, 546), (6, 240, 324), (7, 394, 354.8),
    # tile 3 (NE-left) — BL, BR; TL/TR relocated below
    (10, 450, 324), (11, 500, 320),
    # tile 4 (NE) — TR, BL; TL/BR relocated below
    (13, 798, 546), (14, 695, 320),
    # tile 5 (SW) — TL, TR, BL, BR
    (16, 42, 264), (17, 180, 264), (18, 42, 42), (19, 180, 42),
    # tile 6 (S) — TL, TR (clear of roundabout), BL, BR
    (20, 240, 264), (21, 357.9, 263.6), (22, 240, 42), (23, 390, 42),
    # tile 7 (SE-left) — TL below return line, BL, BR; TR relocated below
    (24, 445, 200), (26, 450, 42), (27, 528, 200),
    # tile 8 (SE) — TL right of start box, TR, BL, BR
    (28, 695, 85), (29, 798, 258.5), (30, 700, 42), (31, 798, 42),
    # relocated into the open roundabout interior / lower map — see note above
    (5, 390, 480), (8, 450, 485), (9, 600, 480), (12, 660, 480),
    (15, 161, 211), (25, 261, 153),
]

START_MM = (589.7, 163.8)  # stem bottom = start zone
# Departure box 150x150: top edge exactly touches the bottom edge of the
# start stem (line y 153.8-173.8 -> box top = 153.8), centred on the stem
# axis so its centre stays collinear with the stem and scale-box centres.
START_BOX = (514.7, 3.8, 150.0, 150.0)
# Scale-bar/origin box: centre aligned with the stem axis (x = 589.7) so the
# start-box centre, the stem centre and the scale-box centre are collinear
# (the car can zero/align before departure). Bottom edge 30 mm above the
# departure road (y = 247.8 + 30 = 277.8), near the 3/4/7/8 tile crossing.
SCALE_BOX = (549.7, 277.8, 80.0, 80.0)
NORTH_MM = (150.0, 555.0)  # north arrow position

SEAM_X = [210.0, 420.0, 630.0]  # vertical page seams
SEAM_Y = 294.0  # horizontal page seam


def pt(mm: float) -> float:
    return mm / MM_PER_PT


def make_tag_png(tag_id: int, size_px: int = 400) -> bytes:
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    marker = cv2.aruco.generateImageMarker(dictionary, tag_id, size_px)
    ok, encoded = cv2.imencode(".png", marker)
    if not ok:
        raise RuntimeError(f"could not encode tag {tag_id}")
    return encoded.tobytes()


# ---------------------------------------------------------------------------
# Geometry helpers (mm)
# ---------------------------------------------------------------------------
def dist_point_seg(px: float, py: float, a, b) -> float:
    ax, ay = a
    bx, by = b
    dx, dy = bx - ax, by - ay
    if dx == 0 and dy == 0:
        return math.hypot(px - ax, py - ay)
    t = ((px - ax) * dx + (py - ay) * dy) / (dx * dx + dy * dy)
    t = max(0.0, min(1.0, t))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def min_route_dist(px: float, py: float) -> float:
    d = min(dist_point_seg(px, py, a, b) for seg in ROUTE for a, b in pairwise(seg))
    cx, cy, r = ROUNDABOUT
    d = min(d, abs(math.hypot(px - cx, py - cy) - r))
    return d


def roundabout_points(n: int = 360):
    """Full 360-degree circle (matches the physical printed ring)."""
    cx, cy, r = ROUNDABOUT
    pts = []
    for i in range(n + 1):
        theta = math.radians(360.0 * i / n)  # 0 -> 360 deg
        pts.append((cx + r * math.cos(theta), cy + r * math.sin(theta)))
    return pts


def dist_to_box(px: float, py: float, box) -> float:
    x, y, w, h = box
    if x <= px <= x + w and y <= py <= y + h:
        return 0.0
    dx = max(x - px, 0.0, px - (x + w))
    dy = max(y - py, 0.0, py - (y + h))
    return math.hypot(dx, dy)


def validate_tags(tags):
    problems = []
    tag_half = TAG_SIZE_MM / 2.0
    route_clear_mm = LINE_W_MM / 2.0 + tag_half + MIN_TAG_CLEAR_MM
    seam_clear_mm = tag_half + 5.0
    box_clear_mm = tag_half + 10.0
    for tid, x, y in tags:
        rd = min_route_dist(x, y)
        if rd < route_clear_mm:
            problems.append(f"tag {tid}: route clearance {rd:.1f} mm < {route_clear_mm:.1f}")
        if min(abs(x - s) for s in SEAM_X) < seam_clear_mm:
            problems.append(f"tag {tid}: too close to vertical seam")
        if abs(y - SEAM_Y) < seam_clear_mm:
            problems.append(f"tag {tid}: too close to horizontal seam")
        if (
            x < TAG_SIZE_MM
            or x > MAP_W_MM - TAG_SIZE_MM
            or y < TAG_SIZE_MM
            or y > MAP_H_MM - TAG_SIZE_MM
        ):
            problems.append(f"tag {tid}: outside map ({x:.0f},{y:.0f})")
        db = min(dist_to_box(x, y, START_BOX), dist_to_box(x, y, SCALE_BOX))
        if db < box_clear_mm:
            problems.append(f"tag {tid}: box clearance {db:.1f} mm < {box_clear_mm:.1f}")
    return problems


# ---------------------------------------------------------------------------
# Drawing (all in mm, SW origin)
# ---------------------------------------------------------------------------
def draw_map(c: canvas.Canvas, tag_pngs: dict[int, bytes], detail: bool = True) -> None:
    # Start (departure) box and scale-bar/origin box: white fill + black
    # border, drawn first so the 15 mm route line stays visible on top.
    c.setStrokeColor(black)
    c.setLineWidth(pt(1.2))
    c.setFillColor(white)
    sx, sy, sw, sh = START_BOX
    c.rect(pt(sx), pt(sy), pt(sw), pt(sh), stroke=1, fill=1)
    bx, by, bw, bh = SCALE_BOX
    c.rect(pt(bx), pt(by), pt(bw), pt(bh), stroke=1, fill=1)

    # Route black line (15 mm wide, round joins) — drawn over the boxes.
    c.setStrokeColor(black)
    c.setLineWidth(pt(LINE_W_MM))
    c.setLineCap(1)
    c.setLineJoin(1)
    for seg in ROUTE:
        p = c.beginPath()
        p.moveTo(pt(seg[0][0]), pt(seg[0][1]))
        for x, y in seg[1:]:
            p.lineTo(pt(x), pt(y))
        c.drawPath(p, stroke=1, fill=0)
    # Roundabout: full 360-degree circle.
    ring = roundabout_points()
    p = c.beginPath()
    p.moveTo(pt(ring[0][0]), pt(ring[0][1]))
    for x, y in ring[1:]:
        p.lineTo(pt(x), pt(y))
    c.drawPath(p, stroke=1, fill=0)

    # AprilTags + labels.
    for tag_id, tx, ty in TAGS:
        png = tag_pngs[tag_id]
        c.drawImage(
            ImageReader(io.BytesIO(png)),
            pt(tx - TAG_SIZE_MM / 2),
            pt(ty - TAG_SIZE_MM / 2),
            width=pt(TAG_SIZE_MM),
            height=pt(TAG_SIZE_MM),
        )
        # thin border so the white quiet zone is visible against the paper
        c.setStrokeColor(black)
        c.setLineWidth(pt(0.3))
        c.rect(
            pt(tx - TAG_SIZE_MM / 2),
            pt(ty - TAG_SIZE_MM / 2),
            pt(TAG_SIZE_MM),
            pt(TAG_SIZE_MM),
            stroke=1,
            fill=0,
        )
        # One-line label "ID n (N ↑)" pointing toward the map centre so it
        # never crowds the corner cross or the page number; flipped below on
        # bottom-row tiles when the upper label would hit the page-number
        # band or the paper edge (tags with y > 262.7, e.g. y=264 and 278.5).
        if detail:
            label_off = TAG_SIZE_MM / 2.0 + 6.0
            label_x = tx
            # Special adjustments to avoid overlapping labels.
            if tag_id == 15:
                label_dy = TAG_SIZE_MM / 2.0 + 14.0
            elif tag_id == 28:
                label_x = tx - 28.0
                label_dy = label_off
            elif tag_id == 31:
                label_dy = -label_off
            else:
                if ty <= 294.0:
                    label_dy = -label_off if ty > 262.7 else label_off
                else:
                    label_dy = label_off if 319.5 < ty < 325.5 else -label_off
            c.setFont("Helvetica-Bold", 14)
            c.setFillColor(black)
            c.drawCentredString(
                pt(label_x), pt(ty + label_dy), f"ID {tag_id} (N \u2191)"
            )

    # North arrow.
    nx, ny = NORTH_MM
    c.setFillColor(black)
    if detail:
        c.setFont("Helvetica-Bold", 10)
        c.drawCentredString(pt(nx), pt(ny + 16), "N")
    p = c.beginPath()
    p.moveTo(pt(nx), pt(ny + 12))
    p.lineTo(pt(nx - 6), pt(ny))
    p.lineTo(pt(nx + 6), pt(ny))
    p.close()
    c.drawPath(p, stroke=0, fill=1)

    if detail:
        # Departure-area labels inside the start box (the 150x150 box holds
        # no route line, so the labels sit centred). 18 pt keeps the text
        # clear of the tile 7 paper edge (the 22 pt label was clipped).
        c.setFillColor(black)
        c.setFont("Helvetica-Bold", 16)
        c.drawCentredString(pt(sx + sw / 2), pt(sy + sh / 2 + 6), "DEPARTURE AREA")
        c.setFont("Helvetica-Bold", 11)
        c.drawCentredString(pt(sx + sw / 2), pt(sy + sh / 2 - 26), "START")

        # Scale cross inside the box: X axis 80 mm (full box width), Y axis
        # 40 mm vertical, both ticked every 10 mm.
        cx_ = bx + bw / 2.0
        cy_ = by + bh / 2.0
        c.setStrokeColor(black)
        c.setLineWidth(pt(1.2))
        # X axis (horizontal, 80 mm).
        c.line(pt(bx), pt(cy_), pt(bx + bw), pt(cy_))
        for i in range(9):  # 0..80 mm every 10
            x = bx + i * bw / 8.0
            h = 4.0 if i % 2 == 0 else 2.0
            c.line(pt(x), pt(cy_), pt(x), pt(cy_ - h))
        # Y axis (vertical, 40 mm).
        c.line(pt(cx_), pt(cy_ - 20.0), pt(cx_), pt(cy_ + 20.0))
        for i in range(5):  # 0..40 mm every 10
            y = cy_ - 20.0 + i * 40.0 / 4.0
            h = 4.0 if i % 2 == 0 else 2.0
            c.line(pt(cx_), pt(y), pt(cx_ + h), pt(y))
        c.setFont("Helvetica", 7)
        c.drawString(pt(bx - 4), pt(cy_ - 8), "0")
        c.drawRightString(pt(bx + bw - 14), pt(cy_ - 8), "80 mm")
        c.drawRightString(pt(cx_ - 18), pt(cy_ + 21), "40 mm")
        c.setFont("Helvetica-Bold", 7)
        c.drawCentredString(pt(cx_), pt(cy_ + 26), "scale bar (X 80 / Y 40 mm)")

    # Map border.
    c.setStrokeColor(black)
    c.setLineWidth(pt(0.5))
    c.rect(pt(0), pt(0), pt(MAP_W_MM), pt(MAP_H_MM), stroke=1, fill=0)


def draw_page_marks(c: canvas.Canvas, tile_no: int, total: int = 8) -> None:
    """Corner crosses + tile number at all four corners.

    Page numbers sit close to the very corner (5 mm in) so they never
    overlap the "ID n" / "N ↑" labels of the corner AprilTags.
    """
    c.setStrokeColor(black)
    c.setLineWidth(pt(0.4))
    # Crosses sit 17 mm in from the paper edge (lines 14-20 mm) so they
    # survive typical printer margins; the old 12 mm corners got clipped.
    corners = [
        (17, 17),
        (A4_W_MM - 17, 17),
        (17, A4_H_MM - 17),
        (A4_W_MM - 17, A4_H_MM - 17),
    ]
    for cx, cy in corners:
        c.line(pt(cx - 3), pt(cy), pt(cx + 3), pt(cy))
        c.line(pt(cx), pt(cy - 3), pt(cx), pt(cy + 3))
    c.setFont("Helvetica-Bold", 8)
    label = f"tile {tile_no}/{total}"
    c.drawString(pt(24), pt(17), label)  # bottom-left
    c.drawRightString(pt(A4_W_MM - 24), pt(17), label)  # bottom-right
    c.drawString(pt(24), pt(A4_H_MM - 17), label)  # top-left
    c.drawRightString(pt(A4_W_MM - 24), pt(A4_H_MM - 17), label)  # top-right


def draw_overview(c: canvas.Canvas, tag_pngs: dict[int, bytes]) -> None:
    """Page 1: whole-map thumbnail with the 8-page grid and tile numbers."""
    s = 0.25
    map_w_mm = MAP_W_MM * s  # 210
    map_h_mm = MAP_H_MM * s  # 147
    ox = (A4_W_MM - map_w_mm) / 2.0
    oy = A4_H_MM - map_h_mm - 26.0

    # Title.
    c.setFillColor(black)
    c.setFont("Helvetica-Bold", 15)
    c.drawCentredString(pt(A4_W_MM / 2), pt(A4_H_MM - 14), "Task-1 Map \u2014 Overview")
    c.setFont("Helvetica", 10)
    c.drawCentredString(
        pt(A4_W_MM / 2),
        pt(A4_H_MM - 23),
        "Full-map overview \u2014 tile layout reference",
    )

    # Thumbnail (route + tags + border, no text labels).
    c.saveState()
    c.translate(pt(ox), pt(oy))
    c.scale(s, s)
    draw_map(c, tag_pngs, detail=False)
    c.restoreState()

    # 8-page grid (dashed) + tile numbers.
    c.setDash(2, 2)
    c.setLineWidth(pt(0.4))
    c.setStrokeColor(black)
    for i in range(1, 4):
        x = ox + i * 210.0 * s
        c.line(pt(x), pt(oy), pt(x), pt(oy + map_h_mm))
    c.line(pt(ox), pt(oy + map_h_mm / 2), pt(ox + map_w_mm), pt(oy + map_h_mm / 2))
    c.setDash()

    c.setFont("Helvetica-Bold", 9)
    for col in range(4):
        for row in range(2):
            tile = row * 4 + col + 1  # row 0 = top (tiles 1-4), row 1 = bottom (5-8)
            cx = ox + (col + 0.5) * 210.0 * s
            cy = oy + ((2 - row) * 294.0 - 147.0) * s
            c.drawCentredString(pt(cx), pt(cy), f"{tile}")

    # Bottom notes.
    y = oy - 10.0
    c.setFont("Helvetica-Bold", 10)
    c.drawString(pt(20), pt(y), "Assemble order")
    y -= 13
    c.setFont("Helvetica", 8.5)
    for ln in (
        "Top row (north): tiles 1 \u2013 4, left to right.",
        "Bottom row (south): tiles 5 \u2013 8, left to right.",
        "Align the corner crosses on adjacent tiles and tape on the back.",
    ):
        c.drawString(pt(22), pt(y), ln)
        y -= 12


# ---------------------------------------------------------------------------
# Instruction page (page 9)
# ---------------------------------------------------------------------------
def draw_guide(c: canvas.Canvas) -> None:
    c.setFillColor(black)
    y = A4_H_MM - 20
    c.setFont("Helvetica-Bold", 16)
    c.drawString(pt(20), pt(y), "Task-1 Map \u2014 Print & Assemble Guide")
    y -= 22

    def para(title, lines):
        nonlocal y
        c.setFont("Helvetica-Bold", 11)
        c.drawString(pt(20), pt(y), title)
        y -= 15
        c.setFont("Helvetica", 9.5)
        for ln in lines:
            c.drawString(pt(24), pt(y), ln)
            y -= 12.5
        y -= 6

    para(
        "1. Print the 8 map tiles (PDF pages 4\u201311)",
        [
            "Print PDF pages 4\u201311 on A4 at 100% / Actual Size. Do NOT use \u201cfit to page\u201d.",
            "Verify the scale bar (X 80 mm / Y 40 mm) with a ruler before assembling.",
            "If the corner crosses are clipped, enable borderless printing or reduce printer margins.",
        ],
    )
    para(
        "2. Assemble (4 columns x 2 rows) \u2014 see the overview on PDF page 1",
        [
            "Top row (north) = tiles 1\u20134 left to right; bottom row (south) = tiles 5\u20138 left to right.",
            "Align the corner crosses on adjacent tiles; tape on the back.",
        ],
    )
    para(
        "3. Orientation",
        [
            "The N arrow points map-north (up). Tags are printed with ID upright facing north.",
        ],
    )
    para(
        "4. AprilTag landmarks (the printed squares)",
        [
            "32 AprilTags (family 36h11, 40 mm) are printed onto the map as absolute-position landmarks.",
            "Each is labelled \u201cID n (N \u2191)\u201d (yaw = 0 = facing map-north).",
            "Source: AprilTag is an open-source project by the University of Michigan APRIL lab.",
            "Official repository: https://github.com/AprilRobotics/apriltag",
            "Web search: \u201cAprilTag\u201d, \u201cAprilTag 36h11\u201d, \u201cAprilTag pose estimation\u201d, \u201cOpenCV aruco AprilTag\u201d.",
        ],
    )
    para(
        "5. Scale",
        [
            "Map 840 x 588 mm = original 1000 x 700 mm Task-1 map scaled 0.84.",
            "Route black line 15 mm; AprilTags 40 mm (physical sizes unchanged).",
        ],
    )


# ---------------------------------------------------------------------------
# AprilTag coordinate table (page 3)
# ---------------------------------------------------------------------------
def draw_tag_table(c: canvas.Canvas) -> None:
    """Page 3: table of every AprilTag's ID and X/Y position in mm."""
    c.setFillColor(black)
    y = A4_H_MM - 20
    c.setFont("Helvetica-Bold", 16)
    c.drawString(pt(20), pt(y), "AprilTag Coordinates")
    y -= 12
    c.setFont("Helvetica", 9.5)
    c.drawString(
        pt(20),
        pt(y),
        "X and Y are in millimetres from the map's SOUTH-WEST corner "
        "(X east, Y north); NE corner = (840, 588). cm = mm \u00f7 10.",
    )
    y -= 16

    col_w = 78.0
    x0, x1 = 20.0, 20.0 + col_w + 26.0
    row_h = 4.6
    rows_per_col = 16

    def header(x):
        c.setFont("Helvetica-Bold", 9)
        c.drawString(pt(x), pt(y), "ID")
        c.drawString(pt(x + 20), pt(y), "X (mm)")
        c.drawString(pt(x + 48), pt(y), "Y (mm)")

    header(x0)
    header(x1)
    y -= 8

    c.setFont("Helvetica", 8.5)
    for i, (tid, tx, ty) in enumerate(TAGS):
        col = i // rows_per_col
        row = i % rows_per_col
        x = x0 if col == 0 else x1
        yy = y - row * row_h
        c.drawString(pt(x), pt(yy), f"{tid}")
        c.drawString(pt(x + 20), pt(yy), f"{tx:.1f}")
        c.drawString(pt(x + 48), pt(yy), f"{ty:.1f}")

    c.setFont("Helvetica", 8.5)
    c.drawString(
        pt(20),
        pt(y - rows_per_col * row_h - 8),
        "Same values as scratch/landmarks/task1-tag-map.json "
        "(metres = mm \u00f7 1000).",
    )


# ---------------------------------------------------------------------------
def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("scratch/landmarks/task1-map.pdf"))
    parser.add_argument(
        "--tag-map-out", type=Path, default=Path("scratch/landmarks/task1-tag-map.json")
    )
    args = parser.parse_args()

    problems = validate_tags(TAGS)
    if problems:
        for p in problems:
            print("ERROR:", p)
        return 1

    tag_pngs = {tid: make_tag_png(tid) for tid, _, _ in TAGS}

    args.output.parent.mkdir(parents=True, exist_ok=True)
    pdf = canvas.Canvas(str(args.output), pagesize=A4)
    pdf.setTitle("Task-1 map (overview + guide + tag table + 8 tiles)")

    # Page 1: overview thumbnail; page 2: guide; page 3: AprilTag coordinates.
    draw_overview(pdf, tag_pngs)
    pdf.showPage()
    draw_guide(pdf)
    pdf.showPage()
    draw_tag_table(pdf)
    pdf.showPage()

    # Pages 4..11: map tiles, 4 columns x 2 rows. Tiles 1-4 = top row
    # (map north, y 294..588), tiles 5-8 = bottom row (y 0..294).
    for page in range(8):
        col = page % 4
        row = 1 - page // 4  # 1 = top row (map top), 0 = bottom row
        # clip to the page, then translate so big-map (col*210, row*294)
        # lands at the page origin.
        pdf.saveState()
        clip = pdf.beginPath()
        clip.rect(0, 0, pt(A4_W_MM), pt(A4_H_MM))
        pdf.clipPath(clip, stroke=0, fill=0)
        pdf.translate(-pt(col * 210.0), -pt(row * 294.0))
        draw_map(pdf, tag_pngs)
        pdf.restoreState()
        draw_page_marks(pdf, page + 1, 8)
        pdf.showPage()

    pdf.save()
    print(f"wrote {args.output} (overview + guide + 8 tiles, {len(TAGS)} tags)")

    # Tag map JSON (metres, SW origin, x east / y north).
    tag_map = {
        "name": "task1-map-scaled-0.84",
        "description": (
            "AprilTags printed directly on the 840x588 mm map (original "
            "1000x700 mm scaled 0.84). Map frame: SW origin, x east, y north, "
            "metres; NE corner = (0.84, 0.588). Tag size 40 mm. yaw_deg 0 = "
            "ID upright facing map-north (N arrow up)."
        ),
        "tags": [
            {
                "id": tid,
                "x_m": round(x / 1000.0, 4),
                "y_m": round(y / 1000.0, 4),
                "z_m": 0.0,
                "yaw_deg": 0,
                "size_m": 0.04,
            }
            for tid, x, y in TAGS
        ],
    }
    args.tag_map_out.parent.mkdir(parents=True, exist_ok=True)
    args.tag_map_out.write_text(json.dumps(tag_map, indent=2) + "\n", encoding="utf-8")
    print(f"wrote tag-map {args.tag_map_out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

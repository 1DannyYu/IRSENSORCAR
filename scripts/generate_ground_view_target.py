#!/usr/bin/env python3
"""Generate a printable floor rectangle for ground-view calibration.

``examples/27_cam_ground_view_calibrate.py --corners`` needs four floor points of
known real-world size. Taping a rectangle by hand and measuring it is fiddly;
this prints a rectangle with axis tick marks and labelled corners (TL, TR,
BR, BL — the same order ``scripts/pick_ground_view_corners.py`` expects) so
the physical size is unambiguous and print-scale errors are easy to catch
with a ruler.

Not a robot script: runs on the workstation, produces a PDF, no camera, no
motors.

Usage (reportlab is not a project dependency, so run it ephemeral with uv):

    uv run --with reportlab python3 scripts/generate_ground_view_target.py

Then print at *actual size / 100%* — not "fit to page" — and measure the
printed rectangle with a ruler before taping it down.
"""

from __future__ import annotations

import argparse
from pathlib import Path

from reportlab.lib.pagesizes import A4
from reportlab.lib.units import mm
from reportlab.pdfgen import canvas

DEFAULT_OUTPUT = Path("scratch/ground-view-calibration/ground-view-calibration-target.pdf")
CROSSHAIR_ARM_MM = 9.0
BORDER_WIDTH_MM = 5.0  # thick, marker-like — a hairline is the first thing a
# manual-focus close-up camera loses to blur; a bold line survives it
MINOR_TICK_MM = 10.0
MAJOR_TICK_X_MM = 10.0  # X labelled every 1cm
MAJOR_TICK_Y_MM = 10.0  # Y labelled every 1cm


def draw_target(c: canvas.Canvas, width_mm: float, height_mm: float) -> None:
    page_w, page_h = A4
    rect_w, rect_h = width_mm * mm, height_mm * mm
    x0 = (page_w - rect_w) / 2
    y1 = page_h - 75 * mm  # top edge of the rectangle, leaves room for the title
    y0 = y1 - rect_h
    x1 = x0 + rect_w
    border_w = BORDER_WIDTH_MM * mm

    # Title / print-scale warning.
    c.setFont("Helvetica-Bold", 13)
    c.drawCentredString(page_w / 2, page_h - 25 * mm, "Ground-view calibration target")
    c.setFont("Helvetica-Bold", 11)
    c.drawCentredString(
        page_w / 2, page_h - 33 * mm, "Print at 100% / actual size — NOT “fit to page”"
    )
    c.setFont("Helvetica", 9)
    c.drawCentredString(
        page_w / 2,
        page_h - 41 * mm,
        f"Rectangle is {width_mm:.0f} x {height_mm:.0f} mm "
        f"({width_mm / 10:.1f} x {height_mm / 10:.1f} cm) if printed correctly.",
    )

    # Rectangle border: thick and solid black, same visual weight as the
    # black track line itself, so it survives camera blur that wipes out
    # thin lines and small text.
    c.setLineWidth(border_w)
    c.setStrokeColorRGB(0, 0, 0)
    c.rect(x0, y0, rect_w, rect_h, stroke=1, fill=0)

    # Corner crosshairs + labels (TL, TR, BR, BL — matches
    # scripts/pick_ground_view_corners.py click order).
    corners = {
        "TL": (x0, y1),
        "TR": (x1, y1),
        "BR": (x1, y0),
        "BL": (x0, y0),
    }
    arm = CROSSHAIR_ARM_MM * mm
    c.setLineWidth(border_w)
    c.setFont("Helvetica-Bold", 22)
    for label, (x, y) in corners.items():
        c.line(x - arm, y, x + arm, y)
        c.line(x, y - arm, x, y + arm)
        label_dx = arm + 2 * mm if x == x0 else -(arm + 2 * mm) - 26
        label_dy = arm + 2 * mm if y == y0 else -(arm + 2 * mm) - 16
        c.drawString(x + label_dx, y + label_dy, label)

    # X-axis ticks: minor every 1cm (thin, unlabelled — ruler verification),
    # major every 2cm (thick, big bold number) below the bottom edge. The
    # two ends coincide with BL/BR, which already carry a label, so skip
    # the redundant (and border-overlapping) 0/max number there.
    steps_x = round(width_mm / MINOR_TICK_MM)
    for i in range(steps_x + 1):
        x = x0 + i * MINOR_TICK_MM * mm
        pos_mm = i * MINOR_TICK_MM
        major = pos_mm % MAJOR_TICK_X_MM == 0
        c.setLineWidth(1.2 * mm if major else 0.4 * mm)
        c.line(x, y0, x, y0 - (5 * mm if major else 2.5 * mm))
        if major and 0 < i < steps_x:
            c.setFont("Helvetica-Bold", 14)
            c.drawCentredString(x, y0 - 12 * mm, str(round(pos_mm / 10)))
    c.setFont("Helvetica-Bold", 11)
    c.drawCentredString((x0 + x1) / 2, y0 - 20 * mm, "X (cm)")

    # Y-axis ticks, same minor/major scheme, drawn just inside the left edge
    # (page margin is too tight at the left to put labels outside).
    steps_y = round(height_mm / MINOR_TICK_MM)
    for i in range(steps_y + 1):
        y = y0 + i * MINOR_TICK_MM * mm
        pos_mm = i * MINOR_TICK_MM
        major = pos_mm % MAJOR_TICK_Y_MM == 0
        c.setLineWidth(1.2 * mm if major else 0.4 * mm)
        c.line(x0, y, x0 + (5 * mm if major else 2.5 * mm), y)
        if major and 0 < i < steps_y:
            c.setFont("Helvetica-Bold", 14)
            c.drawString(x0 + 7 * mm, y - 5, str(round(pos_mm / 10)))
    c.setFont("Helvetica-Bold", 11)
    c.saveState()
    c.translate(x0 + 20 * mm, (y0 + y1) / 2)
    c.rotate(90)
    c.drawCentredString(0, 0, "Y (cm)")
    c.restoreState()

    # Verification note below the rectangle.
    c.setFont("Helvetica", 9)
    c.drawCentredString(
        page_w / 2,
        y0 - 30 * mm,
        "After printing: measure this rectangle with a ruler to confirm "
        f"{width_mm / 10:.1f} x {height_mm / 10:.1f} cm before taping it down.",
    )
    c.setFont("Helvetica", 8)
    c.drawCentredString(
        page_w / 2,
        y0 - 37 * mm,
        "Click corners in order TL, TR, BR, BL "
        "(scripts/pick_ground_view_corners.py) to build the --corners value "
        "for examples/27_cam_ground_view_calibrate.py.",
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument(
        "--width-mm", type=float, default=200.0, help="rectangle width (default 200mm = 20cm)"
    )
    parser.add_argument(
        "--height-mm", type=float, default=150.0, help="rectangle height (default 150mm = 15cm)"
    )
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()

    if args.width_mm > 200 or args.height_mm > 220:
        print(
            "rectangle too large for the A4 layout margins used here", file=__import__("sys").stderr
        )
        return 1

    args.output.parent.mkdir(parents=True, exist_ok=True)
    c = canvas.Canvas(str(args.output), pagesize=A4)
    draw_target(c, args.width_mm, args.height_mm)
    c.showPage()
    c.save()
    print(f"saved {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

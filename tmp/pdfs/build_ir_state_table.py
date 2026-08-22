from reportlab.lib import colors
from reportlab.lib.enums import TA_CENTER, TA_LEFT
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle, getSampleStyleSheet
from reportlab.lib.units import mm
from reportlab.platypus import Paragraph, SimpleDocTemplate, Spacer, Table, TableStyle


OUT = "/Users/dannyyu/Desktop/IRsensorCar/output/pdf/ir-sensor-16-state-table-a4.pdf"

rows = [
    ("0000", "AMBIGUOUS", "Blind band or line lost", "-", "0.40", "Use previous reading to resolve"),
    ("0001", "DRIFT", "Far right, outer sensor only", "+3.2 cm", "0.13", "Steer right strongly"),
    ("0010", "DRIFT", "Drifting right, slight", "+1.0 cm", "0.73", "Steer right slightly"),
    ("0011", "JUNCTION", "Right pair; needs >2.8 cm black", "+1.8 cm", "0.40", "Junction / curve evidence"),
    ("0100", "DRIFT", "Drifting left, slight", "-1.0 cm", "0.73", "Steer left slightly"),
    ("0101", "NOISE", "Split dark regions", "-", "1.00", "Hold previous command"),
    ("0110", "ON LINE", "Centred", "0.0 cm", "1.00", "Drive straight"),
    ("0111", "JUNCTION", "Branch or curve on the right", "+1.6 cm", "0.40", "Junction / curve evidence"),
    ("1000", "DRIFT", "Far left, outer sensor only", "-3.2 cm", "0.13", "Steer left strongly"),
    ("1001", "NOISE", "Outer pair only", "-", "1.00", "Hold previous command"),
    ("1010", "NOISE", "Split dark regions", "-", "1.00", "Hold previous command"),
    ("1011", "NOISE", "P2 dropped out", "-", "1.00", "Hold previous command"),
    ("1100", "JUNCTION", "Left pair; needs >2.8 cm black", "-1.8 cm", "0.40", "Junction / curve evidence"),
    ("1101", "NOISE", "P3 dropped out", "-", "1.00", "Hold previous command"),
    ("1110", "JUNCTION", "Branch or curve on the left", "-1.6 cm", "0.40", "Junction / curve evidence"),
    ("1111", "JUNCTION", "Symmetric crossbar", "0.0 cm", "1.00", "Junction / crossbar evidence"),
]

styles = getSampleStyleSheet()
title = ParagraphStyle("title", parent=styles["Title"], fontName="Helvetica-Bold", fontSize=19, leading=22, alignment=TA_CENTER, textColor=colors.HexColor("#123047"), spaceAfter=3 * mm)
subtitle = ParagraphStyle("subtitle", parent=styles["Normal"], fontName="Helvetica", fontSize=8.5, leading=11, alignment=TA_CENTER, textColor=colors.HexColor("#405466"), spaceAfter=5 * mm)
small = ParagraphStyle("small", parent=styles["Normal"], fontName="Helvetica", fontSize=7.2, leading=9, alignment=TA_LEFT)
cell = ParagraphStyle("cell", parent=small, fontSize=7.4, leading=9)
cell_center = ParagraphStyle("cell_center", parent=cell, alignment=TA_CENTER)
header = ParagraphStyle("header", parent=small, fontName="Helvetica-Bold", fontSize=7.5, leading=9, textColor=colors.white, alignment=TA_CENTER)
note = ParagraphStyle("note", parent=small, fontSize=7.5, leading=10, textColor=colors.HexColor("#263746"))

doc = SimpleDocTemplate(
    OUT,
    pagesize=A4,
    rightMargin=13 * mm,
    leftMargin=13 * mm,
    topMargin=12 * mm,
    bottomMargin=11 * mm,
    title="IR Sensor 16-State Table",
    author="Car and Robotic Arm project",
)

story = [
    Paragraph("4-Channel IR Sensor - 16-State Table", title),
    Paragraph("Physical sensor order: P1 P2 P3 P4, left to right across the sensor bar. 1 = black line detected; 0 = white / no black line.", subtitle),
]

data = [[Paragraph(x, header) for x in ("P1 P2 P3 P4", "Class", "Interpretation", "Line offset", "Inner speed", "Operator / steering action")]]
for state, kind, meaning, offset, ratio, action in rows:
    data.append([
        Paragraph(f"<b>P{state}</b>", cell_center),
        Paragraph(kind, cell_center),
        Paragraph(meaning, cell),
        Paragraph(offset, cell_center),
        Paragraph(ratio, cell_center),
        Paragraph(action, cell),
    ])

table = Table(data, colWidths=[24 * mm, 23 * mm, 53 * mm, 22 * mm, 20 * mm, 42 * mm], repeatRows=1)
table.setStyle(TableStyle([
    ("BACKGROUND", (0, 0), (-1, 0), colors.HexColor("#123047")),
    ("TEXTCOLOR", (0, 0), (-1, 0), colors.white),
    ("GRID", (0, 0), (-1, -1), 0.35, colors.HexColor("#9AAAB5")),
    ("VALIGN", (0, 0), (-1, -1), "MIDDLE"),
    ("LEFTPADDING", (0, 0), (-1, -1), 3.2),
    ("RIGHTPADDING", (0, 0), (-1, -1), 3.2),
    ("TOPPADDING", (0, 0), (-1, -1), 3.5),
    ("BOTTOMPADDING", (0, 0), (-1, -1), 3.5),
]))

kind_colors = {
    "ON LINE": colors.HexColor("#D8F3DC"),
    "DRIFT": colors.HexColor("#FFF2CC"),
    "AMBIGUOUS": colors.HexColor("#FCE4D6"),
    "JUNCTION": colors.HexColor("#DDEBF7"),
    "NOISE": colors.HexColor("#E7E6E6"),
}
for row_index, row in enumerate(rows, start=1):
    table.setStyle(TableStyle([("BACKGROUND", (1, row_index), (1, row_index), kind_colors[row[1]])]))
    if row_index % 2 == 0:
        table.setStyle(TableStyle([("BACKGROUND", (0, row_index), (0, row_index), colors.HexColor("#F4F7F9")), ("BACKGROUND", (2, row_index), (-1, row_index), colors.HexColor("#F4F7F9"))]))

story.extend([table, Spacer(1, 5 * mm)])
story.append(Paragraph("Important: P0000 is context-dependent. After P0010 or P0100 it is the 0.8 cm blind band and steering continues toward the previous offset. After P0001 or P1000 it indicates genuine line loss. P0110 is the centred straight-line state. Non-contiguous patterns are noise and must not steer on their own.", note))
story.append(Spacer(1, 2.5 * mm))
story.append(Paragraph("Source: src/carbot/ir_geometry.py, STATE_TABLE. Print at 100% / Actual Size on A4; do not use Fit to Page if exact physical spacing is needed.", note))

doc.build(story)

# Handoff — 2026-08-16 (evening) Landmark localization: Phase-0 runbook

> **Read first**: [`docs/adr/0003-landmark-localization-task1.md`](adr/0003-landmark-localization-task1.md)
> (the decision: AprilTags, not QR codes; 20 mm tags; map frame conventions) ·
> `src/carbot/landmarks.py` (the localization layer) ·
> the line-follow handoff
> [`docs/handoff-2026-08-16-line-follow.md`](handoff-2026-08-16-line-follow.md)
> **remains the authority for the BEV/LineNav layer** — this handoff only adds
> the absolute-localization layer on top of it.

**Goal (operator-approved, 2026-08-16):** reprint the Task-1 map with
20 mm AprilTags printed in; Task-1 line-follow first. Until the new map
exists, validate the whole pipeline on the current map with taped tags —
**no motors are used in Phase 0**.

---

## 1. What exists now (all on the Mac repo, nothing committed)

| File | What it is |
|---|---|
| `src/carbot/landmarks.py` | Localization layer: tag-map JSON loading, flat-tag rotation convention, per-tag pose re-solve with the map's own size, multi-tag outlier rejection with single-tag fallback. Pure + unit-tested. |
| `tests/test_landmarks.py` | 24 tests; synthetic tags built through `cv2.projectPoints` + the real `estimate_square_pose`, so the round trip is the real pipeline. |
| `src/carbot/vision.py` | **Modified**: `estimate_square_pose` now tries both `SOLVEPNP_IPPE_SQUARE` (closed-form) and `SOLVEPNP_ITERATIVE` and returns the lower-residual positive-depth solution. IPPE_SQUARE's 4-fold symmetry returns wrong-branch solutions on noiseless corners for 90°-rotated tags (~4 px) and for tags viewed from the map's opposite side (~7 px); plain iterative is exact in both cases (verified 2026-08-16). Regression test in `tests/test_vision.py`. |
| `scripts/generate_apriltag_sheet.py` | Printable A4 PDF of 36h11 tags at **exact mm scale**, each with an N-arrow (yaw-0 orientation marker), corner crosses, a 100 mm scale bar, and a `--tag-map-out` JSON template. Defaults: 20 mm tag + 5 mm quiet zone, ids 0–15. Run with `uv run --with reportlab`. |
| `examples/31_cam_ground_tag_pose.py` | No-motor capture tool: detects flat tags, reports per-tag range/reprojection, and (with `--tag-map`) prints the camera's map-frame (x, y, heading) + writes annotated overlay + JSON. Captures at the 2028×1520 preview stream (the 4056×3040 still is unnecessary for pose — see ADR). |
| `scratch/landmarks/task1-tags-20mm.pdf` | The printable sheet (16 tags, ids 0–15). |
| `scratch/landmarks/task1-tag-map-draft.json` | Tag map with the draft positions pre-filled (2 cm size). Edit `x_m`/`y_m` if you tape elsewhere. |

Verified on the Mac: landmarks+vision tests pass, full suite 350 tests pass,
and an end-to-end synthetic run of example 31 recovered a camera at
(0.898, 0.499) m with heading −0.6° (true pose (0.90, 0.50), 0°).

**Convention in one line:** map frame **X east, Y north, Z up, origin at the
map's south-west corner, in metres** — SW = (0, 0), NE = (1.00, 0.70).
Measure **x from the west edge, y from the south edge**. A tag with its **N
arrow pointing map-north** has `yaw_deg = 0`. Heading: 0 = east, 90 = north,
180 = west, −90 = south. SSOT orthophoto pixels (NW origin, y down, 10 px =
1 cm) convert with `map_y_m = 0.70 − photo_y_px / 1000`.

---

## 2. Phase-0 runbook (operator steps, no motors)

### 2.1 Print the tag sheet

The sheet is already generated: `scratch/landmarks/task1-tags-20mm.pdf`.
To regenerate:

```bash
cd /Volumes/KLEVV-4T-1/Danny/Car-and-Robotic-Arm
uv run --with reportlab python3 scripts/generate_apriltag_sheet.py \
    --output scratch/landmarks/task1-tags-20mm.pdf \
    --tag-map-out scratch/landmarks/task1-tag-map-template.json
```

Print **at 100% / Actual Size** (not "fit to page" — printers scale by
default), then verify the **100 mm scale bar** with a ruler before cutting
(same procedure as the ground-view target: the printed calibration rectangle
previously measured exactly 10.0 × 5.0 cm). Cut out the 16 small squares;
each is 2 cm + 5 mm white border = 3 cm footprint.

### 2.2 Tape the tags on the current map — exact placement is NOT required

**You can tape the tags anywhere reasonable.** The robot does not care
whether a position is designed or measured — it only needs the *actual*
position in the tag map JSON. Two options:

- **Follow the draft table** below → keep the numbers already in
  `task1-tag-map-draft.json` (placement error of ±5 mm is irrelevant).
- **Random placement** → after taping, measure each tag's CENTER with a
  ruler: x = cm from the **west (left)** edge, y = cm from the **south
  (bottom)** edge; edit `x_m`/`y_m` in the JSON (`x_m = cm / 100`).

Placement rules (the only hard ones):
- **Never on or touching the 2 cm black line** (keep ≥ 2 cm away).
- Keep the N arrow pointing **map-north (the map's top edge)** → `yaw_deg = 0`.
- Stay within ~60 cm of the route (the camera detects a 2 cm tag reliably
  to ~60–70 cm; the ~20 cm grid keeps every tag within that range).
- If a spot lands on map text/artwork, shift it a few cm — the white border
  must stay clean.
- Corners of the map are good spots (the draft table uses them).

Draft positions (cm from west / south edges, yaw = 0):

| Tag ID | x (cm) | y (cm) | Why there |
|---|---|---|---|
| 0 | 5 | 65 | NW corner |
| 1 | 26 | 58 | Roundabout entry approach (NW quadrant) |
| 2 | 50 | 58 | Top straight, west third (drift check) |
| 3 | 72 | 58 | Top straight, east third (drift check) |
| 4 | 96 | 66 | NE corner |
| 5 | 95 | 42 | East side, mid (Phase 4 north) |
| 6 | 95 | 20 | East side, low |
| 7 | 95 | 5 | SE corner |
| 8 | 5 | 5 | SW corner |
| 9 | 5 | 25 | West side, mid (roundabout west edge) |
| 10 | 5 | 45 | West side, upper |
| 11 | 74 | 34 | T junction approach (visible from the start box) |
| 12 | 58 | 14 | Start zone |
| 13 | 60 | 40 | Interior (return-path drift check) |
| 14 | 86 | 25 | ARC 1 / end of Phase 2 |
| 15 | 52 | 23 | Roundabout 3-o'clock exit |

### 2.3 Fill the tag map JSON

Edit `scratch/landmarks/task1-tag-map.json` (start from the draft):
per tag, `x_m`/`y_m` in **metres** from west/south edges (`cm / 100`),
`yaw_deg: 0` (N arrow up), `size_m: 0.02`.

### 2.4 Sync to the Pi and capture

```bash
# Mac -> Pi (no motors anywhere in this phase)
scp src/carbot/landmarks.py src/carbot/vision.py carpi:~/Car-and-Robotic-Arm/src/carbot/
scp examples/31_cam_ground_tag_pose.py carpi:~/Car-and-Robotic-Arm/examples/
scp scratch/landmarks/task1-tag-map.json carpi:~/Car-and-Robotic-Arm/scratch/landmarks/

# Pi (car parked on the map, camera already focused for ~40 cm distance)
ssh carpi 'cd ~/Car-and-Robotic-Arm && PYTHONPATH=src python3 examples/31_cam_ground_tag_pose.py \
  --tag-map scratch/landmarks/task1-tag-map.json \
  --annotated-out /tmp/ground-tag-pose.jpg --json-out /tmp/ground-tag-pose.json'

# pull the overlay back to look at it
scp carpi:/tmp/ground-tag-pose.jpg /tmp/
```

Repeat from the approach poses that matter: parked in the start box facing the T,
parked near the NE corner facing north, parked near the NW corner facing
west, and parked just before the roundabout exit facing east. Each capture
prints `camera in map frame: x=… y=… heading=… deg` — compare with the tape
measure.

### 2.5 Phase-0 pass criteria

- Every taped tag is detected from at least one approach pose
  (`detected N tag(s)` ≥ 1) with `reprojection ≤ 3 px`.
- The reported (x, y) is within a few cm of the measured parked position and
  `heading` matches the car's facing (0 = east, 90 = north, 180 = west,
  −90 = south).
- The line detector does **not** lock onto tag borders: run
  `examples/25_cam_line_follow_capture.py` once with a tag in view and confirm
  the green cross stays on the 2 cm line. If it does lock on a tag, the width
  filter needs a capture-based check before Phase 2.

Then re-run `git status` and report the results; the tag positions that
actually work become the design positions for the **new map**.

---

## 3. Hardware facts (do not re-derive)

- Camera: IMX500, **manual focus only** (20 cm–infinity). If captures are
  soft, twist the focus ring for ~40–50 cm, do not assume geometry.
- Capture: 2028×1520 preview stream (same stream as the drive loop; fixed
  exposure 50 ms / gain 4.5 in example 31's capture). The 4056×3040 still is
  not needed for pose estimation.
- Mount: forward-tilted, offset right of the axle; ground-view BEV
  calibration absorbs the offset for line-follow.
- Drive: forward @ 200 ≈ 0.104 m/s, spin @ 200 ≈ 53.5 deg/s (time-based
  model, `src/carbot/motion.py`).
- Pi: `ssh carpi`, repo `~/Car-and-Robotic-Arm`; **check `git status` before
  assuming files are current** (it fell behind Mac `main` before).
- Motors: never in Phase 0/1. Phase 2+ requires the operator beside the car.

## 4. What the next session should do first

1. Read this file and the ADR; read the line-follow handoff if continuing
   line-follow work.
2. Run §2.4 with the operator; record the results in
   `docs/progress/2026-08-16-landmark-localization.md` (new progress log for
   this topic).
3. Whatever tag positions pass Phase 0 become the reprint design; then
   decide with the operator whether the new map reuses the corrected
   orthophoto base (`scratch/task1-fix/draw_corrected_route.py`) or the
   original Yahboom artwork with tags overlaid.
4. Phase 1: log standing poses (repeatability) with example 31.
5. Phase 2 (operator, motors): closed-loop heading turns — spin until
   `heading_deg` from the tags matches the plan, then Gate B+/C.

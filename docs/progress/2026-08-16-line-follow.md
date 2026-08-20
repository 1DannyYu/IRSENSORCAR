# 2026-08-16 — Line-follow on Yahboom paper: BEV plan, camera geometry, still not on-route

## Scope and result

Continued supervised line-follow on the Yahboom / YOURFUN Robot paper (2 cm black
line). The **agreed route** is: start box -> stem to first T -> **right** onto the
outer loop -> roundabout **counter-clockwise, exit 3** -> back to the start box.

**Morning result:** topology of the paper is understood and drawn. Closed-loop
driving still does **not** follow that route. The last two supervised runs
(45 s then 20 s @ speed 150) moved for ~7–8 s with stuttering steer/stop, then
held still on `search: hold`. No verified 90° T-turn. Operator described the
motion as wandering. See "Afternoon session" below — this was fixed.

**Afternoon session result:** camera remounted (still forward-tilted, but
higher and closer to chassis centre) at the operator's initiative. Ground-view
(bird's-eye) homography was **calibrated on the robot for the first time**
(previous sessions wrote the code but never ran the calibration). Gate A
(green cross locks the real 2 cm line, not map noise) **passed**, including
with the calibration target and poem text both in frame. Gate B (closed-loop
drive, operator beside the car): **the car drove forward and steered
correctly for ~1–2 s** (first confirmed correct closed-loop segment in this
project's history — err converged from +209 px to ~0, and the saved frames
show the camera view actually advancing along the paper), then a spurious
detection near the T-junction/outer-loop curve triggered a safety stop
(`jump: stop` → `search: hold`, motors to 0). **No spin, no drive-off-paper.**
Full stem-to-T traversal and the T-turn itself are not yet verified.

Intentionally not committed (user did not ask).

## Verification

```text
# Mac
uv run pytest -q tests/test_line_follow.py tests/test_line_nav.py tests/test_ground_view.py
  # 65+ tests in line_follow/nav; ground_view tests exist if the extra is installed

# Pi (operator beside the car)
PYTHONPATH=src python3 examples/25_cam_line_follow_capture.py --output /tmp/line-follow
printf "yes\n" | PYTHONPATH=src python3 examples/26_cam_line_follow_drive.py \
  --duration 20 --start-turn-s 0 --speed 150
```

Operator-observed (2026-08-16 ~03:10–03:20): car in the start box, then ~6–8 s of
lurching (strong right-wheel cut, jump-stops), then motionless until the script
ended. Not a clean stem follow and not a T-right.

## Measurements and configuration

| Item | Value | Notes |
|---|---|---|
| Line width | 2 cm | User-confirmed |
| Stem 2 cm width on inventory BEV | ~27 px | `scratch/line-follow-2026-08-15/` |
| Scale from that width | 13.5 px/cm | Photo BEV, folds remain |
| Planned path length | ~259 cm | stem ~10, outer ~126, roundabout ~93, return ~30 |
| Travel @ speed 200 / 150 | 0.117 m/s / ~0.088 m/s | existing calibration |
| Spin @ speed 200 | 53.5 deg/s | examples/23 |
| Fixed exposure | 50 ms, gain 4.5 | examples/25 and 26 |
| Capture size | 2028×1520 | IMX500 preview |
| Near 2 cm width in camera | ~110–150 px | start-zone stills |
| Far T / poem bar | ~18 px, often `err=0` at x=1014 | forward-tilted camera |
| Camera mount | **right of axle**, tilted **forward/down** | user photos IMG_0576, IMG_0578 |
| `expected_center_fraction` | 0.46 | not 0.5; unmeasured offset |
| `t_bar_min_width_px` | 70 | far bars must not spin |
| SEARCH | hold still (`L0 R0`) | spin-search walked off paper |
| Roundabout FSM | **off** (`--roundabout` not set) | |

Scratch (Mac, not committed):
`scratch/line-follow-2026-08-15/` — overlays, BEV maps,
`planned-route-inventory.jpg`, `draw_planned_route.py`.

Paper topology (start box at **bottom** of rotated BEV): stem **up** to T; **right**
= outer rounded rectangle (poem); **left** = YOURFUN circle. Inventory photo
`assets/inventory/101_Yahboom_Line_Tracking_Track_Instructions.jpg` is the same
map rotated ~90° (start box on the **right**).

## Problems encountered

1. **Forward camera sees the whole map.** From the start box the T and poem appear as a
   horizontal bar. Early code treated that as “arrived at T” and spun or drove
   off the top of the paper. Timed 90° rights were rejected by the operator.
2. **Detector lock is not unique.** Start-box edges, stem, far T, and chairs all
   produce 2 cm-scale dark strips. Overlay: **one green cross** is the steer
   target; cyan = other candidates. Green was often on a box edge or far bar,
   not the stem under the wheels.
3. **Jump-stop.** `max_error_jump=0.35` stops on lock flips. Combined with
   multi-branch start-box views, the car stutters then loses the line and holds.
4. **Do not drive back from tiles.** Off-paper views lock chairs/grout. Operator
   must place the car in the start box; a 0.7 s @ 150 open-loop nudge (~6 cm) was used
   once from inside the box.
5. **Ground-view homography** (`src/carbot/ground_view.py`, example 27) was
   written and unit-tested but **never calibrated on the robot** (no
   `/tmp/line-follow/ground-view.json` on the Pi). Phone BEV of the paper is
   planning only, not the live controller.

## Afternoon session — ground-view calibration and first correct closed-loop segment

### Why: the camera has no autofocus

Confirmed via web search (Raspberry Pi's own AI Camera docs): the IMX500 AI
Camera is **manual focus only**, focus range 20 cm–infinity, no autofocus
actuator. Several capture attempts this session were badly blurred simply
because the lens's manual focus ring was set for a different distance than
whatever the operator had just moved the camera to — not a mount-angle
problem. The operator re-focused by hand (physically twisting the lens
barrel) between captures until text and line edges were sharp.

### Calibration target: three iterations, converging on thick lines

`scripts/generate_ground_view_target.py` (new, `uv run --with reportlab`,
reportlab is not a project dependency) generates a printable rectangle with
crosshair corners (TL/TR/BR/BL, matching `scripts/pick_ground_view_corners.py`
click order) and cm axis ticks, for the `--corners` calibration path of
`examples/27_cam_ground_view_calibrate.py`.

- v1 (20×15 cm, then 10×7.5 cm): hairline (0.75 pt) border and small (6–9 pt)
  labels. **Consistently the blurriest thing in every capture** even after
  the focus was fixed for the black track line and poem text at the same
  distance — a thin line loses its edge to blur long before a thick one does.
- Final (10×5 cm, operator-specified): border widened to **5 mm** (thick,
  same visual weight as the printed track line), corner labels 22 pt,
  axis-tick numbers 14 pt, X labelled every 1 cm, Y every 1 cm. This was
  legible in every subsequent capture, including handheld-focus ones that
  were still slightly soft.
- Operator confirmed the printed rectangle measured **exactly 10.0 × 5.0 cm**
  with a ruler.

### Calibration performed

Corners were extracted with `cv2.findContours` (outer + inner edge of the 5mm
border, averaged for the stroke centreline — the design width in
`generate_ground_view_target.py` is the centreline, not the outer edge) rather
than by hand-clicking, since the border gave enough contrast for a reliable
threshold. Final corner pixels (image space, TL/TR/BR/BL):
`(866.5,616) (1431.5,614.5) (1496.5,866.5) (868,869.5)`.

`examples/27_cam_ground_view_calibrate.py --corners ... --size-m 0.10,0.05
--near-m 0.18` was run, but **`--near-m 0.18` is an unmeasured guess**, not a
tape-measure reading from any physical reference point on the chassis — there
was no time to measure it and, as reasoned through in-session, it mostly
sets an additive offset on the world-y (forward-distance) axis rather than
affecting the homography's correctness, since the calibration rectangle's
own width/height (measured, accurate) is what the fit actually depends on.
Do not treat any absolute forward-distance number derived from this
calibration as measured truth.

`examples/27`'s default BEV window (`y_min_m=0.12, y_max_m=0.72`) **excluded
the real track line** — the near-field stem/T bar visible at the bottom of
every raw capture projected to a world-y below 0.12 m and was invisible in
the bird's-eye crop, producing `no line` even though the homography itself
was correct (verified by warping the full frame and visually confirming the
calibration target rectified into a clean rectangle). Recalibrated directly
via `carbot.ground_view.calibrate_ground_view(..., y_min_m=-0.10,
y_max_m=0.90)` (bypassing the example script, which does not expose these as
CLI flags — worth adding). This made the real line visible and detectable:
`width=10px` (matches `expected_line_width_px=10.0` for the 2 cm line at
2 mm/px) with `rows=11–13`.

Ground-view file used for Gate A/B this session:
`scratch/line-follow-2026-08-16/ground-view.json` (Mac copy, canonical) /
`/tmp/line-follow/ground-view.json` on the Pi (**ephemeral — `/tmp` does not
survive a reboot**; copy it into the repo's `scratch/` or re-run calibration
before the next session if it's gone).

### Gate A — PASSED

`examples/25_cam_line_follow_capture.py --ground-view <path>`: green cross landed
exactly centred on the real 2 cm line (zoomed-pixel check, not just the
overlay) in every capture, including with the calibration target and the full
poem text both in frame (deliberately left in as a noise test at the
operator's request). `candidates` in the debug overlay showed multiple
non-chosen detections (from the target/text) that were correctly rejected by
the existing width filter.

### Gate B — partial pass (first correct segment, then a safe stop)

`examples/26_cam_line_follow_drive.py --duration 8 --speed 150 --ground-view
<path> --save-every 10`, operator beside the car, power-cut ready:

- Frames 1–22 (0.0–1.0 s): `err` converged smoothly from **+209 px to −7 px**,
  wheel speeds adjusted proportionally (`L150 R43` → `L150 R150` deadband).
  Saved frames #10/#20 vs. the first frame show the camera's field of view
  visibly advancing along the paper — **the chassis moved correctly under
  closed-loop control**, not just the wheels spinning in place.
- ~1.1 s: a detection jump to `err=+821px x=1835` (a different, wider dark
  feature — width still 10px but `rows=19`, vs. 6–13 for the real line)
  triggered `jump: stop`. A few more genuine-looking readings and jumps
  alternated until ~2.4 s, then the reading settled onto the spurious
  feature permanently (`err=+794px x=1808 width=10px rows=29-31`, unchanging)
  and the state machine went to `search: hold` (`L0 R0`) for the rest of the
  8 s — motors stayed at zero, car did not move again.
- Saved frame #160 (end of run) shows the **outer-loop curve now near the top
  of frame** and the calibration target lower down — confirming the chassis
  had physically advanced well past its start position before stopping. The
  spurious lock is most likely that outer-loop curve or the T-junction's own
  cross-bar entering the BEV's far range and being misread as the main line.
- **No spin-in-place, no drive-off-paper** — the two failure modes that ended
  every previous session's run. The safety logic (`jump: stop`, non-spinning
  `search: hold`) did what it was designed to do.

## Follow-up

See [`docs/handoff-2026-08-16-line-follow.md`](../handoff-2026-08-16-line-follow.md)
(rewritten this afternoon — the morning version calling closed-loop follow
"not working" is stale). The root-cause sensing problem from the morning
session (perspective ambiguity between near/far dark features) is resolved by
the ground-view calibration. The next problem is narrower: the BEV detector
needs to distinguish the real line from the outer-loop curve / T cross-bar
when they enter its far range, rather than needing a whole new sensing
approach.

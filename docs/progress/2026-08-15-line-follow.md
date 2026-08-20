# 2026-08-15 — Black-line tracking: detection, nav state machine, closed-loop script

## Scope and result

Started the black-line tracking feature for the mod-100 Yahboom track map
(outer loop + roundabout; route: start zone -> turn right -> outer loop ->
roundabout one anti-clockwise lap -> continue -> exit -> back to start zone).
The Raspberry Pi 5 reboot fixed the previously unresponsive IMX500 AI Camera.
A downward still calibrated the line detector; then a pure detection module, a
navigation state machine, and a closed-loop drive script were built, and the
drive loop was exercised on the Pi under supervision (motors ran, the car did
**not** yet track the line reliably — see Problems).

New files (working tree, not committed):

- `src/carbot/line_follow.py` — downward line detection (pure functions)
- `src/carbot/line_nav.py` — follow/search/roundabout state machine (pure)
- `tests/test_line_follow.py` — 18 tests
- `tests/test_line_nav.py` — 17 tests
- `examples/25_cam_line_follow_capture.py` — capture one frame + annotated overlay (no motors)
- `examples/26_cam_line_follow_drive.py` — closed-loop drive (motors; supervised runs only)

## Verification

- Camera after reboot: `python3 examples/05_ai_camera_check.py --photo` — all PASS,
  4056x3040 still (1726594 bytes).
- Detector on real stills (Mac, `uv run --extra vision`): consistent single line
  (x≈3793/4056, width≈230px) on the first start-zone capture.
- Pi capture: `PYTHONPATH=src python3 examples/25_cam_line_follow_capture.py` —
  `line err=+912px (+0.90) x=1926 width=130px rows=113` on the 2028x1520 preview.
- Full suite: `uv run python3 -m pytest tests/` — 250 passed (before the
  afternoon changes; line-follow tests alone: 35 passed, `tests/test_line_follow.py`
  + `tests/test_line_nav.py`).
- `uv run ruff check` — clean.
- Dry-run loop on the Pi: `examples/26_cam_line_follow_drive.py --dry-run --duration 8`
  — ~17 fps, detection stable, state machine emitted sane commands, motors never
  energised.
- **Supervised on-map runs (motors energised, operator beside the car): the car
  did not yet follow the line reliably** — it veered, then spun in small circles,
  then drove straight off the map. Detailed in Problems below.

## Measurements and configuration

- Camera: IMX500, 4056x3040 stills; 2028x1520 preview for the loop. Track paper
  background gray ≈ 208; line ≈ 2.3 % of pixels on the first start-zone still.
- **Exposure drift (key finding): with auto-exposure the frame mean dropped from
  ~195 (static) to ~118-124 while the car moved**, pushing map background under the
  line threshold. Fixed exposure stops this. Values tested at the map:
  `ExposureTime=50000us, AnalogueGain=4.5` → mean 204, dark<100 = 5.5 %
  (auto: mean 197, dark 7.8 %). `examples/26` now sets fixed exposure by default.
- **Line width ≈ 2 cm** (user-confirmed; ≈115px at 2028 width on the first still,
  detected widths vary 100-320px depending on what is being tracked).
- `LinePolicy`: `dark_threshold=100`, `roi_top=0.10`, `roi_bottom=0.68`,
  `min_row_dark_fraction=0.002`, `branch_gap_fraction=0.04`,
  `min_branch_rows_fraction=0.05`.
- `NavPolicy`: `speed=200`, `turn_gain=2.5` (raised from 0.45 — see Problems),
  `min_ratio=0.15`, `search_timeout_s=4.0`, `junction_min_s=1.0`,
  `roundabout_loop_min_s=6.5` (anchored to 53.5 deg/s @ speed 200, examples/23),
  `junction_width_factor=1.5`, `expected_center_fraction=0.571` (see Problems).
- Camera-to-chassis alignment: the camera was **repositioned by the user** during
  the session; its optical axis is not centred on the car's heading. Parking the
  car with its nose along the line put the line at 0.571 of the frame width.

## Problems encountered (chronological, this session)

1. **Camera unresponsive** after first boot — Pi reboot fixed it.
2. **First junction detector over-reported.** Global 1-D clustering split the
   curved main line and counted map print (10-12 rows) as branches. Fixed with
   row-to-row segment tracking (`_track_segments`) plus
   `min_branch_rows_fraction=0.05`.
3. **Auto-exposure drift while moving** (finding above). Detection fell apart as
   soon as the wheels moved. Fixed exposure in `examples/26`; needs re-verification
   on a moving run.
4. **Steering gain far too weak.** `turn_gain=0.45` turned a 138 px offset into a
   12-speed wheel difference (L200 R188); the car drove almost straight, parallel
   to the line, and ran off the map. Raised to 2.5.
5. **Detector main-line flips** (x jumped +0.91 → -0.34 in one frame). Added
   target locking (`LineNav._locked`): steer at the candidate nearest the last
   frame's target, with a release gap (`_LOCK_RELEASE_GAP=0.15`) when the line
   leaves the view.
6. **Forced launch right turn (1.5 s) overshot** — the car turned past the line
   and kept chasing it in circles. `--start-turn-s 0` now lets line-following
   turn naturally. The user's plan says the route *starts* with a right turn, but
   the line itself bends right at the start zone, so no forced turn is needed.
7. **The car spins in small circles / drives off the map on real runs.** The
   locked target stays at a nearly constant frame position (err constant ≈ ±0.5)
   while the car rotates — i.e. it may be chasing a fixed dark structure in the
   frame (chassis/support visible to the camera, or an environmental shadow)
   rather than the 2 cm line. **UNRESOLVED.** Evidence: user confirms the black
   line is "the darkest, thickest, 2 cm" line; but the detected main line
   position differed between runs (x=302, 660, 1233, 1678 on different parkings),
   far more than parking repeatability should allow, and the car circles instead
   of converging. It is not yet confirmed that the detector's main line is the
   real black line. The next engineer must visually confirm this before tuning
   anything else (see handoff).
8. **Parking repeatability is poor.** The user re-parked the car between runs;
   the detected line position moved by up to ±45 % of the frame width, so any
   single `expected_center_fraction` calibration is fragile.

## Follow-up (see `docs/handoff-2026-08-15-line-follow.md`)

1. Visually confirm which dark structure the detector tracks is the real 2 cm
   black line (human eyes are needed; this agent has none).
2. Re-establish `expected_center_fraction` from a confirmed frame, or make
   steering robust to it.
3. Resolve the circling behaviour (fixed target ≈ chasing a chassis/shadow
   feature, or gain/geometry oscillation).
4. Only then tune roundabout entry/exit and run the full route.
5. Supervised runs: `examples/14_all_sensors_preflight_check.py` first, operator beside the
   car, `--dry-run` before any motor run. Keep the car on the map — the room is
   small, walls/wardrobes/mirrors are right outside it.

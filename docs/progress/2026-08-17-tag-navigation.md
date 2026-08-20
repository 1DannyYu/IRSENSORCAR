# 2026-08-17 — Tag-supervised black-line navigation on the Task-1 map

## Scope and result

Redesigned the black-line navigation for the Task-1 reprint map
(`scratch/landmarks/task1-map.pdf`, 15 mm line, 32 AprilTags, SW-origin map
frame x east / y north, metres) and integrated the AprilTag localization
layer (`src/carbot/landmarks.py`, ADR 0003) as a supervisor.

**Result (honest):** the black-line layer now tracks the 15 mm stem reliably
and steadily (multi-second closed-loop runs on the real map), and the
tag-supervised layer correctly gates departure and vetoes early turns. **The
full route is NOT achieved**: the car still drifts off after the stem segment,
because (a) the 2026-08-14 camera intrinsics are wrong for this lens
(verified below) so tag absolute position is off by 15-25 cm, and (b) with a
bad position the vision T-turn fires on the wrong structure. The intrinsics
must be recalibrated before tag supervision can be trusted for turns.

> **Update (same session, run-13):** the "intrinsics are wrong" conclusion
> above was itself a false alarm from a bug in the *diagnostic* script (the
> aruco corner order — corner0 is the tag's north-west corner, not
> south-west — was wrong in my ad-hoc analysis, so every PNP came out
> mirrored). The project's own pipeline (`vision._tag_object_points` +
> `landmarks.localize_camera`) is correct: with the old fx=1553 intrinsics it
> localised the departure-zone camera to x=0.550 y=0.177 z=0.242 heading
> 94.3°, all sane. The real root cause of the recurring "drives north-west
> from departure" was that the stem sits inside the camera blind zone at
> launch, so the detector locked a departure-zone structure at map x≈0.54
> instead of the stem at 0.59. Fix: a DEPART phase that aligns the heading
> to map-north (tag heading), blind-creeps straight, and only switches to
> line-follow when a centred narrow stroke (the stem) appears. **run-13
> (operator beside the car): departure confirm → heading align 75→90 →
> blind creep 1 s → stem lock → stem follow → position-confirmed T right
> turn (anchored y=0.226) → 90° spin → Phase 2 follow → turn cooldown.**
> Position stayed x≈0.55-0.57 with y increasing monotonically — **no
> north-west drift**. The remaining work is the rest of the lap (Phase 2 →
> outer loop → roundabout → return), which the route plan
> (`carbot.route_plan.task1_route`) already describes.

Nothing committed (user did not ask).

## Verification (what actually ran)

```text
# Mac: unit tests
uv run pytest -q tests/                    # 369 passed
uv run pytest -q tests/test_tag_nav.py     # 7 passed (new supervision tests)

# Pi (operator beside the car, power-cut ready) — examples/32_cam_tag_nav_drive.py
# run-10: departure confirmed (0.2 s), stem tracking steady 0-5.5 s
#   (err +0.04..+0.09, L120 R120), then the car was pulled left by a
#   structure the detector locked at x≈757-983 and drifted to x≈0.35
#   (map frame) — 24 cm west of the stem corridor; a vision cross-bar
#   "spin right" then fired at 10.5 s from the wrong place.
```

## Measurements and configuration

| Item | Value | Notes |
|---|---|---|
| Camera height above floor | 0.28 m | operator-measured |
| Camera ahead of chassis centre | 0.095 m | operator-measured |
| Camera blind zone (nearest visible ground) | ~0.17 m from chassis centre | after the operator re-tilted the camera (was ~0.35 m) |
| Ground-view window | y 0.17-0.35 m, x ±0.30 m | recalibrated per move (`examples/27 --auto --near-m 0.35`) |
| BEV edge exclusion | 0.22 of width each side | kills the raw-frame right-edge shadow band (u≈0.84) |
| Line width | 0.015 m | `LinePolicy.line_width_m` / `GroundView.line_width_m` |
| T-turn | anchored position y ≥ 0.20 → fixed 90° spin | `TagNavPolicy` |
| Nav speed | 120 | ~0.070 m/s |

## Problems encountered

1. **The 2026-08-14 camera intrinsics do not match this lens.** Multi-tag
   joint fit with the height fixed at the measured 0.28 m leaves 532 px RMS
   (scipy least-squares on `again-raw.jpg`, 7 tags). Single-tag PNP returns
   camera heights of 0.00-0.12 m (IPPE) or negative (ITERATIVE). Tag
   positions were confirmed exact (±5 mm) by the operator, so the error is
   in fx/distortion (the old file even notes
   "Recalibrate if the camera module, lens, focus, or mounting orientation
   changes"). The old ChArUco board PDF is also obsolete (28 mm pitch board
   was the measured one, not the 30 mm PDF).
2. **Camera tilt vs tag coverage is a fundamental trade-off.** The operator
   re-tilted the camera nearly vertical for black-line visibility (blind
   zone 35 → 17 cm), which cuts tag coverage to 1-2 tags per frame near the
   departure zone → position goes stale during the stem run.
3. **Off-track false alarms from tag-position jumps.** Raw per-tag position
   jumped x 0.25-0.84 between frames (different tag inliers); fixed with a
   temporal outlier rejection in the median filter, but the anchor-corrected
   position still inherits the intrinsic offset.
4. **Vision T-turn fires off the wrong structure once the car drifts.**
   With the car 24 cm west, a near "cross-bar" (likely a tag's black frame)
   triggered the right spin; the anchored position (y 0.11) correctly said
   "not at the T" but was stale so the veto was skipped by design.

## Follow-up

1. Recalibrate the intrinsics from the map's own AprilTags (positions exact):
   capture ≥3 views with the car rotated 30-60° between them
   (`examples/33_cam_tag_self_calibrate.py` exists but needs views with ≥2 tags
   per frame — with the near-vertical camera this requires rotating the car
   to face several tags, or temporarily raising the camera).
2. Re-verify tag localization: camera height must come back ≈0.28 m and the
   multi-tag position spread <5 cm.
3. Then re-run the TagNav full route (departure → stem → T → Phase 2).

Scratch evidence (Mac, not committed):
`scratch/line-follow-2026-08-17/` — captured frames,
`/tmp/linef-new/` — analysis copies, `scratch/landmarks/task1-map.pdf` — map.

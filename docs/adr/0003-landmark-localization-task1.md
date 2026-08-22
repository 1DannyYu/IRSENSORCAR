# ADR 0003: Flat AprilTag Landmarks for Task-1 Navigation

- **Status**: adopted
- **Date**: 2026-08-16
- **Supersedes**: the heuristic-pile-up direction in
  [`docs/handoff-2026-08-16-line-follow.md`](../handoff-2026-08-16-line-follow.md)
  (its Gates B+/C/D remain the acceptance gates, but junction decisions no
  longer come from dark-strip heuristics alone)

## Context

The chassis has **no wheel encoders and no IMU** (`config.HAS_ENCODERS` is
False). Every absolute-position question ("am I at the T?", "have I rounded
the roundabout?") is currently answered by heuristics over a single dark-strip
reading from the forward-tilted camera: `NavPolicy` carries ~28 tunables
(`junction_width_factor`, `t_bar_min_width_px`, `right_turn_after_s`,
`roundabout_loop_min_s`, …), each added after one observed failure mode. The
state of the art on 2026-08-16:

- Gate A (BEV detector locks the real 2 cm line) — **passed**.
- Gate B (closed-loop follow) — **passed with a caveat**: the car loses lock
  when the outer-loop curve or the T cross-bar enters the BEV far range, and
  the `jump: stop` safety holds it safely.
- Gate B+ (full stem → T traversal), Gate C (T right), Gate D (outer loop +
  roundabout exit 3) — **not achieved**; each attempt added another heuristic.

The operator was asked to choose a direction and chose: **reprint the map with
landmarks printed in, Task-1 line-follow first** (2026-08-16).

## Decision

Add an **absolute localization layer** to the Task-1 navigation stack: a small
number of **AprilTag 36h11** fiducials printed flat on the map (or taped on
the current map while the new artwork is being made). Each visible mapped tag
gives the camera a 6-DOF pose in the map frame; aggregated tags give the
camera's (x, y, heading) directly.

- **Fiducial: AprilTag 36h11, not QR codes.** QR codes are designed for data
  scanning; AprilTags are designed for pose estimation (subpixel corner
  refinement, robust at grazing angles, metric size known by design). The
  project already has a hardware-verified tag pipeline
  (`src/carbot/vision.py`: intrinsics, `detect_apriltag_poses`,
  `camera_world_pose_from_tag`, `aggregate_camera_world_poses`; verified on
  the wall tags, examples 12/13/24).
- **Convention** (see `src/carbot/landmarks.py`): map frame **X east, Y north,
  Z up, origin at the map's south-west corner, in metres** — SW = (0, 0),
  NE = (1.00, 0.70). This is the operator's chosen frame (2026-08-16):
  positions are measured from the west and south map edges, and a
  north-facing car has a positive heading (0 = east, 90 = north, 180 = west,
  −90 = south). The SSOT orthophoto pixels (NW origin, y down, 10 px = 1 cm)
  convert with `map_y_m = 0.70 − photo_y_px / 1000`. A tag with `yaw_deg =
  0` has its ID upright when viewed from map-north (printed +X points
  map-east); `yaw_deg` is the angle of that +X axis counterclockwise from
  map-east. The sheet generator prints an N arrow per tag so placement is
  unambiguous.
- **Tag sizes**: **20 mm black square with a 5 mm white quiet zone** (30 mm
  footprint) — the operator's limit for a 100 × 70 cm map (2026-08-16).
  Feasibility (real intrinsics fx ≈ 1553 at the 2028×1520 preview): a 20 mm
  tag spans ~78 px at 40 cm and ~52 px at 60 cm (≥5 px/module), so tags are
  reliably detected to ~60–70 cm; precision is ~2–5 mm position and
  ~0.5–2° heading — far beyond the 2 cm line width and the ~2–3° turn
  tolerance. Tags must not overlap the 2 cm track line; the pattern's white
  interior leaves only a ~1 px black ring in the BEV, which the line
  detector's width filter rejects (to be re-verified by capture once tags
  are on the map).
- **Capture resolution**: the drive and localization loops both use the
  **2028×1520 preview** stream (the stream the line-follow pipeline is
  calibrated on). The 4056×3040 still is **not needed** for pose
  estimation (~78 px vs ~155 px tag span at 40 cm — both far above the
  detection floor). A future 1280×960 (4:3) 720p-class mode would raise the
  frame rate and cut SfM transfer size, but requires re-validating the
  line-follow/ground-view calibration at the new resolution; it is a
  deliberate later step, not part of Phase 0.
- **Placement** (draft, to be confirmed by Phase-0 captures): **16 tags —
  one at each map corner plus a ~20 cm grid** along and around the route
  (corners, T junction, both long straights, roundabout entry and 3-o'clock
  exit, start zone). Positions are listed in the handoff and the draft tag
  map (`scratch/landmarks/task1-tag-map-draft.json`); the capture example
  validates visibility before the new map is printed. Exact placement is not
  required — measured positions are as good as designed ones.
- **Software layer** (new, `src/carbot/landmarks.py`): tag-map JSON (id →
  designed position/yaw/size), per-tag pose re-solved with the map's own size
  (a wrong detection size cannot silently scale localization), outlier
  rejection across tags, graceful single-tag fallback. Pure and unit-tested;
  `examples/ai_camera/31_cam_ground_tag_pose.py` is the no-motor validation tool.

## Why this fixes the heuristic pile-up

1. **Junction decisions become deterministic.** "At the T" means
   "localization says I am within 3 cm of the T waypoint", not "a fat dark
   bar persisted 0.2 s in the lower ROI".
2. **Turns become closed-loop.** Heading comes from the tag pose; a turn is
   "spin until heading = planned heading", not a timed spin (timed turns are
   the historical cause of every drive-off-paper).
3. **Recovery after losing the line.** Any single tag in view re-establishes
   (x, y, heading); the car can reorient instead of waiting out a search
   sweep.
4. **The line-follow layer stays as-is** between landmarks: BEV
   ground-view + `LineNav` remain the smooth-steering low layer. Tags are a
   separate detection channel and are *not* noise for the line detector
   (their white quiet zone and small footprint are rejected by the existing
   width filter — to be re-verified by capture once tags are on the map).

## Consequences

- The **new map artwork** must embed the tags at the designed coordinates and
  print at exact 100 × 70 cm; the SSOT phase table remains the authority for
  track geometry.
- The existing map's photo-measured geometry stays valid for the tape-on
  validation phase; the new map's landmark positions come from design values.
- Phase 2 must measure the fixed camera-to-chassis offset once (or derive it
  from a parked pose) before the localization feeds chassis-level pose
  consumers; junction decisions and heading turns do not need it.
- The 2026-08-15/16 heuristic `NavPolicy` parameters are **not removed** in
  this ADR — they keep the car safe while the localization layer is proven.
  A later ADR may retire them one by one as gates pass.
- SfM photo mapping (ADR 0002) gains a side benefit: every captured frame
  that sees a mapped tag is geotagged in the map frame.

## Acceptance gates (unchanged from the line-follow handoff)

- **Phase 0** (no motors): printed tags detected and localized from the real
  camera at the real mount angles; overlay + JSON match the car's parked
  position within a few cm. (Tool: `examples/ai_camera/31_cam_ground_tag_pose.py`.)
- **Phase 1** (no motors): localization module on the Pi with the taped tags;
  a standing pose log agrees with ruler measurements.
- **Phase 2** (operator beside the car): closed-loop heading turns; Gate B+
  and Gate C pass with localization-triggered turns.
- **Phase 3**: Gate D — outer loop + roundabout exit 3 with the tag at the
  exit.

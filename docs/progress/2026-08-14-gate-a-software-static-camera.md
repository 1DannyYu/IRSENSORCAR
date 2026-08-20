# Gate A — Pure Software and Static Camera Integration (2026-08-14)

## 1. Scope and Result

Implemented **Gate A** from
[`docs/handoff-2026-08-14-vision-to-mapping.md`](../handoff-2026-08-14-vision-to-mapping.md):
pure-software and static-camera fixes only. **No motor or servo command was
sent at any point.** The car was never constructed for motion
(`Car()` is only created after operator confirmation in the repaired
scripts).

Changed:

- **Pi synced** safely from `0bbcd41` to GitHub `origin/main` `d62e5e9`
  (ff-only, untracked runtime copies preserved in `stash@{0}`; latest
  `room-pose-wall.{json,jpg}` evidence backed up to `/tmp/pi-runtime-evidence/`).
- **`src/carbot/sonar.py` (new)** — single shared HC-SR04 implementation with
  injected GPIO/clock, so timeout paths are unit-testable. Replaces the
  duplicated `measure()` in examples 09/10/11 (behaviour unchanged).
- **`src/carbot/frames.py` (new)** — `scan_angle_rad()` (elapsed -> angle from
  the *configured* spin duration), typed `Pose2D` (world heading `atan2(Y,X)`,
  local forward on +y), `SensorExtrinsics` (sensor -> chassis -> world chain
  with explicit offset/yaw, values to be measured in Gate B).
- **`src/carbot/mapping.py`** — `load_polar_scan` now uses `scan_angle_rad`;
  `polar_to_points` docstring fixed: the sensor frame convention is
  **angle=0 -> +y** (`x=d·sinθ, y=d·cosθ`), which the code always did.
- **`examples/10_sonar_motion_calibrate.py`** — `--spin-seconds` now takes effect
  (spin leg uses `args.spin_seconds`); `--spin-speed` added and defaults to
  150, the only speed with a verified 8.2 s/rev; `Car()` is created only
  after operator confirmation; every exit path stops/closes/cleans up.
- **`examples/11_sonar_explore_mapping.py`** — `--spin-speed` and `--drive-speed`
  separated; scan angles derived from the run's `--spin360` via
  `scan_angle_rad`; the speed-150 timing is no longer applied at speed 200;
  operator confirmation before `Car()`; uses `sonar.Sonar`.
- **`examples/09_sonar_room_scan.py`** — `--spin-360` default corrected 8.0 -> 8.2
  (verified value), `--speed` default 150 paired with it, shared `sonar.Sonar`,
  operator confirmation before `Car()`.
- **`src/carbot/vision.py`** — new `anchor_tags()`; examples/13 now rejects a
  duplicated anchor ID explicitly (`duplicate-anchor` frame status) instead of
  silently picking the first detection.
- **`examples/13_cam_room_pose.py`** — consumes `anchor_tags()`; duplicate ID 0
  fails clearly; missing anchor still yields `missing-anchor` status.
- **`examples/06/07/08`** — 8 Ruff findings cleaned (EXE001 x3 via `chmod +x`,
  PLR0402 x4 via `from RPi import GPIO`, PLW1510 x1 via `check=False`); no
  hardware behaviour change.
- **`examples/14_all_sensors_preflight_check.py` (new)** — no-motion preflight: camera
  (Picamera2 open/close), I2C (`NeZha(init_motors=False)` + reset), HC-SR04
  (sensor-only readings), power (`EXT5V_V`, `get_throttled`, temperature),
  encoder availability. Never constructs `Car()`.
- **Tests** — new `tests/test_frames.py` (30 tests), `tests/test_sonar.py`
  (5 tests), extended `tests/test_vision.py` (anchor_tags / duplicate /
  not-visible, 14 vision tests).

## 2. Verification

Mac (this machine):

```text
uv run pytest -q                      -> 97 passed in 27.51s
uv run --extra vision pytest tests/test_vision.py -q  -> 14 passed
uv run ruff check .                   -> All checks passed!
```

New regression coverage (Gate A acceptance criteria):

| Criterion | Test |
|---|---|
| scan angle uses the configured spin360 | `test_scan_angle_uses_configured_spin360`, `test_load_polar_scan_uses_recorded_spin360`, wrap tests |
| axis order / sign contract | `test_polar_to_points_axis_signs`, `test_pose_forward_axis_points_along_heading`, `test_sensor_points_to_world_chain` |
| ultrasonic timeout | `test_measure_returns_none_when_echo_never_heard`, `..._stuck_high`, normal-pulse distance |
| duplicate anchor ID fails clearly | `test_anchor_tags_duplicate_detection_is_visible_to_caller` + 13's `duplicate-anchor` status |
| anchor temporarily not visible | `test_anchor_tags_not_visible`, `test_anchor_not_visible_in_synthetic_blank_image` |
| known transforms | Pose2D / SensorExtrinsics round-trips and known points |
| no hard-coded speed-150 timing at 200 | script 11 spins at `--spin-speed` (150) and drives at `--drive-speed` (200); 8.2 s bound to spin speed |
| no motor object before confirmation | scripts 09/10/11 create `Car()` only after `yes` |

Raspberry Pi (via `ssh carpi`, static checks only, no motion):

```text
PYTHONPATH=src python3 examples/14_all_sensors_preflight_check.py
  [OK] camera     picamera2 opened and closed a still configuration
  [OK] i2c/nezha  NeZha responded at 0x40 bus 1 (reset OK)
  [OK] hc-sr04    HC-SR04 responded: 3/3 readings, avg 54.9 cm
  [FAIL] power    EXT5V_V=4.895 V (OK); get_throttled=0x50000 (throttling NOW); temp=37.8 C (OK)
  [OK] encoders   config.HAS_ENCODERS=False (two-wire motors; encoder reads expected 0)

PYTHONPATH=src python3 examples/13_cam_room_pose.py --anchor-height-cm 14.65
  Room pose from 5/5 inlier frames:
  wall_distance=61.20 cm  wall_right=-0.82 cm  height=11.07 cm
  heading=+179.87 deg  elevation=+16.85 deg
  Inlier position ranges: X=0.25 cm  Y=0.00 cm  Z=0.05 cm
```

The 13 re-run matches the earlier verified baseline
(handoff: 61.33 cm / -0.87 cm / 11.05 cm / 179.82 deg / 16.86 deg) within
0.13 cm / 0.05 cm / 0.02 cm / 0.05 deg, confirming no vision regression and
that the car is still in front of the fixed wall anchor.

The one preflight FAIL is the known persistent battery undervoltage
(`get_throttled=0x50000`, also recorded in
`docs/progress/2026-08-14-sensors-and-ai-camera.md`). The preflight correctly
blocks motion until it is resolved.

## 3. Measurements and Configuration

- Sensor frame convention locked: angle=0 -> +y (sensor forward),
  `x=d·sinθ, y=d·cosθ` — matches `examples/11` odometry axis.
- World frame: X away from anchor wall, Y along wall right, Z up; 2D heading
  `atan2(Y, X)` (matches `vision.CameraWorldPose.heading_deg`).
- `Pose2D` forward axis is local +y; `from_xy_heading(H)` builds
  `R = [[sin H, cos H], [-cos H, sin H]]` so local +y maps to world heading H.
- `SensorExtrinsics`: offset in cm in chassis frame (x right, y forward);
  `yaw_deg` = sensor forward vs chassis forward (0 aligned, + counter-clockwise
  / left turn). Values intentionally **not** measured yet (Gate B).
- Spin timing: 8.2 s/rev is bound to spin speed 150 only; scripts 09/10/11
  now carry and use their own configured values.
- Pi state at end of session: repo on `origin/main` `d62e5e9`, clean working
  tree, runtime copies preserved in `stash@{0}`.

## 4. Problems Encountered

1. **Old annotated image cannot re-validate 13.** `/tmp/room-pose-wall.jpg`
   (the annotated output) yields 0 AprilTag detections (ChArUco 24 corners but
   reprojection 3.49 px > 3.0). Re-validated by live re-capture instead
   (5/5 inliers, see §2). Annotated images must not be reused as detection
   input.
2. **One bad scp.** A batch `scp` sent example files into
   `~/Car-and-Robotic-Arm/src/carbot/` on the Pi. Cleaned up immediately
   (removed 8 stray files) and re-synced with correct paths; Pi `git status`
   afterwards matched the intended change set.
3. **Heading convention trap (tests).** The first draft of `Pose2D` used the
   standard `[[cos,-sin],[sin,cos]]` matrix, which maps local +y to world
   heading `H+90°` — self-inconsistent with `heading_deg`. Fixed to
   `[[sin H, cos H], [-cos H, sin H]]` and locked with axis tests. Three new
   tests initially asserted the wrong expectation (world (110,10) instead of
   (110,5); `atan2` range; matrix inverse form) — corrected after hand-checking
   the math.

## 5. Follow-up

Gate A acceptance is met except the power FAIL, which is hardware state
(recharge/replace the 1200 mAh pack) and intentionally blocks motion.

Measurements still needed for Gate B (manual repositioning, still no motor
commands):

1. Camera optical centre offset relative to the chassis rotation centre (cm).
2. HC-SR04 measurement origin relative to the chassis rotation centre (cm).
3. Camera yaw relative to chassis forward (sign + degrees).
4. HC-SR04 yaw relative to chassis forward (sign + degrees).
5. Re-measure spin360 for the exact spin speed Gate C will use, at the
   current battery voltage.

Suggested next step: Gate B — operator repositions the stopped car to 3-5
marked floor locations, five frames each, and the operator measures the four
offsets above with a tape/protractor. No motor commands.

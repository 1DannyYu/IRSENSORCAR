# Examples Naming Convention — Tool-First Script Names

Date: 2026-08-19

## Scope and Result

`examples/` had no naming rule in [CONVENTIONS.md](../../CONVENTIONS.md), so script names accreted
ad hoc and stopped saying which hardware they drive. The trigger was
`38_map1_line_follow.py` vs `39_map1_ir_tracking.py`: two scripts for the same Map1 route, one
camera-guided and one IR-guided, with nothing in either filename to tell them apart.

Changed:

- **Added [CONVENTIONS.md §3.6](../../CONVENTIONS.md)** — `NN_<tool>_<function>[_<mode>].py`, with
  a closed tool vocabulary (`i2c`, `motor`, `servo`, `cam`, `sonar`, `ir`, `power`, `all_sensors`).
  Existing §3.6/§3.7 renumbered to §3.7/§3.8; no document referenced those numbers.
- **Renamed 31 of 39 scripts** with `git mv`, keeping every `NN` prefix unchanged. Numbers are
  never reused or reordered (same rule as asset photos, §3.3), and progress logs cite scripts by
  number, so only the descriptive part moved.
- **Updated 70 files / 314 inbound references** across `README.md`, `docs/progress/`,
  `docs/handoff-*`, `docs/setup/`, `docs/hardware/`, `docs/adr/`, `src/carbot/`, `scripts/`,
  `tests/`, `Tasks/`, and the untracked `.claude/settings.local.json` permission entries.
- **Fixed a false docstring** in `38_map1_cam_line_follow.py`: it claimed "downward camera **and IR
  sensor** guidance" but imports only `carbot.line_follow` / `carbot.line_nav` and never reads the
  IR sensor. The docstring now states camera-only and points at the IR sibling.
- **Unified vocabulary**: `39_map1_ir_tracking` → `39_map1_ir_line_follow`, so the camera and IR
  versions of the same route read as a pair.

Intentionally unchanged: `01_i2c_probe`, `02_motor_check`, `04_servo_check`, `05_ai_camera_check`,
`06_ultrasonic_avoidance`, `08_battery_check`, `36_ir_tracing_check`, `37_map1_motor_test` — each
already names its tool. No script logic was modified; this is a naming-only change plus one
docstring correction.

## Rename Table

| Before | After | Hidden tool |
|---|---|---|
| `03_drive.py` | `03_motor_drive.py` | motor |
| `07_obstacle_avoidance_drive.py` | `07_sonar_avoidance_drive.py` | sonar |
| `09_room_scan.py` | `09_sonar_room_scan.py` | sonar |
| `10_calibrate_motion.py` | `10_sonar_motion_calibrate.py` | sonar |
| `11_explore_mapping.py` | `11_sonar_explore_mapping.py` | sonar |
| `12_apriltag_pose.py` | `12_cam_apriltag_pose.py` | cam |
| `13_room_pose.py` | `13_cam_room_pose.py` | cam |
| `14_preflight_check.py` | `14_all_sensors_preflight_check.py` | all |
| `15_gate_b_pose_log.py` | `15_cam_gate_b_pose_log.py` | cam |
| `16_capture_room.py` | `16_cam_room_capture.py` | cam |
| `17_patrol_capture.py` | `17_cam_patrol_capture.py` | cam |
| `18_wall_follow_capture.py` | `18_sonar_wall_follow_capture.py` | sonar |
| `20_visual_detection_check.py` | `20_cam_detection_check.py` | cam |
| `21_camera_dual_mode_check.py` | `21_cam_dual_mode_check.py` | cam |
| `22_fused_patrol_capture.py` | `22_cam_sonar_patrol_capture.py` | cam + sonar |
| `23_spin_rate_check.py` | `23_cam_spin_rate_check.py` | cam |
| `24_linear_speed_check.py` | `24_cam_linear_speed_check.py` | cam (AprilTag) |
| `25_line_follow_capture.py` | `25_cam_line_follow_capture.py` | cam |
| `26_line_follow_drive.py` | `26_cam_line_follow_drive.py` | cam |
| `27_ground_view_calibrate.py` | `27_cam_ground_view_calibrate.py` | cam |
| `29_route_nav_drive.py` | `29_cam_route_nav_drive.py` | cam |
| `30_motion_calibrate.py` | `30_cam_motion_calibrate.py` | cam |
| `31_ground_tag_pose.py` | `31_cam_ground_tag_pose.py` | cam |
| `32_tag_nav_drive.py` | `32_cam_tag_nav_drive.py` | cam |
| `33_self_calibrate_tags.py` | `33_cam_tag_self_calibrate.py` | cam |
| `34_collect_tag_views.py` | `34_cam_tag_view_collect.py` | cam |
| `35_object_id_check.py` | `35_cam_object_id_check.py` | cam |
| `38_map1_line_follow.py` | `38_map1_cam_line_follow.py` | cam |
| `39_map1_ir_tracking.py` | `39_map1_ir_line_follow.py` | ir |
| `40_spin_duration_check.py` | `40_motor_spin_duration_check.py` | motor (operator-measured) |
| `41_spin_angle_sweep.py` | `41_motor_spin_angle_sweep.py` | motor (operator-measured) |

`23` vs `40`/`41` is the clearest payoff: all three measure spin, but `23` measures it with the
camera while `40`/`41` are pure timed spins graded by the operator with a protractor. The old names
gave no way to know that.

## Verification

```bash
uv run python -m pytest -q            # 395 passed
uv run python -m compileall -q examples/   # rc=0, all 39 scripts parse
```

Reference sweep after the rename found zero occurrences of any old script name across all tracked
and untracked files. Every example's own `Usage:` block cites its new filename; `04_servo_check.py`
never cited itself, which was already true before this change.

No hardware was run. This change touches no motor, sensor, or I2C code path.

## Problems Encountered

The first bulk-replace pass silently edited nothing. Two causes: the shell `&&` chain
short-circuited before the replace loop ran, and the file list was collected with a plain
`for f in $FILES` word-split that broke on the space-containing `Tasks/IR Sensor Tracking/` paths
(that directory has since been renamed to `tasks/ir-sensor-tracking/`).
Corrected by driving the replace from `git ls-files -z -co --exclude-standard` with a
NUL-delimited read and an extension allowlist, which also guarantees `.venv/`, `.ruff_cache/`, and
`node_modules/` binaries are never rewritten.

## Follow-up

- The tool vocabulary table in §3.6 must be extended in the same commit that adds a new sensor
  script; a new tag invented ad hoc reopens exactly this problem.
- Related open question from the same session: [ir-tracing-sensor.md](../hardware/ir-tracing-sensor.md)
  documents the physical channel order as `Out4, Out3, Out1, Out2` with a dead zone between the two
  halves, but the vendor photo shows a tight middle tracking pair (P2/P3) flanked by two widely
  spaced edge detectors (P1/P4). `src/carbot/ir_line_nav.py` weights all four channels equally and
  never distinguishes tracking from edge channels. Re-verify the physical order and channel roles
  with `examples/36_ir_tracing_check.py` by sliding a black line across the bar and logging the
  order in which channels trip.

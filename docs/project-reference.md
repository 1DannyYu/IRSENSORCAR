# Car and Robotic Arm — Full Technical Reference

> This is the detailed operator/engineering reference (hardware bring-up, the full example script
> table, the IR workflow runbook, SSH access). New to the repo? Start at [README.md](../README.md)
> instead — it links back here for the details.

![Smart car and robotic arm build](../assets/assembly/021_RobotCar_With_RoboticArm_Combined.jpg)

A Raspberry Pi 5 smart car project with real hardware notes, verified wiring, and step-by-step build records.

[Danny's GitHub Repository](https://github.com/1DannyYu/IRSENSORCAR) · [Contact Danny on GitHub](https://github.com/1DannyYu) · [Live Site](https://1dannyyu.github.io/IRSENSORCAR/) · [Inventory](https://1dannyyu.github.io/IRSENSORCAR/inventory/) · [Assembly Guide](https://1dannyyu.github.io/IRSENSORCAR/assembly-guide/)

## Why This Repo

- Verified Raspberry Pi 5 to NeZha I2C communication on real hardware
- Real photos, wiring notes, and assembly references collected in one place
- Runnable Python checks for bring-up, motor mapping, and basic driving
- An English project site for browsing parts and build progress

## Current Status

| Area | Status |
|---|---|
| I2C communication | Verified at address `0x40` |
| Motor mapping | Verified and written back into `src/carbot/config.py` |
| Driving test | First low-speed ground run passed |
| AI Camera (IMX500) | Intrinsics, undistortion, still capture, and fixed-wall room pose verified (2026-08) |
| Obstacle sensor (HC-SR04) | Wiring verified, distance readings OK (2026-08) |
| Room mapping | Vision anchor ready; autonomous exploration scripts require timing fixes before use |
| Robotic arm | Still evolving because of damaged parts and compatibility tradeoffs |

## Start Here

1. Read the verified bring-up guide: [docs/setup/raspberry-pi-first-run.md](setup/raspberry-pi-first-run.md)
2. Browse the project website: [Live Site](https://1dannyyu.github.io/IRSENSORCAR/)
3. Run the hardware checks in order:

```bash
uv sync
uv run python examples/other/01_i2c_probe.py              # I2C link to the NeZha board — no moving parts
uv run python examples/other/02_motor_check.py            # ⚠️ lift the car; operator beside it
uv run python examples/other/03_motor_drive.py            # ⚠️ low-speed ground run; operator beside it
uv run python examples/other/04_servo_check.py            # ⚠️ arm servos; operator beside it
python3 examples/ai_camera/05_ai_camera_check.py --photo      # AI Camera (IMX500) — system interpreter
python3 examples/other/06_ultrasonic_avoidance.py         # HC-SR04 obstacle detector — no moving parts
PYTHONPATH=src python3 examples/other/07_sonar_avoidance_drive.py --dry-run  # sensor only first
PYTHONPATH=src python3 examples/other/07_sonar_avoidance_drive.py            # ⚠️ avoidance run, operator beside it
PYTHONPATH=src python3 examples/other/08_battery_check.py  # battery / power health — no moving parts
PYTHONPATH=src python3 examples/other/09_sonar_room_scan.py     # ⚠️ spin-scan the room (HC-SR04), operator beside it
PYTHONPATH=src python3 examples/other/10_sonar_motion_calibrate.py  # ⚠️ drive/spin calibration, operator beside it
PYTHONPATH=src python3 examples/other/11_sonar_explore_mapping.py   # ⚠️ M3 exploration loop, operator beside it
PYTHONPATH=src python3 examples/ai_camera/12_cam_apriltag_pose.py  # static AprilTag pose; no motors
PYTHONPATH=src python3 examples/ai_camera/13_cam_room_pose.py --anchor-height-cm 14.65  # fixed-wall room pose
PYTHONPATH=src python3 examples/other/14_all_sensors_preflight_check.py  # no-motion preflight — run before any motion test
PYTHONPATH=src python3 examples/ai_camera/15_cam_gate_b_pose_log.py --anchor-height-cm 14.65  # Gate B pose log (static)
PYTHONPATH=src python3 examples/ai_camera/16_cam_room_capture.py --duration 90  # room sweep for SfM (push the car)
```

The chassis and arm are controlled by the **Yourfun NeZha bus driver board**. A Raspberry Pi 5
communicates with the board over I2C at address `0x40` to drive four DC motors, four servo
channels, the onboard LEDs, and optional encoder inputs.

The board ships with Arduino, STM32, and C51 driver code and no protocol specification. This
project has no microcontroller, so the command set was reconstructed from those three vendor SDKs
and reimplemented in Python against the Pi's hardware I2C — see
[ADR 0004](adr/0004-nezha-python-driver-port.md) for the decision and its translation
tradeoffs, and [docs/hardware/nezha-i2c-protocol.md](hardware/nezha-i2c-protocol.md) for the
resulting command reference.

## Hardware

| Item | Model |
|---|---|
| Main controller | Raspberry Pi 5 |
| Driver board | Yourfun NeZha bus driver board (`0x40` over I2C) |
| Chassis | Dasheng multi-form robot car chassis, 4x N20 motors |
| Robotic arm | Desktop-class 3-DOF arm |
| Battery | HXS 18650 11.1V 1200mAh |

## Wiring: NeZha to Raspberry Pi 5 (Verified)

| Raspberry Pi Pin | BCM GPIO | NeZha Signal | Purpose |
|---|---|---|---|
| Pin 3 | GPIO 2 | SDA | I2C data |
| Pin 4 | Power | 5V | Power supply |
| Pin 5 | GPIO 3 | SCL | I2C clock |
| Pin 6 | Ground | GND | Ground reference |

For complete wiring notes, see [docs/hardware/nezha-integration-notes.md](hardware/nezha-integration-notes.md).

## SSH Access to Raspberry Pi 5

The Raspberry Pi is configured for key-based authentication. Connect via SSH alias:

```bash
ssh carpi
```

or explicitly:

```bash
ssh dannypi@danny-raspberrypi5-8gram-225gssd.local
```

**Authentication:** Ed25519 key (`~/.ssh/id_ed25519`), not password.  
**User:** `dannypi`  
**Hostname:** `danny-raspberrypi5-8gram-225gssd.local` (mDNS, requires local network)

After connecting, navigate to the project:

```bash
cd ~/Car-and-Robotic-Arm
```

If the repository is not present yet, clone the current GitHub repository:

```bash
git clone https://github.com/1DannyYu/IRSENSORCAR.git Car-and-Robotic-Arm
cd Car-and-Robotic-Arm
uv sync
```

---

## Map 1 IR Workflow: Edit, Push, Pull, and Test One Phase

This is the repeatable operator workflow for the Map 1 IR tracking task. Edit on the Mac, validate
locally, push the commit to GitHub, pull it onto the Raspberry Pi, and then run exactly one bounded
phase with a timestamped log.

### Safety

Motor-moving commands may run over SSH only when an operator is physically beside the car and can
cut main power immediately. Flatten and secure the entire paper map, clear the chassis underside,
and never continue to the next phase when the current phase fails.

### Which file controls each setting?

| What to change | Source of truth | Notes |
|---|---|---|
| Phase 1-10 distances, headings, and instructions | `src/carbot/map1_phases.py` | Edit `MAP1_PHASES`; Phase 1 constants are immediately above it |
| Phase 1 forward time and PWM | `src/carbot/map1_phases.py` | `PHASE1_FORWARD_S`, `PHASE1_FORWARD_PWM` |
| Phase 1 desired turn and extra pulse compensation | `src/carbot/map1_phases.py` | `PHASE1_RIGHT_TURN_DEG`, `PHASE1_RIGHT_TURN_COMPENSATION_DEG` |
| Phase 2 acquisition, line following, search, and phase-test behavior | `examples/other/39_map1_ir_line_follow.py` | Shared production path used by both integrated and independent tests |
| Run one independent Phase | `examples/other/40_map1_ir_phase_test.py` | Thin wrapper; normally select with `--phase`, do not copy the controller |
| Automated regression expectations | `tests/other/test_map1_phases.py`, `tests/other/test_example_39_hardcoded_start.py`, `tests/other/test_ir_line_nav.py` | Update only when the intended behavior changes |
| Operator procedure | `tasks/ir-sensor-tracking/run-book.md` | Detailed hardware gates and troubleshooting |
| English route diagram | `tasks/ir-sensor-tracking/route-planning.html` | Documentation only; editing it does not change motor behavior |
| Physical-test LOG files | `scratch/ir-sensor-tracking/` | Local evidence; intentionally not committed |

The ten phase specifications currently are:

| Phase | Entry pose | Planned action |
|---:|---|---|
| 1 | Start stem, heading north | Sensor-blind 16cm departure, then desired 90° right turn |
| 2 | East straight, heading east | Actively centre on `P0110`, then follow east for 15.5cm |
| 3 | ARC 1 entry, heading east | Follow the 12cm arc and exit heading north |
| 4 | North straight, heading north | Follow north for 18cm |
| 5 | ARC 2 entry, heading north | Follow the 12cm arc and exit heading west |
| 6 | West straight, heading west | Follow west for 47cm |
| 7 | ARC 3 entry, heading west | Follow the 12cm arc toward the roundabout |
| 8 | Roundabout approach, heading southwest | Follow the 7.5cm approach |
| 9 | Roundabout entry, heading southwest | Follow 270° and take the mapped exit |
| 10 | Final straight, heading west | Follow west for 21.5cm and stop |

### 1. Edit and inspect locally

```bash
cd /Users/dannyyu/Desktop/IRsensorCar
git status --short
git diff
git diff --check
```

Do not edit `vendor/`. Do not use `git add .` when unrelated changes are present; stage the exact
files that belong to the change.

For phase constants and IR controller changes, run the focused checks:

```bash
uv run ruff check \
  src/carbot/map1_phases.py \
  examples/other/39_map1_ir_line_follow.py \
  tests/other/test_map1_phases.py \
  tests/other/test_example_39_hardcoded_start.py

uv run python -m pytest -q \
  tests/other/test_map1_phases.py \
  tests/other/test_example_39_hardcoded_start.py \
  tests/other/test_car.py \
  tests/other/test_ir_line_nav.py \
  tests/other/test_ir_route.py
```

### 2. Commit and push to GitHub

Stage only the files intentionally edited, review the staged diff, then commit and push the local
`main` branch through the configured `danny` remote:

```bash
git add \
  src/carbot/map1_phases.py \
  examples/other/39_map1_ir_line_follow.py \
  tests/other/test_map1_phases.py \
  tests/other/test_example_39_hardcoded_start.py \
  tasks/ir-sensor-tracking/run-book.md

git diff --cached
git diff --cached --check
git commit -m "fix(ir-tracking): describe the change"
git push danny main
git log -1 --oneline
```

Remove paths from `git add` when they were not edited, and add other intentional paths explicitly.
If `git commit` reports that nothing is staged, stop and inspect `git status --short` instead of
forcing a commit.

### 3. Pull safely on the Raspberry Pi

This command refuses to pull when the Pi has uncommitted tracked changes. Untracked LOG files under
`scratch/` do not block it.

```bash
ssh carpi 'cd ~/Car-and-Robotic-Arm && if test -n "$(git status --porcelain --untracked-files=no)"; then echo "ERROR: Pi has uncommitted tracked changes; pull cancelled."; git status --short --untracked-files=no; exit 1; fi && git pull --ff-only origin main && echo "PI HEAD=$(git rev-parse --short HEAD)"'
```

Compare the printed Pi commit with `git log -1 --oneline` on the Mac before a motor test. Never use
`git reset --hard` merely to make a pull succeed.

### 4. Run any one Phase with one editable variable

Change only `PHASE=1` at the beginning to any integer from 1 through 10. The same value selects the
controller phase and generates the LOG filename automatically.

```bash
ssh -t carpi 'cd ~/Car-and-Robotic-Arm && PHASE=1 && PHASE_TAG=$(printf "%02d" "$PHASE") && mkdir -p scratch/ir-sensor-tracking && LOG="scratch/ir-sensor-tracking/$(date +%Y-%m-%d-%H%M%S)-phase${PHASE_TAG}-20s.log" && echo "COMMIT=$(git rev-parse --short HEAD) PHASE=$PHASE LOG=$LOG" && PYTHONPATH=src python3 -u examples/other/40_map1_ir_phase_test.py --phase "$PHASE" --duration 20 --heartbeat-s 2 --speed 150 --start-acquire-timeout-s 5 2>&1 | tee "$LOG"'
```

The text `phase01` inside a LOG filename does **not** control the program. Only `--phase "$PHASE"`
selects the phase. `--phase` accepts one integer, not a range such as `1-4`.

Phase-specific notes:

- Phase 1 ignores all sensors, drives forward using the current calibrated pulse, stops, and then
  performs its compensated stationary right turn.
- Independent Phase 2 actively pivots toward a visible offset and requires stable centred `P0110`
  before its 15.5cm counter starts. The generic command above gives acquisition a bounded five-
  second window with `--start-acquire-timeout-s 5`.
- Independent Phase 3/5/7/9 curve tests retain the requested base PWM; `--speed 150` is no longer
  silently reduced to PWM 90. Their bounded curve mode also steers on directional three-sensor
  patterns such as `P0111/P1110`; otherwise a tight curve would be mistaken for an unconfirmed
  junction and the controller would hold a stale straight command.
- Phases 2-10 require manual placement at the documented entry pose. Test one phase, inspect its
  result, and reposition the stopped car before starting another phase.

### 5. Review or copy the LOG

List recent evidence on the Pi:

```bash
ssh carpi 'cd ~/Car-and-Robotic-Arm && ls -lt scratch/ir-sensor-tracking | head -20'
```

Read one LOG by replacing the filename:

```bash
ssh carpi 'cd ~/Car-and-Robotic-Arm && sed -n "1,260p" scratch/ir-sensor-tracking/YYYY-MM-DD-HHMMSS-phase02-20s.log'
```

Copy all Phase LOG files back to the Mac without deleting the Pi copies:

```bash
mkdir -p /Users/dannyyu/Desktop/IRsensorCar/scratch/ir-sensor-tracking
scp 'carpi:~/Car-and-Robotic-Arm/scratch/ir-sensor-tracking/*.log' \
  /Users/dannyyu/Desktop/IRsensorCar/scratch/ir-sensor-tracking/
```

Only after all ten independent phases pass should the integrated route be tested. The detailed
gates, expected sensor patterns, and troubleshooting procedure are in
[`tasks/ir-sensor-tracking/run-book.md`](../tasks/ir-sensor-tracking/run-book.md).

---

## Quick Start

Run the examples in order. Scripts `01`, `05`-`06`, `08`, and `12`-`15` are safe to run over SSH
(no motors or servos move); motor-moving scripts (`02`-`04`, `07`, `09`-`11`) require an operator
standing beside the robot who can cut main power instantly. Run `14` before any motion test.

Script names are `NN_<tool>_<function>.py`, so the filename tells you which hardware it drives —
`cam` (IMX500), `sonar` (HC-SR04), `ir` (IR tracing sensor), `motor`, `servo`, `i2c`, `power`. See
[CONVENTIONS.md §3.6](../CONVENTIONS.md#36-runnable-scripts-in-examples-nn_tool_function_modepy).

| # | Script | What it checks | Run with | Safety |
|---|---|---|---|---|
| 01 | `examples/other/01_i2c_probe.py` | I2C link to the NeZha driver board at `0x40`, reset command, head LED path | `uv run python examples/other/01_i2c_probe.py` | ✅ No moving parts |
| 02 | `examples/other/02_motor_check.py` | Wheel → motor port mapping (`M1`-`M4`) and forward/reverse direction | `uv run python examples/other/02_motor_check.py` | ⚠️ Lift the car, operator beside it |
| 03 | `examples/other/03_motor_drive.py` | Minimal differential-drive movement on the ground | `uv run python examples/other/03_motor_drive.py` | ⚠️ Low speed, operator beside it |
| 04 | `examples/other/04_servo_check.py` | Arm servos `S2`-`S4`, one channel at a time | `uv run python examples/other/04_servo_check.py` | ⚠️ Operator beside it |
| 05 | `examples/ai_camera/05_ai_camera_check.py` | AI Camera (IMX500) detected, picamera2 capture, models listed; `--photo` saves a still, `--inference` runs an on-sensor object-detection pass (first run uploads the model to the camera — takes a few minutes) | `python3 examples/ai_camera/05_ai_camera_check.py --photo` | ✅ No moving parts (use system interpreter, not `uv`) |
| 06 | `examples/other/06_ultrasonic_avoidance.py` | HC-SR04 obstacle detector: distance readings + obstacle warning; `--trials`/`--threshold` to tune | `python3 examples/other/06_ultrasonic_avoidance.py` | ✅ No moving parts |
| 07 | `examples/other/07_sonar_avoidance_drive.py` | Closed-loop avoidance: HC-SR04 drives the car (forward / stop + spin); `--dry-run` tests the sensor loop only | `PYTHONPATH=src python3 examples/other/07_sonar_avoidance_drive.py` | ⚠️ Operator beside it; lifted by default, `--ground` for a floor run |
| 08 | `examples/other/08_battery_check.py` | Battery / power health: `EXT5V_V`, `get_throttled` bits (live vs since-boot), temperature | `PYTHONPATH=src python3 examples/other/08_battery_check.py` | ✅ No moving parts |
| 09 | `examples/other/09_sonar_room_scan.py` | Room spin-scan (M1): logs the HC-SR04 polar distance profile while the car spins; one frame of the mapping loop | `PYTHONPATH=src python3 examples/other/09_sonar_room_scan.py` | ⚠️ Operator beside it (lifted or floor) |
| 10 | `examples/other/10_sonar_motion_calibrate.py` | Drive-speed and spin calibration with the HC-SR04; honours `--spin-seconds`/`--spin-speed`; confirms before constructing `Car()` | `PYTHONPATH=src python3 examples/other/10_sonar_motion_calibrate.py` | ⚠️ Operator beside it (lifted or floor) |
| 11 | `examples/other/11_sonar_explore_mapping.py` | M3 exploration loop: spin-scan -> ICP -> grid -> small step; `--spin-speed`/`--drive-speed` separated | `PYTHONPATH=src python3 examples/other/11_sonar_explore_mapping.py` | ⚠️ Operator beside it (lifted or floor) |
| 12 | `examples/ai_camera/12_cam_apriltag_pose.py` | Static AprilTag 36h11 metric pose and undistorted image | `PYTHONPATH=src python3 examples/ai_camera/12_cam_apriltag_pose.py` | ✅ No moving parts |
| 13 | `examples/ai_camera/13_cam_room_pose.py` | Five-frame ChArUco + AprilTag fixed-wall room pose with outlier rejection and JSON output | `PYTHONPATH=src python3 examples/ai_camera/13_cam_room_pose.py --anchor-height-cm 14.65` | ✅ No moving parts |
| 14 | `examples/other/14_all_sensors_preflight_check.py` | No-motion preflight: camera, I2C, HC-SR04, power, encoders — run before any motion test | `PYTHONPATH=src python3 examples/other/14_all_sensors_preflight_check.py` | ✅ No moving parts |
| 15 | `examples/ai_camera/15_cam_gate_b_pose_log.py` | Gate B manual-reposition pose log: per-location repeatability + displacement vs tape | `PYTHONPATH=src python3 examples/ai_camera/15_cam_gate_b_pose_log.py --anchor-height-cm 14.65` | ✅ No moving parts (operator moves the car by hand) |
| 16 | `examples/ai_camera/16_cam_room_capture.py` | Room sweep of stills for Structure-from-Motion (push the stopped car; capture every `--interval` s) | `PYTHONPATH=src python3 examples/ai_camera/16_cam_room_capture.py --duration 90` | ✅ No moving parts (operator pushes the car) |
| 17 | `examples/ai_camera/17_cam_patrol_capture.py` | Roomba-style random-bounce patrol + capture (superseded by planned vision fusion) | `PYTHONPATH=src python3 examples/ai_camera/17_cam_patrol_capture.py --frames 150` | ⚠️ Operator beside it |
| 18 | `examples/other/18_sonar_wall_follow_capture.py` | Single-sonar wall-following patrol + capture (same sonar limitation) | `PYTHONPATH=src python3 examples/other/18_sonar_wall_follow_capture.py --frames 150` | ⚠️ Operator beside it |
| 20 | `examples/ai_camera/20_cam_detection_check.py` | IMX500 on-sensor object detection for visual avoidance (`OBSTACLE AHEAD`) | `PYTHONPATH=src python3 examples/ai_camera/20_cam_detection_check.py` | ✅ No moving parts |
| 21 | `examples/ai_camera/21_cam_dual_mode_check.py` | Camera experiments for the patrol: compares `single`/`switch`/`restart` capture modes, and sweeps auto-exposure settings ranked by repeatable keypoints | `PYTHONPATH=src python3 examples/ai_camera/21_cam_dual_mode_check.py` | ✅ No moving parts |
| 22 | `examples/ai_camera/22_cam_sonar_patrol_capture.py` | Vision + sonar fused patrol with 2028×1520 SfM capture — avoids the chairs a single sonar cannot see | `PYTHONPATH=src python3 examples/ai_camera/22_cam_sonar_patrol_capture.py --dry-run --frames 10` | ⚠️ Operator beside it (`--dry-run` is safe) |
| 23 | `examples/ai_camera/23_cam_spin_rate_check.py` | Measures the real spin rate and startup dead time from the camera's own view — no protractor, no encoders | `PYTHONPATH=src python3 examples/ai_camera/23_cam_spin_rate_check.py --speed 200` | ⚠️ Operator beside it |
| 24 | `examples/ai_camera/24_cam_linear_speed_check.py` | Measures forward and reverse travel distance against a wall AprilTag | `PYTHONPATH=src python3 examples/ai_camera/24_cam_linear_speed_check.py --speed 200` | ⚠️ Operator beside it (drives toward a wall) |
| 25 | `examples/ai_camera/25_cam_line_follow_capture.py` | Line-follow capture + overlay: confirms the green cross sits on the real 2 cm line (Gate A) | `PYTHONPATH=src python3 examples/ai_camera/25_cam_line_follow_capture.py` | ✅ No moving parts |
| 26 | `examples/ai_camera/26_cam_line_follow_drive.py` | Closed-loop line-follow drive with auto ground-view calibration (Gate B) | `printf "yes\n" \| PYTHONPATH=src python3 examples/ai_camera/26_cam_line_follow_drive.py --duration 8 --speed 150` | ⚠️ Operator beside it |
| 27 | `examples/ai_camera/27_cam_ground_view_calibrate.py` | Bird's-eye (ground-view) homography calibration from a measured rectangle or flat ChArUco | `PYTHONPATH=src python3 examples/ai_camera/27_cam_ground_view_calibrate.py --auto --size-m 0.10,0.05` | ✅ No moving parts |
| 29 | `examples/ai_camera/29_cam_route_nav_drive.py` | Task-1 route drive: vision-driven nav state machine, route plan advisory only | `PYTHONPATH=src python3 examples/ai_camera/29_cam_route_nav_drive.py --dry-run --duration 10` | ⚠️ Operator beside it (`--dry-run` is safe) |
| 30 | `examples/other/30_cam_motion_calibrate.py` | Time-based motion model calibration (no encoders): forward speed + spin rate | `PYTHONPATH=src python3 examples/other/30_cam_motion_calibrate.py --mode forward --seconds 1.0` | ⚠️ Operator beside it (car drives/spins) |
| 31 | `examples/ai_camera/31_cam_ground_tag_pose.py` | Ground AprilTag pose check: camera (x, y, heading) in the map frame from flat tags (Phase 0 of landmark localization) | `PYTHONPATH=src python3 examples/ai_camera/31_cam_ground_tag_pose.py --tag-map scratch/landmarks/task1-tag-map.json` | ✅ No moving parts |
| 32 | `examples/ai_camera/32_cam_tag_nav_drive.py` | Task-1 route drive with AprilTag-supervised black-line navigation | `PYTHONPATH=src python3 examples/ai_camera/32_cam_tag_nav_drive.py --dry-run --duration 10` | ⚠️ Operator beside it (`--dry-run` is safe) |
| 33 | `examples/ai_camera/33_cam_tag_self_calibrate.py` | Self-calibrates camera intrinsics from the Task-1 map's AprilTags (the 2026-08-14 factory intrinsics are unusable for tag localization) | `PYTHONPATH=src python3 examples/ai_camera/33_cam_tag_self_calibrate.py` | ✅ No moving parts |
| 34 | `examples/ai_camera/34_cam_tag_view_collect.py` | Captures AprilTag corners for camera self-calibration while the operator rotates the car 360° by hand | `PYTHONPATH=src python3 examples/ai_camera/34_cam_tag_view_collect.py` | ✅ No moving parts (operator turns the car by hand) |
| 35 | `examples/ai_camera/35_cam_object_id_check.py` | Times and grades the IMX500 on-sensor object detector for a one-shot "what's on the table" check | `PYTHONPATH=src python3 examples/ai_camera/35_cam_object_id_check.py` | ✅ No moving parts |
| 36 | `examples/other/36_ir_tracing_check.py` | Yahboom 4-channel IR tracing sensor check, uniform 1=black / 0=white readout | `PYTHONPATH=src python3 examples/other/36_ir_tracing_check.py` | ✅ No moving parts (safe over SSH) |
| 37 | `examples/other/37_map1_motor_test.py` | Map1 low-speed motor verification with wheels lifted | `PYTHONPATH=src python3 examples/other/37_map1_motor_test.py --dry-run` | ⚠️ Wheels must be lifted or chassis secured, operator beside it |
| 38 | `examples/ai_camera/38_map1_cam_line_follow.py` | Map1 circular-track drive by downward camera only (no IR sensor) — IR-sensor version is `39` | `PYTHONPATH=src python3 examples/ai_camera/38_map1_cam_line_follow.py --dry-run --duration 10` | ⚠️ Operator beside it (`--dry-run` is safe) |
| 39 | `examples/other/39_map1_ir_line_follow.py` | Map1 circular-track drive by 4-channel IR sensor only (no camera), plus scripted junction turns; `--laps N` stops at the T junction closing lap N | `PYTHONPATH=src python3 examples/other/39_map1_ir_line_follow.py --dry-run --duration 30` | ⚠️ Operator beside it (`--dry-run` is safe) |
| 41.1 | `examples/other/41.1_motor_spin_duration_check.py` | Finds the real spin duration for one target turn angle, on the real track surface | `PYTHONPATH=src python3 examples/other/41.1_motor_spin_duration_check.py --speed 150 --duration-s 2.24 --direction right` | ⚠️ Operator beside it |
| 41.2 | `examples/other/41.2_motor_spin_angle_sweep.py` | Sweeps spin duration vs. actual turned angle (operator reports each angle by eye) to (re)fit `spin_rate_deg_per_s`/`spin_dead_time_s` | `PYTHONPATH=src python3 examples/other/41.2_motor_spin_angle_sweep.py --speed 150` | ⚠️ Operator beside it, reads and types each angle |
| 42 | `examples/other/42_ir_geometry_sweep.py` | Measures the IR bar's physical sensor layout by sweeping a black strip across it | `PYTHONPATH=src python3 examples/other/42_ir_geometry_sweep.py` | ✅ No moving parts (safe over SSH) |

Expected results (verified on this build, 2026-08):

- `01` prints `✓ Probe finished successfully`.
- `02` runs each motor forward then reverse for 1 s; all four wheels move, matching
  `src/carbot/config.py` (`M1`=rear-left, `M2`=rear-right, `M3`=front-right, `M4`=front-left,
  `INVERTED_MOTORS={2, 3}`).
- `03` performs a short low-speed differential-drive run.
- `04` sweeps each arm servo through a small range with the operator watching.
- `05` prints `All checks passed` and (with `--photo`) writes `/tmp/ai-camera-check.jpg`.
- `06` prints an average distance in cm and warns when it drops below the obstacle threshold.
- `07` prints one `clear`/`OBSTACLE` decision per loop; `--dry-run` never drives motors.
- `08` prints `✓ Power health OK.` when `EXT5V_V >= 4.8 V` with no **live** throttle bits.
  In `get_throttled` the low nibble (`0x1`/`0x2`/`0x4`/`0x8`) is the live state and bits
  16-19 (`0x10000`+) are sticky since-boot history. Only a live bit means fix the power
  supply before motor tests; a since-boot bit prints as `[INFO]` because it stays set
  until reboot.
- `21` reports `Use mode 'single' in the fused patrol.` — the IMX500 delivers inference
  frames and 2028×1520 stills from one configuration, so no mode switching is needed.
  Its exposure sweep ranks settings by *repeatable* keypoints (matched across two captures
  of the same scene), not raw counts, because analogue gain manufactures keypoints out of
  sensor noise: `ev+1.5` reports 21% more raw keypoints than the default at gain 15.5 while
  matching fewer of them. `long-shutter+spot` wins at gain 4.0, but its 97 ms shutter still
  has to be proven sharp on a car that has only just stopped.
- `22 --dry-run` reads the sonar and the detector and prints one fused `clear`/`BLOCKED`
  decision per step without sending any motor command. Raise `--obstacle-cm` above the
  measured distance to exercise the sonar branch, or lower `--threshold` to exercise the
  vision branch, before any supervised run.
- `12` detects AprilTag ID 0 using its measured 70 mm black square and writes annotated and
  undistorted images under `/tmp`.
- `13` requires the measured, fixed wall targets documented in
  [`docs/progress/2026-08-14-vision-room-anchor.md`](progress/2026-08-14-vision-room-anchor.md),
  then writes `/tmp/room-pose.json` and `/tmp/room-pose.jpg`.

## Working From a Mac

The code runs on the Raspberry Pi, not on your laptop — only the Pi is wired to the NeZha board
over I2C. [docs/setup/mac-to-raspberry-pi-access.md](setup/mac-to-raspberry-pi-access.md)
covers how to reach it:

| Method | Use it for |
|---|---|
| **SSH** | Terminal work — `git`, `uv`, running the examples. The everyday default. |
| **[Raspberry Pi Connect](https://connect.raspberrypi.com)** | Reaching the Pi from outside your home network, through a browser. Free for personal use. |
| **VNC** | The graphical desktop over the local network. |
| **[Deskflow](setup/deskflow-macos-raspberrypi.md)** | Sharing one keyboard and mouse across a Mac and a Pi on the same desk. |

### SSH Quick Reference (this build)

| Parameter | Value |
|---|---|
| Username | `dannypi` |
| mDNS hostname | `danny-raspberrypi5-8gram-225gssd.local` |
| Current IP | `192.168.1.27` — DHCP assigned, can change; prefer the hostname |

Set up passwordless login once — run these **on the Mac** (enter the Pi password when prompted):

```bash
ssh-copy-id dannypi@danny-raspberrypi5-8gram-225gssd.local

cat >> ~/.ssh/config <<'EOF'

Host carpi
    HostName danny-raspberrypi5-8gram-225gssd.local
    User dannypi
EOF
```

After that, `ssh carpi` connects directly:

```bash
ssh carpi 'whoami && hostname -I'
```

Power health checks — run on the Pi (e.g. via `ssh carpi`):

```bash
vcgencmd get_throttled                # 0x0 = healthy; low nibble = throttling now, bits 16-19 = since boot
sudo vcgencmd pmic_read_adc EXT5V_V   # expect >= 4.8V when fed by the battery pack
```

Read the safety section before running anything remotely: the motor and servo scripts assume an
operator standing within reach of the main power switch.

## Repository Layout

| Path | Purpose |
|---|---|
| [CONVENTIONS.md](../CONVENTIONS.md) | File placement and naming rules for this repository |
| [docs/hardware/](hardware/) | NeZha protocol notes, Raspberry Pi pinout, integration notes |
| [docs/setup/](setup/) | Setup and bring-up guides |
| [docs/adr/](adr/) | Architecture decision records — site architecture, SfM mapping route, landmark localization, the NeZha driver port |
| [src/carbot/](../src/carbot/) | Python driver and control code |
| [examples/](../examples/) | Runnable hardware verification scripts |
| [site/](../site/) | Astro source for the project website |
| [assets/](../assets/) | Photos, diagrams, and other project assets |
| [vendor/](../vendor/) | Reserved for third-party material, read-only when used; empty as of 2026-08-22 (see [CONVENTIONS.md §4](../CONVENTIONS.md#4-vendor-import-rules)) |

## Website

```bash
npm install
npm run dev
```

Local preview:
<http://127.0.0.1:18427/IRSENSORCAR/>

The website uses English-only routes. Architecture notes are recorded in
[ADR 0001](adr/0001-static-site-architecture.md).

## Safety Notes

- The NeZha board must be powered from **12V**. Reversed polarity can destroy the board.
- If the board's `5V` rail powers the Raspberry Pi through `Pin 2` or `Pin 4`, do **not**
  connect USB-C power to the Raspberry Pi at the same time.
- Keep the I2C clock at or below **200kHz**. Raspberry Pi defaults to 100kHz, which is correct.
- Lift the car so all wheels are off the ground before the first motor test.

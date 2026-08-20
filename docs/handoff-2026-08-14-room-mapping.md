# Handoff — Room-Mapping Prototype (2026-08-14)

> **Later same-day update:** camera calibration and a fixed-wall AprilTag + ChArUco room anchor
> are now verified. Read
> [`docs/progress/2026-08-14-vision-room-anchor.md`](progress/2026-08-14-vision-room-anchor.md)
> before continuing.
>
> **Gate A status (2026-08-14):** the spin timing and angle-conversion defects below are **fixed**
> (commits `3359cd6`..`efb5347`); see
> [`docs/progress/2026-08-14-gate-a-software-static-camera.md`](progress/2026-08-14-gate-a-software-static-camera.md)
> before running `examples/10_sonar_motion_calibrate.py` / `examples/11_sonar_explore_mapping.py`. Both
> scripts now confirm with the operator before constructing `Car()`; run
> `examples/14_all_sensors_preflight_check.py` before any motion test.

This document is a handoff for the next engineer continuing this project. It summarises what is
built, what is verified, what failed, and where the open decisions are. Read
[`docs/progress/2026-08-14-sensors-and-ai-camera.md`](progress/2026-08-14-sensors-and-ai-camera.md)
for the day-by-day evidence behind these conclusions.

---

## 1. Project State

- A Raspberry Pi 5 smart car (`Yourfun NeZha` driver board over I2C at `0x40`) that was originally
  a line-follow/obstacle-avoidance demo car with a robotic arm.
- **Direction change (2026-08-14):** the arm servos were removed (one servo motor failed). The
  project is now an **interior-mapping robot for interior designers**: the user places the robot
  in a room, it maps the space and turns around obstacles on its own, and the deliverable is a
  floor map **with dimension annotations** (wall lengths).
- Two parallel technical routes were planned: (1) ultrasonic spin-scan mapping stitched by ICP,
  and (2) photo-based 3D reconstruction with COLMAP on a Mac. Route 1 was prototyped and its
  limits were measured (see §5). Route 2 (COLMAP) is **not started yet**.

## 2. Hardware

| Item | Model / Detail |
|---|---|
| Main controller | Raspberry Pi 5 (aarch64, kernel 6.18.34+rpt-rpi-2712) |
| Driver board | Yourfun NeZha bus driver board, I2C address `0x40` (bus 1) |
| Chassis | Dasheng multi-form robot car, 4x N20 motors (**no encoders**) |
| Obstacle sensor | HC-SR04 ultrasonic — **VCC=Pin 2, GND=Pin 9, TRIG=Pin 11 (GPIO 17), ECHO=Pin 13 (GPIO 27)** via a 2.2k/1k voltage divider |
| Vision | Raspberry Pi AI Camera (IMX500), 4056x3040 stills, on-sensor NPU inference |
| Battery | HXS 18650 11.1V 1200mAh (**small — see pitfalls**) |
| Robotic arm | **Removed** (one servo dead) |

**Wiring constraint:** the NeZha board occupies **Pin 3 (SDA), Pin 4 (5V), Pin 5 (SCL), Pin 6 (GND)**.
Any new sensor must avoid those pins. See `docs/hardware/hc-sr04-ultrasonic-sensor.md` and
`docs/hardware/nezha-integration-notes.md`.

## 3. Environment

- SSH alias on the Mac: `ssh carpi` → `dannypi@danny-raspberrypi5-8gram-225gssd.local`
  (current IP 192.168.1.27, DHCP). Repo lives at `~/Car-and-Robotic-Arm` on the Pi.
- The Pi's `.venv` has no `pip` and no numpy; **run hardware scripts with the system
  interpreter**: `PYTHONPATH=src python3 examples/...` (system python3 has numpy 2.2.4,
  RPi.GPIO 0.7.2, picamera2). `uv run` works only inside the venv for pure-Python code.
- `sudo` on the Pi **requires a password** — remote `sudo -n` fails; apt installs must be run by
  a human (`ssh -t carpi 'sudo ...'`).
- Mac-side prototype tooling (temp): `/tmp/mapvenv` (numpy/matplotlib/scipy),
  `/tmp/venv39` (open3d 0.18 — **segfaults on macOS, avoid**).

## 4. Code Architecture

```
src/carbot/
  config.py    WHEEL_TO_MOTOR, INVERTED_MOTORS={2,3}, HAS_ENCODERS=False,
               ARM_JOINT_*, SAFE_TEST_SPEED=200
  nezha.py     NeZha driver (I2C 0x40): motor/servo/LED commands
  car.py       Car: forward/backward/turn/spin/move_for (uses config)
  mapping.py   Room-mapping core (pure numpy, no hardware):
               load_polar_scan, polar_to_points, icp, register, best_rigid,
               detect_gaps (door/wall gaps), OccupancyGrid (10 cm, inverse
               sensor model), map_scans
tests/         pytest suite (60 passed incl. 7 mapping tests)
```

**examples/** (all verified unless noted):

| # | Script | Safety | Notes |
|---|---|---|---|
| 01 | `01_i2c_probe.py` | safe | NeZha I2C link, reset, LED |
| 02 | `02_motor_check.py` | ⚠️ lifted | M1-M4 mapping verified, matches config |
| 03 | `03_motor_drive.py` | ⚠️ | ground run, **not re-run this session** |
| 04 | `04_servo_check.py` | ⚠️ | arm servos — **arm removed, skip** |
| 05 | `05_ai_camera_check.py` | safe | IMX500 detect/photo/`--inference` |
| 06 | `06_ultrasonic_avoidance.py` | safe | HC-SR04 distance + obstacle warning |
| 07 | `07_sonar_avoidance_drive.py` | ⚠️ | closed-loop avoidance (dry-run/ground) |
| 08 | `08_battery_check.py` | safe | EXT5V_V, get_throttled, temp |
| 09 | `09_sonar_room_scan.py` | ⚠️ | M1 spin-scan → polar CSV |
| 10 | `10_sonar_motion_calibrate.py` | ⚠️ floor | forward-speed calibration (unstable) |
| 11 | `11_sonar_explore_mapping.py` | ⚠️ | M3 loop: scan→ICP→grid→step |

## 5. Verified Facts & Findings (read before continuing)

### 5.1 Hardware / sensors
- I2C at `0x40` responds; reset + head-LED command path work. Encoders read 0 (two-wire motors).
- HC-SR04 on Pin 2/9/11/13 works; 6/6 readings stable; spin rate ≈ **8.2 s/360° at speed 150**.
- IMX500: detection, 4056x3040 stills, and **on-sensor mobilenet-ssd inference at 30 fps** all
  verified. Postprocess package: `rpicam-apps-imx500-postprocess` (json configs in
  `/usr/share/rpi-camera-assets/`, lib in `/usr/lib/aarch64-linux-gnu/rpicam-apps-postproc/`;
  models in `/usr/share/imx500-models/`).
- **Battery: `get_throttled = 0x50000` = undervoltage now + throttling now** (persistent).
  EXT5V_V ~4.89 V is within range but the 1200 mAh pack is marginal under motor load.

### 5.2 Mapping prototype (quantified)
- **M1:** a single ultrasonic sensor spinning in place captures a room's polar distance profile.
- **M2:** pure multi-angle ICP for *non-adjacent* scans of a rectangular room is severely
  multi-modal — 585 initial guesses → 351 solutions, largest basin 1%. High inlier counts do
  NOT mean correct alignment (verified wrong: 3 positions collapsed to within 4 cm).
- **Incremental ICP** (identity initial guess, <15 cm step) converges to **~0.06 cm error** —
  the basis for the M3 small-step design.
- **Door/gap detection works** (`detect_gaps`): corridor gap visible at 102–115° in the doorway
  scan; but the bathroom interior has few gaps (only the 2.1 m side), which limits gap anchoring.
- **M3 live loop (real car, bathroom):**
  - v1 (0.8 s steps, identity guess): car barely moved; position never accumulated.
  - v2 (2.5 s steps, odometry initial guess): accumulated ~23 cm then ICP collapsed.
  - v3 (+ gap-anchor heading + crash detection): accumulated ~50 cm then x-drift; gap anchor
    never engaged because gaps were not detected until step 8.
- **Verdict: without wheel encoders / an IMU, incremental ICP mapping is reliable to roughly
  50 cm of travel in a small rectangular room.** Open-loop odometry is order-of-magnitude only
  (measured forward speed 5–17 cm/s at speed 200, unstable).

## 6. Known Pitfalls

1. `sudo` needs a password over SSH — no remote root ops.
2. NeZha occupies Pin 3/4/5/6 — do not reuse for new sensors.
3. `.venv` lacks pip/numpy — use `PYTHONPATH=src python3` (system interpreter).
4. IMX500 model/postprocess paths differ from the obvious ones (see §5.1).
5. First IMX500 inference run uploads the network firmware (minutes) — don't time out at 5 s.
6. `open3d` 0.18 segfaults on this macOS/Python setup — self-written numpy ICP is used instead.
7. HC-SR04 near-range blind zone (<~20 cm) corrupts distance readings and speed calibration.
8. Battery undervoltage is persistent — recharge/replace before demanding motor runs.

## 7. Open Decisions (need product/engineering input)

1. **Localization approach for reliable mapping** (the core blocker):
   - **A. Hardware**: add encoders (or encoder motors) + optionally an IMU → closed-loop odometry
     feeds ICP initial guesses. Highest reliability; ~1-2 days of modification.
   - **B. Software-only**: accept ~50 cm reliable range; useless for real rooms. Not recommended.
   - **C. Photo route**: skip odometry entirely — robot patrols and photographs; COLMAP (SfM)
     computes camera trajectories + 3D on the Mac. White-wall (feature-poor) rooms are a risk.
2. **Deliverable format**: 2D floor map with dimension annotations was chosen. Wall-length
   accuracy depends on localization accuracy (§7.1).
3. **End-of-mapping criteria** (agreed design, not yet implemented): wall-coverage ≥ 90%,
   convergence (new cells < 0.1%/3 min), battery < 25% return-to-start, manual stop.

## 8. Recommended Next Steps

1. **Decide §7.1.** The evidence strongly favours A (encoders/IMU) for a real product; C
   (COLMAP) is the fastest way to a demo without touching hardware.
2. If A: order encoder motors or magnetic encoders for the N20 gearmotors, wire the encoder
   inputs on the NeZha board (`HAS_ENCODERS=True` in `config.py`), then re-run
   `examples/10_sonar_motion_calibrate.py` and `examples/11_sonar_explore_mapping.py`.
3. If C: write a photo-patrol example (drive + stop + capture with picamera2), transfer images to
   the Mac, `brew install colmap`, run SfM. Verify on the bathroom (tiles/toilet/sink give
   features) and on a white-wall room (expected failure mode).
4. Implement the end-of-mapping criteria (§7.3) on top of whichever localization lands.
5. Recharge/replace the battery before further motor testing.

## 9. Key Files

- `docs/progress/2026-08-14-sensors-and-ai-camera.md` — full evidence log (incl. addenda)
- `docs/progress/2026-07-30-first-drive.md` — earlier motor/drive verification
- `docs/hardware/hc-sr04-ultrasonic-sensor.md` — sensor wiring incl. NeZha-conflict alternative
- `docs/hardware/nezha-integration-notes.md` — driver board wiring, I2C
- `src/carbot/mapping.py` — mapping core (ICP, grid, gaps)
- `scratch/mapping/` — prototype map images (not committed)
- Git history from `2abac93` — every step of this session is one commit.

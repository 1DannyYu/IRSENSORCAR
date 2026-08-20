# 2026-08-14 Sensor & AI Camera Verification

## Result

The HC-SR04 obstacle sensor was wired around the NeZha I2C pin conflict and verified over SSH;
the AI Camera (IMX500) was verified end-to-end including **on-sensor NPU inference (30 fps)**.
Three new test scripts were added (`examples/06/07/08`), the rule about running motor tests over
SSH was relaxed to a conditional form, and the README now documents all eight test scripts.

## HC-SR04 Wiring Correction (NeZha I2C Pin Conflict)

The stock wiring table used `Pin 6` for GND, but this build's NeZha driver board already occupies
`Pin 3 (SDA)`, `Pin 4 (5V)`, `Pin 5 (SCL)`, and `Pin 6 (GND)` for I2C + power.

Verified alternative wiring (documented in `docs/hardware/hc-sr04-ultrasonic-sensor.md`):

| HC-SR04 | Pi Pin | BCM GPIO |
|---|---|---|
| VCC | Pin 2 | 5V |
| GND | Pin 9 | Ground |
| TRIG | Pin 11 | GPIO 17 |
| ECHO | Pin 13 | GPIO 27 (via 2.2k/1k divider) |

GND, TRIG, ECHO sit together on the left column (9, 11, 13); only 5V is pulled separately.

## SSH Verification (Mac -> Pi)

Connection: `ssh carpi` → `dannypi@danny-raspberrypi5-8gram-225gssd.local` (192.168.1.27, Pi 5,
kernel 6.18.34 aarch64).

| Check | Command | Result |
|---|---|---|
| HC-SR04 distance | `python3 /tmp/hc_sr04_test.py` | 6/6 readings, stable 255.6 cm |
| NeZha I2C link | `.venv/bin/python examples/01_i2c_probe.py` | `0x40` responded, reset OK, LED OK |
| Motor mapping | `.venv/bin/python examples/02_motor_check.py` | M1-M4 all moved, matches `config.py` |
| AI Camera | `python3 examples/05_ai_camera_check.py --photo` | IMX500 detected, 4056x3040 still captured |
| AI Camera inference | `python3 examples/05_ai_camera_check.py --inference` | mobilenet-ssd, 30 fps for 120 s |
| Obstacle detector | `python3 examples/06_ultrasonic_avoidance.py --trials 8 --threshold 50` | 8/8, avg 31.3 cm, warning triggered |
| Avoidance loop (dry-run) | `PYTHONPATH=src python3 examples/07_sonar_avoidance_drive.py --dry-run` | 33 loops, sensor logic OK |
| Avoidance loop (ground) | `PYTHONPATH=src python3 examples/07_sonar_avoidance_drive.py --ground --duration 30 --threshold 40` | 109 loops, stop+spin on obstacle |
| Battery health | `python3 examples/08_battery_check.py` | EXT5V_V 4.89 V, **throttle warning, see below** |

## New Test Scripts (committed)

- `examples/06_ultrasonic_avoidance.py` — HC-SR04 distance + obstacle warning (`--trials`,
  `--threshold`). Runs with the system interpreter; no motors involved.
- `examples/07_sonar_avoidance_drive.py` — closed-loop avoidance: distance > threshold → forward,
  distance <= threshold → stop + spin. `--dry-run` never drives motors; `--ground` for a floor run.
  Needs `PYTHONPATH=src python3` (system interpreter has RPi.GPIO; the uv venv does not).
- `examples/08_battery_check.py` — reads `EXT5V_V`, `get_throttled` bits, and temperature; all
  readable without sudo.

## Rule Change (AGENTS.md / CLAUDE.md)

Rule 3 changed from "do not run motor-moving programs on behalf of the user" to: motor tests may
run over SSH when **an operator is physically beside the robot and the wheels are lifted or the
chassis secured**. This matches the existing safety section in `docs/setup/mac-to-raspberry-pi-access.md`.

## Pitfalls Hit Today

1. **`sudo` needs a password over SSH** — `sudo -n` fails; `i2cdetect` and `apt install` had to be
   run by the user on the Pi (or with `ssh -t`).
2. **NeZha occupies Pin 3/4/5/6** — any new sensor wiring must avoid them; HC-SR04 GND moved to Pin 9.
3. **The uv venv has no `RPi.GPIO`/`gpiozero`** — sensor scripts use the system `python3`; to use the
   `carbot` package too, run with `PYTHONPATH=src python3` (src-layout).
4. **IMX500 model path is `/usr/share/imx500-models`** — not `/usr/share/rpicam-apps/imx500` (that
   directory does not exist). 23 `.rpk` models are installed.
5. **IMX500 postprocess package paths** — `rpicam-apps-imx500-postprocess` installs json configs to
   `/usr/share/rpi-camera-assets/` and the lib to
   `/usr/lib/aarch64-linux-gnu/rpicam-apps-postproc/imx500-postproc.so`.
6. **First inference run uploads the network firmware (~3.8 MB)** — takes minutes; a 5 s timeout
   aborts it with exit 255. The check now allows 120 s and warns about the upload.
7. **Wrong default model choice** — `face_detect_cv.json` was picked by name; the check now prefers
   `mobilenet`/`ssd` object-detection configs.
8. **Battery:** `get_throttled = 0x50000` = **undervoltage now + throttling now** (EXT5V_V 4.89 V is
   within range). The 1200 mAh battery is marginal for motor loads — recharge/replace before
   demanding runs.

## Git

- `2abac93` Add HC-SR04 avoidance scripts and document verified hardware tests (8 files)
- `55f4406` fix(05): locate IMX500 postprocess config in rpi-camera-assets; verify on-sensor inference
- Both pushed to `origin/main`.

## Next Steps

- `examples/03_motor_drive.py` ground run (operator beside the car)
- `examples/04_servo_check.py` arm servos (operator beside the car)
- Recharge/replace the battery and re-run `examples/08_battery_check.py` until `✓ Power health OK`
- Optionally wire IMX500 detections into the avoidance loop (vision + ultrasonic fusion)

## Addendum: Room-Mapping Prototype (M1/M2) — Key Findings

Direction change: after the arm servos were removed (one motor failed), the project pivoted to
**vision + ultrasonic fusion for interior mapping** (target customer: interior designers; the
robot is placed in a room and maps it, turning around obstacles on its own).

### M1 — Spin Scan (verified)

- Script: robot spins in place while the HC-SR04 logs distance vs. time (polar profile).
- Verified: one full spin ≈ **8.2 s** at speed 150 (repeated far peaks 8.0–8.2 s apart).
- A 20 s scan yields ~93 readings; range up to 809 cm was observed down a corridor
  (beyond the HC-SR04 400 cm spec — treat as "far/open").
- Bathroom middle scan: 2.1 m long side + a 17 cm near obstacle (fixture).
- Conclusion: a single-point ultrasonic sensor spinning in place CAN capture a room's shape.

### M2 — Incremental Mapping (scan matching + occupancy grid)

- Implemented self-contained 2D point-to-point ICP (numpy) + 10 cm occupancy grid
  (Mac-side prototype, scripts in `scratch/` after testing).
- Stitched the three scans (bath / doorway-corridor / original position) with 93–97%
  reported inliers, but **data-level verification exposed the result was WRONG**:
  the three robot positions collapsed to within 4 cm of each other (real separation
  should be metres).

### Root Cause (verified quantitatively)

- Pure multi-angle ICP on the bathroom scans is **severely multi-modal**: 585 initial
  guesses produced 351 distinct solutions; the largest convergence basin captured only
  1% of guesses. Rectangular-room symmetry + little overlap between non-adjacent scans
  make ICP ambiguous without an initial pose.
- Door/wall-gap feature detection DOES work: the doorway-corridor scan showed a clear
  gap (365–400 cm) at 102–115°; bathroom gaps at 32–53°; the original position had a
  19°-wide gap at 197°.

### Chosen Direction (software route)

- M3 design: **incremental movement** (<50 cm / <30° steps) so ICP works in its
  reliable local-convergence regime, door/gap features as global anchors, and a
  loop-closure check (return to start should re-match the first frame).
- Odometry is still open-loop (no encoders); drift will accumulate — the prototype's
  goal is to measure where that becomes unacceptable.
- Remaining calibration (needs the car, operator beside it): forward speed in cm/s at
  a given speed setting; spin rate already known (360°/8.2 s @ 150).

## Addendum 2: M3 Autonomous-Exploration Prototype — Verified Limits

Motion calibration (`examples/10_sonar_motion_calibrate.py`): ultrasonic-based forward-speed
calibration is unstable (5.4 / 16.7 / -1.1 cm/s across reps) due to the HC-SR04 near-range
blind zone and Mecanum side-slip. Conclusion: open-loop odometry is only good to an order of
magnitude; use ~8 cm/s @ speed 200 as a rough value.

M3 loop (`examples/11_sonar_explore_mapping.py`): spin scan -> ICP -> occupancy grid -> small step.

| Version | Change | Reliable accumulation |
|---|---|---|
| v1 | 0.8 s steps, no-move ICP guess | ~4-8 cm/step, position never accumulated (car barely moved) |
| v2 | 2.5 s steps, odometry initial guess | ~23 cm, then ICP collapsed to origin on step 5 |
| v3 | gap-anchor heading + crash detection | ~50 cm, then x-drift; gap anchor never engaged (bathroom gaps >100 cm, not detected until step 8) |

**Prototype verdict (quantified):** without wheel encoders / IMU, incremental ICP mapping in a
small rectangular room is reliable to roughly **50 cm of travel**, then drift accumulates and
scan matching can lock onto a wrong (high-inlier) solution. The bathroom yields few gap features
(only the 2.1 m side), so door-anchoring cannot rescue heading drift there. Decision point for
the next phase: encoder/IMU localization is required for reliable room-scale mapping; the
software pipeline (ICP, occupancy grid, gap features, crash detection) is ready to consume it.

# Handoff — Fixed Vision Anchor to Room Mapping (2026-08-14)

## Purpose

This report is for the next AI/engineer taking over development. The immediate objective is to
integrate the verified IMX500 + ChArUco + AprilTag room anchor into the 2D mapping pipeline without
repeating today's print-scale, planar-pose, duplicate-anchor, coordinate-frame, and unsafe motion
mistakes.

Do not begin by running the current autonomous exploration script. The vision anchor is ready;
the motion-to-map integration is not.

## Read First

Read these files before editing or running hardware:

1. [AGENTS.md](../AGENTS.md)
2. [CONVENTIONS.md](../CONVENTIONS.md)
3. [Vision room-anchor progress](progress/2026-08-14-vision-room-anchor.md)
4. [Earlier room-mapping handoff](handoff-2026-08-14-room-mapping.md)
5. [NeZha I2C protocol](hardware/nezha-i2c-protocol.md)
6. [Raspberry Pi first-run procedure](setup/raspberry-pi-first-run.md)

`vendor/` is read-only. Motor-moving programs may run only while an operator is physically beside
the robot and can cut power immediately.

## Source-Control State

### Mac / GitHub

- Repository: `git@github.com:1DannyYu/IRSENSORCAR.git`
- Branch: `main`
- Vision implementation base: `cee7f1f72e5495b698e9f6e8ed8e4e7c2fe68353`
- Base commit subject: `feat(vision): add fixed-wall room pose anchor`
- Continue from the current `origin/main`, which also contains this handoff and supporting docs.

At this handoff, the reviewed documentation is committed and generated cache/build directories
have been removed. The 17 full-resolution calibration photos remain local under
`scratch/camera-calibration/2026-08-14-imx500-4056x3040/source-frames/`. They contain the room
interior and are deliberately ignored; do not stage or upload them.

### Raspberry Pi

- SSH alias from the Mac: `carpi`
- Pi repository: `~/Car-and-Robotic-Arm`
- Pi branch at handoff: `main`
- Pi Git `HEAD` at handoff: `0bbcd41a3bbbe183bd2b761e6d2465e403f9538e`

The Pi is behind GitHub because files were copied directly for hardware verification. It has
untracked copies of examples 10-13, `src/carbot/vision.py`, and calibration assets. A plain
`git pull` may refuse to overwrite those files.

Use a recoverable update procedure; do not delete them blindly:

```bash
ssh carpi
cd ~/Car-and-Robotic-Arm
git status --short
git stash push --include-untracked -m "pre-cee7f1f-pi-runtime-copies"
git pull --ff-only origin main
git rev-parse HEAD
```

After pulling, `git rev-parse HEAD` must match `git rev-parse origin/main` on the Pi.

Do not apply the stash immediately; it contains old runtime copies that now overlap tracked files.
Keep it as a recovery point until the synced Pi passes static checks. Preserve the latest runtime
evidence first if needed:

```bash
scp carpi:/tmp/room-pose-wall.json /tmp/room-pose-wall.json
scp carpi:/tmp/room-pose-wall.jpg /tmp/room-pose-wall.jpg
```

## Verified Hardware and Environment

| Item | Verified state |
|---|---|
| Raspberry Pi | Pi 5; repo accessed through `ssh carpi` |
| Motor board | Yourfun NeZha at I2C `0x40` |
| Motors | Forward, reverse, left arc, right arc, stop verified by operator |
| Encoders | None; two-wire motors; `HAS_ENCODERS=False` |
| Ultrasonic | HC-SR04 on BCM 17/27 with divider; obstacle stop/turn verified |
| Camera | Raspberry Pi AI Camera IMX500, physically fixed |
| Camera still mode | 4056 x 3040 |
| Pi Python | Use `PYTHONPATH=src python3` for Picamera2/RPi.GPIO/OpenCV |
| Mac Python | Use `uv`; vision commands need `--extra vision` |
| Power history | `get_throttled=0x50000` was observed earlier; recheck before motion |

The Pi system Python already has OpenCV 4.10 with `aruco` and Picamera2. Do not install a second
OpenCV into the Pi project environment unless there is a demonstrated need.

## Permanent Wall Anchor

The accepted anchor is the single pair now fixed to the vertical wall. Old duplicate prints were
removed from camera view.

| Property | Measured value |
|---|---:|
| AprilTag | tag36h11 ID 0 |
| AprilTag outer black square | 70 x 70 mm |
| ID 0 center above floor | 14.65 cm |
| ChArUco layout | 5 x 7 squares; DICT_5X5_100 |
| ChArUco pattern | 140 x 195 mm |
| ChArUco pitch X | 28.000 mm |
| ChArUco pitch Y | 27.857 mm |

Both targets are flat, upright, on the same wall, and must not move. If either print is replaced,
moved, rotated, curled, or rescaled, remeasure it and rerun static validation.

The right-handed room frame is:

- `X`: away from the anchor wall into the room;
- `Y`: along the wall to the right;
- `Z`: upward from the floor;
- origin: floor point directly below ID 0 center.

## Final Static Result

Command:

```bash
cd ~/Car-and-Robotic-Arm
PYTHONPATH=src python3 examples/13_cam_room_pose.py --anchor-height-cm 14.65
```

Accepted five-frame result:

```text
valid/inliers = 5/5
wall distance = 61.33 cm
wall right    = -0.87 cm
camera height = 11.05 cm
heading       = 179.82 deg
elevation     = 16.86 deg

position range across inliers:
X = 0.08 cm
Y = 0.09 cm
Z = 0.13 cm
```

All five frames detected 24/24 ChArUco corners. This stage is camera-only and does not access the
motors.

Important interpretation: with `+X` pointing away from the wall, a camera heading near 180
degrees means the camera optical axis is facing toward the anchor wall. Do not assume the chassis
is safe to drive forward from that pose until the camera-to-chassis alignment is verified.

## Implemented Vision Code

### `src/carbot/vision.py`

- calibration JSON validation and same-aspect-ratio intrinsic scaling;
- image undistortion;
- AprilTag 36h11 detection and metric square pose;
- measured, anisotropic ChArUco object-point geometry;
- hybrid pose: ChArUco supplies wall orientation, ID 0 supplies world translation;
- right-handed world transforms with reflection rejection;
- multi-frame position/rotation aggregation and outlier rejection;
- diagnostic image annotation.

### `examples/12_cam_apriltag_pose.py`

Static one-frame diagnostic. It is useful for tag detection and distance, but its single-square
pitch/roll must not be treated as a stable room orientation when the tag is nearly front-on.

### `examples/13_cam_room_pose.py`

Static five-frame hybrid estimator. It skips missing/reprojection-rejected frames, rejects pose
outliers, and writes:

```text
/tmp/room-pose.json
/tmp/room-pose.jpg
```

### Verification

At commit `cee7f1f`:

```text
targeted Ruff: passed
full pytest with vision extra: 70 passed
Pi fixed-wall smoke: 5/5 inliers
```

## Critical Gaps Before Mapping

### 1. Camera pose is not yet ultrasonic-sensor pose

`examples/13_cam_room_pose.py` gives the camera optical-center pose. `OccupancyGrid.update()` expects
the 2D pose of the ultrasonic sensor frame. The rigid offsets between these frames are not yet
measured:

- camera optical center relative to chassis rotation center;
- HC-SR04 emitter center relative to chassis rotation center;
- camera yaw relative to chassis forward;
- HC-SR04 yaw relative to chassis forward.

Define and calibrate these transforms before fusing vision pose with range points. For a school
demo, a measured tape/protractor approximation may be acceptable, but record uncertainty and do
not silently assume all origins coincide.

### 2. Mapping and vision axes are not the same contract

The room frame is `(away, wall-right, up)`. `mapping.polar_to_points()` returns:

```python
[distance * sin(angle), distance * cos(angle)]
```

so scan angle zero becomes array value `[0, +distance]`. Existing comments use `x/y`
inconsistently, and script 11 assumes scan angle zero is `+y`. Add explicit frame names and tests
before combining its arrays with room-frame `(X, Y)` values. A swapped axis or sign can produce a
map that looks plausible while being mirrored or rotated 90 degrees.

### 3. The current room pose is a start observation, not perpetual localization

The saved JSON records the camera pose for the car's current physical position. Once the car
moves, that numeric pose is stale. The fixed anchor can correct localization only while the wall
targets are visible.

Options for room-scale correction:

1. repeatedly revisit/face the fixed ID 0 + ChArUco pair for loop closure;
2. place unique tags (IDs 1-4) around the room and measure their world poses;
3. add encoders/IMU;
4. implement visual odometry/SLAM on the Mac.

For the current hardware and assignment deadline, unique measured tags plus periodic ID 0 +
ChArUco loop closure is the smallest practical extension. A single ID should not supply precise
orientation; use multiple known corners/tags or a board when orientation matters.

### 4. Open-loop rotation is currently inconsistent

Do not run `examples/10_sonar_motion_calibrate.py` or `examples/11_sonar_explore_mapping.py` yet:

- script 10 parses `--spin-seconds` but sleeps for a hard-coded 4.0 seconds;
- script 11 drives spin at speed 200 but uses an 8.2-second revolution measured at speed 150;
- script 11's `--spin360` changes duration while angle conversion still uses the global constant;
- neither script uses encoders;
- script 10 creates `Car()` before operator confirmation and does not close it when confirmation
  is refused.

The 8.2-second value must never be transferred across motor speeds without measurement.

### 5. Current mapping tests are too synthetic

`tests/test_mapping.py` covers ideal point clouds, gap detection, CSV conversion, and a simple grid.
It does not replay:

- wrong scan duration/angle scaling;
- real HC-SR04 dropouts and out-of-range returns;
- rectangular-room ICP false lock-in;
- accumulated heading drift;
- duplicate visual anchors;
- stop-on-obstacle behavior;
- pose-frame sign/axis mistakes.

Add captured-data or reduced numerical regression fixtures before another autonomous run. Fixtures
must not contain identifiable room photographs; store numerical scan/pose data only.

## Recommended Architecture

Keep safety-critical control on the Pi and heavier mapping on the MacBook Pro:

```text
Raspberry Pi
  Picamera2 + HC-SR04 + NeZha
  immediate obstacle stop
  timestamped capture bundles
  explicit short motion commands
            |
            | SSH/SCP first; streaming later
            v
MacBook Pro
  calibration/undistortion
  AprilTag/ChArUco localization
  scan transformation + occupancy grid
  ICP/global-anchor correction
  live or near-live map visualization
```

Network loss must never prevent the Pi from stopping. Do not put the ultrasonic emergency stop or
motor timeout exclusively on the Mac.

Begin with stop-and-capture bundles transferred after each step. Real-time streaming is a later
optimization; it is not required to prove the coordinate and mapping pipeline.

Suggested capture-bundle fields:

```text
session ID
frame/step index
monotonic timestamp
wall-clock timestamp
commanded left/right motor speed
command start/stop timestamps
ultrasonic elapsed_s, distance_cm, configured spin speed
camera image path and image size
detected visual-anchor IDs
calibration file/version
operator abort / obstacle-stop reason
```

Never infer scan angle later from a constant that is not stored with that exact capture.

## Development Plan and Gates

### Gate A — Pure software and static camera only

No motors may move in this gate.

1. Sync the Pi safely to the current `origin/main` and rerun examples 12/13.
2. Introduce explicit typed 2D frame/pose conversion functions rather than passing anonymous
   `np.ndarray` values between vision and mapping.
3. Define the camera/chassis/ultrasonic extrinsic configuration and validation rules.
4. Add numerical room-pose fixture(s) without photographs.
5. Add tests for axis order, handedness, heading convention, duplicate ID 0 rejection, unavailable
   anchor, and ChArUco partial detection.
6. Extract scan-angle conversion from script 11 into a pure function and test that its configured
   `spin360` value is used everywhere.
7. Make script 10 honor its arguments, confirm before constructing `Car`, and guarantee cleanup.
8. Add a preflight report that checks camera, I2C, ultrasonic, encoder availability, and power
   state without moving.

Gate A acceptance:

- full tests and Ruff pass;
- no hard-coded speed-150 timing is used for speed 200;
- no motor object is created before operator confirmation;
- all frame conversions have synthetic known-transform tests;
- duplicate anchor IDs fail clearly instead of selecting the first detection.

### Gate B — Manual repositioning, still no motor commands

1. Have the operator physically reposition the powered-off/stopped car to 3-5 marked floor
   locations while keeping the wall anchor fixed.
2. Capture five frames at each location.
3. Verify repeatability at each location and compare camera motion against tape measurements.
4. Measure camera and HC-SR04 offsets relative to the chassis rotation center.
5. Confirm whether camera optical forward is aligned with chassis forward and record its yaw sign.

Gate B acceptance:

- repeated position spread is below 1 cm while the anchor is fully visible;
- measured displacement direction matches the room-frame axis contract;
- camera-to-chassis and ultrasonic-to-chassis extrinsics are recorded with uncertainty;
- moving the car makes the previous `/tmp/room-pose*.json` visibly stale.

### Gate C — Lifted or secured rotation calibration

This gate moves motors. Require the operator beside the car and able to cut power.

1. Run power/I2C/ultrasonic/camera preflight first.
2. Verify wheels lifted or chassis secured.
3. Calibrate one explicit low spin speed; do not reuse the speed-150 value at speed 200.
4. Prefer deriving spin phase/rate from repeated sightings of the fixed visual anchor during a
   rotation, with timestamps, instead of manual timing alone.
5. Confirm motor stop in normal completion, error, Ctrl-C, and camera/sensor failure paths.

Gate C acceptance:

- measured speed and revolution time are stored together;
- scan angle uses that run's measured/configured duration;
- repeated 360-degree result has a stated error range;
- no forward motion occurs.

### Gate D — One low-speed stop-capture-localize step

This gate moves the car on the floor. The operator must remain beside it.

1. Clear the path and place a large obstacle 50-60 cm ahead for the initial safety trial.
2. Use a short, low-speed pulse; stop before taking vision measurements.
3. Keep HC-SR04 stopping active locally on the Pi throughout motion.
4. Capture before/after image, scan, command timing, obstacle status, and pose bundle.
5. Compare visual displacement, rough command odometry, and ICP without yet accepting ICP as
   global truth.

Gate D acceptance:

- the car stops on obstacle and at timeout;
- movement direction matches the declared frame;
- before/after bundle can be replayed entirely on the Mac;
- no autonomous second step is allowed on a failed localization or sparse scan.

### Gate E — Bounded mapping demo

Only after Gates A-D pass:

1. Start with a maximum of 2-3 explicitly authorized steps.
2. Fuse sensor pose, not raw camera pose, into the grid.
3. Apply visual-anchor correction when available and label dead-reckoned intervals as uncertain.
4. Stop on obstacle, low power, sensor failure, localization rejection, operator abort, or step
   budget.
5. Save the session bundle and render the map on the Mac.
6. Increase the step budget only after replay confirms no axis/sign/timestamp errors.

Do not claim room dimensions from the occupancy-grid bounding box until the pose chain and wall
extraction are independently validated against at least one long tape-measured room dimension.

## First Recommended Coding Task

The next AI should implement Gate A only, with this order:

1. add pure scan-timing and SE(2) frame conversion utilities under `src/carbot/`;
2. add regression tests for script 11's configured spin duration and the vision-to-map axis
   conversion;
3. repair script 10's ignored option and cleanup ordering;
4. refactor script 11 to consume the tested utilities and separate `spin_speed` from
   `drive_speed`;
5. add a non-moving `--preflight` or dry-run path;
6. stop and report the diff/test evidence before requesting a hardware motion test.

Do not integrate autonomous movement in the same change as the coordinate/timing fixes. Keep the
first change reviewable and hardware-independent.

## Questions the Next AI Must Resolve Before Ground Motion

1. Where is the chassis rotation center relative to the camera optical center, in centimetres?
2. Where is the HC-SR04 measurement origin relative to that center?
3. Is camera optical forward aligned with chassis forward, or is there a fixed yaw offset?
4. What exact spin speed will mapping use, and what is its measured revolution time at the current
   battery voltage?
5. Will the demo use only ID 0 loop closure, or will unique IDs 1-4 receive measured world poses?
6. What maximum number of steps and floor area is authorized for the first bounded run?

These are product/hardware facts, not values to guess from code.

## Copy-Paste Task for the Next AI

```text
Continue the Car-and-Robotic-Arm project from the current GitHub `main` branch.

Read AGENTS.md, CONVENTIONS.md,
docs/progress/2026-08-14-vision-room-anchor.md, and
docs/handoff-2026-08-14-vision-to-mapping.md completely before acting.

Implement Gate A only: repair and unit-test scan timing, coordinate-frame conversion,
script 10 cleanup/argument handling, script 11 separate spin/drive speed handling, duplicate
anchor rejection, and a no-motion preflight. Do not run motors. Preserve ignored calibration
source frames under their canonical `scratch/` path. The Pi repo is behind GitHub and contains
untracked runtime copies, so follow the recoverable Pi sync procedure in the handoff instead of
running a blind git pull.

Present the plan before non-trivial code changes. After implementation, run targeted Ruff and the
full pytest suite, then report the exact remaining hardware measurements needed for Gate B/C.
Do not commit or push unless explicitly authorized.
```

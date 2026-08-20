# 2026-08-14 Vision Calibration and Fixed Room Anchor

## Result

The IMX500 camera now has measured intrinsics, image undistortion, metric AprilTag 36h11 pose,
and a stable room-frame pose derived from a fixed wall-mounted ChArUco board plus AprilTag ID 0.
The final static test detected all 24 ChArUco corners in 5/5 frames and localized the camera with
less than 1.4 mm range across each position axis.

This makes the vision anchor ready for integration into the mapping pipeline. It does **not** make
the existing autonomous exploration script safe or geometrically correct; see
[Mapping Scripts That Must Not Run Yet](#mapping-scripts-that-must-not-run-yet).

## Final Fixed-Wall Setup

These are measured properties of the physical prints, not the nominal dimensions written in the
PDFs:

| Item | Verified value |
|---|---:|
| AprilTag family / ID | `tag36h11`, ID `0` |
| AprilTag outer black square | 70 x 70 mm |
| AprilTag center above floor | 14.65 cm |
| ChArUco dictionary | `DICT_5X5_100` |
| ChArUco layout | 5 x 7 squares, 24 internal corners |
| Printed checker pattern | 140 x 195 mm |
| Actual square pitch X | 28.000 mm |
| Actual square pitch Y | 27.857 mm |
| Mounting | Both prints flat, upright, and fixed on the same vertical wall |

The room frame is right-handed:

- `X`: away from the anchor wall into the room
- `Y`: along the anchor wall to the right
- `Z`: upward from the floor
- origin: floor point directly below the center of AprilTag ID 0

The wall prints are infrastructure. Moving, rotating, replacing, or rescaling either print
invalidates the anchor measurements and requires this setup to be measured again.

## Camera Intrinsic Calibration

The calibration used 17 selected 4056 x 3040 images from the fixed IMX500 camera. The printed
ChArUco pattern was measured after printing and the object points were corrected independently in
X and Y because the printer scaling was slightly anisotropic.

```text
Camera matrix:
  fx = 3106.5869209698417
  fy = 3106.3569512035956
  cx = 2043.4708180127138
  cy = 1439.3684030301329

Distortion [k1, k2, p1, p2, k3]:
  [0.0396059074286428,
   0.20645962337833404,
  -0.005703086517252836,
   0.0028961090890137478,
  -0.438957017627612]

RMS reprojection error: 1.7266066319941442 px
```

The rational distortion model improved RMS by only 0.028 px and produced large cancelling
coefficients, so the five-coefficient plumb-bob model was retained to avoid overfitting.

The calibration is directly valid only for an uncropped 4056 x 3040 frame. A resized full frame
with the same aspect ratio may scale the camera matrix. A cropped mode needs crop-aware
intrinsics; it must not silently reuse this matrix.

## Why the Hybrid Anchor Is Necessary

AprilTag ID 0 alone gave stable metric translation but unstable rotation when viewed nearly
front-on. Across five stationary images:

- AprilTag forward distance stayed within 59.2-59.6 cm.
- Camera height inferred from the single square varied from 4.8-22.9 cm.
- Inferred elevation varied by about 17 degrees.

This is the planar square-pose ambiguity, not evidence that the camera physically moved. Taking
the precise-looking single-tag rotation as ground truth would have corrupted the world transform.

The final estimator therefore separates responsibilities:

- the larger ChArUco board and its 24 corners determine camera orientation relative to the wall;
- AprilTag ID 0 supplies the known world point and metric translation;
- multiple frames are filtered by reprojection error and pose deviation;
- inlier positions use a median and rotations use an orthonormalized mean;
- missing detections are skipped instead of invalidating an otherwise good burst.

## Final Raspberry Pi Verification

Command:

```bash
cd ~/Car-and-Robotic-Arm
PYTHONPATH=src python3 examples/13_cam_room_pose.py --anchor-height-cm 14.65
```

Final fixed-wall result:

```text
Room pose from 5/5 inlier frames
wall_distance = 61.33 cm
wall_right    = -0.87 cm
height        = 11.05 cm
heading       = 179.82 deg
elevation     = 16.86 deg

Inlier position ranges:
X = 0.08 cm
Y = 0.09 cm
Z = 0.13 cm
```

Every frame detected 24/24 ChArUco corners. Board reprojection error was 0.56-0.65 px and tag
reprojection error was 1.17-2.35 px, below the configured 3 px rejection threshold. The result is
consistent with the operator's approximate 60 cm wall distance and 10.5 cm optical-center height.

Runtime outputs on the Pi:

```text
/tmp/room-pose.json
/tmp/room-pose.jpg
/tmp/room-pose-capture-01.jpg ... /tmp/room-pose-capture-05.jpg
```

The script is camera-only and never imports or accesses the motors.

## Pitfalls Hit Today

### 1. Printed dimensions are not PDF dimensions

The AprilTag PDF specified a 75 mm black square, but the physical print measured 70 mm. Assuming
75 mm would overestimate every metric distance by about 7.1%.

The ChArUco PDF specified a 150 x 210 mm pattern and a 100 mm check line. The physical print
measured 140 x 195 mm and the check line measured 93.5 mm. X and Y scaling differed by about 0.5%,
so applying one uniform scale was also wrong. Always measure the physical black/checker pattern
after printing and disable `Fit`, `Scale to fit`, and `Shrink oversized pages` where possible.

### 2. A digital result is not automatically ground truth

The camera can print several decimal places while still using a wrong tag size, distorted lens,
ambiguous planar rotation, or incorrect world transform. The manual 60 cm and 10.5 cm
measurements were not used to force the solution, but they were valuable independent checks that
exposed the single-tag height failure and later validated the hybrid result.

### 3. The camera must be mechanically fixed

Changing camera tilt after calibration does not invalidate intrinsic lens calibration, but it does
invalidate camera-to-robot extrinsics and changes the mapping viewpoint. The camera was fixed
before the final anchor measurements. Keep the mount rigid for all mapping runs.

### 4. A movable board is not a room anchor

The first ChArUco and ID 0 pair was mounted on a cardboard box. It produced a stable pose over a
short burst, but moving or rotating the box changed the room result by centimetres and degrees.
Both targets were reprinted and mounted on a permanent wall before accepting the final result.

### 5. Duplicate IDs and duplicate ChArUco boards break association

For one test, the old box-mounted targets remained visible while the new wall targets were also in
frame. The camera detected two AprilTags both claiming ID 0, and the two ChArUco boards reused the
same marker IDs. The estimator could not determine a unique anchor and produced zero valid
frames. Remove or fully cover old copies; IDs visible in the same operating area must be unique.

### 6. The full quiet border and flat mounting matter

Do not cut into an AprilTag's white quiet border. Keep both targets flat, matte, fully visible, and
free of glare or wrinkles. A partly hidden target may be skipped, so the burst requires at least
three valid frames.

### 7. Coordinate-axis order can accidentally create a reflection

Using `X=wall-right`, `Y=away`, and `Z=up` creates a left-handed basis. The implemented room frame
uses `X=away`, `Y=wall-right`, and `Z=up`, and rejects reflection matrices rather than silently
accepting them as rotations.

### 8. Pi and Mac OpenCV environments differ

The Pi's apt-managed system Python already provides Picamera2 and OpenCV with `aruco`; hardware
scripts run with `PYTHONPATH=src python3`. Local development uses:

```bash
uv run --extra vision pytest
```

Installing OpenCV into the Pi project venv is not necessary for this path.

## Mapping Scripts That Must Not Run Yet

The vision anchor is ready, but `examples/10_sonar_motion_calibrate.py` and
`examples/11_sonar_explore_mapping.py` still contain known preflight defects:

1. `10_sonar_motion_calibrate.py --spin-seconds` is parsed but ignored; the motor duration is hard-coded
   to 4.0 seconds while the message describes a full-circle calibration.
2. `11_sonar_explore_mapping.py` spins at `DRIVE_SPEED = 200`, but `SPIN_360_S = 8.2` was measured at
   speed 150.
3. `--spin360` changes scan duration, but angle conversion still uses the global 8.2-second
   constant, distorting the polar scan when another value is supplied.
4. Neither script consumes encoder state even though the earlier handoff recommends re-running
   them after adding encoders.
5. The current mapping tests are synthetic algorithm tests; they do not replay the real drift,
   wrong-spin-rate, duplicate-anchor, or crash cases observed on hardware.

Do not start autonomous room exploration with script 11 until these issues are fixed and tested at
low speed with an operator beside the robot.

## Files Added

- `src/carbot/vision.py` — calibration loading/scaling, undistortion, AprilTag pose, measured
  ChArUco pose, hybrid room transform, multi-frame outlier rejection, and annotation.
- `examples/12_cam_apriltag_pose.py` — static single-frame AprilTag measurement and diagnostic output.
- `examples/13_cam_room_pose.py` — static multi-frame hybrid room pose and JSON output.
- `tests/test_vision.py` — synthetic and real-metadata regression tests.
- `assets/reference/apriltags/apriltag-tag36h11-id-0-to-4-75mm-a4.pdf` — printable tags; physical
  size must still be measured.
- `assets/reference/camera-calibration/imx500-charuco-5x7-30mm-a4.pdf` — printable calibration
  board; physical pattern must still be measured.
- `assets/reference/camera-calibration/2026-08-14-imx500-4056x3040/calibration.json` — selected
  calibration and per-view residual metadata. The 17 full-resolution source captures remain local
  under `scratch/camera-calibration/2026-08-14-imx500-4056x3040/source-frames/` and are not
  published because they contain the room interior and add about 36 MB.

## Verification

```text
uv run ruff check src/carbot/vision.py examples/12_cam_apriltag_pose.py \
  examples/13_cam_room_pose.py tests/test_vision.py
All checks passed

PYTHONPATH=src uv run --extra vision pytest -q
70 passed
```

## Next Development Step

Feed `room-pose.json` into the mapping pipeline as a global correction whenever the fixed anchor is
visible. Before any motor-moving mapping run, repair scripts 10/11, add real captured-data
regressions, retain continuous HC-SR04 stopping, and begin with low-speed stop-capture-localize
steps while an operator is beside the car.

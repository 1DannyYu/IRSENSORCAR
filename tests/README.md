# Tests Overview

This folder contains the automated regression and validation tests for the robot project. The tests cover hardware protocol checks, camera and vision behavior, line following, navigation, mapping, and motion logic.

The goal of the suite is to verify that the system behavior remains correct after changes to the codebase and to catch regressions before running on real hardware.

## Test categories

### 1. Hardware and low-level control
- `test_nezha.py` — Validates the `NeZha` board interface and I2C command generation.
- `test_servo_check.py` — Verifies servo channel behavior and servo command mapping.
- `test_power.py` — Checks power-status decoding and electrical state logic.
- `test_sonar.py` — Tests ultrasonic distance calculations and echo interpretation.
- `test_ir_tracing.py` — Tests IR tracing channel normalization: every channel reports 1 on black / 0 on white.
- `test_motion.py` — Validates motion model math, such as distance-to-time conversion.

### 2. Vision and camera processing
- `test_vision.py` — Covers camera calibration, distortion handling, AprilTag detection, and pose estimation.
- `test_frame_quality.py` — Validates frame quality assessment, tiling, and image-shape validation.
- `test_ground_view.py` — Tests ground-view line tracking and error estimation.
- `test_visual_yaw.py` — Checks visual yaw estimation against known turns.
- `test_vision_avoid.py` — Verifies obstacle and avoidance detection geometry and decision logic.

### 3. Navigation and control
- `test_line_follow.py` — Verifies black-line detection, threshold handling, ROI filtering, and line error calculations.
- `test_line_nav.py` — Tests left/right wheel command generation from line-follow state.
- `test_tag_nav.py` — Validates tag-based navigation state transitions and hold/drive logic.
- `test_route_nav.py` — Validates route-tracking phase transitions and state control.
- `test_route_plan.py` — Checks route plan structure and expected step configuration.

### 4. Mapping and localization
- `test_mapping.py` — Covers ICP alignment, rigid transform estimation, gap detection, occupancy grids, and scan conversion.
- `test_scale.py` — Validates scale recovery and metric conversion between camera and world space.
- `test_landmarks.py` — Checks landmark/tag geometry, rotation math, and JSON map loading.

### 5. Frame and utility math
- `test_frames.py` — Tests scan-angle math and frame-related geometry helpers.
- `test_tags` is not a standalone file here, but tag and landmark logic is exercised through the landmark and navigation tests.

---

## Script-by-script summary

### `test_frame_quality.py`
Main purpose:
- Validate image quality scoring logic.
- Confirm frame dimensions, tile grids, and keypoint extraction behave as expected.
- Reject invalid image sizes or unsupported shapes.

What it checks:
- image dimensions and grid layout
- grayscale/color normalization behavior
- invalid input rejection
- tile-based quality assessment logic

### `test_frames.py`
Main purpose:
- Verify scan angle and frame-related geometry functions.

What it checks:
- angle wrap-around behavior across revolutions
- configured spin settings and angular normalization
- motion math related to the scan sequence

### `test_ground_view.py`
Main purpose:
- Validate ground-view line tracking and visual error estimation.

What it checks:
- line offset in bird's-eye view
- tracking stability under distractions
- preferred existing target selection
- robust estimation under noise and near-field blockers

### `test_landmarks.py`
Main purpose:
- Validate tag and map landmark math, including rotation and map loading.

What it checks:
- world-to-tag rotation transforms
- proper rotation matrix properties
- JSON landmark map parsing and validation
- landmark/pose logic for navigation tasks

### `test_line_follow.py`
Main purpose:
- Verify the black-line detection pipeline used for following a track.

What it checks:
- centred line yields near-zero error
- shifted line yields signed error values
- blank floor returns no detected line
- ROI and threshold rules exclude chassis/shadow artifacts
- noisy or sparse features are ignored

### `test_line_nav.py`
Main purpose:
- Validate command generation from line-follow state.

What it checks:
- offset-to-wheel-speed mapping
- steering behavior for left/right drift
- stability when the line is centered
- no-drive behavior when the line is invisible
- search/follow state transitions

### `test_mapping.py`
Main purpose:
- Test map generation, scan registration, and occupancy-grid logic.

What it checks:
- ICP registration accuracy
- translation/rotation recovery
- gap detection in range scans
- occupancy grid updates and bounds
- scan file parsing and conversion to points

### `test_motion.py`
Main purpose:
- Validate simple robot motion equations.

What it checks:
- forward-distance time conversion
- basic model calculations used for driving commands

### `test_nezha.py`
Main purpose:
- Verify low-level NeZha motor, encoder, servo, and I2C command behavior.

What it checks:
- correct command register selection
- encoded PWM values for motor output
- encoder sign interpretation
- servo angle conversion
- invalid input rejection

### `test_power.py`
Main purpose:
- Validate power-status interpretation and bit decoding.

What it checks:
- status bit parsing
- undervoltage and throttle state reporting
- clean/dirty power conditions

### `test_route_nav.py`
Main purpose:
- Verify route-tracker state machine logic.

What it checks:
- phase progression across route steps
- turn detection and roundabout transitions
- remaining-distance reporting
- no self-driving or invalid command invention

### `test_route_plan.py`
Main purpose:
- Validate route-plan structure and expected step count.

What it checks:
- route name and step count
- step metadata and labels
- route consistency

### `test_scale.py`
Main purpose:
- Validate metric scale estimation from scene geometry.

What it checks:
- true scale recovery
- scale estimation under rotation/offset
- robustness across multiple scale values

### `test_servo_check.py`
Main purpose:
- Validate servo behavior and safety limits.

What it checks:
- servo fixture setup
- channel initialization and motion sequencing
- command generation for servo movement

### `test_sonar.py`
Main purpose:
- Validate ultrasonic range calculation.

What it checks:
- distance conversion from pulse width
- echo interpretation
- expected readings for normal and zero-distance cases

### `test_ir_tracing.py`
Main purpose:
- Verify the IR tracing sensor normalization contract: all channels report
  1 on black and 0 on white, regardless of raw comparator polarity.

What it checks:
- default polarity mapping (raw HIGH = black, raw LOW = white)
- per-channel independent mapping of mixed surfaces
- `invert` flipping only the named channels
- `raw()` passthrough for calibration
- read order follows channel (Out) order
- invalid `invert` indices and empty channel lists are rejected

### `test_tag_nav.py`
Main purpose:
- Validate tag-based navigation state logic.

What it checks:
- state progression between departure, follow, and hold phases
- whether a valid position fix is required before moving
- navigation command generation under different conditions

### `test_vision_avoid.py`
Main purpose:
- Check obstacle detection and avoidance logic from vision inputs.

What it checks:
- detection geometry and area calculations
- center and edge clipping behavior
- label fallback handling
- central obstacle blocking logic

### `test_vision.py`
Main purpose:
- Validate camera calibration and visual pose estimation.

What it checks:
- real calibration loading
- scale-to-resolution conversion
- board and tag pose estimation
- AprilTag detection and undistortion behavior

### `test_visual_yaw.py`
Main purpose:
- Validate visual yaw estimation from camera input.

What it checks:
- known-turn recovery accuracy
- confidence/trustworthiness of yaw estimation
- estimation under different input conditions

---

## Recommended usage

When debugging a subsystem, start with the test file that matches the area of interest:

- Hardware protocol → `test_nezha.py`
- Line following → `test_line_follow.py`
- Navigation logic → `test_line_nav.py`, `test_route_nav.py`, `test_tag_nav.py`
- Mapping/localization → `test_mapping.py`, `test_landmarks.py`, `test_scale.py`
- Vision/camera → `test_vision.py`, `test_visual_yaw.py`, `test_ground_view.py`
- Obstacle detection → `test_vision_avoid.py`

This folder is the project’s safety net: each test documents a specific contract that the robot code must keep working.

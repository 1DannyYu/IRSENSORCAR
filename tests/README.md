# Tests Overview

This folder contains the automated regression and validation tests for the robot project. The tests cover hardware protocol checks, line following, navigation, mapping, and motion logic.

The goal of the suite is to verify that the system behavior remains correct after changes to the codebase and to catch regressions before running on real hardware.

## Folder layout

Tests are flat, one file per subsystem: NeZha I2C board, IR 4-channel tracing sensor, sonar
distance math, motion and timing calculations, route/tag navigation state machines, and the
extracted pure decision logic from the numbered example scripts.

| File | Purpose |
|---|---|
| `test_car.py` | Differential-drive `Car` layer and the I2C-safe stop path |
| `test_example_39_hardcoded_start.py` | Example 39 sensor-blind Phase 1 manoeuvre decisions |
| `test_example_46_transition_drive.py` | Example 46 argument consolidation helper |
| `test_example_47_motor_one_second_check.py` | Example 48's one-second `move_for` wrapper |
| `test_example_49_phase1_phase2_trace.py` | Example 49's per-tick `decide_step` state machine |
| `test_frames.py` | Scan angle math, pose conventions, and sensor extrinsics |
| `test_ir_geometry.py` | IR 4-bit pattern classification and wheel ratios |
| `test_ir_line_nav.py` | IR-guided per-tick navigation and line-follow command policy |
| `test_ir_modes.py` | IR auto-tracing modes, circle-mode state, and roundabout timing |
| `test_ir_route.py` | IR route waypoint structure and step metadata |
| `test_ir_route_plan.py` | IR route plan construction and step count |
| `test_ir_tracing.py` | IR sensor polarity normalization (1 = black / 0 = white) |
| `test_line_nav.py` | Line-follow state machine: offset -> wheel speeds, search/follow |
| `test_map1_phases.py` | Map 1 multi-phase controller transitions |
| `test_mapping.py` | Sonar scan ICP, occupancy grid, and gap detection |
| `test_motion.py` | Distance-to-time conversion and basic motion equations |
| `test_nezha.py` | NeZha I2C register selection, PWM encoding, servo angles |
| `test_power.py` | `vcgencmd get_throttled` bit decoding and summary text |
| `test_route_nav.py` | Route-tracker phase progression and turn detection |
| `test_route_plan.py` | Task 1 route plan structure and expected step count |
| `test_servo.py` | Servo channel mapping and safety limits |
| `test_sonar.py` | HC-SR04 pulse-width-to-distance conversion |
| `test_tag_nav.py` | AprilTag-supervised departure/follow/hold navigation states |

---

## Script-by-script summary

### `test_frames.py`
Main purpose:
- Verify scan angle and frame-related geometry functions.

What it checks:
- angle wrap-around behavior across revolutions
- configured spin settings and angular normalization
- motion math related to the scan sequence

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

### `test_servo.py`
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

---

## Recommended usage

Run the whole suite from the repository root:

```bash
uv run pytest tests/ -v
```

When debugging a subsystem, start with the test file that matches the area of interest:

- Hardware protocol -> `test_nezha.py`
- IR line follow -> `test_ir_modes.py`, `test_ir_line_nav.py`
- Navigation logic -> `test_line_nav.py`, `test_route_nav.py`, `test_tag_nav.py`
- Sonar mapping -> `test_mapping.py`
- Example 49 decisions -> `test_example_49_phase1_phase2_trace.py`

This folder is the project's safety net: each test documents a specific contract that the robot code must keep working.

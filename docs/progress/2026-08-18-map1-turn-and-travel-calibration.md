# Map1 Turn and Travel Calibration — Work Log (2026-08-18)

Continues the motion-calibration line from
[`2026-08-14-travel-speed-and-coverage.md`](2026-08-14-travel-speed-and-coverage.md)
(spin rate 53.5 deg/s at speed 200, measured on a textured wall for the camera
feature-matcher) and [`examples/37_map1_motor_test.py`](../../examples/37_map1_motor_test.py).
This session re-measured both turn and forward travel **directly on the Map1
printed track paper**, because friction differs between the paper and other
floors, so wall-measured rates do not transfer reliably to the Task-1 surface.

All runs were done on **Map 1** with an operator standing beside the car, able
to cut main power instantly.

## 1. Scope and Result

- **Turn calibration (direction changes)** — for each of the durations
  **2, 4, 6, 8, 10 seconds**, the car spun in place and the operator read the
  resulting angle by eye (reference: the Map1 T-junction's printed right angle /
  a straightedge). See
  [`examples/41_motor_spin_angle_sweep.py`](../../examples/41_motor_spin_angle_sweep.py).
- **Forward travel distance** — for each of the durations **1, 2, 3 seconds**,
  the car drove straight and the distance travelled was measured.
- The two calibrations give the duration -> angle and duration -> distance
  conversions needed to script real Map1 moves (turns and straight legs).

## 2. Turn Tests — duration vs. turned degrees (Map 1)

Script:
[`examples/41_motor_spin_angle_sweep.py`](../../examples/41_motor_spin_angle_sweep.py)
(spin speed 150, direction right, durations `2,4,6,8,10`).

| Duration (s) | Observed angle (deg) | Deg/s |
|---|---|---|
| 2 | 77 | 38.5 |
| 4 | 160 | 40.0 |
| 6 | 210 | 35.0 ← outlier |
| 8 | 327 | 40.9 |
| 10 | 398 | 39.8 |

Average deg/s across all readings: 38.8 (39.8 if the 6 s outlier is excluded)

> Note from the script: short durations include the motor's startup dead time
> and read a lower deg/s than longer ones; weight the longer-duration rows more
> when picking a final number.

## 3. Forward Travel Tests — duration vs. distance (Map 1)

For each duration the car drove forward on the map and the distance travelled
was measured (straight-line distance on the paper).

<!-- two runs were done for the 2 s duration -->

| Duration (s) | Distance moved (cm) | Speed (cm/s) |
|---|---|---|
| 1 | 10.5 | 10.5 |
| 2 | 21 / 20 (two runs) | ~10.3 (10.5 / 10.0) |
| 3 | 30 | 10.0 |

Implied forward speed on the paper is consistent at roughly **10.0-10.5 cm/s**.

## 4. Observations / Notes

- **T-junction creep distance (~9.5 cm).** When the car reaches a T-junction,
  it must keep moving forward a further **~9.5 cm** after the junction is first
  detected, so that it can rotate at the junction point (its rotation centre
  ends up on the crossing) instead of spinning short of it.
- **Line lost in the IR sensor gap.** If the black line falls in the gap
  *between* the IR sensor's detectors (no channel reads it), the car recovers
  by **rotating left and right and moving forward in small increments** until
  it re-detects the line.
- At ~10.0-10.5 cm/s forward, the ~9.5 cm creep is roughly a 1 s drive at the
  measured Map1 speed.

## 5. Follow-up

- Decide the Map1 turn recipe: pick the duration that produces a clean
  **90-degree** (and 180-degree) turn from the table in §2, and the forward
  durations for the leg lengths needed by the Map1 route
  ([canonical Map 1 route image](../../assets/reference/map-1/2026-08-21-ir-route-map.png)).
- The wall-measured 53.5 deg/s (speed 200) in
  [`2026-08-14-travel-speed-and-coverage.md`](2026-08-14-travel-speed-and-coverage.md)
  remains the camera feature-matcher value; the Map1 numbers above are the
  on-track values to use for navigation on the printed map.

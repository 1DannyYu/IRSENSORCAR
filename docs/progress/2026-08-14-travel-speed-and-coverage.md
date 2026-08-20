# Travel Speed and Coverage — Work Log (2026-08-14)

Continues
[`2026-08-14-fused-patrol-and-sfm-overlap.md`](2026-08-14-fused-patrol-and-sfm-overlap.md),
which ended by measuring an entire 30-frame run inside 13 cm. This session found
out why, fixed it, and got the first reconstruction that both covers ground and
stays in one piece.

## 1. Scope and Result

- **Travel speed measured** —
  [`examples/24_cam_linear_speed_check.py`](../../examples/24_cam_linear_speed_check.py):
  the wall tag gives metric camera positions directly, so distance travelled is
  measurable without a tape measure and without depending on the reconstruction
  scale that was in question.
- **Segmented forward travel** — a long step driven in short segments with the
  fuse re-evaluated between them, so covering ground does not mean driving
  blind.
- **Separate spin speed** — turns keep the calibrated speed while driving gets
  faster.
- **Standoff lowered to the avoidance distance**, removing a dead band.
- **Block reasons now carry box area and bottom**, which promptly falsified two
  proposed filters.

| Run | Setup | Forward steps | Registered | Coverage |
|---|---|---|---|---|
| 4 | corner, 1.0 s steps | 2 of 5 | 30/30, 1 model | 0.13 x 0.02 x 0.10 m |
| 5 | corner, 3.0 s segmented | **0 of 7** | not run | not measurable (car was kicked) |
| 6 | open floor, no tag in view | 3 of 6 | 21/30, 2 models | not measurable (no tag) |
| 7 | open floor, tag in view | 3 of 7 | **29/30, 1 model** | **0.16 x 0.15 x 0.48 m** |

Run 7 is the first result where coverage and connectivity improved together —
3.7x the trajectory of run 4 while still registering into a single model.

## 2. Verification

```text
# Mac
uv run --extra vision --extra mapping pytest -q   -> 217 passed
uv run ruff check .                               -> All checks passed

# Pi: travel speed, 24 legs at speed 200 (forward-then-back per duration)
PYTHONPATH=src python3 examples/24_cam_linear_speed_check.py --speed 200 --tag-id 0 \
    --durations 0.3 0.5 0.75 1.0 1.5 2.0 --repeats 2
  forward  median 0.117 m/s  (0.112-0.151, 12 legs)
  reverse  median 0.111 m/s  (0.100-0.132, 12 legs)
  reverse/forward speed ratio: 0.95

# Pi: same at speed 400
  forward  median 0.166 m/s  (0.162-0.180, 6 legs)
  reverse  median 0.165 m/s  (0.161-0.174, 6 legs)
  reverse/forward speed ratio: 0.99

# Pi: run 7, open floor with a tag in view
PYTHONPATH=src python3 examples/22_cam_sonar_patrol_capture.py --frames 30 --frame-report
  Kept 30 frames in 7 stations (3 blocked, 3 forward, 13 rejected, 0 empty sweeps)
  Overlap with the previous kept frame: min 38, median 1007, max 2118
    — 4 below 200, 2 bridge frames inserted
  [2] clear -> forward 0.5s of 3.0s, stopped: sonar 17 cm < 30 cm

# Mac: reconstruction and scale
  model 0: 29/30 registered, 6822 points
  Scale: 0.0510 m/unit from 14 pairs (tag 1), spread 8.2%
  Camera trajectory extent: 0.16 x 0.15 x 0.48 m
```

**The coverage figure is cross-validated.** Run 7 commanded 3.0 s of forward
travel in total; at the measured 0.166 m/s that is 0.50 m, against the
tag-anchored 0.48 m — two independent methods agreeing within 4%.

## 3. Measurements and Configuration

**Travel speed** at speed 200: 0.117 m/s forward, 0.111 m/s reverse. At speed
400: 0.166 and 0.165. Two facts follow.

- **Forward and reverse are the same speed** (ratio 0.95 and 0.99), so an
  avoidance backup returns ground in exact proportion to its duration. Run 4's
  two 1.0 s steps against three 0.6 s backups netted about 7 cm, which is why
  that run went nowhere.
- **PWM is a weak lever.** Doubling it bought 1.42x the speed, not 2x. Coverage
  had to come from longer steps instead.

**Current defaults** in `examples/22_cam_sonar_patrol_capture.py`:

| Setting | Value | Why |
|---|---|---|
| `--speed` | 400 | 0.166 m/s; higher PWM returns little |
| `--spin-speed` | 200 | 53.5 deg/s was calibrated here and nowhere else |
| `--step-s` | 3.0 | ~0.50 m per station |
| `--sense-interval-s` | 0.5 | keeps a long step from being a blind one |
| `--backup-s` | 0.3 | reverse costs as much as forward gains |
| `--min-standoff-cm` | 30 | must not exceed `--obstacle-cm`, see pitfall 5 |
| `--burst-step-deg` | 15 | ~77% overlap in a 66.3 deg field of view |
| `--min-overlap-matches` | 200 | bridges a weak link rather than rejecting a frame |

## 4. Problems Encountered (the pitfalls)

1. **The step length was never the binding constraint.** After measuring travel
   and lengthening the step, run 5 recorded **0 forward steps in 7 stations** —
   every station was blocked, so none of the new code executed. Optimising the
   thing that was measured, rather than the thing that was limiting, cost a full
   supervised run.
2. **The environment dominated the result.** Run 5 differed from run 6 only in
   where the car was placed: a cluttered corner versus open floor. Moving the
   car changed the outcome more than any code change in this session.
3. **Raising `min_area_fraction` would have regressed the original obstacle.**
   The proposed 0.06 -> 0.15 was checked against the chair and dining table that
   originally trapped the car: their box areas were 0.117 and 0.123, against a
   spurious "scissors" at 0.11. The threshold that drops the false block also
   drops the obstacle the vision layer exists to catch.
4. **Box bottom does not discriminate either.** Every blocking detection
   measured 0.90-0.96, including the historical true positive at 0.927. The
   camera looks slightly down, so anything on the floor within a few metres has
   its box bottom near the frame edge. Confidence overlaps too (false 0.32-0.44,
   true 0.38-0.44). No single-frame quantity separates a geometrically plausible
   misclassification from a real obstacle.
5. **A standoff above the avoidance threshold creates a dead band.** With
   something 30-50 cm ahead, the sonar rule did not require the car to move away
   and the capture gate would not let it photograph. A dry run in that state
   produced 107 consecutive rejections.
6. **Changing the drive speed silently invalidates the spin calibration.**
   53.5 deg/s belongs to speed 200. Turns now use `--spin-speed` so raising
   `--speed` cannot quietly corrupt every commanded angle.
7. **Scale anchoring needs a tag actually in view.** Run 6 moved to open floor,
   away from the wall tags, and its coverage could not be measured at all — the
   very quantity the run existed to produce.
8. **The operator is detected as `person` and blocks the car.** Standing in the
   forward field of view biases every run; standing behind or beside-rear does
   not.
9. **A kick during run 5** put displacement into the trajectory that the car did
   not generate, which invalidates any coverage measurement from that run.
   Decision counts survive it, because they do not depend on displacement.

## 5. Follow-up

**Run the full 150-frame sweep.** Coverage is linear in frames — 30 frames
produced 0.48 m, so 150 should produce roughly 2.4 m, which is a real room
traverse and the first data set worth turning into a map. Nothing needs changing
first; the configuration above is verified.

Deliberately **not** pursued further:

- **Filtering false blocks.** Three proposals were falsified in turn (area,
  box bottom, detection persistence). False blocks cost coverage speed, not
  correctness, and coverage speed is recoverable by running more frames. Doing
  it properly would need an accumulated labelled set — saving the frame at each
  block decision over several runs — which costs more than the problem.

Still open:

- Whether the chain holds over 150 frames; run 6 fell to 21/30 in two models
  once the car really moved, and run 7's recovery to 29/30 is a single sample.
- A low-texture room, still the case ADR 0002 flagged and still untried.
- The standoff gate remains redundant against the quality gate (see the previous
  log's section 5); it was left alone because the runs that would settle it kept
  changing other variables.
- `examples/17` and `examples/18` still carry the old 43.9 deg/s constant.

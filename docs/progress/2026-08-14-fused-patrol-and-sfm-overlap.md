# Fused Patrol and the SfM Overlap Problem — Work Log (2026-08-14)

Continues
[`2026-08-14-camera-modes-exposure-and-preflight-fix.md`](2026-08-14-camera-modes-exposure-and-preflight-fix.md).
This session built the fused vision+sonar patrol the handoff asked for, ran it
on hardware three times, and used COLMAP to find out why the frames were not
reconstructing. The answer was not image quality.

## 1. Scope and Result

- **Fused patrol** — [`examples/22_cam_sonar_patrol_capture.py`](../../examples/22_cam_sonar_patrol_capture.py):
  sonar and IMX500 detections combined through `carbot.vision_avoid.fuse`,
  capture in mode `single`, back up before a random turn, `--dry-run` for a
  stationary logic test.
- **Spin rate measured, not assumed** —
  [`examples/23_cam_spin_rate_check.py`](../../examples/23_cam_spin_rate_check.py) with
  [`src/carbot/visual_yaw.py`](../../src/carbot/visual_yaw.py): the camera
  measures the car's own rotation by matching features across a spin, so no
  protractor and no encoders are needed (ADR 0002 ruled out measuring the robot
  by hand).
- **Capture gates** — a pose is only photographed when the sonar says the camera
  has room and `carbot.frame_quality` finds enough textured tiles.
- **Burst capture** — several overlapping frames per station instead of one.
- **Shared sonar helper** — `Sonar.measure_nearest` replaces the copy of
  `read_distance` that `examples/17` and `examples/18` each carried.

Registration across the three hardware runs:

| Run | Capture policy | Largest model | Points | Models |
|---|---|---|---|---|
| 1 | one frame per station, no gates | not run (2 of 10 frames unusable) | — | — |
| 2 | one frame per station, gated | 3/10 (30%) | 202 | 4 islands |
| 3 | burst of 5 at ~20 deg, gated | 17/30 (57%) | 1606 | 3 |
| 4 | **capture through the turn, 15 deg, live overlap repair** | **30/30 (100%)** | **7972** | **1 connected** |

Run 4 is the first single connected model the photo route has produced. Three
changes got it there — the avoidance turn became a capture sweep (run 3's model
boundaries all sat on those turns), the step tightened from 20 to 15 deg for
~77% overlap, and a weak link is now repaired during the run by inserting a
bridging frame. It also got cheaper: 30 frames in 5 stations with 3 rejections,
against run 3's 10 stations and 18 rejections.

**Read that result with its scale.** Anchoring the model to the wall tag
(section 7) put the whole camera trajectory inside **0.13 x 0.02 x 0.10 m**. Run
4 is therefore a rotation panorama from essentially one spot, not a traverse of a
room. Registration is easy for near-pure rotation and the parallax is small, so
30/30 says the capture policy now produces a connected chain — it does not yet
say the robot can map a room.

## 2. Verification

```text
# Mac
uv run --extra vision --extra mapping pytest -q   -> 195 passed
uv run ruff check .                               -> All checks passed

# Pi: spin rate at speed 200 (14 measurements, 7 durations x 2 directions)
PYTHONPATH=src python3 examples/23_cam_spin_rate_check.py --speed 200
  Direction: spin_left/spin_right match the chassis on all 14 trusted measurements
  Fitted: angle = 53.5 deg/s x (duration - 0.005s) at speed 200
  Previously assumed 43.9 deg/s with no dead time (measured at speed 150)

# Pi: 30-frame burst patrol, operator beside the car
PYTHONPATH=src python3 examples/22_cam_sonar_patrol_capture.py --frames 30 --frame-report
  Kept 30 frames in 10 stations (5 blocked, 5 forward, 18 rejected, 0 empty bursts)
    rejected 12x: standoff
    rejected  6x: quality

# Mac: reconstruction of those 30 frames
uv run python scripts/run_colmap_sfm.py sfm2/images sfm2/work
  model 0:  4/30 registered, 1716 points
  model 1: 17/30 registered, 1606 points
  model 2: 13/30 registered, 3272 points

# Pi: run 4 — capture through the turn, 15 deg step, live overlap repair
PYTHONPATH=src python3 examples/22_cam_sonar_patrol_capture.py \
    --frames 30 --frame-report --keep-rejected
  Kept 30 frames in 5 stations (3 blocked, 2 forward, 3 rejected, 0 empty sweeps)
    rejected 3x: standoff
  Overlap with the previous kept frame: min 35, median 966, max 1453
    — 2 below 200, 1 bridge frames inserted

# Mac: reconstruction of run 4
  model 0:  4/30 registered, 4312 points
  model 1: 30/30 registered, 7972 points   <- every frame, one model
```

## 3. Measurements and Configuration

**Spin rate**: 53.5 deg/s at speed 200, dead time 0.005 s. The previous
43.9 deg/s came from speed 150, so every turn the patrol commanded was about 22%
larger than intended. The dead time is negligible, which was the opposite of the
expectation that motivated measuring it — small turns need no compensation, so a
20 deg burst step is simply 0.374 s of spin.

**Motor direction**: all 14 measurements agreed with the commanded direction, so
`spin_right` really does rotate the car right and the `config.py` wheel mapping
is correct. This closes the "vendor docs and code comments disagree" hazard in
[CLAUDE.md](../../CLAUDE.md) with a measurement rather than an opinion.

**Overlap**, matched with `frame_quality.repeatable_keypoints` on run 3:

```text
within a burst (~20 deg apart):   199 - 823 matches
across a station boundary:        11, 19, 23, 25, 30, 63, 71 matches
```

Run 2, for contrast, had *no* strong links at all — its ten frames formed four
islands with 600-1500 matches inside an island and 10-40 between, which is why
COLMAP reported "no good initial image pair found" and registered only 3.

**Model membership** in run 3 shows the same boundary structure: model 2 holds
frames 0-1 and 14-24, model 1 holds 2-13 and 25-29. Each model spans roughly
three stations and then breaks, and the breaks line up with the stations where
an avoidance turn of 30-150 deg happened.

**Capture gates**: standoff 50 cm, minimum 6 of 12 textured tiles. In run 3 they
rejected 18 of 48 attempted captures (12 standoff, 6 quality) and left no
station empty. Exposure is back to `auto`.

## 4. Problems Encountered (the pitfalls)

1. **Frame quality was never the reason the sweep failed.** Run 2's frames were
   sharp, correctly exposed, and individually good; the reconstruction failed
   because consecutive frames shared nothing. Photogrammetry needs *pairs*, and
   every per-frame metric in `carbot.frame_quality` is blind to that. The
   pairwise match matrix is what diagnosed it in minutes after per-frame metrics
   had been chased for two sessions.
2. **A frame shot 30 cm from a whiteboard looks like motion blur.** Run 1
   produced frames scoring 12 and 15 sharpness with 1/12 textured tiles, which
   read as blur; the images turned out to be perfectly sharp pictures of a blank
   panel filling 70% of the frame. Looking at the image settled in seconds what
   the metrics could not.
3. **The 97 ms shutter blur risk was imaginary.** At 38 ms with a 1.0 s settle,
   captures from a moving car came back sharp, so the open question from the
   exposure sweep is closed.
4. **Spot metering does not survive a rotating camera.** The stationary sweep
   picked it, but the patrol faces every direction: two frames came back with
   12.4% and 43.3% of pixels clipped when the car turned toward a window. A
   setting tuned on a camera that never moves needs re-testing on one that does.
5. **A sign error in `visual_yaw` passed its own test**, because the synthetic
   fixture warped the image the same wrong way. Every one of the first 14 spin
   measurements was rejected as "direction disagrees with the command" while the
   underlying numbers were correct and consistent. The test now checks the sign
   against a stripe whose displacement is computed independently of the warp
   helper — a second, differently-derived source of truth.
6. **A direction disagreement should not discard a magnitude.** Rejecting those
   samples threw away a perfectly good rate measurement. Spin rate is a scalar;
   direction is a separate fact and is now reported as its own measurement.
7. **`--keep-rejected` saved nothing when standoff did all the rejecting**,
   because a standoff rejection happens before the shutter. The flag now
   captures anyway, purely to record what was skipped — that is the case where
   the threshold most needs calibrating.
8. **Rounding made a working comparison look broken**: `standoff: 50 cm < 50 cm`
   was a 49.6 cm reading printed to whole centimetres.
9. **A scratch file named `struct.py` shadows the standard library** and breaks
   `pycolmap` with a confusing circular-import trace.

## 5. Threshold Calibration Against Run 3

Run 3 is the first data set with reconstruction ground truth, so the provisional
`QualityPolicy` thresholds were checked against it. Two of the three results are
negative.

**All 30 kept frames registered.** Model membership covers every frame, so there
is no negative example in the kept set. `min_textured_tiles=6` is therefore not
too lenient: the weakest frame that passed sat exactly on the threshold (6 of 12
tiles, 738 keypoints) and still registered.

**`min_sharpness=20` was falsified by the only frame that tested it.** Frame 13
scored 19 sharpness with 925 keypoints — the softest and sparsest capture in the
set — and reconstructed normally. It was never rejected because the patrol gates
on `textured_tiles` alone; the other five thresholds are measured and printed but
not enforced. That inconsistency is now a deliberate, documented choice, and
`min_sharpness` dropped to 10 so it flags gross failure only.

**A consecutive-match threshold cannot gate connectivity.** The obvious next
gate — reject a capture that shares too few matches with the previous one — does
not work, and the data says so before it was built:

```text
same model, consecutive:      19, 30, 63, 71, 145, 199, ... 823
different model, consecutive: 11, 23, 25
lowest same-model link:  19
highest different gap:   25   -> the ranges overlap; not separable
```

A pair with 19 matches stayed in one model while pairs with 23 and 25 split.
COLMAP matches exhaustively, so a frame can join a model through a
*non-adjacent* pair: connectivity is a property of the whole match graph, not of
the link to the previous frame.

**The standoff gate now looks redundant.** Run 4 used `--keep-rejected` and
rejected only 3 captures, all on standoff. Assessed afterwards, **all three would
have passed the quality gate comfortably** — 9/12, 8/12 and 7/12 textured tiles
with 3629-5475 keypoints. Standoff was introduced to stop the car photographing a
whiteboard from 30 cm, but that failure shows up directly as low textured tiles,
so the quality gate already covers it while standoff additionally discards good
frames.

Three samples is thin evidence and run 4 registered 30 of 30, so nothing was
changed: losing 3 captures out of 33 attempts costs about 9% of the run and
retuning risks a regression against a result that currently works. Revisit with
more rejects, and prefer lowering the threshold over removing the gate — the
sonar read is far cheaper than the ORB pass behind the quality gate.

## 7. Scale Anchoring, and What It Revealed

[`src/carbot/scale.py`](../../src/carbot/scale.py) and
[`scripts/anchor_sfm_scale.py`](../../scripts/anchor_sfm_scale.py) recover metres
per reconstruction unit from the 70 mm wall tag, using distance ratios only —
a COLMAP model differs from reality by a similarity transform, and ratios are
blind to its rotation and translation.

```text
uv run --extra vision --extra mapping python scripts/anchor_sfm_scale.py \
    sfm3/work/sparse/1 sfm3/images
  Tags: 12 images carry a usable detection (tag 0 in 7, tag 2 in 6)
  Scale: 0.0107 m/unit from 16 pairs, spread 9.5%
  Camera trajectory extent: 0.13 x 0.02 x 0.10 m
```

The first attempt returned 0.0114 m/unit with a 25% spread and was correctly
rejected by its own trust check. Two filters were too loose:

- **The baseline floor was absolute.** Reconstruction units are arbitrary, so a
  fixed `0.02` units meant nothing; pairs separated by 0.7-1.8 units produced
  ratios up to seven times those from well-separated pairs. The floor is now a
  fraction of the trajectory's own extent.
- **The reprojection limit was 2.0 px.** Every remaining outlier came from one
  detection at 2.02 px. Tightening to 1.0 px removed them.

**The finding that matters is the extent.** The car travelled about 13 cm during
the entire 30-frame run, and the mechanism is arithmetic: three blocked stations
reversed for 0.6 s each (1.8 s) against two forward steps of 1.0 s (2.0 s), so
net travel was roughly 0.2 s of driving. The tag range corroborates it
independently — it stayed between 0.55 and 0.64 m across seven frames spanning
stations 1 to 5, which a room traverse could not produce.

So with a high block rate the avoidance manoeuvre reverses nearly as far as the
patrol advances, and the robot explores almost nothing. That is a patrol design
problem, not a scale-recovery problem, and it is the next thing to fix.

## 6. Follow-up

**The patrol has to actually travel.** Section 7 measured the whole run inside
13 cm, because the avoidance reverse very nearly cancels the forward step
whenever the block rate is high. Until that is fixed nothing downstream can
produce a room map, however well it reconstructs. Candidate levers, none yet
tested: longer forward steps, a shorter backup, and fewer false blocks — the
detector flags chairs and people constantly at the 0.30 threshold.

**Linear speed has never been measured.** Spin rate was measured carefully while
travel speed was assumed. The wall tag makes a clean measurement possible without
a tape measure and without circularity: point the camera at the tag, record its
metric range, drive forward for a known time, record it again. The difference is
the distance travelled.

Also open:

- **One room, one run**, and now known to be one spot. ADR 0002 flagged
  low-texture rooms as this route's weak point; that case is still untried.
- Random-bounce routing was left alone deliberately. It is now testable against
  a working capture policy — but only once the car covers ground.
- The standoff gate looks redundant against the quality gate (section 5).
- `examples/17` and `examples/18` still carry the old 43.9 deg/s constant.

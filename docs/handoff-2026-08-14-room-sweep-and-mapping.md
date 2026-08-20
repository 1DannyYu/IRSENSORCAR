# Handoff — Room Sweep and Mapping (2026-08-14)

> For the next engineer or agent. The capture pipeline works end to end and is
> measured. **The immediate task is to run a full 150-frame sweep and build the
> first room map from it.** Nothing needs fixing first.

## Read First

1. [`progress/2026-08-14-travel-speed-and-coverage.md`](progress/2026-08-14-travel-speed-and-coverage.md)
   — the current state, every verified number, and nine pitfalls.
2. [`progress/2026-08-14-fused-patrol-and-sfm-overlap.md`](progress/2026-08-14-fused-patrol-and-sfm-overlap.md)
   — why the capture policy has the shape it does, and the scale-anchoring work.
3. [`progress/2026-08-14-camera-modes-exposure-and-preflight-fix.md`](progress/2026-08-14-camera-modes-exposure-and-preflight-fix.md)
   — camera configuration, the exposure sweep, and the preflight decoding fix.
4. [`adr/0002-visual-sfm-mapping-route.md`](adr/0002-visual-sfm-mapping-route.md)
   — the route decision: photograph, reconstruct offline, anchor with a known
   target rather than measuring the robot.
5. [AGENTS.md](../AGENTS.md), [CLAUDE.md](../CLAUDE.md), [CONVENTIONS.md](../CONVENTIONS.md).

`vendor/` is read-only. Motor-moving scripts run only with an operator beside the
car able to cut power. Do not commit or push unless asked.

## Source-Control State

GitHub `main` = local at `8473b16`. The Pi is synced by `tar | ssh` per file, not
by git — **check it before assuming**, e.g.
`ssh carpi 'cd ~/Car-and-Robotic-Arm && git rev-parse --short HEAD'`. Several
modules were pushed to the Pi individually during the session; a `git pull` on
the Pi is the safe way to level it.

## What Works, With Numbers

| Thing | State |
|---|---|
| Preflight ([`14`](../examples/14_all_sensors_preflight_check.py)) | 5/5 pass; power reads live bits only |
| Camera mode | One config does inference **and** 2028x1520 stills; 0.05 s per capture |
| Detection + fusion (`carbot.vision_avoid`) | Verified on hardware; labels come from the network, never a local table |
| Frame quality (`carbot.frame_quality`) | Per-tile keypoints; `repeatable_keypoints` is the pair metric |
| Spin rate | 53.5 deg/s at speed 200, dead time ~0, direction verified 14/14 |
| Travel speed | 0.117 m/s at speed 200, 0.166 at speed 400; reverse within 5% of forward |
| Patrol ([`22`](../examples/22_cam_sonar_patrol_capture.py)) | 29/30 registered in one model, 0.48 m coverage over 30 frames |
| Scale anchoring ([`anchor_sfm_scale.py`](../scripts/anchor_sfm_scale.py)) | 0.0510 m/unit, 8.2% spread, cross-validated to 4% |

## The Next Task

### 1. Run the full sweep (operator required)

```bash
ssh carpi 'cd ~/Car-and-Robotic-Arm && PYTHONPATH=src python3 examples/14_all_sensors_preflight_check.py'
ssh -t carpi 'cd ~/Car-and-Robotic-Arm && PYTHONPATH=src python3 examples/22_cam_sonar_patrol_capture.py --frames 150 --frame-report'
```

Four things decide whether the run is usable, and three of them are physical:

- **Start on open floor**, not in a corner. A cluttered start produced 0 forward
  steps in 7 stations; moving the car fixed more than any code change.
- **Keep a wall AprilTag on the route.** Without one in view the reconstruction
  cannot be anchored and coverage cannot be measured at all.
- **Stand behind or beside-rear of the car.** In the forward field of view you
  are detected as `person` and block it.
- Expect roughly 30-40 minutes and about 2.4 m of trajectory.

### 2. Reconstruct and anchor (workstation, no robot)

```bash
scp 'carpi:/tmp/room-sfm/frame-*.jpg' <images>/
uv run --extra vision --extra mapping python scripts/run_colmap_sfm.py <images> <work>
uv run --extra vision --extra mapping python scripts/anchor_sfm_scale.py <work>/sparse/<n> <images>
```

Acceptance: one model holding most of the 150 frames, and a trustworthy scale
(`spread <= 10%`, at least 3 pairs). `anchor_sfm_scale.py` exits non-zero when
the scale is not trustworthy — believe it, the first run it rejected was wrong by
a factor of 20.

### 3. Build the map (not started)

`scale.json` and `trajectory-m.csv` give metric camera positions; the sparse
points are in the same units and scale the same way. A floor map is a projection
of those points onto the ground plane. Nothing exists for this yet — it is the
first genuinely new work.

## Diagnosing a Bad Sweep

Per-frame metrics will not tell you why a reconstruction failed; that lesson cost
two sessions. Go straight to the **pairwise match matrix** — `frame_quality.repeatable_keypoints_between_files`
over consecutive frames. Strong links run 500-2000 matches, breaks show 10-70,
and where the breaks sit tells you which manoeuvre caused them.

## Known Hazards

Beyond the ones in [CLAUDE.md](../CLAUDE.md):

- **`--speed` and `--spin-speed` are separate on purpose.** 53.5 deg/s was
  calibrated at speed 200. Raising the turn speed invalidates every commanded
  angle, and nothing will warn you. Re-measure with
  [`23_cam_spin_rate_check.py`](../examples/23_cam_spin_rate_check.py) if you change it.
- **Keep `--min-standoff-cm` at or below `--obstacle-cm`.** A higher standoff
  opens a band where the car may neither photograph nor is required to move.
- **The SSD model's labels are its own 90-entry COCO-91 list.** The common
  80-class table is off by several entries and renames chairs to "toilet". Never
  reintroduce a local label table.
- **`convert_inference_coords` returns `(x, y, w, h)`**, not corners.
- **Do not name a scratch file `struct.py`** — it shadows the standard library
  and breaks `pycolmap` with a confusing trace.

## Open Questions, Ranked

1. **Does the chain hold over 150 frames?** Run 6 fell to 21/30 in two models
   once the car really moved; run 7's recovery to 29/30 is one sample.
2. **A low-texture room.** ADR 0002 named blank walls as this route's weak point
   and it has never been tried.
3. **False blocks cost coverage speed.** Three filters were falsified in turn —
   box area, box bottom, detection persistence — because a geometrically
   plausible misclassification overlaps a real obstacle in every single-frame
   quantity. Doing it properly means saving the frame at each block decision over
   several runs to build a labelled set. It was judged not worth the cost,
   because coverage speed is recoverable by running more frames. Revisit only if
   150 frames turns out not to be enough.
4. **The standoff gate looks redundant** against the quality gate; the runs that
   would settle it kept changing other variables.
5. **Routing is still random-bounce.** Whether a deliberate perimeter-then-interior
   route is needed is now testable against a working baseline.
6. `examples/17` and `examples/18` still carry the old 43.9 deg/s constant.

## A Note on Method

Four times this session a plausible fix was falsified by data that already
existed: the exposure winner did not survive a rotating camera, a sign
convention passed a test that was wrong the same way, an area threshold would
have dropped the original verified obstacle, and lengthening the step did
nothing because the car never got a clear verdict. Each was caught by checking
the proposal against a previously measured case before shipping it. The
measurements in these logs are there to be checked against — use them that way.

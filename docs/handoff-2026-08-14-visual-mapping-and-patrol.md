# Handoff — Visual Mapping and Patrol (2026-08-14)

> **Superseded by
> [`handoff-2026-08-14-room-sweep-and-mapping.md`](handoff-2026-08-14-room-sweep-and-mapping.md).**
> The fusion described below was built, verified on hardware, and has since been
> through four supervised runs. Read the newer handoff for the current state and
> the next task; keep this one for the reasoning that led here.

> For the next AI/engineer continuing this project. The immediate goal is to
> fuse the **verified** IMX500 visual obstacle detection into the patrol loop,
> so the car stops driving under chairs/tables that the single sonar cannot
> see.

## Read First

1. [`docs/progress/2026-08-14-visual-mapping-and-patrol.md`](progress/2026-08-14-visual-mapping-and-patrol.md) — today's work and every pitfall.
2. [`docs/adr/0002-visual-sfm-mapping-route.md`](adr/0002-visual-sfm-mapping-route.md) — the route decision (visual SfM, no manual extrinsics).
3. [`docs/handoff-2026-08-14-vision-to-mapping.md`](handoff-2026-08-14-vision-to-mapping.md) — earlier architecture and gates.
4. [AGENTS.md](../AGENTS.md) and [CONVENTIONS.md](../CONVENTIONS.md).

`vendor/` is read-only. Motor-moving scripts run only with an operator beside
the car able to cut power. Do not commit/push unless asked.

## Source-Control State

- GitHub `main` = local = Pi at `4ca4eae`.
- The Pi is synced (`git rev-parse HEAD` = `4ca4eae`, clean tree). Runtime
  copies are preserved in stashes (`pre-cee7f1f-pi-runtime-copies`,
  `gate-a-scp-copies`, `gate-b-tools-scp`).
- Evidence: `/tmp/pi-runtime-evidence/room-pose-wall.{json,jpg}` on the Mac;
  `/tmp/room-sfm/` on the Pi holds the last (superseded) sweep.

## What Is Already Verified

| Thing | State |
|---|---|
| SfM toolchain (`scripts/run_colmap_sfm.py`, `pycolmap`) | Works; 5/5 registration on calibration frames, camera poses export |
| **IMX500 visual obstacle detection** (`examples/20_cam_detection_check.py`) | **Works on the Pi**: open→`clear`, chair/table ahead→`OBSTACLE AHEAD` |
| Random-bounce patrol (`examples/17_cam_patrol_capture.py`) | Drives, but single sonar can't avoid thin legs/overhead → keep as reference only |
| Wall-follow patrol (`examples/18_sonar_wall_follow_capture.py`) | Same single-sonar limitation |
| Manual-push capture (`examples/16_cam_room_capture.py`) | Abandoned (user wants the car to drive itself) |

`examples/19_visual_obstacle_check.py` (edge-density heuristic) was **deleted**
— superseded by the real detector in `20`.

## The Next Task: Fuse Vision + Sonar into the Patrol

Write the fused patrol loop. Two parts:

### A. Camera-mode switching (the hard part)

> **Superseded — do not build this.** Measured in
> [`progress/2026-08-14-camera-modes-exposure-and-preflight-fix.md`](progress/2026-08-14-camera-modes-exposure-and-preflight-fix.md):
> the IMX500 delivers the inference tensor *and* 2028×1520 stills from a single
> configuration (5/5 frames, 0.05 s per capture). No mode switching is needed.
> The rest of this section is kept as the record of what was assumed.

IMX500 inference runs on a **640×480 preview stream**; SfM needs **2028×1520
stills**. One camera must do both, so the loop has to switch modes:

```
loop:
    # in preview/inference mode: read latest detection + sonar
    obstacle = visual_obstacle() or sonar_obstacle()
    if obstacle:
        back up, turn; continue
    # switch to still mode, capture a high-res frame, switch back
    capture_still_2028x1520()
    drive_forward_short_step()
```

Picamera2 pattern for switching: build two configs (preview + still), call
`picam2.switch_mode(still_config)` → `capture_file` → `picam2.switch_mode(preview_config)`,
or use `stop()`/`start(config)` around the still. The IMX500 firmware is
already uploaded, so re-start is fast. Prefer `switch_mode` first; fall back to
`stop/start` if it drops the network.

The detection code to reuse is in `examples/20_cam_detection_check.py`
(`IMX500`, `parse_detections`, and the central-lower-box obstacle test). Factor
it into `src/carbot/vision_avoid.py` (importable, testable) rather than
copy-pasting.

### B. The fusion rule (conservative)

- Obstacle if: sonar `< 30 cm` **or** sonar `None` (blind zone) **or** a
  detection with `conf >= 0.30` whose box is central + low + large (same test
  as `20`).
- On obstacle: **back up 0.6 s first, then turn** (a corner physically blocks
  in-place spin — pitfall 5).
- Turn by a random angle (30–150°) — proven to break dead corners better than
  fixed 90°.

### Acceptance for the fused patrol

- The car drives around a room containing a chair/table and **does not go under
  them**, for a full 150-frame run, operator-supervised.
- Stills are 2028×1520 and sharp (settle ≥ 1.0 s before capture).
- A room sweep then registers **substantially more than 9/40** frames in
  `run_colmap_sfm.py` (the earlier sweep was mostly blur/small overlap).

## Remaining Risks / Open Questions

1. **Battery** — `get_throttled=0x50000` was persistent. Re-run
   `examples/14_all_sensors_preflight_check.py` before motor tests; recharge/replace if it
   still fails.
2. **Spin rate** — 8.2 s/360° is only verified at speed 150. Re-measure for the
   speed the fused patrol will use.
3. **SfM scale** — the reconstruction is up-to-scale; it still needs the 70 mm
   wall AprilTag (or ChArUco 28 mm pitch) to become a real-size floor map. Not
   yet implemented.
4. **Detector confidence is low** (0.32–0.44 for chairs) — the 0.30 threshold is
   deliberately conservative. Expect occasional false "obstacle" verdicts;
   that is acceptable (better a turn than a collision).

## Suggested Order for the Next Session

1. Factor detection out of `examples/20` into `src/carbot/vision_avoid.py` +
   a unit test on a synthetic detection list (no camera needed).
2. Write the fused patrol with camera-mode switching (smallest viable loop
   first: 10 frames, then 150).
3. Dry-test obstacle logic with the car stationary, then run supervised on the
   floor.
4. Re-run `14_all_sensors_preflight_check.py` first (battery), then run the sweep, pull the
   frames, and check the SfM registration count.

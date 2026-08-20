# ADR 0002: Visual SfM Mapping Route (Photo Route)

- **Status**: adopted
- **Date**: 2026-08-14

## Context

The project builds an interior-mapping robot. The original handoff
([`docs/handoff-2026-08-14-vision-to-mapping.md`](../handoff-2026-08-14-vision-to-mapping.md))
laid out three localization approaches as its Open Decision #1:

| Option | Approach | Trade-off |
|---|---|---|
| A | Add wheel encoders + optional IMU for closed-loop odometry | Reliable, but ~1-2 days of hardware modification |
| B | Software-only incremental ICP with open-loop odometry | Verified reliable only to ~50 cm; "not recommended" |
| C | Photo route: car patrols and photographs; SfM (COLMAP) computes camera trajectories on the Mac | Fast demo; risk on feature-poor rooms |

The handoff also specified Gates B/C that required the operator to manually
measure sensor-to-chassis extrinsics with a tape measure and protractor.

## Decision

Adopt **Option C (photo route)**. The car captures overlapping stills while
moving through the room, the frames transfer to the MacBook Pro, and COLMAP
Structure-from-Motion recovers the camera trajectory and sparse 3D geometry.
The manual extrinsics measurement planned for Gate B is **dropped**.

The route switch is driven by the product intent: the car should roam, find
the wall AprilTags (IDs 0-5), and upload photos for the Mac to build the map
— not require the user to measure a large set of physical offsets by hand.

## Rationale

- **SfM recovers the camera trajectory directly**, so it does not need the
  camera's mounting offset or yaw relative to the chassis. The six extrinsics
  that Gate B would have measured are only needed to fuse ultrasonic scans
  with visual poses in the same map; the photo route does not fuse them.
- **Manual measurement is the exact friction the user rejected**, and it was
  the weak point of the ultrasonic + odometry route (its own handoff verdict:
  "without wheel encoders ... reliable to roughly 50 cm").
- **Real scale comes from a known-size target, not a tape measure.** The wall
  AprilTag (70 mm black square) and ChArUco board (28 mm pitch) are already
  measured and fixed; they anchor the up-to-scale SfM result.

## Tooling Decision

COLMAP is used via the pip-installed **`pycolmap`** wheel
(`[project.optional-dependencies] mapping`), not a system `brew install
colmap`, because the agent sandbox cannot write `~/.homebrew` and the wheel
is self-contained. `scripts/run_colmap_sfm.py` wraps feature extraction →
exhaustive matching → incremental mapping and exports camera poses.
`examples/16_cam_room_capture.py` captures the overlapping frame set on the Pi.

## Consequences

- No sensor-extrinsics tape measurement is required to start mapping.
- SfM is vulnerable to texture-poor scenes (blank walls): the first room
  validation should use a textured space (bathroom tiles/furniture).
- A roaming-capture orchestration (avoidance + stop-and-photograph + bundle
  upload) still needs to be written; the current capture is manual
  (operator pushes the car).
- The scale-anchor step (derive metres/pixel from the 70 mm tag inside the
  reconstruction) is the next software task after a room sweep reconstructs.
- `docs/setup/gate-b-manual-pose-measurement.md` and
  `examples/15_cam_gate_b_pose_log.py` remain available if a later decision needs
  the ultrasonic+vision fusion path, but are no longer on the critical path.

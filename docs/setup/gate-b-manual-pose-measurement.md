# Gate B — Manual Repositioning, Still No Motor Commands

> Repeatable procedure for Gate B of
> [`docs/handoff-2026-08-14-vision-to-mapping.md`](../handoff-2026-08-14-vision-to-mapping.md).
> The car is moved **by hand only**; no script in this gate sends a motor
> command. Gate A must be complete first (see
> [`docs/progress/2026-08-14-gate-a-software-static-camera.md`](../progress/2026-08-14-gate-a-software-static-camera.md)).

## 1. Purpose and Safety

Verify that the static camera pose is repeatable at a fixed location, that
camera displacement between locations matches tape-measured room-frame
displacement (axis and sign contract), and measure the four sensor-to-chassis
extrinsics that Gate C/D will use.

Safety: the car stays **powered off or stopped** while being repositioned.
Nobody stands in the drive path. The scripts never access motors.

## 2. Prerequisites

- Gate A committed and the Pi synced to `origin/main` (HEAD `efb5347` or newer).
- Battery charged: run `examples/other/14_all_sensors_preflight_check.py`; the power check must
  not show `get_throttled=0x50000` before any later motion gate (for Gate B
  itself the battery state does not matter — no motors run).
- The fixed wall anchor (AprilTag ID 0 + ChArUco) is unchanged and fully
  visible from every marked location.
- `ssh carpi` reachable; run everything on the Pi with
  `PYTHONPATH=src python3`.

## 3. Procedure

### 3.1 Mark 3-5 floor locations

Put tape marks on the floor, 30-60 cm from the anchor wall, spread roughly
along the wall (varying both X and Y). The camera must see the anchor from
every spot. Number them 1..N.

### 3.2 Measure the locations with a tape

For each consecutive pair (1->2, 2->3, ...), record:

| Pair | dX (cm, away from wall, + = away) | dY (cm, along wall to the right, + = right) |
|---|---|---|
| 1 -> 2 | | |
| 2 -> 3 | | |
| ... | | |

Use a tape from the anchor wall for X and a parallel offset for Y, or measure
the straight-line distance and direction and decompose.

### 3.3 Log camera poses per location

```bash
cd ~/Car-and-Robotic-Arm
PYTHONPATH=src python3 examples/ai_camera/15_cam_gate_b_pose_log.py --positions 5 --anchor-height-cm 14.65
```

The script waits at each location: place the (stopped) car on mark *i*, press
Enter, it captures 5 frames, processes them, and reports inliers and spread.
Continue until all locations are logged.

### 3.4 Compare displacement

The script prints per-pair camera displacement in world axes
(dX away from wall, dY wall-right). Compare signs and magnitudes with the
table from §3.2:

- Gate B acceptance: repeated-position spread < 1 cm per axis (fully visible
  anchor); measured displacement direction matches the printed axis signs.

### 3.5 Measure the four extrinsics (tape + protractor)

From a top view with the car on the floor:

1. **Chassis rotation centre**: intersection of the two wheel-base
   diagonals; mark it on the chassis.
2. **Camera optical centre**: lens centre of the Pi AI Camera.
3. **HC-SR04 origin**: centre of the two ultrasonic transducers (the
   measurement origin is roughly the emitter centre).

Record, with uncertainty (±):

| Extrinsic | Value | Unit | ± |
|---|---|---|---|
| camera offset x (right) | | cm | |
| camera offset y (forward) | | cm | |
| camera yaw vs chassis forward (+, CCW/left) | | deg | |
| sonar offset x (right) | | cm | |
| sonar offset y (forward) | | cm | |
| sonar yaw vs chassis forward (+, CCW/left) | | deg | |

Definitions match `src/carbot/frames.py` `SensorExtrinsics`:
offset in the chassis frame (x right, y forward), yaw = sensor forward vs
chassis forward, 0 = aligned, positive = counter-clockwise (left turn).

### 3.6 Record results

Put the numbers from §3.2/§3.5 and the script's JSON
(`/tmp/gate-b-poses.json`) into a new progress log
`docs/progress/YYYY-MM-DD-gate-b-manual-pose.md`, together with the 13/15
console output and the accepted vs rejected frames, then commit.

## 4. Acceptance Checklist

- [ ] ≥ 3 valid locations logged; each with ≥ 3 inliers and spread < 1 cm per axis.
- [ ] Displacement signs match the tape table for every consecutive pair.
- [ ] All six extrinsics recorded with uncertainty in the progress log.
- [ ] Moving the car made the old `/tmp/room-pose*.json` visibly stale
      (script prints a warning when < 2 valid locations; the JSON timestamp
      also changes).
- [ ] No motor command was sent; the Pi log shows only camera/sensor output.

## 5. Files

- `examples/ai_camera/15_cam_gate_b_pose_log.py` — live or `--input-dir` pose logging.
- `src/carbot/frames.py` — `SensorExtrinsics` contract the values fill in.
- `examples/ai_camera/13_cam_room_pose.py` — single-location static pose (source of the
  per-location logic).

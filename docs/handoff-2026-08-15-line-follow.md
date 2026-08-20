**Superseded by** [`docs/handoff-2026-08-16-line-follow.md`](handoff-2026-08-16-line-follow.md)
(2026-08-16). Do not follow the next-steps in this file.

# Handoff — 2026-08-15 Line-Following: pass to next engineer

> **Read first**: work log [`docs/progress/2026-08-15-line-follow.md`](progress/2026-08-15-line-follow.md)
> (facts, measurements, problems) · stable hardware [`docs/hardware/ai-camera.md`](hardware/ai-camera.md)
> and [`docs/hardware/nezha-integration-notes.md`](hardware/nezha-integration-notes.md) ·
> bring-up [`docs/setup/raspberry-pi-first-run.md`](setup/raspberry-pi-first-run.md) ·
> remote-access [`docs/setup/mac-to-raspberry-pi-access.md`](setup/mac-to-raspberry-pi-access.md).

Supersedes: nothing (first handoff on this topic). The 2026-08-15 progress log is
the live record for this session.

---

## 1. Where the project stands

**Goal**: the car autonomously traces the black line on the Yahboom track map
(mod-100): start zone → right turn → outer loop → roundabout (one anti-clockwise
lap) → continue → exit → back to the start zone.

**What works (verified on the Pi, no motors or dry-run only):**

- IMX500 camera works after the Pi reboot; 2028x1520 preview stream at ~17 fps.
- `carbot.line_follow.detect_line` — pure line detector: ROI crop, threshold,
  row-to-row segment tracking, main-line + branch (junction) reporting,
  candidate lines for target locking. Unit-tested (18 tests).
- `carbot.line_nav.LineNav` — pure state machine: FOLLOW (proportional steering
  with target locking + release), SEARCH (spin to re-acquire), ROUNDABOUT
  (fork + elapsed-lap-time double confirmation; lap time anchored to the
  verified 53.5 deg/s @ speed 200). Unit-tested (17 tests).
- `examples/25_cam_line_follow_capture.py` — one frame + annotated overlay, no
  motors, safe over SSH.
- `examples/26_cam_line_follow_drive.py` — closed loop; fixed exposure
  (50 ms / gain 4.5); dry-run verified on the Pi (motors never energised).
- Full suite: 250 tests passed before the afternoon changes; line-follow tests
  alone 35 passed; ruff clean.

**What does NOT work yet (the handoff point):**

- **On real supervised runs the car does not follow the line.** It first veered
  (detector main-line flips), then spun in small circles (locked target stays at a
  constant frame position), then drove straight off the map (weak steering gain).
  The steering-gain and locking fixes are in the code but the circling behaviour
  has **not been re-verified** after the last code change (`turn_gain=2.5`,
  `_LOCK_RELEASE_GAP=0.15`).

## 2. The blocker: we have not confirmed the detector tracks the real line

The user (human, at the robot) says the real black line is "the darkest and
thickest one, about 2 cm wide". The detector's main line jumped between
x = 302 / 660 / 1233 / 1678 (of 2028) on different parkings — far more than
parking repeatability explains. The car circles with a nearly constant locked
target, which is the signature of chasing a **fixed dark structure in the frame**
(chassis/support visible to the camera, or an environmental shadow), not the
line. **No one has visually confirmed that the green cross (detected main line)
sits on the real black line.** This agent has no vision; it tried to ask the user
to confirm via annotated images, which the user could not interpret reliably.

**Do this first, before any more tuning:**

1. Park the car on the start zone, nose along the black line (as the user did).
2. Capture: `PYTHONPATH=src python3 examples/25_cam_line_follow_capture.py --output /tmp/line-follow`
   → view `/tmp/line-follow/line-follow-overlay.jpg` (on the Pi) or pull it to a
   machine with a screen.
3. Check the green cross (main line) is on the 2 cm black line. If it is on a
   different dark structure, the fix is in `line_follow.py` line selection
   (currently: most persistent tracked line wins) and/or `dark_threshold`,
   `roi_top`/`roi_bottom` (the ROI may include chassis/environment).
4. If the cross IS on the line, read its x position → that is the correct
   `expected_center_fraction` (currently hard-coded 0.571, which was measured at
   one particular parking). Re-run and confirm the car drives straight when the
   line is centred.

Screenshots the user can provide are fine; the user is responsive and at the
robot. Ask the user to open the overlay URL (files placed in the repo root are
served by the local server at `http://127.0.0.1:4321/Car-and-Robotic-Arm/<file>`)
or pull the file with `scp`.

## 3. Current runtime state

- **Pi**: `ssh carpi` works (`danny-raspberrypi5-...local`, user `dannypi`).
  Repo at `~/Car-and-Robotic-Arm`. The four new files
  (`src/carbot/line_follow.py`, `src/carbot/line_nav.py`,
  `examples/25_...`, `examples/26_...`) and the two test files are **already
  synced to the Pi** (scp). The progress log is NOT on the Pi.
- **Mac**: working tree at `/Volumes/KLEVV-4T-1/Danny/Car-and-Robotic-Arm`.
  All new files are untracked (`??` in `git status`) — nothing committed (per
  project rule, nothing commits without the user asking).
- **Unrelated worktree changes present** (not touched by this session, leave
  alone): `site/src/components/InventoryList.astro`,
  `site/src/i18n/ui.ts`, `site/src/components/Pi5PinoutCard.astro`,
  `site/src/data/pi5-pinout.ts`.
- **Runtime artifacts** on the Pi: `/tmp/line-follow/` (captures, annotated
  frames); on the Mac: `/tmp/car-vision/`. Raw/private captures belong under
  `scratch/` per CONVENTIONS if they need to be kept; none are committed.
- Local preview server runs on port 4321 (serves repo root under
  `/Car-and-Robotic-Arm/`), useful for showing the user images.

## 4. Key measurements the next engineer needs

| Item | Value | Where |
|---|---|---|
| Map paper background gray | ≈208 | progress log |
| Line width | ≈2 cm ≈ 115 px @ 2028 | progress log, user-confirmed |
| `dark_threshold` | 100 (90-120 all separated the first still) | `LinePolicy` |
| ROI | y 10%-68% of frame (excludes top shadows, bottom chassis) | `LinePolicy` |
| Auto-exposure drift while moving | mean ~195 → ~118 | progress log problem 3 |
| Fixed exposure | 50 ms / AnalogueGain 4.5 → mean 204, dark 5.5 % | `examples/26` defaults |
| Spin rate | 53.5 deg/s @ speed 200 (examples/23) | line_nav docstring |
| Steering gain | 2.5 (0.45 was far too weak) | `NavPolicy` |
| `expected_center_fraction` | 0.571 (unstable — see blocker) | `NavPolicy` |
| Car speed @ speed 200 | 0.117 m/s forward | examples/22 |

## 5. Immediate next steps (in order)

1. **Confirm the tracked line is the real line** (section 2). This unblocks
   everything else; do not tune without it.
2. Re-establish `expected_center_fraction` from a confirmed frame; if the user's
   parking varies, consider measuring it each session, or make steering tolerate
   a ±0.1 error instead of relying on the exact value.
3. Re-run the closed loop supervised and watch whether the circling is gone
   (`--start-turn-s 0`, fixed exposure already default). Collect
   `--save-every` frames and inspect the locked target across frames: if it is a
   fixed frame position while the car rotates, the detector is locked to a
   chassis/shadow feature — exclude that region (tighter ROI) or reject
   candidates whose frame position is invariant to the car's rotation.
4. Only after the car reliably follows straight and curved line: tune
   roundabout entry/exit (junction width factor, lap time) and run the full
   route, keeping the car on the map (the room is small; walls/wardrobes/
   mirrors sit right outside the map).
5. Commit the work once the user asks. Follow CONVENTIONS: update this handoff's
   "superseded by" note when a newer handoff replaces it.

## 6. Safety rules for the next engineer

- Motor/servo programs only when a person is physically beside the robot with
  power within reach (AGENTS.md). Over SSH that means the operator at the robot;
  wheels lifted for the first smoke test.
- Run `examples/14_all_sensors_preflight_check.py` before any supervised motion test.
- `examples/26_cam_line_follow_drive.py` prompts for operator confirmation; use
  `--dry-run` first. Do not let the car drive off the map unsupervised.
- `vendor/` is read-only; I2C at 0x40, bus ≤200 kHz; keep the 500 ms init and
  100 ms reset delays (see `docs/hardware/nezha-i2c-protocol.md`).

## 7. What changed in code (quick map)

- `src/carbot/line_follow.py`: `LinePolicy`, `LineReading` (visible, error,
  width, junction, `candidate_centroids`), `detect_line`, `_track_segments`.
- `src/carbot/line_nav.py`: `NavPolicy` (speed, turn_gain, expected_center_fraction,
  junction_width_factor, roundabout timings), `LineNav` (FOLLOW/SEARCH/ROUNDABOUT,
  target locking with `_LOCK_RELEASE_GAP=0.15`, baseline-width junction
  confirmation), `steer_command`.
- `examples/25_cam_line_follow_capture.py`, `examples/26_cam_line_follow_drive.py`:
  capture/overlay and closed-loop drive respectively.

# Map 1 staged phase controller and safety corrections

## Scope and result

Introduced an explicit Phase 1-10 model and a bounded physical phase-test path before attempting
another integrated lap. The ten phase distances and headings now live in
`src/carbot/map1_phases.py`; `examples/40_map1_ir_phase_test.py` delegates to Example 39's same
production control path so staged tests cannot silently diverge from the integrated program.

The review of `map1_run_20260821_025231.log` identified four concrete downstream-control faults:

- full-speed distance was credited even while the wheel command was slowed or asymmetric;
- sustained `0000` could count as an arc and the 20cm arc trigger could accept the roundabout
  entry on an unrelated reading (the log showed acceptance on `P0001`);
- SEARCH pivot commands were recorded into reverse history, so replay undid the search pendulum;
- a junction-turn timeout returned a forward command instead of a latched safety stop.

Those faults are corrected. Distance credit now scales from the issued left/right command;
`0000`, junction, and noise readings do not establish a phase transition; arc distance only opens
the real approach matcher and never accepts a junction by itself; reverse history contains only
forward-progress commands; turn/search failure latches zero wheel speeds.

The Phase 1-to-2 handoff no longer drives blindly for up to three seconds. It holds the wheels at
zero and requires 0.10s of stable `ON_LINE`/`DRIFT` input within a bounded timeout. Failure exits
for operator repositioning or sensor calibration.

No motor-moving program was run during this development session. The local `Map1-North.JPG`
deletion and `tasks/ir-sensor-tracking/route-planning.html` redesign were initially preserved
outside the controller commit, then reviewed and committed separately after explicit operator
authorization.

## Work completed today

- Rectified the Map 1 source image into a top-down 1000x700 reference and restored the Phase 1
  stem specification to 16.0cm after a temporary, incorrect 10cm edit.
- Added the English interactive route-planning page covering all ten phases, their distances,
  headings, arcs, and the 270-degree roundabout traversal. The red arrowhead went through two
  visual corrections: the first size obscured labels and the next was too small to see. Its current
  checked-in presentation is intended to remain visible without covering route text.
- Made Phase 1 a genuinely sensor-blind operation: forward through the start-stem T junction,
  stop, then spin right in place. Passive IR samples in its log are observations only.
- Added timestamped output and bounded 20-second evidence windows so a physical test can be
  reconstructed without producing another unbounded log.
- Introduced `examples/40_map1_ir_phase_test.py`, allowing Phases 1-10 to be placed and tested
  independently while still calling the same implementation used by Example 39.
- Audited and corrected downstream distance accounting, phase-transition matching, reverse replay,
  line-search history, and timeout stopping behavior before permitting an integrated lap.
- Reworked timed movement to refresh the active motor command at 100Hz and log sampled writes. This
  was useful instrumentation even though it did not turn out to be the physical cause of the buzz.
- Calibrated the right spin from the observed 85 degrees to approximately 2.68s at PWM 150. A later
  physical check still found the turn short, so the 90-degree route target now uses a 5-degree
  pulse compensation, approximately 2.80s total, pending another isolated Phase 1 measurement.

## Chronology of pitfalls and lessons

1. **The route source briefly disagreed with the physical specification.** Phase 1 was changed from
   16cm to 10cm while trying to align the T-junction turn, then restored to the operator-specified
   16cm. Geometry and control calibration must not be conflated: the target remains 16cm even when
   a timed pulse falls short.
2. **The T-junction workaround was not initially isolated enough.** General sensor recovery could
   still react to `0000` near the start. Phase 1 now has a dedicated open-loop path, and its sensor
   readings cannot branch, stop, or alter either motor command.
3. **A timer log was mistaken for proof of wheel motion.** The 4.0s run reported a four-second
   forward interval, but the wheels visibly moved for only about one second and then the motors
   buzzed. Elapsed Python time proves only that the control function remained alive; it does not
   prove physical displacement on a chassis without encoders.
4. **The missing evidence encouraged the wrong hypothesis.** Because the old timed move wrote one
   motor command and then slept, the first logs could not show whether another command overwrote it.
   The refresh/audit loop now records repeated non-zero L/R commands and passive IR snapshots.
5. **`0000` correlated with the stop but did not cause it.** The instrumented run showed `P0000`
   around 1.23s while L150/R150 writes continued through 1.60s. Correlation at the edge of the black
   paper was initially treated as sensor interference; the command trace disproved that theory.
6. **Increasing time did not address the actual failure.** The forward pulse was tried at 1.6s,
   2.5s, and 4.0s. Longer commands mostly extended the audible buzz because the chassis was already
   mechanically caught; time and PWM changes cannot repair a blocked underside.
7. **The apparent distance produced an invalid linear calibration.** A roughly 7cm obstructed run
   was scaled to `1.6s * 16cm / 7cm = 3.66s`. After finding the raised paper under the chassis, that
   result and commit were explicitly rejected. The safe baseline returned to PWM 150 / 1.6s for a
   fresh measurement on a flat, secured surface; that later unobstructed run still fell short, so
   the next bounded operator-requested trial is 2.2s without changing PWM.
8. **The real fault was outside the software stack.** Raised paper caught the bottom of the car;
   the wheels did not rotate while the energised motors buzzed. Before changing code after a stall,
   inspect the complete travel surface, paper seams, chassis clearance, wheel clearance, and cable
   drag while power is off.
9. **Python memory and Pi power were plausible questions but unsupported by evidence.** There was no
   OOM, killed process, segfault, I2C error, undervoltage flag, competing navigation process, or
   enabled robot service. The Pi had about 3.5GiB available, zero swap use, 5.016V external supply,
   and `get_throttled=0x0`. These checks ruled out resource management without guessing.
10. **The right turn needed its own calibration.** Forward travel and stationary spin are different
    motions and must not share inferred rates. The first spin was about 85 degrees. Although the
    first corrected test appeared to reach 90 degrees, a later physical check found it short; the
    desired route geometry remains 90 degrees while the pulse receives 5 degrees of compensation.
11. **A full Phase 1-10 test was requested before isolated phases were trustworthy.** The safer
    workflow is independent placement and testing of every phase through Example 40, then integration
    into Example 39 only after each phase has measured entry pose, exit pose, distance, and log.
12. **Several downstream transitions could falsely advance the route.** `0000` could be counted as
    an arc, the roundabout approach could accept an unrelated `P0001`, SEARCH pivots contaminated
    reverse history, and a turn timeout resumed forward motion. All now require valid evidence or
    latch a stopped state.
13. **The original free-running loop created unusable evidence.** One run produced 6.2 million
    frames and a 608MB log in 148s. Tests now default to a 100Hz loop, sampled logging, heartbeat
    messages, explicit duration, and timestamped files.
14. **The local default Python environment is not a reliable full-suite runtime.** Its Python 3.15
    alpha interpreter is binary-incompatible with the installed NumPy build. Focused IR tests pass
    locally; full validation uses an isolated Python 3.13 environment.
15. **Deployment state must be explicit.** A local edit is not present on the Raspberry Pi until it
    is committed, pushed, and pulled with a fast-forward update. Each physical-test instruction must
    identify the expected commit/parameters before the operator starts the motors.

## Verification

Focused state-machine, phase-model, route, and Example 39 regression tests:

```text
uv run python -m pytest -q \
  tests/test_car.py \
  tests/test_map1_phases.py \
  tests/test_example_39_hardcoded_start.py \
  tests/test_ir_line_nav.py \
  tests/test_ir_route.py

139 passed
```

Full repository test suite in a clean compatible runtime:

```text
uv run --isolated --python 3.13 --all-extras --group dev python -m pytest -q

572 passed in 91.61s
```

Static checks:

```text
uv run ruff check src/carbot/car.py src/carbot/map1_phases.py src/carbot/ir_line_nav.py \
  src/carbot/ir_route.py examples/39_map1_ir_line_follow.py \
  examples/40_map1_ir_phase_test.py tests/test_map1_phases.py \
  tests/test_car.py tests/test_ir_line_nav.py tests/test_example_39_hardcoded_start.py

All checks passed
```

## Measurements and configuration

- Phase 1 forward target: 16.0cm.
- A supervised Phase 1 run at 04:50 used PWM 150 / 4.0s. The car moved for about the first
  second, then the wheels stopped while the motors remained audibly energised for the remaining
  three seconds. The spin began only after the full forward timer and produced about 85 degrees.
- Phase 1 had no sensor-dependent branch, but the original log did not audit the actual motor
  writes during the sleep. Extending the pulse from 1.6 to 2.5 to 4.0 seconds did not add travel.
- The instrumented PWM 150 / 1.6s run sent 117 non-zero forward commands. Passive IR changed from
  `P1111` to `P0000` around 1.23s, but L150/R150 continued through 1.60s, proving `0000` did not
  stop or alter Phase 1.
- The operator then found the actual cause of the audible no-rotation interval: raised paper caught
  underneath the chassis. The apparent 7cm measurement was obstructed and cannot calibrate speed.
  The temporary 3.66s extrapolation is rejected. After flattening the paper, the restored PWM 150 /
  1.6s trial still travelled less than 16cm, so the next isolated Phase 1 test uses 2.2s at the
  operator's request. This remains an open-loop trial, not a measured speed calibration.
  `Car.move_for` reasserts the same non-zero command every 0.01s, matching the working 100Hz
  navigation loop. Example 39 logs every tenth write together with a passive IR reading marked
  `OBSERVE ONLY`. This makes the next run distinguish
  an overwritten command from a board-side PWM/H-bridge condition without letting IR control it.
- Corrected Phase 1 spin model: 39.7deg/s plus 0.41s dead time. The 90-degree route target now uses
  a 95-degree open-loop pulse after the operator reported the turn was still short, giving about
  2.80s at PWM 150. This compensation requires another isolated physical measurement.
- `Car.move_for` now reasserts its active command every 0.01s, so a one-shot external write or board
  reset cannot silently replace a multi-second timed command until its deadline.
- ARC 1/2/3: 12.0cm each.
- Phase 2/4/6/8/10: 15.5 / 18.0 / 47.0 / 7.5 / 21.5cm.
- Roundabout 270-degree estimate: 78.93cm from the measured 33.5cm inner black-line diameter.
- Independent arc/roundabout tests use 60% of the requested base PWM and scale distance credit
  from that actual command.

These remain open-loop distance estimates; the chassis has no encoder feedback.

## Problems encountered

The Phase 1 buzz was checked against the Pi and vendor evidence before changing motor power:

- At the 04:50 run there was one SSH session and no second car/navigation Python process, tmux,
  screen, cron job, or enabled robot service. Only that run's timestamped log changed.
- The kernel and system journals contain no OOM, killed process, segfault, I2C, or undervoltage
  event for the run window. A later read showed about 3.5GiB memory available, zero swap use,
  `EXT5V_V=5.016V`, and `get_throttled=0x0`; Python memory management is not a plausible cause.
- The 25-page vendor manual states that motor PWM is fixed at 100Hz and duty is value / 1000, but
  documents no one-second command timeout or IR-dependent motor behavior. The IR board is connected
  to Pi GPIO, not the NeZha motor controller.
- The concrete software difference is that normal line navigation reissues `car.drive` at about
  100Hz, while the old Phase 1 wrote L150/R150 once and slept. The diagnostic fix makes timed moves
  use the same refresh pattern while retaining PWM 150.

The repository's existing `.venv` uses a Python 3.15 alpha interpreter with an incompatible NumPy
binary, so the ordinary full-suite collection fails in unrelated vision tests. The suite was run
successfully in an isolated Python 3.13 environment instead. Focused IR tests also pass in the
normal project environment.

Existing synthetic route helpers directly seed phase-transition and arc-distance internals. New
tests therefore cover the command-aware distance estimator and the specific false-trigger and
safety-latch regressions, but recorded real sensor streams are still needed for end-to-end replay.

## Follow-up and next safe gate

1. Flatten and secure the paper, then run only Phase 1 with a ruler; verify PWM 150 / 2.2s reaches
   16cm and that the ~2.80s compensated spin physically reaches the desired 90-degree heading.
2. Place the car manually at Phase 2's entry pose and run only Phase 2 through Example 40 with a
   timestamped log.
3. Do not run Phase 1-10 as one route until each numbered phase passes independently.
4. Store future raw per-cycle sensor/command captures under `scratch/ir-sensor-tracking/` and add
   sanitized recorded traces as automated replay fixtures when their physical context is known.

## Phase 2/3 independent-test correction

Later Phase 2 and Phase 3 tests exposed two independent runner defects:

- Phase 2's independent mode set `start_on_loop=True`, which bypassed the acquisition gate. The
  05:53:31 log started directly in FOLLOW on `P0001`, repeatedly alternated `L150/R20` with SEARCH,
  entered SEARCH 27 times, and credited the full 15.5cm without ever reaching centred `P0110`.
  Independent Phase 2 now performs an active in-place acquisition, pivots toward a visible line,
  carries that direction through adjacent `P0000`, and requires stable `P0110` before resetting the
  controller and starting its distance counter.
- Independent ARC/roundabout tests silently scaled the requested speed by 0.6. With `--speed 150`,
  Phase 3 therefore issued only `L90/R90` when centred and as little as `L90/R12` at `P0001`; the
  operator heard motor buzz without chassis motion. The hidden scale is removed. Curve steering
  still slows the inside wheel, but the outside/base wheel retains the requested PWM 150.

Neither correction was physically executed during development. The next gate is another isolated
Phase 2 test followed by Phase 3 only if Phase 2 visibly centres and completes successfully.

### Phase 3 full-PWM escape and curve-pattern correction

The subsequent 06:08:04 Phase 3 test proved that restoring the requested PWM fixed the buzz but
exposed a separate steering defect. The log began on `P1000` with a hard left correction, reached
centred `P0110`, then changed to `P0111` at 1.7s. Because `P0111` is classified as junction-shaped,
the generic route safety rule held the previous `L150/R150` command. The car therefore drove
straight off the map until the open-loop phase-distance estimate reached 12cm about 0.5s later.

A new policy flag enables directional junction-shaped patterns only for bounded independent
ARC/roundabout tests. In that mode `P0111/P0011` steer right and `P1110/P1100` steer left using
their existing geometry-table ratios; symmetric `P1111` still holds the previous command. Straight
phase tests and the integrated route keep the original sequence-confirmed junction safety rule.
No motor-moving command was run while implementing this correction.

### Phase 3 midpoint stop and ARC distance calibration

The next 06:15:18 Phase 3 run followed the line instead of escaping. Its sequence was `P0110`
straight, `P0100` left correction, `P0000` left blind-band correction, and `P1000` with an active
`L20/R150` hard-left command. It stopped at 2.1s because `Map1PhaseProgress` reached its estimated
12cm boundary and deliberately emitted `STOP-AFTER-PHASE 3`; `P1000` did not stop or overwrite the
motors. The operator observed that the chassis was only about halfway around the physical ARC 1.

The command-distance estimate was derived from Phase 1's straight-line timing and over-credited
curve progress by approximately two times. Independent ARC tests now retain full base PWM 150 but
apply a provisional 0.50 distance-credit scale. This should move the Phase 3 stop from roughly the
midpoint toward the physical 12cm exit while keeping the newly verified curve steering active.
The scale is intentionally limited to bounded independent ARC tests until another measured run
confirms it; roundabout and integrated-route distance behavior are unchanged.

### ARC 1/2/3 raised-paper noise and left-only steering guard

The 06:21:04 retest ran materially farther with the 0.50 ARC distance-credit scale, but still left
the printed route. The log contained right-side `P0010`, `P0001`, `P0011`, and `P0111` readings and
therefore issued right corrections such as `L150/R20` and `L150/R60`. The operator then supplied
the missing physical context: the black detections on the car's right were caused by raised map
paper, not by ARC 1 moving to the right. ARC 1 changes heading east to north and must not issue a
right-turn command.

Independent Phase 3 initially applied a left-only steering guard. Right-side DRIFT and curve-shaped
patterns are logged as opposite-side noise, hold the last safe straight/left command, and do not
replace `_last_localising`; a following `P0000` is therefore resolved from the last geometrically
valid reading. If the line is genuinely lost, Phase 3 performs an in-place left-only search and
stops at the configured sweep-angle ceiling if no valid line returns. It never performs the normal
pendulum's right-hand return sweep.

The same rule now comes from the phase specification and applies to all three bounded ARC tests:
ARC 1 is east-to-north, ARC 2 is north-to-west, and ARC 3 is west-to-southwest, so each is a mapped
left curve. Phases 3, 5, and 7 therefore share the same left-only controller; straight phases and
the roundabout remain unrestricted. The extension to ARC 2/3 is based on map geometry and must
still be confirmed by their individual hardware runs.

The 06:39:57 Phase 3 retest proved the direction guard removed every right-turn command, but also
exposed a second-order error in the meaning of "ignore." From 3.3s through the 4.1s phase stop,
right-side patterns were correctly rejected; however, the immediately preceding accepted command
was frequently centred `L150/R150`. Generic noise hold therefore kept driving straight for roughly
0.6s through a left curve, and the chassis still left the route. The rejected signal did not turn
the car right; preserving a stale straight command caused the escape.

The one-way guard now remembers the latest permitted left correction across intervening centred
frames. Opposite-side noise reuses that correction instead of generic hold. If an ARC begins with
only centred frames and no left correction has yet been observed, the fallback is the slight-left
geometry state (`L110/R150` at speed 150), never straight and never right. This behavior applies to
bounded Phases 3, 5, and 7.

The 06:46:27 retest confirmed that correction: right-side noise maintained `L60/R150`, with no
SEARCH, REVERSE, or right-turn command. The bounded runner nevertheless emitted
`STOP-AFTER-PHASE 3` at 4.4s when its scaled `pc` reached 12cm. The chassis was then pointing
northeast instead of the required north, approximately half of the intended 90-degree heading
change. That observation initially led to a provisional 0.25 distance-credit trial.

The 06:51:23 trial disproved that diagnosis. The runner spent its first 1.0s on centred `P0110`
with `L150/R150`, and returned to equal-wheel straight commands each time the sensor recentered.
This cannot follow a physical arc: it follows the tangent until the reactive controller sees a
large error, then corrects sharply. At 4.5s the reading became sustained `P1111`; the chassis had
already left the valid 2cm line/paper region. Off-track reverse-replay began at 6.5s, but the 0.25
boundary kept the invalid run alive until 10.1s. Extending the time was therefore unsafe and did
not complete the ARC.

The 0.25 trial is reverted to the previous 0.50 safety boundary. The controller now applies the
slight curve-side state while centred on bounded ARC 1/2/3 (`L110/R150` at speed 150), providing
continuous left curvature from the phase start instead of treating `P0110` as a straight segment.
Straight phases still use `L150/R150` on `P0110`. This is the next isolated hardware gate; Phase 4
must not be attempted until Phase 3 stops on the line with a north heading.

The operator additionally confirmed that off-map carpet reads `P1111` and that the 06:51 recovery
did not reverse far enough to find the black line. The log explains why: `P1111` began at 4.5s, the
generic 2.0s dwell delayed reverse until 6.5s, and replaying only the previous 2.0s could at best
return the sensor to the paper edge. At 8.5s a right-side `P0111` then ended reverse prematurely,
even though that pattern is rejected as raised-paper/edge noise by the bounded left-ARC controller.

Bounded ARC tests now trigger reverse after 0.3s of continuous `P1111`/`P0000` and retain 3.0s of
command history. During reverse, opposite-side patterns and non-contiguous NOISE no longer count as
reacquisition; reverse continues until centred `P0110`, a valid left-side line reading, or an
allowed left curve-shaped pattern appears. The ordinary full-route defaults remain 2.0s/2.0s.

The 07:00:18 retest validated continuous left curvature: Phase 3 issued no straight or right-turn
command and remained on the route, with no SEARCH or REVERSE event. It nevertheless stopped at
4.3s when the x0.50 progress estimate reached 12cm, while the chassis still pointed northeast.
Because the intended ARC changes the heading from east to north, this is approximately half of the
required turn. The bounded ARC distance-credit scale is therefore x0.25 again, now with the
centred-on-curve feed-forward fix present. This should make the boundary about 8.6s under a similar
command trace; hardware must confirm that it stops on-line with a north heading before Phase 4.

The 07:03:01 x0.25 run exposed a more fundamental test-setup error. The operator cannot place the
car precisely at the Phase 2/3 interface and instead starts at any centred `P0110` point on the
eastbound Phase 2 straight. The old independent Phase 3 runner nevertheless enabled left-only ARC
control on its first frame. Its repeated `P0110 -> P0010 -> P0000` departures therefore produced
six forward/reverse cycles in 12 seconds; this was Phase 3 control being applied prematurely on
Phase 2, not a Phase 2 line-follow failure.

Independent Phase 3 now begins with ordinary unrestricted Phase 2 line following. It recognises
the measured leftward ARC-entry progression `P0100 -> P0000 -> P1000`, or the direct left-pair
curve shapes `P1100/P1110`, before creating a clean left-only ARC controller. Right-side signals
cannot activate the transition because raised paper produces false black there. The lead-in has a
five-second ceiling and stops safely if ARC 1 is not confirmed.

The operator also rejected command-derived distance as the Phase 3 stop gate. With no wheel
encoders, PWM/time credit is not measured physical distance and repeatedly stopped the chassis
mid-ARC. The independent Phase 3 run therefore performs no distance accumulation and has no 12cm
stop. Logs print `pc=disabled` so this behavior cannot be confused with an older Pi copy.

The intended test envelope was then clarified as Phase 2 tail -> all of Phase 3 -> the opening of
Phase 4, not "hold left until 20 seconds." Phase 3 now requires real left-curve evidence after its
lead-in, then 0.5s of continuous centred `P0110` before switching from the left-only ARC policy to
ordinary unrestricted Phase 4 line following. It must then accumulate 2.0s of valid ON_LINE/DRIFT
FOLLOW; SEARCH, REVERSE, or a non-localising reading resets that proof interval. Completion sends
an explicit zero-wheel STOPPED command. The 20-second duration remains the ceiling for the entire
test, including acquisition and Phase 2 lead-in, and timeout is not a passing result.

Validation after this change:

- `uv run --group dev python -m pytest -q tests/test_ir_line_nav.py` plus the Example 39 and
  phase-model tests: 106 passed;
- `uv run --isolated --python 3.13 --group dev --extra vision python -m pytest -q`: 579 passed;
- Ruff, Python bytecode compilation, and `git diff --check`: passed.

The default project `.venv` currently points at Python 3.15 alpha and its installed NumPy wheel
fails to import with `unknown slot ID 85`. A first full-suite attempt therefore failed during test
collection, and a second isolated attempt without the `vision` extra reached 451 passed but left
eight landmark tests without `cv2`. The successful command above deliberately used stable Python
3.13 plus the declared `vision` extra; no dependency or virtual-environment files were changed.

## README operator workflow

The root `README.md` now documents the complete repeatable Map 1 workflow used during this session:

- which source file controls phase geometry, Phase 1 calibration, shared navigation, tests,
  operator documentation, route visualization, and ignored raw logs;
- local inspection, focused validation, exact-file staging, commit, and push through the `danny`
  remote to `https://github.com/1DannyYu/IRSENSORCAR.git`;
- a guarded Pi `git pull --ff-only` that refuses to overwrite uncommitted tracked changes;
- one-variable (`PHASE=N`) independent testing for Phases 1-10 with timestamped 20-second logs,
  entry-pose requirements, Phase 2 acquisition behavior, and curve-test PWM behavior;
- commands to list, read, and copy test logs back to the Mac.

The README's stale Pi working directory and clone URL were also corrected to
`~/Car-and-Robotic-Arm` and the current `1DannyYu/IRSENSORCAR` repository. No motor-moving command
was run while creating or verifying the documentation.

## End-of-session record

Work stopped for the day after preparing, but not physically running, the complete independent
Phase 3 transition test. The required test envelope is now explicit:

1. place the car at any stable `P0110` point on the Phase 2 eastbound straight, heading east;
2. follow Phase 2 normally and detect the real ARC 1 entry;
3. run the left-only Phase 3 controller without a command-distance stop;
4. after observed left-curve evidence, require 0.5s of centred `P0110` to enter Phase 4;
5. follow Phase 4 normally for 2.0s of uninterrupted ON_LINE/DRIFT evidence, then stop;
6. treat the 20-second whole-test timeout as failure, not completion.

The last physical run remains
`scratch/ir-sensor-tracking/2026-08-21-070301-phase03-quarter-credit-12s.log`. It proved that the
previous runner applied Phase 3 left-only control while the chassis was still on Phase 2, causing
six forward/reverse cycles. No physical run has yet validated the new Phase 2 -> 3 -> 4 state
machine. The next session must begin with that isolated test; Phases 4-10 and the integrated route
remain blocked on its result.

Final software verification for the staged transition controller:

- focused navigation/phase tests: 111 passed;
- full stable-Python test suite: 584 passed in 92.65s;
- Ruff, bytecode compilation, and `git diff --check`: passed;
- deployed code before this closeout record: commit `ba9deea` on GitHub and Raspberry Pi 5;
- no motor command was run by the development session while implementing or deploying it.

Git history was unexpectedly replaced twice by an external "Initial project snapshot by Danny
Yu" process while commits were being published. No force-push was performed by this work. The Pi
deployment retained its previous heads as recoverable local branches
`pre-rewrite-main-20260821` (`949cfa1`) and `pre-rewrite-main-20260821-2` (`a4de0eb`) before tracking
the final rewritten `origin/main`. Before the next edit session, identify or disable the process
that recreates root commits; otherwise it can again make ordinary fast-forward pulls impossible.

Next-session motor test command (operator beside the car and ready to cut power):

```bash
ssh -t carpi 'cd ~/Car-and-Robotic-Arm && mkdir -p scratch/ir-sensor-tracking && LOG="scratch/ir-sensor-tracking/$(date +%Y-%m-%d-%H%M%S)-phase02-03-04-transition-20s.log" && echo "COMMIT=$(git rev-parse --short HEAD) LOG=$LOG" && PYTHONPATH=src python3 -u examples/40_map1_ir_phase_test.py --phase 3 --duration 20 --start-acquire-timeout-s 5 --phase3-lead-in-timeout-s 5 --phase3-exit-confirm-s 0.5 --phase4-proof-s 2 --heartbeat-s 0.5 --log-every --log-min-interval-s 0.1 --speed 150 2>&1 | tee "$LOG"'
```

A passing log must contain `Phase 3 ARC 1 detected`,
`Phase 3 ARC 1 sensor exit confirmed -> Phase 4 North straight`, and
`Phase 4 proof complete`. `Duration limit reached (20.0s)` is a failed/incomplete test.

### 07:26 transition-test failure after closeout

The operator ran the prepared test once more and reported that the chassis left the map. Log
`scratch/ir-sensor-tracking/2026-08-21-072629-phase02-03-04-transition-20s.log` proves that the
lead-in was correct: it detected `P0100 -> P0000 -> P1000` after 2.11s. During ARC control, left
evidence was followed by centred `P0110` from approximately 3.5s through 4.0s. That roughly 0.6s
window was physically sufficient to reach the ARC exit, but the 0.8s confirmation gate did not
open. From 4.1s the line progressed rightward through `P0111 -> P0011 -> P0001`; LEFT ONLY rejected
all of it and continued a left command until the chassis reached carpet `P1111` at 6.0s. The run
timed out with `mode=arc`, three reverse replays, and 55.3% noise/hold frames.

The Phase 3 exit confirmation is now 0.5s, so the observed centred window switches to ordinary
Phase 4 control before the rightward departure. This calibration is software-tested but has not
yet been physically verified; do not treat it as a passed route segment until the next log reaches
`Phase 4 proof complete` on the black line.

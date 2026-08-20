# Map1 Junction Detection Rewrite: Ordered Signal Sequences — Work Log (2026-08-20)

Continues [`2026-08-20-map1-spin-recalibration-carpet.md`](2026-08-20-map1-spin-recalibration-carpet.md).
Same-day track attempts kept failing after the spin recalibration and the earlier
`confirm_signatures` dwell-timer fixes (see [`carbot.ir_route`](../../src/carbot/ir_route.py)
module docstring, "second pass"). This session replaces the whole junction-detection model —
single reading, sustained for a time threshold — with **ordered signal sequences** built
directly from the operator watching the real per-frame `P1..P4` log, and switches junction
turns from a pure timed spin to closed-loop (watch for the line, don't just wait out a clock).

## 1. Why the dwell-timer model was wrong at the root

Both the 2026-08-20 "second pass" fixes (widening `confirm_signatures` for the roundabout
exit, holding instead of steering mid-dwell) patched the *dwell* model without questioning
whether "one reading, sustained" is what these junctions actually produce. It is not. Direct
signal tracing showed every real junction produces a **sequence of distinct readings in a
fixed order**, and several of those readings are not junction-shaped at all under
[`carbot.ir_geometry`](../../src/carbot/ir_geometry.py)'s single-line model:

- The roundabout exit's sequence includes `0101`, which `STATE_TABLE` classifies as
  `Kind.NOISE` ("no single line can produce this") — a confirm-set widening can never
  represent that, no matter how many individual readings get added to it, because the signal
  is the *order*, not any one reading in isolation.
- The same sequence *ends* on `0110` — ordinary centred `FOLLOW`, indistinguishable from
  normal line-tracking on its own.

A single-reading dwell check, however wide the confirm set, structurally cannot express "A
then B then C in order." Only an ordered state machine can.

## 2. Real signal data, per junction (operator-traced, real track)

| Junction | Sequence | Notes |
|---|---|---|
| **a** Start-stem T | `1111` (2cm) → `0000` | Symmetric crossbar — the stem approaches head-on |
| **e** Roundabout entry | `1111` (1.5-1.8cm) → `1001` (0.2cm) → `0000` | The `1001` shoulder is what tells it apart from the plain T |
| **f** Roundabout exit | `0111` → `0101` → `0100` → `0110` | `0110` (heading ~north here) is arrival, not just re-centring |
| **g/h** Lap-crossing T (Phase 10 approach) | `0111` (2cm) → `0110` | Same physical junction as (a), but read **asymmetrically** (`0111` not `1111`) because Phase 10 approaches off-centre, not head-on |

(g)/(h) share one important consequence: reaching `0110` after the 2cm hold **is** arrival —
no creep, no turn. The car is already centred on the new line the moment the sequence
completes.

## 3. Turn completion is also closed-loop now, and why that's safe

The three turning junctions (a/e/f) were found to end their turn on the *same* re-centring
sequence: `0001 → 0011 → 0000 → 0010 → 0110`. `IRLineNav._turn_step` now watches for the last
reading (`0110`, `TURN_COMPLETE_READING`) instead of spinning for a fixed
`nominal_turn_s()`.

This looks like it reintroduces the exact bug the original timed design was built to avoid
(2026-08-18: an early "any channel visible" exit fired at ~12° into a 90° turn, because the
junction crossbar itself reads black while the car pivots on top of it). It does not, for one
specific reason: the original bug checked *any visible channel*, which the crossbar itself
satisfies almost immediately. This checks for **one specific reading** (`0110`) that, per the
real-track trace above, is only reached once the car has swept far enough that the outer
sensors have cleared the old crossbar/curve entirely. Two guards on top:

- A minimum elapsed time (`spin_dead_time_s`) before a `0110` read is trusted at all, using
  the same "motor hasn't really started moving yet" floor the spin calibration itself relies
  on — guards against a coincidental `0110` in the first instant.
- A generous timeout (`turn_timeout_s`, `turn_timeout_scale × nominal timed duration`) in
  case `0110` never comes back — misalignment, a genuine sensor fault — so a car that can't
  reacquire the line doesn't spin forever.

## 4. What changed in code

- [`carbot.ir_route`](../../src/carbot/ir_route.py): new `SequenceStep`
  (`bits`, `min_cm`) and `RouteJunction.approach` (ordered tuple of `SequenceStep`), replacing
  `confirm_signatures`/`DEFAULT_JUNCTION_SIGNATURES`/`ROUNDABOUT_EXIT_SIGNATURES` entirely.
  Added `RouteJunction.creep_cm` (per-junction blind creep, replacing the single global
  `creep_before_turn_cm`) and `.turn_deg` (expected turn magnitude, used only to bound the
  closed-loop turn's timeout). `TURN_COMPLETE_READING = (0, 1, 1, 0)`.
- [`carbot.ir_line_nav`](../../src/carbot/ir_line_nav.py): `_approach_step` replaces the old
  dwell-timer branch in `_follow_step` — an ordered index + per-step distance accumulator,
  resetting to step 0 (and retrying once) on a reading that matches neither the current nor
  the next expected step, rather than getting stuck partway through a stale match.
  `_turn_step` is now closed-loop (takes the current `reading`, not just `dt`) per §3 above.
  `IRNavPolicy` lost `junction_min_s`, `turn_deg`, `creep_before_turn_cm`,
  `nominal_turn_s()`, `creep_duration_s()` (all superseded by the per-junction fields above);
  gained `turn_timeout_scale` / `turn_timeout_s()`.
- [`examples/39_map1_ir_line_follow.py`](../../examples/39_map1_ir_line_follow.py): dropped
  the now-meaningless `--junction-min-s`/`--turn-deg`/`--creep-before-turn-cm` flags, added
  `--turn-timeout-scale`.

Concrete values used (cm since the previous junction's own last accept, or degrees):

| Junction | `creep_cm` | `turn_deg` |
|---|---|---|
| a Start-stem T | 8.5 (midpoint of 8-9) | 90 (midpoint of 85-95) |
| e Roundabout entry | 8.0 (midpoint of 7.5-8.5) | 42.5 (midpoint of 40-45) |
| f Roundabout exit | 6.5 (midpoint of 6-7) | 90 (midpoint of 85-95) |
| g/h Lap-crossing T | — (no turn) | — (no turn) |

These are first estimates from the operator's ranges, not measured constants — re-tune from
real track logs, the same way the spin calibration itself was re-measured in
[`2026-08-20-map1-spin-recalibration-carpet.md`](2026-08-20-map1-spin-recalibration-carpet.md).

## 5. What did *not* change

- ARC 1/2/3 (`b`/`c`/`d`, `CornerWindow`) — still continuous curves handled by ordinary
  proportional steering with a temporary speed/gain boost, never a scripted turn. No
  real-track anomaly was reported for these this session.
- The distance gate (`min_cm_since_previous`) — unchanged, still runs as an outer sanity
  check once an approach sequence completes.
- The 2026-08-20 "second pass" fixes this session's rewrite absorbs rather than reverts:
  SEARCH not fabricating distance credit, and the post-turn `_last_localising` reset (still
  present in `_turn_step`, both completion paths).

## 6. First real-track run of the rewrite: a launch-time regression, then a real-track refinement

`--laps 1` on the rewritten code steered left at ~0.2s into the run (`P1110`, "branch or
curve on the left") and never recovered — 0 junctions taken in 37s, a wandering
search/drift loop, and the run ended on an unrelated I2C bus fault (operator cut power; see
the safety note below, not a code issue).

Root cause: the rewrite dropped the old blanket "a `Kind.JUNCTION` reading never steers"
rule when it replaced the dwell timer with `_approach_step`, and nothing took its place — a
reading that didn't match the pending junction's current approach step fell all the way
through to the generic offset-based steer, exactly the class of bug the 2026-08-20 "second
pass" had already fixed once for the dwell-timer design.

The immediate fix (blanket "any unmatched `Kind.JUNCTION` reading holds") turned out to be
wrong too, once the operator supplied a further real-track observation: `0111`/`1110`
commonly appear **just before** a real `1111` crossbar (and, symmetrically, just before the
post-crossbar `0000`) — an ordinary skewed-approach reading, not noise. Holding here would
have reintroduced the 2026-08-19 regression from the *other* direction: `Kind.JUNCTION` also
covers "badly skewed pass over a curve" (see `carbot.ir_geometry`), and that case has to keep
steering or the car drives off the map.

The two are told apart by whether any progress has actually been made on the pending
junction's approach sequence yet (`started`: this step's `min_cm` partly satisfied, or
already past step 0):

- **Not started** — indistinguishable from an ordinary curve; keep steering on it.
- **Started** — close to a real, mostly-confirmed junction; an unrelated `Kind.JUNCTION`
  reading here is far more likely crossbar shoulder noise than a genuine curve, so it holds
  instead, without losing the progress already made.

A related bug surfaced while implementing this: the "fast transition" allowance (skip
straight to the next approach step if the reading matches it, in case a frame was missed)
was checking the next step's target even from a completely fresh, unstarted sequence —
meaning an ordinary `0000` (the blind band, or any real line loss, common everywhere) would
instantly "complete" any junction whose *last* approach step happens to be `0000` (both the
start-stem T and the roundabout entry), with zero persistence ever actually checked. Fixed
by requiring `started` before the fast-transition check runs at all.

See `IRLineNav._approach_step`'s docstring for the final logic, and
`test_a_skewed_approach_reading_before_the_first_step_still_steers` /
`test_a_junction_shaped_reading_that_breaks_a_started_sequence_holds` in
`tests/test_ir_line_nav.py` for the two cases as regression tests.

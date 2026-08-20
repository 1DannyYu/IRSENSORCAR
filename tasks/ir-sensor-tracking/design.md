# IR Tracking — Sensor Model and Route Logic

## Conventions

`1 = LED lit = black` (GPIO LOW, `invert={0,1,2,3}` applied).
Bit order below is **physical `P1 P2 P3 P4`, left to right along the bar** —
not channel order.

| Position | Channel | BCM GPIO | Pi pin | Offset |
|---|---|---|---|---|
| `P1` leftmost | `Out2` | 25 | Pin 22 | −3.2 cm |
| `P2` | `Out1` | 24 | Pin 18 | −0.4 cm |
| `P3` | `Out3` | 22 | Pin 15 | +0.4 cm |
| `P4` rightmost | `Out4` | 23 | Pin 16 | +3.2 cm |

Spacing `2.8 / 0.8 / 2.8 cm`, bar spans 6.4 cm, line is 2.0 cm wide.

**Blind band = gap − line width = 2.8 − 2.0 = 0.8 cm**, centred at ±1.8 cm.
**Detection limit = ±4.2 cm.**

## The 16 readings

Implemented in [`carbot.ir_geometry.STATE_TABLE`](../../src/carbot/ir_geometry.py).
A unit test asserts the noise class is *exactly* the non-contiguous readings, so
the split is a consequence of geometry rather than a hand-written list.

### A. Line-following — the only readings one 2 cm line can produce

| Reading | Line centre | Window | Meaning | L | R |
|---|---|---|---|--:|--:|
| `1000` | −4.2…−2.2 | 2.0 cm | far left | **20** | 150 |
| `0000` | −2.2…−1.4 | 0.8 cm | **left blind band — still on the line** | see below | |
| `0100` | −1.4…−0.6 | 0.8 cm | slight left | **110** | 150 |
| `0110` | −0.6…+0.6 | 1.2 cm | **centred** | 150 | 150 |
| `0010` | +0.6…+1.4 | 0.8 cm | slight right | 150 | **110** |
| `0000` | +1.4…+2.2 | 0.8 cm | **right blind band — still on the line** | see below | |
| `0001` | +2.2…+4.2 | 2.0 cm | far right | 150 | **20** |

Only **0.8 cm** of warning separates centred from blind, which is why the slight
correction is not gentle.

### B. Junction / curve — needs a second dark feature

| Reading | Meaning | Action |
|---|---|---|
| `1111` | symmetric crossbar | sustained ≥0.15 s → **roundabout entry** |
| `0111` | branch or curve on the right | sustained → exit *or* T junction (below) |
| `1110` | branch or curve on the left | steer left (medium); not on this route |
| `0011` | needs >2.8 cm of black | roundabout skew → steer right (medium) |
| `1100` | needs >2.8 cm of black | roundabout skew → steer left (medium) |

`0011` and `1100` are impossible on a straight line. They occur when the bar
passes a curve at a shallow angle, so their meaning is "skewed on the
roundabout", not "offset on a straight".

### C. Noise — non-contiguous black, one line cannot produce it

`1010`, `0101`, `1001`, `1011`, `1101` → **hold the previous command, count it.**

Causes, in order of likelihood on this build: undulating paper lifting a channel
out of range (which reads as *black*), a potentiometer drifting to the edge of
its working range, or a genuine second dark feature. These readings are a
diagnostic gauge, never a steering input — if they exceed ~5% of frames, raise
the bar or re-tune the pots.

## `0000` — resolved by history, not a timer

The line can only leave the bar past an **outer** sensor, so the previous
reading is decisive:

| Previous | Verdict | Action |
|---|---|---|
| `0010` | right blind band, still on the line | steer right, inner wheel 60 |
| `0100` | left blind band, still on the line | steer left, inner wheel 60 |
| `0001` | line has passed +4.2 cm | SEARCH |
| `1000` | line has passed −4.2 cm | SEARCH |
| `0110` | unreachable in one step | undulation → hold previous |

## Route — continuous loop, no return to the start box

Counter-clockwise. Per lap the car passes three junctions:

| Order | Where | Reading | Action |
|---|---|---|---|
| 1 | Roundabout entry (12 o'clock, perpendicular) | `1111` | creep 9.5 cm → **right 90°** |
| 2 | Roundabout exit (3 o'clock) | `0111` | creep 9.5 cm → **right 90°** |
| 3 | T junction (crossed heading east) | `0111` | **straight through** |

Junctions 2 and 3 have identical signatures — a right branch — and the map gives
no way to tell them apart from a single reading.

The first attempt used one `in_roundabout` boolean, anchored on `1111` as "the
roundabout entry, unambiguous, once per lap". **The 2026-08-19 track run
disproved all three premises:**

| Premise | What the run showed |
|---|---|
| The stem T junction reads `0111` | It reads **`1111`** — the crossbar runs east *and* west, so it is symmetric. The first junction of the run set the flag backwards |
| The roundabout entry reads `1111` | It read **`0111`** — the car arrived skewed. The two signatures had swapped |
| `1111` appears once per lap | **Four more** sustained `1111` events on the roundabout alone |

Result: six junction events, six right turns, **zero** straight-through
crossings, and the car finished by turning into the start box.

So the action does not come from the reading. It comes from the route sequence
in [`carbot.ir_route`](../../src/carbot/ir_route.py):

```
prologue:  start stem T junction  → right 90°   (gate 0 cm)
loop:      roundabout entry       → right 90°   (gate 60 cm)
           roundabout exit        → right 90°   (gate 40 cm)
           T junction             → cross       (gate 10 cm)
```

The sensor's only job is "a junction is under the bar". A **distance gate**
then rejects a junction that turns up before the route expects one — that is
the junction just handled being read a second time, or a curve taken shallow
enough to light the whole bar. Distance is time × the measured 10 cm/s, and a
pivot is excluded because it covers no ground.

This works only because the spacings differ by more than 6×: 23 cm from the
roundabout exit to the T, ~150 cm from the T back to the entry. A coarse
estimate is enough to keep the sequence in step.

The trade against the old design is explicit: a counter that slips stays
slipped, whereas the boolean could re-synchronise. The boolean's
re-synchronisation was worth nothing here because its anchor was not unique,
and the gate rejects the repeats that would slip the counter in the first
place. `--start-on-loop` drops the prologue for starting on the east-west line.

**2026-08-20 correction: `1111` is not unique either, anywhere in the lap.**
An earlier version of this doc (and the top of
[docs/task1-single-source-of-truth.md](../../docs/task1-single-source-of-truth.md))
described the roundabout entry's `1111` as an "unambiguous" signal used to
sync each lap. That is wrong in the same way the retired boolean design was
wrong: `1111` reads at the start stem T, the roundabout entry, *and* the
roundabout exit, and a flat straight section can spuriously produce a
junction-shaped reading (e.g. `0111`) from paper unevenness or a sensor
misread. Nothing about `1111` (or any single reading) is unique — the system
works despite that, purely from the three layers already described above:
sequence position, the dwell timer filtering brief noise, and the distance
gate filtering real features that turn up too early. The roundabout entry has
no special status beyond being node 2 of a fixed, known order.

## 2026-08-20 two-lap track run: two more failures, both fixed

Running the two-lap plan end to end did not complete. Root cause both times:
the nav layer treated a junction-shaped reading as if it were still a normal
line-following reading, either for classification or for steering.

1. **Roundabout exit dwell timer reset by noise.** The exit produced `0111`,
   `1001`, `1111`, `1110` in no fixed order. `0111`/`1111`/`1110` are
   `Kind.JUNCTION` and fed the dwell counter; `1001` is `Kind.NOISE` (no
   single 2cm line can produce it) and fell outside it. Every `1001` frame
   between qualifying ones reset the counter to zero, so the sustained bar
   never got confirmed. Fix: `RouteJunction.confirm_signatures` — each
   junction now declares its own accepted reading set instead of relying on
   the generic `Kind.JUNCTION` classification; the roundabout exit's set adds
   `1001` (`ROUNDABOUT_EXIT_SIGNATURES` in
   [`carbot.ir_route`](../../src/carbot/ir_route.py)). No other junction
   showed this problem, so their sets stayed at the default.
2. **Steering did not stop for a junction-shaped reading mid-dwell.** Before
   the dwell timer finished, `IRLineNav` was still steering proportionally on
   the confirming reading's `offset_cm` — a number derived from where a
   single straight line sits under the bar, meaningless for a curve, branch,
   or crossbar. Correcting on it pulled the car off its approach before the
   route-driven turn/cross ever ran. Fix: hold the last steady line-following
   command during that dwell window instead (`IRLineNav._hold`); the
   distance-gate-rejected path is unchanged, since that one has to keep
   steering for a different, already-fixed failure (see its own comment in
   `ir_line_nav.py`).
3. **A related latent bug found while diagnosing the above: post-turn `0000`
   used stale pre-turn line position.** The pivot (`_turn_step`) is a pure
   timed spin, not angle-verified — measured 85-93° on the real track, not a
   clean 90. A `0000` right after landing is therefore common, and
   `resolve_blind()` was still consulting `_last_localising` from *before*
   the turn to decide "blind band" vs "lost". That geometry belongs to the
   old heading and says nothing about the new one. Fix: `_turn_step` clears
   `_last_localising` when the turn completes, so a post-turn `0000` always
   resolves to "lost" (search) instead of a stale "keep going straight"
   guess.

The three outer corners (ARC 1 SE, ARC 2 NE, ARC 3 NW) are **left** curves and
are not junctions — they produce no `1111` and are followed by ordinary
steering. Together with the roundabout's 270°, roughly 43% of the route is a
left-hand curve, which is why a blanket "turn right when lost" rule was rejected.

## 2026-08-20, second pass: ran off the map at ARC 1, and a fabricated-distance bug

A same-day retest (after the three fixes above) still did not survive one lap: the
car ran off the map turning from Phase 2 (east) onto Phase 4 (north) at ARC 1.
Two more findings, both from the operator watching the physical car, not from
the log alone — the log alone had already produced a plausible-looking
"route complete" trace for a run that did not physically happen; log
self-consistency is not proof of physical correctness on this system, since
nothing here corroborates position against the printed map except the sensor
readings the nav loop is already interpreting.

1. **`JunctionSequencer.travel()` credited SEARCH's sweep sub-phases as forward
   motion.** Sweeping left/right is a pure rotation, exactly like
   `JUNCTION_TURN` (already excluded) — but SEARCH itself was not excluded, so
   a lost car spinning in place could rack up fabricated "distance since the
   last junction" without moving, opening gates it had not physically reached
   and letting the sequence drift arbitrarily far from the real map position.
   Fix: `step()` now excludes the whole `SEARCH` state from distance credit,
   not only `JUNCTION_TURN` — including the forward-creep sub-phase, since its
   speed does not match `forward_speed_cm_per_s` either and the car's real
   position is unknown while still lost.
2. **ARC 1's turn is tighter than steady-state FOLLOW gains can track.**
   Back-computing a radius from its ~3.6cm arc length over a 90° heading
   change gives roughly **2.3cm** — smaller than the car's own footprint. At
   full speed (150) and the STATE_TABLE's fixed `inner_ratio` values, the
   proportional correction cannot turn tightly enough before the curve bends
   away, so the car ran wide off the outside of the corner. This is not a
   junction-style problem (there is one continuous printed line the whole way
   through, no ambiguity for the sensor to resolve) and is **not** solved by a
   scripted/blind turn like a T junction gets — that would throw away the
   working sensor signal for no reason. Fix: `CornerWindow` in
   `carbot.ir_route` — a stretch of `cm_since_previous` (while "roundabout
   entry" is pending) over which `IRLineNav._steer` drives slower and
   sharpens the correction, while continuing to read the line every cycle.
   `TASK1_CORNER_WINDOWS` covers all three arcs; the cm ranges come from the
   SSOT phase table, margins widen for later corners since the 10cm/s
   distance estimate drifts further the longer since the last confirmed
   junction. Scale factors (0.6× speed, 0.5× inner_ratio) are a first
   estimate, not a measured constant.

## 2026-08-20, third pass: chassis fault mid-sweep, then a clean recalibration

The car undershot its first T-junction turn (~45° instead of 90°) on the third
same-day track attempt. A spin-angle sweep to re-measure `spin_rate_deg_per_s`/
`spin_dead_time_s` (`examples/41_motor_spin_angle_sweep.py`) turned up something
the angle numbers alone would not have caught: partway through, the "in-place"
pivot was not in place — the chassis translated ~15cm north-east while
spinning, radius should have been zero. The operator checked the wheels/axles
by hand (the likely cause: today's earlier off-map excursions and abrupt
power cuts) before re-running the sweep; the second sweep confirmed a true
zero-radius pivot on every one of its 5 readings and fit a linear
angle-vs-duration model far more consistently than the first (residuals under
11° vs. wildly inconsistent deg/s across the first sweep's readings). See
[docs/progress/2026-08-20-map1-spin-recalibration-carpet.md](../../docs/progress/2026-08-20-map1-spin-recalibration-carpet.md)
for both sweeps' raw data.

This is the same lesson as the fabricated-distance bug above, one level up:
a plausible-looking number (an angle reading, a deg/s rate) is not proof the
underlying assumption (pure pivot, no chassis fault) held while it was taken.
Re-running the *whole* sweep after the fix, not just patching the affected
readings, is what caught it.

## 2026-08-20, fourth pass: junction detection rewritten to ordered signal sequences

Same-day track attempts kept failing even after the spin recalibration above. Direct
signal tracing (operator watching the real per-frame `P1..P4` log) showed the whole
single-reading-sustained-for-a-time-threshold model was wrong at the root: every real
junction produces an **ordered sequence** of distinct readings, several of which
(`0101`, `0110`) are not junction-shaped at all in isolation. `junction_min_s` and the
single global `creep_before_turn_cm` are gone — each junction now carries its own
`approach` sequence and `creep_cm`, and junction turns are closed-loop (watch for `0110`)
instead of a fixed duration. Full writeup:
[docs/progress/2026-08-20-map1-junction-signal-sequences.md](../../docs/progress/2026-08-20-map1-junction-signal-sequences.md).

## Timing constants

| Parameter | Value | Source |
|---|---|---|
| `speed` | 150 | verified |
| Spin rate | 42.0 °/s | measured 2026-08-20, 5-point sweep, Map1 paper on carpet, all readings confirmed a true pivot (supersedes the 2026-08-18 value, 40.5 °/s) |
| Spin dead time | 0.41 s | measured 2026-08-20 (supersedes 2026-08-18, 0.2 s); also the closed-loop turn's minimum time before trusting `0110` |
| `turn_timeout_scale` | 2.0 | safety ceiling multiplier on a closed-loop turn's nominal duration |
| Forward speed | 10 cm/s | measured on the map paper |
| Creep per junction | a=8.5cm, e=8.0cm, f=6.5cm, g/h=0 (no turn) | operator-estimated ranges, midpoint — see the ordered-sequence writeup above |
| Turn magnitude per junction | a=90°, e=42.5°, f=90°, g/h=0 (no turn) | used only to bound the closed-loop turn's timeout, not as the stop condition |

The creep exists because the sensor detects the crossbar/curve well before the axle
reaches it. Turning immediately was measured on 2026-08-18 to leave the axle
short of the junction: the car pivoted onto the gap, read nothing, and spent the
rest of the run searching — 22 searches in 60 s.

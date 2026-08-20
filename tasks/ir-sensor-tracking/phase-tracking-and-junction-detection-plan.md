# Phase Tracking + Junction Detection — Planning Draft (2026-08-20)

**Status: implemented, not yet real-hardware verified.** All open questions were resolved in
discussion and `src/carbot/ir_route.py`/`ir_line_nav.py` were updated to match (see
`docs/progress/2026-08-20-map1-phase-tracking-and-reverse-recovery.md` for the implementation
writeup). §5's reverse-replay is a brand new physical motor behaviour (this project has never
driven in reverse before) and needs a live test with the operator beside the car and rear
clearance confirmed before it is trusted.

**Governing principle (operator, 2026-08-20): everything below is probabilistic, not exact
matching.** Real paper has texture, sensor reads have single-frame noise, forward speed is an
estimate — none of the distances, hold thresholds, or phase-length targets in this document
are meant to be hit exactly. Every mechanism here (approach-sequence `min_cm`, the arc/straight
phase tracker, the junction preconditions) needs a **tolerance band**, not an equality check,
and needs to tolerate an occasional single-frame misread without derailing. Where a table below
gives one number, treat it as the centre of a range, not a hard requirement — the actual
band width is itself something to tune from real track logs, not decide up front.

This continues [`2026-08-20-map1-junction-signal-sequences.md`](../../docs/progress/2026-08-20-map1-junction-signal-sequences.md)
after the operator raised three problems with that design once it hit the real track:

1. The start-stem T's `1111` hold distance (`2.0cm`) likely includes the skewed-entry frames
   (`1110`/`0111`) that precede a clean `1111` read, so the real sustained-`1111` portion is
   probably smaller — closer to the `1.5-1.8cm` already measured for the roundabout entry's
   first step.
2. `b`/`c`/`d` (the three arcs) have never had their own detection logic — the current design
   just applies a speed/gain boost over a *distance-estimated* window
   (`CornerWindow`/`cm_since_previous`), with no check that the car is actually on the arc it
   thinks it's on. The operator has a concrete, sensor-grounded way to tell an arc from a
   straight phase.
3. `e` (roundabout entry) and `f` (roundabout exit) both have a real-world precondition beyond
   "the distance gate is open": `e` should only be considered once Phase 6 and ARC 3 are
   actually done, and `f` only once Phase 9 (the roundabout traversal) is actually done. The
   current design has no such precondition — it will try to match `e`'s approach sequence the
   moment the distance gate opens, regardless of what phase the car is really in.

## 1. Corrected re-measured distances (see the updated map)

| Segment | Old value | New value (operator-measured) |
|---|---|---|
| Phase 2 (east, T→ARC1) | 16.0 cm | **15.5 cm** |
| Phase 4 (north, ARC1→ARC2) | 18.7 cm | **18.0 cm** |
| Phase 6 (west, ARC2→ARC3) | 58.5 cm | **47.0 cm** |
| Phase 8 (south, ARC3→roundabout entry) | 7.5 cm | 7.5 cm (unchanged) |
| Phase 10 (east, roundabout exit→T) | 23.0 cm | **21.5 cm** |
| ARC 1 / ARC 2 / ARC 3 | ~3.6 / ~14.2 / ~7.2 cm (SSOT, all different) | **~12 cm each** (operator-measured, same shape, only position differs) |

Map updated: [`assets/reference/map-1/2026-08-21-ir-route-map.png`](../../assets/reference/map-1/2026-08-21-ir-route-map.png).

**Cumulative position (cm since the T junction/start of Phase 2 — the same reference frame
`CornerWindow` already uses via `cm_since_previous` while `"roundabout entry"` is pending),
recomputed from the corrected lengths above:**

| Segment | Start | End | Length |
|---|---|---|---|
| Phase 2 (east) | 0 | 15.5 | 15.5 |
| ARC 1 (SE) | 15.5 | 27.5 | 12.0 |
| Phase 4 (north) | 27.5 | 45.5 | 18.0 |
| ARC 2 (NE) | 45.5 | 57.5 | 12.0 |
| Phase 6 (west) | 57.5 | 104.5 | 47.0 |
| ARC 3 (NW) | 104.5 | 116.5 | 12.0 |
| Phase 8 (south, entrance) | 116.5 | 124.0 | 7.5 |
| → roundabout entry | 124.0 | — | — |

**This directly affects the already-live `TASK1_CORNER_WINDOWS`** in
[`ir_route.py`](../../src/carbot/ir_route.py) (the ARC 1/2/3 speed/gain-boost windows from
earlier today), which were built from the old, wrong SSOT arc lengths and straight lengths:

| Window (current, wrong) | start_cm | end_cm | Recomputed (±margin around the table above) |
|---|---|---|---|
| ARC 1 SE corner | 12.0 | 23.0 | ~13-30 (centre 15.5-27.5) |
| ARC 2 NE corner | 33.0 | 58.0 | ~42-61 (centre 45.5-57.5) |
| ARC 3 NW corner | 102.0 | 124.0 | ~101-120 (centre 104.5-116.5) |

**DECIDED (operator): hold.** `CornerWindow` stays as-is (still the old, wrong SSOT-derived
distances) until §3/§4 below are settled, then correct it in one combined change rather than
twice.

## 2. Start-stem T's approach: shorten the `1111` hold

**Correction (operator): the two junctions do NOT share one number — the start-stem T's
`1111` hold is longer than the roundabout entry's, not the same.** Different physical
approach geometry (head-on stem vs. the entry's own approach angle), so each keeps its own
measured range:

| | Old | Proposed |
|---|---|---|
| **a** Start T, step 1 | `1111`, min 2.0cm | `1111`, min **1.9cm** (midpoint of 1.8-2.0cm) |
| **e** Roundabout entry, step 1 | `1111`, min 1.65cm | unchanged — `1111`, min **1.65cm** (midpoint of 1.5-1.8cm) |
| Both, step 2 | `0000`, immediate | unchanged |

Same underlying reasoning for both (the raw "2cm" figure for `a` very likely included a frame
or two of `1110`/`0111` skewed-entry noise before the crossbar reads cleanly symmetric, so the
real sustained-`1111` window is shorter than first recorded) — but the two junctions' actual
numbers are distinct and must stay that way, not merged to one shared value.

## 3. Phase tracking for the arcs (`b`/`c`/`d`) — new mechanism

**The operator's description, formalised:**

- On an arc (a real left curve), the line continuously pulls away from the bar centre, so the
  car repeats a short cycle: `0110` (briefly centred) → `0100` (drifted, correct left) →
  back toward `0110` → `0100` again → ... This repeats roughly continuously while genuinely on
  the curve.
- On a straight phase, once the arc ends, `0110` holds **without** the `0100` corrections
  recurring, for (approximately) the phase's known length — occasional single-frame noise
  (paper unevenness) is expected and must not reset this, per the operator's own caveat.
- So: **sustained `0110` for close to a known phase's length, without a recurring `0100`
  cycle, is itself the signal "this phase is done / a specific phase is now in progress."**

**Proposed state**: track two things per cycle, in `IRLineNav` (not per-`RouteJunction`, since
arcs are not junctions — see [`CornerWindow`](../../src/carbot/ir_route.py)):

- `_straight_cm`: accumulated distance since the last `0100`/`0010` (or other non-`0110`)
  correction reading — resets whenever a correction reading occurs, accumulates on `0110`.
- A **noise tolerance**: a single non-`0110` frame surrounded by `0110` frames (i.e. one frame
  that doesn't fit the arc-correction rhythm) must not reset `_straight_cm`.
  **DECIDED (operator): a non-`0110` reading must persist ≥ 0.8s before it counts as a real
  arc-correction event and resets `_straight_cm`.** Below that, it's treated as noise and
  ignored — `_straight_cm` keeps accumulating through it. (Deliberately longer than
  `junction_min_s`'s old 0.15s, which filtered single-sample paper-fold noise for a *sustained
  crossbar* check — this is a different signal, filtering the much shorter single-frame misreads
  that must not be confused with the genuine, repeating arc-correction rhythm.)

**Phase boundaries this lets us detect** (`_straight_cm` reaching close to the segment's known
length, with some tolerance band, e.g. ±20%):

| Segment | Length | `_straight_cm` target |
|---|---|---|
| Phase 2 (east) | 15.5 cm | ~12-18cm sustained `0110` (with the arc-rhythm `0100` cycle beforehand near the T) |
| Phase 4 (north) | 18.0 cm | ~14-22cm |
| Phase 6 (west) | 47.0 cm | ~38-56cm |
| Phase 8 (south, entrance) | 7.5 cm | ~6-9cm |
| Phase 10 (east, return) | 21.5 cm | ~17-26cm |

**OPEN QUESTION**: should `_straight_cm` reset on *every* junction-approach step too (since
those also aren't `0110`), or only on the arc-rhythm `0100` reading specifically? If a
junction's own approach sequence (e.g. roundabout exit's `0111→0101→0100→0110`) shares the
`0100` reading, does that interfere with counting `_straight_cm` for the phase right before it?
This needs to be worked through once the exact precondition wiring (§4) is settled, since the
two mechanisms will run concurrently near a junction.

## 4. Junction preconditions beyond the distance gate

| Junction | Existing precondition | New precondition (operator) |
|---|---|---|
| `a` Start T | `min_cm_since_previous` = 3.0cm (§ vs departure noise) | none added |
| `e` Roundabout entry | `min_cm_since_previous` = 60.0cm | **Phase 6 AND ARC 3 must already be confirmed complete** (via §3's phase tracker) |
| `f` Roundabout exit | `min_cm_since_previous` = 40.0cm | **Phase 9 (roundabout traversal) must already be confirmed complete** |
| `g`/`h` T junction | `min_cm_since_previous` = 10.0cm | none added (Phase 10 already gates this implicitly via distance) |

**Design implication**: `RouteJunction.approach` matching (`_approach_step`) should not even
*start* trying to match `e`'s or `f`'s sequence until the phase tracker confirms the
precondition — otherwise a coincidental early reading (e.g. a stray `1111` mid-Phase-6) could
start the approach sequence prematurely, same class of risk the distance gate already guards
against, but distance alone doesn't capture "have we actually been through Phase 6 and ARC 3,"
only "roughly how much time/distance has elapsed since the last junction."

**OPEN QUESTION**: is the phase tracker meant to *replace* `min_cm_since_previous`, or run
*alongside* it (both must pass)? Running alongside is lower-risk (keeps the existing,
already-working distance gate as a backstop) but means two independent systems must agree.
Recommend alongside unless there's a reason not to.

## 5. Off-track recovery — decided design

### Map geometry (operator, 2026-08-20)

Absolute reference, printed map: **X 1000mm × Y 700mm**. Car centre at rest in the departure
box is approximately **(700, 100)** in that frame. Beyond the paper's edge is **carpet**.

**Two distinct off-track signatures, not one:**

| Condition | Reading | Physical meaning |
|---|---|---|
| Continuous `0000` ≥ 2s | blank paper | drove into open white space on the map, off any line |
| Continuous `1111` ≥ 2s | carpet | drove **off the paper entirely** — carpet is a non-reflective surface, and per [`docs/hardware/ir-tracing-sensor.md`](../../docs/hardware/ir-tracing-sensor.md) ("floating / no surface reads the same as black"), no return reads as all-4-black, same polarity as a real crossbar |

This also gives a clean, principled way to tell a genuine junction's `1111` from "drove off the
map": every real junction's `1111` hold is well under 1 second (1.65-1.9cm at ~10cm/s ≈
0.15-0.19s — see §2). **A `1111` sustained for a full 2 seconds cannot be a junction**, full
stop — it can only be carpet.

### Recovery mechanism: reverse-replay, not a blind reverse manoeuvre

Not a newly-guessed reverse speed/duration — **replay the actual commanded wheel-speed history
of the last 2 seconds, in reverse order, with each `(left, right)` pair sign-flipped.** Since
the history is the exact sequence of commands that got the car to its current (wrong) position,
replaying it inverted retraces the real path back, including through any turns — a blind
"reverse at some new speed for 2s" would not follow the same path if the car curved during those
2 seconds.

1. `IRLineNav` keeps a rolling buffer of the last ~2s of `(left, right, dt)` commands issued
   (ring buffer, bounded — no unbounded growth).
2. Once `0000` or `1111` has been continuous for 2s, start popping the buffer **newest-first**,
   emitting each `(-left, -right)` for that entry's `dt`.
3. Check every frame during replay: if a channel becomes visible with a reading that isn't
   itself `0000`/`1111`-continuing, stop the replay immediately and resume normal `FOLLOW` from
   there.
4. If the full 2s buffer replays out and the car still reads `0000`/`1111`, only then enter the
   existing sweep-based SEARCH.

**Safety note (new physical behaviour — this project has never driven in reverse before):**
this needs real-hardware verification with the operator beside the car and physical clearance
behind it confirmed before the first live test, same as any other new motor behaviour.

**Implementation note**: while the reverse-replay is active, distance/phase-tracking credit for
those frames should almost certainly be treated the same way `SEARCH` already is (excluded from
`JunctionSequencer.travel()` — see the 2026-08-20 "second pass" fix) rather than double-counted,
since the car is retracing ground it already has credit for, not making new progress.

## 6. What is NOT proposed to change

- The closed-loop turn mechanism (watch for `0110`, `TURN_COMPLETE_READING`) — unaffected by
  any of the above.
- `e`/`f`'s own `approach` *sequences* (`1111→1001→0000` / `0111→0101→0100→0110`) — unchanged,
  only *when they're allowed to start matching* changes (§4).
- `g`/`h` (T junction) — no phase-tracking precondition proposed; flag if one should exist.

# Map1 Phase Tracker + Off-Track Reverse Recovery — Implementation (2026-08-20)

Implements [`tasks/ir-sensor-tracking/phase-tracking-and-junction-detection-plan.md`](../../tasks/ir-sensor-tracking/phase-tracking-and-junction-detection-plan.md)
after discussion resolved its open questions. Continues
[`2026-08-20-map1-junction-signal-sequences.md`](2026-08-20-map1-junction-signal-sequences.md).

## What changed

**`src/carbot/ir_route.py`:**
- `RouteJunction` gained `min_phase_transitions`/`min_arc_cm` — preconditions on the new phase
  tracker, checked before a junction's `approach` sequence is even attempted (not just the
  existing distance gate).
- Start-stem T's `1111` hold: `2.0cm` → **1.9cm** (midpoint of 1.8-2.0cm). Roundabout entry's
  `1111` hold stays **1.65cm** (midpoint of 1.5-1.8cm) — the two are distinct, not shared.
- Roundabout entry: `min_phase_transitions=6` (Phase 6 + ARC 3 confirmed done).
- Roundabout exit: `min_arc_cm=68.0` (~80% of Phase 9's 84.8cm — one continuous curve, so
  mode-flip counting doesn't apply the way it does for the entry).
- `TASK1_CORNER_WINDOWS` distances recomputed from the operator's re-measured lengths (Phase 2
  = 15.5, ARC 1/2/3 all ~12.0 — same shape, only position differs, not the old 3.6/14.2/7.2 —
  Phase 4 = 18.0, Phase 6 = 47.0, Phase 8 = 7.5).

**`src/carbot/ir_line_nav.py`:**
- New straight/arc **phase tracker**: `_phase_mode` ("straight"/"arc"), `_straight_cm`/
  `_arc_cm` accumulators, `_phase_transitions` counter. A non-`0110` reading must persist
  `phase_transition_dwell_s` (0.8s) before it counts as a real arc-correction event and flips
  the mode — shorter blips are noise and are ignored, `_straight_cm`/`_arc_cm` keep
  accumulating through them. Runs only on readings that fall through `_approach_step` (not
  part of an active junction approach). Resets on every accepted junction.
- `_approach_step` now checks `_phase_precondition_met()` first — a junction whose
  `min_phase_transitions`/`min_arc_cm` isn't satisfied yet is not attempted at all, falling
  through to ordinary steering (and feeding the phase tracker) instead.
- New **off-track reverse-replay recovery**: continuous `0000` (blank paper) or `1111`
  (off the paper onto carpet — see `docs/hardware/ir-tracing-sensor.md`, no return reads the
  same as black) for `off_track_dwell_s` (2.0s) triggers `IRNavState.REVERSE`. Not a freshly
  guessed reverse manoeuvre — replays the actual `(left, right, dt)` command history of the
  last `reverse_replay_window_s` (2.0s), newest-first, each entry sign-flipped, retracing the
  real path (including through turns) rather than a generic backward drive. Stops early the
  moment a real signal reappears; falls through to the existing sweep `SEARCH` only if the
  whole window replays with nothing found. Skipped during `JUNCTION_CREEP`/`JUNCTION_TURN`
  (short, deliberately sensor-blind, already have their own timeout) — checked only during
  `FOLLOW`/`SEARCH`. Excluded from `JunctionSequencer.travel()` credit, same as `SEARCH`.
- `IRNavPolicy` gained `phase_transition_dwell_s`, `off_track_dwell_s`,
  `reverse_replay_window_s`.

**`examples/39_map1_ir_line_follow.py`**: exposes the three new policy fields as CLI flags,
and reports off-track reverse-replay entries in the run summary.

**Map**: [`assets/reference/map-1/2026-08-21-ir-route-map.png`](../../assets/reference/map-1/2026-08-21-ir-route-map.png)
relabelled with the re-measured Phase 2/4/6/10 and ARC 1/2/3 lengths.

## Not yet real-hardware verified

The reverse-replay manoeuvre is a genuinely new physical behaviour (this project has never
driven the car backward before). Needs a live test with the operator beside the car, rear
clearance on the track confirmed, before it is trusted — same as any other new motor
behaviour on this project. The phase-tracker thresholds (0.8s dwell, the `min_phase_transitions`/
`min_arc_cm` precondition values, the corrected `CornerWindow` distances) are first estimates
per the planning doc's governing principle ("probabilistic, not exact matching") and should be
re-tuned from the next real-track log, not assumed correct on the first run.

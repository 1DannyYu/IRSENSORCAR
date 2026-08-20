# IR Channel Order Corrected, 16-State Table Implemented

Date: 2026-08-19

## Scope and Result

Measured the IR bar's physical channel order on hardware for the first time and
found the committed order was **mirror-reversed**, which meant every steering
correction was being applied to the wrong side. Replaced the count-based
steering with a geometry-derived table covering all 16 readings.

All measurement was sensor-only over SSH — no motors moved, nobody stood beside
the robot.

### The channel order was backwards

`src/carbot/ir_line_nav.py` recorded the bar as `Out4 Out3 Out1 Out2` left to
right, read off the potentiometer silkscreen on 2026-08-18. Measured order is
**`Out2 Out1 Out3 Out4`** — the exact mirror.

Method: the operator swept a black card across the bar while
[examples/42_ir_geometry_sweep.py](../../examples/42_ir_geometry_sweep.py)
logged transitions. The card's leading edge tripped the channels in the order
`Out2 → Out1 → Out3 → Out4`, and its trailing edge released them in the same
order — two independent edges agreeing. The operator separately reported `Out4`
is the rightmost sensor, which matches.

| Position | Channel | BCM GPIO | Pi pin | Offset |
|---|---|---|---|---|
| P1 leftmost | `Out2` | 25 | Pin 22 | −3.2 cm |
| P2 | `Out1` | 24 | Pin 18 | −0.4 cm |
| P3 | `Out3` | 22 | Pin 15 | +0.4 cm |
| P4 rightmost | `Out4` | 23 | Pin 16 | +3.2 cm |

This table did not exist anywhere before today. The wiring table recorded which
GPIO each *header pin* used, but nothing connected header pins to physical
positions, and they are not in order.

### New `src/carbot/ir_geometry.py`

Physical layout and the meaning of all 16 readings, separated from the
navigation state machine so it is testable without hardware. `STATE_TABLE`
splits the readings three ways, and a unit test asserts the noise class is
*exactly* the non-contiguous readings — the split is a consequence of geometry,
not a hand-maintained list:

- **6 line readings** a single 2 cm line can produce → steer
- **5 junction/curve readings** needing a second dark feature → junction logic
- **5 non-contiguous readings** one line cannot produce → hold, never steer

### `0000` is not "line lost"

With `P1–P2 = 2.8 cm` and a 2.0 cm line, there is a `2.8 − 2.0 = 0.8 cm` band on
each side where the car is squarely on the line and no channel sees it. The
previous code treated any all-dark reading as a lost line and entered the
sweep-and-creep search, which is the wrong response to a 1.8 cm offset.

Resolved by history rather than a timer, because the line can only leave the bar
past an *outer* sensor: `0000` after `0010`/`0100` is the blind band and
steering continues; after `0001`/`1000` it is a real loss; straight from `0110`
it is unreachable by drifting and is treated as paper undulation.

### Junction sequencing without a script

The route is now a continuous loop with no return to the start box, so each lap
passes three junctions. Two of them — the roundabout exit and the T junction —
have identical right-branch signatures that no single reading can separate. One
boolean does it, anchored on the unambiguous symmetric crossbar:

```
1111 sustained         -> in_roundabout = True,  creep -> right 90
0111 sustained + True  -> roundabout exit, in_roundabout = False, creep -> right 90
0111 sustained + False -> T junction, cross straight through
```

`1111` occurs once per lap, so a mis-sequenced lap re-synchronises at the next
entry. An earlier proposal used a four-step counter; it was rejected as
hardcoded, and correctly so — a counter that slips once stays wrong forever.

### Two other numbers were wrong

The module docstring claimed the bar "spans ~10mm" while also describing a
2.4 cm gap — impossible together. Actual span is 64 mm and the outer gap is
2.8 cm. What matters for recovery is neither: it is `gap − line width = 0.8 cm`.

## Verification

```bash
uv run python -m pytest -q     # 445 passed, was 403
uv run ruff check src/ tests/  # clean
```

42 new tests in `tests/test_ir_geometry.py`, including a total-coverage check
over all 16 readings and the contiguity property. `tests/test_ir_line_nav.py`
updated to the measured channel order.

Hardware evidence, all sensor-only:

- Static probe on the line: raw `0101`, 400/400 samples, zero transitions
- Pull-up/pull-down probe: all four channels driven, not floating
- 90-second continuous trace: `1111` on white, `0101` on the line, plus `1011`
  and `0111` occurring for real — 22 sub-0.25 s flickers filtered out
- Sweep: leading and trailing edges gave the same channel order

## Problems Encountered

**Polarity was concluded wrongly from single snapshots.** Readings taken before
and after the operator moved the car were assigned to the wrong conditions,
producing a confident but wrong conclusion that `invert={0,1,2,3}` was stale.
A single continuous trace with the conditions changed *during* it settled it:
`invert={0,1,2,3}` was correct all along. Snapshots taken across a physical
change have no labels and must not be compared.

**"The sensor is broken" was the Pi rebooting.** A static read returned all-LOW
while the pull-up/pull-down diagnostic returned all-HIGH a minute later. The Pi
had rebooted at 11:33. A genuinely failed channel sticks on its own; it does not
take the other three with it. Four channels misbehaving together is a power or
wiring fault.

**The first sweep produced nothing usable** because the car was still parked on
the track line — two channels read black for 118 of 150 seconds and every
derived number was meaningless. The sweep script now refuses to start until it
has seen an all-white baseline held for 2 seconds, so that failure is structural
rather than merely documented.

**Spacing could not be recovered from the sweep.** The operator used a card
wider than the 6.4 cm bar rather than a 2 cm strip, so there was no known scale,
and the hand sweep averaged 0.56 cm/s with visibly uneven speed — leading-edge
intervals were `3.8 / 3.5 / 4.1 s` where the stated `2.8 / 0.8 / 2.8 cm` spacing
predicts roughly `3.5 : 1 : 3.5`. The operator's ruler measurement was kept
rather than overwritten with a worse estimate.

## Follow-up

- **Nothing has been driven under the corrected model.** The wheel speeds
  (110 / 60 / 20 inner) are proposed, not tuned.
- **Raise the sensor bar to ~2 cm.** It sits at ~1 cm, the near edge of the
  1–3 cm detection range, so undulation pushes channels out of range in both
  directions — and out-of-range reads as *black*. Cheapest available fix.
- **`IRTracingSensor.read()` has no filtering.** One sample per channel per
  control cycle, free-running loop, so a single comparator flicker reaches the
  wheels. Oversampling with a majority vote plus per-channel debounce belongs in
  `carbot.ir_tracing`; a stuck-channel check belongs there too, since debounce
  cannot detect a channel that never changes.
- **`docs/task1-single-source-of-truth.md` still ends the route at the start
  box.** Phase 11 should be deleted now that the task is a continuous loop, and
  the T junction documented as crossed straight through.
- **Re-measure the spacing** with a strip narrower than the bar, or accept the
  ruler figures. `examples/42_ir_geometry_sweep.py` is ready either way.

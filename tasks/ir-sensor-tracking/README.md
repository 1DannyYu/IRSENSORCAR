# IR Sensor Tracking — Task 1 Continuous Loop

Drive the Task-1 map using the 4-channel IR tracing sensor only, no camera, and
keep lapping the circuit indefinitely.

Rewritten from scratch on 2026-08-19. The previous plan and quickstart were
deleted because three of their premises turned out to be wrong — the physical
channel order was mirrored, the route ended at the start box instead of looping,
and `0000` was documented as "line lost" when it is usually the opposite. Old
versions are recoverable with `git show b75b740:tasks/ir-sensor-tracking/`.

## Files

| File | What it is |
|---|---|
| [design.md](design.md) | Sensor model, all 16 readings, route, junction sequencing |
| [run-book.md](run-book.md) | Operator procedure from startup through supervised track runs |
| [../../examples/46_map1_ir_transition_drive.py](../../examples/46_map1_ir_transition_drive.py) | One-command bounded Phase 2 -> ARC 1 -> Phase 4 run using the 2026-08-21 trace-derived settings |

Stable facts live outside this directory and are the source of truth:

- [docs/hardware/ir-tracing-sensor.md](../../docs/hardware/ir-tracing-sensor.md)
  — wiring, the position ↔ channel ↔ GPIO table, polarity, blind band
- [src/carbot/ir_geometry.py](../../src/carbot/ir_geometry.py) — the 16-state
  table as executable, unit-tested code
- [docs/task1-single-source-of-truth.md](../../docs/task1-single-source-of-truth.md)
  — the route geometry

## Status

**Driven on the track 2026-08-19.** The corrected channel order works: the car followed the
line, and every steering correction slowed the wheel on the side the line was on. It completed
the lap up to the return T junction, then turned right into the start box instead of crossing.

| Item | State |
|---|---|
| Polarity `invert={0,1,2,3}` | ✅ measured 2026-08-19 |
| Physical order `Out2 Out1 Out3 Out4` | ✅ measured 2026-08-19 |
| Steering direction on a moving car | ✅ driven 2026-08-19 — `0100` slowed the left wheel, `0010` the right |
| 16-state table | ✅ implemented, 42 unit tests |
| Junction sequencing (1 boolean) | ❌ **replaced** — all three of its premises were false on the track, see design.md |
| Junction sequencing (route + distance gate) | ⚠️ implemented and unit-tested, **not yet driven** |
| Bus error leaves the wheels turning | ✅ fixed — retry in `nezha`, best-effort `Car.stop`, loud warning if it cannot confirm |
| Spacing 2.8 / 0.8 / 2.8 cm | ⚠️ operator ruler measurement, not re-measured by sweep |
| Wheel speeds 110 / 60 / 20 | ⚠️ proposed, never driven |
| Roundabout speed | ❌ unknown — R=18cm is tight, may need to drop below 150 |
| Sensor ride height | ❌ ~1cm, at the near limit of the 1–3cm range — should be raised |

## Open Decisions

1. **Raise the sensor bar to ~2 cm.** At 1 cm it sits on the near edge of the
   detection range, so undulating paper pushes channels out of range in both
   directions — and out-of-range reads as *black*. This is the cheapest
   available improvement and should happen before any tuning run.
2. **Route spec still says the car returns to the start box.**
   `docs/task1-single-source-of-truth.md` Phase 11 ends at the garage. The task
   is now a continuous loop, so Phase 11 should be deleted and the T junction
   documented as crossed straight through. That document is labelled the
   authoritative spec, so it needs an explicit edit rather than being quietly
   contradicted here.
3. **Sampling has no filter.** `IRTracingSensor.read()` takes one sample per
   channel per control cycle and the loop is free-running, so a single
   comparator flicker reaches the wheels. A 90-second static trace recorded 22
   sub-0.25s flickers. Oversampling with a majority vote, plus per-channel
   debounce, belongs in `carbot.ir_tracing`.

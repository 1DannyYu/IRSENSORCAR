# Example 47 - Mode-Specific IR 16-State Tables

Bits use physical sensor order `P1 P2 P3 P4`, left to right across the sensor
bar. `1` means black line detected and `0` means no black line detected.

Example 47 has four modes:

1. Phase 1 -> Phase 2
2. Auto tracing
3. Enter roundabout
4. Exit roundabout

Phase 1 -> Phase 2 has no 16-state table because it is deliberately sensor-blind:
the car drives its calibrated forward distance, turns right, and only then enables
sensor control.

The other three modes each have a complete table below. Circle transitions are
sequence-aware, so the tables include both the single-reading action and the
sequence role where applicable.

## 1. Auto tracing mode

Auto tracing follows the canonical state meanings. It never turns right: right-side
drift and right-side junction readings are treated as forward motion.

| State | Canonical meaning | Auto tracing action |
|---|---|---|
| `P0000` | Blind band or line lost | Resolve from previous position; forward or left recovery; search only after 1s continuous `P0000` |
| `P0001` | Far right, outer sensor only | Forward; no right correction |
| `P0010` | Slight right drift | Forward; no right correction |
| `P0011` | Right pair / junction evidence | Forward; no right turn |
| `P0100` | Slight left drift | Strong left correction: `L=0, R=+speed` |
| `P0101` | Non-contiguous noise | Hold/continue forward policy |
| `P0110` | Centred on line | Forward: `L=+speed, R=+speed` |
| `P0111` | Left branch / curve evidence | Strong left correction: `L=0, R=+speed` |
| `P1000` | Far left, outer sensor only | Hard left pivot: `L=-speed, R=+speed` |
| `P1001` | Outer pair only / noise | Hold/continue forward policy |
| `P1010` | Non-contiguous noise | Hold/continue forward policy |
| `P1011` | Non-contiguous noise | Hold/continue forward policy |
| `P1100` | Left pair / junction evidence | Strong left correction: `L=0, R=+speed` |
| `P1101` | Non-contiguous noise | Hold/continue forward policy |
| `P1110` | Left branch / curve evidence | Strong left correction: `L=0, R=+speed` |
| `P1111` | Symmetric crossbar | Forward/hold; no right turn |

## 2. Enter roundabout mode

Enter roundabout mode starts either when elapsed time is at least 25.6 seconds
and the physical reading is `P1110` or `P1111`, or when both `P1110` and `P1111`
are detected within a one-second window. The car then completes the calibrated
right turn into the roundabout. Before that trigger, the car remains in normal
auto tracing.

| State | Canonical meaning | Enter roundabout action |
|---|---|---|
| `P0000` | Blind band or line lost | Normal auto tracing; search only after 1s continuous `P0000` |
| `P0001` | Far right, outer sensor only | Right pivot |
| `P0010` | Slight right drift | Right pivot |
| `P0011` | Right pair / junction evidence | Right pivot |
| `P0100` | Slight left drift | Right pivot |
| `P0101` | Non-contiguous noise | Right pivot |
| `P0110` | Centred on line | Right pivot |
| `P0111` | Left branch / curve evidence | Right pivot; not an entry-sequence step |
| `P1000` | Far left, outer sensor only | Right pivot |
| `P1001` | Outer pair only / noise | Right pivot |
| `P1010` | Non-contiguous noise | Right pivot |
| `P1011` | Non-contiguous noise | Right pivot |
| `P1100` | Left pair / junction evidence | Right pivot |
| `P1101` | Non-contiguous noise | Right pivot |
| `P1110` | Left branch / curve evidence | Right-turn entry trigger after 25.6s, or pair-window step |
| `P1111` | Symmetric crossbar | Right-turn entry trigger after 25.6s, or pair-window step |

## 3. Exit roundabout mode

After entering, the car returns to auto tracing while inside the roundabout. Exit
mode tracks this verified ordered sequence:

```text
P0111 -> P0101 -> P0100 -> P0110
```

Only the complete sequence confirms the exit. The final `P0110` causes one right
turn out of the roundabout; a single `P0110` without the preceding sequence is just
the normal centred state.

| State | Canonical meaning | Exit roundabout action |
|---|---|---|
| `P0000` | Blind band or line lost | Continue auto tracing/recovery; search only after 1s continuous `P0000` |
| `P0001` | Far right, outer sensor only | Forward; no right correction |
| `P0010` | Slight right drift | Forward; no right correction |
| `P0011` | Right pair / junction evidence | Forward; do not confirm exit |
| `P0100` | Slight left drift | Strong left correction; sequence step 3 after `P0111 -> P0101` |
| `P0101` | Non-contiguous noise | Sequence step 2 after `P0111`; hold while tracking |
| `P0110` | Centred on line | Sequence step 4 confirms exit; turn right, then resume tracing |
| `P0111` | Left branch / curve evidence | Sequence step 1; begin exit tracking |
| `P1000` | Far left, outer sensor only | Hard left pivot |
| `P1001` | Outer pair only / noise | If continuous for over 0.2s inside the roundabout: drive 5cm, then turn right 50 degrees (one-shot) |
| `P1010` | Non-contiguous noise | Hold/continue auto-tracing policy |
| `P1011` | Non-contiguous noise | Hold/continue auto-tracing policy |
| `P1100` | Left pair / junction evidence | Strong left correction; not an exit step |
| `P1101` | Non-contiguous noise | Hold/continue auto-tracing policy |
| `P1110` | Left branch / curve evidence | Strong left correction; not the exit step |
| `P1111` | Symmetric crossbar | Forward/hold; not an exit step |

## 4. Phase 1 -> Phase 2

There is intentionally no sensor-state table for this mode. Sensors do not control
the movement during the hardcoded manoeuvre:

1. Drive forward for the configured Phase 1 distance and timing.
2. Stop and turn right 90 degrees.
3. Start sensor-controlled Phase 2 auto tracing.

## Sources

- Canonical meanings: [`src/carbot/ir_geometry.py`](../../src/carbot/ir_geometry.py)
- Mode policy and circle state machine: [`src/carbot/ir_modes.py`](../../src/carbot/ir_modes.py)
- Hardware runner: [`examples/47_ir_three_modes.py`](../../examples/47_ir_three_modes.py)
- Verified route sequences: [`2026-08-20-map1-junction-signal-sequences.md`](../progress/2026-08-20-map1-junction-signal-sequences.md)

Example 47 starts the roundabout entry turn after 25.6 seconds when it detects
`P1110` or `P1111`, or earlier when both readings occur within one second. The
entry turn is about 42.5 degrees; the 90-degree hardcoded turn belongs to
Phase 1 -> Phase 2, not roundabout entry.

# IR Circle Mode - 16-State Tables

This is the mode-specific reference for Example 47. Bits are shown in physical
sensor order `P1 P2 P3 P4`, left to right across the sensor bar. `1` means black
line detected and `0` means no black line detected.

The circle controller is sequence-aware. A single sensor state does not identify
every event by itself. In particular:

- Enter-circle mode becomes active after 22 seconds and uses `P1110` as the
  operator-requested entry trigger.
- After the entry turn, the car uses ordinary auto-tracing inside the roundabout.
- Exit-circle mode recognises the ordered sequence
  `P0111 -> P0101 -> P0100 -> P0110`, then performs the right turn out of the
  roundabout.
- `P1000` is the hard-left state: at speed 150 it commands `L=-150, R=150`.

## Combined 16-state reference

| State | Canonical meaning | Enter-circle mode | Inside / exit-circle mode |
|---|---|---|---|
| `P0000` | Blind band or line lost | Resolve from previous localising state; forward or left recovery | Resolve from previous localising state; hold or recover without a right turn |
| `P0001` | Far right, outer sensor only | Forward-only; do not correct right | Forward-only; do not correct right |
| `P0010` | Slight right drift | Forward-only; do not correct right | Forward-only; do not correct right |
| `P0011` | Right pair / junction evidence | Forward-only; do not take a right turn | Forward-only; do not take a right turn |
| `P0100` | Slight left drift | Strong left correction | Exit sequence step 3 when preceded by `P0111 -> P0101`; otherwise strong left correction |
| `P0101` | Non-contiguous noise | Hold/continue forward policy | Exit sequence step 2 after `P0111`; hold while sequence advances |
| `P0110` | Centred on line | Drive forward | Exit sequence step 4 after `P0111 -> P0101 -> P0100`; confirm exit and turn right |
| `P0111` | Left branch / curve evidence | Auto-trace left; not the entry trigger | Exit sequence step 1; continue auto-tracing while recording the sequence |
| `P1000` | Far left, outer sensor only | Hard left pivot: `L=-speed, R=+speed` | Hard left pivot: `L=-speed, R=+speed` |
| `P1001` | Outer pair only / noise | Hold/continue forward policy | Hold/continue forward policy |
| `P1010` | Non-contiguous noise | Hold/continue forward policy | Hold/continue forward policy |
| `P1011` | Non-contiguous noise | Hold/continue forward policy | Hold/continue forward policy |
| `P1100` | Left pair / junction evidence | Strong left correction | Strong left correction; not an exit-sequence step |
| `P1101` | Non-contiguous noise | Hold/continue forward policy | Hold/continue forward policy |
| `P1110` | Left branch / curve evidence | After 22 seconds: enter circle by turning right; before 22 seconds: strong left curve correction | Strong left curve correction; not the exit-sequence trigger |
| `P1111` | Symmetric crossbar | Forward/hold; not the custom Example 47 entry trigger | Forward/hold; not an exit-sequence step |

## Exit sequence state machine

| Order | Reading | Meaning in circle mode | Action |
|---:|---|---|---|
| 1 | `P0111` | Exit feature first appears | Keep auto-tracing and begin exit-sequence tracking |
| 2 | `P0101` | Exit feature progresses | Hold/continue while advancing the sequence |
| 3 | `P0100` | Exit line moves left across the sensor bar | Apply the strong left correction and advance the sequence |
| 4 | `P0110` | Exit line is centred on the new heading | Confirm exit, turn right, then resume auto-tracing |

## Source and implementation

- Canonical meanings: [`src/carbot/ir_geometry.py`](../../src/carbot/ir_geometry.py)
- Mode policy and sequence state: [`src/carbot/ir_modes.py`](../../src/carbot/ir_modes.py)
- Hardware runner: [`examples/47_ir_three_modes.py`](../../examples/47_ir_three_modes.py)
- Verified route signal notes: [`2026-08-20-map1-junction-signal-sequences.md`](../progress/2026-08-20-map1-junction-signal-sequences.md)

The historical verified route notes describe a separate physical roundabout-entry
sequence (`P1111 -> P1001 -> P0000`). Example 47 currently follows the operator-requested
custom entry trigger `P1110` after 22 seconds; do not confuse the two entry definitions.

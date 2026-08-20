# Yahboom 4-Channel IR Tracing Sensor Integration

This document records the verified wiring and signal convention for the Yahboom
4-channel IR tracing (line-follower) module in `assets/inventory/` (photos
`041`/`042`). **Verified on the real car (2026-08-17)** — tested with
`examples/36_ir_tracing_check.py`.

## Overview

The module is a 4-channel reflective IR line sensor built around an LM339
comparator. Each channel has an IR LED/phototransistor pair and its own
sensitivity potentiometer. It distinguishes a black line on a white surface and
provides four independent TTL digital outputs (`Out1`–`Out4`).

**Detection distance**: ~1–3 cm, adjustable per-channel via the onboard
potentiometers.

## Reading Convention (Applied in Code)

`src/carbot/ir_tracing.py` normalizes every channel to the same convention used
by the rest of the line-following code:

```
normalized 1  ->  black line under the channel
normalized 0  ->  white surface under the channel
```

Raw comparator polarity varies between boards, so the driver takes an
`invert` set of channel indices. The default assumes **raw HIGH = black**
(`BLACK_IS_HIGH = True`); if a channel reads 0 while over black, add its index
to `invert`.

## Verified Wiring (2026-08-17)

This build uses these GPIO pins:

- NeZha driver board: `Pin 3` (SDA), `Pin 4` (5V), `Pin 5` (SCL), `Pin 6` (GND)
- HC-SR04 ultrasonic: `Pin 2` (5V), `Pin 9` (GND), `Pin 11` (GPIO 17, TRIG),
  `Pin 13` (GPIO 27, ECHO)
- **IR tracing sensor (verified):** powered from 3.3V, no level-shifting needed

| Tracing Pin | Raspberry Pi Pin | BCM GPIO | Purpose |
|---|---|---|---|
| **VCC** | Pin 1 | 3.3V | Power supply (safe for 3.3V logic outputs) |
| **GND** | Pin 14 | Ground | Ground reference |
| **Out1** | Pin 18 | GPIO 24 | Channel 1 digital out |
| **Out2** | Pin 22 | GPIO 25 | Channel 2 digital out |
| **Out3** | Pin 15 | GPIO 22 | Channel 3 digital out |
| **Out4** | Pin 16 | GPIO 23 | Channel 4 digital out |

### Physical Position ↔ Channel ↔ GPIO (verified 2026-08-19)

**This table did not exist before 2026-08-19.** The wiring table above records
which GPIO each header pin lands on, but nothing recorded which header pin
belongs to which *physical sensor position* — and they are not in order.

| Position | Sensor label | Channel | BCM GPIO | Pi header pin | Lateral offset |
|---|---|---|---|---|---|
| **Leftmost** | `P1` | `Out2` | GPIO 25 | Pin 22 | −3.2 cm |
| Left-inner | `P2` | `Out1` | GPIO 24 | Pin 18 | −0.4 cm |
| Right-inner | `P3` | `Out3` | GPIO 22 | Pin 15 | +0.4 cm |
| **Rightmost** | `P4` | `Out4` | GPIO 23 | Pin 16 | +3.2 cm |

So the physical left-to-right order is **`Out2, Out1, Out3, Out4`**.

Measured with [examples/42_ir_geometry_sweep.py](../../examples/42_ir_geometry_sweep.py):
a black card swept left to right tripped the channels in that order, and the
card's *trailing* edge released them in the same order — two independent edges
agreeing. The operator separately confirmed `Out4` is the rightmost sensor.

`src/carbot/ir_line_nav.py` previously recorded the order as `Out4, Out3, Out1,
Out2`, read off the potentiometer silkscreen on 2026-08-18. That is the exact
mirror of the measured order, which means every steering correction was being
applied to the wrong side. Corrected on 2026-08-19; the order now lives in
`src/carbot/ir_geometry.PHYSICAL_ORDER`.

### Spacing and the Blind Band

Operator ruler measurement: `P1–P2 = 2.8 cm`, `P2–P3 = 0.8 cm`, `P3–P4 = 2.8 cm`
— the bar spans **6.4 cm**, and the four sensors are **not** evenly spaced. The
route line is 2.0 cm wide.

A 2 cm line cannot cover two sensors 2.8 cm apart, so there is a band where the
line is under the bar but **no channel sees it**:

```
blind band width = gap − line width = 2.8 − 2.0 = 0.8 cm, centred at ±1.8 cm
```

A car sitting squarely on the line reads `0000` inside that band. `0000`
therefore does not mean "line lost". The two cases are separated by the previous
reading, not a timer: the line can only leave the bar past an *outer* sensor, so
`0000` after `0010`/`0100` is the blind band and `0000` after `0001`/`1000` is a
real loss.

Consequences worth knowing before tuning:

- Only **`0110`** can light two sensors from a single line. `1100` and `0011`
  require more than 2.8 cm of black, so on the track they mean a junction, or a
  badly skewed pass over the roundabout curve — never a straight-line offset.
- The warning window between centred (`0110`) and blind is only **0.8 cm** wide
  (`0010` / `0100`), so corrections must be applied immediately at those
  readings.
- Ride height is ~1 cm, the **near** limit of the 1–3 cm detection range.
  Undulating paper pushes sensors out of range in *both* directions, and an
  out-of-range sensor reads the same as black — so surface texture produces
  false **black**, never false white. Raising the bar to ~2 cm, the middle of
  the range, is the cheapest fix available.

### Pin Conflict Resolution (HC-SR04)

The original planned wiring conflicted with the HC-SR04 TRIG/ECHO pins (GPIO 17/27).
The conflict was **resolved by moving the IR tracing sensor to free GPIOs** (GPIO 24/25),
avoiding the need for voltage-level shifting. The 3.3V power rail is used directly.

## Signal Logic

```
raw HIGH (default) -> black line under the channel  -> normalized 1
raw LOW            -> white surface under the channel -> normalized 0
```

Flip a channel with `invert={index}` when its raw polarity is opposite.

## Python Usage

```python
from RPi import GPIO
from carbot.ir_tracing import IRTracingSensor

PINS = (24, 25, 22, 23)  # Out1..Out4 — verified on this build

GPIO.setmode(GPIO.BCM)
for pin in PINS:
    GPIO.setup(pin, GPIO.IN)

sensor = IRTracingSensor(PINS, GPIO, invert={0, 1, 2, 3})  # all 4 inverted — see below
readings = sensor.read()  # (1, 0, 1, 0): 1 = black, 0 = white, in Out order
```

Test on hardware (no motors, safe over SSH):

```bash
PYTHONPATH=src python3 examples/36_ir_tracing_check.py --pins 24,25,22,23 --invert 0,1,2,3
```

**Calibration result (2026-08-17):** All four channels verified with the
potentiometers at their as-received setting:
- Over black surface: 1, 1, 1, 1 ✓
- Over white surface: 0, 0, 0, 0 ✓
- No inversion required

**Recalibration (2026-08-18):** The four sensitivity potentiometers were
manually retuned (fully CW/CCW sweep to find the working range, then set so
that a strong IR return — white paper, high reflectance — reads differently
from a weak return — black line or no surface at all, low/no reflectance).
After retuning, **all four channels flipped polarity together** and now
require `invert={0, 1, 2, 3}` (all channels), replacing the 2026-08-17
no-inversion result:

- Over black line: raw `0, 0, 0, 0` → normalized `1, 1, 1, 1` (with invert) ✓
- Over white paper: raw `1, 1, 1, 1` → normalized `0, 0, 0, 0` (with invert) ✓
- Floating / no surface (e.g. resting on a wood table) reads the same as
  black — expected, since both give a weak/no IR return. This is not a
  fault; it only matters if the sensor is ever truly airborne over the
  track, which does not happen during normal line-following.

**Re-verified 2026-08-19 (still `invert={0, 1, 2, 3}`):** the sensor was
adjusted again between sessions, so polarity was re-measured from scratch rather
than assumed. Three conditions were logged in one continuous trace, and the
operator reported the board LEDs at the same moment:

| Condition | Board LED | Raw GPIO | Meaning |
|---|---|---|---|
| Plain white paper | **dark** | `1111` (HIGH) | strong IR return |
| Black line | **lit** | LOW | weak return |
| Held in the air | **lit** | LOW | no return, same as black |

So `LED lit = LOW = black`, and `invert={0, 1, 2, 3}` remains correct.

Two traps this session walked into, recorded so the next one does not:

- **Single snapshots cannot settle polarity.** Readings taken before and after
  moving the car were assigned to the wrong conditions and produced a confident
  but wrong conclusion. Only a single continuous trace, with the conditions
  changed *during* it, is safe.
- **"All four channels reading nonsense" is a power or wiring fault, not a dead
  sensor.** A run where the static read and the pull-up/pull-down diagnostic
  disagreed within a minute turned out to be the Pi rebooting. A genuinely
  failed channel sticks on its own; it does not take the other three with it.
  Check VCC (Pin 1) and GND (Pin 14) first.

Potentiometer polarity is sensitive to the CW/CCW sweep endpoint, not just a
gradual gain change: turning **all the way CW** pinned every channel HIGH
regardless of surface, and **all the way CCW** pinned every channel LOW
regardless of surface. The working range is between those extremes — verify
with `examples/36_ir_tracing_check.py` after any pot adjustment, since the
polarity is not guaranteed to stay the same across retuning sessions.

Tune each potentiometer with the car powered but wheels lifted, watching the
readout while moving the sensor between black and white.

## Related Files in This Project

- [docs/hardware/raspberry-pi-5-pinout.md](raspberry-pi-5-pinout.md) — Complete GPIO reference and this build's sensor wiring table
- [docs/hardware/hc-sr04-ultrasonic-sensor.md](hc-sr04-ultrasonic-sensor.md) — Ultrasonic sensor (shares planned GPIO 17/27)
- [docs/hardware/ir-obstacle-sensor.md](ir-obstacle-sensor.md) — Single-channel IR obstacle sensor (different module)
- [docs/progress/2026-08-17-ir-tracing-sensor.md](../progress/2026-08-17-ir-tracing-sensor.md) — Work log for this driver
- `assets/inventory/041_Yahboom_4Channel_Tracing_Sensor_*.jpg` — Physical module photos
- `site/src/data/modules.json` — Module catalog entry with the original wiring plan

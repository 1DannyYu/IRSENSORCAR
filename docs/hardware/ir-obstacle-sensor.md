# IR Obstacle Avoidance Sensor Integration

This document records the recommended wiring for the IR obstacle avoidance sensor module in
`assets/inventory/` (photos `050`/`051`, a second unit at `061`/`062`) and how it fits alongside the
already-connected camera and NeZha driver board. **Not yet wired on the real car — this is a
planned configuration, not a verified one.** Update this file to "Verified" once it has been
built and tested on the Pi.

## Overview

The module is an active IR proximity sensor: an IR LED emits modulated light, a phototransistor
picks up the reflection, and an onboard comparator (with a sensitivity potentiometer) turns that
into a clean digital signal. It reads `LOW` when an object is in range and `HIGH` otherwise.

**Pins on the module**: `VCC`, `GND`, `DO` (digital out), `AO` (analog out)
**Detection range**: ~2cm – 30cm, adjustable via the onboard potentiometer
**Detection angle**: ~35° cone
**Output logic**: active-low digital (`DO` goes `LOW` when an obstacle is detected)

## Recommended Wiring (Not Yet Verified)

This build already uses several GPIO pins:

- NeZha driver board: `Pin 3` (SDA), `Pin 4` (5V), `Pin 5` (SCL), `Pin 6` (GND) — see
  [nezha-integration-notes.md](nezha-integration-notes.md)
- HC-SR04 ultrasonic sensor: `Pin 2` (5V), `Pin 9` (GND), `Pin 11` (GPIO 17, TRIG), `Pin 13`
  (GPIO 27, ECHO) — see [hc-sr04-ultrasonic-sensor.md](hc-sr04-ultrasonic-sensor.md)

The IR sensor should use pins none of the above touch:

| IR Sensor Pin | Raspberry Pi Pin | BCM GPIO | Voltage | Purpose |
|---|---|---|---|---|
| **VCC** | Pin 1 | Power | 3.3V | Power supply |
| **GND** | Pin 14 | Ground | 0V | Ground reference |
| **DO** | Pin 15 | GPIO 22 | 3.3V (input) | Digital obstacle signal |
| **AO** | Not connected | — | — | Analog out; the Pi has no built-in ADC (see below) |

### Why 3.3V instead of 5V

The module's datasheet family (same comparator design as the
[Yahboom 4-channel IR tracing sensor](../../site/src/data/modules.json) already in this kit) is
rated `3.3V – 5V`. Powering it from **Pin 1 (3.3V)** instead of a 5V pin makes `DO` swing
`0V–3.3V`, which is directly safe for a Raspberry Pi 5 GPIO input — no voltage divider needed,
unlike the HC-SR04 ECHO line.

If detection range at 3.3V turns out too short in testing, fall back to 5V power (`Pin 2` or
`Pin 4`, whichever is free) and add the same 2.2kΩ/1kΩ divider used for the HC-SR04 ECHO pin
(see [hc-sr04-ultrasonic-sensor.md](hc-sr04-ultrasonic-sensor.md#critical-voltage-level-shifting-for-echo-pin))
between `DO` and GPIO 22.

### AO (analog output)

`AO` is a variable voltage proportional to reflected IR intensity. The Raspberry Pi 5 has no
analog input pins, so `AO` can't be read directly. Leave it disconnected unless an external ADC
(e.g. MCP3008 over SPI) is added later — the digital `DO` pin is sufficient for simple
obstacle-present / obstacle-absent logic.

## Second Unit (Board 2, photos `061`/`062`)

If a second IR sensor is wired (e.g. one facing left, one facing right), give it its own GND and
DO pin — do not share `DO` between two sensors. A free, unused pin at this point is `Pin 16`
(GPIO 23):

| IR Sensor Pin | Raspberry Pi Pin | BCM GPIO |
|---|---|---|
| **VCC** | Pin 1 (3.3V, shared rail) | Power |
| **GND** | Pin 20 | Ground |
| **DO** | Pin 16 | GPIO 23 |

## Signal Logic

```
DO = LOW  -> obstacle within range (increase or decrease with the potentiometer)
DO = HIGH -> no obstacle detected
```

No pulse timing is involved (unlike the HC-SR04) — `DO` is a plain level that can be polled or
watched with an edge interrupt.

## Python Integration Notes (Planned)

```python
import RPi.GPIO as GPIO

IR_PIN = 22  # GPIO 22 (Pin 15)

GPIO.setmode(GPIO.BCM)
GPIO.setup(IR_PIN, GPIO.IN)


def obstacle_detected() -> bool:
    return GPIO.input(IR_PIN) == GPIO.LOW
```

Tune the onboard potentiometer with the car powered but wheels lifted, watching `obstacle_detected()`
output as an object is moved through the desired range.

## Related Files in This Project

- [docs/hardware/raspberry-pi-5-pinout.md](raspberry-pi-5-pinout.md) — Complete GPIO reference
- [docs/hardware/hc-sr04-ultrasonic-sensor.md](hc-sr04-ultrasonic-sensor.md) — Ultrasonic sensor, same "not yet wired" status
- [docs/hardware/nezha-integration-notes.md](nezha-integration-notes.md) — Driver board pin usage (I2C)
- [assets/reference/raspberry-pi-5/car-sensor-wiring-diagram.svg](../../assets/reference/raspberry-pi-5/car-sensor-wiring-diagram.svg) — Full-system wiring diagram (camera, NeZha, HC-SR04, IR sensor)
- `assets/inventory/050_IR_Obstacle_Sensor_Board1_*.jpg`, `061_IR_Obstacle_Sensor_Board2_*.jpg` — Physical module photos

# HC-SR04 Ultrasonic Distance Sensor Integration

This document records the HC-SR04 ultrasonic distance sensor wiring, specifications, and integration notes for obstacle detection in the smart car project.

## Overview

The HC-SR04 is a popular ultrasonic distance sensor that measures distance by emitting an ultrasonic pulse and measuring the time it takes for the echo to return. The sensor is used for real-time obstacle detection to enable autonomous navigation and collision avoidance.

**Model**: HC-SR04  
**Range**: 2cm – 400cm  
**Accuracy**: ±0.3cm  
**Frequency**: 40kHz  

## Verified Wiring Configuration (This Build)

The HC-SR04 is connected to the Raspberry Pi 5 using **GPIO pins** (not I2C). The NeZha driver board does **not** connect to the HC-SR04; they operate independently.

| HC-SR04 Pin | Raspberry Pi Pin | BCM GPIO | Voltage | Purpose |
|---|---|---|---|---|
| **VCC** | Pin 2 | Power | 5V | Power supply |
| **GND** | Pin 6 | Ground | 0V | Ground reference |
| **TRIG** | Pin 11 | GPIO 17 | 3.3V (output) | Pulse trigger (from Pi to sensor) |
| **ECHO** | Pin 13 | GPIO 27 | 5V (input) | Echo pulse (from sensor to Pi) |

### ⚠️ Alternative Wiring for This Build (NeZha I2C in Use)

In this project, the NeZha driver board already occupies **Pin 3 (GPIO 2 / SDA), Pin 4 (5V), Pin 5 (GPIO 3 / SCL), and Pin 6 (GND)** for I2C communication and power (see [nezha-integration-notes.md](nezha-integration-notes.md)). **Do not reuse Pin 4 or Pin 6** for the HC-SR04.

When the NeZha board is wired, use this alternative connection instead — only the **GND** moves (Pin 6 → Pin 9); TRIG/ECHO keep the same BCM GPIOs, so **no code changes are needed**:

| HC-SR04 Pin | Raspberry Pi Pin | BCM GPIO | Voltage | Purpose |
|---|---|---|---|---|
| **VCC** | Pin 2 | Power | 5V | Power supply (single wire from top-right) |
| **GND** | Pin 9 | Ground | 0V | Ground reference |
| **TRIG** | Pin 11 | GPIO 17 | 3.3V (output) | Pulse trigger (from Pi to sensor) |
| **ECHO** | Pin 13 | GPIO 27 | 5V (input) | Echo pulse (from sensor to Pi) |

- **GND, TRIG, ECHO sit together on the left column as consecutive pins 9, 11, 13**; only the 5V line is pulled separately from Pin 2.
- The Echo voltage divider (below) is unchanged, except its `1kΩ → GND` leg connects to the same HC-SR04 GND — use Pin 9 in the alternative wiring.
- `TRIG_PIN = 17` and `ECHO_PIN = 27` in the Python code stay the same.
- The pinout diagram in the next section still shows the default GND on Pin 6; when NeZha occupies Pin 6, use Pin 9 for HC-SR04 GND.

## Critical: Voltage Level Shifting for Echo Pin

⚠️ **The HC-SR04 ECHO pin outputs a 5V pulse, but Raspberry Pi GPIO pins are 3.3V only.**

**Risk**: Connecting 5V directly to GPIO 27 can damage the GPIO pin or the entire Raspberry Pi.

### Recommended Solution: Resistive Voltage Divider

Use a simple 2-resistor divider to step down the 5V ECHO signal to 3.3V:

```
HC-SR04 ECHO (5V) ──┬── 2.2kΩ resistor ──┬── GPIO 27 (3.3V max)
                    │                     │
                    └─ 1kΩ resistor ──┴── Pin 6 (GND)
```

**Calculation**:
- 5V × (1kΩ / (2.2kΩ + 1kΩ)) ≈ 5V × 0.31 ≈ 1.55V ✓ (safe for 3.3V GPIO)

**Parts Needed** (from your 4-in-1 Sensors Pack):
- 1× 2.2kΩ resistor (or 2.2k, ±5% tolerance is fine)
- 1× 1kΩ resistor (or 1k, ±5% tolerance is fine)
- Breadboard and jumper wires for testing

## Signal Timing

### Pulse Sequence

1. **TRIG pulse** (from Pi):
   - Set GPIO 17 to LOW for 2µs
   - Set GPIO 17 to HIGH for 10µs
   - Set GPIO 17 to LOW again
   
2. **ECHO pulse** (from sensor):
   - HC-SR04 sends ultrasonic waves
   - Waits for echo to return
   - Pulls GPIO 27 HIGH when echo is received
   - Holds HIGH for a duration proportional to distance
   - Pulls LOW when echo ends

3. **Distance calculation**:
   ```
   duration = time that GPIO 27 was HIGH (in microseconds)
   distance (cm) = duration × 0.034 / 2
   ```
   - 0.034 = speed of sound (340 m/s) in cm/µs
   - Divide by 2 because the sound travels out and back

## Pinout Reference

```
Raspberry Pi 5 (Front View, USB Ports at Top)

Pin 1  ┌──────────────────────────────────┐  Pin 2
       │ ●● 3.3V Power                   │
       │                                  │
Pin 3  │ ●● GPIO 2 (SDA) │ GPIO 14 (TXD) │  Pin 8
       │                                  │
Pin 5  │ ●● GPIO 3 (SCL) │ GPIO 15 (RXD) │  Pin 10
       │                                  │
Pin 11 │ ●● GPIO 17 ◄─── HC-SR04 TRIG    │  Pin 12
       │ HC-SR04 TRIG input               │
       │                                  │
Pin 13 │ ★●  GPIO 27 ◄─── HC-SR04 ECHO   │  Pin 14
       │ ★ HC-SR04 ECHO (needs divider)  │
       │                                  │
Pin 2  │ ●●  5V Power ─── HC-SR04 VCC    │  (top row, left)
       │ ★ Power supply                  │
       │                                  │
Pin 6  │ ●●  GND ─────── HC-SR04 GND     │  (top-left area)
       │ ★ Ground reference              │
       │                                  │
       └──────────────────────────────────┘

★ = HC-SR04 connection points
```

## Electrical Specifications

| Parameter | Value | Notes |
|---|---|---|
| Operating Voltage (VCC) | 5V | Connect to Pi Pin 2 |
| Operating Current | ~15mA | Typical; keep power supply stable |
| Frequency | 40kHz | Ultrasonic |
| Min Distance | 2cm | Closer readings are unreliable |
| Max Distance | 400cm | In ideal conditions (open air) |
| Measurement Angle | 15° (approx) | ±7.5° from center |
| Trigger Pulse Width | 10µs (nominal) | Set GPIO 17 HIGH for 10µs |
| Echo Pulse Range | 150µs – 25ms | Corresponds to 2.55cm – 425cm |

## Python Integration Notes

### Required Libraries
- `RPi.GPIO` or `gpiozero` for GPIO control (already available on Raspberry Pi OS)
- Standard `time` module for pulse timing

### Example Pseudo-Code
```python
import RPi.GPIO as GPIO
import time

TRIG_PIN = 17  # GPIO 17 (Pin 11)
ECHO_PIN = 27  # GPIO 27 (Pin 13)

GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIG_PIN, GPIO.OUT)
GPIO.setup(ECHO_PIN, GPIO.IN)


def measure_distance():
    # Send trigger pulse
    GPIO.output(TRIG_PIN, GPIO.LOW)
    time.sleep(0.000002)  # 2µs
    GPIO.output(TRIG_PIN, GPIO.HIGH)
    time.sleep(0.00001)  # 10µs
    GPIO.output(TRIG_PIN, GPIO.LOW)

    # Wait for echo start
    while GPIO.input(ECHO_PIN) == GPIO.LOW:
        pulse_start = time.time()

    # Measure echo duration
    while GPIO.input(ECHO_PIN) == GPIO.HIGH:
        pulse_end = time.time()

    # Calculate distance
    pulse_duration = pulse_end - pulse_start
    distance_cm = pulse_duration * 34000 / 2  # Speed of sound ≈ 340 m/s

    return distance_cm
```

### Blocking vs. Non-Blocking Implementation
- **Blocking** (simple): Wait for echo to complete before returning
  - Easier to understand
  - Blocks other tasks during measurement
  - Typical measurement time: 0.1 – 10ms per ping

- **Non-Blocking** (recommended for autonomous car):
  - Use `RPi.GPIO.add_event_detect()` or `gpiozero.MCP3008`
  - Allow other tasks to run while measuring
  - Better for real-time obstacle detection

## Troubleshooting

### Symptom: Distance readings are 0 or very small (< 5cm)
- **Cause 1**: ECHO pin not receiving signal (likely voltage divider issue)
- **Cause 2**: Sensor not powered correctly (check Pin 2 for 5V)
- **Solution**: Verify voltage divider resistors with a multimeter

### Symptom: Distance readings unstable or jumping
- **Cause**: Reflections from nearby objects, or sensor vibrating
- **Solution**: Take multiple readings and average, add small delay between pings

### Symptom: GPIO 27 pin gets hot or smells burnt
- **Cause**: 5V ECHO signal connected directly without voltage divider (critical!)
- **Solution**: STOP immediately and reconnect with proper voltage divider

## References

- HC-SR04 Datasheet: [Available from supplier or Waveshare docs](https://www.waveshare.com/wiki/Ultrasonic_Sensor)
- Raspberry Pi GPIO documentation: `man gpio` or https://www.raspberrypi.com/documentation/computers/gpio.html
- Speed of sound in air: 340 m/s (20°C)

## Related Files in This Project

- [docs/hardware/raspberry-pi-5-pinout.md](raspberry-pi-5-pinout.md) — Complete GPIO reference
- [docs/hardware/nezha-i2c-protocol.md](nezha-i2c-protocol.md) — Driver board commands (separate from HC-SR04)
- [docs/hardware/ir-obstacle-sensor.md](ir-obstacle-sensor.md) — IR obstacle sensor, wired to avoid these pins
- [assets/reference/raspberry-pi-5/car-sensor-wiring-diagram.svg](../../assets/reference/raspberry-pi-5/car-sensor-wiring-diagram.svg) — Full-system wiring diagram
- `src/carbot/ultrasonic.py` — Python driver module (to be implemented)
- `examples/06_ultrasonic_avoidance.py` — Usage example (to be implemented)

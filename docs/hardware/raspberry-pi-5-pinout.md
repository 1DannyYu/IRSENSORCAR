# Raspberry Pi 5 40-Pin GPIO Pinout Guide

This document provides a practical reference for the Raspberry Pi 5 40-pin GPIO header, including
the full pin mapping table, grouped interface categories, and wiring notes relevant to this project.

## Reference Diagram

![Raspberry Pi 5 GPIO Pinout](../../assets/reference/raspberry-pi-5/gpio-pinout-diagram.png)

## Before You Start

- Raspberry Pi 5 GPIO logic is **3.3V**.
- **Do not apply 5V directly to GPIO signal pins.**
- The header has **40 physical pins** arranged in two columns.
- This guide uses **BCM GPIO names** such as `GPIO 2` and `GPIO 14`.

## Pin Numbering Orientation

When the Raspberry Pi 5 is viewed from the top, with the 40-pin header on the right:

- The left column contains odd-numbered pins: `1, 3, 5, ..., 39`
- The right column contains even-numbered pins: `2, 4, 6, ..., 40`

## Full 40-Pin Mapping Table

| Physical Pin | Function | Physical Pin | Function |
|---|---|---|---|
| 1 | 3.3V Power | 2 | 5V Power |
| 3 | GPIO 2 (SDA) | 4 | 5V Power |
| 5 | GPIO 3 (SCL) | 6 | Ground (GND) |
| 7 | GPIO 4 (GPCLK0) | 8 | GPIO 14 (TXD) |
| 9 | Ground (GND) | 10 | GPIO 15 (RXD) |
| 11 | GPIO 17 | 12 | GPIO 18 (PCM_CLK) |
| 13 | GPIO 27 | 14 | Ground (GND) |
| 15 | GPIO 22 | 16 | GPIO 23 |
| 17 | 3.3V Power | 18 | GPIO 24 |
| 19 | GPIO 10 (MOSI) | 20 | Ground (GND) |
| 21 | GPIO 9 (MISO) | 22 | GPIO 25 |
| 23 | GPIO 11 (SCLK) | 24 | GPIO 8 (CE0) |
| 25 | Ground (GND) | 26 | GPIO 7 (CE1) |
| 27 | GPIO 0 (ID_SD) | 28 | GPIO 1 (ID_SC) |
| 29 | GPIO 5 | 30 | Ground (GND) |
| 31 | GPIO 6 | 32 | GPIO 12 (PWM0) |
| 33 | GPIO 13 (PWM1) | 34 | Ground (GND) |
| 35 | GPIO 19 (PCM_FS) | 36 | GPIO 16 |
| 37 | GPIO 26 | 38 | GPIO 20 (PCM_DIN) |
| 39 | Ground (GND) | 40 | GPIO 21 (PCM_DOUT) |

## Pin Groups by Function

### Power Pins

| Type | Pins |
|---|---|
| 3.3V | `1`, `17` |
| 5V | `2`, `4` |
| GND | `6`, `9`, `14`, `20`, `25`, `30`, `34`, `39` |

### I2C Pins

| Interface | Pins |
|---|---|
| SDA | `Pin 3` = `GPIO 2` |
| SCL | `Pin 5` = `GPIO 3` |

#### Difference Between Pin 3 and Pin 5

- `Pin 3` is `GPIO 2 (SDA)`, the I2C data line.
- `Pin 5` is `GPIO 3 (SCL)`, the I2C clock line.
- In typical wiring, a module `SDA` pin connects to Raspberry Pi `Pin 3`.
- In typical wiring, a module `SCL` pin connects to Raspberry Pi `Pin 5`.

Common uses:

- I2C sensors
- OLED displays
- IMU and environmental sensor modules

### UART Pins

| Interface | Pins |
|---|---|
| TXD | `Pin 8` = `GPIO 14` |
| RXD | `Pin 10` = `GPIO 15` |

Common uses:

- Serial communication
- Connecting to microcontrollers
- Debug console access

### SPI Pins

| Interface | Pins |
|---|---|
| MOSI | `Pin 19` = `GPIO 10` |
| MISO | `Pin 21` = `GPIO 9` |
| SCLK | `Pin 23` = `GPIO 11` |
| CE0 | `Pin 24` = `GPIO 8` |
| CE1 | `Pin 26` = `GPIO 7` |

Common uses:

- ADC and DAC modules
- High-speed sensors
- Displays and storage devices

### PWM Pins

| PWM | Pins |
|---|---|
| PWM0 | `Pin 32` = `GPIO 12` |
| PWM1 | `Pin 33` = `GPIO 13` |

Common uses:

- Servo motor control
- LED brightness control
- Motor driver signal output

### PCM / I2S Pins

| Interface | Pins |
|---|---|
| PCM_CLK | `Pin 12` = `GPIO 18` |
| PCM_FS | `Pin 35` = `GPIO 19` |
| PCM_DIN | `Pin 38` = `GPIO 20` |
| PCM_DOUT | `Pin 40` = `GPIO 21` |

Common uses:

- Digital audio devices
- I2S microphones
- Audio codecs

## Common General-Purpose GPIO Pins

These pins are commonly used as plain GPIO input and output:

`GPIO 4`, `GPIO 17`, `GPIO 27`, `GPIO 22`, `GPIO 23`, `GPIO 24`, `GPIO 25`, `GPIO 5`, `GPIO 6`, `GPIO 16`, `GPIO 26`

## Wiring Notes

1. **Do not connect a 5V signal directly to a GPIO pin.**
2. If an external board feeds 5V power into the Raspberry Pi, use **Pin 2** or **Pin 4**.
3. **Do not feed 5V into Pin 1 or Pin 17**, because those are 3.3V rails.
4. Confirm whether your sensor or module expects `3.3V` or `5V`.
5. For motors, relays, and servos, use a driver board or external power supply instead of driving them directly from GPIO.
6. If hardware does not respond, check pin numbering, shared ground, and whether the required interface is enabled.

## Quick Reference

- `3.3V`: `1`, `17`
- `5V`: `2`, `4`
- `GND`: `6`, `9`, `14`, `20`, `25`, `30`, `34`, `39`
- `I2C`: `3 (SDA)`, `5 (SCL)`
- `UART`: `8 (TXD)`, `10 (RXD)`
- `SPI`: `19`, `21`, `23`, `24`, `26`
- `PWM`: `32`, `33`

## This Project's Sensor Wiring

Per-device wiring, chosen to avoid pin conflicts between all sensors on this build:

| Device | Pins Used | Status | Docs |
|---|---|---|---|
| NeZha driver board (I2C, `0x40`) | 3, 4, 5, 6 | Connected | [nezha-integration-notes.md](nezha-integration-notes.md) |
| AI camera (IMX500) | CSI port, no GPIO | Connected | [ai-camera.md](ai-camera.md) |
| HC-SR04 ultrasonic | 2, 9, 11, 13 | Verified (2026-08) | [hc-sr04-ultrasonic-sensor.md](hc-sr04-ultrasonic-sensor.md) |
| IR obstacle sensor | 1, 14, 15 | Not yet wired | [ir-obstacle-sensor.md](ir-obstacle-sensor.md) |
| IR tracing sensor (4-channel) | 1, 14, 18, 22, 15, 16 | Verified (2026-08-17) | [ir-tracing-sensor.md](ir-tracing-sensor.md) |

Full-system diagram: [assets/reference/raspberry-pi-5/car-sensor-wiring-diagram.svg](../../assets/reference/raspberry-pi-5/car-sensor-wiring-diagram.svg)
Editable schematic: [assets/reference/raspberry-pi-5/car-sensor-wiring-diagram.cddx](../../assets/reference/raspberry-pi-5/car-sensor-wiring-diagram.cddx) — import at [circuit-diagram.org/editor/open](https://www.circuit-diagram.org/editor/open)

## File Information

- Image source: `GPIO-Pinout-Diagram-2.png`
- Purpose: Raspberry Pi 5 GPIO reference and wiring guide

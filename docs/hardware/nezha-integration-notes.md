# NeZha Driver Board + Raspberry Pi Integration Notes

This document records the currently verified wiring, power observations, I2C detection results, and open questions for the Raspberry Pi + NeZha driver board setup used in this car project.

## Purpose

This note is intended to separate:

- hardware facts that have already been verified on the real system
- practical recommendations based on current testing
- items that still need confirmation from official documentation

## Current Setup Summary

| Item | Current Status |
|---|---|
| Main computer | Raspberry Pi 5 |
| Driver board | NeZha bus driver board |
| Wheel motors | Connected to the four motor ports on the driver board |
| Driver board to Raspberry Pi communication | I2C |
| **Raspberry Pi Pins Used** | **Pin 3, Pin 4, Pin 5, Pin 6** |
| Raspberry Pi I2C pins | `Pin 3` = `SDA`, `Pin 5` = `SCL` |
| Raspberry Pi power input from driver board | `Pin 4` (5V), `Pin 6` (GND) |

## Verified Facts

The following items were directly observed or measured on the real device:

### 1. Raspberry Pi Pin Usage

**Wiring Configuration (This Build):**

| Raspberry Pi Pin | BCM GPIO | NeZha Connection | Purpose |
|---|---|---|---|
| Pin 3 | GPIO 2 | SDA | I2C data line (to driver board) |
| Pin 4 | 5V | Power | 5V power (from driver board) |
| Pin 5 | GPIO 3 | SCL | I2C clock line (to driver board) |
| Pin 6 | GND | Ground | Ground reference (from driver board) |

**Key Notes:**
- `Pin 3` is `GPIO 2 (SDA)` and is used for I2C data.
- `Pin 5` is `GPIO 3 (SCL)` and is used for I2C clock.
- `Pin 4` is `5V Power` — power supplied by the driver board.
- `Pin 6` is `GND` — ground reference from the driver board.
- Therefore:
  - use `Pin 4` (not Pin 2) for the `5V` power input in this configuration
  - use `Pin 6` for ground reference
  - never connect `5V` power to `Pin 1`, `Pin 17`, or regular GPIO signal pins

### 2. I2C Detection Result

The Raspberry Pi successfully detected an I2C device at address `0x40`.

Example command:

```bash
sudo i2cdetect -y 1
```

Observed result:

```text
40: 40
```

Interpretation:

- the Raspberry Pi I2C bus is working
- the driver board is responding on address `0x40`
- communication at the scan level is successful

### 3. Power Health Checks

Measured commands:

```bash
vcgencmd pmic_read_adc EXT5V_V
vcgencmd get_throttled
```

Observed values:

| Check | Result | Interpretation |
|---|---|---|
| `EXT5V_V` | `4.856V` | Normal |
| `EXT5V_V` | `4.916V` | Normal |
| `EXT5V_V` | `4.809V` | Normal |
| `get_throttled` | `0x0` | No undervoltage, throttling, or overheating flags |

Conclusion:

- the Raspberry Pi power rail appears stable during these measurements
- the voltage fluctuated between about `4.81V` and `4.92V`, which is acceptable
- the measured values remain above the commonly used `4.8V` comfort threshold
- they are also above the typical undervoltage warning region near `4.63V`

## Recommended Practice

Based on the current setup and measurements:

1. **Keep the current wiring configuration:**
   - `Pin 3` (GPIO 2 / SDA) to NeZha SDA
   - `Pin 4` (5V) to NeZha 5V Power
   - `Pin 5` (GPIO 3 / SCL) to NeZha SCL
   - `Pin 6` (GND) to NeZha GND
2. Re-check `EXT5V_V` after the battery has been in use for a while.
3. If voltage trends downward toward `4.63V`, save work and shut down cleanly.
4. Before writing motor-control Python code, identify the driver board's I2C command format or official SDK.
5. **Do not connect additional I2C devices to Pin 3 and Pin 5** without checking for address conflicts (NeZha uses `0x40`).

## Important Clarification

Two different "5V" situations were discussed and should not be confused:

### Safe Case

- External `5V` power feeding the Raspberry Pi power rail through `Pin 2` or `Pin 4`

### Unsafe Cases

- sending `5V` into regular GPIO signal pins
- sending `5V` into `Pin 1` or `Pin 17` (`3.3V` power pins)

## Battery Charging While Powering the Raspberry Pi

The battery pack is currently connected to AC power while also supplying power to the Raspberry Pi through the driver board.

### What is currently known

- the Raspberry Pi is running normally during measurement
- the voltage readings are currently acceptable
- `throttled=0x0` indicates no active power-related warning flags

### What is not yet confirmed

The official specifications for the exact battery pack and driver board power-path behavior have not yet been confirmed in this workspace.

That means the following is still unknown:

- whether the board officially supports simultaneous charging and output
- whether its charging and output paths are isolated properly
- whether long-term operation in this mode is recommended by the vendor

### Practical Recommendation

- If official documentation confirms support for simultaneous charging and output, continue using it according to spec.
- If official documentation is unclear, treat this mode as unverified.
- For the most conservative setup, power the Raspberry Pi from a known-good dedicated supply and let the battery system handle only the motor/driver side if needed.

## What Is Verified vs. What Still Needs Verification

### Verified

- Raspberry Pi pin roles for `3.3V`, `5V`, `GND`, `SDA`, and `SCL`
- I2C device detected at address `0x40`
- Power readings currently look healthy
- No throttling flags are present at the time of measurement

### Still To Verify

- NeZha driver board official I2C protocol / command format
- Whether an official Python SDK exists for Raspberry Pi use
- Whether the driver board officially supports charging while powering the load
- Whether the exact battery and board combination is designed for pass-through power usage

## Suggested Next Steps

1. Find the official NeZha I2C control protocol or SDK.
2. Write a minimal Python script that only tests communication with address `0x40`.
3. Add a very small motor movement test after the command format is confirmed.
4. Continue monitoring voltage during longer battery-powered runs.

# NeZha Driver Board I2C Protocol

This document records the I2C command set for the Yourfun NeZha bus driver board.

**Sources:** vendor STM32, Arduino, and C51 driver code under `vendor/yourfun-nezha/sdk/`, plus
the official user manual in `vendor/yourfun-nezha/manual/` (V1.0.0, 2023-11-27). The vendor does
not provide a standalone protocol specification, so this file is reconstructed from the shipped
code. That vendor material has since been removed from the working tree — see
[ADR 0004](../adr/0004-nezha-python-driver-port.md) for a permalink to it and the full
reconstruction rationale.

The Python implementation lives in [`src/carbot/nezha.py`](../../src/carbot/nezha.py). The decision
to reimplement the vendor C SDK in Python, and the translation choices it required, are recorded in
[ADR 0004](../adr/0004-nezha-python-driver-port.md).

## Addressing

| Item | Value |
|---|---|
| 7-bit I2C address (Linux / `i2cdetect`) | **`0x40`** |
| Vendor `NEZHA_ADDR` | `0x80` (8-bit write address, where `0x80 >> 1 == 0x40`) |
| Read address | `0x81` (`NEZHA_ADDR \| 0x01`) |

`0x40` is also the default PCA9685 address. If a PCA9685-based HAT is connected at the same time,
there will be an address conflict unless the HAT jumpers are changed.

## Transfer Format

Every command is first written to command register `0x00`. Commands with arguments then send a
second data frame whose first byte is the command itself.

```text
No-argument command:
  START  0x80  0x00  <cmd>  STOP

Command with arguments:
  START  0x80  0x00  <cmd>  STOP
  START  0x80  <cmd>  <data...>  STOP

Read command:
  START  0x80  <cmd>  RESTART  0x81  <hi>  <lo>  STOP
```

Do not merge the two writes into a single transfer. The vendor implementation explicitly sends them
in two steps.

SMBus equivalents:

| Action | `smbus2` call |
|---|---|
| No-argument command | `write_byte_data(0x40, 0x00, cmd)` |
| Command with arguments | `write_byte_data(...)`, then `write_i2c_block_data(0x40, cmd, data)` |
| Read | `read_i2c_block_data(0x40, cmd, 2)` |

## Command Table

### System

| Command | Code | Notes |
|---|---|---|
| Reset | `0xFF` | Wait **100ms** after sending |
| Motor init | `0x01` | No arguments |

### Motors M1-M4

| Port | Set speed |
|---|---|
| M1 | `0x05` |
| M2 | `0x09` |
| M3 | `0x0D` |
| M4 | `0x11` |

Arguments are 4 bytes: `[motor_a_hi, motor_a_lo, motor_b_hi, motor_b_lo]`

- Each channel accepts **0-1000**, where duty cycle is `value / 1000`
- `motor_a` and `motor_b` must not both be non-zero at the same time
- PWM frequency is fixed at 100Hz in firmware

### Encoders

| Port | Init | Read |
|---|---|---|
| M1 | `0x15` | `0x16` |
| M2 | `0x18` | `0x19` |
| M3 | `0x1B` | `0x1C` |
| M4 | `0x1E` | `0x1F` |

Reads return a signed big-endian 16-bit value. Sampling period is fixed at 20ms in firmware. Each
encoder must be initialized before reading.

### Servos S1-S4

| Port | Init | Set PWM |
|---|---|---|
| S1 | `0x21` | `0x22` |
| S2 | `0x24` | `0x25` |
| S3 | `0x27` | `0x28` |
| S4 | `0x2A` | `0x2B` |

Arguments are 2 bytes: `[pwm_hi, pwm_lo]`

- Valid PWM range is **50-250**
- This maps linearly to 0-180 degrees for standard hobby servos
- Each servo port must be initialized individually before use

Servo pin order from outside to inside is `GND / 5V / Sx signal`.

### LEDs

| LED | On | Off | Toggle |
|---|---|---|---|
| Head LED | `0x2D` | `0x2E` | `0x2F` |
| Left tail LED | `0x30` | `0x31` | `0x32` |
| Right tail LED | `0x33` | `0x34` | `0x35` |
| Ambient LED | `0x36` | `0x37` | `0x38` |

## Raspberry Pi Integration Hazards

### 1. I2C Speed Must Stay at or Below 200kHz

Vendor code in `NeZha_I2C.c` explicitly states that the NeZha board must not be driven above
200kHz. Raspberry Pi defaults to 100kHz, which is correct. Do not override it with
`dtparam=i2c_arm_baudrate=400000`.

### 2. Keep the Required Power-On Delays

The vendor code marks two delays as mandatory:

- **500ms** after I2C initialization
- **100ms** after reset

### 3. Vendor Code Does Not Handle ACK Normally

The vendor implementation does not validate ACK behavior in the standard way, and even acknowledges
the final byte in reads. Raspberry Pi hardware I2C does validate ACKs. If the address appears in
`i2cdetect` but data transfers fail with:

```text
OSError: [Errno 121] Remote I/O error
```

this mismatch is a likely cause. A fallback is software I2C:

```text
dtoverlay=i2c-gpio,bus=3,i2c_gpio_sda=23,i2c_gpio_scl=24
```

### 4. Do Not Power the Raspberry Pi Twice

The manual warns against powering the main controller from the NeZha board while also feeding it
from another source. If the NeZha `5V` rail is connected to Raspberry Pi `Pin 2` or `Pin 4`, do
not also connect USB-C power.

## Direction Semantics Are Contradictory

The vendor manual and vendor source comments disagree about the meaning of `motor_a` and `motor_b`:

| Source | Claim |
|---|---|
| Manual, page 13 | `motor_a` drives forward, `motor_b` drives reverse |
| `NeZha.c` comments | `motor_a` is reverse, `motor_b` is forward |

The byte order is consistent across both sources. The disagreement is only in interpretation, which
means the real hardware must be tested. Run `examples/02_motor_check.py` and flip
`FORWARD_IS_MOTOR_A` in `src/carbot/nezha.py` only if the observed directions do not match.

## Connector Quick Reference

- **12V battery input**: must receive 12V; reversed polarity can damage the board
- **I2C header** from right to left: `5V / SCL / SDA / GND`
- **Motor ports x4**: each has a 2-pin motor side and a 4-pin encoder side
- **Servo headers x4**: do not use the position marked `NC`
- **5V power headers x3**: for external peripherals
- **Firmware LED**: solid on indicates working firmware

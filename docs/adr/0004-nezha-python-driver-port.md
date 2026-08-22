# ADR 0004: Port the Vendor NeZha SDK to a Python Driver on the Raspberry Pi

- **Status**: adopted, hardware-verified 2026-07-30
- **Date**: 2026-07-30 (recorded retroactively 2026-08-19)

## Context

The Yourfun NeZha bus driver board ships with driver code for three microcontroller targets and
nothing else:

| Target | Files |
|---|---|
| Arduino | [`vendor/yourfun-nezha/sdk/arduino/`](https://github.com/1DannyYu/IRSENSORCAR/tree/1da6a8e/vendor/yourfun-nezha/sdk/arduino) — `NeZha.cpp` (575 lines), `NeZha.h`, `NeZha_I2C.cpp`, `NeZha_I2C.h` |
| STM32 | [`vendor/yourfun-nezha/sdk/stm32/`](https://github.com/1DannyYu/IRSENSORCAR/tree/1da6a8e/vendor/yourfun-nezha/sdk/stm32) — same API, different bit-bang layer |
| C51 | [`vendor/yourfun-nezha/sdk/c51/`](https://github.com/1DannyYu/IRSENSORCAR/tree/1da6a8e/vendor/yourfun-nezha/sdk/c51) — same again |

(`vendor/yourfun-nezha/` was removed from the working tree once the facts below were fully
captured; the links above point at the last commit that still had it, `1da6a8e`.)

Three constraints applied:

1. **The host is a Raspberry Pi 5.** There is no Arduino or STM32 in this build, and adding one as
   an I2C-to-serial bridge would put a second microcontroller, its firmware, and its own failure
   modes between the navigation code and the wheels.
2. **There is no protocol specification.** The vendor manual
   (`vendor/yourfun-nezha/manual/`, V1.0.0, 2023-11-27) documents the board's features, not its
   wire format. The command set exists only as `#define`s inside the three SDKs.
3. **`vendor/` is read-only** (CLAUDE.md hard rule 1), so the vendor sources cannot be patched into
   a Pi-compatible shape in place.

The board also had to be driven from Python because the rest of the stack — camera, AprilTag pose,
line detection — is Python on the Pi.

## Decision

**Reconstruct the I2C protocol by cross-reading all three vendor SDKs, then reimplement it as a
Python driver.** No vendor file was translated line by line, and no Arduino compatibility layer was
introduced.

The port produced four artifacts, all committed on 2026-07-30:

| Artifact | Commit | Role |
|---|---|---|
| [`docs/hardware/nezha-i2c-protocol.md`](../hardware/nezha-i2c-protocol.md) | `c33f8dd` | The reconstructed command reference — the protocol source of truth for this project |
| [`src/carbot/nezha.py`](../../src/carbot/nezha.py) | `d5b2231` | Board driver: 4 motors, 4 servos, 4 encoder channels, 4 LEDs |
| [`src/carbot/car.py`](../../src/carbot/car.py) | `3bb0178` | Differential-drive layer (no vendor counterpart — see below) |
| [`src/carbot/servo.py`](../../src/carbot/servo.py) | `a3f7254`, extracted in `eb7f86e` | Robotic-arm servo check |

Supporting: [`src/carbot/config.py`](../../src/carbot/config.py) (wheel mapping),
[`tests/test_nezha.py`](../../tests/test_nezha.py), and `examples/01`–`04`.

### Translation decisions

1. **Address `0x80` → `0x40`.** The vendor's `NEZHA_ADDR` is the 8-bit write address. Linux
   `smbus2` and `i2cdetect` take the 7-bit address, so the driver uses `0x80 >> 1 == 0x40`. This is
   also the PCA9685 default address, which is why the address clash is a standing project hazard.
2. **Software bit-bang → Pi hardware I2C.** `NeZha_I2C.cpp` in its entirety —
   `Start`/`Stop`/`SendByte`/`ReadByte`/`ACK`/`NACK`/`SdaDir`, plus the microsecond delay loops —
   was dropped. The Pi drives `/dev/i2c-1` through `smbus2`. Two behaviours had to be preserved
   across that swap: the bus must stay at or below **200 kHz** (Pi defaults to 100 kHz, which is
   correct — do not raise it), and the **500 ms post-init / 100 ms post-reset** delays the vendor
   code marks as mandatory are kept as `INIT_DELAY_S` and `RESET_DELAY_S`.
3. **Per-channel functions collapsed into channel arguments.** The vendor exposes
   `NeZha_Motor1_SetPwm()`…`Motor4`, `Servo1_Init()`…`Servo4`, and twelve separate LED functions.
   The Python driver takes the channel as a parameter — `motor(n, speed)`, `servo(n, angle)`,
   `led(name, state)` — with the command codes held as tuples and a dict
   (`CMD_MOTOR_SET = (0x05, 0x09, 0x0D, 0x11)`, `CMD_LED`). Channels stay **1-4** to match the
   `M1`-`M4` / `S1`-`S4` silkscreen rather than becoming 0-based.
4. **Signed speed instead of an `(motor_a, motor_b)` pair.** The vendor API takes two PWM values
   and the manual marks both-non-zero as invalid. `motor(n, speed)` takes one value in -1000…1000
   and derives the pair, so the invalid state is unrepresentable.
5. **`FORWARD_IS_MOTOR_A` is an explicit constant.** Manual P.13 and the vendor source comments
   contradict each other on which channel is forward. Rather than pick one silently, the choice is
   a single named constant to be settled by a lifted-wheel test (it was; see below).
6. **The two-frame transfer is preserved.** Every command is written to register `0x00` first, and
   commands with arguments then send a second frame starting with the command byte. The vendor does
   this in two explicit transfers, so `_command_with_data()` does too — they are not merged.
7. **`car.py` was written, not ported.** `NeZha_Forward/Backward/TurnLeft/TurnRight/TransLeft/
   TransRight` are **declared in all three vendor headers and implemented in none of the vendor
   source files**. The differential-drive layer therefore had no original to follow. `Car` computes
   the two sides from `config.WHEEL_TO_MOTOR` and `config.INVERTED_MOTORS`, so a rewired chassis is
   a config change, not a driver change.
8. **Safety behaviour has no vendor equivalent and was added deliberately.** `Car.move_for()` stops
   in a `finally` block so a `Ctrl-C` or an exception cannot leave the car driving; `NeZha` and
   `Car` are both context managers; `Car.stop(best_effort=True)` keeps trying the remaining motors
   after one fails and warns the operator if any refuses.

### Verification strategy

Because there is no official protocol document to check against, the tests assert the **exact byte
sequence put on the bus**, not the shape of the Python API — a `FakeBus` records every
`write_byte_data` / `write_i2c_block_data` / `read_i2c_block_data` call and the tests compare those
tuples against what the vendor C emits for the same operation. That is the only available way to
prove the reimplementation is faithful.

Hardware confirmation followed the same day: see
[`docs/progress/2026-07-30-first-drive.md`](../progress/2026-07-30-first-drive.md) — I2C probe at
`0x40`, all four motor ports mapped to real wheel positions, six lifted movements correct, then a
0.5 s ground run. That test settled item 5 (`FORWARD_IS_MOTOR_A` stayed `True`, because only M2 and
M3 needed inversion) and populated `config.INVERTED_MOTORS`.

## Consequences

- **The protocol document is a reconstruction, not a specification.** Anything not exercised by
  `examples/01`–`04` is inferred from vendor code. Encoder reads in particular are untested on this
  build — it uses two-wire DC motors and `config.HAS_ENCODERS` is `False`.
- **Vendor SDK updates require manual re-comparison.** There is no build-time link between
  `vendor/` and `src/carbot/nezha.py`; if Yourfun ships a new SDK, the command tables and the
  byte-sequence tests have to be re-diffed by hand.
- **The `0x40` address clash is permanent.** A PCA9685-based HAT cannot share this bus without
  changing its jumpers.
- **Pi hardware I2C validates ACKs; the vendor bit-bang did not** (it even ACKs the final byte of a
  read). Marginal wiring that a bit-bang master tolerated shows up here as
  `OSError: [Errno 121] Remote I/O error`. This is the reason `WRITE_RETRIES` was later added
  (`88c38d1`, 2026-08-19) after errno 121 killed a line-follow run and its stop path with it.
- **The vendor's high-level movement API is not available and will not be added.** Callers use
  `Car`; anything reaching for `NeZha_Forward`-style helpers should extend `car.py` instead.

## Links

- Vendor sources (removed from the working tree, see note above): [`vendor/yourfun-nezha/sdk/`](https://github.com/1DannyYu/IRSENSORCAR/tree/1da6a8e/vendor/yourfun-nezha/sdk)
- Protocol reference: [`docs/hardware/nezha-i2c-protocol.md`](../hardware/nezha-i2c-protocol.md)
- Wiring and power facts:
  [`docs/hardware/nezha-integration-notes.md`](../hardware/nezha-integration-notes.md)
- Bring-up procedure:
  [`docs/setup/raspberry-pi-first-run.md`](../setup/raspberry-pi-first-run.md)
- Hardware verification:
  [`docs/progress/2026-07-30-first-drive.md`](../progress/2026-07-30-first-drive.md)

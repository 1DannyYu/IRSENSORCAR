# Raspberry Pi First Bring-Up

This guide goes from zero to verified wheel movement. Every step has a concrete check. Do not skip steps.

Prerequisite: Raspberry Pi 5 is running Raspberry Pi OS and has network access.

---

## 0. Verify Wiring Before Power-On

| Check | Correct wiring |
|---|---|
| Driver board power | Connect **12V** to the 12V battery input. Reversed polarity can destroy the board. |
| Raspberry Pi 5V input | Connect the board `5V` pin to Raspberry Pi **Pin 2** or **Pin 4** |
| No dual power | If NeZha powers the Pi, do **not** also connect Raspberry Pi USB-C power |
| SDA | Board `SDA` -> Raspberry Pi **Pin 3** |
| SCL | Board `SCL` -> Raspberry Pi **Pin 5** |
| GND | Board `G` -> any Raspberry Pi ground pin, with a shared ground |
| Motor plug | Two-wire DC motors go into the **2-pin** motor connector, not the 4-pin encoder side |

After power-on, both the power LED and firmware LED on the board should remain on. If the power LED
turns off, disconnect power immediately and inspect for a short.

## 1. Enable I2C

```bash
sudo raspi-config
```

Go to `Interface Options` -> `I2C` -> `Yes`, then reboot.

Install the tools and confirm that the board responds:

```bash
sudo apt update && sudo apt install -y i2c-tools
sudo i2cdetect -y 1
```

You should see `40`. If not, go back to Step 0 and re-check power and wiring.

Do not set `dtparam=i2c_arm_baudrate=400000` in `/boot/firmware/config.txt`. The NeZha board is
specified for up to 200kHz, and Raspberry Pi's default 100kHz is correct.

Add your user to the `i2c` group so later commands do not require `sudo`:

```bash
sudo usermod -aG i2c $USER && echo "Log out and back in for this to take effect"
```

## 2. Clone the Repository onto the Raspberry Pi

```bash
cd ~ && git clone https://github.com/1DannyYu/IRSENSORCAR.git Car-and-Robotic-Arm && cd Car-and-Robotic-Arm
```

## 3. Install `uv` and Dependencies

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
source ~/.bashrc && uv sync
```

## 4. Verify Communication Without Moving Anything

```bash
uv run python examples/other/01_i2c_probe.py
```

Expected result: `✓ Reset command sent successfully; the driver board responded`, followed by a brief
flash of the head LED.

If the LED flashes, the command path is working. If there is no flash and no error, check the
firmware status LED on the board.

## 5. Map the Motors With the Car Lifted

Lift the car so that all four wheels are off the ground.

```bash
uv run python examples/other/02_motor_check.py
```

Each motor runs forward for one second, stops, then runs in reverse for one second. Record:

1. Which physical wheel corresponds to `M1`, `M2`, `M3`, and `M4`
2. Whether the wheel moves forward or backward during the script's reported "forward" direction

This step is required because the vendor wiring diagrams and vendor code comments disagree about
direction semantics. See [nezha-i2c-protocol.md](../hardware/nezha-i2c-protocol.md) for details.

## 6. Apply the Verified Mapping

Edit [`src/carbot/config.py`](../../src/carbot/config.py):

```python
WHEEL_TO_MOTOR = {
    "front_left": 4,
    "front_right": 3,
    "rear_left": 1,
    "rear_right": 2,
}
```

Direction handling:

| Situation | What to change |
|---|---|
| All four motors are reversed | Set `FORWARD_IS_MOTOR_A = False` in `src/carbot/nezha.py` |
| Only some motors are reversed | Add their motor numbers to `config.INVERTED_MOTORS`, for example `frozenset({1, 3})` |

## 7. Run the Minimal Driving Test

Keep the car lifted.

```bash
uv run python examples/other/03_motor_drive.py
```

The script tests:

- forward
- backward
- turn left
- turn right
- spin left
- spin right

Every movement should match its label. If any movement is wrong, return to Step 6 and fix the configuration.

## 8. Put the Car on the Ground

Only place the car on the floor after all lifted tests are correct. Start with `SAFE_TEST_SPEED = 200`
and increase only after confirming stable straight driving.

```python
from carbot import Car

with Car() as car:
    car.move_for(1.0, 300, 300)
    car.move_for(0.6, -250, 250)
```

`Car` is a context manager, so motors stop automatically when the `with` block exits or an exception occurs.

---

## Troubleshooting

### `OSError: [Errno 121] Remote I/O error`

The board is not acknowledging data correctly. Vendor I2C code does not check ACK handling, but
Raspberry Pi hardware I2C does.

1. Confirm that `i2cdetect -y 1` still shows `40`
2. Confirm that the I2C speed has not been raised
3. If it still fails, switch to software I2C by adding this to `/boot/firmware/config.txt`:

   ```
   dtoverlay=i2c-gpio,bus=3,i2c_gpio_sda=23,i2c_gpio_scl=24
   ```

   Then move the wiring to GPIO 23 and 24 and use `NeZha(bus=3)`.

### `i2cdetect` Does Not Show `40`

- Is the board powered on, with both status LEDs lit?
- Are SDA and SCL swapped? (`Pin 3` is SDA, `Pin 5` is SCL)
- Is ground shared between the board and the Raspberry Pi?
- Is a PCA9685-based HAT connected? Its default address is also `0x40`.

### Encoders Always Read Zero

That is expected for this build. The current car uses two-wire DC motors without encoders, so
`config.HAS_ENCODERS = False` is correct. If you later switch to N20 encoder motors, set it to
`True` and connect the encoder wires to the 4-pin side of each motor port.

### The Car Stops Suddenly or the Raspberry Pi Reboots

This usually indicates a power drop. Check:

```bash
vcgencmd pmic_read_adc EXT5V_V && vcgencmd get_throttled
```

If `get_throttled` is anything other than `0x0`, the power rail has dipped. Reduce test speed or
use a separate, known-good Raspberry Pi power source. Do not feed the Pi from both USB-C and the
NeZha `5V` rail at the same time.

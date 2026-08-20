# 2026-07-30 First Real Driving Test

## Result

The Raspberry Pi 5 successfully controlled the NeZha driver board over I2C. All four motor ports
were verified against the actual wheel positions, all six lifted differential-drive movements were
correct, and a short low-speed ground test completed successfully.

## Communication Check

Command:

```bash
uv run python examples/01_i2c_probe.py
```

Verified results:

- I2C bus 1 responded at address `0x40`
- Reset command succeeded
- Encoder reads were skipped because this build uses two-wire DC motors
- Head LED control worked

## Motor Mapping Verification

Command:

```bash
uv run python examples/02_motor_check.py
```

Observed mapping:

| Port | Wheel position | Reported forward rotation |
|---|---|---|
| M1 | Rear left | Forward |
| M2 | Rear right | Backward |
| M3 | Front right | Backward |
| M4 | Front left | Forward |

This confirmed the expected physical wheel layout. M2 and M3 needed inversion, so the verified
configuration is:

```python
WHEEL_TO_MOTOR = {
    "front_right": 3,
    "front_left": 4,
    "rear_right": 2,
    "rear_left": 1,
}

INVERTED_MOTORS = frozenset({2, 3})
```

Because not all motors were reversed, `FORWARD_IS_MOTOR_A` remained unchanged.

## Driving Verification

Command:

```bash
uv run python examples/03_motor_drive.py
```

All of the following lifted movements matched their labels:

- forward
- backward
- turn left
- turn right
- spin left
- spin right

After that, the car was placed on the ground and driven forward at speed 200 for 0.5 seconds. The
direction was correct and the car stopped cleanly when the script ended.

```bash
uv run python -c \
'from carbot import Car; car = Car(); car.move_for(0.5, 200, 200); car.close()'
```

## Code Verification

The verified hardware mapping was written back into `src/carbot/config.py`. Automated checks then
passed against the updated logical motor directions.

```text
pytest: 44 passed
ruff: All checks passed
```

## Next Steps

- Tune safe speed and straight-line behavior on open floor space
- Measure left/right drivetrain imbalance and add calibration if needed
- Verify servo ports, neutral positions, and safe angle limits before attaching the robotic arm

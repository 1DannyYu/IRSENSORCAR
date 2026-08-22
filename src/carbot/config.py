"""Hardware mapping for the current car build.

## Confirmed from vendor documentation

1. **Motor port geometry on the back of the driver board**
   (the complete-car wiring PDF under `vendor/yourfun-nezha/wiring/`, right-side diagram):

   ```
   M3 ┌───────────┐ M4
      │           │
   M2 └───────────┘ M1
   ```

2. **The servo header side (`S1 S2 S3 S4`) is the front of the car.**
   That side has the two white LEDs used as `HEADLED` in the command set. The opposite side carries
   the I2C header and the firmware/power status LEDs.

## Confirmed on real hardware

On 2026-07-30, `examples/other/02_motor_check.py` confirmed:
M1 = rear left, M2 = rear right, M3 = front right, M4 = front left.
M2 and M3 need inversion.
"""

from __future__ import annotations

from typing import Literal

Wheel = Literal["front_left", "front_right", "rear_left", "rear_right"]

# Wheel -> driver board motor port number (1-4).
# Verified on real hardware on 2026-07-30.
WHEEL_TO_MOTOR: dict[Wheel, int] = {
    "front_right": 3,
    "front_left": 4,
    "rear_right": 2,
    "rear_left": 1,
}

# List motors here when only some motors need direction inversion.
# If all motors are reversed, change `FORWARD_IS_MOTOR_A` in `nezha.py` instead.
INVERTED_MOTORS: frozenset[int] = frozenset({2, 3})

# This build uses two-wire DC motors without Hall encoders.
# The board supports encoders, but `encoder()` always reads zero on this hardware.
# Switch this to True only after moving to encoder motors and wiring the 4-pin encoder side.
HAS_ENCODERS = False

# Robotic arm joint -> driver board servo port number (1-4).
ARM_JOINT_TO_SERVO: dict[str, int] = {
    "base": 1,
    "shoulder": 2,
    "elbow": 3,
    "gripper": 4,
}

# Conservative joint limits to reduce the chance of self-collision.
ARM_JOINT_LIMITS: dict[str, tuple[float, float]] = {
    "base": (0.0, 180.0),
    "shoulder": (20.0, 160.0),
    "elbow": (20.0, 160.0),
    "gripper": (30.0, 120.0),
}

# Safe starting speed for first powered tests. Increase only after lifted verification passes.
SAFE_TEST_SPEED = 200

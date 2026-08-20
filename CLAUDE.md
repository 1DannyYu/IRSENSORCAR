# Car and Robotic Arm - AI Agent Rules

## Read This First

**[CONVENTIONS.md](CONVENTIONS.md) is the source of truth for file placement and naming.**
Read it before adding, moving, or renaming files.

## Project Nature

This is a real robotics project. Code runs on a Raspberry Pi 5 and controls the NeZha driver board
over I2C. Bad assumptions can move hardware unexpectedly, damage components, or create unsafe tests.

## Hard Rules

1. **`vendor/` is read-only.** If vendor code needs changes, copy it into `src/` or `scripts/`.
2. **Do not guess motor behavior.** Use
   [docs/hardware/nezha-i2c-protocol.md](docs/hardware/nezha-i2c-protocol.md) as the protocol source.
3. **Motor-moving programs may only run when a person is physically beside the robot and can cut
   power instantly.** Over SSH, that means the operator must be at the robot; lift the wheels or
   secure the chassis before any motor/servo test. Otherwise, provide code and instructions instead.
4. **Do not commit or push unless the user explicitly asks.**
5. Follow [docs/setup/raspberry-pi-first-run.md](docs/setup/raspberry-pi-first-run.md) for the verified
   hardware bring-up sequence.
6. Asset photo numbers are never reused or reordered.

## Known Hazards

| Item | Notes |
|---|---|
| I2C address `0x40` | Conflicts with the default PCA9685 address |
| I2C speed | Must stay at or below 200kHz |
| Dual power input | Do not power the Raspberry Pi from both NeZha `5V` and USB-C |
| Motor direction | Vendor docs and code comments disagree; trust verified tests |
| Power-on delays | Keep the required 500ms init and 100ms reset delays |

## Tooling

- Python: `uv`
- Frontend and site architecture: see [docs/adr/](docs/adr/)

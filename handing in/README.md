# Task 1 — IR Line Following (Submission Code Export)

This folder is a standalone export of the Task 1 IR line-following implementation from
the main project repository. It contains the driver package, the example scripts used
to run and calibrate Task 1, and the automated tests that cover them — enough to install
and run on its own, without the rest of the repo.

For the full project (hardware docs, architecture decisions, progress logs, and
reflections), see the GitHub repository:
<https://github.com/1DannyYu/IRSENSORCAR>

## Contents

- `src/carbot/` — the importable Python driver package (NeZha I2C protocol, motion,
  sensors, navigation/state-machine logic)
- `examples/other/` — the runnable scripts behind Task 1:
  - `39_map1_ir_line_follow.py` — main IR line-follow driver for the Task 1 route
  - `46_map1_ir_transition_drive.py` — consolidated transition-phase run
  - `48_motor_one_second_check.py` — one-second motor sanity check
  - `49_ir_phase1_to_phase2_then_original_trace.py` — Phase 1 → Phase 2 → original-trace run
- `tests/other/` — automated tests (pytest) for the non-camera subsystems: IR sensor
  decoding, line-follow/route/state-machine logic, motion, sonar, servo, and the NeZha
  I2C protocol, including regression tests for the four example scripts above

## Setup

Requires Python 3.11+ and [uv](https://docs.astral.sh/uv/).

```bash
uv sync
```

## Run the tests

```bash
uv run pytest tests/other/ -v
```

## Run an example

Example scripts drive real hardware over I2C. Only run them with the robot powered,
wheels lifted or the chassis secured, and a person beside it who can cut power
instantly.

```bash
uv run python examples/other/49_ir_phase1_to_phase2_then_original_trace.py --help
```

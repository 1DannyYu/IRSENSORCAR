# 2026-08-17 IR Tracing Sensor Driver

## Scope and Result

Added the first code for the Yahboom 4-channel IR tracing sensor
(`assets/inventory/041`/`042`), which was previously only an inventory entry in
`site/src/data/modules.json`. The goal: every channel reports **1 on the black
line and 0 on white**, with a single normalized reading convention.

Changed:

- `src/carbot/ir_tracing.py` — `IRTracingSensor` driver. Reads N channels
  through an injectable GPIO (same pattern as `src/carbot/sonar.py`), returns
  `read()` as a tuple of normalized `1 = black` / `0 = white` values in
  Out-order, and exposes `raw()` for calibration. Per-channel polarity is
  configurable via `invert={indices}` because raw comparator polarity is not
  guaranteed across boards.
- `tests/test_ir_tracing.py` — 8 fake-GPIO tests covering both polarities,
  per-channel independence, invert handling, ordering, and input validation.
- `examples/36_ir_tracing_check.py` — no-motor verification readout (safe over
  SSH) with `--pins`, `--invert`, `--count`, `--interval`; prints raw and
  normalized values per channel so the operator can confirm and flip polarity.
- `docs/hardware/ir-tracing-sensor.md` — wiring plan, reading convention,
  pin-conflict warning.

Intentionally not changed: no motor/line-follow logic wired to the sensor yet,
and no `config.py` pin mapping — the wiring is unverified and conflicts with the
HC-SR04 (see below), so pins stay caller-supplied until verified.

## Verification

- `uv run pytest -q tests/test_ir_tracing.py` — **8 passed**.
- `python3 -m py_compile examples/36_ir_tracing_check.py src/carbot/ir_tracing.py tests/test_ir_tracing.py` — OK.
- Driver smoke test with a stub GPIO: `read()` returned `(1, 0, 1, 0)` for raw
  `HIGH/LOW/HIGH/LOW` under the default polarity.

No hardware was exercised; the sensor is not yet wired (operator confirmed
"not tested yet").

## Measurements and Configuration

- Planned wiring (from `site/src/data/modules.json`): Out1–Out4 -> GPIO
  17/27/22/23; VCC Pin 2 (5V) or Pin 1 (3.3V); GND Pin 6/9.
- Polarity assumption: raw HIGH = black (`BLACK_IS_HIGH = True`). **Unverified —
  must be confirmed with `examples/36_ir_tracing_check.py` on the Pi.**
- Detection distance ~1–3 cm, per-channel potentiometers.

## Problems Encountered

1. **Pin conflict**: planned Out1/Out2 (GPIO 17/27) are the verified HC-SR04
   TRIG/ECHO pins. The tracing sensor and ultrasonic cannot both use them;
   the wiring plan must be resolved before the sensor is connected.
2. **Unknown raw polarity**: LM339 tracing boards vary in whether black reads
   HIGH or LOW. Handled in code with the `invert` set rather than guessing a
   single fixed convention.

## Follow-up

1. Wire the sensor on the Pi, resolving the HC-SR04 conflict first.
2. Run `PYTHONPATH=src python3 examples/36_ir_tracing_check.py` with the sensor
   held over black and white; record each channel's raw polarity.
3. Re-run with `--invert` for any inverted channel, then bake the verified
   `invert` set into the calling code.
4. Update `docs/hardware/ir-tracing-sensor.md` and this log to "Verified" with
   the confirmed pins and polarity, and link a progress log entry.

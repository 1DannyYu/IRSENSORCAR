# Servo Check Extracted to `carbot.servo`

Date: 2026-08-19

## Scope and Result

First application of the `src/` vs `examples/` boundary rule: logic that needs unit tests belongs in
an importable module. `examples/04_servo_check.py` held the arm servo check's sequencing and its
safety gate, and [tests/test_servo.py](../../tests/test_servo.py) reached them through an
`importlib.util.spec_from_file_location` hack, because `04_servo_check` is not a valid Python
identifier and cannot be imported normally.

- **New [src/carbot/servo.py](../../src/carbot/servo.py)** — `SERVO_CHANNELS`, `TEST_ANGLES`,
  `CLEARANCE_ANSWER`, the `ServoBoard` / `ManagedServoBoard` protocols, `clearance_confirmed()`,
  `run_check()`, and `run_session()`. Exit codes are named constants (`EXIT_OK`, `EXIT_REFUSED`,
  `EXIT_INTERRUPTED`) instead of bare integers.
- **`examples/04_servo_check.py` reduced from 71 lines to 27**, and is now what an example should
  be: a docstring, a safety warning, and one call that supplies the real board.
  `run_session(lambda: NeZha(init_motors=False))`.
- **`tests/test_servo_check.py` renamed to `tests/test_servo.py`** to match the `test_<module>.py`
  convention, with the `importlib` loader and all six `monkeypatch.setattr(module, ...)` calls
  deleted. Behaviour is now reached by plain injection: `run_session` takes a board *factory*, so a
  test passes a fake instead of patching a name inside a hand-loaded module.

Behaviour is unchanged: same channels, same angles, same operator prompt before every individual
move, same Traditional-Chinese operator strings, same exit codes, same `close(stop_motors=False)`.

## Why `run_session` Takes a Factory

`open_board` is a callable, not a board. Nothing touches I2C until the clearance answer is an exact
`yes`, so a refused run is provably a run in which no board was ever constructed — which is exactly
what `test_session_rejects_any_confirmation_except_exact_yes` asserts, by passing a factory that
raises if called. The old test approximated this by monkeypatching `NeZha` with a constructor that
set a flag.

## Verification

```bash
uv run python -m pytest -q          # 403 passed (was 395; 8 new servo tests)
uv run ruff check src/carbot/servo.py tests/test_servo.py examples/04_servo_check.py   # clean
```

Test count for this behaviour went from 8 to 16. The added cases cover `clearance_confirmed()`
directly (including `" yes"` and `"Yes"`, which the old parametrisation did not exercise) and a
board that fails to open, which previously had no coverage at all.

No hardware was run — this is a refactor with no protocol or timing change. The next real-hardware
run of `examples/04_servo_check.py` should confirm the prompts and movement order are unchanged.

## Problems Encountered

`ruff check .` was already failing repo-wide before this change (13 errors). Seven were `EXE001`
(shebang present, file not executable) on `examples/` scripts, fixed with `chmod +x` since the
other 32 examples are already executable and the inconsistency was accidental.

## Follow-up

Six pre-existing ruff errors remain, all in files untouched by this change and none introduced
here:

- `examples/37_map1_motor_test.py` — `F401` unused `IRTracingSensor` import, `BLE001` blind
  `except Exception`
- `examples/38_map1_cam_line_follow.py` — `F401` unused `Path` and `cv2` imports, two `BLE001`

The `F401`s are auto-fixable. The `BLE001`s are in hardware-cleanup paths where a blind catch may
be deliberate; check intent before narrowing them.

Next candidates for the same extraction, by the 200-line / 3-def threshold discussed with the
operator: `examples/21_cam_dual_mode_check.py` (700 lines, 20 module-level defs, only 2 carbot
imports) and `examples/22_cam_sonar_patrol_capture.py` (662 lines, 12 defs). Their
recommendation and scoring helpers — `_recommend_exposure`, `_count_repeatable`, `_overlap`,
`_quality_reason` — are pure decision logic with no test coverage.

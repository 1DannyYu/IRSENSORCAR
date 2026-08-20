# Repository Archiving Work Log — 2026-08-14

Related files: [archiving and naming conventions](../../CONVENTIONS.md),
[vision room-anchor progress](2026-08-14-vision-room-anchor.md),
[vision-to-mapping handoff](../handoff-2026-08-14-vision-to-mapping.md),
[robot-base research](../robot-base-platform-research.md), and
[project terminology](../project-terminology.md).

## Scope and Result

- Audited tracked, untracked, generated, and ignored files before integrating the day's work.
- Moved 17 private 4056 x 3040 IMX500 calibration source frames from
  `assets/reference/camera-calibration/2026-08-14-imx500-4056x3040/` to
  `scratch/camera-calibration/2026-08-14-imx500-4056x3040/source-frames/`. The reviewed
  `calibration.json` remains in `assets/reference/`; runtime code does not depend on the ignored
  source frames.
- Sent repository-root `tmp/`, `_site/`, `.astro/`, `.pytest_cache/`, `.ruff_cache/`, and the
  `examples/` and `tests/` Python caches to the system Trash. These were generated outputs and can
  be rebuilt.
- Preserved the only byte-identical tracked pairs, `NeZha(1).h` / `NeZha.h` and
  `NeZha(1).cpp` / `NeZha.cpp`, because they are under the read-only `vendor/` boundary.
- Expanded `CONVENTIONS.md` with canonical document roles, discoverability rules, work-log
  requirements, privacy-sensitive calibration storage, generated-file cleanup, and the exception
  for school-provided filenames.
- Integrated the school assessment sources, bilingual Folio HTML, project glossary, robot-base
  research report, and the vision-to-mapping developer handoff. No motor or other hardware-moving
  command was run during this work.

## Verification

```text
PYTHONDONTWRITEBYTECODE=1 uv run pytest -q -p no:cacheprovider
70 passed in 27.60s

uv run python scripts/check_inventory_data.py
Dataset is complete (35 modules, 69 referenced images)

npm run build
88 pages built successfully
```

Both committed school PDFs were rendered page by page (10 pages total) and visually inspected; no
clipping, blank pages, overlap, or damaged fonts were found. The bilingual Folio HTML was opened in
Google Chrome through a local HTTP server. It produced no console errors or warnings and no
horizontal overflow at a 1904 px viewport.

The repository-wide command `uv run ruff check --no-cache .` reported eight pre-existing findings
in unchanged examples 06–08: three non-executable files with shebangs, four preferred GPIO import
forms, and one missing explicit `subprocess.run(check=...)`. This documentation-only integration
did not change those hardware examples.

## Measurements and Configuration

- Calibration session key: `2026-08-14-imx500-4056x3040`.
- Local private frame count: 17 JPEG files, approximately 36 MB total.
- Printable target measurements remain recorded in
  `docs/progress/2026-08-14-vision-room-anchor.md`.
- Tests in this work log were software-only; no Raspberry Pi GPIO, I2C, motor, or camera command was
  issued.

## Problems Encountered

- Generated PDF builders and previews had accumulated under a repository-root `tmp/`, making it
  unclear which files were authoritative. The final reviewed PDFs were already under
  `assets/reference/`; the stale intermediates were trashed and root `tmp/` is now prohibited.
- Calibration frames were initially placed beside the runtime calibration output. Although useful
  evidence, they expose the room interior and added about 36 MB. The corrected location is the
  ignored session-specific `scratch/` path.
- A handoff that embedded an exact GitHub head hash would become stale as soon as its own commit was
  pushed. It now identifies the vision implementation base but directs the next developer to the
  current `origin/main`.

## Follow-up

1. Resolve the existing Ruff findings in examples 06–08 as a separate behavior-reviewed change.
2. Follow `docs/handoff-2026-08-14-vision-to-mapping.md` and implement mapping Gate A without moving
   the robot.
3. Keep future session evidence in the canonical `scratch/` layout and add a dated progress log
   before each development-session commit.

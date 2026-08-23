# IR chained-mode tuning and repository reorganization

> **Note on this entry:** unlike other logs in this folder, this is a retrospective summary
> compiled from `git log` on 2026-08-23, not a same-day first-hand account. No verification
> commands were re-run to produce it; commit messages and diffs are the source of truth below.

## Scope

2026-08-22 covers 42 commits from `55ecc98` (11:14) to `ab1cfdd` (21:33), roughly
176 files changed, +32,845/-4,308 lines. The day splits into two halves: a long iterative
IR chained-mode tuning session on the Pi (morning through early afternoon), then a large
documentation/repository restructuring pass (evening).

## Chained IR driving modes (11:14–14:01)

- `55ecc98` added [`examples/47_ir_three_modes.py`](../../examples/other/47_ir_three_modes.py) and
  [`src/carbot/ir_modes.py`](../../src/carbot/ir_modes.py) as a new chained-mode driving loop, with
  `tests/test_ir_modes.py`.
- Followed by rapid iteration tuning correction strength and turn behavior: `78cb2c3` increased
  left correction strength, `b47c708` strengthened left curves and started the circle earlier,
  `2e0d408` made "P1000" a hard left pivot, `83f2b8f` wired the roundabout turn into chained mode,
  `c1843af` split roundabout mode into separate entry-trace and exit phases.
- `17f5817` added `scripts/sync-to-pi.sh`, and `2a9bad4` switched it to timestamp-based commit
  messages — this is why most commits between 11:49 and 14:01 have a bare timestamp as their
  message: they were auto-committed by the sync script during on-robot tuning of
  `docs/hardware/ir-circle-mode-16-state-tables-by-mode.md`, `ir_modes.py`, and
  `examples/47_ir_three_modes.py`.
- `a31aa80` (13:36) onward introduced
  [`examples/49_ir_phase1_to_phase2_then_original_trace.py`](../../examples/other/49_ir_phase1_to_phase2_then_original_trace.py),
  combining the chained-mode phase 1/2 logic with the original tracing loop; tuned through 14:01.

## Diagrams, docs, and a frozen handoff snapshot (16:55–17:42)

- `b449c90` regenerated `flowchart.md`, `structure_chart.md`, `uml_class_diagram.md`, and an
  IR state-table PDF/PNG build, plus captured `SuccessfulLogs/Success1.log` and `Success2.log`.
- `0e79df2` translated remaining Chinese text in `README.md` to English.
- `9f9bf45` added `handing in/` — a frozen, dated snapshot of `src/carbot/` and example 49, kept
  intentionally static as a point-in-time submission record (later commits explicitly leave it
  un-updated; see `ab1cfdd` below).

## Testable extraction (20:12)

- `cb57106` pulled a testable `decide_step` function out of example 49's tracing loop into
  [`tests/test_example_49_phase1_phase2_trace.py`](../../tests/other/test_example_49_phase1_phase2_trace.py),
  so the drive-loop decision logic can be unit tested without hardware.

## Documentation and repository restructuring (20:47–21:33)

A dense sequence of structural cleanup commits, closing out the day:

- `1da6a8e` — rewrote `README.md` as navigation-only, moved the full reference into
  [`docs/project-reference.md`](../project-reference.md); `CONVENTIONS.md` updated to match.
- `7bd0fac` — removed `vendor/yourfun-nezha` and `vendor/raspberry-pi` material entirely
  (SDKs, wiring PDFs, manuals) after the project no longer depended on the vendored copies.
- `5d168db` — split `tests/` into `tests/ai_camera/` (vision-dependent) and `tests/other/`.
- `0730e61` and `b7a0bfd` — updated `README.md`, `CONVENTIONS.md`, `docs/project-reference.md`,
  and ADR 0004 to reflect the vendor removal, repointing ADR 0004's links to a GitHub permalink
  instead of the now-deleted `vendor/` tree.
- `4b47f30` — trimmed `README.md` further, removing explanations duplicated elsewhere.
- `50c0670` — split `examples/` into `examples/ai_camera/` and `examples/other/`, mirroring the
  tests split, and updated every doc/reference that pointed at the old flat paths.
- `f70c992` — removed the `docs/Mechatronics Folio and Journal/` folder and its dead references.
- `ab1cfdd` — renumbered `40_motor_spin_duration_check.py` / `41_motor_spin_angle_sweep.py` to
  `41.1`/`41.2` to resolve a numbering collision with `40_map1_ir_phase_test.py` (CONVENTIONS.md
  §3.6, numbers are never reused). Explicitly left `docs/progress/*.md` and `handing in/`
  un-updated, as dated records of what was true at the time they were written.

## Files most touched

- [`src/carbot/ir_modes.py`](../../src/carbot/ir_modes.py) and
  [`examples/other/47_ir_three_modes.py`](../../examples/other/47_ir_three_modes.py) — the
  morning's tuning target.
- [`README.md`](../../README.md), [`CONVENTIONS.md`](../../CONVENTIONS.md),
  [`docs/project-reference.md`](../project-reference.md) — rewritten multiple times during the
  evening restructuring.
- `vendor/` — deleted outright.

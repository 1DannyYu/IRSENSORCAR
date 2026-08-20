# English-only site and interactive homepage map

## Scope

- Removed the Chinese website routes and localized data so the generated site is English-only.
- Kept the three operator-facing documents bilingual at the operator's request.
- Consolidated the current Map 1 image under `assets/reference/map-1/` and preserved the distinct legacy image in the task directory.
- Embedded `tasks/ir-sensor-tracking/route-planning.html` as the interactive Map 1 section on the homepage.
- Rendered the root `README.md` on the homepage and added Danny's repository and GitHub profile links.
- Reconciled old repository metadata and site URLs with `https://github.com/1DannyYu/IRSENSORCAR`.
- Repaired `open-project-homepage.command` and assigned port `18427`, with automatic fallback through `18526`.

## Verification

- `npm run build`: 45 pages built successfully.
- `uv run python scripts/check_inventory_data.py`: 36 modules and 70 referenced images validated.
- `uv run --isolated --python 3.13 --extra vision python -m pytest`: 576 tests passed.
- Browser check: Phase 9 selection updated the embedded detail panel, and the full-route reset worked.
- Launcher check: reused an existing checkout server, selected `18428` when `18427` was occupied, and returned to `18427` after the test.
- Conflict-marker and CJK scans found no unresolved merge files or unexpected first-party Chinese text.

No motor, servo, or other hardware-moving command was run.

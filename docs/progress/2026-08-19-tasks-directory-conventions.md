# `tasks/` Directory — Convention Compliance Pass

Date: 2026-08-19

## Scope and Result

`Tasks/` was added in `3c7e543` without being registered in
[CONVENTIONS.md](../../CONVENTIONS.md) §1, so nothing constrained its naming and it accumulated
violations of rules the rest of the repository follows. This pass registers the directory and
brings its contents into line. No document content was rewritten beyond the path and factual
corrections listed below.

### Violations found and fixed

| # | Violation | Rule | Fix |
|---|---|---|---|
| 1 | `Tasks/` absent from the §1 repository tree | §2.1 — no new top-level directory without updating §1 in the same commit | Added to the §1 tree, the §2 placement table, and a new §2.1 bullet defining what belongs in it |
| 2 | `Tasks/` title-cased, unlike every other top-level directory | §3.1 | Renamed to `tasks/` |
| 3 | `3D Mapping/`, `Camera Inferences/`, `IR Sensor Tracking/` contained spaces | §3.5 — "No spaces" | Renamed to `3d-mapping/`, `camera-inferences/`, `ir-sensor-tracking/` |
| 4 | `Map1-IR-Tracking-Plan.md` title-cased | §3.1 — `lower-kebab-case` | `map1-ir-tracking-plan.md` |
| 5 | `QUICKSTART.md` generic and unfindable by topic | §2.1 — names like `latest.md` are prohibited for this reason | `map1-ir-tracking-quickstart.md` |
| 6 | `front_check_2026-08-17.jpg` snake_case, date as suffix | §3.4 — kebab-case, ISO date prefix | `2026-08-17-front-object-detection-check.jpg` |
| 7 | §3.3 claimed the highest asset number was `102`, but `103_YourFun_NeZha_Breakout_Driver_Board.jpg` exists | §3.3 | Corrected to `103`; next photo starts at `104` |
| 8 | `camera-inferences/README.md` claimed the directory was not tracked, but it is tracked | Factual error | Claim removed and replaced with a note recording the real status and the open question below |

All inbound references updated: the `<code>Tasks/3D Mapping/</code>` self-reference inside
`indoor-mapping-process-plan.html`, the log-destination path in `map1-ir-tracking-plan.md`, the
plan link in `map1-ir-tracking-quickstart.md` (now an actual markdown link, previously bare text),
and the photo filename in `camera-inferences/README.md`.

### Commit shape

The `indoor-mapping-process-plan.html` change arrived in the working tree as an untracked new file
plus a deleted Chinese-named file, which git scores at 39% similarity — below the default 50%
rename threshold — because the body was fully translated at the same time. Split into two commits,
a byte-identical rename followed by the translation, so `git log --follow` traces the file back
past the move.

## Verification

```bash
git log --follow --oneline -- tasks/3d-mapping/indoor-mapping-process-plan.html
# 2828865 docs(tasks): translate the indoor mapping plan and mark it on hold
# 9ab20d8 refactor(tasks): rename the indoor mapping plan to an English filename
# 3c7e543 chore: add tasks, reference diagrams, and progress documentation
```

Reference sweep found no surviving occurrence of any old `Tasks/` path or old filename. The one
remaining literal mention of `Tasks/IR Sensor Tracking/` is in
[2026-08-19-examples-naming-convention.md](2026-08-19-examples-naming-convention.md), where it
narrates a past shell bug caused by those spaces; it is annotated with the new path rather than
rewritten, because the sentence is a statement about how the directory used to be named.

`pytest -q` was not re-run for this pass — it touches no Python. The preceding naming commit
verified 395 passing.

## Problems Encountered

`tasks/` is a case-only rename of `Tasks/`. macOS and Windows checkouts are case-insensitive, so a
collaborator pulling this with a dirty working tree can end up with both spellings or a confusing
merge. It was done as a two-step `git mv Tasks tasks-tmp && git mv tasks-tmp tasks` so the commit
records a clean rename, but anyone pulling should commit or stash local work under `Tasks/` first.

## Follow-up

- **Open question — `tasks/camera-inferences/2026-08-17-front-object-detection-check.jpg`.** Its
  own README described the folder's contents as disposable material, which by §7 means the
  file belongs in the ignored `scratch/`, not in version control. Untracking a 328 KB photo a
  colleague committed is a deletion from the working tree, so it was left in place pending a
  decision. Either move it to `scratch/camera-inferences/` and `git rm --cached` it, or keep it and
  reclassify it as project evidence under `assets/reference/` with a number per §3.3.
- `tasks/3d-mapping/indoor-mapping-process-plan.html` has no inbound link from anywhere in the
  repository, so §2.1's discoverability rule is only nominally satisfied. It states it should
  become a `docs/adr/` record once the plan is resumed and decided; until then it is findable only
  by directory browsing.

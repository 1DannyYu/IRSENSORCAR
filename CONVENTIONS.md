# Project Archiving and Naming Conventions

> This document is the single source of truth for file placement and naming in this repository.
> If you are unsure where a file belongs or how it should be named, follow this document.
> If the repository disagrees with this document, update the repository layout instead of weakening the rule.

Last updated: 2026-08-19

---

## 1. Repository Structure

```
Car-and-Robotic-Arm/
├── README.md              Project entry point
├── CONVENTIONS.md         This file
├── CLAUDE.md              AI agent working rules
├── pyproject.toml         Python project definition (managed with uv)
│
├── docs/                  First-party project documentation
│   ├── project-reference.md   Full technical/operator reference (hardware bring-up, example
│   │                          script table, IR workflow runbook, SSH access); README.md is the
│   │                          short entry point and links here for detail
│   ├── hardware/          Hardware specs, protocol notes, wiring
│   ├── setup/             Bring-up and environment setup guides
│   ├── progress/          Verified progress logs from real hardware work
│   ├── adr/               Architecture decision records
│   ├── reflections/       Project reflection and engineering role reports
│   ├── handoff-*.md       Current continuation notes for another developer/agent
│   ├── robot-base-platform-research.md   Research background report (modular robot base platform)
│   └── project-terminology.md            English project glossary
│
├── src/carbot/            Importable Python package
├── tests/                 Automated tests, split into ai_camera/ (vision-dependent) and other/
├── examples/              Runnable example and verification scripts, split the same way:
│   ├── ai_camera/         Scripts that read the Raspberry Pi AI Camera (IMX500) — the `cam` tag
│   └── other/             Everything else — motor, servo, I2C, sonar, IR, power, multi-sensor
├── scripts/               One-off tools and validators
│
├── tasks/                 Per-task working notes, plans, and run books
│   └── <task-slug>/       One directory per task, `lower-kebab-case`
│
├── assets/                Binary assets
│   ├── inventory/         Inventory photos (numbered sequence)
│   ├── assembly/          Assembly photos (numbered sequence)
│   ├── reference/         Diagrams, screenshots, spec images
│   └── assembly-guide/    Extracted assembly manual pages and text
│
├── site/                  Astro website source
│   ├── src/data/          Shared website data files
│   ├── src/pages/         English-only routes
│   ├── src/components/    Components
│   ├── src/layouts/       Layouts
│   ├── src/styles/        Shared tokens and page styles
│   └── public/            Static files copied as-is
│
├── astro.config.mjs       Build configuration (`_site/` output is not committed)
├── package.json
│
└── vendor/                Third-party material, kept read-only when present (empty as of 2026-08-22 —
                             the NeZha SDK/manual and the BCM2711 datasheet were removed; the I2C
                             facts they sourced are preserved in docs/hardware/nezha-i2c-protocol.md
                             and docs/adr/0004-nezha-python-driver-port.md)
```

## 2. Decide Placement by Asking One Question

**Who created this file?**

| Source | Where it goes | Editable |
|---|---|---|
| First-party documentation | `docs/` | Yes |
| First-party code | `src/`, `tests/`, `examples/`, `scripts/` | Yes |
| Working notes and run books scoped to one task | `tasks/<task-slug>/` | Yes |
| Curated photos we captured and intend to publish | `assets/` | Append only |
| Website frontend | `site/` | Yes |
| Vendor-provided material | `vendor/` | No |
| Private raw captures and generated scratch output | `scratch/` | Not committed |

`vendor/` is a hard boundary. If vendor code needs adaptation, copy it into `src/` or `scripts/`
and keep the original files untouched for reference.

### 2.1 Keep Files Discoverable

- Do not invent a new top-level directory or permanent document category without updating the
  repository tree in §1 in the same commit.
- Before adding a file, search by topic and update the canonical file when one already exists.
  Prefer links over copied facts.
- Move tracked files with `git mv`, then use `rg` to update every inbound path. A move is incomplete
  while documentation, scripts, or site data still point to the old location.
- Put the topic in every dated progress/handoff filename. Names such as `notes.md`, `latest.md`,
  `report-final.md`, and `handoff.md` are prohibited because they cannot be found reliably later.
- Each handoff's **Read First** section must link to the applicable progress log and stable
  hardware/setup/ADR sources. Each progress log must link to the files it changed or produced.
- `tasks/<task-slug>/` holds working notes and run books for one task, in `lower-kebab-case` like
  everything else. It is version-controlled, so it is not a dumping ground: capture evidence
  (photos, raw logs, scoring runs) belongs in the ignored `scratch/` per §7, and a decision that
  outlives the task belongs in `docs/adr/` or `docs/hardware/`.

## 3. Naming Rules

### 3.1 Code and Documents: `lower-kebab-case`

```
docs/hardware/nezha-i2c-protocol.md
scripts/build-assembly-html.py
site/inventory/index.html
```

The only exception is Python modules, which use `snake_case` because they must be imported, for
example `src/carbot/nezha.py`.

### 3.2 Document Language and Filenames

Filenames never carry a language suffix. Use plain `name.md`:

```
good: raspberry-pi-5-pinout.md
good: mac-to-raspberry-pi-access.md
bad:  deskflow-macos-raspberrypi.en.md
```

This rule costs nothing, because the website does not resolve language through filenames either.
The project website is English-only. Its content is organised in three places, none of which uses
a language suffix in the filename:

| Layer | Mechanism |
|---|---|
| Routes | English pages directly under `site/src/pages/` |
| UI strings | The `en` dictionary in `site/src/i18n/ui.ts` |
| Page data | The `i18n.en` field inside `site/src/data/*.json` |

Adding a `.en` or `.zh` suffix to a file therefore signals nothing to any build step, and only
creates a second naming style to remember.

**Language inside a document:** first-party source code, website content, data, tests, and technical
reference material are written in English so that terminology matches the code and vendor sources.
Existing operator-facing procedures may remain bilingual when translating them is outside the
scope of the current work; do not add new non-English content to technical files.

| Document | Language |
|---|---|
| `docs/hardware/`, `docs/adr/`, `docs/progress/` | English |
| `docs/reflections/` | English |
| `docs/setup/mac-to-raspberry-pi-access.md` | Bilingual (visitor-facing) |
| `tasks/ir-sensor-tracking/run-book.md` | Bilingual (operator-facing) |
| Operator workflow sections in `README.md` | Bilingual (operator-facing) |
| Other `docs/setup/` procedures | English |

### 3.3 Asset Photos: `NNN_Title_Case_Description.ext`

```
assets/inventory/027_Waveshare_PanTilt_HAT_Front.jpg
assets/assembly/003_Car_Chassis_Bottom_Wiring.jpg
```

- `NNN` is a three-digit inventory number.
- Once assigned, a number is never reused, reordered, or recycled.
- The numbering sequence is global across `assets/inventory/` and `assets/assembly/`.
- The current highest number is `103`. Number `048` is intentionally unused. The next photo starts at `104`.

Title case is intentional here. The number is part of the identity, and these filenames are easier
to browse visually than kebab case. This is the only repository-wide exception to the standard naming style.

### 3.4 Reference Images: `lower-kebab-case`

Use an ISO date prefix when the image is tied to a dated observation.

```
assets/reference/raspberry-pi-5/gpio-pinout-diagram.png
assets/reference/nezha/2026-07-30-stm32-car-wiring-diagram.png
```

`assets/reference/` stores unnumbered diagrams, screenshots, and spec sheets grouped by source.

### 3.5 Hard Prohibitions

Rename these before adding them to the repository:

```
bad: Screenshot 2026-07-30 at 3.30.58 AM.png
bad: IMG_0325.JPG
bad: G SDA SCL 5V.JPG
bad: untitled.pdf

good: assets/reference/nezha/2026-07-30-i2c-header-g-sda-scl-5v.jpg
good: assets/inventory/091_HXS_18650_Battery_Pack_Label.jpg
```

- No spaces
- No non-English first-party filenames
- File extensions must be lowercase
- Dates must use ISO `YYYY-MM-DD`

Exceptions:

- `vendor/`, where original filenames are preserved for traceability.

### 3.6 Runnable Scripts in `examples/`: `NN_<tool>_<function>[_<mode>].py`

A reader scanning `ls examples/` must be able to tell **which hardware a script drives** without
opening it. The filename therefore names the tool before it names the task.

`examples/` itself is split into two folders on the same axis used for `tests/`: `ai_camera/` for
every script that actually reads the IMX500 (including ones that fuse it with another sensor, e.g.
`22_cam_sonar_patrol_capture.py`), and `other/` for everything else. A script keeps its `cam` tag
even if placed in `other/` when it does not actually import a camera module — that is a naming bug
to fix (tag and filename together), not a placement decision; `30_cam_motion_calibrate.py` is the
current example (it calibrates from operator-measured tape distance, not from the camera).

```
good: 26_cam_line_follow_drive.py       camera-guided line following
good: 39_map1_ir_line_follow.py         same route, IR sensor instead
good: 18_sonar_wall_follow_capture.py   the wall is measured by HC-SR04, not seen
bad:  03_drive.py                       drive with what?
bad:  22_fused_patrol_capture.py        fused from which two sensors?
bad:  38_map1_line_follow.py            camera or IR? both scripts exist
```

**`NN`** is the two-digit creation sequence. Like asset numbers (§3.3) it is never reused,
reordered, or recycled — progress logs and handoffs cite scripts by number, and gaps (`19`, `28`)
record scripts that were removed. Renaming a script keeps its number and changes only the
descriptive part.

**`<tool>`** comes from this closed vocabulary. Extend the table in the same commit that
introduces a new tag.

| Tag | Hardware / subsystem |
|---|---|
| `i2c` | I2C bus and NeZha board communication |
| `motor` | Motors and encoders only, with no external sensor feedback |
| `servo` | Servos |
| `cam` | Raspberry Pi AI Camera (IMX500), including anything read through it such as AprilTags |
| `sonar` | HC-SR04 ultrasonic sensor |
| `ir` | Yahboom 4-channel IR tracing sensor |
| `power` | Battery and supply health |
| `all_sensors` | Deliberately exercises every attached sensor, e.g. the preflight check |

Combine tags with `_` in reading order when a script genuinely fuses two sources
(`22_cam_sonar_patrol_capture.py`). A route or map scope may precede the tool when several scripts
target the same course (`38_map1_cam_line_follow.py`, `39_map1_ir_line_follow.py`).

**`<function>`** is `noun_verb`, not `verb_noun` — `motion_calibrate`, not `calibrate_motion` — so
that related scripts sort together. Reuse the established verbs: `check` (read and report, no
motion), `calibrate`, `capture`, `drive`, `sweep`, `probe`, `log`.

The tool tag must match what the script actually imports. A script whose docstring claims a sensor
it never reads is a naming bug; fix the docstring and the filename together.

### 3.7 Dated Work Records

Use one role per document; do not duplicate the same mutable status in several files.

| Kind | Canonical location and name | Contents |
|---|---|---|
| Development work log | `docs/progress/YYYY-MM-DD-topic.md` | Completed work, verification, hardware evidence, and problems encountered |
| Developer continuation | `docs/handoff-YYYY-MM-DD-topic.md` | Current Git/runtime state, remaining risks, acceptance gates, and exact next steps |
| Stable procedure | `docs/setup/lower-kebab-case.md` | Repeatable setup or operating instructions |
| Stable hardware fact | `docs/hardware/lower-kebab-case.md` | Wiring, protocols, limits, and verified device behavior |
| Architecture decision | `docs/adr/NNNN-lower-kebab-case.md` | A durable decision and its rationale |

Every completed, verifiable development session must be recorded in `docs/progress/` before the
session's commit. A work log must include:

1. Scope and result: what changed, what was intentionally left unchanged, and why.
2. Verification: exact automated commands and results; for hardware, the operator-observed result.
3. Measurements and configuration: physical dimensions, calibration inputs, device/firmware state,
   and safety conditions that affect repeatability.
4. Problems encountered: failed approaches, root cause when known, and the corrected procedure.
5. Follow-up: remaining limitations and the next safe, testable step.

Continue the same `YYYY-MM-DD-topic.md` when work on the same topic resumes that day. Create a new
file when the date or topic changes; do not make one generic forever-growing journal. Write facts
from completed work in past tense. Planned or unfinished work belongs in a handoff or issue, not in
the completed-results section.

A handoff is required only when work is being transferred or intentionally paused. It is a
snapshot, not a second source of truth and not a replacement for the work log. Link to stable setup,
hardware, ADR, or progress documents instead of copying their full content. When a newer handoff
supersedes an older one, state that relationship at the top of the newer file.

### 3.8 Calibration and Mapping Evidence

Use the session key `YYYY-MM-DD-device-resolution` consistently:

```text
assets/reference/camera-calibration/<session>/calibration.json
scratch/camera-calibration/<session>/source-frames/view-NN.jpg
scratch/mapping/<session>/
```

- Commit only reviewed, shareable calibration outputs needed at runtime, such as
  `calibration.json` and printable reference PDFs.
- Raw camera frames, room photographs, rejected views, annotated previews, maps under active
  investigation, and other privacy-sensitive evidence stay under `scratch/`.
- A progress or handoff document must record the local `scratch/` path, capture resolution, printed
  target dimensions, and which frames were accepted. Do not make code depend on ignored raw files.
- Once a generated diagram or PDF becomes a maintained project artifact, keep its reproducible
  generator under `scripts/` or document the authoritative external source beside the artifact.

## 4. `vendor/` Import Rules

Every `vendor/<supplier>/` directory must contain a `README.md` that records:

1. Supplier name and official link
2. Download date and version
3. Location of the original archive or media
4. What was excluded during import

Always remove generated build artifacts such as `.o`, `.crf`, `.d`, `.lst`, `.dep`, `.map`,
`.axf`, and `.uvoptx`. The full ignore list lives in `.gitignore`. These files can be rebuilt and
consume most of the unnecessary space in imported SDKs.

## 5. Website Data

Keep page content in data files instead of hardcoding it inside HTML or component markup.

The website uses a single shared dataset with English content under `i18n.en`:

```js
{
  id, number, name, category, tags, images,
  i18n: {
    en: { title, desc, specs, ... }
  }
}
```

`images` stores paths relative to the repository root, such as
`assets/inventory/001_Example_Module.jpg`.

Current data files:

| File | Purpose |
|---|---|
| `site/src/data/modules.json` | Inventory catalog |
| `site/src/data/assembly-guide.json` | Assembly guide content |
| `site/src/data/categories.ts` | Inventory category labels |

After changing website data, run:

```bash
uv run python scripts/check_inventory_data.py
```

The validator checks that assets exist, IDs are unique, English fields are present, and asset
filenames follow the `NNN_` rule from §3.3.

### Local Preview

```bash
npm run dev
```

The site is served at <http://localhost:4321/Car-and-Robotic-Arm/> so the local path matches the
GitHub Pages base path.

## 6. Git

### Commit Messages

Use Conventional Commits, with the top-level folder as the scope when helpful:

```text
docs: add NeZha register mapping notes
src: add Python driver for the NeZha board
assets: add binocular camera photos 092-093
vendor: import 37-in-1 sensor kit vendor files
site: fix inventory image paths
chore: add gitignore
```

Prefer commit bodies that explain **why** the change exists rather than repeating the diff.

### Size Guardrails

This repository contains many binary assets. Keep these limits in mind:

| Item | Current state | Limit |
|---|---|---|
| Working tree | ~52MB (excluding `node_modules/`, `.venv/`, build output) | n/a |
| `.git` history | ~102MB | GitHub recommends under 1GB |
| Largest file | ~2.4MB | GitHub hard limit is 100MB |
| `assets/` | ~45MB | GitHub Pages publish limit is 1GB |
| `vendor/` | Empty (2026-08-22) | Import only what the project actually cites |

Rules:

- Compress single photos below **1MB** before committing them.
- PDF and document-like files should stay below **10MB** unless discussed first.
- Videos, firmware images, and archives do not belong in this repository.

`git rm` does not shrink history by itself. The repository history was rewritten once on
2026-07-30 with `git filter-repo` to reduce size. That kind of operation is destructive and must
always be discussed before repeating it.

## 7. Scratch Files

Experimental output and temporary files belong in `scratch/`, which is already ignored. Use the
operating system's `/tmp` for disposable runtime output. Never create a repository-root `tmp/`.
Do not scatter files such as `test.py`, `tmp.json`, or generic placeholders in the repository root.

Generated directories such as `_site/`, `.astro/`, `.pytest_cache/`, `.ruff_cache/`,
`__pycache__/`, `node_modules/`, and `.venv/` are not project records. They remain ignored and may
be regenerated; never cite them as the only copy of evidence. Keep lockfiles and source inputs.

Before deleting an unfamiliar file, search for references and inspect Git status. Move private raw
evidence to its canonical `scratch/` location rather than deleting it merely to obtain a clean
worktree. Use the system Trash for recoverable cleanup when practical.

## 8. Checklist Before Adding a File

1. Who created it? Choose the top-level directory from §2.
2. Is there already a canonical file for the same fact? Update or link to it instead of duplicating it.
3. Does the filename contain spaces, non-English text, or uppercase extensions?
4. Is it a publishable inventory/assembly photo? Assign the next number and never reuse an old one.
5. Does it expose a room or other private environment? Keep the raw file under `scratch/`.
6. Is it larger than 1MB? Compress it first.
7. Is it vendor material? Remove build artifacts and add a `README.md`.
8. Is it generated? Keep the generator/source, and do not commit rebuildable output unless it is a
   reviewed deliverable required by users or runtime code.

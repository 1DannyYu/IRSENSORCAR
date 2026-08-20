# `vendor/` Boundary Cleanup — Ground-View Targets Relocated

Date: 2026-08-19

## Scope and Result

`vendor/` held two loose PDFs at its root:

```
vendor/ground-view-calibration-target-10x5cm.pdf
vendor/ground-view-calibration-target-10x7.5cm.pdf
```

They are output of [scripts/generate_ground_view_target.py](../../scripts/generate_ground_view_target.py),
so they are first-party generated artifacts sitting in the one directory CONVENTIONS §2 reserves
for third-party material and CLAUDE.md hard rule 1 declares read-only. They were also outside any
`vendor/<supplier>/` directory, which §4 requires. Committed in `9cc9132` ("chore: move calibration
PDFs into vendor/"); this pass reverses that move.

Moved to `assets/reference/ground-view-calibration/`, following the existing sibling precedent
`assets/reference/camera-calibration/imx500-charuco-5x7-30mm-a4.pdf`, and renamed to state the
rectangle in the same unit the generator's flags use:

| Before | After |
|---|---|
| `vendor/ground-view-calibration-target-10x5cm.pdf` | `assets/reference/ground-view-calibration/ground-view-target-100x50mm-a4.pdf` |
| `vendor/ground-view-calibration-target-10x7.5cm.pdf` | `assets/reference/ground-view-calibration/ground-view-target-100x75mm-a4.pdf` |

Both recorded by git as 100% renames. Added
[assets/reference/ground-view-calibration/README.md](../../assets/reference/ground-view-calibration/README.md)
with the exact reproduce commands, satisfying §3.8's requirement that a maintained generated
artifact keeps a reproducible generator or a documented authoritative source beside it.

`vendor/` now contains only `raspberry-pi/` and `yourfun-nezha/`, both with the §4 README.

## Recovering the Generation Parameters

No document recorded what flags produced these two files, and the generator's `DEFAULT_OUTPUT`
writes to `scratch/`, so the committed copies were hand-placed. Rather than infer the sizes from
the filenames, the parameters were read out of the PDFs themselves — the generator draws its own
dimensions into the page:

```
Rectangle is 100 x 50 mm (10.0 x 5.0 cm) if printed correctly.
Rectangle is 100 x 75 mm (10.0 x 7.5 cm) if printed correctly.
```

So `--width-mm 100 --height-mm 50` and `--width-mm 100 --height-mm 75`, exactly.

## Verification

Both files were regenerated from the current script with the recovered flags and their decoded
content streams compared against the committed copies:

- **100 × 50 mm** — identical except one footer string, which embeds
  `examples/27_ground_view_calibrate.py`. That example was renamed to `27_cam_ground_view_calibrate.py`
  earlier the same day, so the committed PDF prints a path that no longer resolves. Geometry
  unaffected.
- **100 × 75 mm** — 159 content-stream lines differ. Its corner crosshairs use reportlab's default
  0.75 pt hairline; the committed generator draws them at `BORDER_WIDTH_MM` (5 mm), and that
  constant has been 5 mm since the generator's first commit `b3172e1`. This PDF was therefore
  produced by a generator version that is not in git. Rectangle dimensions and tick positions are
  unaffected; only the corner markers are thinner.

```bash
uv run python scripts/check_inventory_data.py   # ✓ Dataset is complete
```

The inventory validator passes — the new directory is under `assets/reference/`, which is exempt
from the `NNN_` rule of §3.3.

## Problems Encountered

The PDFs are ASCII85 + Flate encoded, so a plain `stream`/`endstream` extraction yields nothing
readable; both filters have to be undone in order before the text operators are visible.

## Follow-up

- Neither PDF was regenerated, because they are what was physically printed and calibrated against,
  and replacing them would silently diverge from whatever is currently taped to the floor. Reprint
  both from the README's commands when the floor target is next replaced — the bold crosshairs are
  a deliberate improvement, recorded in the generator as surviving the camera blur that erases a
  hairline.
- `generate_ground_view_target.py` still defaults `--output` to `scratch/`. That is correct for
  casual runs and should stay, but it means the maintained copies can only be refreshed by passing
  `--output` explicitly, as the README does.
- An untracked `Tracing Sensor.png` is sitting in the repository root. It violates §3.5 (spaces in
  a first-party filename) and §7 (no scattered files at the root). It is a vendor product photo of
  the Yahboom 4-channel tracing sensor; if it is worth keeping it belongs in
  `assets/reference/yahboom/` with a kebab-case name, otherwise in `scratch/`. Left in place
  pending a decision.

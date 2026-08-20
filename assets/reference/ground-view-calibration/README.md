# Ground-View Calibration Targets

Printable floor rectangles of known real-world size, used to calibrate the bird's-eye ground view.
Print at **100% / actual size — not "fit to page"** and measure the printed rectangle with a ruler
before taping it down; a print-scale error silently corrupts every downstream measurement.

Consumed by [examples/27_cam_ground_view_calibrate.py](../../../examples/27_cam_ground_view_calibrate.py)
via `--corners`, whose value is produced by clicking the four crosshairs in TL, TR, BR, BL order
with [scripts/pick_ground_view_corners.py](../../../scripts/pick_ground_view_corners.py).

## Files and how to reproduce them

Generator: [scripts/generate_ground_view_target.py](../../../scripts/generate_ground_view_target.py).
`reportlab` is not a project dependency, so run it ephemerally.

| File | Rectangle | Command |
|---|---|---|
| `ground-view-target-100x50mm-a4.pdf` | 100 × 50 mm (10.0 × 5.0 cm) | see below |
| `ground-view-target-100x75mm-a4.pdf` | 100 × 75 mm (10.0 × 7.5 cm) | see below |

```bash
uv run --with reportlab python3 scripts/generate_ground_view_target.py \
    --width-mm 100 --height-mm 50 \
    --output assets/reference/ground-view-calibration/ground-view-target-100x50mm-a4.pdf
```

```bash
uv run --with reportlab python3 scripts/generate_ground_view_target.py \
    --width-mm 100 --height-mm 75 \
    --output assets/reference/ground-view-calibration/ground-view-target-100x75mm-a4.pdf
```

The dimensions above were recovered from the PDFs' own embedded text
(`Rectangle is 100 x 50 mm (10.0 x 5.0 cm) if printed correctly.`), not inferred from the
filenames, so they are exact.

## Provenance of the committed copies

These two files were committed in `9cc9132` under `vendor/`, which CONVENTIONS §2 reserves for
third-party material and CLAUDE.md hard rule 1 declares read-only. They are first-party generator
output, so they were moved here on 2026-08-19. `vendor/` now contains only supplier directories, as
CONVENTIONS §4 requires.

Neither committed PDF is byte-identical to what the current generator produces. Both were kept as
they are, rather than regenerated, because they are what was physically printed and calibrated
against — but the differences are worth knowing before you reprint:

- **`ground-view-target-100x50mm-a4.pdf`** — reproduces exactly except for one footer string, which
  names `examples/27_ground_view_calibrate.py`. That script was renamed to
  `27_cam_ground_view_calibrate.py` on 2026-08-19, so the printed footer now points at a path that
  no longer exists. Geometry is unaffected.
- **`ground-view-target-100x75mm-a4.pdf`** — differs substantially (159 content-stream lines). Its
  corner crosshairs are drawn at reportlab's default 0.75 pt hairline, while the committed
  generator draws them at `BORDER_WIDTH_MM` (5 mm). That width has been 5 mm since the generator's
  first commit `b3172e1`, so this PDF was produced by a generator version that is not in git. The
  rectangle dimensions and tick positions are unaffected; only the corner markers are thinner.

The hairline is the weaker artifact on purpose-built grounds: the generator's own comment records
that a hairline is the first thing a manual-focus close-up camera loses to blur, which is why the
crosshairs were made bold. Regenerating the 100 × 75 mm target from the committed script would
produce a better print, at the cost of no longer matching whatever is currently taped to the floor.
Reprint both from the commands above when the floor target is next replaced.

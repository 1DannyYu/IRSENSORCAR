# Yahboom Reference Images

Vendor-published diagrams for Yahboom modules used on this build. These are
reference material we did not photograph ourselves, kept here rather than under
`vendor/` because they are loose images with no supplier import record — see
CONVENTIONS §3.4, which is where unnumbered diagrams and spec images belong.

| File | What it shows |
|---|---|
| `yb-mvx01-tracing-sensor-channel-labels.png` | The YB-MVX01 4-channel tracing sensor with the vendor's own channel labels: `P1`/`P4` marked "Edge detection", `P2`/`P3` marked "Tracking in line", and the `VCC X1 X2 X3 X4 GND` header |

## Note on the vendor's channel roles

The image labels the outer pair as edge detectors and the tight middle pair as
the tracking pair. **This project does not implement that split** — all four
channels are weighted by measured lateral position instead, in
[`carbot.ir_geometry`](../../../src/carbot/ir_geometry.py). The vendor's
grouping matches the physical spacing we measured (`2.8 / 0.8 / 2.8 cm`, so the
middle pair really is tight and the outer two really are far out), but the
geometry alone determines the behaviour, so no separate "edge" mode is needed.

The image also does **not** record which header pin `X1`..`X4` belongs to which
physical position. That mapping was measured on hardware on 2026-08-19 and is
recorded in
[docs/hardware/ir-tracing-sensor.md](../../../docs/hardware/ir-tracing-sensor.md);
it is not the naive `X1 = P1` order.

Resized to 1400px wide and re-encoded to stay under the 1MB per-photo limit in
CONVENTIONS §6; the original was 1810x1092 at 1.5MB.

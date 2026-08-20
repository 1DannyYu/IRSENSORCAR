# Camera Modes, Exposure, and the Preflight Fix — Work Log (2026-08-14)

Continues [`2026-08-14-visual-mapping-and-patrol.md`](2026-08-14-visual-mapping-and-patrol.md)
and its handoff
[`handoff-2026-08-14-visual-mapping-and-patrol.md`](../handoff-2026-08-14-visual-mapping-and-patrol.md).
This session did the groundwork the fused patrol needs and **answered two of the
handoff's assumptions with a "no"**.

## 1. Scope and Result

Completed:

- **Preflight unblocked.** The "persistent battery undervoltage" that gated all
  motor work was a decoding bug, not a power fault. New
  [`src/carbot/power.py`](../../src/carbot/power.py) is the single decoder;
  [`examples/08_battery_check.py`](../../examples/08_battery_check.py) and
  [`examples/14_all_sensors_preflight_check.py`](../../examples/14_all_sensors_preflight_check.py) use it.
  All five preflight checks now pass.
- **Detection logic factored out**, as the handoff asked:
  [`src/carbot/vision_avoid.py`](../../src/carbot/vision_avoid.py) holds
  `Detection`, `ObstaclePolicy`, `is_blocking`, `fuse`, and the one
  hardware-facing `detections_from_metadata`.
  [`examples/20_cam_detection_check.py`](../../examples/20_cam_detection_check.py)
  now consumes the module instead of carrying its own copy.
- **SfM frame-quality measurement**:
  [`src/carbot/frame_quality.py`](../../src/carbot/frame_quality.py) reports
  sharpness, exposure, and per-tile keypoint spread separately, plus
  `repeatable_keypoints` for matches across two views.
- **Camera experiments** (no motors):
  [`examples/21_cam_dual_mode_check.py`](../../examples/21_cam_dual_mode_check.py)
  compares the three capture modes and sweeps auto-exposure.

Two handoff assumptions did not survive measurement:

| Handoff said | Measured |
|---|---|
| "Camera-mode switching (the hard part)" — build a preview↔still switch | **Not needed.** The IMX500 delivers inference *and* 2028×1520 stills from one configuration |
| Blur was the leading suspect for 9/40 SfM registration | **Not blur.** Focus is fine; the frame was half blank wall and underexposed |

## 2. Verification

```text
# Mac (this machine)
uv run --extra vision --extra mapping pytest -q   -> 175 passed (was 97)
uv run ruff check .                               -> All checks passed

# Pi: preflight, after the get_throttled fix
PYTHONPATH=src python3 examples/14_all_sensors_preflight_check.py
  [OK] camera / i2c-nezha / hc-sr04 / encoders
  [OK] power  EXT5V_V=4.915 V (OK); get_throttled=0x50000 | no live throttling
              | since boot: undervoltage, throttled; temp=41.7 °C (OK)
  ✓ All preflight checks passed — safe to proceed to a supervised motion test.

# Pi: capture-mode comparison
PYTHONPATH=src python3 examples/21_cam_dual_mode_check.py --check modes
  mode      tensor  dets       still  sharp  bright  textured  capture  resume
  single       5/5     0   2028x1520     53     123      9/12    0.05s   0.00s
  switch       5/5     0   2028x1520     56     128      9/12    0.37s   0.11s
  restart      5/5     0   2028x1520     55     128      9/12    1.07s   0.13s
  -> Use mode 'single' in the fused patrol.

# Pi: refactored detection path still labels correctly
PYTHONPATH=src python3 examples/20_cam_detection_check.py --frames 2 --threshold 0.10
  chair conf=0.27 / dining table / bench / tv    (was a flood of "toilet" — see pitfall 3)
```

The `detections_from_metadata` port was exercised against real IMX500 metadata:
an earlier run of `21` reported `refrigerator 0.32`, `sink`, `vase` and fused
them into a verdict, so the moved adapter behaves on hardware.

## 3. Measurements and Configuration

**Capture mode.** Mode `single` is one `create_preview_configuration` with
`main={"size": (2028, 1520)}` and `controls={"FrameRate": inference_rate}`,
`buffer_count=4`. The CNN output tensor arrives on 5/5 frames at that
resolution, so the patrol needs no mode switch and no inference-resume wait.
`switch` (via `switch_mode_and_capture_file`) and `restart` both work as
fallbacks and neither drops the loaded network; they cost 0.37 s and 1.07 s per
capture against 0.05 s.

**Exposure sweep** (mode `single`, 11 settings, static scene, 2 s AE settle):

```text
setting            bright   clip  textured  keypts  repeat  shutter   gain
default               152  0.3%    9/12      5744    3601   38.2ms    7.5
ev+0.5                178  1.1%    9/12      6458    3647   38.2ms   11.0
ev+1.0                200  3.9%    9/12      6907    3644   38.2ms   15.5
ev+1.5                200  3.9%    9/12      6944    3622   38.2ms   15.5
shadows               188  2.1%    9/12      6672    3633   38.2ms   12.8
shadows+ev1.0         200  3.9%    9/12      6940    3650   38.2ms   15.5
centre-metered        153  0.3%    9/12      5855    3598   38.2ms    7.6
spot-metered          171  0.8%    9/12      6570    3613   38.2ms   10.1
long-shutter          152  0.3%    9/12      5807    3611   72.3ms    4.0
long-shutter+spot     171  0.8%    9/12      6660    3684   97.4ms    4.0
default (control)     153  0.3%    9/12      5797    3619   38.2ms    7.6
```

- **Winner: `long-shutter+spot`** — `FrameDurationLimits=(100000, 100000)` plus
  `AeMeteringMode=Spot`. Highest repeatable keypoints (3684) at the lowest
  analogue gain (4.0).
- **Run-to-run control**: the same `default` setting measured 3601 then 3619
  (±18). Spread across settings was 86, so the ranking is ~4.6× the noise floor
  and therefore real.
- **The effect is small: +2.3% repeatable keypoints.** `textured` stayed 9/12
  for *every* setting — exposure never unlocked a new part of the frame. The
  three empty tiles are the top row (ceiling), which is camera aim, not
  exposure. **Exposure is not the fix for 9/40 registration.**
- **Open**: the 97.4 ms shutter is untested on a car that has only just
  stopped. Fallback if it blurs is `spot-metered` (38.2 ms, gain 10.1,
  3613 repeatable — 71 fewer).
- Baseline frame duration for this configuration is 38450 µs, restored between
  settings so a widened duration cannot leak into the control.

**Frame-quality thresholds** in `QualityPolicy` (`min_tile_keypoints=40`,
`min_textured_tiles=6`, `min_mean_brightness=60`, `max_clipped_fraction=0.05`,
`min_sharpness=20`) are **provisional** — derived from one capture, not from a
correlation against COLMAP output. Calibrate them in the sweep by recording
which frames `scripts/run_colmap_sfm.py` registers.

**Platform**: picamera2 0.3.36, numpy 2.2.4 on the Pi; `switch_mode` and
`switch_mode_and_capture_file` both available. `AeConstraintMode`,
`AeMeteringMode`, `AeExposureMode` all expose ranges 0-3; `ExposureValue` is
-8..8; `AnalogueGain` tops out at 22.26.

## 4. Problems Encountered (the pitfalls)

1. **`vcgencmd get_throttled` halves were inverted in both power scripts.** The
   **low nibble** (`0x1`/`0x2`/`0x4`/`0x8`) is the live state; bits 16-19
   (`0x10000`+) are sticky since-boot history. `examples/14` masked `0x1F0000`
   and called it "now"; `examples/08` labelled `0x1`-`0x8` "past" and
   `0x10000`+ "current". The observed `0x50000` means undervoltage and
   throttling *occurred since boot*, with nothing throttling now — and the rail
   measures 4.9 V. This false failure was recorded as a real battery fault in
   three earlier logs. A since-boot flag must never fail a gate: it stays set
   until reboot, so one power dip blocks motor work for the rest of the session.
2. **Whole-frame Laplacian variance cannot tell blur from a blank wall.** The
   first capture scored 40 and looked out of focus. Per-tile: furniture tiles
   127-139 with ~1500 ORB keypoints, white-wall tiles 4-15 with 0-164; mean
   brightness 70/255, 4.9% crushed to black, 1.8% clipped. Focus was fine. The
   metric conflates blur, missing texture, and bad exposure — three different
   fixes.
3. **The model's labels are not the 80-class COCO list.** The SSD mobilenetv2
   `.rpk` carries **90** labels in the COCO-91 index space: 61 = chair,
   81 = refrigerator, 84 = clock. In the common 80-class list index 61 is
   `toilet`, so chairs printed as "toilet" and indices 81+ fell off the end.
   `examples/20` happened to be correct only because of its
   `intrinsics.labels or COCO_LABELS` short-circuit; the wrong table sat in the
   file waiting to be copied, and the refactor copied it. `vision_avoid` now
   keeps **no** label table — names come from the network, and an unnamed
   detection reports its raw index rather than a guess.
4. **Raw keypoint counts are inflated by analogue gain.** `ev+1.5` reports 21%
   more keypoints than the default at gain 15.5 but matches *fewer* of them
   across two frames. Ranking exposure by raw counts would have picked the
   noisiest setting. `repeatable_keypoints` (ORB + Lowe ratio test between two
   captures) is the honest measure.
5. **Shutter time could not rank the exposure settings.** The first eight
   settings all settled on exactly 38.2 ms because `FrameRate` caps the frame
   duration, leaving AE only gain to work with. An initial hard 33 ms blur veto
   therefore rejected every setting and produced no recommendation.
   `FrameDurationLimits` is what lets AE spend time instead of gain.
6. **A 3% difference is meaningless without a control.** The between-setting
   spread only became interpretable after repeating `default` at the end of the
   sweep to measure run-to-run variation (±18).
7. **A widened `FrameDurationLimits` leaks into later settings.** libcamera
   keeps controls until overwritten, so `long-shutter` would have corrupted the
   control group that follows it. Every setting now starts from an explicitly
   restored baseline.
8. **`picamera2.__version__` does not exist** — use
   `importlib.metadata.version("picamera2")`.
9. **`capture_file` does not tell you the exposure of the frame it saved.** Use
   `capture_request()`, read `request.get_metadata()`, and always
   `request.release()` — a leaked request starves the buffer pool.

## 5. Follow-up

Next is the fused patrol itself (step 4): mode `single`, `vision_avoid.fuse`,
back up 0.6 s before a random 30-150° turn, `--dry-run` for a stationary logic
test. It is the first motor-moving work since the preflight was unblocked, so an
operator must be beside the car.

Two questions only a moving test can answer:

1. Does the 97.4 ms shutter of `long-shutter+spot` blur once the car stops?
   A/B it against `spot-metered` in the first supervised run.
2. Does the settle time need to exceed 1.0 s at that shutter?

Still open from the earlier log: re-measure spin rate at the chosen speed, and
the real registration fix — camera aim, route overlap, and scene texture, none
of which exposure addressed.

"""Bird's-eye ground-view calibration and 2 cm line lock."""

from __future__ import annotations

from dataclasses import replace

import numpy as np
import pytest

from carbot.ground_view import (
    auto_calibrate_ground_view,
    calibrate_ground_view,
    detect_line_on_ground,
    find_target_corners,
    load_ground_view,
    save_ground_view,
)
from carbot.line_follow import detect_line

pytest.importorskip("cv2", reason="ground view needs OpenCV")

PAPER = 208
LINE = 40


def _view():
    image_pts = np.array([[0, 0], [299, 0], [299, 399], [0, 399]], dtype=float)
    world_pts = np.array(
        [[-0.30, 0.92], [0.30, 0.92], [0.30, 0.12], [-0.30, 0.12]],
        dtype=float,
    )
    return calibrate_ground_view(
        image_pts,
        world_pts,
        x_min_m=-0.30,
        x_max_m=0.30,
        y_min_m=0.12,
        y_max_m=0.92,
        metres_per_pixel=0.002,
    )


def _topdown_with_line(x: int = 150, width: int = 10) -> np.ndarray:
    image = np.full((400, 300), PAPER, dtype=np.uint8)
    image[:, x - width // 2 : x + width // 2] = LINE
    return image


def test_centred_2cm_line_in_birds_eye_has_near_zero_error():
    view = _view()
    reading = detect_line_on_ground(_topdown_with_line(), view)
    assert reading.visible
    assert reading.axis == "vertical"
    assert reading.error_fraction is not None
    assert abs(reading.error_fraction) < 0.08


def test_line_to_the_right_in_birds_eye_reports_positive_error():
    view = _view()
    reading = detect_line_on_ground(_topdown_with_line(x=210), view)
    assert reading.visible
    assert reading.error_fraction is not None
    assert reading.error_fraction > 0.15


def test_far_distractor_does_not_steal_lock_from_near_field_line():
    """A stray 2 cm-scale feature farther out (outer-loop curve, junction
    cross-bar, calibration-target crosshair) must not outvote the near-field
    line just because it happens to sit closer to the BEV centre in x — this
    is the jump-stop failure from the 2026-08-16 Gate B run."""
    view = _view()
    image = np.full((400, 300), PAPER, dtype=np.uint8)
    # Near-field real line (closest to the wheels), off-centre.
    image[300:400, 165:175] = LINE
    # Far-field distractor, closer to BEV centre (x=150) than the real line.
    image[100:200, 145:155] = LINE
    reading = detect_line_on_ground(image, view)
    assert reading.visible
    assert reading.error_fraction is not None
    assert reading.error_fraction > 0.1


def test_prefer_u_keeps_lock_on_previously_tracked_line_over_closer_distractor():
    """A junction feature (outer-loop curve, T cross-bar) can become just as
    near and just as 2 cm-wide as the real line by the time the car is close
    to a junction — nearness and width alone cannot separate them there
    (2026-08-16 Gate B: the false lock read width=9-10px at rows=27-33,
    indistinguishable from the real line on those signals). Continuity is the
    remaining signal: keep the line the car was already tracking."""
    view = _view()
    image = np.full((400, 300), PAPER, dtype=np.uint8)
    # Already-tracked line, off-centre.
    image[300:400, 165:175] = LINE
    # New near-field distractor, closer to BEV centre and clearly a separate
    # cluster (not just sampling noise on the same stroke) — must not steal
    # lock just because it is more central.
    image[300:400, 135:145] = LINE
    reading = detect_line_on_ground(image, view, prefer_u=170.0)
    assert reading.visible
    assert reading.error_fraction is not None
    assert reading.error_fraction > 0.1


def test_prefer_u_falls_back_to_centre_pick_once_the_tracked_line_is_gone():
    view = _view()
    image = np.full((400, 300), PAPER, dtype=np.uint8)
    image[300:400, 145:155] = LINE  # only a centred line remains
    reading = detect_line_on_ground(image, view, prefer_u=999.0)  # stale lock
    assert reading.visible
    assert reading.error_fraction is not None
    assert abs(reading.error_fraction) < 0.1


def test_wide_near_bar_is_reported_as_a_horizontal_crossing():
    """A T cross-bar is wide in a single BEV row (it runs across the car's
    heading); the outer-loop curve that fooled the vertical scan never is —
    row by row it stayed ~2 cm wide, just offset in x as it curved. A wide,
    near, persistent bar should be reported as `axis="horizontal"` so
    `LineNav`'s existing T-turn state machine (`_is_near_t`) can act on it."""
    view = _view()
    image = np.full((400, 300), PAPER, dtype=np.uint8)
    image[340:380, 50:250] = LINE  # near field (y close to 400), 200px wide, centred
    reading = detect_line_on_ground(image, view)
    assert reading.visible
    assert reading.axis == "horizontal"
    assert reading.line_width_px >= 52  # NavPolicy.t_bar_min_width_px default (15 mm line)
    assert reading.error_fraction is not None
    assert abs(reading.error_fraction) < 0.15


def test_far_wide_bar_does_not_trigger_crossing_detection():
    """A wide dark feature that has not reached the near field yet (still far
    ahead, e.g. the outer loop just entering view) must not be mistaken for
    arrival at a crossing — only a real, near cross-bar should switch axis."""
    view = _view()
    image = np.full((400, 300), PAPER, dtype=np.uint8)
    image[100:140, 50:250] = LINE  # wide, but far (small y)
    image[300:400, 145:155] = LINE  # real near-field 2 cm path, centred
    reading = detect_line_on_ground(image, view)
    assert reading.visible
    assert reading.axis == "vertical"


def test_padding_outside_the_captured_frame_is_not_mistaken_for_a_crossing():
    """`warpPerspective` samples pixel locations implied by the calibration,
    not by the actual captured frame size. The near edge of a wide BEV window
    can extrapolate past the bottom of the raw frame; that region has to read
    as background, not as a dark crossing that would trigger a turn — a false
    "wide dark span" from black warp padding was misread as a T cross-bar in
    the 2026-08-16 capture (a real capture, not a synthetic one: `width` came
    back at 7794px, and every one of the 47 scanned near rows "found" it)."""
    view = _view()
    image = np.full((250, 300), PAPER, dtype=np.uint8)  # shorter than the calibrated 400 rows
    reading = detect_line_on_ground(image, view)
    assert not reading.visible


def test_exclude_world_box_keeps_a_marked_region_out_of_detection():
    """A calibration target left in view is itself a small dark mark near
    the 2 cm scale — excluding its known world footprint (rather than hoping
    width/position heuristics reject it) is how `auto_calibrate_ground_view`
    keeps line detection from locking onto the target instead of the real
    line (a suspected cause of a 2026-08-16 run reading a rock-steady error
    for 12 straight frames despite the car actually moving)."""
    view = _view()
    excluded = replace(view, exclude_world_box_m=(-0.05, 0.05, 0.70, 0.90))
    image = np.full((400, 300), PAPER, dtype=np.uint8)
    # Decoy inside the excluded world box — would otherwise look like a line.
    image[10:100, 145:155] = LINE
    # Real near-field line, outside the excluded box.
    image[300:400, 200:210] = LINE
    reading = detect_line_on_ground(image, excluded)
    assert reading.visible
    assert reading.error_fraction is not None
    assert reading.error_fraction > 0.2


def test_auto_calibrate_ground_view_sets_the_target_exclusion_box():
    image = _target_image()
    view = auto_calibrate_ground_view(
        image,
        target_width_m=0.10,
        target_height_m=0.05,
        near_m=0.18,
    )
    assert view.exclude_world_box_m is not None
    x_lo, x_hi, y_lo, y_hi = view.exclude_world_box_m
    assert x_lo < -0.05 and x_hi > 0.05
    assert y_lo < 0.18 and y_hi > 0.23


def test_blank_paper_in_birds_eye_is_not_a_line():
    view = _view()
    image = np.full((400, 300), PAPER, dtype=np.uint8)
    reading = detect_line_on_ground(image, view)
    assert not reading.visible


def test_detect_line_uses_ground_view_when_provided():
    view = _view()
    reading = detect_line(_topdown_with_line(), ground_view=view)
    assert reading.visible
    assert reading.error_fraction is not None
    assert abs(reading.error_fraction) < 0.08


def test_ground_view_round_trips_through_json(tmp_path):
    view = _view()
    path = tmp_path / "ground-view.json"
    save_ground_view(path, view)
    loaded = load_ground_view(path)
    assert loaded.bev_width == view.bev_width
    np.testing.assert_allclose(loaded.homography, view.homography, rtol=1e-9)


def test_calibrate_rejects_fewer_than_four_points():
    with pytest.raises(ValueError):
        calibrate_ground_view([[0, 0], [1, 0], [1, 1]], [[0, 0], [1, 0], [1, 1]])


def _target_image(
    tl=(200, 150),
    br=(600, 450),
    thickness=10,
    size=(800, 600),
):
    import cv2

    image = np.full((size[1], size[0]), PAPER, dtype=np.uint8)
    cv2.rectangle(image, tl, br, color=0, thickness=thickness)
    return image


def test_find_target_corners_locates_a_synthetic_target():
    """A camera mount on a toy chassis is not rigid — it shifts every time
    the car is handled, so recalibration needs to be automatic rather than
    a manual four-click job (`scripts/pick_ground_view_corners.py`)."""
    image = _target_image()
    corners = find_target_corners(image)
    assert corners is not None
    tl, tr, br, bl = corners
    assert abs(tl[0] - 200) < 8 and abs(tl[1] - 150) < 8
    assert abs(tr[0] - 600) < 8 and abs(tr[1] - 150) < 8
    assert abs(br[0] - 600) < 8 and abs(br[1] - 450) < 8
    assert abs(bl[0] - 200) < 8 and abs(bl[1] - 450) < 8


def test_find_target_corners_returns_none_without_a_target():
    image = np.full((600, 800), PAPER, dtype=np.uint8)
    assert find_target_corners(image) is None


def test_auto_calibrate_ground_view_fits_from_the_target():
    image = _target_image()
    view = auto_calibrate_ground_view(
        image,
        target_width_m=0.10,
        target_height_m=0.05,
        near_m=0.18,
    )
    assert view.bev_width > 32
    assert view.exclude_world_box_m is not None


def test_detect_line_on_ground_prefers_near_field_stem_and_prefer_u():
    """Near-field stem trajectory must take precedence over far-field spurious curves/bars."""
    view = _view()
    image = np.full((400, 300), PAPER, dtype=np.uint8)
    # Near-field stem line centered in BEV
    image[250:350, 145:155] = LINE
    # Far-field spurious curve/bar
    image[100:150, 215:225] = LINE
    reading = detect_line_on_ground(image, view, prefer_u=150.0)
    assert reading.visible
    assert reading.ground_u_px is not None
    assert abs(reading.ground_u_px - 150.0) < 10.0


def test_auto_calibrate_ground_view_raises_without_a_target():
    image = np.full((600, 800), PAPER, dtype=np.uint8)
    with pytest.raises(ValueError):
        auto_calibrate_ground_view(
            image,
            target_width_m=0.10,
            target_height_m=0.05,
            near_m=0.18,
        )


# ----------------------------------------------------------------------
# 15 mm Task-1 reprint line semantics
# ----------------------------------------------------------------------


def test_15mm_line_in_birds_eye_is_detected_with_near_zero_error():
    """The Task-1 reprint map's route line is 15 mm wide — at the default
    2 mm/px BEV scale that is 7.5 px, not the old 2 cm (10 px) stroke. The
    detector's width band must accept it as the main line."""
    view = _view()  # default line_width_m = 0.015
    assert view.expected_line_width_px == pytest.approx(7.5, abs=1e-6)
    image = np.full((400, 300), PAPER, dtype=np.uint8)
    image[:, 146:154] = LINE  # 8 px wide stroke, near the 7.5 px expectation
    reading = detect_line_on_ground(image, view)
    assert reading.visible
    assert reading.axis == "vertical"
    assert reading.error_fraction is not None
    assert abs(reading.error_fraction) < 0.08
    assert reading.line_width_px == pytest.approx(8.0, abs=2.0)


def test_line_width_m_scales_the_detection_band():
    """The width band scales with ``line_width_m``: a 44 mm stroke is inside
    a 20 mm view's band (max 2.5 * 10 = 25 px) but outside a 15 mm view's
    band (max 2.5 * 7.5 = 18.75 px)."""
    image = np.full((400, 300), PAPER, dtype=np.uint8)
    image[:, 139:161] = LINE  # 22 px = 44 mm stroke
    view20 = replace(_view(), line_width_m=0.020)
    assert view20.expected_line_width_px == pytest.approx(10.0, abs=1e-6)
    assert detect_line_on_ground(image, view20).visible
    view15 = _view()
    assert not detect_line_on_ground(image, view15).visible


def test_line_width_m_round_trips_through_json():
    view = replace(_view(), line_width_m=0.020)
    loaded = load_ground_view(save_ground_view_roundtrip(view))
    assert loaded.line_width_m == pytest.approx(0.020)
    assert loaded.expected_line_width_px == pytest.approx(10.0, abs=1e-6)


def save_ground_view_roundtrip(view) -> str:
    import tempfile

    with tempfile.NamedTemporaryFile(suffix=".json", delete=False) as fh:
        save_ground_view(fh.name, view)
        return fh.name

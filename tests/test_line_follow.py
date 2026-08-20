"""Tests for downward black-line detection, using synthetic images.

Each fixture isolates one property: a centred vertical line reports a near-zero
error, a shifted line reports a proportional error, a blank floor reports "no
line", and a bottom band of chassis/shadow must not leak into the reading
because the ROI excludes it. The real 2026-08-15 capture is not committed, so
every case here is synthetic; the module docstring records what the real frame
measured (line ~230 px wide, threshold 90-120 all valid, chassis band below
68 % of the frame).
"""

from __future__ import annotations

import numpy as np
import pytest

from carbot.line_follow import LinePolicy, detect_line

pytest.importorskip("cv2", reason="line detection needs OpenCV")

HEIGHT, WIDTH = 480, 640
PAPER = 208  # verified background gray of the track map


def blank(paper: int = PAPER) -> np.ndarray:
    return np.full((HEIGHT, WIDTH), paper, dtype=np.uint8)


def vertical_line(x: int, width: int = 20) -> np.ndarray:
    """White paper with a black vertical strip centred on ``x``."""
    image = blank()
    image[:, x - width // 2 : x + width // 2] = 40  # verified line gray << 100
    return image


def chassis_band(image: np.ndarray, y_start: int = int(HEIGHT * 0.72)) -> np.ndarray:
    """Simulate the dark chassis/shadow strip at the bottom of the frame."""
    image = image.copy()
    image[y_start:, :] = 50
    return image


# --------------------------------------------------------------------- basics


def test_centred_line_reports_near_zero_error():
    reading = detect_line(vertical_line(WIDTH // 2))
    assert reading.visible
    assert reading.error_px is not None
    assert abs(reading.error_px) < 5
    assert abs(reading.error_fraction or 0) < 0.02
    assert reading.centroid_x == pytest.approx(WIDTH / 2, abs=5)


def test_line_to_the_left_reports_negative_error():
    reading = detect_line(vertical_line(WIDTH // 4))
    assert reading.visible
    assert reading.error_px is not None
    assert reading.error_px < 0
    assert reading.centroid_x == pytest.approx(WIDTH / 4, abs=10)


def test_line_to_the_right_reports_positive_error():
    reading = detect_line(vertical_line(3 * WIDTH // 4))
    assert reading.visible
    assert reading.error_px is not None
    assert reading.error_px > 0
    assert reading.error_fraction == pytest.approx(reading.error_px / (WIDTH / 2), abs=1e-9)


def test_error_fraction_is_normalised_to_unit_range():
    reading = detect_line(vertical_line(WIDTH // 4))
    assert -1.0 <= reading.error_fraction <= 1.0


def test_blank_floor_reports_no_line():
    reading = detect_line(blank())
    assert not reading.visible
    assert reading.error_px is None
    assert reading.error_fraction is None


def test_line_width_is_reported():
    reading = detect_line(vertical_line(WIDTH // 2, width=30))
    assert reading.line_width_px == pytest.approx(30, abs=5)


# ------------------------------------------------------- ROI / real-frame cases


def test_chassis_band_does_not_leak_into_the_reading():
    """A downward-camera ROI that stops at 0.68 must ignore the bottom band."""
    image = chassis_band(vertical_line(WIDTH // 2))
    reading = detect_line(image, LinePolicy(roi_top=0.10, roi_bottom=0.68))
    assert reading.visible
    assert reading.centroid_x == pytest.approx(WIDTH / 2, abs=5)


def test_full_width_chassis_band_is_not_a_2cm_line():
    """Forward-looking ROI includes the bottom; a full-width dark band is too wide."""
    reading = detect_line(chassis_band(blank()))
    assert not reading.visible


def test_line_only_inside_far_field_is_ignored():
    """Map ink above the look-ahead band is not the path by the wheels."""
    image = blank()
    image[: int(HEIGHT * 0.05), :] = 40  # top shadow
    image[int(HEIGHT * 0.05) : int(HEIGHT * 0.18), WIDTH // 2 - 10 : WIDTH // 2 + 10] = 40
    reading = detect_line(image)
    assert not reading.visible


def test_line_in_lookahead_band_is_visible():
    image = blank()
    image[int(HEIGHT * 0.68) : int(HEIGHT * 0.88), WIDTH // 2 - 10 : WIDTH // 2 + 10] = 40
    reading = detect_line(image)
    assert reading.visible
    assert reading.centroid_x == pytest.approx(WIDTH / 2, abs=5)


def test_threshold_headroom_matches_verified_frame():
    """The verified still separated at every threshold 90-120; policy must too."""
    for threshold in (90, 100, 110, 120):
        reading = detect_line(vertical_line(WIDTH // 2), LinePolicy(dark_threshold=threshold))
        assert reading.visible, f"threshold {threshold} lost the line"


def test_sparse_noise_does_not_read_as_a_line():
    """Scattered dark specks are under min_row_dark_fraction per row."""
    image = blank()
    rng = np.random.default_rng(3)
    xs, ys = rng.integers(0, WIDTH, 40), rng.integers(0, HEIGHT, 40)
    for x, y in zip(xs, ys):
        image[y : y + 2, x : x + 2] = 40
    reading = detect_line(image)
    assert not reading.visible


# ----------------------------------------------------------------- validation


def test_policy_rejects_invalid_thresholds():
    with pytest.raises(ValueError):
        LinePolicy(dark_threshold=256)
    with pytest.raises(ValueError):
        LinePolicy(roi_top=0.9, roi_bottom=0.1)
    with pytest.raises(ValueError):
        LinePolicy(min_tracked_rows=0)


def test_detect_line_rejects_non_image_input():
    with pytest.raises(ValueError):
        detect_line(np.zeros((10, 10, 5), dtype=np.uint8))  # 5 channels


# ----------------------------------------------------------- junction / forks


def y_fork(branch_rows: int = 80) -> np.ndarray:
    """Main vertical line plus a diagonal branch joining it partway down.

    The bottom of the frame shows only the main line; the top shows the main
    line and the branch side by side — the geometry the downward camera sees
    when the car approaches a roundabout entry. Default length is well above
    ``LinePolicy.min_branch_rows_fraction`` (0.10 of the ROI ≈ 28 rows here)
    after the first few join-rows merge into the main line.
    """
    image = blank()
    main_x = WIDTH // 2
    # main line, full ROI
    image[int(HEIGHT * 0.10) : int(HEIGHT * 0.68), main_x - 10 : main_x + 10] = 40
    # diagonal branch from the main line at y_branch up to the top-right
    y_branch = int(HEIGHT * 0.40)
    for step in range(branch_rows):
        yy = y_branch - step
        xx = main_x + step * 3
        image[yy, xx - 8 : xx + 8] = 40
    return image


def test_plain_line_is_not_a_junction():
    reading = detect_line(vertical_line(WIDTH // 2))
    assert reading.junction is False
    assert reading.branch_count == 1


def test_wide_line_is_not_a_junction():
    """A fat single strip is not a fork; above 2 cm it is also not the path."""
    reading = detect_line(vertical_line(WIDTH // 2, width=50))
    assert reading.visible
    assert reading.junction is False
    assert reading.branch_count == 1


def test_fork_reports_two_branches():
    reading = detect_line(y_fork())
    assert reading.visible
    assert reading.junction
    assert reading.branch_count >= 2
    assert len(reading.branch_centroids) >= 2
    # main branch is the persistent vertical line at the frame centre
    assert reading.branch_centroids[0] == pytest.approx(WIDTH // 2, abs=20)


def test_too_short_a_branch_is_not_a_junction():
    """A flicker that appears on only a couple of rows is noise, not a fork."""
    reading = detect_line(y_fork(branch_rows=2))
    assert reading.junction is False


def test_junction_summary_labels_it():
    reading = detect_line(y_fork())
    assert reading.summary.startswith("JUNCTION")


def test_thin_tall_shadow_loses_to_track_width():
    """A chair-leg strip spanning the ROI must not beat a 2 cm-scale line."""
    image = blank()
    y0, y1 = int(HEIGHT * 0.10), int(HEIGHT * 0.68)
    image[y0:y1, 40:48] = 40
    image[y0:y1, 300:330] = 40
    reading = detect_line(image)
    assert reading.visible
    assert reading.centroid_x == pytest.approx(315, abs=15)
    assert reading.line_width_px == pytest.approx(30, abs=8)


def test_horizontal_2cm_bar_puts_centroid_on_the_dark_stroke():
    """A forward camera sees a crossing as a horizontal bar; the marker must sit on it."""
    image = blank()
    y = int(HEIGHT * 0.75)
    image[y - 8 : y + 8, 40 : WIDTH - 40] = 40
    reading = detect_line(image)
    assert reading.visible
    assert reading.centroid_x is not None and reading.centroid_y is not None
    assert image[int(reading.centroid_y), int(reading.centroid_x)] < 100
    assert reading.centroid_y == pytest.approx(y, abs=12)
    assert reading.axis == "horizontal"


def test_vertical_path_beats_a_horizontal_crossing():
    """The line along the heading wins over a box-edge / crossing bar."""
    image = blank()
    y = int(HEIGHT * 0.75)
    image[y - 8 : y + 8, 40 : WIDTH - 40] = 40
    image[int(HEIGHT * 0.20) : int(HEIGHT * 0.85), 300:330] = 40
    reading = detect_line(image)
    assert reading.visible
    assert reading.axis == "vertical"
    assert reading.centroid_x == pytest.approx(315, abs=20)


def test_crossing_bar_beats_its_own_right_end():
    """The far curve of a horizontal 2 cm line must not steal the green lock."""
    image = blank()
    y = int(HEIGHT * 0.55)
    image[y - 8 : y + 8, 20 : WIDTH - 20] = 40
    image[y : y + 80, WIDTH - 70 : WIDTH - 34] = 40
    reading = detect_line(image)
    assert reading.visible
    assert reading.axis == "horizontal"
    assert reading.centroid_x == pytest.approx(WIDTH / 2, abs=25)
    assert reading.error_fraction is not None
    assert abs(reading.error_fraction) < 0.12


def test_near_field_2cm_line_beats_a_far_thin_strip():
    """Chair-leg strips higher in the frame must lose to the line by the wheels."""
    image = blank()
    image[int(HEIGHT * 0.10) : int(HEIGHT * 0.50), 40:48] = 40
    image[int(HEIGHT * 0.70) : HEIGHT, 300:330] = 40
    reading = detect_line(image)
    assert reading.visible
    assert reading.centroid_x == pytest.approx(315, abs=15)
    assert reading.centroid_y is not None
    assert reading.centroid_y > HEIGHT * 0.45


def test_edge_only_strip_is_not_the_path():
    """A 2 cm-scale strip at the frame edge is a chair/map border, not the route."""
    image = blank()
    image[int(HEIGHT * 0.65) : int(HEIGHT * 0.90), 8:28] = 40
    reading = detect_line(image)
    assert not reading.visible


def test_line_in_mid_lookahead_is_visible():
    """From the start box the 2 cm stem sits around mid-frame, not at the bumper."""
    image = blank()
    image[int(HEIGHT * 0.46) : int(HEIGHT * 0.58), WIDTH // 2 - 10 : WIDTH // 2 + 10] = 40
    reading = detect_line(image)
    assert reading.visible
    assert reading.centroid_x == pytest.approx(WIDTH / 2, abs=10)


def test_start_box_wide_blob_is_not_the_2cm_path():
    """The start box is ~330 px at 2028; that must not lock as the tracking line."""
    image = blank()
    image[int(HEIGHT * 0.65) : int(HEIGHT * 0.90), 200:360] = 40
    reading = detect_line(image)
    assert not reading.visible


def test_printed_text_blob_loses_to_track_width():
    """A huge dark print block is not the tracking line."""
    image = blank()
    y0, y1 = int(HEIGHT * 0.10), int(HEIGHT * 0.68)
    image[int(HEIGHT * 0.20) : int(HEIGHT * 0.50), 40:420] = 40
    image[y0:y1, 500:530] = 40
    reading = detect_line(image)
    assert reading.visible
    assert reading.centroid_x == pytest.approx(515, abs=15)
    assert reading.junction is False


def test_start_box_left_wall_loses_to_centered_stem():
    """Two 2 cm strokes: start-box left wall plus the outgoing stem. Lock the stem."""
    image = blank()
    y0, y1 = int(HEIGHT * 0.40), int(HEIGHT * 0.95)
    image[y0:y1, 140:176] = 40
    image[y0:y1, WIDTH // 2 - 18 : WIDTH // 2 + 18] = 40
    reading = detect_line(image)
    assert reading.visible
    assert reading.centroid_x == pytest.approx(WIDTH / 2, abs=15)
    assert reading.error_fraction is not None
    assert abs(reading.error_fraction) < 0.08


# ----------------------------------------------------------------------
# 15 mm Task-1 reprint line semantics (line_width_m)
# ----------------------------------------------------------------------


def test_line_policy_defaults_are_anchored_to_15mm():
    policy = LinePolicy()
    assert policy.line_width_m == pytest.approx(0.015)
    assert policy.expected_line_width_fraction == pytest.approx(0.043, abs=1e-3)
    assert policy.min_line_width_fraction == pytest.approx(0.019, abs=1e-3)
    assert policy.max_line_width_fraction == pytest.approx(0.10, abs=1e-3)


def test_line_policy_width_fractions_scale_with_line_width_m():
    wide = LinePolicy(line_width_m=0.020)  # old 2 cm Yahboom paper
    nominal = LinePolicy(line_width_m=0.015)
    ratio = 0.020 / 0.015
    assert wide.expected_line_width_fraction == pytest.approx(
        nominal.expected_line_width_fraction * ratio, rel=1e-6
    )
    assert wide.min_line_width_fraction == pytest.approx(
        nominal.min_line_width_fraction * ratio, rel=1e-6
    )
    assert wide.max_line_width_fraction == pytest.approx(
        nominal.max_line_width_fraction * ratio, rel=1e-6
    )


def test_line_policy_explicit_fractions_are_not_overridden():
    policy = LinePolicy(expected_line_width_fraction=0.10, line_width_m=0.030)
    assert policy.expected_line_width_fraction == pytest.approx(0.10)
    assert policy.line_width_m == pytest.approx(0.030)


def test_line_policy_rejects_non_positive_line_width():
    with pytest.raises(ValueError):
        LinePolicy(line_width_m=0.0)
    with pytest.raises(ValueError):
        LinePolicy(line_width_m=-0.01)

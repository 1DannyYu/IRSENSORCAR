"""Tests for SfM frame-quality assessment, using synthetic images.

Each test builds an image with exactly one defect so the reported `problems`
can be pinned to that defect: seeded noise is richly textured, flat grey is a
feature desert, blurred texture is soft.

The textured fixture is noise rather than a checkerboard because ORB finds only
~36 keypoints per tile on a regular checkerboard — every corner looks alike —
while noise yields ~1000, the same order as the furniture tiles in the real
2028x1520 capture this module was written against.
"""

from __future__ import annotations

import numpy as np
import pytest

from carbot.frame_quality import QualityPolicy, assess, repeatable_keypoints

pytest.importorskip("cv2", reason="frame-quality assessment needs OpenCV")

HEIGHT, WIDTH = 480, 640


def texture(height: int = HEIGHT, width: int = WIDTH, seed: int = 7) -> np.ndarray:
    """Feature-rich mid-tone texture; the range avoids dark and clipped pixels."""
    rng = np.random.default_rng(seed)
    return rng.integers(40, 216, size=(height, width), dtype=np.uint8)


def checkerboard(square: int = 8, height: int = HEIGHT, width: int = WIDTH) -> np.ndarray:
    """Regular high-contrast edges, used where sharpness is what matters."""
    rows = (np.arange(height) // square) % 2
    columns = (np.arange(width) // square) % 2
    return np.where(rows[:, None] ^ columns[None, :], 210, 45).astype(np.uint8)


def flat(value: int = 128, height: int = HEIGHT, width: int = WIDTH) -> np.ndarray:
    return np.full((height, width), value, dtype=np.uint8)


def half_textured() -> np.ndarray:
    """The measured failure: textured on the left, blank wall on the right."""
    image = flat(200)
    image[:, : WIDTH // 2] = texture()[:, : WIDTH // 2]
    return image


# ------------------------------------------------------------------- geometry


def test_reports_dimensions_and_tile_grid():
    q = assess(texture())
    assert (q.width, q.height) == (640, 480)
    assert len(q.tile_keypoints) == 3
    assert all(len(row) == 4 for row in q.tile_keypoints)
    assert q.total_tiles == 12


def test_custom_tile_grid():
    q = assess(texture(), tile_rows=2, tile_columns=2)
    assert q.total_tiles == 4


def test_colour_input_is_converted():
    colour = np.repeat(texture()[:, :, None], 3, axis=2)
    assert assess(colour).keypoints == assess(texture()).keypoints


def test_rejects_unsupported_shape():
    with pytest.raises(ValueError, match="unsupported image shape"):
        assess(np.zeros((10, 10, 2), dtype=np.uint8))


def test_rejects_image_smaller_than_the_grid():
    with pytest.raises(ValueError, match="too small"):
        assess(np.zeros((2, 2), dtype=np.uint8))


def test_rejects_degenerate_grid():
    with pytest.raises(ValueError, match="at least one row and column"):
        assess(texture(), tile_rows=0)


# -------------------------------------------------------------------- texture


def test_rich_texture_is_textured_everywhere_and_usable():
    q = assess(texture())
    assert q.textured_tiles == q.total_tiles
    assert q.problems == ()
    assert q.usable is True


def test_flat_image_is_a_feature_desert():
    q = assess(flat(128))
    assert q.textured_tiles == 0
    assert q.keypoints == 0
    assert q.usable is False
    assert any("feature desert" in p for p in q.problems)


def test_half_blank_wall_is_reported_as_a_desert_not_as_blur():
    """The measured case: sharp where there is furniture, empty where the wall is."""
    q = assess(half_textured())
    left = [row[0] for row in q.tile_keypoints] + [row[1] for row in q.tile_keypoints]
    right = [row[2] for row in q.tile_keypoints] + [row[3] for row in q.tile_keypoints]
    assert min(left) > max(right)
    assert q.textured_tiles == 6
    assert not any("soft" in p for p in q.problems)


def test_min_tile_keypoints_decides_which_tiles_count():
    image = half_textured()
    assert assess(image, QualityPolicy(min_tile_keypoints=1)).textured_tiles == 6
    assert assess(image, QualityPolicy(min_tile_keypoints=100_000)).textured_tiles == 0


def test_keypoints_total_is_the_sum_of_the_tiles():
    q = assess(texture())
    assert q.keypoints == sum(sum(row) for row in q.tile_keypoints)


# ------------------------------------------------------------------- exposure


def test_dark_image_is_reported_as_underexposed_and_crushed():
    q = assess(flat(5))
    assert q.mean_brightness == pytest.approx(5.0)
    assert q.dark_fraction == 1.0
    assert any("underexposed" in p for p in q.problems)
    assert any("crushed shadows" in p for p in q.problems)


def test_blown_image_is_reported_as_clipped():
    q = assess(flat(255))
    assert q.clipped_fraction == 1.0
    assert any("blown highlights" in p for p in q.problems)


def test_bright_textured_image_has_no_exposure_problem():
    q = assess(texture())
    assert not any("underexposed" in p or "clipped" in p for p in q.problems)
    assert q.dark_fraction == 0.0
    assert q.clipped_fraction == 0.0


# ------------------------------------------------------------------ sharpness


def test_blur_lowers_sharpness_without_emptying_every_tile():
    """Blur must read as 'soft', which is what a real out-of-focus lens looks like."""
    cv2 = pytest.importorskip("cv2")
    sharp = assess(checkerboard(square=32))
    soft = assess(cv2.GaussianBlur(checkerboard(square=32), (31, 31), 0))
    assert soft.sharpness < sharp.sharpness
    assert any("soft" in p for p in soft.problems)


def test_tile_sharpness_grid_matches_the_keypoint_grid_shape():
    q = assess(texture())
    assert [len(r) for r in q.tile_sharpness] == [len(r) for r in q.tile_keypoints]


# --------------------------------------------------------------------- report


def test_summary_lists_the_problems_when_unusable():
    text = assess(flat(5)).summary()
    assert "underexposed" in text
    assert "feature desert" in text


def test_summary_is_just_the_numbers_when_usable():
    text = assess(texture()).summary()
    assert "640x480" in text
    assert "|" not in text


# ------------------------------------------------------- repeatable keypoints


def test_identical_frames_match_almost_every_keypoint():
    image = texture()
    assert repeatable_keypoints(image, image) > 500


def test_independent_noise_matches_far_worse_than_the_same_frame():
    """The point of the metric: noise keypoints do not reappear, real ones do."""
    same = repeatable_keypoints(texture(seed=1), texture(seed=1))
    different = repeatable_keypoints(texture(seed=1), texture(seed=2))
    assert different < same / 10


def test_shifted_scene_still_matches():
    """A real sweep moves the camera; matches must survive translation."""
    image = texture()
    shifted = np.roll(image, 12, axis=1)
    assert repeatable_keypoints(image, shifted) > 100


def test_featureless_frames_match_nothing():
    assert repeatable_keypoints(flat(128), flat(128)) == 0


def test_stricter_ratio_keeps_fewer_matches():
    image = texture()
    shifted = np.roll(image, 12, axis=1)
    assert repeatable_keypoints(image, shifted, ratio=0.5) <= repeatable_keypoints(
        image, shifted, ratio=0.9
    )


def test_rejects_out_of_range_ratio():
    with pytest.raises(ValueError, match="ratio must be"):
        repeatable_keypoints(texture(), texture(), ratio=0.0)


def test_colour_frames_are_accepted():
    colour = np.repeat(texture()[:, :, None], 3, axis=2)
    assert repeatable_keypoints(colour, colour) > 500


# --------------------------------------------------------------------- policy


@pytest.mark.parametrize(
    "kwargs",
    [
        {"min_tile_keypoints": -1},
        {"min_textured_tiles": -1},
        {"min_mean_brightness": 300.0},
        {"max_dark_fraction": 1.5},
        {"max_clipped_fraction": -0.1},
        {"min_sharpness": -1.0},
    ],
)
def test_policy_rejects_out_of_range_values(kwargs):
    with pytest.raises(ValueError):
        QualityPolicy(**kwargs)

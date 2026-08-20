"""Tests for metric scale recovery from tags in an up-to-scale reconstruction.

Ground truth is synthesised: camera positions are chosen in metres, then a
COLMAP-like coordinate system is built from them with an arbitrary rotation,
translation and known scale. The estimator must recover that scale, and must be
unaffected by the rotation and translation — which is the whole reason it works
on distances only.
"""

from __future__ import annotations

import numpy as np
import pytest

from carbot.scale import (
    ScaleEstimate,
    estimate_scale,
    scale_positions,
    trajectory_extent_m,
)

TRUE_SCALE = 0.42  # metres per reconstruction unit


def metric_positions(count: int = 6, seed: int = 3) -> dict[str, np.ndarray]:
    """Camera centres in a tag's frame, in metres, spread over a few metres."""
    rng = np.random.default_rng(seed)
    return {f"frame-{i:03d}.jpg": rng.uniform(-2.0, 2.0, size=3) for i in range(count)}


def arbitrary_rotation() -> np.ndarray:
    angle = 0.7
    axis = np.asarray([0.3, 0.8, 0.5])
    axis = axis / np.linalg.norm(axis)
    cross = np.asarray(
        [
            [0.0, -axis[2], axis[1]],
            [axis[2], 0.0, -axis[0]],
            [-axis[1], axis[0], 0.0],
        ]
    )
    return np.eye(3) + np.sin(angle) * cross + (1 - np.cos(angle)) * (cross @ cross)


def reconstruction_from(metric: dict[str, np.ndarray], scale: float = TRUE_SCALE):
    """A COLMAP-like frame: same shape, different units, rotated and shifted."""
    rotation = arbitrary_rotation()
    offset = np.asarray([11.0, -4.0, 7.5])
    return {name: rotation @ (p / scale) + offset for name, p in metric.items()}


def single_tag(metric: dict[str, np.ndarray], tag_id: int = 0):
    return {name: {tag_id: p} for name, p in metric.items()}


# ------------------------------------------------------------------- recovery


def test_recovers_the_true_scale():
    metric = metric_positions()
    estimate = estimate_scale(reconstruction_from(metric), single_tag(metric))
    assert estimate is not None
    assert estimate.metres_per_unit == pytest.approx(TRUE_SCALE, rel=1e-9)
    assert estimate.trustworthy


def test_scale_is_immune_to_the_reconstruction_rotation_and_offset():
    """Distances only, so where COLMAP put its origin cannot matter."""
    metric = metric_positions()
    plain = {name: p / TRUE_SCALE for name, p in metric.items()}
    transformed = reconstruction_from(metric)
    tags = single_tag(metric)
    assert estimate_scale(plain, tags).metres_per_unit == pytest.approx(
        estimate_scale(transformed, tags).metres_per_unit, rel=1e-9
    )


@pytest.mark.parametrize("scale", [0.05, 1.0, 17.5])
def test_recovers_a_range_of_scales(scale):
    metric = metric_positions()
    estimate = estimate_scale(reconstruction_from(metric, scale), single_tag(metric))
    assert estimate is not None
    assert estimate.metres_per_unit == pytest.approx(scale, rel=1e-9)


def test_pair_count_is_every_combination_of_views():
    metric = metric_positions(count=5)
    estimate = estimate_scale(reconstruction_from(metric), single_tag(metric))
    assert estimate.pair_count == 5 * 4 // 2


def test_two_tags_both_contribute():
    metric = metric_positions()
    # A second tag elsewhere in the room: its own frame, so its camera positions
    # differ by a rigid transform — distances between them are unchanged.
    shifted = {name: p + np.asarray([3.0, -1.0, 0.5]) for name, p in metric.items()}
    tags = {name: {0: metric[name], 7: shifted[name]} for name in metric}
    estimate = estimate_scale(reconstruction_from(metric), tags)
    assert estimate.tag_ids == (0, 7)
    assert estimate.metres_per_unit == pytest.approx(TRUE_SCALE, rel=1e-9)


# -------------------------------------------------------------------- filters


def test_short_baselines_are_skipped():
    """Two nearly-coincident views divide two small noisy numbers."""
    metric = {
        "a.jpg": np.asarray([0.0, 0.0, 0.0]),
        "b.jpg": np.asarray([0.001, 0.0, 0.0]),
        "c.jpg": np.asarray([2.0, 0.0, 0.0]),
    }
    estimate = estimate_scale(reconstruction_from(metric), single_tag(metric))
    assert estimate is not None
    # a-b is below the metric floor; a-c and b-c remain.
    assert estimate.pair_count == 2


def test_returns_none_when_no_pair_qualifies():
    metric = {"a.jpg": np.zeros(3), "b.jpg": np.asarray([0.001, 0.0, 0.0])}
    assert estimate_scale(reconstruction_from(metric), single_tag(metric)) is None


def test_returns_none_when_no_image_sees_a_tag():
    metric = metric_positions()
    assert estimate_scale(reconstruction_from(metric), {}) is None


def test_images_without_a_shared_tag_contribute_nothing():
    metric = metric_positions(count=4)
    names = sorted(metric)
    tags = {names[0]: {0: metric[names[0]]}, names[1]: {5: metric[names[1]]}}
    assert estimate_scale(reconstruction_from(metric), tags) is None


def test_rejects_non_positive_baseline_limits():
    metric = metric_positions()
    with pytest.raises(ValueError, match="baseline minimums must be positive"):
        estimate_scale(reconstruction_from(metric), single_tag(metric), min_baseline_units=0)


# --------------------------------------------------------------- trust report


def test_noisy_tag_poses_widen_the_spread_and_lose_trust():
    metric = metric_positions(count=8)
    rng = np.random.default_rng(11)
    noisy = {name: p + rng.normal(0, 0.35, size=3) for name, p in metric.items()}
    estimate = estimate_scale(reconstruction_from(metric), single_tag(noisy))
    assert estimate is not None
    assert estimate.relative_spread > 0.10
    assert not estimate.trustworthy


def test_too_few_pairs_is_not_trustworthy():
    assert not ScaleEstimate(0.42, pair_count=2, tag_ids=(0,), relative_spread=0.0).trustworthy
    assert ScaleEstimate(0.42, pair_count=3, tag_ids=(0,), relative_spread=0.0).trustworthy


def test_describe_flags_an_untrustworthy_estimate():
    text = ScaleEstimate(0.42, pair_count=2, tag_ids=(0,), relative_spread=0.5).describe()
    assert "NOT trustworthy" in text
    assert "0.42" in text


# ------------------------------------------------------------------- helpers


def test_scale_positions_converts_to_metres():
    scaled = scale_positions({"a": np.asarray([1.0, 2.0, 3.0])}, 0.5)
    assert scaled["a"] == pytest.approx([0.5, 1.0, 1.5])


@pytest.mark.parametrize("bad", [0.0, -1.0, float("nan"), float("inf")])
def test_scale_positions_rejects_a_bad_factor(bad):
    with pytest.raises(ValueError):
        scale_positions({"a": np.zeros(3)}, bad)


def test_trajectory_extent():
    positions = {"a": np.asarray([0.0, 0.0, 0.0]), "b": np.asarray([3.0, 1.0, 0.5])}
    assert trajectory_extent_m(positions) == pytest.approx([3.0, 1.0, 0.5])


def test_trajectory_extent_needs_a_position():
    with pytest.raises(ValueError):
        trajectory_extent_m({})

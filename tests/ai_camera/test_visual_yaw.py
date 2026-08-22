"""Tests for measuring the car's turn from two frames.

A known rotation is synthesised by warping a textured image with the exact
homography ``K R K^-1`` for a yaw of theta, so the estimator can be checked
against ground truth without a robot. These tests also pin the sign convention:
a camera turning right sees features move left, and `estimate_yaw_deg` returns a
positive angle for it.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from carbot.visual_yaw import estimate_yaw_deg

cv2 = pytest.importorskip("cv2", reason="yaw estimation needs OpenCV")

WIDTH, HEIGHT = 2028, 1520
FX = 1553.3  # measured for the IMX500 at 2028x1520
CX, CY = 1021.7, 719.7


def texture(seed: int = 11) -> np.ndarray:
    """Feature-rich image; ORB needs real corners to match."""
    rng = np.random.default_rng(seed)
    small = rng.integers(30, 226, size=(HEIGHT // 4, WIDTH // 4), dtype=np.uint8)
    return cv2.resize(small, (WIDTH, HEIGHT), interpolation=cv2.INTER_LINEAR)


def rotated(image: np.ndarray, yaw_deg: float) -> np.ndarray:
    """The same scene viewed by a camera yawed ``yaw_deg`` to the right.

    The rotation is negated because ``warpPerspective`` maps source pixels to
    destination pixels: to synthesise what a camera sees after turning *right*,
    the scene content has to be moved *left*. Getting this backwards is what let
    a sign error in the estimator pass its own test.
    """
    k = np.asarray([[FX, 0.0, CX], [0.0, FX, CY], [0.0, 0.0, 1.0]], dtype=np.float64)
    angle = math.radians(-yaw_deg)
    rotation = np.asarray(
        [
            [math.cos(angle), 0.0, math.sin(angle)],
            [0.0, 1.0, 0.0],
            [-math.sin(angle), 0.0, math.cos(angle)],
        ],
        dtype=np.float64,
    )
    homography = k @ rotation @ np.linalg.inv(k)
    return cv2.warpPerspective(image, homography, (WIDTH, HEIGHT))


@pytest.mark.parametrize("truth", [5.0, 10.0, 20.0, 30.0])
def test_recovers_a_known_right_turn(truth):
    estimate = estimate_yaw_deg(texture(), rotated(texture(), truth), FX, CX)
    assert estimate is not None
    assert estimate.yaw_deg == pytest.approx(truth, abs=0.5)
    assert estimate.trustworthy


@pytest.mark.parametrize("truth", [-5.0, -20.0])
def test_recovers_a_known_left_turn(truth):
    estimate = estimate_yaw_deg(texture(), rotated(texture(), truth), FX, CX)
    assert estimate is not None
    assert estimate.yaw_deg == pytest.approx(truth, abs=0.5)


def test_a_right_turn_is_positive_and_a_left_turn_is_negative():
    """Pins the sign convention the spin-rate script depends on.

    Verified independently of the warp helper: turning right moves scene content
    left, so the brightest column of a synthetic stripe must end up at a smaller
    x — and the estimate must be positive.
    """
    stripe = np.full((HEIGHT, WIDTH), 30, dtype=np.uint8)
    stripe[:, WIDTH // 2 - 20 : WIDTH // 2 + 20] = 220
    turned_right = rotated(stripe, 15.0)
    assert int(turned_right.sum(axis=0).argmax()) < WIDTH // 2

    right = estimate_yaw_deg(texture(), rotated(texture(), 15.0), FX, CX)
    left = estimate_yaw_deg(texture(), rotated(texture(), -15.0), FX, CX)
    assert right is not None and left is not None
    assert right.yaw_deg > 0 > left.yaw_deg


def test_no_rotation_reads_as_zero():
    estimate = estimate_yaw_deg(texture(), texture(), FX, CX)
    assert estimate is not None
    assert estimate.yaw_deg == pytest.approx(0.0, abs=0.1)
    assert estimate.spread_deg < 0.5


def test_spread_is_small_for_a_clean_rotation():
    estimate = estimate_yaw_deg(texture(), rotated(texture(), 20.0), FX, CX)
    assert estimate is not None
    assert estimate.spread_deg < 1.0


def test_unrelated_frames_are_not_trustworthy():
    """Two different scenes must not yield a confident angle."""
    estimate = estimate_yaw_deg(texture(seed=1), texture(seed=2), FX, CX)
    assert estimate is None or not estimate.trustworthy


def test_featureless_frames_return_none():
    flat = np.full((HEIGHT, WIDTH), 128, dtype=np.uint8)
    assert estimate_yaw_deg(flat, flat, FX, CX) is None


def test_colour_frames_are_accepted():
    colour = np.repeat(texture()[:, :, None], 3, axis=2)
    turned = np.repeat(rotated(texture(), 10.0)[:, :, None], 3, axis=2)
    estimate = estimate_yaw_deg(colour, turned, FX, CX)
    assert estimate is not None
    assert estimate.yaw_deg == pytest.approx(10.0, abs=0.5)


def test_rejects_invalid_focal_length():
    with pytest.raises(ValueError, match="focal_length_px must be positive"):
        estimate_yaw_deg(texture(), texture(), 0.0, CX)


def test_rejects_invalid_centre_fraction():
    with pytest.raises(ValueError, match="centre_fraction must be"):
        estimate_yaw_deg(texture(), texture(), FX, CX, centre_fraction=1.5)


def test_trustworthy_needs_both_matches_and_agreement():
    from carbot.visual_yaw import YawEstimate

    assert YawEstimate(20.0, matches=50, spread_deg=1.0).trustworthy
    assert not YawEstimate(20.0, matches=5, spread_deg=1.0).trustworthy
    assert not YawEstimate(20.0, matches=50, spread_deg=9.0).trustworthy

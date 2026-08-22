"""Measure how far the car turned, from two camera frames.

The chassis has no encoders, so turn angles were open-loop: a spin rate of
8.2 s per 360 deg measured once at speed 150 was used to time every turn. That
is fine for random-bounce coverage and useless for a photogrammetry sweep, which
needs small, repeatable steps — a burst of frames about 20 deg apart so
consecutive views overlap. At that size the motor's startup transient is a large
fraction of the whole motion, so the rate has to be measured, not extrapolated.

ADR 0002 rejected measuring the robot by hand, so the camera measures itself. For
a pure yaw rotation, a feature's angle from the optical axis is
``atan((x - cx) / fx)``, and the rotation is the difference of that angle between
two frames. Averaging over matched features gives the turn in degrees without a
protractor or an encoder.

The car rotates about the chassis centre rather than the camera's optical centre,
so a small translation rides along with the rotation. The resulting parallax is
minor when the scene is far compared with that offset (roughly 10 cm of chassis
radius against 1-2 m of room), but it is a real error term and the reason this
module reports the spread of its per-feature estimates alongside the median.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

import numpy as np


@dataclass(frozen=True)
class YawEstimate:
    """A measured rotation, with the evidence needed to judge it."""

    yaw_deg: float
    matches: int
    spread_deg: float

    @property
    def trustworthy(self) -> bool:
        """Enough agreeing matches to believe the median.

        A handful of matches, or a wide spread, means the frames overlapped
        poorly or the match set is mostly noise — report it rather than feeding
        a fabricated angle into a calibration.
        """
        return self.matches >= 20 and self.spread_deg <= 3.0


def _cv2() -> Any:
    try:
        import cv2
    except ImportError as exc:
        raise RuntimeError(
            "OpenCV is required to estimate yaw from frames. On the Pi use the system "
            "python3; on a development machine run with `uv run --extra vision`."
        ) from exc
    return cv2


def estimate_yaw_deg(
    first: np.ndarray,
    second: np.ndarray,
    focal_length_px: float,
    principal_x_px: float,
    centre_fraction: float = 0.6,
    max_vertical_shift_px: float = 40.0,
    features: int = 4000,
) -> YawEstimate | None:
    """Yaw from ``first`` to ``second``, positive when the camera turned right.

    Only features within ``centre_fraction`` of the frame width are used: the
    estimator is exact for a pure rotation anywhere in the frame, but lens
    distortion and parallax both grow toward the edges. Matches that jump
    vertically by more than ``max_vertical_shift_px`` are dropped, because a yaw
    rotation does not move features up or down and such a pair is a mismatch.

    Returns ``None`` when too few matches survive to estimate anything.
    """
    cv2 = _cv2()
    if focal_length_px <= 0:
        raise ValueError("focal_length_px must be positive")
    if not 0.0 < centre_fraction <= 1.0:
        raise ValueError("centre_fraction must be in (0, 1]")

    grey = [f if f.ndim == 2 else cv2.cvtColor(f, cv2.COLOR_BGR2GRAY) for f in (first, second)]
    orb = cv2.ORB_create(nfeatures=features)
    keypoints_a, descriptors_a = orb.detectAndCompute(grey[0], None)
    keypoints_b, descriptors_b = orb.detectAndCompute(grey[1], None)
    if descriptors_a is None or descriptors_b is None:
        return None
    if len(descriptors_a) < 2 or len(descriptors_b) < 2:
        return None

    width = grey[0].shape[1]
    half_window = centre_fraction * width / 2.0
    matcher = cv2.BFMatcher(cv2.NORM_HAMMING)
    angles: list[float] = []
    for candidates in matcher.knnMatch(descriptors_a, descriptors_b, k=2):
        if len(candidates) != 2:
            continue
        best, runner_up = candidates
        if best.distance >= 0.75 * runner_up.distance:
            continue
        ax, ay = keypoints_a[best.queryIdx].pt
        bx, by = keypoints_b[best.trainIdx].pt
        if abs(ax - principal_x_px) > half_window:
            continue
        if abs(ay - by) > max_vertical_shift_px:
            continue
        angles.append(
            math.degrees(
                math.atan((ax - principal_x_px) / focal_length_px)
                - math.atan((bx - principal_x_px) / focal_length_px)
            )
        )

    if len(angles) < 4:
        return None
    values = np.asarray(angles, dtype=np.float64)
    # A camera turning right sees features move left, so x_b < x_a and the angle
    # difference above is already positive. An earlier negation here inverted the
    # whole convention, and the synthetic test that was supposed to catch it
    # warped its fixture the same wrong way, so the two errors agreed.
    median = float(np.median(values))
    # Median absolute deviation, scaled to compare with a standard deviation;
    # a plain std would be dominated by the surviving mismatches.
    spread = float(np.median(np.abs(values - median)) * 1.4826)
    return YawEstimate(yaw_deg=median, matches=len(angles), spread_deg=spread)


def estimate_yaw_between_files(
    first_path: str,
    second_path: str,
    focal_length_px: float,
    principal_x_px: float,
) -> YawEstimate | None:
    """:func:`estimate_yaw_deg` for two images on disk."""
    cv2 = _cv2()
    images = []
    for path in (first_path, second_path):
        image = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
        if image is None:
            raise ValueError(f"could not read image: {path}")
        images.append(image)
    return estimate_yaw_deg(images[0], images[1], focal_length_px, principal_x_px)

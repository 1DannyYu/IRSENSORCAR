"""Recover metric scale for an up-to-scale Structure-from-Motion reconstruction.

COLMAP reconstructs shape, not size: its camera positions and points sit in
arbitrary units, so a room comes out correctly proportioned but could be a
doll's house or a warehouse. ADR 0002 chose to fix that with a target of known
size rather than by measuring the robot — the 70 mm wall AprilTag is already
printed, fixed, and measured.

The method uses distances only. A COLMAP reconstruction is related to reality by
a similarity transform (scale, rotation, translation), and distance ratios are
invariant to the rotation and translation parts. So for two images that both see
the same tag:

    metres_per_unit = |p_i - p_j| / |c_i - c_j|

where ``p`` is the camera centre in the *tag's* frame — metric, because
:func:`carbot.vision.detect_apriltag_poses` solves it against the known tag edge
length — and ``c`` is the camera centre COLMAP reported. Averaging over every
such pair gives a scale with a spread that says whether to believe it.

Only distances are used, so nothing here needs the tags' positions relative to
each other. Two tags on different walls each yield an independent estimate, and
disagreement between them is a useful warning rather than a modelling problem.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from typing import Any

import numpy as np

MIN_PAIRS_FOR_TRUST = 3
MAX_RELATIVE_SPREAD = 0.10


@dataclass(frozen=True)
class ScaleEstimate:
    """A metres-per-COLMAP-unit factor, with the evidence behind it."""

    metres_per_unit: float
    pair_count: int
    tag_ids: tuple[int, ...]
    relative_spread: float

    @property
    def trustworthy(self) -> bool:
        """Enough agreeing pairs to anchor a map on.

        A handful of pairs, or ratios that disagree by more than a tenth, means
        the tag was seen from too few viewpoints or its pose was poorly
        conditioned — report it rather than publishing a confident wrong size.
        """
        return (
            self.pair_count >= MIN_PAIRS_FOR_TRUST and self.relative_spread <= MAX_RELATIVE_SPREAD
        )

    def describe(self) -> str:
        tags = ", ".join(str(t) for t in self.tag_ids)
        return (
            f"{self.metres_per_unit:.4f} m/unit from {self.pair_count} pairs "
            f"(tags {tags}), spread {self.relative_spread:.1%}"
            f"{'' if self.trustworthy else '  — NOT trustworthy'}"
        )


def camera_position_in_tag_frame(
    rotation_vector: np.ndarray, translation_m: np.ndarray
) -> np.ndarray:
    """Where the camera sits in the tag's own frame, in metres.

    ``detect_apriltag_poses`` returns the tag's pose in the camera frame:
    a tag point maps to the camera as ``X_cam = R @ X_tag + t``. The camera
    centre is ``X_cam = 0``, so in tag coordinates it is ``-R.T @ t``.
    """
    from carbot.vision import _cv2

    rotation, _ = _cv2().Rodrigues(np.asarray(rotation_vector, dtype=np.float64))
    return -rotation.T @ np.asarray(translation_m, dtype=np.float64).reshape(3)


def tag_frame_positions(tag_poses: Sequence[Any]) -> dict[int, np.ndarray]:
    """Map tag id -> camera position in that tag's frame, for one image.

    A tag id seen twice in one image is dropped: a duplicated marker cannot be
    told apart, and guessing which detection is the real one would silently
    corrupt every pair it takes part in.
    """
    seen: dict[int, np.ndarray] = {}
    duplicated: set[int] = set()
    for pose in tag_poses:
        if pose.tag_id in seen:
            duplicated.add(pose.tag_id)
            continue
        seen[pose.tag_id] = camera_position_in_tag_frame(pose.rotation_vector, pose.translation_m)
    for tag_id in duplicated:
        seen.pop(tag_id, None)
    return seen


def estimate_scale(
    camera_centres: Mapping[str, np.ndarray],
    tag_positions: Mapping[str, Mapping[int, np.ndarray]],
    min_baseline_units: float | None = None,
    min_baseline_m: float = 0.05,
    baseline_fraction: float = 0.2,
) -> ScaleEstimate | None:
    """Solve metres per reconstruction unit from tags seen in multiple views.

    ``camera_centres`` maps image name to its COLMAP camera centre;
    ``tag_positions`` maps image name to {tag id: camera position in that tag's
    frame, in metres}.

    Pairs whose views sit too close together are skipped, because the ratio then
    divides two small noisy numbers. The threshold defaults to
    ``baseline_fraction`` of the trajectory's own extent rather than an absolute
    number of units: reconstruction units are arbitrary, so a fixed floor is
    meaningless. On the first real model this mattered a great deal — pairs
    separated by under two units returned ratios up to seven times those from
    well-separated pairs, and dragged the median with them.

    Returns ``None`` when no pair qualifies.
    """
    if min_baseline_m <= 0 or baseline_fraction <= 0:
        raise ValueError("baseline minimums must be positive")
    if min_baseline_units is None:
        if not camera_centres:
            return None
        extent = trajectory_extent_m(camera_centres)
        min_baseline_units = baseline_fraction * float(np.linalg.norm(extent))
    if min_baseline_units <= 0:
        raise ValueError("baseline minimums must be positive")

    ratios: list[float] = []
    used_tags: set[int] = set()
    names = sorted(set(camera_centres) & set(tag_positions))
    for index, first in enumerate(names):
        for second in names[index + 1 :]:
            shared = set(tag_positions[first]) & set(tag_positions[second])
            if not shared:
                continue
            units = float(
                np.linalg.norm(
                    np.asarray(camera_centres[first], dtype=np.float64)
                    - np.asarray(camera_centres[second], dtype=np.float64)
                )
            )
            if units < min_baseline_units:
                continue
            for tag_id in shared:
                metres = float(
                    np.linalg.norm(tag_positions[first][tag_id] - tag_positions[second][tag_id])
                )
                if metres < min_baseline_m:
                    continue
                ratios.append(metres / units)
                used_tags.add(tag_id)

    if not ratios:
        return None
    values = np.asarray(ratios, dtype=np.float64)
    median = float(np.median(values))
    # Median absolute deviation rather than a standard deviation: a few
    # badly-conditioned tag poses would otherwise swamp an otherwise clean set.
    spread = float(np.median(np.abs(values - median)) * 1.4826)
    return ScaleEstimate(
        metres_per_unit=median,
        pair_count=len(ratios),
        tag_ids=tuple(sorted(used_tags)),
        relative_spread=spread / median if median > 0 else float("inf"),
    )


def scale_positions(
    positions: Mapping[str, np.ndarray],
    metres_per_unit: float,
) -> dict[str, np.ndarray]:
    """Convert reconstruction positions to metres, keeping the origin and axes."""
    if not np.isfinite(metres_per_unit) or metres_per_unit <= 0:
        raise ValueError("metres_per_unit must be a positive finite number")
    return {
        name: np.asarray(value, dtype=np.float64) * metres_per_unit
        for name, value in positions.items()
    }


def trajectory_extent_m(positions: Mapping[str, np.ndarray]) -> np.ndarray:
    """Bounding-box size of a set of metric positions, as (x, y, z) in metres."""
    if not positions:
        raise ValueError("at least one position is required")
    points = np.asarray(list(positions.values()), dtype=np.float64)
    return points.max(axis=0) - points.min(axis=0)

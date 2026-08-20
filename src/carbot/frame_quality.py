"""Is a captured still usable for Structure-from-Motion?

The first room sweep registered only 9 of 40 frames in COLMAP, and blur was the
leading suspect. Measuring a static capture showed otherwise: a whole-frame
Laplacian variance of 40 looked like blur, but the tiles containing furniture
scored 127-139 with ~1500 ORB keypoints each, while the tiles covering a blank
white wall scored 4-15 with 0-164. Focus was fine. The frame was half feature
desert and underexposed (mean brightness 70/255, 4.9% crushed to black).

Global sharpness therefore conflates three different failures — blur, missing
texture, and bad exposure — and cannot tell the operator which one to fix. This
module reports them separately, per tile, so a sweep can be diagnosed instead
of guessed at:

- **blur** shows as low sharpness in tiles that *do* have keypoints
- **feature desert** shows as tiles with almost no keypoints
- **exposure** shows as low mean brightness, or a large dark/clipped fraction

Keypoint spread is the part that predicts registration: COLMAP needs features
distributed across the frame and shared with its neighbours, not all crowded
into one corner.

OpenCV is a Pi/`--extra vision` dependency, so it is imported lazily.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np

TILE_ROWS = 3
TILE_COLUMNS = 4
ORB_FEATURES_PER_TILE = 2000
DARK_LEVEL = 16  # 8-bit values at or below this carry no recoverable detail
CLIPPED_LEVEL = 250


def _cv2() -> Any:
    try:
        import cv2
    except ImportError as exc:
        raise RuntimeError(
            "OpenCV is required for frame-quality assessment. On the Pi use the system "
            "python3; on a development machine run with `uv run --extra vision`."
        ) from exc
    return cv2


@dataclass(frozen=True)
class QualityPolicy:
    """Thresholds for calling a frame SfM-usable.

    Calibrated against one reconstruction: a 30-frame burst patrol whose frames
    all passed ``min_textured_tiles`` and of which COLMAP registered **30 of 30**
    into some model. What that evidence does and does not establish:

    - ``min_textured_tiles=6`` is not too lenient. The weakest frame that passed
      sat exactly on the threshold — 6 of 12 tiles, 738 keypoints — and still
      registered.
    - ``min_sharpness`` was **falsified** by the only frame that tested it: a
      capture scoring 19 registered normally. It was lowered from 20 so it flags
      gross failure only. Global sharpness has consistently failed to predict
      anything useful here — it cannot tell blur from a blank wall — so do not
      raise it without evidence.
    - Whether any threshold is too *strict* is still unknown, because the
      rejected captures were not kept. Re-run the patrol with
      ``--keep-rejected`` and check whether the rejects would have added usable
      pairs.

    The deeper limit is structural: registration is a property of *pairs*, not
    of single frames. Every frame above registered because its burst gave it
    overlapping neighbours, including the softest and sparsest one in the set.
    No per-frame threshold can predict what only :func:`repeatable_keypoints`
    between two frames can measure.
    """

    min_tile_keypoints: int = 40
    min_textured_tiles: int = 6
    min_mean_brightness: float = 60.0
    max_dark_fraction: float = 0.10
    max_clipped_fraction: float = 0.05
    min_sharpness: float = 10.0

    def __post_init__(self) -> None:
        if self.min_tile_keypoints < 0 or self.min_textured_tiles < 0:
            raise ValueError("keypoint thresholds must be non-negative")
        if not 0.0 <= self.min_mean_brightness <= 255.0:
            raise ValueError("min_mean_brightness must be in [0, 255]")
        for name in ("max_dark_fraction", "max_clipped_fraction"):
            value = getattr(self, name)
            if not 0.0 <= value <= 1.0:
                raise ValueError(f"{name} must be in [0, 1]")
        if self.min_sharpness < 0:
            raise ValueError("min_sharpness must be non-negative")


@dataclass(frozen=True)
class FrameQuality:
    """Per-frame and per-tile measurements, plus the reasons a frame fails."""

    width: int
    height: int
    sharpness: float
    mean_brightness: float
    dark_fraction: float
    clipped_fraction: float
    tile_keypoints: tuple[tuple[int, ...], ...]
    tile_sharpness: tuple[tuple[float, ...], ...]
    policy: QualityPolicy

    @property
    def total_tiles(self) -> int:
        return sum(len(row) for row in self.tile_keypoints)

    @property
    def keypoints(self) -> int:
        """Total keypoints, capped per tile at ``ORB_FEATURES_PER_TILE``."""
        return sum(sum(row) for row in self.tile_keypoints)

    @property
    def textured_tiles(self) -> int:
        """Tiles carrying enough keypoints to contribute to a reconstruction."""
        return sum(
            1
            for row in self.tile_keypoints
            for count in row
            if count >= self.policy.min_tile_keypoints
        )

    @property
    def problems(self) -> tuple[str, ...]:
        """Every threshold this frame misses, named so the fix is obvious."""
        issues: list[str] = []
        if self.textured_tiles < self.policy.min_textured_tiles:
            issues.append(
                f"feature desert: only {self.textured_tiles}/{self.total_tiles} tiles have "
                f">={self.policy.min_tile_keypoints} keypoints"
            )
        if self.mean_brightness < self.policy.min_mean_brightness:
            issues.append(
                f"underexposed: mean brightness {self.mean_brightness:.0f} < "
                f"{self.policy.min_mean_brightness:.0f}"
            )
        if self.dark_fraction > self.policy.max_dark_fraction:
            issues.append(f"crushed shadows: {self.dark_fraction:.1%} of pixels near black")
        if self.clipped_fraction > self.policy.max_clipped_fraction:
            issues.append(f"blown highlights: {self.clipped_fraction:.1%} of pixels clipped")
        if self.sharpness < self.policy.min_sharpness:
            issues.append(f"soft: sharpness {self.sharpness:.0f} < {self.policy.min_sharpness:.0f}")
        return tuple(issues)

    @property
    def usable(self) -> bool:
        """True when the frame misses no threshold. See :class:`QualityPolicy`."""
        return not self.problems

    def summary(self) -> str:
        head = (
            f"{self.width}x{self.height} sharp={self.sharpness:.0f} "
            f"bright={self.mean_brightness:.0f} textured={self.textured_tiles}/"
            f"{self.total_tiles} keypoints={self.keypoints}"
        )
        return head if self.usable else f"{head} | {'; '.join(self.problems)}"


def _grayscale(image: np.ndarray) -> np.ndarray:
    if image.ndim == 2:
        return image
    if image.ndim == 3 and image.shape[2] in (3, 4):
        cv2 = _cv2()
        code = cv2.COLOR_BGR2GRAY if image.shape[2] == 3 else cv2.COLOR_BGRA2GRAY
        return cv2.cvtColor(image, code)
    raise ValueError(f"unsupported image shape {image.shape}")


def assess(
    image: np.ndarray,
    policy: QualityPolicy | None = None,
    tile_rows: int = TILE_ROWS,
    tile_columns: int = TILE_COLUMNS,
) -> FrameQuality:
    """Measure one frame's sharpness, exposure, and keypoint spread."""
    cv2 = _cv2()
    policy = policy or QualityPolicy()
    if tile_rows < 1 or tile_columns < 1:
        raise ValueError("tile grid must have at least one row and column")
    gray = _grayscale(image)
    height, width = gray.shape[:2]
    if height < tile_rows or width < tile_columns:
        raise ValueError(
            f"image {width}x{height} is too small for a {tile_columns}x{tile_rows} grid"
        )

    orb = cv2.ORB_create(nfeatures=ORB_FEATURES_PER_TILE)
    keypoints: list[tuple[int, ...]] = []
    sharpness: list[tuple[float, ...]] = []
    for row in range(tile_rows):
        counts: list[int] = []
        variances: list[float] = []
        for column in range(tile_columns):
            tile = gray[
                row * height // tile_rows : (row + 1) * height // tile_rows,
                column * width // tile_columns : (column + 1) * width // tile_columns,
            ]
            counts.append(len(orb.detect(tile, None)))
            variances.append(float(cv2.Laplacian(tile, cv2.CV_64F).var()))
        keypoints.append(tuple(counts))
        sharpness.append(tuple(variances))

    return FrameQuality(
        width=width,
        height=height,
        sharpness=float(cv2.Laplacian(gray, cv2.CV_64F).var()),
        mean_brightness=float(gray.mean()),
        dark_fraction=float((gray < DARK_LEVEL).mean()),
        clipped_fraction=float((gray > CLIPPED_LEVEL).mean()),
        tile_keypoints=tuple(keypoints),
        tile_sharpness=tuple(sharpness),
        policy=policy,
    )


def assess_file(path: str, policy: QualityPolicy | None = None) -> FrameQuality:
    """Assess an image on disk. Raises ``ValueError`` when it cannot be read."""
    cv2 = _cv2()
    image = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise ValueError(f"could not read image: {path}")
    return assess(image, policy)


def repeatable_keypoints(
    first: np.ndarray,
    second: np.ndarray,
    ratio: float = 0.75,
    features: int = 4000,
) -> int:
    """Count keypoints that match between two views of the same scene.

    A raw keypoint count is not a measure of SfM value, because sensor noise
    creates keypoints too: raising auto-exposure gain from 5.9 to 15.8 increased
    the reported keypoints by 26%, and some of that gain is noise the detector
    cannot find twice. A keypoint only contributes to a reconstruction if it
    reappears in another image, so matching two captures — of a static scene for
    an exposure comparison, or of consecutive positions for a real sweep —
    measures the part COLMAP can actually use.

    Matches are filtered with Lowe's ratio test; a lower ``ratio`` is stricter.
    """
    cv2 = _cv2()
    if not 0.0 < ratio <= 1.0:
        raise ValueError("ratio must be in (0, 1]")
    orb = cv2.ORB_create(nfeatures=features)
    _, first_descriptors = orb.detectAndCompute(_grayscale(first), None)
    _, second_descriptors = orb.detectAndCompute(_grayscale(second), None)
    if first_descriptors is None or second_descriptors is None:
        return 0
    if len(first_descriptors) < 2 or len(second_descriptors) < 2:
        return 0
    matcher = cv2.BFMatcher(cv2.NORM_HAMMING)
    good = 0
    for candidates in matcher.knnMatch(first_descriptors, second_descriptors, k=2):
        if len(candidates) == 2 and candidates[0].distance < ratio * candidates[1].distance:
            good += 1
    return good


def repeatable_keypoints_between_files(
    first_path: str,
    second_path: str,
    ratio: float = 0.75,
) -> int:
    """:func:`repeatable_keypoints` for two images on disk."""
    cv2 = _cv2()
    images = []
    for path in (first_path, second_path):
        image = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
        if image is None:
            raise ValueError(f"could not read image: {path}")
        images.append(image)
    return repeatable_keypoints(images[0], images[1], ratio)

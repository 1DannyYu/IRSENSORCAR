"""Bird's-eye (ground-plane) view of the track in front of the car.

The IMX500 sits low and looks forward, so a 15 mm floor line is a trapezoid
in the camera frame and is easy to confuse with box edges and chair legs. A
homography from four ground correspondences (or a ChArUco board lying on the
paper) warps each frame into a metric top-down patch. Line following then
steers on a stroke that is actually ~15 mm wide in that patch (the Task-1
reprint map's route line; the line's physical width is stored in
``GroundView.line_width_m`` and defaults to 0.015 m).

This module is pure: no camera, no motors, no I2C.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Any

import numpy as np

from carbot.line_follow import LinePolicy, LineReading, _grayscale, _row_segments, _snap_to_dark


def _cv2() -> Any:
    try:
        import cv2
    except ImportError as exc:
        raise RuntimeError(
            "OpenCV is required for the ground view. On the Pi use system python3; "
            "on a development machine run with `uv run --extra vision`."
        ) from exc
    return cv2


@dataclass(frozen=True)
class GroundView:
    """Image-to-bird's-eye homography for one camera pose above the paper.

    Bird's-eye pixels: x increases to the car's right, y increases toward the
    bottom of the image (near the wheels). World metres: x right, y forward
    on the ground, matching the four calibration correspondences.
    """

    homography: np.ndarray
    bev_width: int
    bev_height: int
    metres_per_pixel: float
    x_min_m: float
    x_max_m: float
    y_min_m: float
    y_max_m: float
    image_points_px: tuple[tuple[float, float], ...]
    world_points_m: tuple[tuple[float, float], ...]
    # Physical width of the track line in metres (15 mm on the Task-1 reprint
    # map). Width filters are derived from it so a recalibration for a
    # different map does not silently keep 2 cm-era thresholds.
    line_width_m: float = 0.015
    # Fraction of the BEV width excluded on each side when scanning for the
    # line. The 2026-08-17 departure-area captures had a persistent dark
    # shadow/gradient band along the raw frame's right edge; warped to BEV it
    # became a full-height dark strip whose narrow edge read as a 15 mm
    # "line" at u≈0.84·width, and the detector's near-field preference locked
    # it instead of the real centred stroke. The real track never sits at the
    # very edge of the calibrated window (the window is wider than the car),
    # so excluding the outer 22% on each side removes that whole class of
    # edge artefacts (the departure-area shadow band's narrow edge sits at
    # u≈0.80·width, still inside an 18% exclusion).
    edge_exclude_fraction: float = 0.22
    # World-metre box (x_lo, x_hi, y_lo, y_hi) of a physical object that must
    # never be mistaken for the track line — the calibration target itself,
    # when it was used to fit this homography (`auto_calibrate_ground_view`).
    # None for calibrations not tied to a keep-out object (e.g. a ChArUco
    # board treated as the floor itself, or the synthetic views in tests).
    exclude_world_box_m: tuple[float, float, float, float] | None = None

    def __post_init__(self) -> None:
        if self.homography.shape != (3, 3):
            raise ValueError("homography must be 3x3")
        if self.bev_width < 32 or self.bev_height < 32:
            raise ValueError("bird's-eye size must be at least 32x32")
        if self.metres_per_pixel <= 0:
            raise ValueError("metres_per_pixel must be positive")
        if self.line_width_m <= 0:
            raise ValueError("line_width_m must be positive")
        if not 0.0 <= self.edge_exclude_fraction < 0.5:
            raise ValueError("edge_exclude_fraction must be in [0, 0.5)")
        if not self.x_min_m < self.x_max_m:
            raise ValueError("x_min_m must be < x_max_m")
        if not self.y_min_m < self.y_max_m:
            raise ValueError("y_min_m must be < y_max_m")

    @property
    def expected_line_width_px(self) -> float:
        return self.line_width_m / self.metres_per_pixel

    def world_to_bev(self, x_m: float, y_m: float) -> tuple[float, float]:
        u = (x_m - self.x_min_m) / self.metres_per_pixel
        v = (self.y_max_m - y_m) / self.metres_per_pixel
        return u, v

    def bev_to_world(self, u: float, v: float) -> tuple[float, float]:
        x_m = self.x_min_m + u * self.metres_per_pixel
        y_m = self.y_max_m - v * self.metres_per_pixel
        return x_m, y_m

    def warp(self, image: np.ndarray) -> np.ndarray:
        cv2 = _cv2()
        # BEV pixels that map outside the captured frame (the near edge of
        # the calibrated window can extrapolate past the bottom of the raw
        # image) default to black padding, which reads as well below
        # `dark_threshold` — indistinguishable from real ink. A false "wide
        # dark span" from padding at the bottom of the window was read as a
        # T cross-bar in the 2026-08-16 capture. Pad with paper-white (255)
        # instead so out-of-frame regions read as background, never as line.
        return cv2.warpPerspective(
            image,
            self.homography,
            (self.bev_width, self.bev_height),
            borderValue=255,
        )

    def bev_to_image(self, u: float, v: float) -> tuple[float, float]:
        inv = np.linalg.inv(self.homography)
        point = inv @ np.array([u, v, 1.0], dtype=np.float64)
        if abs(point[2]) < 1e-9:
            raise ValueError("bird's-eye point is at infinity in the camera frame")
        return float(point[0] / point[2]), float(point[1] / point[2])

    def to_json(self) -> dict[str, Any]:
        return {
            "homography": self.homography.tolist(),
            "bev_width": self.bev_width,
            "bev_height": self.bev_height,
            "metres_per_pixel": self.metres_per_pixel,
            "x_min_m": self.x_min_m,
            "x_max_m": self.x_max_m,
            "y_min_m": self.y_min_m,
            "y_max_m": self.y_max_m,
            "line_width_m": self.line_width_m,
            "edge_exclude_fraction": self.edge_exclude_fraction,
            "image_points_px": [list(p) for p in self.image_points_px],
            "world_points_m": [list(p) for p in self.world_points_m],
            "exclude_world_box_m": (
                list(self.exclude_world_box_m) if self.exclude_world_box_m else None
            ),
        }


def calibrate_ground_view(
    image_points_px: np.ndarray,
    world_points_m: np.ndarray,
    *,
    x_min_m: float = -0.30,
    x_max_m: float = 0.30,
    y_min_m: float = 0.12,
    y_max_m: float = 0.72,
    metres_per_pixel: float = 0.002,
    line_width_m: float = 0.015,
) -> GroundView:
    """Fit a ground homography from four or more image/world pairs.

    ``image_points_px`` are pixel coordinates in the camera frame.
    ``world_points_m`` are metres on the floor, x right, y forward.
    """
    cv2 = _cv2()
    image_points = np.asarray(image_points_px, dtype=np.float64).reshape(-1, 2)
    world_points = np.asarray(world_points_m, dtype=np.float64).reshape(-1, 2)
    if image_points.shape[0] < 4 or image_points.shape != world_points.shape:
        raise ValueError("need at least four matching image and world points")
    bev_width = max(round((x_max_m - x_min_m) / metres_per_pixel), 32)
    bev_height = max(round((y_max_m - y_min_m) / metres_per_pixel), 32)
    bev_points = np.array(
        [
            [
                (x - x_min_m) / metres_per_pixel,
                (y_max_m - y) / metres_per_pixel,
            ]
            for x, y in world_points
        ],
        dtype=np.float64,
    )
    homography, _ = cv2.findHomography(image_points, bev_points, method=0)
    if homography is None:
        raise ValueError("could not fit a ground homography from the given points")
    return GroundView(
        homography=np.asarray(homography, dtype=np.float64),
        bev_width=bev_width,
        bev_height=bev_height,
        metres_per_pixel=metres_per_pixel,
        x_min_m=x_min_m,
        x_max_m=x_max_m,
        y_min_m=y_min_m,
        y_max_m=y_max_m,
        line_width_m=line_width_m,
        image_points_px=tuple((float(x), float(y)) for x, y in image_points),
        world_points_m=tuple((float(x), float(y)) for x, y in world_points),
    )


def ground_view_from_charuco(
    image: np.ndarray,
    *,
    x_min_m: float = -0.30,
    x_max_m: float = 0.30,
    y_min_m: float = 0.12,
    y_max_m: float = 0.72,
    metres_per_pixel: float = 0.002,
    line_width_m: float = 0.015,
) -> GroundView:
    """Build a ground view from a ChArUco board lying flat on the paper.

    The board plane is treated as the floor. Place the board in front of the
    car with its +Y side farther away (the same orientation used for camera
    calibration). Outer board corners become the four world correspondences.
    """
    from carbot.vision import (
        calibration_for_image,
        detect_charuco_board_pose,
        load_calibration,
        load_charuco_board_geometry,
    )

    cv2 = _cv2()
    calib_path = (
        Path(__file__).resolve().parents[2]
        / "assets/reference/camera-calibration/2026-08-14-imx500-4056x3040/calibration.json"
    )
    calibration = load_calibration(calib_path)
    geometry = load_charuco_board_geometry(calib_path)
    pose = detect_charuco_board_pose(image, calibration, geometry)
    if pose is None:
        raise ValueError("no ChArUco board on the floor in this frame")
    width_m = geometry.squares_x * geometry.square_pitch_x_m
    length_m = geometry.squares_y * geometry.square_pitch_y_m
    object_corners = np.array(
        [[0.0, 0.0, 0.0], [width_m, 0.0, 0.0], [width_m, length_m, 0.0], [0.0, length_m, 0.0]],
        dtype=np.float64,
    )
    adjusted = calibration_for_image(calibration, image)
    projected, _ = cv2.projectPoints(
        object_corners,
        pose.rotation_vector,
        pose.translation_m,
        adjusted.camera_matrix,
        adjusted.distortion_coefficients,
    )
    image_points = projected.reshape(-1, 2)
    # Board +Y is farther from the car; shift so the board sits in the BEV
    # ahead of the origin rather than straddling x=0 only.
    world_points = np.array(
        [
            [-width_m / 2, y_min_m + 0.02],
            [width_m / 2, y_min_m + 0.02],
            [width_m / 2, y_min_m + 0.02 + length_m],
            [-width_m / 2, y_min_m + 0.02 + length_m],
        ],
        dtype=np.float64,
    )
    return calibrate_ground_view(
        image_points,
        world_points,
        x_min_m=x_min_m,
        x_max_m=x_max_m,
        y_min_m=y_min_m,
        y_max_m=y_max_m,
        metres_per_pixel=metres_per_pixel,
        line_width_m=line_width_m,
    )


def _order_corners(points: np.ndarray) -> list[tuple[float, float]]:
    """Order four unordered 2D points as TL, TR, BR, BL (image axes: x right,
    y down). Standard sum/difference trick — robust as long as the
    quadrilateral is not rotated close to 45 degrees, true for a target
    photographed close to fronto-parallel."""
    pts = np.asarray(points, dtype=np.float64)
    s = pts[:, 0] + pts[:, 1]
    d = pts[:, 0] - pts[:, 1]
    tl = pts[int(np.argmin(s))]
    br = pts[int(np.argmax(s))]
    tr = pts[int(np.argmax(d))]
    bl = pts[int(np.argmin(d))]
    return [tuple(tl), tuple(tr), tuple(br), tuple(bl)]


def find_target_corners(
    image: np.ndarray,
    dark_threshold: int = 100,
    min_area_px: float = 2000.0,
) -> tuple[tuple[float, float], ...] | None:
    """Locate the printed calibration target's border and return its four
    corners as TL, TR, BR, BL image-pixel coordinates, or ``None`` if no
    plausible target is in frame.

    The target (`scripts/generate_ground_view_target.py`) is a thick (5mm)
    rectangular outline — a black stroke enclosing a white interior. Its dark
    mask therefore contains two nested contours: the stroke's outer edge and
    its inner edge (the hole). The 2026-08-16 session found by hand that
    averaging the two, corner for corner, lands closer to the printed
    rectangle's true centreline than either edge alone (the design width in
    `generate_ground_view_target.py` is the centreline) — this automates that
    same technique instead of `scripts/pick_ground_view_corners.py`'s manual
    clicks, so a bumped camera mount can be recalibrated before every run
    instead of requiring a person to re-click corners each time.
    """
    cv2 = _cv2()
    gray = _grayscale(image)
    mask = (gray < dark_threshold).astype(np.uint8) * 255
    contours, _ = cv2.findContours(mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None

    quads: list[tuple[float, np.ndarray]] = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < min_area_px:
            continue
        peri = cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)
        if len(approx) != 4:
            continue
        quads.append((area, approx.reshape(-1, 2).astype(np.float64)))
    if not quads:
        return None

    quads.sort(key=lambda q: -q[0])
    outer_area, outer = quads[0]
    outer_span = float(np.hypot(*(outer.max(axis=0) - outer.min(axis=0))))
    inner = None
    for area, pts in quads[1:]:
        if area >= outer_area:
            continue
        centroid_gap = float(np.hypot(*(pts.mean(axis=0) - outer.mean(axis=0))))
        if centroid_gap < 0.15 * outer_span:
            inner = pts
            break

    ordered_outer = _order_corners(outer)
    if inner is None:
        return tuple(ordered_outer)
    ordered_inner = _order_corners(inner)
    return tuple(
        ((ox + ix) / 2.0, (oy + iy) / 2.0)
        for (ox, oy), (ix, iy) in zip(ordered_outer, ordered_inner)
    )


def auto_calibrate_ground_view(
    image: np.ndarray,
    *,
    target_width_m: float,
    target_height_m: float,
    near_m: float,
    dark_threshold: int = 100,
    x_min_m: float = -0.30,
    x_max_m: float = 0.30,
    y_min_m: float = -0.10,
    y_max_m: float = 0.90,
    metres_per_pixel: float = 0.002,
    line_width_m: float = 0.015,
) -> GroundView:
    """Recalibrate from the printed target instead of a stored homography.

    A toy chassis' camera mount is not rigid the way a commercial rig's is —
    it shifts every time the car is picked up or bumped, so a saved
    `ground-view.json` from a previous session is a guess about the current
    pose, not a fact. This is meant to run once at the start of every drive
    session (see `examples/28_auto_calibrate.py`) with the target taped
    somewhere the car sees from its starting pose, so the homography is
    always fresh rather than assumed to still be valid.
    """
    corners = find_target_corners(image, dark_threshold)
    if corners is None:
        raise ValueError(
            "calibration target not found in frame — check it is taped in view, "
            "in focus, and not occluded"
        )
    far_m = near_m + target_height_m
    world_points = np.array(
        [
            [-target_width_m / 2, far_m],
            [target_width_m / 2, far_m],
            [target_width_m / 2, near_m],
            [-target_width_m / 2, near_m],
        ],
        dtype=np.float64,
    )
    view = calibrate_ground_view(
        np.array(corners, dtype=np.float64),
        world_points,
        x_min_m=x_min_m,
        x_max_m=x_max_m,
        y_min_m=y_min_m,
        y_max_m=y_max_m,
        metres_per_pixel=metres_per_pixel,
        line_width_m=line_width_m,
    )
    # Mark the target's own footprint as a keep-out zone (padded for the
    # crosshair arms and tick marks that extend past its measured corners,
    # and for the target's true position drifting slightly as the car moves
    # away from the calibration pose) so line detection never mistakes the
    # target's own ink for the track line.
    margin_m = 0.05
    exclude_box = (
        -target_width_m / 2 - margin_m,
        target_width_m / 2 + margin_m,
        near_m - margin_m,
        far_m + margin_m,
    )
    return replace(view, exclude_world_box_m=exclude_box)


def save_ground_view(path: str | Path, view: GroundView) -> None:
    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(json.dumps(view.to_json(), indent=2) + "\n", encoding="utf-8")


def load_optional_ground_view(path: str | Path | None = None) -> GroundView | None:
    """Load a ground view if the file exists; otherwise return None."""
    candidates: list[Path] = []
    if path is not None:
        candidates.append(Path(path))
    candidates.append(Path("/tmp/line-follow/ground-view.json"))
    candidates.append(
        Path(__file__).resolve().parents[2] / "assets/reference/ground-view/imx500-2028x1520.json"
    )
    for candidate in candidates:
        if candidate.is_file():
            return load_ground_view(candidate)
    return None


def load_ground_view(path: str | Path) -> GroundView:
    source = Path(path)
    data = json.loads(source.read_text(encoding="utf-8"))
    try:
        return GroundView(
            homography=np.asarray(data["homography"], dtype=np.float64),
            bev_width=int(data["bev_width"]),
            bev_height=int(data["bev_height"]),
            metres_per_pixel=float(data["metres_per_pixel"]),
            x_min_m=float(data["x_min_m"]),
            x_max_m=float(data["x_max_m"]),
            y_min_m=float(data["y_min_m"]),
            y_max_m=float(data["y_max_m"]),
            # Older ground-view files predate line_width_m and encoded the
            # 20 mm Yahboom-era line; keep their semantics on load so a stale
            # file does not silently switch width filters under new code.
            line_width_m=float(data.get("line_width_m", 0.02)),
            edge_exclude_fraction=float(data.get("edge_exclude_fraction", 0.22)),
            image_points_px=tuple((float(x), float(y)) for x, y in data["image_points_px"]),
            world_points_m=tuple((float(x), float(y)) for x, y in data["world_points_m"]),
            exclude_world_box_m=(
                tuple(float(v) for v in data["exclude_world_box_m"])
                if data.get("exclude_world_box_m")
                else None
            ),
        )
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(f"invalid ground-view file: {source}") from exc


_NO_EXCLUSION = (0.0, -1.0, 0.0, -1.0)  # u_lo > u_hi: never matches _in_box


def _target_exclusion_bev_box(view: GroundView) -> tuple[float, float, float, float]:
    """BEV-pixel bounding box (u_lo, u_hi, v_lo, v_hi) covering the physical
    object ``view.exclude_world_box_m`` marks as a keep-out zone (the printed
    calibration target, when it was used to fit this homography — see
    `auto_calibrate_ground_view`), or a box that never matches if there is
    none.

    A calibration target left in view (per the current 2026-08-16 track
    setup) is itself a small dark, ~line-width-scale-adjacent mark on the
    floor — exactly the kind of thing the line detector is built to lock
    onto. We know precisely where it sits in the same world coordinates the
    homography was fit from, so exclude it outright instead of hoping
    width/position heuristics reject it.
    """
    if view.exclude_world_box_m is None:
        return _NO_EXCLUSION
    x_lo, x_hi, y_lo, y_hi = view.exclude_world_box_m
    us: list[float] = []
    vs: list[float] = []
    for x, y in ((x_lo, y_lo), (x_hi, y_lo), (x_hi, y_hi), (x_lo, y_hi)):
        u, v = view.world_to_bev(x, y)
        us.append(u)
        vs.append(v)
    return min(us), max(us), min(vs), max(vs)


def _in_box(x: float, y: float, box: tuple[float, float, float, float]) -> bool:
    x_lo, x_hi, y_lo, y_hi = box
    return x_lo <= x <= x_hi and y_lo <= y <= y_hi


def _detect_cross_bar(
    bev: np.ndarray,
    view: GroundView,
    policy: LinePolicy,
    *,
    u_lo: int,
    u_hi: int,
    y_lo: int,
    y_hi: int,
    min_pixels: int,
    gap: float,
    expected_x: float,
    exclude: tuple[float, float, float, float],
    image: np.ndarray,
) -> LineReading | None:
    """T / crossing detection, used as a *fallback* when no along-heading
    stroke is visible.

    A cross-bar is a dark span much wider than the track line itself (it runs
    *across* the car's heading, not away from it). The width band of the
    vertical scan deliberately excludes spans this wide, since a wide segment
    is not "the path". The outer-loop curve that fooled the vertical scan
    (2026-08-16) never triggers this: row by row it is still only
    ~line-width wide (`width` stayed 9-13px even at its most persistent),
    just offset in x from row to row as it curves — a real cross-bar is wide
    in a single row. Require it to persist across several rows so a single
    noisy row cannot trigger a turn.

    The whole window is scanned, not just the near band: a T cross-bar is
    first seen far ahead (e.g. 34-42 cm from the departure zone on the Task-1
    map, verified 2026-08-17). :class:`carbot.line_nav.LineNav` decides
    *when* to spin using ``t_min_roi_y_fraction`` / ``t_bar_min_width_px``,
    so an early, far cross-bar keeps the car driving straight ("far
    crossing: keep straight to T") instead of turning early.
    """
    cross_min_width = 0.08 / view.metres_per_pixel  # >= 8 cm, well past any stroke
    cross_rows: list[tuple[float, float, float, float]] = []  # centroid, y, x0, x1
    for y in range(y_lo, y_hi, 2):
        segs = _row_segments(bev[y], policy.dark_threshold, min_pixels, gap)
        wide = [
            s
            for s in segs
            if s[1] >= cross_min_width
            and u_lo <= s[0] <= u_hi
            and not _in_box(s[0], y, exclude)
        ]
        if wide:
            centroid, _seg_w, x0, x1 = max(wide, key=lambda s: s[1])
            cross_rows.append((centroid, float(y), float(x0), float(x1)))
    if len(cross_rows) < 4:
        return None
    cu = float(np.median([r[0] for r in cross_rows]))
    cv = float(np.percentile([r[1] for r in cross_rows], 75))
    cu, cv = _snap_to_dark(bev, cu, cv, policy.dark_threshold)
    x0 = float(np.median([r[2] for r in cross_rows]))
    x1 = float(np.median([r[3] for r in cross_rows]))
    cross_image_x, cross_image_y = view.bev_to_image(cu, cv)
    # Estimate the image-space width from the local scale at the bar's own
    # (trusted, near-field) position, not by projecting its far edges through
    # the inverse homography — those can sit well outside the calibrated patch
    # and the projection blows up there (a 200px BEV span projected to 7794
    # image px in the 2026-08-16 capture).
    step_x, step_y = view.bev_to_image(cu + 1.0, cv)
    px_per_bev_px = float(np.hypot(step_x - cross_image_x, step_y - cross_image_y))
    cross_width_px = px_per_bev_px * (x1 - x0)
    if not np.isfinite(cross_width_px) or cross_width_px <= 0:
        cross_width_px = 0.0
    cross_width_px = min(cross_width_px, 4.0 * image.shape[1])
    cross_error_px = cross_image_x - image.shape[1] / 2
    cross_error_fraction = (cu - expected_x) / (bev.shape[1] / 2)
    return LineReading(
        visible=True,
        error_px=cross_error_px,
        error_fraction=float(np.clip(cross_error_fraction, -1.0, 1.0)),
        centroid_x=cross_image_x,
        centroid_y=cross_image_y,
        line_width_px=cross_width_px,
        dark_fraction=float((bev < policy.dark_threshold).mean()),
        tracked_rows=len(cross_rows),
        roi=(0, image.shape[0], 0, image.shape[1]),
        axis="horizontal",
        candidate_centroids=(cu,),
        ground_u_px=cu,
    )


def detect_line_on_ground(
    image: np.ndarray,
    view: GroundView,
    policy: LinePolicy | None = None,
    prefer_u: float | None = None,
) -> LineReading:
    """Find the track line in the bird's-eye patch and report camera-frame error.

    ``prefer_u`` is the BEV x (``LineReading.ground_u_px``) the caller was
    tracking last frame. A junction feature (the outer-loop curve, a T
    cross-bar) can become just as near and just as line-width-wide as the
    real path by the time the car is close to a junction, so nearness and
    width alone cannot tell them apart there (verified 2026-08-16: the
    outer-loop curve read `width=9-10px` at `rows=27-33`,
    indistinguishable from the real line on those two signals alone).
    Continuity is the remaining signal: whichever feature the car was
    already driving on a moment ago is still it, unless that feature
    genuinely disappears. Only then does selection fall back to the cluster
    nearest BEV centre (first acquisition, or re-acquiring after a real
    loss).
    """
    policy = policy or LinePolicy()
    gray = _grayscale(image)
    bev = view.warp(gray)
    height, width = bev.shape[:2]
    expected_w = view.expected_line_width_px
    # Width band around the track line's physical width: accept 65%..250% of
    # it (10..37 mm for the 15 mm Task-1 line at 2 mm/px). The band is
    # derived from ``view.line_width_m`` so a recalibration for another line
    # width propagates everywhere.
    min_width = 0.65 * view.line_width_m / view.metres_per_pixel
    max_width = 2.5 * view.line_width_m / view.metres_per_pixel
    y_lo = int(height * 0.25)
    y_hi = height
    min_pixels = max(int(min_width * 0.5), 4)
    gap = 0.04 * width
    expected_x = width / 2
    exclude = _target_exclusion_bev_box(view)
    # Exclude the outer edge bands of the BEV window: a persistent shadow /
    # gradient band on the raw frame edge warps into a full-height dark strip
    # here whose narrow edge reads as a line-width stroke (2026-08-17). The
    # real track never sits within the outermost fraction of the window.
    u_lo = int(view.edge_exclude_fraction * width)
    u_hi = width - u_lo

    # Vertical (along-heading) line scan. This is the primary signal: it
    # carries the steering error. A cross-bar (T junction) ahead is reported
    # only as a fallback when no usable along-heading stroke exists, so a
    # long straight segment is steered on the line itself even while a far
    # cross-bar is in view (verified 2026-08-17: the departure-zone run saw
    # the T cross-bar 34-42 cm ahead, and reporting it as `horizontal`
    # unconditionally would have starved the steering correction).
    hits: list[tuple[float, float, float]] = []
    for y in range(y_lo, y_hi, 2):
        segs = _row_segments(bev[y], policy.dark_threshold, min_pixels, gap)
        for centroid, seg_w, _x0, _x1 in segs:
            if (
                min_width <= seg_w <= max_width
                and u_lo <= centroid <= u_hi
                and not _in_box(centroid, y, exclude)
            ):
                hits.append((centroid, float(y), float(seg_w)))
    # Minimum number of scan rows that must see a line-width stroke before
    # the frame counts as "line visible". 6 was the 2 cm-era threshold; on
    # the Task-1 map a single shadow/text edge in the BEV window routinely
    # produced 6-9 spurious hits (2026-08-17 departure-area captures), while
    # the real 15 mm stroke in view spans 11+ rows. Raising the floor kills
    # that whole class of false locks at the cost of ignoring a genuinely
    # brief partial view.
    if len(hits) < 10:
        cross = _detect_cross_bar(
            bev,
            view,
            policy,
            u_lo=u_lo,
            u_hi=u_hi,
            y_lo=y_lo,
            y_hi=y_hi,
            min_pixels=min_pixels,
            gap=gap,
            expected_x=expected_x,
            exclude=exclude,
            image=image,
        )
        if cross is not None:
            return cross
        return LineReading(
            visible=False,
            error_px=None,
            error_fraction=None,
            centroid_x=None,
            centroid_y=None,
            line_width_px=0.0,
            dark_fraction=float((bev < policy.dark_threshold).mean()),
            tracked_rows=len(hits),
            roi=(0, image.shape[0], 0, image.shape[1]),
            axis="vertical",
        )

    hits: list[tuple[float, float, float]] = []
    for y in range(y_lo, y_hi, 2):
        segs = _row_segments(bev[y], policy.dark_threshold, min_pixels, gap)
        for centroid, seg_w, _x0, _x1 in segs:
            if (
                min_width <= seg_w <= max_width
                and u_lo <= centroid <= u_hi
                and not _in_box(centroid, y, exclude)
            ):
                hits.append((centroid, float(y), float(seg_w)))
    # Minimum number of scan rows that must see a line-width stroke before
    # the frame counts as "line visible". 6 was the 2 cm-era threshold; on
    # the Task-1 map a single shadow/text edge in the BEV window routinely
    # produced 6-9 spurious hits (2026-08-17 departure-area captures), while
    # the real 15 mm stroke in view spans 11+ rows. Raising the floor kills
    # that whole class of false locks at the cost of ignoring a genuinely
    # brief partial view.
    if len(hits) < 10:
        return LineReading(
            visible=False,
            error_px=None,
            error_fraction=None,
            centroid_x=None,
            centroid_y=None,
            line_width_px=0.0,
            dark_fraction=float((bev < policy.dark_threshold).mean()),
            tracked_rows=len(hits),
            roi=(0, image.shape[0], 0, image.shape[1]),
            axis="vertical",
        )
    # Prefer the near field (largest v, closest to the wheels) over anything
    # farther out. The whole point of the ground-view homography is that the
    # near-field reading is unambiguous — a stray line-width-scale feature further
    # out (the outer-loop curve, a junction cross-bar, the calibration
    # target's own crosshair) must never outvote it just because it happens
    # to sit closer to the BEV centre line in x. Only fall back to the full
    # scanned band when the near field alone does not carry enough hits.
    near_y_lo = y_lo + int((y_hi - y_lo) * 0.6)
    near_hits = [h for h in hits if h[1] >= near_y_lo]
    near_nominal = [h for h in near_hits if abs(h[2] - expected_w) / expected_w <= 0.6]
    if len(near_nominal) >= 4:
        pool = near_nominal
    elif len(near_hits) >= 4:
        pool = near_hits
    else:
        near_nominal_all = [h for h in hits if abs(h[2] - expected_w) / expected_w <= 0.6]
        pool = near_nominal_all or hits
    # Stay on the line the car was already tracking, rather than re-deciding
    # from scratch by BEV-centre proximity every frame. Prefer near-field
    # sticky hits first if available so a far-field curve cannot steal lock.
    members: list[tuple[float, float, float]] | None = None
    if prefer_u is not None:
        sticky_near = [h for h in near_hits if abs(h[0] - prefer_u) <= 0.08 * width]
        if len(sticky_near) >= 3:
            members = sticky_near
        else:
            sticky = [h for h in pool if abs(h[0] - prefer_u) <= 0.08 * width]
            if len(sticky) >= 4:
                members = sticky
    if members is None:
        # Prefer the cluster nearest heading (BEV centre).
        xs = np.array([h[0] for h in pool])
        chosen_x = float(np.median(xs[np.abs(xs - expected_x).argsort()[: max(len(pool) // 3, 6)]]))
        members = [h for h in pool if abs(h[0] - chosen_x) <= 0.06 * width]
        if len(members) < 4:
            members = pool
    u = float(np.median([h[0] for h in members]))
    v = float(np.percentile([h[1] for h in members], 75))
    line_width = float(np.median([h[2] for h in members]))
    u, v = _snap_to_dark(bev, u, v, policy.dark_threshold)
    image_x, image_y = view.bev_to_image(u, v)
    error_px = image_x - image.shape[1] / 2
    # Steer in bird's-eye metres, not raw perspective pixels.
    error_fraction = (u - expected_x) / (width / 2)
    return LineReading(
        visible=True,
        error_px=error_px,
        error_fraction=float(np.clip(error_fraction, -1.0, 1.0)),
        centroid_x=image_x,
        centroid_y=image_y,
        line_width_px=line_width,
        dark_fraction=float((bev < policy.dark_threshold).mean()),
        tracked_rows=len(members),
        roi=(0, image.shape[0], 0, image.shape[1]),
        axis="vertical",
        candidate_centroids=tuple(dict.fromkeys(round(h[0], 1) for h in pool[:8])),
        ground_u_px=u,
    )

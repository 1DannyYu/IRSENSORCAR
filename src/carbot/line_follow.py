"""Downward-looking line detection for black-line tracking on the track map.

The camera is mounted pointing down at the floor, so the frame shows the
track map directly under the car plus a fixed band of chassis/shadow at the
bottom edge. A verified still (2026-08-15, 4056x3040, IMX500 AI Camera):

- track paper background peaks around gray 208
- the black line is ~2.3 % of the pixels and sits ~230 px wide
- any threshold in 90..120 separates line from paper (``dark_threshold``
  defaults to 100)
- the chassis/shadow band starts near y=2081 (68 % of the frame height) and
  the map edge shadows occupy the top ~8 %; the ROI excludes both by default
- the main line is chosen by width (about ``line_width_m``, 15 mm on the
  Task-1 reprint map), not by which dark strip spans the most ROI rows —
  chair legs and map-edge shadows otherwise win

The default ``line_width_m`` is 0.015 m because the current track is the
Task-1 reprint map (``scripts/generate_task1_map.py``), whose black route
line is 15 mm wide (the old 20 mm Yahboom paper is no longer the target).

The line is not assumed to be straight. Each scan row's dark-pixel centroid is
computed and the row centroids are combined with a median, which is robust to
branches, junctions, and the roundabout the car must trace. When too few rows
see a line the reading reports ``visible=False`` and ``error_px=None`` so the
controller can switch to a "search for the line" behaviour instead of steering
on garbage.

OpenCV is a Pi/`--extra vision` dependency, so it is imported lazily. This
module is pure: no camera, no motors, no I2C.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np


@dataclass(frozen=True)
class LinePolicy:
    """Thresholds for treating dark pixels as the tracking line.

    Defaults come from the verified 2026-08-15 downward still described in the
    module docstring. The ROI is expressed in fractions of the frame so the same
    policy works at any capture resolution; ``roi_top`` and ``roi_bottom`` cut
    the map-edge shadows and the chassis band, which would otherwise read as
    huge dark regions and swamp the line centroid.

    ``dark_threshold`` is the gray value below which a pixel counts as "line".
    It is deliberately not aggressive: paper peaks at ~208, the line is well
    below 90 in the verified frame, so 100 leaves headroom for exposure changes
    without pulling in map shadows or dirt.

    ``line_width_m`` is the physical width of the track line in metres — the
    15 mm route line of the Task-1 reprint map. The pixel-width fractions
    below are anchored to it at the 2028-px preview width (a 15 mm stroke
    near the bumper reads ~86 px vs ~115 px for the old 20 mm line); pass a
    different value for a different track. Width *fractions* (not raw pixels)
    keep the same policy valid at any capture resolution.
    """

    dark_threshold: int = 100
    line_width_m: float = 0.015
    roi_top: float = 0.10
    # Forward-looking mount: the bottom of the frame is the track in front of
    # the wheels, not a chassis band. Cutting at 0.68 (the old downward-camera
    # default) dropped the 15 mm line and left the green overlay cross on paper.
    roi_bottom: float = 1.0
    min_row_dark_fraction: float = 0.002
    min_tracked_rows: int = 6
    min_line_fraction: float = 0.001
    # Junction (crossing) detection. A branch is a second dark line that
    # persists across at least ``min_branch_rows_fraction`` of the ROI height
    # while the main line is also present, with its centroid separated from the
    # main line by more than ``branch_gap_fraction`` of the frame width. The
    # verified 2026-08-15 map frame carries scattered dark print/shadows that
    # each last under ~1 % of the ROI (10-12 rows of 1763), while a real
    # crossing line spans ~13 % of it, so the fraction doubles as the noise
    # floor; a wide line alone is not a fork. Increase this to 0.10 to require
    # branches to span 10% (88 rows on 1763-row ROI), rejecting transient marks.
    branch_gap_fraction: float = 0.04
    min_branch_rows_fraction: float = 0.10
    # Main-line selection. Persistence (row count) alone prefers chair legs
    # and map-edge shadows that span the ROI as thin dark strips (~11 px on
    # the 2026-08-15 start-zone preview). The real tracking line is
    # ``line_width_m`` (~86 px for 15 mm at 2028, ~0.043 of frame width).
    # Width bounds reject both the thin strips and the huge dark blocks of
    # printed text; among survivors the score prefers a width close to the
    # expected value. All three are ``None`` by default and derived from
    # ``line_width_m`` in ``__post_init__`` (a 15 mm line at the 2028-px
    # preview: min 0.019, max 0.10, expected 0.043 of frame width); pass a
    # concrete value to override.
    min_line_width_fraction: float | None = None
    # 0.10 allowed the start box (~330 px) to count as the path and trigger a
    # false right turn. The 15 mm stroke stays ~86-115 px at 2028.
    max_line_width_fraction: float | None = None
    expected_line_width_fraction: float | None = None
    # Look-ahead band: the 15 mm stroke the camera actually sees ahead of the
    # car (often a *horizontal* bar under a forward mount). Mean-x/mean-y of a
    # vertically tracked blob lands on paper; the target must be a dark pixel
    # on this band.
    lookahead_top: float = 0.40
    lookahead_bottom: float = 0.98
    # Drop a 15 mm-scale strip farther than this from frame centre (fraction
    # of width). The 15 s start-zone run locked x≈50 / x≈1840 (chairs / map
    # edge) after the real line left the band; those sit ~0.47 of width off
    # centre.
    max_center_offset_fraction: float = 0.40
    min_horizontal_length_fraction: float = 0.15
    # Horizontal crossing-bar thickness band (fraction of frame height),
    # also derived from ``line_width_m`` when left None (a 15 mm bar reads
    # ~9..91 px of 1520: 0.006..0.06).
    min_bar_thickness_fraction: float | None = None
    max_bar_thickness_fraction: float | None = None

    def __post_init__(self) -> None:
        if not 0 <= self.dark_threshold <= 255:
            raise ValueError("dark_threshold must be in [0, 255]")
        if self.line_width_m <= 0:
            raise ValueError("line_width_m must be positive")
        # Width fractions are anchored to the nominal 15 mm Task-1 line at
        # the 2028x1520 preview (86 px near the bumper); any other physical
        # line width scales them linearly. Frozen dataclass, so assign via
        # object.__setattr__.
        scale = self.line_width_m / 0.015
        defaults = {
            "min_line_width_fraction": 0.019 * scale,
            "max_line_width_fraction": 0.10 * scale,
            "expected_line_width_fraction": 0.043 * scale,
            "min_bar_thickness_fraction": 0.006 * scale,
            "max_bar_thickness_fraction": 0.06 * scale,
        }
        for name, value in defaults.items():
            if getattr(self, name) is None:
                object.__setattr__(self, name, value)
        if not 0.0 <= self.roi_top < self.roi_bottom <= 1.0:
            raise ValueError("roi must satisfy 0 <= roi_top < roi_bottom <= 1")
        if not 0.0 <= self.min_row_dark_fraction <= 1.0:
            raise ValueError("min_row_dark_fraction must be in [0, 1]")
        if self.min_tracked_rows < 1:
            raise ValueError("min_tracked_rows must be positive")
        if not 0.0 <= self.min_line_fraction <= 1.0:
            raise ValueError("min_line_fraction must be in [0, 1]")
        if not 0.0 <= self.branch_gap_fraction <= 1.0:
            raise ValueError("branch_gap_fraction must be in [0, 1]")
        if not 0.0 <= self.min_branch_rows_fraction <= 1.0:
            raise ValueError("min_branch_rows_fraction must be in [0, 1]")
        if not 0.0 <= self.min_line_width_fraction < self.max_line_width_fraction <= 1.0:
            raise ValueError("line-width fractions must satisfy 0 <= min < max <= 1")
        if not 0.0 < self.expected_line_width_fraction < 1.0:
            raise ValueError("expected_line_width_fraction must be in (0, 1)")
        if not 0.0 <= self.lookahead_top < self.lookahead_bottom <= 1.0:
            raise ValueError("lookahead must satisfy 0 <= top < bottom <= 1")
        if not 0.0 < self.max_center_offset_fraction <= 1.0:
            raise ValueError("max_center_offset_fraction must be in (0, 1]")
        if not 0.0 < self.min_horizontal_length_fraction <= 1.0:
            raise ValueError("min_horizontal_length_fraction must be in (0, 1]")
        if not 0.0 < self.min_bar_thickness_fraction < self.max_bar_thickness_fraction <= 1.0:
            raise ValueError("bar-thickness fractions must satisfy 0 < min < max <= 1")


@dataclass(frozen=True)
class LineReading:
    """What one frame says about the line, in the controller's terms.

    ``error_px`` is the line centroid minus the frame centre in pixels:
    positive means the line is to the right of the car's heading, negative to
    the left. ``error_fraction`` normalises it to [-1, 1]. Both are ``None``
    when the line is not visible. ``centroid_x`` stays meaningful even when the
    reading is not visible, so a search behaviour can steer toward the last
    seen line. ``centroid_y`` is the mean scan-row of that same line, so the
    overlay can put the green cross on the line pixels instead of the ROI
    midpoint.

    ``branch_count``/``branch_centroids``/``junction`` describe crossing
    geometry. At a T-junction or crossroads the line widens and the per-row
    dark pixels split into two stable clusters; ``junction`` is True when two
    or more clusters persist across enough rows. The controller uses that flag
    to stop steering on a blended centroid and instead choose a branch by
    rule — see `carbot.line_nav`.
    """

    visible: bool
    error_px: float | None
    error_fraction: float | None
    centroid_x: float | None
    line_width_px: float
    dark_fraction: float
    tracked_rows: int
    roi: tuple[int, int, int, int]
    centroid_y: float | None = None
    branch_count: int = 0
    branch_centroids: tuple[float, ...] = ()
    junction: bool = False
    candidate_centroids: tuple[float, ...] = ()
    # "vertical" = path along heading (steer on x). "horizontal" = line across
    # the view; driving straight would climb the stroke, so nav spins to align.
    axis: str = "vertical"
    # BEV x (bird's-eye pixels) of the chosen cluster, ground-view mode only.
    # Feed back as `detect_line(..., prefer_u=...)` next frame so the
    # detector stays on the line it was already tracking instead of
    # re-deciding by BEV-centre proximity every frame — see
    # `detect_line_on_ground`. None outside ground-view mode.
    ground_u_px: float | None = None

    @property
    def summary(self) -> str:
        if not self.visible:
            return "no line"
        state = (
            f"line err={self.error_px:+.0f}px ({self.error_fraction:+.2f}) "
            f"x={self.centroid_x:.0f} width={self.line_width_px:.0f}px "
            f"rows={self.tracked_rows}"
        )
        if self.junction:
            return f"JUNCTION branches={self.branch_count} {state}"
        return state


def _cv2() -> Any:
    try:
        import cv2
    except ImportError as exc:
        raise RuntimeError(
            "OpenCV is required for line detection. On the Pi use the system "
            "python3; on a development machine run with `uv run --extra vision`."
        ) from exc
    return cv2


def _grayscale(image: np.ndarray) -> np.ndarray:
    if image.ndim == 2:
        return image
    if image.ndim == 3 and image.shape[2] in (3, 4):
        cv2 = _cv2()
        code = cv2.COLOR_BGR2GRAY if image.shape[2] == 3 else cv2.COLOR_BGRA2GRAY
        return cv2.cvtColor(image, code)
    raise ValueError(f"unsupported image shape {image.shape}")


def _track_segments(
    dark: np.ndarray, min_row_pixels: int, gap: float
) -> tuple[list[float], list[int], list[float], list[int], list[float], int]:
    """Follow dark segments row by row into continuous lines.

    Each scan row's dark pixels are split into segments. Every segment is
    attached to the line from the previous rows whose running centroid is
    closest, provided the distance is within ``gap``; otherwise it starts a new
    line. Returns ``(segment_centroids, segment_widths, line_centroids,
    line_rows, line_mean_y, rows_with_segments)`` where each ``line_*`` list
    is one entry per tracked line (mean x, member count, mean row in the ROI)
    and ``rows_with_segments`` is the number of scan rows that saw at least
    one segment.

    This row-to-row tracking is what keeps a curved main line together: its
    centroid drifts a few pixels per row, well inside ``gap``, whereas a global
    clustering of all segment centroids splits it whenever the drift exceeds
    the gap. Scattered map print and shadows die after a few rows.
    """
    segment_centroids: list[float] = []
    segment_widths: list[int] = []
    lines: list[list[float]] = []
    line_ys: list[list[int]] = []
    rows_with_segments = 0

    for y, row in enumerate(dark):
        indices = np.flatnonzero(row)
        if indices.size < min_row_pixels:
            continue
        rows_with_segments += 1
        breaks = np.flatnonzero(np.diff(indices) > gap)
        for segment in np.split(indices, breaks + 1):
            if segment.size < min_row_pixels:
                continue
            centroid = float(segment.mean())
            segment_centroids.append(centroid)
            segment_widths.append(int(segment.size))
            best_i, best_dist = None, gap
            for i, line in enumerate(lines):
                distance = abs(line[-1] - centroid)
                if distance <= best_dist:
                    best_i, best_dist = i, distance
            if best_i is None:
                lines.append([centroid])
                line_ys.append([y])
            else:
                lines[best_i].append(centroid)
                line_ys[best_i].append(y)

    line_centroids = [float(np.mean(line)) for line in lines]
    line_rows = [len(line) for line in lines]
    line_mean_y = [float(np.mean(ys)) for ys in line_ys]
    return (
        segment_centroids,
        segment_widths,
        line_centroids,
        line_rows,
        line_mean_y,
        rows_with_segments,
    )


def _row_segments(
    row: np.ndarray, threshold: int, min_pixels: int, gap: float
) -> list[tuple[float, int, int, int]]:
    """Dark segments on one scan row as (centroid, width, x0, x1)."""
    indices = np.flatnonzero(row < threshold)
    if indices.size < min_pixels:
        return []
    breaks = np.flatnonzero(np.diff(indices) > gap)
    out: list[tuple[float, int, int, int]] = []
    for segment in np.split(indices, breaks + 1):
        if segment.size < min_pixels:
            continue
        out.append((float(segment.mean()), int(segment.size), int(segment[0]), int(segment[-1])))
    return out


def _bar_thickness(gray: np.ndarray, y: int, x0: int, x1: int, threshold: int) -> int:
    """Consecutive rows around ``y`` that stay mostly dark on ``[x0, x1]``."""
    height = gray.shape[0]
    span_width = max(x1 - x0, 1)
    up = down = 0
    for dy in range(1, 120):
        yy = y - dy
        if yy < 0 or (gray[yy, x0:x1] < threshold).sum() < 0.5 * span_width:
            break
        up += 1
    for dy in range(1, 120):
        yy = y + dy
        if yy >= height or (gray[yy, x0:x1] < threshold).sum() < 0.5 * span_width:
            break
        down += 1
    return up + down + 1


def _snap_to_dark(gray: np.ndarray, x: float, y: float, threshold: int) -> tuple[float, float]:
    """Move ``(x, y)`` onto the nearest dark pixel within a small window."""
    height, width = gray.shape
    xi, yi = round(x), round(y)
    xi = min(max(xi, 0), width - 1)
    yi = min(max(yi, 0), height - 1)
    if gray[yi, xi] < threshold:
        return float(xi), float(yi)
    for radius in range(1, 80):
        y0, y1 = max(0, yi - radius), min(height, yi + radius + 1)
        x0, x1 = max(0, xi - radius), min(width, xi + radius + 1)
        patch = gray[y0:y1, x0:x1]
        dark = np.argwhere(patch < threshold)
        if dark.size == 0:
            continue
        # nearest to the original point
        local = dark.astype(float)
        local[:, 0] += y0 - y
        local[:, 1] += x0 - x
        nearest = dark[np.argmin(local[:, 0] ** 2 + local[:, 1] ** 2)]
        return float(x0 + nearest[1]), float(y0 + nearest[0])
    return float(xi), float(yi)


def _lookahead_target(
    gray: np.ndarray, policy: LinePolicy
) -> tuple[float, float, float, tuple[float, ...], str, tuple[float, ...]] | None:
    """Pick a track-width stroke in the look-ahead band; the point is on the stroke.

    A forward-looking camera often sees both a track-width *vertical* path
    (along the heading) and a *horizontal* bar (box edge or crossing). The
    car must lock the vertical path when it exists; a horizontal bar is
    only used when no vertical track-width stroke is in the band, and then nav
    spins to align rather than driving straight onto the bar.
    """
    height, width = gray.shape
    y_lo = int(height * policy.lookahead_top)
    y_hi = int(height * policy.lookahead_bottom)
    if y_hi <= y_lo:
        return None
    min_pixels = max(int(policy.min_row_dark_fraction * width), 8)
    gap = policy.branch_gap_fraction * width
    min_width = policy.min_line_width_fraction * width
    max_width = policy.max_line_width_fraction * width
    min_horiz = policy.min_horizontal_length_fraction * width
    min_thick = policy.min_bar_thickness_fraction * height
    max_thick = policy.max_bar_thickness_fraction * height
    expected_x = width / 2
    expected_w = policy.expected_line_width_fraction * width
    max_off = policy.max_center_offset_fraction * width
    cluster_gap = 0.06 * width

    best_horiz: tuple[float, float, float, float] | None = None
    hits_nominal: list[tuple[float, float, float]] = []
    hits_any: list[tuple[float, float, float]] = []
    vert_xs: list[float] = []

    for y in range(y_lo, y_hi, 2):
        segs = _row_segments(gray[y], policy.dark_threshold, min_pixels, gap)
        if not segs:
            continue
        vert = [
            s for s in segs if min_width <= s[1] <= max_width and abs(s[0] - expected_x) <= max_off
        ]
        horiz = [s for s in segs if s[1] >= min_horiz]
        for s in vert:
            hit = (s[0], float(y), float(s[1]))
            hits_any.append(hit)
            vert_xs.append(s[0])
            if abs(s[1] - expected_w) / expected_w <= 0.55:
                hits_nominal.append(hit)
        if horiz:
            for picked in horiz:
                thickness = _bar_thickness(gray, y, picked[2], picked[3] + 1, policy.dark_threshold)
                if not (min_thick <= thickness <= max_thick):
                    continue
                x0, x1 = picked[2], picked[3]
                covering = x0 <= expected_x <= x1
                target_x = expected_x if covering else picked[0]
                closeness = 1.0 - min(abs(target_x - expected_x) / (width / 2), 1.0)
                score = (2.0 if covering else 0.0) + closeness + 0.05 * (y / height)
                cand = (score, target_x, float(y), float(thickness))
                if best_horiz is None or cand[0] > best_horiz[0]:
                    best_horiz = cand

    best_vert = _cluster_vertical_hits(hits_nominal or hits_any, cluster_gap, expected_x)
    junction_xs: tuple[float, ...] = ()
    near_vert = best_vert is not None and abs(best_vert[1] - expected_x) <= 0.22 * width
    center_horiz = best_horiz is not None and abs(best_horiz[1] - expected_x) <= 0.12 * width
    # The right end of a crossing bar looks like a track-width vertical blob. Only
    # a stroke near the red centre line is the path; otherwise lock the bar.
    if near_vert:
        groups = _x_clusters(hits_nominal or hits_any, cluster_gap)
        if len(groups) >= 2:
            junction_xs = tuple(float(np.median([h[0] for h in g])) for g in groups[:4])
        _, x, y, line_width = best_vert
        x, y = _snap_to_dark(gray, x, y, policy.dark_threshold)
        unique_xs = tuple(dict.fromkeys(round(v, 1) for v in vert_xs))
        return x, y, line_width, unique_xs[:8], "vertical", junction_xs
    if center_horiz or best_horiz is not None:
        _, x, y, line_width = best_horiz
        x, y = _snap_to_dark(gray, x, y, policy.dark_threshold)
        return x, y, line_width, (x,), "horizontal", ()
    if best_vert is not None:
        _, x, y, line_width = best_vert
        x, y = _snap_to_dark(gray, x, y, policy.dark_threshold)
        unique_xs = tuple(dict.fromkeys(round(v, 1) for v in vert_xs))
        return x, y, line_width, unique_xs[:8], "vertical", junction_xs
    return None


def _x_clusters(
    hits: list[tuple[float, float, float]], gap: float
) -> list[list[tuple[float, float, float]]]:
    """Group look-ahead hits whose x stays within ``gap`` of the previous hit."""
    if not hits:
        return []
    ordered = sorted(hits, key=lambda h: h[0])
    groups: list[list[tuple[float, float, float]]] = [[ordered[0]]]
    for hit in ordered[1:]:
        if abs(hit[0] - groups[-1][-1][0]) <= gap:
            groups[-1].append(hit)
        else:
            groups.append([hit])
    return groups


def _cluster_vertical_hits(
    hits: list[tuple[float, float, float]],
    gap: float,
    expected_x: float,
) -> tuple[float, float, float, float] | None:
    """Pick the track-width stroke nearest frame centre across the whole look-ahead.

    One scan row used to win by a slightly closer left box-wall; the outgoing
    stem then sat unused as a cyan candidate. Voting by x-cluster, then taking
    the cluster closest to centre, locks the path the car is sitting on.
    """
    groups = _x_clusters(hits, gap)
    if not groups:
        return None
    min_hits = 6
    durable = [g for g in groups if len(g) >= min_hits]
    pool = durable or groups
    chosen = min(pool, key=lambda g: abs(float(np.median([h[0] for h in g])) - expected_x))
    xs = [h[0] for h in chosen]
    ys = [h[1] for h in chosen]
    ws = [h[2] for h in chosen]
    x = float(np.median(xs))
    # Steer on a point nearer the wheels than the far end of the same stroke.
    y = float(np.percentile(ys, 75))
    w = float(np.median(ws))
    closeness = 1.0 - min(abs(x - expected_x) / max(expected_x, 1.0), 1.0)
    return closeness, x, y, w


def detect_line(
    image: np.ndarray,
    policy: LinePolicy | None = None,
    ground_view: object | None = None,
    prefer_u: float | None = None,
) -> LineReading:
    """Locate the dark tracking line in one downward-looking frame.

    Pipeline: grayscale -> ROI crop -> median blur -> threshold dark pixels ->
    per-row segments -> row-to-row tracking into lines -> main line centroid.
    When a second line persists alongside the main one the reading reports
    ``junction`` so the controller can choose a branch instead of steering at
    a blended centroid. Returns :class:`LineReading`; see its docstring.

    When ``ground_view`` is a calibrated bird's-eye homography the line is
    measured on the ground plane instead of in the raw perspective frame.
    ``prefer_u`` (ground-view mode only) is the previous frame's
    ``LineReading.ground_u_px``; see `detect_line_on_ground`.
    """
    if ground_view is not None:
        from carbot.ground_view import GroundView, detect_line_on_ground

        if not isinstance(ground_view, GroundView):
            raise TypeError("ground_view must be a GroundView")
        return detect_line_on_ground(image, ground_view, policy, prefer_u=prefer_u)

    policy = policy or LinePolicy()
    gray = _grayscale(image)
    height, width = gray.shape[:2]

    y_top = int(height * policy.roi_top)
    y_bottom = int(height * policy.roi_bottom)
    if y_bottom <= y_top:
        raise ValueError("ROI collapses to zero height; check roi_top/roi_bottom")
    roi = gray[y_top:y_bottom, :]

    blur = _cv2().medianBlur(roi, 5)
    dark = blur < policy.dark_threshold

    dark_fraction = float(dark.mean())
    min_row_pixels = policy.min_row_dark_fraction * width
    branch_gap = policy.branch_gap_fraction * width

    (
        segment_centroids,
        segment_widths,
        line_centroids,
        line_rows,
        line_mean_y,
        rows_with_segments,
    ) = _track_segments(dark, min_row_pixels, branch_gap)
    branch_min_rows = policy.min_branch_rows_fraction * (y_bottom - y_top)
    min_width = policy.min_line_width_fraction * width
    max_width = policy.max_line_width_fraction * width
    expected_width = policy.expected_line_width_fraction * width
    roi_height = max(y_bottom - y_top, 1)
    main_y = None
    if line_centroids:
        widths = [
            _line_width(segment_centroids, segment_widths, centroid, branch_gap)
            for centroid in line_centroids
        ]
        in_band = [
            i
            for i, line_width in enumerate(widths)
            if min_width <= line_width <= max_width and line_rows[i] >= policy.min_tracked_rows
        ]
        # Do not fall back to over-wide blobs (printed text, chassis). If
        # nothing is track-width-scale, report no line.
        if in_band:

            def _score(i: int) -> float:
                width_score = line_rows[i] / (
                    1.0 + abs(widths[i] - expected_width) / expected_width
                )
                # Prefer the line nearer the bottom of the frame — that is
                # what sits in front of the wheels on this forward mount.
                y_frac = line_mean_y[i] / roi_height
                return width_score * (0.25 + 0.75 * y_frac)

            main_i = max(in_band, key=_score)
            main_centroid = line_centroids[main_i]
            main_rows = line_rows[main_i]
            main_y = y_top + line_mean_y[main_i]
            branches = tuple(
                line_centroids[i]
                for i in in_band
                if i != main_i and line_rows[i] >= branch_min_rows
            )
        else:
            main_centroid, main_rows, branches = None, 0, ()
    else:
        main_centroid, main_rows, branches = None, 0, ()

    candidates = tuple(
        line_centroids[i]
        for i in sorted(range(len(line_centroids)), key=lambda i: line_rows[i], reverse=True)
    )

    look = _lookahead_target(gray, policy)
    look_width = 0.0
    look_junction: tuple[float, ...] = ()
    axis = "vertical"
    if look is not None:
        look_x, look_y, look_width, look_cands, axis, look_junction = look
        main_centroid = look_x
        main_y = look_y
        main_rows = max(main_rows, policy.min_tracked_rows)
        if look_cands:
            candidates = look_cands + tuple(
                c for c in candidates if all(abs(c - k) > 8 for k in look_cands)
            )
        nearby_fork = tuple(b for b in branches if branch_gap < abs(b - look_x) <= 0.28 * width)
        if look_junction:
            branches = look_junction
        elif nearby_fork:
            branches = nearby_fork[:3]

    visible = look is not None

    if not visible:
        centroid = float(np.mean(segment_centroids)) if segment_centroids else None
        return LineReading(
            visible=False,
            error_px=None,
            error_fraction=None,
            centroid_x=centroid,
            centroid_y=None,
            line_width_px=float(np.mean(segment_widths)) if segment_widths else 0.0,
            dark_fraction=dark_fraction,
            tracked_rows=rows_with_segments,
            roi=(y_top, y_bottom, 0, width),
            candidate_centroids=candidates,
        )

    junction = (
        (len(look_junction) >= 2) if look is not None and look_junction else (len(branches) >= 1)
    )
    if look is not None:
        main_width = look_width
    else:
        main_width = _line_width(segment_centroids, segment_widths, main_centroid, branch_gap)
    error_px = main_centroid - width / 2
    return LineReading(
        visible=True,
        error_px=error_px,
        error_fraction=error_px / (width / 2),
        centroid_x=main_centroid,
        centroid_y=main_y,
        line_width_px=main_width,
        dark_fraction=dark_fraction,
        tracked_rows=rows_with_segments,
        roi=(y_top, y_bottom, 0, width),
        branch_count=1 + len(branches),
        branch_centroids=(main_centroid,) + branches,
        junction=junction,
        candidate_centroids=candidates,
        axis=axis,
    )


def _line_width(
    segment_centroids: list[float], segment_widths: list[int], centroid: float, gap: float
) -> float:
    """Median width of segments belonging to the line at ``centroid``."""
    widths = [w for w, c in zip(segment_widths, segment_centroids) if abs(c - centroid) <= gap]
    return float(np.median(widths)) if widths else 0.0


def detect_line_file(path: str, policy: LinePolicy | None = None) -> LineReading:
    """Run :func:`detect_line` on an image on disk. Raises ``ValueError`` when
    the file cannot be read."""
    cv2 = _cv2()
    image = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise ValueError(f"could not read image: {path}")
    return detect_line(image, policy)

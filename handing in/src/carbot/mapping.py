"""2D occupancy-grid mapping from polar ultrasonic scans.

Core pieces reused by the M3 autonomous-exploration loop (and the M2 prototype
verification):

- ``load_polar_scan`` / ``polar_to_points`` — convert a spin-scan CSV (or raw
  (angle, distance) rows) into a 2D point cloud in the sensor frame.
- ``icp`` / ``register`` — 2D point-to-point ICP with an optional initial pose
  guess (rotation + translation). Pure numpy; small clouds (tens of points) run
  in well under a millisecond.
- ``detect_gaps`` — door / wall-gap features: contiguous angle runs whose range
  is much longer than the frame median. Gaps are strong, symmetry-breaking
  anchors for scan matching in rectangular rooms.
- ``OccupancyGrid`` — fixed-resolution 10 cm grid fused with a simple
  inverse-sensor model (beam path = free, endpoint = occupied).

Known limitation (verified on this build): pure multi-angle ICP is severely
multi-modal for *non-adjacent* scans of a rectangular room (585 initial guesses
-> 351 solutions). Use ``register`` with an initial guess from odometry and keep
movement incremental (<50 cm / <30 deg), and/or use ``detect_gaps`` as an
anchor. See docs/progress/2026-08-14-sensors-and-ai-camera.md.
"""

from __future__ import annotations

import csv
import math
from pathlib import Path

import numpy as np

from carbot.frames import scan_angle_rad

DEFAULT_MAX_RANGE_CM = 400.0  # HC-SR04 spec limit; longer readings = "open/far"
DEFAULT_CELL_CM = 10.0


# --------------------------------------------------------------------------
# Scan loading and conversion
# --------------------------------------------------------------------------


def load_polar_scan(path: str | Path, max_range_cm: float = DEFAULT_MAX_RANGE_CM) -> np.ndarray:
    """Read a spin-scan CSV (elapsed_s, distance_cm, spin360_s) into (angle, dist) rows.

    Returns an Nx2 array of [angle_rad, distance_cm]; readings above
    ``max_range_cm`` are clamped to it (they mean "far/open", e.g. a corridor).
    """
    rows: list[tuple[float, float]] = []
    with open(path) as f:
        for r in csv.DictReader(f):
            elapsed = float(r["elapsed_s"])
            dist = float(r["distance_cm"])
            spin360 = float(r["spin360_s"])
            angle = scan_angle_rad(elapsed, spin360)
            rows.append((angle, min(dist, max_range_cm)))
    return np.asarray(rows, dtype=np.float64)


def polar_to_points(scan: np.ndarray) -> np.ndarray:
    """Nx2 [angle, dist] -> Nx2 [x, y] in the sensor frame.

    The sensor frame convention is **angle=0 -> +y (sensor forward)**:
    ``x = d*sin(angle)``, ``y = d*cos(angle)``. This matches the HC-SR04
    spin-scan, where the sensor's forward axis is the odometry heading axis.
    """
    ang = scan[:, 0]
    dist = scan[:, 1]
    return np.column_stack([dist * np.sin(ang), dist * np.cos(ang)])


def points_to_scan(points: np.ndarray) -> np.ndarray:
    """Inverse of :func:`polar_to_points` (used in tests)."""
    x, y = points[:, 0], points[:, 1]
    return np.column_stack([np.arctan2(x, y), np.hypot(x, y)])


# --------------------------------------------------------------------------
# 2D point-to-point ICP
# --------------------------------------------------------------------------


def best_rigid(src: np.ndarray, tgt: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Least-squares 2D rigid transform (R, t) mapping ``src`` onto ``tgt``."""
    cs, ct = src.mean(0), tgt.mean(0)
    h = (src - cs).T @ (tgt - ct)
    u, _, vt = np.linalg.svd(h)
    r = vt.T @ u.T
    if np.linalg.det(r) < 0:  # avoid reflection
        vt[-1] *= -1
        r = vt.T @ u.T
    return r, ct - r @ cs


def icp(
    src: np.ndarray,
    tgt: np.ndarray,
    rot0_deg: float = 0.0,
    trans0: np.ndarray | None = None,
    max_dist: float = 25.0,
    iters: int = 40,
) -> tuple[np.ndarray, np.ndarray]:
    """Point-to-point ICP from an initial guess; returns (R, t) mapping src onto tgt.

    ``rot0_deg`` is the initial rotation in degrees, ``trans0`` the initial
    translation. With ``rot0_deg=0`` and no translation this is "no-move ICP",
    appropriate for incremental mapping where consecutive frames overlap heavily.
    """
    r0 = np.array(
        [
            [math.cos(math.radians(rot0_deg)), -math.sin(math.radians(rot0_deg))],
            [math.sin(math.radians(rot0_deg)), math.cos(math.radians(rot0_deg))],
        ]
    )
    t0 = np.zeros(2) if trans0 is None else np.asarray(trans0, dtype=np.float64)
    cur = src @ r0.T + t0
    r_tot, t_tot = r0.copy(), t0.copy()
    for _ in range(iters):
        d2 = ((cur[:, None, :] - tgt[None, :, :]) ** 2).sum(-1)
        j = d2.argmin(1)
        m = d2[np.arange(len(cur)), j] < max_dist**2
        if m.sum() < 5:
            break
        r, t = best_rigid(cur[m], tgt[j[m]])
        cur = cur @ r.T + t
        r_tot = r @ r_tot
        t_tot = r @ t_tot + t
    return r_tot, t_tot


def register(
    src: np.ndarray,
    tgt: np.ndarray,
    rot0_deg: float = 0.0,
    trans0: np.ndarray | None = None,
    max_dist: float = 25.0,
) -> tuple[np.ndarray, np.ndarray, int, float]:
    """ICP with a rotation search fallback.

    Runs ICP from the given initial guess, then (if ``rot0_deg`` was 0) tries
    rotations every 10 degrees and keeps the best fit by inlier count.

    Returns ``(R, t, inliers, rmse_cm)``.
    """
    best: tuple[int, float, np.ndarray, np.ndarray] | None = None
    starts = [rot0_deg] if rot0_deg else range(0, 360, 10)
    for deg in starts:
        r, t = icp(src, tgt, deg, trans0, max_dist)
        aligned = src @ r.T + t
        d = np.sqrt(((aligned[:, None, :] - tgt[None, :, :]) ** 2).sum(-1).min(1))
        inl = int((d < max_dist).sum())
        rmse = float(np.sqrt((d[d < max_dist] ** 2).mean())) if inl else float("inf")
        key = (inl, -rmse)
        if best is None or key > (best[0], best[1]):
            best = (inl, rmse, r, t)
    assert best is not None
    return best[2], best[3], best[0], best[1]


def transform(points: np.ndarray, r: np.ndarray, t: np.ndarray) -> np.ndarray:
    """Apply 2D rigid transform: points @ R.T + t."""
    return points @ r.T + t


# --------------------------------------------------------------------------
# Door / wall-gap features
# --------------------------------------------------------------------------


def detect_gaps(
    scan: np.ndarray,
    min_gap_cm: float = 100.0,
    ratio: float = 3.0,
) -> list[dict[str, float]]:
    """Find contiguous angle runs whose range is much longer than the median wall.

    ``scan`` is Nx2 [angle, dist]. Returns a list of dicts with keys
    ``center_deg``, ``span_deg``, ``dist_cm`` (mean distance inside the gap).
    A gap is where ``dist > max(min_gap_cm, ratio * median)``.
    """
    order = np.argsort(scan[:, 0])
    ang, dist = scan[order, 0], scan[order, 1]
    med = float(np.median(dist))
    threshold = max(min_gap_cm, ratio * med)
    mask = dist > threshold
    gaps: list[dict[str, float]] = []
    start: int | None = None
    for i in range(len(mask) + 1):
        is_gap = i < len(mask) and mask[i]
        if is_gap and start is None:
            start = i
        elif not is_gap and start is not None:
            a0, a1 = ang[start], ang[i - 1]
            span = (a1 - a0) * 180.0 / math.pi
            center = math.degrees(a0 + (a1 - a0) / 2.0) % 360.0
            gaps.append(
                {"center_deg": center, "span_deg": span, "dist_cm": float(dist[start:i].mean())}
            )
            start = None
    return gaps


# --------------------------------------------------------------------------
# Occupancy grid
# --------------------------------------------------------------------------


class OccupancyGrid:
    """Fixed 10 cm occupancy grid with a simple inverse-sensor model.

    Cell values: 0 = unknown, 1 = free, 2 = occupied.
    The grid spans ``side_cm`` x ``side_cm`` centered on the world origin.
    """

    def __init__(self, cell_cm: float = DEFAULT_CELL_CM, side_cm: float = 1000.0) -> None:
        self.cell = cell_cm
        self.n = round(side_cm / cell_cm)
        self.origin = self.n // 2  # grid index of world (0,0)
        self.grid = np.zeros((self.n, self.n), dtype=np.int8)

    def world_to_grid(self, xy: np.ndarray) -> np.ndarray:
        return np.round((xy + self.origin * self.cell) / self.cell).astype(int)

    def update(self, r: np.ndarray, t: np.ndarray, points: np.ndarray) -> None:
        """Fuse one scan frame: pose (R, t) maps sensor frame into world frame."""
        wall = transform(points, r, t)
        sensor = t
        s = self.world_to_grid(sensor)
        occ: set[tuple[int, int]] = set()
        free: set[tuple[int, int]] = set()
        for wg in self.world_to_grid(wall):
            if not (0 <= wg[0] < self.n and 0 <= wg[1] < self.n):
                continue
            occ.add((int(wg[0]), int(wg[1])))
            dx, dy = wg - s
            steps = int(np.hypot(dx, dy))
            for k in range(1, steps):
                g = s + np.round(np.array([dx, dy]) * k / steps).astype(int)
                if 0 <= g[0] < self.n and 0 <= g[1] < self.n:
                    free.add((int(g[0]), int(g[1])))
        for g in free - occ:
            self.grid[g] = max(self.grid[g], 1)
        for g in occ:
            self.grid[g] = 2

    def occupied_bounds_cm(self) -> tuple[float, float] | None:
        """Bounding box of occupied cells as (width_cm, height_cm), or None."""
        idx = np.argwhere(self.grid == 2)
        if not len(idx):
            return None
        xmin, xmax = idx[:, 0].min(), idx[:, 0].max()
        ymin, ymax = idx[:, 1].min(), idx[:, 1].max()
        return (xmax - xmin) * self.cell, (ymax - ymin) * self.cell


def map_scans(
    frames: list[tuple[str, np.ndarray]],
    grid: OccupancyGrid,
    incremental: bool = True,
) -> list[tuple[str, np.ndarray, np.ndarray, int, float]]:
    """Register a list of (name, sensor-frame point cloud) scans into one map.

    The first frame is anchored at the world origin. Each subsequent frame is
    ICP-registered against the accumulated map. Returns per-frame
    ``(name, R, t, inliers, rmse_cm)``.
    """
    poses: list[tuple[str, np.ndarray, np.ndarray, int, float]] = []
    map_pts: np.ndarray | None = None
    for i, (name, pts) in enumerate(frames):
        if i == 0:
            r, t = np.eye(2), np.zeros(2)
            inl, rmse = len(pts), 0.0
            map_pts = pts.copy()
        else:
            r, t, inl, rmse = register(pts, map_pts)
            map_pts = np.vstack([map_pts, transform(pts, r, t)])
        poses.append((name, r, t, inl, rmse))
        if incremental:
            grid.update(r, t, pts)
    return poses

"""Tests for the mapping module: ICP, gap detection, and the occupancy grid."""

from __future__ import annotations

import numpy as np

from carbot.mapping import (
    OccupancyGrid,
    best_rigid,
    detect_gaps,
    icp,
    load_polar_scan,
    polar_to_points,
    register,
)


def _known_transform_test(rot_deg: float, trans: np.ndarray, rng_seed: int = 0) -> float:
    rng = np.random.default_rng(rng_seed)
    pts = rng.uniform(-200, 200, (60, 2))
    ang = np.radians(rot_deg)
    r = np.array([[np.cos(ang), -np.sin(ang)], [np.sin(ang), np.cos(ang)]])
    moved = pts @ r.T + trans
    # ICP with a good initial guess must recover the transform exactly.
    guess_t = trans + np.array([7.0, -5.0])  # within the convergence basin
    r2, t2 = icp(pts, moved, rot0_deg=rot_deg, trans0=guess_t)
    return float(np.max(np.abs((pts @ r2.T + t2) - moved)))


def test_icp_recovers_known_transform():
    assert _known_transform_test(90, np.array([50.0, -30.0])) < 1e-6
    assert _known_transform_test(-30, np.array([-100.0, 40.0]), rng_seed=1) < 1e-6


def test_icp_incremental_small_move():
    """The M3 assumption: a small move (<15 cm, <15 deg) needs no initial guess."""
    rng = np.random.default_rng(2)
    pts = rng.uniform(-200, 200, (80, 2))
    ang = np.radians(10.0)
    r = np.array([[np.cos(ang), -np.sin(ang)], [np.sin(ang), np.cos(ang)]])
    moved = pts @ r.T + np.array([12.0, -8.0])
    r2, t2 = icp(pts, moved)  # no-move initial guess
    err = np.max(np.abs((pts @ r2.T + t2) - moved))
    assert err < 5.0, f"incremental ICP error too large: {err:.2f} cm"


def test_best_rigid_pure_translation():
    pts = np.array([[1.0, 2.0], [3.0, 4.0], [5.0, 6.0]])
    r, t = best_rigid(pts, pts + np.array([10.0, -3.0]))
    assert np.allclose(r, np.eye(2), atol=1e-12)
    assert np.allclose(t, [10.0, -3.0], atol=1e-12)


def test_register_returns_pose_inliers_rmse():
    rng = np.random.default_rng(3)
    pts = rng.uniform(-150, 150, (50, 2))
    ang = np.radians(25.0)
    r = np.array([[np.cos(ang), -np.sin(ang)], [np.sin(ang), np.cos(ang)]])
    moved = pts @ r.T + np.array([20.0, 15.0])
    r2, t2, inl, rmse = register(pts, moved, rot0_deg=25, trans0=np.array([15.0, 12.0]))
    assert inl == len(pts)
    assert rmse < 1e-6
    assert np.max(np.abs((pts @ r2.T + t2) - moved)) < 1e-6


def test_detect_gaps_finds_door():
    # synthetic scan: walls at 60 cm, a 220 cm gap spanning 30 degrees
    # (gap threshold is max(100, 3*median) = 180 cm, so 220 counts as a gap)
    ang = np.linspace(0, 2 * np.pi, 120, endpoint=False)
    dist = np.full_like(ang, 60.0)
    gap = (ang > np.radians(100)) & (ang < np.radians(130))
    dist[gap] = 220.0
    scan = np.column_stack([ang, dist])
    gaps = detect_gaps(scan)
    assert len(gaps) == 1
    assert abs(gaps[0]["center_deg"] - 115) < 5
    assert gaps[0]["span_deg"] > 25


def test_occupancy_grid_update_and_bounds():
    grid = OccupancyGrid(cell_cm=10.0, side_cm=200.0)
    # sensor at origin, wall points spread over >1 cell in both axes
    pts = np.array([[50.0, 0.0], [40.0, 25.0], [50.0, -25.0]])
    grid.update(np.eye(2), np.zeros(2), pts)
    assert (grid.grid == 2).sum() == 3  # three occupied cells
    assert (grid.grid == 1).sum() >= 4  # beam path marked free
    w, _ = grid.occupied_bounds_cm()
    assert w is not None and w > 0


def test_load_polar_scan_roundtrip(tmp_path):
    csv_path = tmp_path / "scan.csv"
    csv_path.write_text("elapsed_s,distance_cm,spin360_s\n0.00,100.0,8.0\n2.00,200.0,8.0\n")
    scan = load_polar_scan(csv_path)
    assert scan.shape == (2, 2)
    assert np.allclose(scan[:, 1], [100.0, 200.0])
    pts = polar_to_points(scan)
    assert np.allclose(np.hypot(pts[:, 0], pts[:, 1]), [100.0, 200.0])

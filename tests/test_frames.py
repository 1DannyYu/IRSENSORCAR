"""Gate A regression tests: scan timing and coordinate-frame conventions.

These lock the contracts that the mapping loop depends on:

- ``scan_angle_rad`` derives the angle from the *configured* spin duration, so
  ``--spin360`` can never drift apart from the angle conversion.
- ``polar_to_points`` uses angle=0 -> +y (sensor forward), matching the
  odometry axis in examples/11_sonar_explore_mapping.py.
- ``Pose2D`` heading follows the room convention atan2(Y, X) with the local
  forward axis on +y.
- ``SensorExtrinsics`` chains sensor -> chassis -> world with explicit signs.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from carbot.frames import Pose2D, SensorExtrinsics, scan_angle_rad
from carbot.mapping import load_polar_scan, points_to_scan, polar_to_points

# --------------------------------------------------------------------------
# Scan timing
# --------------------------------------------------------------------------


def test_scan_angle_zero_and_half_turn():
    assert scan_angle_rad(0.0, 8.2) == pytest.approx(0.0)
    assert scan_angle_rad(8.2 / 2.0, 8.2) == pytest.approx(math.pi)
    assert scan_angle_rad(8.2, 8.2) == pytest.approx(0.0)  # full turn wraps


def test_scan_angle_wraps_past_one_revolution():
    assert scan_angle_rad(9.0, 8.2) == pytest.approx(scan_angle_rad(0.8, 8.2))
    assert scan_angle_rad(16.4, 8.2) == pytest.approx(0.0)  # two full turns


def test_scan_angle_uses_configured_spin360():
    """Half a turn is pi regardless of the revolution duration used."""
    assert scan_angle_rad(2.05, 4.1) == pytest.approx(math.pi)
    assert scan_angle_rad(4.1, 8.2) == pytest.approx(math.pi)
    # Same physical turn, different configured duration -> different angle rate
    assert scan_angle_rad(2.0, 8.2) == pytest.approx(scan_angle_rad(1.0, 4.1))


def test_scan_angle_rejects_invalid_inputs():
    with pytest.raises(ValueError, match="positive"):
        scan_angle_rad(0.0, 0.0)
    with pytest.raises(ValueError, match="positive"):
        scan_angle_rad(0.0, -1.0)
    with pytest.raises(ValueError, match="non-negative"):
        scan_angle_rad(-0.1, 8.2)


# --------------------------------------------------------------------------
# Polar -> cartesian axis signs (sensor frame, angle=0 -> +y)
# --------------------------------------------------------------------------


def test_polar_to_points_axis_signs():
    pts = polar_to_points(
        np.asarray([[0.0, 100.0], [math.pi / 2, 100.0], [math.pi, 100.0], [-math.pi / 2, 100.0]])
    )
    expected = np.asarray([[0.0, 100.0], [100.0, 0.0], [0.0, -100.0], [-100.0, 0.0]])
    assert np.allclose(pts, expected, atol=1e-9)


def test_polar_to_points_roundtrip():
    rng = np.random.default_rng(7)
    scan = np.column_stack([rng.uniform(0, 2 * math.pi, 40), rng.uniform(20, 300, 40)])
    back = points_to_scan(polar_to_points(scan))
    # atan2 returns (-pi, pi]; compare angles modulo 2*pi
    diff = (back[:, 0] - scan[:, 0] + math.pi) % (2 * math.pi) - math.pi
    assert np.allclose(diff, 0.0, atol=1e-9)
    assert np.allclose(back[:, 1], scan[:, 1], atol=1e-9)


def test_load_polar_scan_uses_recorded_spin360(tmp_path):
    """CSV rows carry their own spin360; angle conversion must honour it."""
    csv_path = tmp_path / "scan.csv"
    csv_path.write_text("elapsed_s,distance_cm,spin360_s\n2.00,100.0,8.0\n2.00,100.0,4.0\n")
    scan = load_polar_scan(csv_path)
    assert scan[0, 0] == pytest.approx(math.pi / 2)  # 2/8 turn -> 90 deg
    assert scan[1, 0] == pytest.approx(math.pi)  # 2/4 turn -> 180 deg


# --------------------------------------------------------------------------
# Pose2D: heading convention and known transforms
# --------------------------------------------------------------------------


def test_pose_heading_roundtrip():
    for heading in (0.0, 90.0, 180.0, 270.0, -30.0, 33.7):
        assert Pose2D.from_xy_heading(0.0, 0.0, heading).heading_deg == pytest.approx(
            heading % 360.0
        )


def test_pose_forward_axis_points_along_heading():
    """Local +y (forward) must map to world direction `heading`."""
    for heading, world in (
        (0.0, [1.0, 0.0]),
        (90.0, [0.0, 1.0]),
        (180.0, [-1.0, 0.0]),
        (270.0, [0.0, -1.0]),
    ):
        pose = Pose2D.from_xy_heading(10.0, 20.0, heading)
        forward_world = pose.transform_points(np.asarray([[0.0, 1.0]]))[0] - pose.translation
        assert np.allclose(forward_world, world, atol=1e-9)


def test_pose_transform_known_point():
    """Car at (10,20) heading 0 (faces +X): point 100 cm ahead -> (110, 20)."""
    pose = Pose2D.from_xy_heading(10.0, 20.0, 0.0)
    assert np.allclose(
        pose.transform_points(np.asarray([[0.0, 100.0]])), [[110.0, 20.0]], atol=1e-9
    )


def test_pose_transform_roundtrip_inverse():
    rng = np.random.default_rng(11)
    pts = rng.uniform(-200, 200, (25, 2))
    pose = Pose2D.from_xy_heading(-15.0, 42.0, 77.0)
    moved = pose.transform_points(pts)
    # Exact inverse of x' = x @ R.T + t is x = x' @ R - t @ R (i.e. (R.T, -R.T@t))
    inv = Pose2D(pose.rotation.T, -(pose.rotation.T @ pose.translation))
    back = inv.transform_points(moved)
    assert np.allclose(back, pts, atol=1e-9)


def test_pose_rejects_reflection():
    with pytest.raises(ValueError, match="proper rotation"):
        Pose2D(np.diag([1.0, -1.0]), np.zeros(2))


def test_pose_rejects_non_orthonormal():
    with pytest.raises(ValueError, match="orthonormal"):
        Pose2D(np.asarray([[2.0, 0.0], [0.0, 1.0]]), np.zeros(2))


# --------------------------------------------------------------------------
# SensorExtrinsics: sensor -> chassis -> world
# --------------------------------------------------------------------------


def test_sensor_to_chassis_aligned():
    ext = SensorExtrinsics(np.asarray([5.0, 0.0]), yaw_deg=0.0)
    pts = ext.sensor_points_to_chassis(np.asarray([[0.0, 100.0]]))
    assert np.allclose(pts, [[5.0, 100.0]], atol=1e-9)  # forward + offset


def test_sensor_to_chassis_yaw_sign():
    """yaw=+90 rotates the sensor forward (local +y) onto chassis -x (left)."""
    ext = SensorExtrinsics(np.asarray([5.0, 0.0]), yaw_deg=90.0)
    pts = ext.sensor_points_to_chassis(np.asarray([[0.0, 100.0]]))
    assert np.allclose(pts, [[-95.0, 0.0]], atol=1e-9)


def test_sensor_points_to_world_chain():
    """Sensor at chassis (5,0) yaw 0; chassis at (10,10) heading 0 (faces +X).

    A wall point 100 cm ahead of the sensor sits at chassis (5,100). With the
    chassis facing +X, chassis +y (forward) maps to world +X and chassis +x
    (right) maps to world -Y — so the world position is
    (10 + 100, 10 - 5) = (110, 5).
    """
    ext = SensorExtrinsics(np.asarray([5.0, 0.0]), yaw_deg=0.0)
    chassis = Pose2D.from_xy_heading(10.0, 10.0, 0.0)
    world = ext.sensor_points_to_world(np.asarray([[0.0, 100.0]]), chassis)
    assert np.allclose(world, [[110.0, 5.0]], atol=1e-9)


def test_sensor_points_to_world_heading_90():
    """Chassis facing +Y (heading 90): world Y advances along the wall."""
    ext = SensorExtrinsics(np.asarray([0.0, 0.0]), yaw_deg=0.0)
    chassis = Pose2D.from_xy_heading(10.0, 20.0, 90.0)
    world = ext.sensor_points_to_world(np.asarray([[0.0, 50.0]]), chassis)
    assert np.allclose(world, [[10.0, 70.0]], atol=1e-9)


def test_extrinsics_rejects_bad_offset():
    with pytest.raises(ValueError):
        SensorExtrinsics(np.asarray([1.0, 2.0, 3.0]))
    with pytest.raises(ValueError):
        SensorExtrinsics(np.asarray([float("nan"), 0.0]))

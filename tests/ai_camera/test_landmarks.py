"""Tests for the flat-tag landmark localization layer (``carbot.landmarks``).

The camera-frame convention here is verified against the existing
hardware-verified wall-tag chain: a flat tag with ``yaw_deg=0`` (ID upright
when viewed from map-north) has its printed +X axis pointing map-east, +Y
map-south, +Z out of the paper. The synthetic poses in these tests are built
with a physically-constructed camera pose (proper rotation, optical axis
pointing at the tag), projected through ``cv2.projectPoints``, and solved
back with the same ``estimate_square_pose`` the example scripts use, so the
round trip exercises the real pipeline rather than circular math.
"""

from __future__ import annotations

import json

import numpy as np
import pytest

from carbot.landmarks import (
    TagMap,
    TagMapEntry,
    flat_tag_rotation_world_from_tag,
    load_tag_map,
    localize_camera,
)
from carbot.vision import (
    AprilTagPose,
    CameraCalibration,
    CameraWorldPose,
    estimate_square_pose,
)

CALIBRATION = CameraCalibration(
    width=2028,
    height=1520,
    camera_matrix=np.array([[1400.0, 0.0, 1014.0], [0.0, 1400.0, 760.0], [0.0, 0.0, 1.0]]),
    distortion_coefficients=np.zeros(5),
)
TAG_SIZE = 0.05


# ------------------------------------------------------------- helpers
def _object_points(size_m: float) -> np.ndarray:
    half = size_m / 2.0
    return np.asarray(
        [
            [-half, half, 0.0],
            [half, half, 0.0],
            [half, -half, 0.0],
            [-half, -half, 0.0],
        ],
        dtype=np.float64,
    )


def _build_camera_pose(
    position_m: tuple[float, float, float],
    look_at_m: tuple[float, float, float],
) -> CameraWorldPose:
    """A proper camera pose whose optical axis points at ``look_at_m``.

    Camera X = right of the heading, Y = image-down, Z = forward, built with
    the same construction used to verify the flat-tag convention on real
    geometry (see the probe that produced the module docstring numbers).
    """
    cam_pos = np.asarray(position_m, dtype=np.float64)
    forward = np.asarray(look_at_m, dtype=np.float64) - cam_pos
    forward = forward / np.linalg.norm(forward)
    right = np.cross(forward, np.array([0.0, 0.0, 1.0]))
    right = right / np.linalg.norm(right)
    down = np.cross(forward, right)
    rotation = np.column_stack([right, down, forward])
    return CameraWorldPose(cam_pos, rotation)


def _tag_pose_in_camera(
    tag_id: int,
    entry: TagMapEntry,
    camera_pose: CameraWorldPose,
    *,
    solve_size_m: float | None = None,
) -> AprilTagPose:
    """Synthesize the AprilTagPose a camera would detect for this tag."""
    import cv2

    world_from_camera = camera_pose.rotation_world_from_camera
    rotation_cam_from_tag = world_from_camera.T @ entry.rotation_world_from_tag
    translation = world_from_camera.T @ (entry.position_m - camera_pose.position_m)
    rotation_vector, _ = cv2.Rodrigues(rotation_cam_from_tag)
    corners, _ = cv2.projectPoints(
        _object_points(TAG_SIZE),
        rotation_vector,
        translation,
        CALIBRATION.camera_matrix,
        CALIBRATION.distortion_coefficients,
    )
    corners = corners.reshape(4, 2)
    solved, _, error = estimate_square_pose(
        corners, solve_size_m if solve_size_m is not None else TAG_SIZE, CALIBRATION
    )
    return AprilTagPose(
        tag_id=tag_id,
        corners_px=corners,
        rotation_vector=solved,
        translation_m=translation,
        reprojection_error_px=error,
        yaw_deg=0.0,
        pitch_deg=0.0,
        roll_deg=0.0,
    )


def _map(*entries: TagMapEntry) -> TagMap:
    return TagMap(name="test", entries={entry.tag_id: entry for entry in entries})


def _entry(
    tag_id: int, x: float, y: float, yaw: float = 0.0, size: float = TAG_SIZE
) -> TagMapEntry:
    return TagMapEntry(tag_id=tag_id, x_m=x, y_m=y, yaw_deg=yaw, size_m=size)


# ------------------------------------------------- rotation helper
class TestFlatTagRotation:
    def test_zero_yaw_is_identity(self) -> None:
        assert np.allclose(flat_tag_rotation_world_from_tag(0.0), np.eye(3))

    def test_positive_yaw_maps_east_to_north(self) -> None:
        # +90 deg = the printed +X axis points map-north (yaw is measured
        # counterclockwise from map-east in the SW-origin frame).
        rotation = flat_tag_rotation_world_from_tag(90.0)
        assert np.allclose(rotation @ np.array([1.0, 0.0, 0.0]), np.array([0.0, 1.0, 0.0]))

    def test_is_proper_rotation(self) -> None:
        for yaw in (0.0, 45.0, 90.0, 180.0, 270.0, -30.0):
            rotation = flat_tag_rotation_world_from_tag(yaw)
            assert np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-9)
            assert np.isclose(np.linalg.det(rotation), 1.0, atol=1e-9)

    def test_rejects_non_finite_yaw(self) -> None:
        with pytest.raises(ValueError):
            flat_tag_rotation_world_from_tag(float("nan"))


# ------------------------------------------------- tag map I/O
class TestTagMap:
    def test_load_valid_json(self, tmp_path) -> None:
        path = tmp_path / "tags.json"
        path.write_text(
            json.dumps(
                {
                    "name": "task1",
                    "tags": [
                        {"id": 0, "x_m": 0.64, "y_m": 0.38, "yaw_deg": 0.0, "size_m": 0.05},
                        {"id": 1, "x_m": 0.97, "y_m": 0.20, "yaw_deg": 90.0, "size_m": 0.05},
                    ],
                }
            ),
            encoding="utf-8",
        )
        tag_map = load_tag_map(path)
        assert tag_map.name == "task1"
        assert tag_map.entry(0) is not None
        assert tag_map.entry(0).position_m.shape == (3,)
        assert np.allclose(
            tag_map.entry(1).rotation_world_from_tag @ np.array([1.0, 0.0, 0.0]),
            np.array([0.0, 1.0, 0.0]),
        )
        assert tag_map.entry(2) is None

    @pytest.mark.parametrize(
        "payload",
        [
            {"tags": []},
            {"tags": [{"id": 0, "x_m": 0.0, "y_m": 0.0, "yaw_deg": 0.0, "size_m": 0.0}]},
            {"tags": [{"id": 0, "x_m": 0.0, "y_m": 0.0, "yaw_deg": 0.0, "size_m": -1.0}]},
            {"tags": [{"id": 0, "x_m": float("nan"), "y_m": 0.0, "yaw_deg": 0.0, "size_m": 0.05}]},
            {"tags": [{"id": 0, "x_m": 0.0, "y_m": 0.0, "yaw_deg": float("inf"), "size_m": 0.05}]},
            {
                "tags": [
                    {"id": 0, "x_m": 0.0, "y_m": 0.0, "yaw_deg": 0.0, "size_m": 0.05},
                    {"id": 0, "x_m": 0.1, "y_m": 0.1, "yaw_deg": 0.0, "size_m": 0.05},
                ]
            },
        ],
    )
    def test_load_rejects_invalid(self, tmp_path, payload) -> None:
        path = tmp_path / "tags.json"
        path.write_text(json.dumps(payload), encoding="utf-8")
        with pytest.raises(ValueError):
            load_tag_map(path)

    @pytest.mark.parametrize(
        "payload",
        [
            {"tags": "nope"},
            [],
            {"tags": [{"id": "a", "x_m": 0.0, "y_m": 0.0, "yaw_deg": 0.0, "size_m": 0.05}]},
            {"tags": [{"x_m": 0.0, "y_m": 0.0, "yaw_deg": 0.0, "size_m": 0.05}]},
        ],
    )
    def test_load_rejects_invalid_types(self, tmp_path, payload) -> None:
        path = tmp_path / "tags.json"
        path.write_text(json.dumps(payload), encoding="utf-8")
        with pytest.raises(TypeError):
            load_tag_map(path)


# ------------------------------------------------- localization
class TestLocalizeCamera:
    def test_recovers_camera_pose_from_one_tag(self) -> None:
        entry = _entry(0, 1.0, 0.5)
        camera_pose = _build_camera_pose((0.9, 0.5, 0.3), (1.0, 0.5, 0.0))
        tag_poses = [_tag_pose_in_camera(0, entry, camera_pose)]

        result = localize_camera(tag_poses, CALIBRATION, _map(entry))

        assert result is not None
        assert np.allclose(result.position_m, camera_pose.position_m, atol=0.01)
        assert result.heading_deg == pytest.approx(0.0, abs=1.0)
        assert result.visible_tag_ids == (0,)
        assert result.inlier_ids == (0,)

    def test_recovers_pose_with_rotated_tag(self) -> None:
        entry = _entry(3, 0.4, 0.6, yaw=90.0)
        camera_pose = _build_camera_pose((0.3, 0.4, 0.25), (0.4, 0.6, 0.0))
        tag_poses = [_tag_pose_in_camera(3, entry, camera_pose)]

        result = localize_camera(tag_poses, CALIBRATION, _map(entry))

        assert result is not None
        assert np.allclose(result.position_m, camera_pose.position_m, atol=0.01)
        # Camera looks south-east (heading ≈ atan2(+,-) in the 90-180 band).
        assert -90.0 <= result.heading_deg <= 90.0

    def test_heading_convention_counterclockwise_from_east(self) -> None:
        entry = _entry(0, 1.0, 1.0)
        # Camera north of the tag looking south: heading = -90 (0 = east,
        # 90 = north in the SW-origin frame).
        camera_pose = _build_camera_pose((1.0, 1.2, 0.3), (1.0, 1.0, 0.0))
        result = localize_camera(
            [_tag_pose_in_camera(0, entry, camera_pose)], CALIBRATION, _map(entry)
        )
        assert result is not None
        assert result.heading_deg == pytest.approx(-90.0, abs=1.0)

    def test_aggregates_three_tags_and_rejects_outlier(self) -> None:
        entries = [
            _entry(0, 1.0, 0.5),
            _entry(1, 0.9, 0.6, yaw=180.0),
            _entry(2, 0.95, 0.55, yaw=90.0),
        ]
        camera_pose = _build_camera_pose((0.9, 0.5, 0.3), (1.0, 0.5, 0.0))
        poses = [_tag_pose_in_camera(entry.tag_id, entry, camera_pose) for entry in entries]

        result = localize_camera(poses, CALIBRATION, _map(*entries))

        assert result is not None
        assert np.allclose(result.position_m, camera_pose.position_m, atol=0.01)
        assert set(result.visible_tag_ids) == {0, 1, 2}
        assert set(result.inlier_ids) == {0, 1, 2}

        # Now lie about tag 1's position: 10 cm off. The majority (0, 2)
        # agrees, so outlier rejection must drop the liar.
        wrong = _entry(1, 0.9, 0.70, yaw=180.0)
        result = localize_camera(poses, CALIBRATION, _map(entries[0], wrong, entries[2]))
        assert result is not None
        assert set(result.inlier_ids) == {0, 2}
        assert np.allclose(result.position_m, camera_pose.position_m, atol=0.01)

    def test_all_outliers_fall_back_to_best_single_tag(self) -> None:
        """Two disagreeing tags must not crash the navigation loop."""
        entries = [_entry(0, 1.0, 0.5), _entry(1, 0.9, 0.6, yaw=180.0)]
        camera_pose = _build_camera_pose((0.9, 0.5, 0.3), (1.0, 0.5, 0.0))
        poses = [
            _tag_pose_in_camera(0, entries[0], camera_pose),
            _tag_pose_in_camera(1, entries[1], camera_pose),
        ]
        # Both entries wrong by 10 cm in opposite directions.
        wrong0 = _entry(0, 1.0, 0.40)
        wrong1 = _entry(1, 0.9, 0.70, yaw=180.0)
        result = localize_camera(poses, CALIBRATION, _map(wrong0, wrong1))
        assert result is not None
        assert len(result.inlier_ids) == 1

    def test_ignores_unmapped_tags(self) -> None:
        entry = _entry(0, 1.0, 0.5)
        camera_pose = _build_camera_pose((0.9, 0.5, 0.3), (1.0, 0.5, 0.0))
        mapped = _tag_pose_in_camera(0, entry, camera_pose)
        stray = _tag_pose_in_camera(7, _entry(7, 0.5, 0.5), camera_pose)

        result = localize_camera([stray], CALIBRATION, _map(entry))
        assert result is None

        result = localize_camera([stray, mapped], CALIBRATION, _map(entry))
        assert result is not None
        assert result.visible_tag_ids == (0,)

    def test_returns_none_when_no_mapped_tag_visible(self) -> None:
        entry = _entry(0, 1.0, 0.5)
        assert localize_camera([], CALIBRATION, _map(entry)) is None

    def test_wrong_detection_size_is_fixed_by_map_size(self) -> None:
        """Pose solved at the wrong tag size must be re-solved with the map's.

        The pose translation scales linearly with tag size, so a detector
        configured with 40 mm while the map says 50 mm would scale every
        range by 0.8 and corrupt the localization. ``localize_camera`` must
        re-solve with the entry's size and still recover the camera.
        """
        entry = _entry(0, 1.0, 0.5)
        camera_pose = _build_camera_pose((0.9, 0.5, 0.3), (1.0, 0.5, 0.0))
        # Detect with a wrong size (40 mm instead of 50 mm).
        tag_poses = [_tag_pose_in_camera(0, entry, camera_pose, solve_size_m=0.04)]

        result = localize_camera(tag_poses, CALIBRATION, _map(entry))

        assert result is not None
        assert np.allclose(result.position_m, camera_pose.position_m, atol=0.01)

    def test_rejects_high_reprojection_error(self) -> None:
        """Corners with realistic pixel noise must fail a tight tolerance."""
        rng = np.random.default_rng(7)
        entry = _entry(0, 1.0, 0.5)
        camera_pose = _build_camera_pose((0.9, 0.5, 0.3), (1.0, 0.5, 0.0))
        tag_poses = [_tag_pose_in_camera(0, entry, camera_pose)]
        noisy = tag_poses[0]
        noisy = AprilTagPose(
            tag_id=noisy.tag_id,
            corners_px=noisy.corners_px + rng.normal(0.0, 0.4, noisy.corners_px.shape),
            rotation_vector=noisy.rotation_vector,
            translation_m=noisy.translation_m,
            reprojection_error_px=noisy.reprojection_error_px,
            yaw_deg=noisy.yaw_deg,
            pitch_deg=noisy.pitch_deg,
            roll_deg=noisy.roll_deg,
        )
        # Tight tolerance rejects the noisy observation...
        assert (
            localize_camera([noisy], CALIBRATION, _map(entry), max_reprojection_error_px=0.05)
            is None
        )
        # ...while the default tolerance still accepts it (as on hardware).
        assert localize_camera([noisy], CALIBRATION, _map(entry)) is not None

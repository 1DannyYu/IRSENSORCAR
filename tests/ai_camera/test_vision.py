"""Tests for calibration scaling, undistortion, and metric AprilTag pose."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

cv2 = pytest.importorskip("cv2")

from carbot.vision import (
    AprilTagPose,
    CameraCalibration,
    CameraWorldPose,
    CharucoBoardPose,
    aggregate_camera_world_poses,
    anchor_tags,
    camera_world_pose_from_tag,
    camera_world_pose_from_wall_board_and_tag,
    detect_apriltag_poses,
    detect_charuco_board_pose,
    estimate_square_pose,
    load_calibration,
    load_charuco_board_geometry,
    undistort_image,
    wall_board_rotation_world_from_board,
    wall_tag_rotation_world_from_tag,
)

ROOT = Path(__file__).resolve().parents[2]
CALIBRATION_PATH = (
    ROOT / "assets/reference/camera-calibration/2026-08-14-imx500-4056x3040/calibration.json"
)


def test_load_and_scale_real_calibration():
    calibration = load_calibration(CALIBRATION_PATH)
    scaled = calibration.scaled_to(2028, 1520)
    assert (calibration.width, calibration.height) == (4056, 3040)
    assert np.allclose(scaled.camera_matrix[:2], calibration.camera_matrix[:2] * 0.5)
    assert np.array_equal(scaled.distortion_coefficients, calibration.distortion_coefficients)


def test_load_measured_charuco_geometry():
    geometry = load_charuco_board_geometry(CALIBRATION_PATH)
    assert (geometry.squares_x, geometry.squares_y) == (5, 7)
    assert geometry.dictionary_name == "DICT_5X5_100"
    assert geometry.square_pitch_x_m == pytest.approx(0.028)
    assert geometry.square_pitch_y_m == pytest.approx(0.195 / 7)


def test_scale_rejects_cropped_aspect_ratio():
    calibration = load_calibration(CALIBRATION_PATH)
    with pytest.raises(ValueError, match="aspect ratio"):
        calibration.scaled_to(1920, 1080)


def test_estimate_square_pose_recovers_synthetic_translation():
    calibration = CameraCalibration(
        1280,
        960,
        np.asarray([[900.0, 0.0, 640.0], [0.0, 900.0, 480.0], [0.0, 0.0, 1.0]]),
        np.zeros(5),
    )
    tag_size = 0.070
    half = tag_size / 2.0
    object_points = np.asarray(
        [[-half, half, 0.0], [half, half, 0.0], [half, -half, 0.0], [-half, -half, 0.0]]
    )
    expected_rotation = np.asarray([0.08, -0.12, 0.03])
    expected_translation = np.asarray([0.04, -0.02, 0.75])
    corners, _ = cv2.projectPoints(
        object_points,
        expected_rotation,
        expected_translation,
        calibration.camera_matrix,
        calibration.distortion_coefficients,
    )
    _, translation, error = estimate_square_pose(corners, tag_size, calibration)
    assert np.allclose(translation, expected_translation, atol=1e-6)
    assert error < 1e-6


def test_estimate_square_pose_exact_for_rotated_tag():
    """IPPE_SQUARE's symmetry branch must be refined to the exact pose.

    On noiseless corners of a tag rotated 90 deg in the image, the closed-form
    IPPE_SQUARE solver returns a ~4 px-residual solution (the square's 4-fold
    symmetry); the iterative refinement seeded from it must recover the exact
    pose (verified 2026-08-16, see docs/adr/0003).
    """
    calibration = CameraCalibration(
        2028,
        1520,
        np.asarray([[1400.0, 0.0, 1014.0], [0.0, 1400.0, 760.0], [0.0, 0.0, 1.0]]),
        np.zeros(5),
    )
    tag_size = 0.05
    half = tag_size / 2.0
    object_points = np.asarray(
        [[-half, half, 0.0], [half, half, 0.0], [half, -half, 0.0], [-half, -half, 0.0]]
    )
    # Camera pitched down 40 deg toward the tag, whose +X is rotated 90 deg.
    pitch = np.deg2rad(40.0)
    rotation_cam_from_tag, _ = cv2.Rodrigues(np.asarray([0.0, pitch, np.pi / 2.0]))
    translation = np.asarray([0.0, 0.0, 0.4])
    corners, _ = cv2.projectPoints(
        object_points,
        rotation_cam_from_tag,
        translation,
        calibration.camera_matrix,
        calibration.distortion_coefficients,
    )
    rvec, tvec, error = estimate_square_pose(corners, tag_size, calibration)
    assert error < 1e-3
    assert np.allclose(tvec, translation, atol=1e-4)
    recovered, _ = cv2.Rodrigues(rvec)
    assert np.allclose(recovered, rotation_cam_from_tag, atol=1e-4)


def test_detect_marker_and_undistort_synthetic_image():
    width = height = 1000
    calibration = CameraCalibration(
        width,
        height,
        np.asarray([[900.0, 0.0, 500.0], [0.0, 900.0, 500.0], [0.0, 0.0, 1.0]]),
        np.zeros(5),
    )
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    marker = cv2.aruco.generateImageMarker(dictionary, 3, 400)
    image = np.full((height, width), 255, dtype=np.uint8)
    image[300:700, 300:700] = marker
    poses = detect_apriltag_poses(image, calibration, 0.070)
    corrected = undistort_image(image, calibration)
    assert [pose.tag_id for pose in poses] == [3]
    assert poses[0].forward_m == pytest.approx(0.1575, abs=0.003)
    assert poses[0].right_m == pytest.approx(0.0, abs=0.001)
    assert corrected.shape == image.shape


def test_detect_complete_charuco_board_pose():
    geometry = load_charuco_board_geometry(CALIBRATION_PATH)
    calibration = CameraCalibration(
        1000,
        1000,
        np.asarray([[900.0, 0.0, 500.0], [0.0, 900.0, 500.0], [0.0, 0.0, 1.0]]),
        np.zeros(5),
    )
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
    board = cv2.aruco.CharucoBoard((5, 7), 0.03, 0.022, dictionary)
    board_image = board.generateImage((500, 700))
    image = np.full((1000, 1000), 255, dtype=np.uint8)
    image[150:850, 250:750] = board_image

    pose = detect_charuco_board_pose(image, calibration, geometry)

    assert pose is not None
    assert pose.corner_count == 24
    assert pose.translation_m[2] > 0
    assert pose.reprojection_error_px < 2.0


def test_camera_world_pose_inverts_known_tag_observation():
    world_from_tag = wall_tag_rotation_world_from_tag(180.0)
    expected_world_from_camera = np.asarray([[0.0, 0.0, -1.0], [0.0, 1.0, 0.0], [1.0, 0.0, 0.0]])
    expected_camera_position = np.asarray([0.6, 0.08, 0.11])
    tag_position = np.asarray([0.0, 0.0, 0.132])
    camera_from_tag = expected_world_from_camera.T @ world_from_tag
    rotation_vector, _ = cv2.Rodrigues(camera_from_tag)
    translation = expected_world_from_camera.T @ (tag_position - expected_camera_position)
    detected = AprilTagPose(
        tag_id=0,
        corners_px=np.zeros((4, 2)),
        rotation_vector=rotation_vector.reshape(3),
        translation_m=translation,
        reprojection_error_px=0.0,
        yaw_deg=0.0,
        pitch_deg=0.0,
        roll_deg=0.0,
    )

    actual = camera_world_pose_from_tag(detected, tag_position, world_from_tag)

    assert np.allclose(actual.position_m, expected_camera_position)
    assert np.allclose(actual.rotation_world_from_camera, expected_world_from_camera)
    assert actual.wall_distance_m == pytest.approx(0.6)
    assert actual.wall_right_m == pytest.approx(0.08)
    assert actual.height_m == pytest.approx(0.11)
    assert abs(actual.heading_deg) == pytest.approx(180.0)
    assert actual.elevation_deg == pytest.approx(0.0)


def test_camera_world_pose_rejects_reflection():
    detected = AprilTagPose(
        tag_id=0,
        corners_px=np.zeros((4, 2)),
        rotation_vector=np.zeros(3),
        translation_m=np.asarray([0.0, 0.0, 1.0]),
        reprojection_error_px=0.0,
        yaw_deg=0.0,
        pitch_deg=0.0,
        roll_deg=0.0,
    )
    reflection = np.diag([1.0, 1.0, -1.0])
    with pytest.raises(ValueError, match="proper rotation"):
        camera_world_pose_from_tag(detected, np.zeros(3), reflection)


def test_hybrid_pose_uses_board_rotation_and_tag_translation():
    expected_world_from_camera = np.asarray([[0.0, 0.0, -1.0], [0.0, 1.0, 0.0], [1.0, 0.0, 0.0]])
    expected_camera_position = np.asarray([0.62, -0.01, 0.105])
    tag_position = np.asarray([0.0, 0.0, 0.147])
    world_from_board = wall_board_rotation_world_from_board()
    camera_from_board = expected_world_from_camera.T @ world_from_board
    board_rotation, _ = cv2.Rodrigues(camera_from_board)
    tag_translation = expected_world_from_camera.T @ (tag_position - expected_camera_position)
    tag = AprilTagPose(0, np.zeros((4, 2)), np.ones(3), tag_translation, 0.5, 0.0, 0.0, 0.0)
    board = CharucoBoardPose(
        np.zeros((24, 2)),
        np.arange(24),
        board_rotation.reshape(3),
        np.asarray([0.0, 0.0, 0.6]),
        0.5,
    )

    actual = camera_world_pose_from_wall_board_and_tag(tag, board, tag_position, world_from_board)

    assert np.allclose(actual.position_m, expected_camera_position)
    assert np.allclose(actual.rotation_world_from_camera, expected_world_from_camera)


def test_aggregate_camera_world_poses_rejects_outlier():
    rotations = [np.eye(3), cv2.Rodrigues(np.asarray([0.0, 0.0, 0.01]))[0]]
    poses = [
        CameraWorldPose(np.asarray([0.60, 0.0, 0.10]), rotations[0]),
        CameraWorldPose(np.asarray([0.602, 0.001, 0.101]), rotations[1]),
        CameraWorldPose(np.asarray([1.20, 0.5, 0.8]), rotations[0]),
    ]

    aggregate, inliers = aggregate_camera_world_poses(poses, 0.03, 3.0)

    assert inliers == [0, 1]
    assert np.allclose(aggregate.position_m, [0.601, 0.0005, 0.1005])


def _tag_pose(tag_id: int, dist_m: float = 1.0) -> AprilTagPose:
    return AprilTagPose(
        tag_id=tag_id,
        corners_px=np.zeros((4, 2)),
        rotation_vector=np.zeros(3),
        translation_m=np.asarray([0.0, 0.0, dist_m]),
        reprojection_error_px=0.0,
        yaw_deg=0.0,
        pitch_deg=0.0,
        roll_deg=0.0,
    )


def test_anchor_tags_filters_by_id_and_keeps_order():
    tags = [_tag_pose(0, 0.5), _tag_pose(1, 0.7), _tag_pose(0, 0.9)]
    hits = anchor_tags(tags, 0)
    assert [tag.tag_id for tag in hits] == [0, 0]
    assert [tag.range_m for tag in hits] == pytest.approx([0.5, 0.9])


def test_anchor_tags_not_visible():
    assert anchor_tags([_tag_pose(1), _tag_pose(2)], 0) == []
    assert anchor_tags([], 0) == []


def test_anchor_tags_duplicate_detection_is_visible_to_caller():
    """Two detections of the anchor ID must be distinguishable from one."""
    hits = anchor_tags([_tag_pose(0), _tag_pose(0)], 0)
    assert len(hits) == 2  # examples/13_cam_room_pose.py rejects this as duplicate-anchor


def test_anchor_not_visible_in_synthetic_blank_image():
    """A frame without the anchor yields zero detections (missing-anchor path)."""
    calibration = CameraCalibration(
        1000,
        1000,
        np.asarray([[900.0, 0.0, 500.0], [0.0, 900.0, 500.0], [0.0, 0.0, 1.0]]),
        np.zeros(5),
    )
    image = np.full((1000, 1000), 255, dtype=np.uint8)
    tags = detect_apriltag_poses(image, calibration, 0.070)
    assert tags == []
    assert anchor_tags(tags, 0) == []

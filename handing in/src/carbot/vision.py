"""Camera calibration, undistortion, and AprilTag pose estimation."""

from __future__ import annotations

import json
import math
from collections.abc import Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

APRILTAG_DICTIONARY = "DICT_APRILTAG_36h11"
DEFAULT_TAG_SIZE_M = 0.070


def _cv2() -> Any:
    try:
        import cv2
    except ImportError as exc:
        raise RuntimeError(
            "OpenCV with the aruco module is required. On the Pi use the system python3; "
            "on a development machine run with `uv run --extra vision`."
        ) from exc
    if not hasattr(cv2, "aruco"):
        raise RuntimeError("The installed OpenCV build does not include the aruco module.")
    return cv2


@dataclass(frozen=True)
class CameraCalibration:
    """OpenCV camera intrinsics for one uncropped image size."""

    width: int
    height: int
    camera_matrix: np.ndarray
    distortion_coefficients: np.ndarray

    def scaled_to(self, width: int, height: int) -> CameraCalibration:
        """Scale intrinsics for a resized full frame with the same aspect ratio."""
        if width <= 0 or height <= 0:
            raise ValueError("image dimensions must be positive")
        source_ratio = self.width / self.height
        target_ratio = width / height
        if not math.isclose(source_ratio, target_ratio, rel_tol=1e-6, abs_tol=1e-6):
            raise ValueError(
                "cannot scale calibration to a different aspect ratio; cropped modes need "
                "crop-aware intrinsics"
            )
        sx, sy = width / self.width, height / self.height
        matrix = self.camera_matrix.copy()
        matrix[0, 0] *= sx
        matrix[0, 2] *= sx
        matrix[1, 1] *= sy
        matrix[1, 2] *= sy
        return CameraCalibration(
            width=width,
            height=height,
            camera_matrix=matrix,
            distortion_coefficients=self.distortion_coefficients.copy(),
        )


@dataclass(frozen=True)
class AprilTagPose:
    """One tag pose in the OpenCV camera frame."""

    tag_id: int
    corners_px: np.ndarray
    rotation_vector: np.ndarray
    translation_m: np.ndarray
    reprojection_error_px: float
    yaw_deg: float
    pitch_deg: float
    roll_deg: float

    @property
    def right_m(self) -> float:
        return float(self.translation_m[0])

    @property
    def down_m(self) -> float:
        return float(self.translation_m[1])

    @property
    def forward_m(self) -> float:
        return float(self.translation_m[2])

    @property
    def range_m(self) -> float:
        return float(np.linalg.norm(self.translation_m))


@dataclass(frozen=True)
class CameraWorldPose:
    """Camera pose in a right-handed room frame: X away, Y right, Z up."""

    position_m: np.ndarray
    rotation_world_from_camera: np.ndarray

    @property
    def wall_distance_m(self) -> float:
        return float(self.position_m[0])

    @property
    def wall_right_m(self) -> float:
        return float(self.position_m[1])

    @property
    def height_m(self) -> float:
        return float(self.position_m[2])

    @property
    def optical_forward_world(self) -> np.ndarray:
        return self.rotation_world_from_camera[:, 2].copy()

    @property
    def heading_deg(self) -> float:
        forward = self.optical_forward_world
        return math.degrees(math.atan2(float(forward[1]), float(forward[0])))

    @property
    def elevation_deg(self) -> float:
        forward = self.optical_forward_world
        return math.degrees(
            math.atan2(float(forward[2]), math.hypot(float(forward[0]), float(forward[1])))
        )


@dataclass(frozen=True)
class CharucoBoardGeometry:
    """Printed ChArUco geometry, including independently measured X/Y scale."""

    squares_x: int
    squares_y: int
    dictionary_name: str
    nominal_square_length_m: float
    nominal_marker_length_m: float
    square_pitch_x_m: float
    square_pitch_y_m: float


@dataclass(frozen=True)
class CharucoBoardPose:
    """One ChArUco board pose in the OpenCV camera frame."""

    corners_px: np.ndarray
    corner_ids: np.ndarray
    rotation_vector: np.ndarray
    translation_m: np.ndarray
    reprojection_error_px: float

    @property
    def corner_count(self) -> int:
        return int(self.corner_ids.size)


def load_calibration(path: str | Path) -> CameraCalibration:
    """Load and validate the project's camera calibration JSON."""
    source = Path(path)
    with source.open(encoding="utf-8") as handle:
        data = json.load(handle)
    try:
        camera = data["camera"]
        lens = data["lens_model"]
        width = int(camera["image_width_px"])
        height = int(camera["image_height_px"])
        matrix = np.asarray(lens["camera_matrix"], dtype=np.float64)
        coefficients = np.asarray(lens["distortion_coefficients"], dtype=np.float64)
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(f"invalid calibration file: {source}") from exc
    if width <= 0 or height <= 0 or matrix.shape != (3, 3):
        raise ValueError(f"invalid calibration dimensions or camera matrix: {source}")
    if coefficients.ndim != 1 or coefficients.size not in (4, 5, 8, 12, 14):
        raise ValueError(f"unsupported distortion coefficient count: {coefficients.size}")
    if not np.isfinite(matrix).all() or not np.isfinite(coefficients).all():
        raise ValueError(f"calibration contains non-finite values: {source}")
    return CameraCalibration(width, height, matrix, coefficients)


def load_charuco_board_geometry(path: str | Path) -> CharucoBoardGeometry:
    """Load the measured printed-board geometry from a calibration JSON file."""
    source = Path(path)
    with source.open(encoding="utf-8") as handle:
        data = json.load(handle)
    try:
        board = data["calibration"]["board"]
        pitch = board["object_point_pitch_xy_m"]
        geometry = CharucoBoardGeometry(
            squares_x=int(board["squares_x"]),
            squares_y=int(board["squares_y"]),
            dictionary_name=str(board["dictionary"]),
            nominal_square_length_m=float(board["nominal_square_length_m"]),
            nominal_marker_length_m=float(board["nominal_marker_length_m"]),
            square_pitch_x_m=float(pitch[0]),
            square_pitch_y_m=float(pitch[1]),
        )
    except (KeyError, TypeError, ValueError, IndexError) as exc:
        raise ValueError(f"invalid ChArUco board metadata: {source}") from exc
    numeric = (
        geometry.nominal_square_length_m,
        geometry.nominal_marker_length_m,
        geometry.square_pitch_x_m,
        geometry.square_pitch_y_m,
    )
    if geometry.squares_x < 2 or geometry.squares_y < 2:
        raise ValueError(f"invalid ChArUco board dimensions: {source}")
    if not all(math.isfinite(value) and value > 0 for value in numeric):
        raise ValueError(f"invalid ChArUco board lengths: {source}")
    if geometry.nominal_marker_length_m >= geometry.nominal_square_length_m:
        raise ValueError(f"ChArUco marker must be smaller than its square: {source}")
    return geometry


def calibration_for_image(
    calibration: CameraCalibration,
    image: np.ndarray,
) -> CameraCalibration:
    """Return intrinsics scaled to an image's full-frame resolution."""
    if image.ndim not in (2, 3):
        raise ValueError("image must be grayscale or color")
    height, width = image.shape[:2]
    if (width, height) == (calibration.width, calibration.height):
        return calibration
    return calibration.scaled_to(width, height)


def undistort_image(image: np.ndarray, calibration: CameraCalibration) -> np.ndarray:
    """Correct lens distortion without changing the image dimensions."""
    cv2 = _cv2()
    adjusted = calibration_for_image(calibration, image)
    return cv2.undistort(
        image,
        adjusted.camera_matrix,
        adjusted.distortion_coefficients,
        None,
        adjusted.camera_matrix,
    )


def _tag_object_points(tag_size_m: float) -> np.ndarray:
    if not math.isfinite(tag_size_m) or tag_size_m <= 0:
        raise ValueError("tag_size_m must be a positive finite number")
    half = tag_size_m / 2.0
    return np.asarray(
        [
            [-half, half, 0.0],
            [half, half, 0.0],
            [half, -half, 0.0],
            [-half, -half, 0.0],
        ],
        dtype=np.float64,
    )


def estimate_square_pose(
    corners_px: np.ndarray,
    tag_size_m: float,
    calibration: CameraCalibration,
) -> tuple[np.ndarray, np.ndarray, float]:
    """Estimate a square tag pose and return ``(rvec, tvec, reprojection_error)``.

    Two solvers are tried and the one with the lower residual wins:
    ``SOLVEPNP_IPPE_SQUARE`` (closed-form, fast) and ``SOLVEPNP_ITERATIVE``
    (no seed). The closed-form solver's 4-fold symmetry can land on a wrong
    branch — verified 2026-08-16: on noiseless corners a tag rotated 90 deg
    resolves with ~4 px residual, and a tag viewed from the opposite side of
    the map with ~7 px residual, while plain iterative solves both exactly.
    Both candidates are validated for positive depth.
    """
    cv2 = _cv2()
    corners = np.asarray(corners_px, dtype=np.float64).reshape(4, 2)
    object_points = _tag_object_points(tag_size_m)

    def _projection_error(rotation, translation) -> float:
        projected, _ = cv2.projectPoints(
            object_points,
            rotation,
            translation,
            calibration.camera_matrix,
            calibration.distortion_coefficients,
        )
        residual = projected.reshape(4, 2) - corners
        return float(np.sqrt(np.mean(np.sum(residual**2, axis=1))))

    candidates = []
    for flags in (cv2.SOLVEPNP_IPPE_SQUARE, cv2.SOLVEPNP_ITERATIVE):
        ok, rotation, translation = cv2.solvePnP(
            object_points,
            corners,
            calibration.camera_matrix,
            calibration.distortion_coefficients,
            flags=flags,
        )
        if ok and float(translation.reshape(3)[2]) > 0:
            candidates.append(
                (
                    rotation.reshape(3),
                    translation.reshape(3),
                    _projection_error(rotation, translation),
                )
            )
    if not candidates:
        raise ValueError("could not estimate a positive-depth AprilTag pose")
    rotation, translation, error = min(candidates, key=lambda item: item[2])
    return rotation, translation, error


def _orientation_degrees(rotation_vector: np.ndarray) -> tuple[float, float, float]:
    cv2 = _cv2()
    rotation_matrix, _ = cv2.Rodrigues(rotation_vector)
    facing = -rotation_matrix[:, 2]
    yaw = math.degrees(math.atan2(float(facing[0]), float(facing[2])))
    pitch = math.degrees(
        math.atan2(float(facing[1]), math.hypot(float(facing[0]), float(facing[2])))
    )
    tag_right = rotation_matrix[:, 0]
    roll = math.degrees(math.atan2(float(tag_right[1]), float(tag_right[0])))
    return yaw, pitch, roll


def wall_tag_rotation_world_from_tag(clockwise_degrees: float = 0.0) -> np.ndarray:
    """Return an upright wall tag's orientation in the room frame.

    The angle describes the marker's canonical image rotation when viewed from the room.
    Zero maps marker right/up/outward to room right/up/away-from-wall.
    """
    if not math.isfinite(clockwise_degrees):
        raise ValueError("clockwise_degrees must be finite")
    angle = math.radians(clockwise_degrees)
    cosine, sine = math.cos(angle), math.sin(angle)
    return np.asarray(
        [
            [0.0, 0.0, 1.0],
            [cosine, sine, 0.0],
            [-sine, cosine, 0.0],
        ],
        dtype=np.float64,
    )


def wall_board_rotation_world_from_board(clockwise_degrees: float = 0.0) -> np.ndarray:
    """Return an upright ChArUco board's orientation in the room frame.

    OpenCV board X points right, Y points down, and Z points into the wall. The
    angle is the printed board's clockwise rotation when viewed from the room.
    """
    if not math.isfinite(clockwise_degrees):
        raise ValueError("clockwise_degrees must be finite")
    angle = math.radians(clockwise_degrees)
    cosine, sine = math.cos(angle), math.sin(angle)
    return np.asarray(
        [
            [0.0, 0.0, -1.0],
            [cosine, -sine, 0.0],
            [-sine, -cosine, 0.0],
        ],
        dtype=np.float64,
    )


def camera_world_pose_from_tag(
    pose: AprilTagPose,
    tag_position_world_m: np.ndarray,
    rotation_world_from_tag: np.ndarray,
) -> CameraWorldPose:
    """Invert a detected tag pose to locate the camera in a known room frame."""
    cv2 = _cv2()
    tag_position = np.asarray(tag_position_world_m, dtype=np.float64).reshape(-1)
    world_from_tag = np.asarray(rotation_world_from_tag, dtype=np.float64)
    if tag_position.shape != (3,) or not np.isfinite(tag_position).all():
        raise ValueError("tag_position_world_m must contain three finite values")
    if world_from_tag.shape != (3, 3) or not np.isfinite(world_from_tag).all():
        raise ValueError("rotation_world_from_tag must be a finite 3x3 matrix")
    if not np.allclose(world_from_tag.T @ world_from_tag, np.eye(3), atol=1e-6):
        raise ValueError("rotation_world_from_tag must be orthonormal")
    if not math.isclose(float(np.linalg.det(world_from_tag)), 1.0, abs_tol=1e-6):
        raise ValueError("rotation_world_from_tag must be a proper rotation")

    camera_from_tag, _ = cv2.Rodrigues(pose.rotation_vector)
    world_from_camera = world_from_tag @ camera_from_tag.T
    camera_position = tag_position - world_from_camera @ pose.translation_m
    return CameraWorldPose(camera_position, world_from_camera)


def camera_world_pose_from_wall_board_and_tag(
    tag_pose: AprilTagPose,
    board_pose: CharucoBoardPose,
    tag_position_world_m: np.ndarray,
    rotation_world_from_board: np.ndarray,
) -> CameraWorldPose:
    """Use a wall board for orientation and a known tag center for translation."""
    cv2 = _cv2()
    tag_position = np.asarray(tag_position_world_m, dtype=np.float64).reshape(-1)
    world_from_board = np.asarray(rotation_world_from_board, dtype=np.float64)
    if tag_position.shape != (3,) or not np.isfinite(tag_position).all():
        raise ValueError("tag_position_world_m must contain three finite values")
    if world_from_board.shape != (3, 3) or not np.isfinite(world_from_board).all():
        raise ValueError("rotation_world_from_board must be a finite 3x3 matrix")
    if not np.allclose(world_from_board.T @ world_from_board, np.eye(3), atol=1e-6):
        raise ValueError("rotation_world_from_board must be orthonormal")
    if not math.isclose(float(np.linalg.det(world_from_board)), 1.0, abs_tol=1e-6):
        raise ValueError("rotation_world_from_board must be a proper rotation")

    camera_from_board, _ = cv2.Rodrigues(board_pose.rotation_vector)
    world_from_camera = world_from_board @ camera_from_board.T
    camera_position = tag_position - world_from_camera @ tag_pose.translation_m
    return CameraWorldPose(camera_position, world_from_camera)


def _charuco_board(
    geometry: CharucoBoardGeometry,
) -> tuple[Any, np.ndarray]:
    cv2 = _cv2()
    dictionary_id = getattr(cv2.aruco, geometry.dictionary_name, None)
    if dictionary_id is None:
        raise RuntimeError(f"OpenCV does not provide {geometry.dictionary_name}")
    dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
    board = cv2.aruco.CharucoBoard(
        (geometry.squares_x, geometry.squares_y),
        geometry.nominal_square_length_m,
        geometry.nominal_marker_length_m,
        dictionary,
    )
    points = np.asarray(board.getChessboardCorners(), dtype=np.float64)
    points[:, 0] *= geometry.square_pitch_x_m / geometry.nominal_square_length_m
    points[:, 1] *= geometry.square_pitch_y_m / geometry.nominal_square_length_m
    return board, points


def detect_charuco_board_pose(
    image: np.ndarray,
    calibration: CameraCalibration,
    geometry: CharucoBoardGeometry,
    min_corners: int = 8,
) -> CharucoBoardPose | None:
    """Detect the measured ChArUco board and estimate its camera-relative pose."""
    cv2 = _cv2()
    if min_corners < 4:
        raise ValueError("min_corners must be at least 4")
    adjusted = calibration_for_image(calibration, image)
    board, measured_points = _charuco_board(geometry)
    parameters = cv2.aruco.CharucoParameters()
    parameters.cameraMatrix = adjusted.camera_matrix
    parameters.distCoeffs = adjusted.distortion_coefficients
    detector = cv2.aruco.CharucoDetector(board, parameters)
    corners, corner_ids, _, _ = detector.detectBoard(image)
    if corner_ids is None or len(corner_ids) < min_corners:
        return None

    ids = np.asarray(corner_ids, dtype=np.int32).reshape(-1)
    image_points = np.asarray(corners, dtype=np.float64).reshape(-1, 2)
    object_points = measured_points[ids]
    ok, rotation, translation = cv2.solvePnP(
        object_points,
        image_points,
        adjusted.camera_matrix,
        adjusted.distortion_coefficients,
        flags=cv2.SOLVEPNP_ITERATIVE,
    )
    if not ok or float(translation.reshape(3)[2]) <= 0:
        return None
    projected, _ = cv2.projectPoints(
        object_points,
        rotation,
        translation,
        adjusted.camera_matrix,
        adjusted.distortion_coefficients,
    )
    residual = projected.reshape(-1, 2) - image_points
    error = float(np.sqrt(np.mean(np.sum(residual**2, axis=1))))
    return CharucoBoardPose(
        corners_px=image_points,
        corner_ids=ids,
        rotation_vector=rotation.reshape(3),
        translation_m=translation.reshape(3),
        reprojection_error_px=error,
    )


def _mean_rotation(rotations: Sequence[np.ndarray]) -> np.ndarray:
    matrix = np.mean(np.asarray(rotations, dtype=np.float64), axis=0)
    left, _, right = np.linalg.svd(matrix)
    rotation = left @ right
    if np.linalg.det(rotation) < 0:
        left[:, -1] *= -1
        rotation = left @ right
    return rotation


def _rotation_distance_degrees(first: np.ndarray, second: np.ndarray) -> float:
    cosine = float(np.clip((np.trace(first.T @ second) - 1.0) / 2.0, -1.0, 1.0))
    return math.degrees(math.acos(cosine))


def aggregate_camera_world_poses(
    poses: Sequence[CameraWorldPose],
    max_position_deviation_m: float = 0.03,
    max_rotation_deviation_deg: float = 3.0,
) -> tuple[CameraWorldPose, list[int]]:
    """Reject pose outliers and return a median-position, mean-rotation estimate."""
    if not poses:
        raise ValueError("at least one camera pose is required")
    if max_position_deviation_m <= 0 or max_rotation_deviation_deg <= 0:
        raise ValueError("outlier thresholds must be positive")
    positions = np.asarray([pose.position_m for pose in poses], dtype=np.float64)
    rotations = [pose.rotation_world_from_camera for pose in poses]
    center_position = np.median(positions, axis=0)
    center_rotation = _mean_rotation(rotations)
    inliers = [
        index
        for index, (position, rotation) in enumerate(zip(positions, rotations, strict=True))
        if float(np.linalg.norm(position - center_position)) <= max_position_deviation_m
        and _rotation_distance_degrees(rotation, center_rotation) <= max_rotation_deviation_deg
    ]
    if not inliers:
        raise ValueError("all camera poses were rejected as outliers")
    aggregate_position = np.median(positions[inliers], axis=0)
    aggregate_rotation = _mean_rotation([rotations[index] for index in inliers])
    return CameraWorldPose(aggregate_position, aggregate_rotation), inliers


def detect_apriltag_poses(
    image: np.ndarray,
    calibration: CameraCalibration,
    tag_size_m: float = DEFAULT_TAG_SIZE_M,
) -> list[AprilTagPose]:
    """Detect AprilTag 36h11 markers and estimate metric camera-relative poses."""
    cv2 = _cv2()
    adjusted = calibration_for_image(calibration, image)
    dictionary_id = getattr(cv2.aruco, APRILTAG_DICTIONARY, None)
    if dictionary_id is None:
        raise RuntimeError(f"OpenCV does not provide {APRILTAG_DICTIONARY}")
    dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
    parameters = cv2.aruco.DetectorParameters()
    parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    detector = cv2.aruco.ArucoDetector(dictionary, parameters)
    gray = image if image.ndim == 2 else cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    marker_corners, marker_ids, _ = detector.detectMarkers(gray)
    if marker_ids is None:
        return []

    poses: list[AprilTagPose] = []
    for corners, marker_id in zip(marker_corners, marker_ids.reshape(-1), strict=True):
        points = np.asarray(corners, dtype=np.float64).reshape(4, 2)
        rotation, translation, error = estimate_square_pose(points, tag_size_m, adjusted)
        yaw, pitch, roll = _orientation_degrees(rotation)
        poses.append(
            AprilTagPose(
                tag_id=int(marker_id),
                corners_px=points,
                rotation_vector=rotation,
                translation_m=translation,
                reprojection_error_px=error,
                yaw_deg=yaw,
                pitch_deg=pitch,
                roll_deg=roll,
            )
        )
    return sorted(poses, key=lambda pose: pose.tag_id)


def anchor_tags(tags: Sequence[AprilTagPose], anchor_id: int) -> list[AprilTagPose]:
    """All detections of ``anchor_id``, in detection order.

    Callers must treat ``len(result) == 0`` as "anchor not visible" and
    ``len(result) > 1`` as a duplicate-anchor failure — never silently pick
    the first detection, because a duplicated wall tag would otherwise
    produce a plausible-but-wrong pose.
    """
    return [tag for tag in tags if tag.tag_id == anchor_id]


def annotate_apriltag_poses(
    image: np.ndarray,
    poses: list[AprilTagPose],
    calibration: CameraCalibration,
    axis_length_m: float = 0.035,
) -> np.ndarray:
    """Draw tag outlines, axes, and metric distance labels."""
    cv2 = _cv2()
    adjusted = calibration_for_image(calibration, image)
    output = image.copy()
    for pose in poses:
        corners = np.round(pose.corners_px).astype(np.int32).reshape(-1, 1, 2)
        cv2.polylines(output, [corners], True, (0, 255, 0), 3, cv2.LINE_AA)
        cv2.drawFrameAxes(
            output,
            adjusted.camera_matrix,
            adjusted.distortion_coefficients,
            pose.rotation_vector,
            pose.translation_m,
            axis_length_m,
            3,
        )
        x, y = corners.reshape(4, 2).min(axis=0)
        label = f"ID {pose.tag_id}: {pose.forward_m * 100:.1f} cm"
        cv2.putText(
            output,
            label,
            (int(x), max(30, int(y) - 12)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )
    return output


def annotate_charuco_board_pose(
    image: np.ndarray,
    pose: CharucoBoardPose,
    calibration: CameraCalibration,
    axis_length_m: float = 0.05,
) -> np.ndarray:
    """Draw detected ChArUco corners and the measured board pose axes."""
    cv2 = _cv2()
    adjusted = calibration_for_image(calibration, image)
    output = image.copy()
    cv2.aruco.drawDetectedCornersCharuco(
        output,
        pose.corners_px.reshape(-1, 1, 2).astype(np.float32),
        pose.corner_ids.reshape(-1, 1),
    )
    cv2.drawFrameAxes(
        output,
        adjusted.camera_matrix,
        adjusted.distortion_coefficients,
        pose.rotation_vector,
        pose.translation_m,
        axis_length_m,
        3,
    )
    return output

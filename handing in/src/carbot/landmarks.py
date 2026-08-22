"""Flat AprilTag landmarks on the Task-1 map: the absolute localization layer.

The chassis has no wheel encoders and no IMU, so the only absolute reference
is what the camera sees. AprilTags printed flat on the map (or taped on
while the map artwork is being finalised) give the camera a known 6-DOF
pose; combined with each tag's designed map position, the camera's position
and heading in the map frame follow directly.

This is the layer that removes the heuristic pile-up in ``carbot.line_nav``:
once the car knows *where it is*, junction decisions (turn at the T, exit
the roundabout at 3 o'clock) become deterministic events instead of guesses
from dark-strip width and timing. See ``docs/adr/0003-landmark-localization-task1.md``.

Frames and conventions (all right-handed):

* **map frame**: X east, Y north, Z up, origin at the map's **south-west
  corner**, in metres — the SW corner is (0, 0), the NE corner is (1.00,
  0.70). This is the operator's chosen frame (2026-08-16): distances are
  measured from the map's west and south edges, and a north-facing car has a
  positive heading. The SSOT orthophoto pixels (NW origin, y down, 10 px =
  1 cm) convert with ``map_y_m = 0.70 - photo_y_px / 1000`` (x unchanged).
* **tag frame** (OpenCV): X right / Y down in the printed tag's image plane,
  Z out of the paper (up when the tag lies flat). ``yaw_deg=0`` means the
  tag's ID is upright when viewed from map-north, i.e. the printed +X axis
  points map-east; ``yaw_deg`` is the angle of that +X axis measured
  counterclockwise from map-east (90 = +X points map-north).
* **camera frame** (OpenCV): X right, Y down, Z forward.

The camera-to-chassis offset is deliberately NOT in this module yet: for
junction decisions and heading turns the camera pose is sufficient, and the
offset is small (~10-20 cm) and fixed. Phase 2 of the landmark plan measures
it once (or derives it from a parked pose) and adds it as a constant offset
on top of :class:`LandmarkLocalization`.
"""

from __future__ import annotations

import json
import math
from collections.abc import Sequence
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from carbot.vision import (
    AprilTagPose,
    CameraCalibration,
    CameraWorldPose,
    aggregate_camera_world_poses,
    camera_world_pose_from_tag,
    estimate_square_pose,
)

#: Per-tag pose is re-solved from the detected corners with the *map's* size
#: for that tag, so a wrong uniform detection size cannot silently scale the
#: localization (pose translation scales linearly with tag size).
_LOCALIZE_RANGE_TOLERANCE_M = 0.5


def flat_tag_rotation_world_from_tag(yaw_degrees: float = 0.0) -> np.ndarray:
    """Return a flat tag's orientation in the map frame.

    ``yaw_degrees`` is the direction the tag's printed +X axis points,
    measured counterclockwise from map-east: 0 = +X east (ID upright when
    viewed from map-north), 90 = +X north, 180 = +X west, 270 = +X south.
    """
    if not math.isfinite(yaw_degrees):
        raise ValueError("yaw_degrees must be finite")
    angle = math.radians(yaw_degrees)
    cosine, sine = math.cos(angle), math.sin(angle)
    return np.asarray(
        [
            [cosine, -sine, 0.0],
            [sine, cosine, 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )


@dataclass(frozen=True)
class TagMapEntry:
    """One landmark's designed position and orientation on the map."""

    tag_id: int
    x_m: float
    y_m: float
    yaw_deg: float
    size_m: float
    z_m: float = 0.0

    @property
    def position_m(self) -> np.ndarray:
        return np.asarray([self.x_m, self.y_m, self.z_m], dtype=np.float64)

    @property
    def rotation_world_from_tag(self) -> np.ndarray:
        return flat_tag_rotation_world_from_tag(self.yaw_deg)


@dataclass(frozen=True)
class TagMap:
    """Designed landmark set for one map (the new reprint or a tape-on set)."""

    name: str
    entries: dict[int, TagMapEntry]

    def entry(self, tag_id: int) -> TagMapEntry | None:
        return self.entries.get(tag_id)


@dataclass(frozen=True)
class TagObservation:
    """One mapped tag seen in the current frame, with its camera pose."""

    tag_id: int
    range_m: float
    reprojection_error_px: float
    camera_pose: CameraWorldPose


@dataclass(frozen=True)
class LandmarkLocalization:
    """Camera pose in the map frame from all visible mapped tags."""

    camera_pose: CameraWorldPose
    observations: tuple[TagObservation, ...]
    inlier_ids: tuple[int, ...]

    @property
    def position_m(self) -> np.ndarray:
        return self.camera_pose.position_m

    @property
    def x_m(self) -> float:
        return float(self.position_m[0])

    @property
    def y_m(self) -> float:
        return float(self.position_m[1])

    @property
    def heading_deg(self) -> float:
        """Direction the camera faces on the map, degrees counterclockwise
        from map-east: 0 = east, 90 = north, 180 = west, -90 = south.

        The optical axis is projected onto the ground plane, so the forward
        tilt of the mount does not bias the heading. (The chassis heading
        equals this up to the fixed camera-to-chassis yaw offset, measured
        in Phase 2.)
        """
        forward = self.camera_pose.optical_forward_world
        return math.degrees(math.atan2(float(forward[1]), float(forward[0])))

    @property
    def visible_tag_ids(self) -> tuple[int, ...]:
        return tuple(obs.tag_id for obs in self.observations)


def load_tag_map(path: str | Path) -> TagMap:
    """Load a tag-map JSON: ``{"name": str, "tags": [{"id", "x_m", "y_m",
    "yaw_deg", "size_m", "z_m"?}]}``."""
    with Path(path).open(encoding="utf-8") as handle:
        raw = json.load(handle)
    if not isinstance(raw, dict):
        raise TypeError("tag map JSON must be an object")
    name = raw.get("name", "unnamed")
    raw_entries = raw.get("tags")
    if not isinstance(raw_entries, list):
        raise TypeError('tag map JSON "tags" must be a list')
    if not raw_entries:
        raise ValueError('tag map JSON "tags" must be non-empty')
    entries: dict[int, TagMapEntry] = {}
    for item in raw_entries:
        if not isinstance(item, dict):
            raise TypeError("each tag map entry must be an object")
        tag_id = item.get("id")
        if not isinstance(tag_id, int) or isinstance(tag_id, bool):
            raise TypeError('each tag map entry needs an integer "id"')
        if tag_id in entries:
            raise ValueError(f"duplicate tag id in tag map: {tag_id}")
        try:
            entry = TagMapEntry(
                tag_id=tag_id,
                x_m=float(item["x_m"]),
                y_m=float(item["y_m"]),
                z_m=float(item.get("z_m", 0.0)),
                yaw_deg=float(item["yaw_deg"]),
                size_m=float(item["size_m"]),
            )
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError(f"invalid tag map entry {tag_id}: {exc}") from exc
        if (
            not math.isfinite(entry.x_m)
            or not math.isfinite(entry.y_m)
            or not math.isfinite(entry.z_m)
        ):
            raise ValueError(f"tag {tag_id}: position must be finite")
        if not math.isfinite(entry.yaw_deg):
            raise ValueError(f"tag {tag_id}: yaw_deg must be finite")
        if not math.isfinite(entry.size_m) or entry.size_m <= 0:
            raise ValueError(f"tag {tag_id}: size_m must be positive")
        entries[tag_id] = entry
    return TagMap(name=name, entries=entries)


def _solve_pose_with_entry_size(
    pose: AprilTagPose, entry: TagMapEntry, calibration: CameraCalibration
) -> AprilTagPose:
    """Re-solve a detected tag with the map's true size for that tag."""
    rotation, translation, error = estimate_square_pose(pose.corners_px, entry.size_m, calibration)
    return AprilTagPose(
        tag_id=pose.tag_id,
        corners_px=pose.corners_px,
        rotation_vector=rotation,
        translation_m=translation,
        reprojection_error_px=error,
        yaw_deg=pose.yaw_deg,
        pitch_deg=pose.pitch_deg,
        roll_deg=pose.roll_deg,
    )


def localize_camera(
    tag_poses: Sequence[AprilTagPose],
    calibration: CameraCalibration,
    tag_map: TagMap,
    *,
    max_position_deviation_m: float = 0.03,
    max_rotation_deviation_deg: float = 3.0,
    max_reprojection_error_px: float = 3.0,
) -> LandmarkLocalization | None:
    """Locate the camera in the map frame from all visible mapped tags.

    Only tags with a map entry are used; unknown ids are ignored (they may be
    stray tags in the background). Per-tag poses are re-solved with the map's
    own size for that tag, then aggregated with outlier rejection from
    ``carbot.vision.aggregate_camera_world_poses``. Returns ``None`` when no
    mapped tag is visible.
    """
    observations: list[TagObservation] = []
    for pose in tag_poses:
        entry = tag_map.entry(pose.tag_id)
        if entry is None:
            continue
        if pose.range_m > _LOCALIZE_RANGE_TOLERANCE_M:
            continue
        solved = _solve_pose_with_entry_size(pose, entry, calibration)
        if solved.reprojection_error_px > max_reprojection_error_px:
            continue
        camera_pose = camera_world_pose_from_tag(
            solved, entry.position_m, entry.rotation_world_from_tag
        )
        observations.append(
            TagObservation(
                tag_id=pose.tag_id,
                range_m=solved.range_m,
                reprojection_error_px=solved.reprojection_error_px,
                camera_pose=camera_pose,
            )
        )
    if not observations:
        return None

    camera_poses = [obs.camera_pose for obs in observations]
    if len(camera_poses) == 1:
        aggregate, inliers = camera_poses[0], [0]
    else:
        try:
            aggregate, inliers = aggregate_camera_world_poses(
                camera_poses,
                max_position_deviation_m=max_position_deviation_m,
                max_rotation_deviation_deg=max_rotation_deviation_deg,
            )
        except ValueError:
            # Every tag disagrees (e.g. two tags, one mislabelled in the map):
            # degrade to the single most-trusted observation instead of
            # crashing the navigation loop.
            best = min(observations, key=lambda obs: obs.reprojection_error_px)
            aggregate, inliers = best.camera_pose, [observations.index(best)]
    return LandmarkLocalization(
        camera_pose=aggregate,
        observations=tuple(observations),
        inlier_ids=tuple(observations[index].tag_id for index in inliers),
    )

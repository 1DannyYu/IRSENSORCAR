"""Typed 2D frame and pose conversions for the mapping pipeline.

Gate A goal: stop passing anonymous ``np.ndarray`` values between vision and
mapping, and give the camera / chassis-rotation-center / HC-SR04 offsets an
explicit, validated representation. The numeric extrinsics are *not* measured
yet — they are filled in during Gate B (manual measurement) and Gate C
(rotation calibration). What is fixed here is the transform contract:

- **Sensor frame** (HC-SR04 spin-scan): polar (angle, distance), where
  ``angle=0`` points at the sensor's forward axis. ``mapping.polar_to_points``
  converts to ``[d*sin(angle), d*cos(angle)]`` so forward is ``+y``.
- **Chassis frame**: x right, y forward, rotation center at the origin.
- **World frame** (room, from vision): X away from the anchor wall, Y along
  the wall to the right, Z up. 2D projection uses (X, Y) with heading
  ``atan2(Y, X)`` — the same convention as ``vision.CameraWorldPose.heading_deg``.

All rotations are right-handed (counter-clockwise positive in the usual
x/y plane); 2D rotation matrices are stored as 2x2 arrays and applied as
``points @ R.T + t``, matching ``mapping.transform``.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Self

import numpy as np

# --------------------------------------------------------------------------
# Scan timing
# --------------------------------------------------------------------------


def scan_angle_rad(elapsed_s: float, spin360_s: float) -> float:
    """Map an elapsed spin time to a scan angle in radians.

    ``spin360_s`` is the duration of one full revolution **for this run's
    spin speed** — the value must travel with the capture, never be inferred
    from a global constant measured at another speed. Angles wrap past 360
    degrees (the caller may slice a scan across the wrap boundary).
    """
    if spin360_s <= 0.0:
        raise ValueError(f"spin360_s must be positive, got {spin360_s}")
    if elapsed_s < 0.0:
        raise ValueError(f"elapsed_s must be non-negative, got {elapsed_s}")
    return (elapsed_s % spin360_s) / spin360_s * 2.0 * math.pi


# --------------------------------------------------------------------------
# 2D pose
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class Pose2D:
    """A rigid 2D pose: rotation R (2x2) and translation t (2,).

    ``transform_points`` maps points from the pose's local frame into the
    parent frame: ``points @ R.T + t``.
    """

    rotation: np.ndarray
    translation: np.ndarray

    def __post_init__(self) -> None:
        r = np.asarray(self.rotation, dtype=np.float64)
        t = np.asarray(self.translation, dtype=np.float64)
        if r.shape != (2, 2) or t.shape != (2,):
            raise ValueError("rotation must be 2x2 and translation must be (2,)")
        if not np.allclose(r.T @ r, np.eye(2), atol=1e-9):
            raise ValueError("rotation must be orthonormal")
        if not np.allclose(np.linalg.det(r), 1.0, atol=1e-9):
            raise ValueError("rotation must be a proper rotation (det=+1)")
        object.__setattr__(self, "rotation", r)
        object.__setattr__(self, "translation", t)

    @classmethod
    def from_xy_heading(cls, x: float, y: float, heading_deg: float) -> Self:
        """Build a pose from world (x, y) and heading in degrees.

        Heading follows the room convention ``atan2(Y, X)``: 0 deg faces +X
        (away from the anchor wall), 90 deg faces +Y (along the wall right).
        The local **forward axis is +y** (matching the sensor spin-scan
        frame), so the rotation matrix maps local +y onto the world heading:
        ``R = [[sin H, cos H], [-cos H, sin H]]``.
        """
        h = math.radians(heading_deg)
        return cls(
            np.asarray(
                [
                    [math.sin(h), math.cos(h)],
                    [-math.cos(h), math.sin(h)],
                ]
            ),
            np.asarray([x, y], dtype=np.float64),
        )

    @property
    def heading_deg(self) -> float:
        """World heading of the local +y axis (forward), degrees."""
        return (
            math.degrees(math.atan2(float(self.rotation[1, 1]), float(self.rotation[0, 1]))) % 360.0
        )

    def transform_points(self, points: np.ndarray) -> np.ndarray:
        """Map local-frame points into the parent frame."""
        pts = np.asarray(points, dtype=np.float64)
        return pts @ self.rotation.T + self.translation


# --------------------------------------------------------------------------
# Sensor extrinsics on the chassis
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class SensorExtrinsics:
    """Rigid offset of a sensor frame relative to the chassis rotation center.

    ``offset_cm`` is the sensor origin (camera optical center or HC-SR04 echo
    midpoint) expressed in the chassis frame (x right, y forward).
    ``yaw_deg`` rotates the sensor's forward axis relative to chassis forward;
    the sensor frame is ``pose = (rotate by yaw_deg) then (translate by
    offset_cm)`` when moving from sensor to chassis.
    """

    offset_cm: np.ndarray
    yaw_deg: float = 0.0

    def __post_init__(self) -> None:
        offset = np.asarray(self.offset_cm, dtype=np.float64)
        if offset.shape != (2,):
            raise ValueError("offset_cm must be (2,)")
        if not np.all(np.isfinite(offset)):
            raise ValueError("offset_cm must be finite")
        object.__setattr__(self, "offset_cm", offset)

    @property
    def sensor_to_chassis(self) -> Pose2D:
        """Pose mapping sensor-frame points into the chassis frame.

        ``yaw_deg`` rotates the sensor's forward axis relative to chassis
        forward: 0 = aligned (sensor +y on chassis +y), positive = counter-
        clockwise (left turn), matching the standard rotation matrix
        ``R = [[cos yaw, -sin yaw], [sin yaw, cos yaw]]``.
        """
        rad = math.radians(self.yaw_deg)
        return Pose2D(
            np.asarray(
                [
                    [math.cos(rad), -math.sin(rad)],
                    [math.sin(rad), math.cos(rad)],
                ]
            ),
            self.offset_cm,
        )

    def sensor_points_to_chassis(self, points: np.ndarray) -> np.ndarray:
        """Map sensor-frame points into the chassis frame (offset + yaw)."""
        return self.sensor_to_chassis.transform_points(points)

    def sensor_points_to_world(self, points: np.ndarray, chassis_in_world: Pose2D) -> np.ndarray:
        """Map sensor-frame points into the world frame.

        ``chassis_in_world`` is the chassis rotation-center pose in the room
        frame (e.g. from the vision anchor). Chain: sensor -> chassis -> world.
        """
        return chassis_in_world.transform_points(self.sensor_points_to_chassis(points))

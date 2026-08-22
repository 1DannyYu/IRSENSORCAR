"""Visual + sonar obstacle fusion for the patrol loop, testable without hardware.

A single forward HC-SR04 cannot see thin chair legs or an overhead tabletop, so
the car drove under a chair and stuck on its underside
(`docs/progress/2026-08-14-visual-mapping-and-patrol.md`, pitfall 6).
`examples/20_cam_detection_check.py` proved on the Pi that the IMX500
on-sensor SSD detector does see them. This module holds that verdict logic and
the sonar fusion around it as pure functions, so the patrol's stop/turn
decision can be unit-tested with synthetic detections — no camera, no motors.

Picamera2 and the IMX500 API exist only on the Pi, so nothing here imports them
at module scope. :func:`detections_from_metadata` is the one hardware-facing
function and imports lazily.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class Detection:
    """One detector box in preview-stream pixels.

    The field names spell out the box convention on purpose:
    ``IMX500.convert_inference_coords`` returns ``(x, y, w, h)``, not
    ``(x0, y0, x1, y1)``. Unpacking it as corners produced negative widths and
    heights (pitfall 7), so a non-positive ``width``/``height`` is rejected here
    rather than silently yielding a wrong verdict. ``x``/``y`` may be negative
    because a box can extend past the frame edge.

    ``name`` carries the class name resolved from the *network's own* label
    list. This module deliberately keeps no label table of its own: the SSD
    mobilenetv2 `.rpk` on this Pi reports 90 labels in the COCO-91 index space
    (61 = chair, 81 = refrigerator, 84 = clock), so the common 80-class COCO
    list is off by several entries and silently renames chairs to "toilet".
    Without a name, :meth:`label` reports the raw index rather than guessing.
    """

    category: int
    confidence: float
    x: int
    y: int
    width: int
    height: int
    name: str | None = None

    def __post_init__(self) -> None:
        if self.width <= 0 or self.height <= 0:
            raise ValueError(
                f"detection box must have positive width and height, got "
                f"{self.width}x{self.height}; convert_inference_coords returns "
                f"(x, y, w, h), not corners"
            )

    @property
    def center_x(self) -> float:
        return self.x + self.width / 2

    @property
    def bottom(self) -> int:
        return self.y + self.height

    def label(self) -> str:
        """Class name from the network, or the raw index when it carried none."""
        return self.name if self.name else str(self.category)

    def area_fraction(self, frame_width: int, frame_height: int) -> float:
        """Box area as a fraction of the frame."""
        if frame_width <= 0 or frame_height <= 0:
            raise ValueError("frame dimensions must be positive")
        return (self.width * self.height) / (frame_width * frame_height)


@dataclass(frozen=True)
class ObstaclePolicy:
    """Thresholds for the fused stop decision.

    Defaults are the values verified on the Pi. ``confidence_threshold`` is
    deliberately low: the SSD detector reports chairs at 0.32-0.44, and 0.55
    silently dropped every detection (pitfall 8). Occasional false "obstacle"
    verdicts are acceptable — a needless turn beats a collision.
    """

    confidence_threshold: float = 0.30
    center_x_fraction: float = 0.35
    min_bottom_fraction: float = 0.45
    min_area_fraction: float = 0.06
    sonar_stop_cm: float = 30.0

    def __post_init__(self) -> None:
        if not 0.0 < self.confidence_threshold <= 1.0:
            raise ValueError("confidence_threshold must be in (0, 1]")
        if not 0.0 < self.center_x_fraction <= 1.0:
            raise ValueError("center_x_fraction must be in (0, 1]")
        if not 0.0 <= self.min_bottom_fraction <= 1.0:
            raise ValueError("min_bottom_fraction must be in [0, 1]")
        if not 0.0 <= self.min_area_fraction <= 1.0:
            raise ValueError("min_area_fraction must be in [0, 1]")
        if self.sonar_stop_cm <= 0:
            raise ValueError("sonar_stop_cm must be positive")


@dataclass(frozen=True)
class ObstacleVerdict:
    """The fused decision plus the reason, for the patrol log."""

    blocked: bool
    reason: str
    sonar_cm: float | None
    blocking: tuple[Detection, ...]


def is_blocking(
    detection: Detection,
    frame_width: int,
    frame_height: int,
    policy: ObstaclePolicy | None = None,
) -> bool:
    """True when a detection sits in the car's path: central, low, and large.

    "Low in frame" stands in for "close to the car" — the camera looks forward
    and slightly down, so a nearer object's box bottom falls further down the
    frame. A high, small, or off-centre box is something the car will drive
    past, not into.

    The confidence test is ``>=`` so the documented 0.30 rule includes the
    boundary; `examples/20` filtered with a strict ``>``, which differs only on
    an exact-equality float that no detector emits in practice.
    """
    policy = policy or ObstaclePolicy()
    if frame_width <= 0 or frame_height <= 0:
        raise ValueError("frame dimensions must be positive")
    if detection.confidence < policy.confidence_threshold:
        return False
    central = abs(detection.center_x - frame_width / 2) < policy.center_x_fraction * frame_width
    low = detection.bottom > policy.min_bottom_fraction * frame_height
    large = detection.area_fraction(frame_width, frame_height) > policy.min_area_fraction
    return central and low and large


def blocking_detections(
    detections: Sequence[Detection],
    frame_width: int,
    frame_height: int,
    policy: ObstaclePolicy | None = None,
) -> tuple[Detection, ...]:
    """Every detection that blocks the path, in input order."""
    policy = policy or ObstaclePolicy()
    return tuple(d for d in detections if is_blocking(d, frame_width, frame_height, policy))


def fuse(
    sonar_cm: float | None,
    detections: Sequence[Detection],
    frame_size: tuple[int, int],
    policy: ObstaclePolicy | None = None,
) -> ObstacleVerdict:
    """Combine sonar and vision into one conservative stop decision.

    Blocked when the sonar reads closer than ``sonar_stop_cm``, **or** the sonar
    returns ``None``, **or** a detection blocks the path. ``None`` means "cannot
    confirm clear" — the HC-SR04 is blind below roughly 20 cm, and the first
    patrol treated that as free space and drove into a wall (pitfall 3).
    """
    policy = policy or ObstaclePolicy()
    frame_width, frame_height = frame_size
    blocking = blocking_detections(detections, frame_width, frame_height, policy)

    if sonar_cm is None:
        reason = "no sonar reading (blind zone or fault)"
        if blocking:
            reason += f"; vision: {_describe(blocking, frame_size)}"
        return ObstacleVerdict(True, reason, None, blocking)

    if sonar_cm < policy.sonar_stop_cm:
        reason = f"sonar {sonar_cm:.0f} cm < {policy.sonar_stop_cm:.0f} cm"
        if blocking:
            reason += f"; vision: {_describe(blocking, frame_size)}"
        return ObstacleVerdict(True, reason, sonar_cm, blocking)

    if blocking:
        return ObstacleVerdict(
            True,
            f"vision: {_describe(blocking, frame_size)} (sonar {sonar_cm:.0f} cm)",
            sonar_cm,
            blocking,
        )

    return ObstacleVerdict(False, f"clear (sonar {sonar_cm:.0f} cm)", sonar_cm, ())


def _describe(detections: Sequence[Detection], frame_size: tuple[int, int]) -> str:
    """Name each blocking detection with the box area that made it block.

    Area and box bottom are both included because they are the two tunables that
    decide how close something has to be before it stops the car. Area alone
    turned out not to separate the cases: the dining table that originally
    trapped the car filled 0.123 of the frame, and a spurious "scissors" at over
    a metre filled 0.11 — so raising the area threshold enough to drop the false
    block would also drop the obstacle the whole vision layer exists to catch.
    The box bottom is the other candidate, since something on the floor in front
    of the car sits lower in frame than a small object across the room.
    """
    width, height = frame_size
    return ", ".join(
        f"{d.label()} {d.confidence:.2f} area={d.area_fraction(width, height):.2f} "
        f"bottom={d.bottom / height:.2f}"
        for d in detections
    )


def detections_from_metadata(
    metadata: dict[str, Any],
    imx500: Any,
    intrinsics: Any,
    picam2: Any,
    policy: ObstaclePolicy | None = None,
    iou: float = 0.65,
    max_detections: int = 10,
) -> list[Detection]:
    """Read one IMX500 inference result out of a frame's metadata.

    The only hardware-facing function in this module; ported from
    `examples/20_cam_detection_check.py` after it was verified on the Pi.
    Returns an empty list when the frame carries no inference output, which
    happens on the first frames after the network is uploaded. Class names come
    from ``intrinsics.labels`` — the network's own list — never from a table in
    this module; see :class:`Detection`.
    """
    import numpy as np

    policy = policy or ObstaclePolicy()
    labels: Sequence[str] = getattr(intrinsics, "labels", None) or ()
    outputs = imx500.get_outputs(metadata, add_batch=True)
    if outputs is None:
        return []
    input_w, input_h = imx500.get_input_size()

    if intrinsics.postprocess == "nanodet":
        from picamera2.devices.imx500 import postprocess_nanodet_detection
        from picamera2.devices.imx500.postprocess import scale_boxes

        boxes, scores, classes = postprocess_nanodet_detection(
            outputs=outputs[0],
            conf=policy.confidence_threshold,
            iou_thres=iou,
            max_out_dets=max_detections,
        )[0]
        boxes = scale_boxes(boxes, 1, 1, input_h, input_w, False, False)
    else:
        boxes, scores, classes = outputs[0][0], outputs[1][0], outputs[2][0]
        if intrinsics.bbox_normalization:
            boxes = boxes / input_h
        if intrinsics.bbox_order == "xy":
            boxes = boxes[:, [1, 0, 3, 2]]

    results: list[Detection] = []
    for box, score, category in zip(boxes, scores, classes):
        if float(score) < policy.confidence_threshold:
            continue
        coords = np.asarray(box, dtype=np.float64)
        x, y, w, h = imx500.convert_inference_coords(coords, metadata, picam2)
        if w <= 0 or h <= 0:
            continue  # a box clipped to nothing at the frame edge
        index = int(category)
        results.append(
            Detection(
                category=index,
                confidence=float(score),
                x=int(x),
                y=int(y),
                width=int(w),
                height=int(h),
                name=labels[index] if 0 <= index < len(labels) else None,
            )
        )
    return results

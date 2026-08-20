#!/usr/bin/env python3
"""Measure how far the car actually drives, forward and in reverse.

Spin rate was measured carefully; travel speed never was. Scale-anchoring the
last patrol showed why that matters: the whole 30-frame run fitted inside 13 cm,
because three avoidance manoeuvres reversed for 0.6 s each against two forward
steps of 1.0 s. Whether that arithmetic really cancels depends on the forward and
reverse speeds being comparable, and nobody has checked.

The wall AprilTag makes the measurement clean and non-circular: its printed edge
length is known, so :func:`carbot.vision.detect_apriltag_poses` returns the
camera's position relative to it **in metres**, with no dependence on the
reconstruction whose scale is in question. Distance travelled is then just the
change in that position — no tape measure, which ADR 0002 ruled out.

Each duration is measured as a there-and-back pair: drive forward for ``t``,
measure, drive back for the same ``t``, measure again. That yields the forward
distance, the reverse distance, and the residual offset between them — the
quantity that decides whether an avoidance manoeuvre gives ground back.

**Motor-moving, and the car drives toward a wall.** An operator must stand beside
it able to cut main power instantly. Run `examples/14_all_sensors_preflight_check.py` first.

Setup: put the car on clear floor roughly 1.5-2 m from a wall AprilTag, facing it
squarely. Facing it squarely matters — the range component of the measurement is
the best-conditioned part, and it equals the full distance only when the car
drives along the line of sight.

    PYTHONPATH=src python3 examples/24_cam_linear_speed_check.py --speed 200
"""

from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from carbot.scale import camera_position_in_tag_frame
from carbot.vision import DEFAULT_TAG_SIZE_M, detect_apriltag_poses, load_calibration

DEFAULT_CALIBRATION = Path(
    "assets/reference/camera-calibration/2026-08-14-imx500-4056x3040/calibration.json"
)
STILL_SIZE = (2028, 1520)
DEFAULT_DURATIONS = (0.3, 0.5, 0.75, 1.0, 1.5)
MAX_REPROJECTION_PX = 1.0


@dataclass
class Observation:
    """One tag sighting: metric camera position in the tag frame, plus its range."""

    position_m: np.ndarray
    range_m: float
    reprojection_px: float


@dataclass
class Leg:
    """One commanded move and the displacement the tag reported for it."""

    duration_s: float
    direction: str
    displacement_m: float | None = None
    range_change_m: float | None = None
    note: str = ""

    @property
    def speed_m_s(self) -> float | None:
        if self.displacement_m is None or self.duration_s <= 0:
            return None
        return self.displacement_m / self.duration_s


def _observe(camera, calibration, tag_size_m: float, tag_id: int | None) -> Observation | None:
    """Capture one frame and locate the camera relative to the chosen tag."""
    import cv2

    path = Path("/tmp/linear-speed-frame.jpg")
    camera.capture_file(str(path))
    image = cv2.imread(str(path))
    if image is None:
        return None
    poses = [
        p
        for p in detect_apriltag_poses(image, calibration, tag_size_m)
        if p.reprojection_error_px <= MAX_REPROJECTION_PX
    ]
    if tag_id is not None:
        poses = [p for p in poses if p.tag_id == tag_id]
    if len(poses) != 1:
        return None  # zero tags, or an ambiguous choice between several
    pose = poses[0]
    return Observation(
        position_m=camera_position_in_tag_frame(pose.rotation_vector, pose.translation_m),
        range_m=pose.range_m,
        reprojection_px=pose.reprojection_error_px,
    )


def _drive(car, speed: int, seconds: float, direction: str) -> None:
    if car is None:
        return
    (car.forward if direction == "forward" else car.backward)(speed)
    time.sleep(seconds)
    car.stop()


def main() -> int:
    parser = argparse.ArgumentParser(description="Measure forward and reverse travel distance")
    parser.add_argument("--speed", type=int, default=200, help="drive speed 0-1000")
    parser.add_argument(
        "--durations",
        type=float,
        nargs="+",
        default=list(DEFAULT_DURATIONS),
        help="drive durations to measure (seconds)",
    )
    parser.add_argument("--repeats", type=int, default=1, help="measurements per duration")
    parser.add_argument(
        "--settle-s",
        type=float,
        default=1.0,
        help="seconds to let the chassis stop rocking before a frame",
    )
    parser.add_argument(
        "--min-range-m",
        type=float,
        default=0.45,
        help="abort before driving if the tag is closer than this",
    )
    parser.add_argument(
        "--tag-id",
        type=int,
        default=None,
        help="restrict to one tag id; default accepts whichever single tag is seen",
    )
    parser.add_argument("--tag-size-m", type=float, default=DEFAULT_TAG_SIZE_M)
    parser.add_argument("--calibration", type=Path, default=DEFAULT_CALIBRATION)
    args = parser.parse_args()

    if args.repeats < 1 or any(d <= 0 for d in args.durations):
        print("--repeats and --durations must be positive", file=sys.stderr)
        return 1

    try:
        calibration = load_calibration(args.calibration).scaled_to(*STILL_SIZE)
    except (OSError, ValueError) as exc:
        print(f"Could not load the camera calibration: {exc}", file=sys.stderr)
        return 1

    answer = input("Operator beside the car, clear floor ahead, power ready to cut? (yes/no) ")
    if answer.strip().lower() != "yes":
        print("Re-run when an operator is ready beside the car.")
        return 1

    from carbot import Car, NeZhaError

    try:
        car = Car()
    except NeZhaError as exc:
        print(f"Connection failed: {exc}")
        print("Run `examples/01_i2c_probe.py` first to debug the link.")
        return 1

    from picamera2 import Picamera2

    camera = Picamera2()
    try:
        camera.configure(camera.create_still_configuration(main={"size": STILL_SIZE}))
        camera.start()
        time.sleep(max(args.settle_s, 1.5))
    except Exception:
        camera.close()
        car.close()
        raise

    legs: list[Leg] = []
    try:
        start = _observe(camera, calibration, args.tag_size_m, args.tag_id)
        if start is None:
            print(
                "No single AprilTag is visible. Point the car squarely at one wall tag and re-run.",
                file=sys.stderr,
            )
            return 1
        print(f"Tag at {start.range_m:.3f} m, reprojection {start.reprojection_px:.2f} px")
        print(
            f"Measuring {len(args.durations)} durations x {args.repeats} repeats "
            f"at speed {args.speed}"
        )
        print("=" * 72)

        for duration in args.durations:
            for repeat in range(args.repeats):
                before = _observe(camera, calibration, args.tag_size_m, args.tag_id)
                if before is None:
                    print(f"  {duration:.2f}s: lost the tag, stopping")
                    break
                if before.range_m < args.min_range_m:
                    print(
                        f"  {duration:.2f}s: tag at {before.range_m:.2f} m is inside the "
                        f"{args.min_range_m:.2f} m safety margin — stopping"
                    )
                    break

                lost = False
                for direction in ("forward", "backward"):
                    leg = Leg(duration_s=duration, direction=direction)
                    _drive(car, args.speed, duration, direction)
                    time.sleep(args.settle_s)
                    after = _observe(camera, calibration, args.tag_size_m, args.tag_id)
                    if after is None:
                        # Carrying on would measure the next leg from a stale
                        # position and silently report the two moves as one.
                        leg.note = "lost the tag after moving — stopping this duration"
                        lost = True
                    else:
                        leg.displacement_m = float(
                            np.linalg.norm(after.position_m - before.position_m)
                        )
                        leg.range_change_m = before.range_m - after.range_m
                        before = after
                    legs.append(leg)

                    moved = (
                        f"{leg.displacement_m:.3f} m"
                        if leg.displacement_m is not None
                        else "   -   "
                    )
                    speed = f"{leg.speed_m_s:.3f} m/s" if leg.speed_m_s is not None else "-"
                    change = (
                        f"{leg.range_change_m:+.3f} m" if leg.range_change_m is not None else "-"
                    )
                    print(
                        f"  {duration:.2f}s {direction:8s} -> moved {moved} "
                        f"({speed}), range {change} {leg.note}"
                    )
                    if lost:
                        break
                if lost:
                    break

                if repeat == args.repeats - 1 and before is not None:
                    drift = float(np.linalg.norm(before.position_m - start.position_m))
                    print(f"  {duration:.2f}s residual from the start: {drift:.3f} m")
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        car.stop()
        camera.stop()
        camera.close()
        car.close()

    print("-" * 72)
    forward = [leg for leg in legs if leg.direction == "forward" and leg.speed_m_s is not None]
    reverse = [leg for leg in legs if leg.direction == "backward" and leg.speed_m_s is not None]
    if not forward or not reverse:
        print("Not enough measurements to compare forward and reverse travel.")
        return 1

    def _summary(name: str, group: list[Leg]) -> float:
        speeds = np.asarray([leg.speed_m_s for leg in group], dtype=np.float64)
        median = float(np.median(speeds))
        print(
            f"{name:8s} median {median:.3f} m/s  (range {speeds.min():.3f}-{speeds.max():.3f}, "
            f"{len(group)} legs)"
        )
        return median

    forward_speed = _summary("forward", forward)
    reverse_speed = _summary("reverse", reverse)
    print(f"reverse/forward speed ratio: {reverse_speed / forward_speed:.2f}")

    # The patrol's own numbers, so the answer lands where the question came from.
    net = forward_speed * 1.0 - reverse_speed * 0.6
    print(
        f"\nAt the patrol's defaults, one forward step (1.0 s) advances "
        f"{forward_speed:.2f} m and one avoidance backup (0.6 s) gives back "
        f"{reverse_speed * 0.6:.2f} m — net {net:+.2f} m per blocked-then-clear pair."
    )
    if net <= 0.05:
        print(
            "That is why the last run stayed inside 13 cm: the manoeuvre cancels the step. "
            "Lengthen --step-s, shorten --backup-s, or block less often."
        )
    return 0


if __name__ == "__main__":
    sys.exit(main())

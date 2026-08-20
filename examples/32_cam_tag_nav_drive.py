#!/usr/bin/env python3
"""Drive the Task-1 route with AprilTag-supervised black-line navigation.

**Motor-moving. An operator must stand beside the car able to cut main power
instantly.**

Pipeline per frame: bird's-eye black-line reading (``carbot.ground_view``)
for steering, plus AprilTag detection (``carbot.landmarks``) whose *position*
is median-filtered over the last frames and used by
:class:`carbot.tag_nav.TagSupervisedNav` to:

* hold until the car is confirmed in the departure zone (map position
  x≈0.59, y≈0.08);
* veto a visual T-turn until the tracked position has actually reached the T
  (y ≥ 0.20) — the 2026-08-17 runs spun ~17 cm too early and left the line;
* stop with a report if the position drifts more than 6 cm off the stem
  corridor x≈0.59.

Heading from tags is deliberately NOT used: per-tag heading estimates
disagree by tens of degrees (print/paste misalignment), so turns are driven
by the vision cross-bar + position supervision.

    # dry run (no motors): camera + detection + state machine
    PYTHONPATH=src python3 examples/32_cam_tag_nav_drive.py --dry-run --duration 10

    # supervised run
    PYTHONPATH=src python3 examples/32_cam_tag_nav_drive.py --duration 15 \\
        --ground-view /tmp/line-follow/ground-view.json \\
        --tag-map scratch/landmarks/task1-tag-map.json
"""

from __future__ import annotations

import argparse
import collections
import sys
import time
from pathlib import Path

import numpy as np

from carbot.line_follow import LinePolicy
from carbot.line_nav import NavPolicy


def _open_camera():
    from picamera2 import Picamera2

    camera = Picamera2()
    camera.configure(camera.create_preview_configuration(main={"size": (2028, 1520)}))
    camera.start()
    time.sleep(1.0)
    try:
        camera.set_controls(
            {"AeEnable": False, "ExposureTime": 50_000, "AnalogueGain": 4.5}
        )
        time.sleep(0.3)
    except Exception:  # noqa: BLE001 - camera controls are optional
        time.sleep(1.0)
    return camera


class PositionFilter:
    """Median filter over the last N camera positions (x, y only).

    Per-tag position estimates are usually consistent to a few mm but
    occasionally one tag (small/grazing view) is off by tens of cm — enough
    to pollute a plain median and false-trigger the off-track guard
    (2026-08-17: x jumped 0.55 → 0.84 → 0.25 across frames). Frames farther
    than ``max_deviation_m`` from the window median are dropped before the
    final median, so a single bad tag cannot move the filtered position.
    """

    def __init__(
        self,
        window: int = 7,
        min_fixes: int = 3,
        max_deviation_m: float = 0.05,
    ) -> None:
        self._xs: collections.deque[float] = collections.deque(maxlen=window)
        self._ys: collections.deque[float] = collections.deque(maxlen=window)
        self._min_fixes = min_fixes
        self._max_deviation = max_deviation_m
        self._last: tuple[float, float] | None = None

    def update(self, localization) -> tuple[float, float] | None:
        if localization is None:
            return None
        x, y = localization.x_m, localization.y_m
        # Temporal outlier rejection: a fresh fix that jumps more than
        # ``max_deviation_m`` from the last filtered position is a bad tag
        # observation (2026-08-17: x jumped 0.55 -> 0.43 and the median
        # window alone did not reject it, false-triggering OFF_TRACK).
        if (
            self._last is not None
            and abs(x - self._last[0]) > self._max_deviation
            and abs(y - self._last[1]) > self._max_deviation
        ):
            return None
        self._xs.append(x)
        self._ys.append(y)
        if len(self._xs) < self._min_fixes:
            return None
        med_x = float(np.median(self._xs))
        med_y = float(np.median(self._ys))
        ok = [
            (xx, yy)
            for xx, yy in zip(self._xs, self._ys, strict=True)
            if abs(xx - med_x) <= self._max_deviation
            and abs(yy - med_y) <= self._max_deviation
        ]
        if len(ok) < self._min_fixes:
            return None
        result = float(np.median([p[0] for p in ok])), float(np.median([p[1] for p in ok]))
        self._last = result
        return result


class _FilteredLoc:
    """Minimal localization-like object carrying the filtered (x, y) and the
    latest raw heading (used only for the coarse departure heading align)."""

    def __init__(self, x_m: float, y_m: float, heading_deg: float | None) -> None:
        self.x_m = x_m
        self.y_m = y_m
        self.heading_deg = heading_deg


def main() -> int:
    parser = argparse.ArgumentParser(description="Tag-supervised line-follow drive")
    parser.add_argument(
        "--dry-run", action="store_true", help="camera + nav only; never drive motors"
    )
    parser.add_argument("--duration", type=float, default=15.0)
    parser.add_argument("--speed", type=int, default=120)
    parser.add_argument("--threshold", type=int, default=70)
    parser.add_argument("--blind-creep-s", type=float, default=1.0)
    parser.add_argument(
        "--ground-view",
        type=Path,
        default=None,
        help="bird's-eye homography JSON (required for line detection)",
    )
    parser.add_argument(
        "--tag-map",
        type=Path,
        default=Path("scratch/landmarks/task1-tag-map.json"),
        help="AprilTag map JSON",
    )
    parser.add_argument("--save-every", type=int, default=0)
    parser.add_argument("--log-dir", type=Path, default=Path("/tmp/tag-nav"))
    args = parser.parse_args()

    from carbot.ground_view import load_optional_ground_view
    from carbot.landmarks import load_tag_map, localize_camera
    from carbot.line_follow import detect_line
    from carbot.line_nav import LineNav
    from carbot.tag_nav import TagSupervisedNav
    from carbot.vision import detect_apriltag_poses, load_calibration

    gv = load_optional_ground_view(args.ground_view)
    if gv is None:
        print("no ground view available; pass --ground-view", file=sys.stderr)
        return 1
    tag_map = load_tag_map(args.tag_map)
    calibration = load_calibration(
        "assets/reference/camera-calibration/2026-08-14-imx500-4056x3040/calibration.json"
    )

    line_policy = LinePolicy(dark_threshold=args.threshold)
    nav_policy = NavPolicy(speed=args.speed, blind_creep_s=args.blind_creep_s)
    nav = TagSupervisedNav(nav=LineNav(nav_policy))
    pos_filter = PositionFilter()
    print(
        f"TagSupervisedNav: departure ({0.59:.2f},{0.08:.2f}) → T ({0.59:.2f},{0.248:.2f}) "
        f"→ Phase 2 east; speed {args.speed}"
    )

    if not args.dry_run:
        answer = input("Operator beside the car, path clear, power ready to cut? (yes/no) ").strip()
        if answer.lower() != "yes":
            print("Re-run when an operator is ready beside the car.")
            return 1

    import cv2

    from carbot import Car, NeZhaError

    car = None
    if not args.dry_run:
        try:
            car = Car()
        except NeZhaError as exc:
            print(f"Connection failed: {exc}", file=sys.stderr)
            return 1

    try:
        camera = _open_camera()
    except Exception as exc:  # noqa: BLE001 - report any camera backend error
        print(f"camera failed: {exc}", file=sys.stderr)
        if car:
            car.close()
        return 1

    if args.save_every and args.save_every > 0:
        args.log_dir.mkdir(parents=True, exist_ok=True)

    start = time.monotonic()
    last = start
    frame_index = 0
    try:
        while True:
            now = time.monotonic()
            dt = min(now - last, 0.5)
            last = now
            frame_index += 1

            frame = camera.capture_array("main")
            reading = detect_line(frame, line_policy, ground_view=gv)
            image = frame[:, :, ::-1]
            h, w = image.shape[:2]
            scaled = calibration.scaled_to(w, h)
            poses = detect_apriltag_poses(image, scaled, 0.02)
            loc = localize_camera(poses, scaled, tag_map) if poses else None
            position = pos_filter.update(loc)

            # Pass the filtered position to the supervisor. Raw single-frame x values
            # previously jumped from 0.25 to 0.84 and falsely triggered OFF_TRACK.
            if position is not None:
                filtered_loc = _FilteredLoc(position[0], position[1], loc.heading_deg)
            else:
                filtered_loc = None
            cmd = nav.step(reading, dt, localization=filtered_loc)
            if car:
                car.drive(cmd.left, cmd.right)

            pos_str = (
                f"pos=({position[0]:.3f},{position[1]:.3f})" if position is not None else "pos=?"
            )
            print(
                f"[{now - start:6.1f}s] #{frame_index:4d} {pos_str} nav={nav.state} "
                f"{cmd.state.value}:{cmd.action} L{cmd.left} R{cmd.right} | "
                f"{reading.summary} | {cmd.reason[:60]}"
            )

            if args.save_every and frame_index % args.save_every == 0:
                annotated = frame.copy()
                cv2.putText(
                    annotated,
                    f"nav={nav.state} {pos_str}",
                    (20, 60),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1.2,
                    (0, 255, 0),
                    3,
                )
                cv2.imwrite(str(args.log_dir / f"frame-{frame_index:05d}.jpg"), annotated)

            if args.duration and (now - start) >= args.duration:
                print("duration limit reached; stopping")
                break
    except KeyboardInterrupt:
        print("\nstopped by operator")
    finally:
        if car:
            car.stop()
            car.close()
        camera.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())

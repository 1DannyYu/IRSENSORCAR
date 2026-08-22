#!/usr/bin/env python3
"""Measure the car's real spin rate and startup dead time, using the camera.

Turn angles have been open-loop from one measurement — 8.2 s per 360 deg at
speed 150, i.e. 43.9 deg/s — extrapolated to every speed and every angle. That
is good enough for random-bounce coverage. It is not good enough for a
photogrammetry sweep, which needs a burst of frames roughly 20 deg apart so
consecutive views overlap: a 20 deg step is under half a second of spin, and at
that scale the motor's startup transient is a large share of the motion.

This script commands a series of spin durations and measures what the car
actually did, by matching features between a frame taken before and after each
spin (:mod:`carbot.visual_yaw`). It then fits

    angle = rate * (duration - dead_time)

and prints the duration needed for a target step angle. No protractor and no
encoders — ADR 0002 ruled out measuring the robot by hand.

Durations are kept short on purpose: the lens covers about 66 deg, so a spin
past roughly 40 deg leaves too little overlap for the frames to be matched, and
the measurement would silently degrade instead of failing.

**Motor-moving.** An operator must stand beside the car able to cut main power
instantly. Run `examples/other/14_all_sensors_preflight_check.py` first. Directions alternate, so
the car oscillates around its starting heading instead of walking away — but
lift or secure the chassis if the space is tight.

    PYTHONPATH=src python3 examples/ai_camera/23_cam_spin_rate_check.py --speed 200
    PYTHONPATH=src python3 examples/ai_camera/23_cam_spin_rate_check.py --speed 150 --repeats 3

Point the car at a textured part of the room. A blank wall gives the matcher
nothing to work with; the script checks the first frame and says so.
"""

from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from carbot.frame_quality import assess_file
from carbot.vision import load_calibration
from carbot.visual_yaw import estimate_yaw_between_files

DEFAULT_CALIBRATION = Path(
    "assets/reference/camera-calibration/2026-08-14-imx500-4056x3040/calibration.json"
)
STILL_SIZE = (2028, 1520)
# Chosen to stay inside the overlap the matcher needs. Measured at speed 200:
# 0.65 s produced ~35 deg with only 41 matches and 0.80 s produced none at all,
# because the lens covers ~66 deg and a 42 deg turn leaves too little in common.
DEFAULT_DURATIONS = (0.15, 0.20, 0.25, 0.30, 0.35, 0.45, 0.55)
PREVIOUS_ASSUMPTION_DEG_PER_S = 360.0 / 8.2


@dataclass
class SpinSample:
    """One commanded spin and what the camera says actually happened."""

    duration_s: float
    direction: str
    measured_deg: float | None = None
    matches: int = 0
    spread_deg: float = 0.0
    trustworthy: bool = False
    note: str = ""

    @property
    def direction_agrees(self) -> bool | None:
        """Whether the measured rotation went the way the command claimed.

        Kept separate from ``trustworthy``: the spin *rate* is a magnitude and
        does not care which way the car went, so a direction surprise must not
        throw away a good rate measurement. It is reported on its own because
        the project's notes record that vendor docs and code comments disagree
        about motor direction.
        """
        if self.measured_deg is None:
            return None
        expected = 1.0 if self.direction == "right" else -1.0
        return self.measured_deg * expected > 0


def _fit_rate_and_dead_time(samples: list[SpinSample]) -> tuple[float, float] | None:
    """Least-squares fit of ``angle = rate * (duration - dead_time)``."""
    usable = [s for s in samples if s.trustworthy and s.measured_deg is not None]
    if len({s.duration_s for s in usable}) < 2:
        return None
    durations = np.asarray([s.duration_s for s in usable], dtype=np.float64)
    angles = np.asarray([abs(s.measured_deg) for s in usable], dtype=np.float64)
    slope, intercept = np.polyfit(durations, angles, 1)
    if slope <= 0:
        return None
    return float(slope), float(-intercept / slope)


def main() -> int:
    parser = argparse.ArgumentParser(description="Measure spin rate and startup dead time")
    parser.add_argument("--speed", type=int, default=200, help="drive speed 0-1000")
    parser.add_argument(
        "--repeats", type=int, default=2, help="measurements per duration; directions alternate"
    )
    parser.add_argument(
        "--durations",
        type=float,
        nargs="+",
        default=list(DEFAULT_DURATIONS),
        help="spin durations to measure (seconds)",
    )
    parser.add_argument(
        "--target-deg", type=float, default=20.0, help="step angle to report a duration for"
    )
    parser.add_argument(
        "--settle-s",
        type=float,
        default=1.0,
        help="seconds to let the chassis stop rocking before a frame",
    )
    parser.add_argument("--calibration", type=Path, default=DEFAULT_CALIBRATION)
    parser.add_argument("--out-dir", type=Path, default=Path("/tmp/spin-rate"))
    args = parser.parse_args()

    if args.repeats < 1:
        print("--repeats must be at least 1", file=sys.stderr)
        return 1
    if any(d <= 0 for d in args.durations):
        print("--durations must be positive", file=sys.stderr)
        return 1

    try:
        calibration = load_calibration(args.calibration).scaled_to(*STILL_SIZE)
    except (OSError, ValueError) as exc:
        print(f"Could not load the camera calibration: {exc}", file=sys.stderr)
        return 1
    focal_x = float(calibration.camera_matrix[0, 0])
    principal_x = float(calibration.camera_matrix[0, 2])
    print(
        f"Calibration: fx={focal_x:.1f}px cx={principal_x:.1f}px at {STILL_SIZE[0]}x{STILL_SIZE[1]}"
    )

    from RPi import GPIO  # noqa: F401 — imported for parity with the other motor scripts

    answer = input("Operator beside the car, path clear, power ready to cut? (yes/no) ").strip()
    if answer.lower() != "yes":
        print("Re-run when an operator is ready beside the car.")
        return 1

    from carbot import Car, NeZhaError

    try:
        car = Car()
    except NeZhaError as exc:
        print(f"Connection failed: {exc}")
        print("Run `examples/other/01_i2c_probe.py` first to debug the link.")
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

    args.out_dir.mkdir(parents=True, exist_ok=True)
    samples: list[SpinSample] = []
    try:
        scene = args.out_dir / "scene.jpg"
        camera.capture_file(str(scene))
        quality = assess_file(str(scene))
        print(f"Scene: {quality.summary()}")
        if quality.textured_tiles < 4:
            print(
                "The camera is facing a nearly featureless surface — point the car at a "
                "textured part of the room and re-run, or the measurements will be rejected.",
                file=sys.stderr,
            )
            return 1

        print(
            f"\nMeasuring {len(args.durations)} durations x {args.repeats} repeats "
            f"at speed {args.speed}"
        )
        print("=" * 72)
        for index, duration in enumerate(args.durations):
            for repeat in range(args.repeats):
                # Alternate so the car oscillates about its starting heading.
                direction = "right" if (index + repeat) % 2 == 0 else "left"
                sample = SpinSample(duration_s=duration, direction=direction)
                before = args.out_dir / f"d{duration:.2f}-r{repeat}-a.jpg"
                after = args.out_dir / f"d{duration:.2f}-r{repeat}-b.jpg"

                camera.capture_file(str(before))
                if direction == "right":
                    car.spin_right(args.speed)
                else:
                    car.spin_left(args.speed)
                time.sleep(duration)
                car.stop()
                time.sleep(args.settle_s)
                camera.capture_file(str(after))

                estimate = estimate_yaw_between_files(str(before), str(after), focal_x, principal_x)
                if estimate is None:
                    sample.note = "no usable matches (too little overlap?)"
                else:
                    sample.measured_deg = estimate.yaw_deg
                    sample.matches = estimate.matches
                    sample.spread_deg = estimate.spread_deg
                    sample.trustworthy = estimate.trustworthy
                    if not estimate.trustworthy:
                        sample.note = (
                            f"weak: {estimate.matches} matches, spread "
                            f"{estimate.spread_deg:.1f} deg"
                        )
                    elif not sample.direction_agrees:
                        sample.note = f"spin_{direction} rotated the other way"
                samples.append(sample)

                measured = (
                    f"{sample.measured_deg:+6.1f} deg"
                    if sample.measured_deg is not None
                    else "      -   "
                )
                print(
                    f"  {duration:.2f}s {direction:5s} -> {measured} "
                    f"({sample.matches:4d} matches, spread {sample.spread_deg:.1f}) "
                    f"{sample.note}"
                )
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        car.stop()
        camera.stop()
        camera.close()
        car.close()

    print("-" * 72)
    print(f"{'duration':>9s} {'commanded':>10s} {'measured':>10s} {'deg/s':>8s} {'matches':>8s}")
    for sample in samples:
        if sample.measured_deg is None:
            print(
                f"{sample.duration_s:8.2f}s {sample.direction:>10s} {'-':>10s} "
                f"{'-':>8s} {sample.matches:8d}"
            )
            continue
        naive = PREVIOUS_ASSUMPTION_DEG_PER_S * sample.duration_s
        rate = abs(sample.measured_deg) / sample.duration_s
        flag = "" if sample.trustworthy else "  (rejected)"
        print(
            f"{sample.duration_s:8.2f}s {naive:9.1f}d {sample.measured_deg:+9.1f}d "
            f"{rate:8.1f} {sample.matches:8d}{flag}"
        )

    print("-" * 72)
    checked = [s for s in samples if s.trustworthy and s.direction_agrees is not None]
    if checked:
        agreeing = sum(1 for s in checked if s.direction_agrees)
        if agreeing == len(checked):
            print(
                f"Direction: spin_left/spin_right match the chassis on all {len(checked)} "
                f"trusted measurements — the config.py wheel mapping is correct."
            )
        elif agreeing == 0:
            print(
                f"Direction: spin_left/spin_right are INVERTED on all {len(checked)} trusted "
                f"measurements — spin_right rotates the car left. Check "
                f"config.INVERTED_MOTORS against examples/other/02_motor_check.py."
            )
        else:
            print(
                f"Direction: inconsistent — {agreeing}/{len(checked)} measurements agreed with "
                f"the command. Do not trust either direction until this is resolved."
            )

    fit = _fit_rate_and_dead_time(samples)
    if fit is None:
        print("Not enough trustworthy measurements at two or more durations to fit a rate.")
        print("Point the car at more texture, raise --repeats, or check that it is spinning.")
        return 1
    rate, dead_time = fit
    print(f"Fitted: angle = {rate:.1f} deg/s x (duration - {dead_time:.3f}s) at speed {args.speed}")
    print(
        f"Previously assumed {PREVIOUS_ASSUMPTION_DEG_PER_S:.1f} deg/s with no dead time "
        f"(measured at speed 150)."
    )
    if dead_time > 0:
        needed = args.target_deg / rate + dead_time
        print(
            f"A {args.target_deg:.0f} deg step needs {needed:.3f}s of spin; ignoring the "
            f"dead time would command {args.target_deg / rate:.3f}s and undershoot by "
            f"about {rate * dead_time:.1f} deg."
        )
    else:
        needed = args.target_deg / rate
        print(
            f"No positive dead time was measured; a {args.target_deg:.0f} deg step needs "
            f"{needed:.3f}s."
        )
    print(f"Frames under {args.out_dir}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

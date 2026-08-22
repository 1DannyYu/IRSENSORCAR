#!/usr/bin/env python3
"""Autonomous patrol that avoids what the sonar cannot see, capturing for SfM.

A single forward HC-SR04 sees neither thin chair legs nor an overhead tabletop,
so `examples/17` drove under a chair and stuck on its underside. This patrol
fuses the IMX500's on-sensor object detection with the sonar through
:func:`carbot.vision_avoid.fuse`, and it captures 2028x1520 stills for COLMAP
from the **same camera configuration** — mode 'single', measured in
`docs/progress/2026-08-14-camera-modes-exposure-and-preflight-fix.md`, needs no
mode switching at all.

Per station: read the sonar and the latest detections, fuse them, then either
back up and turn away, or advance one short step. Every rotation is then stepped
in ~15 deg increments with a capture attempt at each stop — the avoidance turn
and the standing burst are the same sweep, differing only in how far they go and
whether the car rotates back afterwards.

This shape came from three hardware runs. Single frames per station did not
reconstruct: ten individually good frames (all sharp, all well exposed) formed
four disconnected islands when matched pairwise — 600-1500 matches within an
island, 10-40 between — and COLMAP registered 3 of 10, reporting "no good initial
image pair found". Adding a burst raised that to 17 of 30, and the remaining
breaks all sat on avoidance turns, which were dead time. Photographing through
the turn bridges those headings, since a 15 deg step keeps ~77% of the 66 deg
field of view in common, the upper end COLMAP wants indoors.

A weak link is also repaired during the run rather than discovered later on the
workstation: when a capture shares too few matches with the previous one, the car
rotates halfway back and inserts a bridging frame. That check is the same
`repeatable_keypoints` measurement that diagnosed the problem offline.

**The car has to cover ground, and it barely did.** Scale-anchoring a 30-frame
run put the entire trajectory inside 13 cm. Measured travel explains it: 0.117
m/s at speed 200, with reverse within 5% of forward, so a 0.6 s avoidance backup
handed back most of a 1.0 s step. Raising the PWM is a weak lever — doubling it
to 400 bought only 1.42x the speed — so the step lengthened instead, and to keep
a long step safe the forward travel is driven in segments with the fuse
re-evaluated between them. Turns still run at the calibrated `--spin-speed`,
because the 53.5 deg/s figure belongs to speed 200 and nothing else.

Each shot is also gated on being worth keeping. That gate exists because the
first supervised run kept frames shot 30 cm
from a whiteboard: sharp, correctly exposed, and 70% blank panel. Nothing was
blurred; the frames were simply useless to COLMAP. A capture is now rejected
when the sonar says the camera has no room in front (``--min-standoff-cm``) or
when :mod:`carbot.frame_quality` finds too few textured tiles. Rejected poses
cost a step, not a frame, so the saved set stays contiguous and every file in it
is worth matching. ``--max-steps`` bounds the run when a room rejects most poses.

The avoidance threshold is deliberately *not* raised to the standoff distance: a
45 cm turn threshold is what made an earlier patrol spin in place.

**Motor-moving.** An operator must stand beside the car able to cut main power
instantly. Run `examples/other/14_all_sensors_preflight_check.py` first.

    # stationary logic check — reads sensors, never drives
    PYTHONPATH=src python3 examples/ai_camera/22_cam_sonar_patrol_capture.py --dry-run --frames 10

    # supervised run
    PYTHONPATH=src python3 examples/ai_camera/22_cam_sonar_patrol_capture.py --frames 10
    PYTHONPATH=src python3 examples/ai_camera/22_cam_sonar_patrol_capture.py --frames 150

Exposure defaults to `auto`. The stationary sweep in `examples/21` preferred spot
metering, but a patrol turns to face every direction, and spot metering blew out
43% of one frame when the car pointed at a window — the sweep never tested that
because the camera never moved. `spot` and `long-shutter-spot` remain available
for comparison. The 97 ms shutter blur risk that sweep left open is *resolved*:
at 38 ms with a 1.0 s settle, captures came back sharp while moving.
"""

from __future__ import annotations

import argparse
import random
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from carbot.frame_quality import assess_file, repeatable_keypoints_between_files
from carbot.sonar import Sonar
from carbot.vision_avoid import ObstaclePolicy, detections_from_metadata, fuse

TRIG_PIN = 17  # GPIO 17 (Pin 11)
ECHO_PIN = 27  # GPIO 27 (Pin 13)
# Measured at speed 200 by examples/ai_camera/23_cam_spin_rate_check.py: 53.5 deg/s with a
# startup dead time of 0.005 s, i.e. none worth compensating. The older 43.9
# deg/s came from speed 150 and made every commanded turn ~22% too large.
VERIFIED_SPIN_DEG_PER_S = 53.5
VERIFIED_AT_SPEED = 200
DEFAULT_MODEL = "/usr/share/imx500-models/imx500_network_ssd_mobilenetv2_fpnlite_320x320_pp.rpk"
STILL_SIZE = (2028, 1520)
LONG_FRAME_DURATION_US = 100_000

# Exposure presets, measured in examples/21. 'long-shutter-spot' won on
# repeatable keypoints at the lowest gain, but only with the camera stationary.
EXPOSURE_PRESETS = ("auto", "spot", "long-shutter-spot")


def _exposure_controls(preset: str) -> dict[str, object]:
    from libcamera import controls as libcamera_controls

    if preset == "auto":
        return {}
    spot = {"AeMeteringMode": libcamera_controls.AeMeteringModeEnum.Spot}
    if preset == "spot":
        return spot
    return {
        **spot,
        "FrameDurationLimits": (LONG_FRAME_DURATION_US, LONG_FRAME_DURATION_US),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="Vision + sonar fused patrol with SfM capture")
    parser.add_argument("--frames", type=int, default=150, help="number of stills to capture")
    parser.add_argument(
        "--dry-run", action="store_true", help="read sensors and fuse, but never drive the motors"
    )
    parser.add_argument(
        "--step-s",
        type=float,
        default=3.0,
        help="seconds of forward travel per station, driven in segments",
    )
    parser.add_argument(
        "--sense-interval-s",
        type=float,
        default=0.5,
        help="re-check sonar and detections after this much forward travel",
    )
    parser.add_argument("--speed", type=int, default=400, help="drive speed 0-1000")
    parser.add_argument(
        "--spin-speed",
        type=int,
        default=VERIFIED_AT_SPEED,
        help=f"speed used for turns; {VERIFIED_SPIN_DEG_PER_S} deg/s was "
        f"calibrated at {VERIFIED_AT_SPEED}, so raising it invalidates "
        f"--spin-deg-per-s",
    )
    parser.add_argument(
        "--backup-s",
        type=float,
        default=0.3,
        help="seconds to reverse before turning (frees the car from a corner)",
    )
    parser.add_argument(
        "--obstacle-cm",
        type=float,
        default=30.0,
        help="sonar distance below which the car turns away",
    )
    parser.add_argument("--turn-min-deg", type=float, default=30.0)
    parser.add_argument("--turn-max-deg", type=float, default=150.0)
    parser.add_argument(
        "--spin-deg-per-s",
        type=float,
        default=VERIFIED_SPIN_DEG_PER_S,
        help=f"spin rate (deg/s); measured {VERIFIED_SPIN_DEG_PER_S} at speed "
        f"{VERIFIED_AT_SPEED}, re-measure with examples/23 for other speeds",
    )
    parser.add_argument(
        "--burst-frames",
        type=int,
        default=5,
        help="overlapping frames to capture per station (1 = no burst)",
    )
    parser.add_argument(
        "--burst-step-deg",
        type=float,
        default=15.0,
        help="rotation between frames; 15 deg keeps ~77%% of the 66 deg "
        "field of view in common, the upper end COLMAP wants indoors",
    )
    parser.add_argument(
        "--min-overlap-matches",
        type=int,
        default=200,
        help="insert a bridging frame when a capture shares fewer matches "
        "with the previous one; 0 disables the check",
    )
    parser.add_argument(
        "--settle-s",
        type=float,
        default=1.0,
        help="seconds to let the chassis stop rocking before a capture",
    )
    parser.add_argument(
        "--threshold", type=float, default=0.30, help="detection confidence threshold"
    )
    parser.add_argument(
        "--exposure",
        choices=EXPOSURE_PRESETS,
        default="auto",
        help="auto-exposure preset (see examples/21)",
    )
    parser.add_argument(
        "--min-standoff-cm",
        type=float,
        default=30.0,
        help="do not photograph when the nearest surface is closer than this; "
        "keep at or below --obstacle-cm to avoid a no-capture dead band",
    )
    parser.add_argument(
        "--min-textured-tiles",
        type=int,
        default=6,
        help="reject a capture with fewer textured tiles (of 12)",
    )
    parser.add_argument(
        "--max-steps",
        type=int,
        default=0,
        help="stop after this many steps even if --frames is unmet (default: 4x --frames)",
    )
    parser.add_argument(
        "--keep-rejected",
        action="store_true",
        help="save rejected captures under <out-dir>/rejected for calibration",
    )
    parser.add_argument(
        "--frame-report",
        action="store_true",
        help="print the SfM quality of every accepted capture",
    )
    parser.add_argument("--model", default=DEFAULT_MODEL)
    parser.add_argument("--out-dir", type=Path, default=Path("/tmp/room-sfm"))
    args = parser.parse_args()

    if args.turn_min_deg > args.turn_max_deg:
        print("--turn-min-deg must not exceed --turn-max-deg", file=sys.stderr)
        return 1

    policy = ObstaclePolicy(confidence_threshold=args.threshold, sonar_stop_cm=args.obstacle_cm)

    from RPi import GPIO

    GPIO.setmode(GPIO.BCM)
    GPIO.setup(TRIG_PIN, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(ECHO_PIN, GPIO.IN)
    sonar = Sonar(TRIG_PIN, ECHO_PIN, GPIO)

    if not args.dry_run:
        answer = input("Operator beside the car, path clear, power ready to cut? (yes/no) ").strip()
        if answer.lower() != "yes":
            print("Re-run when an operator is ready beside the car.")
            GPIO.cleanup()
            return 1

    car = None
    if not args.dry_run:
        from carbot import Car, NeZhaError

        try:
            car = Car()
        except NeZhaError as exc:
            print(f"Connection failed: {exc}")
            print("Run `examples/other/01_i2c_probe.py` first to debug the link.")
            GPIO.cleanup()
            return 1

    from picamera2 import Picamera2
    from picamera2.devices import IMX500
    from picamera2.devices.imx500 import NetworkIntrinsics

    imx500 = IMX500(args.model)
    intrinsics = imx500.network_intrinsics
    if not intrinsics:
        intrinsics = NetworkIntrinsics()
        intrinsics.task = "object detection"
    if intrinsics.task != "object detection":
        print(f"Model is not an object-detection network (task={intrinsics.task})", file=sys.stderr)
        if car:
            car.close()
        GPIO.cleanup()
        return 1
    intrinsics.update_with_defaults()
    imx500.show_network_fw_progress_bar()

    camera = Picamera2(imx500.camera_num)
    try:
        # Mode 'single': inference and the 2028x1520 still come from one
        # configuration, so there is nothing to switch between captures.
        camera.configure(
            camera.create_preview_configuration(
                main={"size": STILL_SIZE},
                controls={"FrameRate": intrinsics.inference_rate},
                buffer_count=4,
            )
        )
        camera.start()
        controls = _exposure_controls(args.exposure)
        if controls:
            camera.set_controls(controls)
        time.sleep(max(args.settle_s, 1.0))
        frame_width, frame_height = camera.camera_configuration()["main"]["size"]
    except Exception:
        camera.close()
        if car:
            car.close()
        GPIO.cleanup()
        raise

    args.out_dir.mkdir(parents=True, exist_ok=True)
    mode = "DRY RUN (no motors)" if args.dry_run else f"driving at speed {args.speed}"
    print(f"Fused patrol: {args.frames} frames, {mode}, exposure={args.exposure}")
    print(
        f"stop below {args.obstacle_cm:.0f} cm or on a central-low detection "
        f">={args.threshold:.2f}; turn {args.turn_min_deg:.0f}-{args.turn_max_deg:.0f} deg "
        f"after backing up {args.backup_s:.1f}s. Ctrl-C to stop."
    )
    print(
        f"forward {args.step_s:.1f}s per station, re-sensing every "
        f"{args.sense_interval_s:.1f}s; turns at speed {args.spin_speed} "
        f"({args.spin_deg_per_s:.1f} deg/s)"
    )
    print("=" * 72)

    max_steps = args.max_steps if args.max_steps > 0 else args.frames * 4
    rejected_dir = args.out_dir / "rejected"
    if args.keep_rejected:
        rejected_dir.mkdir(parents=True, exist_ok=True)
    pending = args.out_dir / "pending.jpg"

    ctx = PatrolContext(
        args=args,
        car=car,
        camera=camera,
        sonar=sonar,
        pending=pending,
        rejected_dir=rejected_dir,
        imx500=imx500,
        intrinsics=intrinsics,
        policy=policy,
        frame_size=(frame_width, frame_height),
    )

    blocked_count = 0
    empty_sweeps = 0
    step = 0
    try:
        while ctx.kept < args.frames and step < max_steps:
            step += 1
            distance = sonar.measure_nearest()
            metadata = camera.capture_metadata()
            detections = detections_from_metadata(metadata, imx500, intrinsics, camera, policy)
            verdict = fuse(distance, detections, (frame_width, frame_height), policy)

            if verdict.blocked:
                blocked_count += 1
                angle = random.uniform(args.turn_min_deg, args.turn_max_deg)
                direction = "left" if random.random() < 0.5 else "right"
                print(f"[{step}] BLOCKED {verdict.reason} -> back up, {direction} {angle:.0f} deg")
                if car:
                    # Back up first: a chassis wedged in a corner physically
                    # cannot spin in place.
                    car.backward(args.speed)
                    time.sleep(args.backup_s)
                    car.stop()
                    time.sleep(0.3)
                # The avoidance turn IS the station's sweep. Leaving it as dead
                # time is what broke the reconstruction into pieces: every model
                # boundary in the last run sat on one of these turns. Stepping it
                # and shooting on the way round bridges the two headings instead.
                kept_here = _sweep_and_capture(ctx, angle, direction, "turn")
            else:
                driven, stopped_early = _advance(ctx)
                if stopped_early:
                    print(
                        f"[{step}] clear   {verdict.reason} -> forward {driven:.1f}s of "
                        f"{args.step_s:.1f}s, stopped: {stopped_early}"
                    )
                else:
                    print(f"[{step}] clear   {verdict.reason} -> forward {driven:.1f}s")
                if car:
                    time.sleep(args.settle_s)
                span = args.burst_step_deg * (max(1, args.burst_frames) - 1)
                kept_here = _sweep_and_capture(ctx, span, "right", "burst")
                # Undo the sweep so the next forward move continues along the
                # previous heading. A straight run is what linked frames across
                # stations, so a burst must not quietly re-aim the car. An
                # avoidance turn is exempt: changing heading is its whole point.
                _spin(ctx, span, "left")

            if kept_here == 0:
                empty_sweeps += 1
                print("      sweep kept nothing — this station is unusable")
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        if car:
            car.stop()
        camera.stop()
        camera.close()
        if car:
            car.close()
        GPIO.cleanup()
        pending.unlink(missing_ok=True)

    print("-" * 72)
    print(
        f"Kept {ctx.kept} frames -> {args.out_dir} in {step} stations "
        f"({blocked_count} blocked, {step - blocked_count} forward, "
        f"{ctx.rejected} captures rejected, {empty_sweeps} empty sweeps)"
    )
    for key, count in sorted(ctx.reject_reasons.items(), key=lambda kv: -kv[1]):
        print(f"  rejected {count}x: {key}")
    if ctx.overlaps:
        weak = sum(1 for v in ctx.overlaps if v < args.min_overlap_matches)
        print(
            f"Overlap with the previous kept frame: min {min(ctx.overlaps)}, "
            f"median {sorted(ctx.overlaps)[len(ctx.overlaps) // 2]}, "
            f"max {max(ctx.overlaps)} — {weak} below {args.min_overlap_matches}, "
            f"{ctx.bridges} bridge frames inserted"
        )
    if ctx.kept < args.frames:
        print(
            f"Stopped at the {max_steps}-step cap with {ctx.kept}/{args.frames} frames — the "
            f"room is rejecting most poses. Check the reject reasons before raising "
            f"--max-steps."
        )
    if args.dry_run:
        print("Dry run: no motor commands were sent.")
    return 0


@dataclass
class PatrolContext:
    """State the capture helpers share, so each takes one argument instead of ten."""

    args: Any
    car: Any  # None during a dry run
    camera: Any
    sonar: Sonar
    pending: Path
    rejected_dir: Path
    imx500: Any = None
    intrinsics: Any = None
    policy: ObstaclePolicy | None = None
    frame_size: tuple[int, int] = (0, 0)
    kept: int = 0
    last_kept: Path | None = None
    rejected: int = 0
    bridges: int = 0
    overlaps: list[int] = field(default_factory=list)
    reject_reasons: dict[str, int] = field(default_factory=dict)


def _spin(ctx: PatrolContext, degrees: float, direction: str) -> None:
    """Rotate and settle. A no-op during a dry run.

    Turns run at ``--spin-speed``, not the drive speed: 53.5 deg/s was measured
    at speed 200, and driving faster than that would silently invalidate every
    commanded angle. Turning accuracy matters here, turning speed does not.
    """
    if not ctx.car or degrees <= 0:
        return
    spin = ctx.car.spin_left if direction == "left" else ctx.car.spin_right
    spin(ctx.args.spin_speed)
    time.sleep(degrees / ctx.args.spin_deg_per_s)
    ctx.car.stop()
    time.sleep(ctx.args.settle_s)


def _sense(ctx: PatrolContext):
    """One fused sonar + vision verdict at the current pose."""
    distance = ctx.sonar.measure_nearest()
    metadata = ctx.camera.capture_metadata()
    detections = detections_from_metadata(
        metadata, ctx.imx500, ctx.intrinsics, ctx.camera, ctx.policy
    )
    return fuse(distance, detections, ctx.frame_size, ctx.policy)


def _advance(ctx: PatrolContext) -> tuple[float, str | None]:
    """Drive forward in segments, re-sensing between them.

    Long steps are what make the patrol cover ground. Measured at speed 400 the
    car travels 0.166 m/s, so the old 1.0 s step advanced 17 cm and a 0.6 s
    avoidance backup handed almost all of it back — an entire 30-frame run
    finished inside 13 cm. But driving 3 s blind covers half a metre while the
    obstacle threshold is 30 cm, so the step is split and the fuse re-evaluated
    between segments. Anything that appears stops the remaining travel at once.

    Returns the seconds actually driven and the reason travel stopped early.
    """
    args = ctx.args
    segment = max(0.05, min(args.sense_interval_s, args.step_s))
    driven = 0.0
    while driven < args.step_s - 1e-6:
        this = min(segment, args.step_s - driven)
        if ctx.car:
            ctx.car.forward(args.speed)
            time.sleep(this)
            ctx.car.stop()
        driven += this
        if driven >= args.step_s - 1e-6:
            break
        verdict = _sense(ctx)
        if verdict.blocked:
            return driven, verdict.reason
    return driven, None


def _sweep_and_capture(ctx: PatrolContext, total_deg: float, direction: str, tag: str) -> int:
    """Rotate through ``total_deg`` in steps, trying to capture at every stop.

    Both the burst and the avoidance turn are this same primitive; they differ
    only in how far they rotate and whether the caller rotates back afterwards.
    Returns how many frames were kept.
    """
    args = ctx.args
    step_deg = args.burst_step_deg
    stops = max(1, round(total_deg / step_deg) + 1) if total_deg > 0 else 1
    kept_here = 0
    for index in range(stops):
        if ctx.kept >= args.frames:
            break
        if index > 0:
            _spin(ctx, step_deg, direction)
        shot = _try_capture(ctx, f"[{tag} {index + 1}/{stops}]")
        if not shot.kept:
            continue
        kept_here += 1
        # Loop A: a weak link is repaired on the spot rather than discovered
        # minutes later on the workstation. Rotating halfway back puts a frame
        # between the two headings, which overlaps both.
        if (
            shot.overlap is not None
            and shot.overlap < args.min_overlap_matches
            and ctx.kept < args.frames  # a bridge is still a frame, and must not overrun
        ):
            print(f"      weak link ({shot.overlap} matches) -> bridging")
            back = "left" if direction == "right" else "right"
            _spin(ctx, step_deg / 2, back)
            if _try_capture(ctx, f"[{tag} bridge]").kept:
                ctx.bridges += 1
                kept_here += 1
            _spin(ctx, step_deg / 2, direction)
    return kept_here


@dataclass
class ShotResult:
    """Whether a capture was kept, and how well it linked to the previous one."""

    kept: bool
    overlap: int | None = None


def _try_capture(ctx: PatrolContext, label: str) -> ShotResult:
    """Run the gates at the current pose and keep the frame if they all pass."""
    args = ctx.args
    # Re-checked at every stop: rotating changes what the camera and the sonar
    # are pointed at, so the verdict changes with them.
    reason = _standoff_reason(ctx.sonar, args.min_standoff_cm)
    quality = None
    if reason is None:
        ctx.camera.capture_file(str(ctx.pending))
        quality, reason = _quality_reason(ctx.pending, args.min_textured_tiles)
    elif args.keep_rejected:
        # Shoot anyway, purely to record what the standoff gate skipped. Without
        # this the flag saves nothing when standoff is doing all the rejecting,
        # which is exactly when the threshold needs calibrating.
        ctx.camera.capture_file(str(ctx.pending))

    if reason is not None:
        ctx.rejected += 1
        key = reason.split(":")[0]
        ctx.reject_reasons[key] = ctx.reject_reasons.get(key, 0) + 1
        print(f"      no capture {label}: {reason}")
        if args.keep_rejected and ctx.pending.exists():
            ctx.pending.replace(ctx.rejected_dir / f"reject-{ctx.rejected:03d}.jpg")
        return ShotResult(kept=False)

    overlap = _overlap(ctx.last_kept, ctx.pending) if args.min_overlap_matches > 0 else None
    path = args.out_dir / f"frame-{ctx.kept:03d}.jpg"
    ctx.pending.replace(path)
    ctx.kept += 1
    ctx.last_kept = path
    if overlap is not None:
        ctx.overlaps.append(overlap)

    line = f"      {path.name} {label}"
    if args.frame_report and quality:
        line += f": {quality.summary()}"
    else:
        line += f" kept ({ctx.kept}/{args.frames})"
    if overlap is not None:
        line += f" overlap={overlap}"
    print(line)
    return ShotResult(kept=True, overlap=overlap)


def _overlap(previous: Path | None, current: Path) -> int | None:
    """Matches shared with the previously kept frame, or None when there is none.

    This is the measurement the workstation used to diagnose why three runs of
    good-looking frames would not reconstruct. Running it on the Pi is what turns
    that diagnosis into something the car can act on during the run.
    """
    if previous is None or not previous.exists():
        return None
    try:
        return repeatable_keypoints_between_files(str(previous), str(current))
    except (RuntimeError, ValueError):
        return None


def _standoff_reason(sonar: Sonar, minimum_cm: float) -> str | None:
    """Reject a pose whose camera is pressed up against a nearby surface.

    A frame shot 30 cm from a whiteboard is 70% blank panel: sharp, correctly
    exposed, and useless to COLMAP. Two such frames were what made the first
    supervised run look like a motion-blur problem when nothing was blurred.

    The threshold started at 50 cm and was lowered to match the avoidance
    distance. Two findings converged on it. Every one of the three captures this
    gate rejected in the run that registered 30/30 would have passed the quality
    gate comfortably — that gate measures the actual failure, a lack of texture
    to match, while standoff only proxies it. And a standoff above the avoidance
    threshold opens a dead band: with something 30-50 cm ahead the sonar rule
    does not require the car to move away, yet nothing may be photographed, so
    the patrol can sit unable to shoot and unobliged to leave. A dry run in that
    state produced 107 consecutive rejections.
    """
    distance = sonar.measure_nearest()
    if distance is None:
        return "standoff: no sonar reading, cannot confirm the camera has room"
    if distance < minimum_cm:
        # One decimal: rounding to whole cm printed "50 cm < 50 cm", which reads
        # as a broken comparison rather than a 49.6 cm reading.
        return f"standoff: {distance:.1f} cm < {minimum_cm:.1f} cm, too close to a surface"
    return None


def _quality_reason(path: Path, min_textured_tiles: int):
    """Assess a capture; return ``(quality, reason)`` with reason None when usable.

    Only the textured-tile count gates a capture, deliberately. `QualityPolicy`
    measures five other things and they are printed with ``--frame-report``, but
    gating on them is not supported by evidence: in the run that registered
    30 of 30 frames, the softest capture scored 19 sharpness — under the old
    ``min_sharpness`` — and reconstructed normally. Exposure and sharpness
    describe how a frame looks; only the keypoint spread relates to whether
    Structure-from-Motion has anything to match.
    """
    try:
        quality = assess_file(str(path))
    except (RuntimeError, ValueError) as exc:
        return None, f"quality: unavailable ({exc})"
    if quality.textured_tiles < min_textured_tiles:
        return quality, (
            f"quality: only {quality.textured_tiles}/{quality.total_tiles} textured tiles "
            f"(need {min_textured_tiles}), {quality.keypoints} keypoints"
        )
    return quality, None


if __name__ == "__main__":
    sys.exit(main())

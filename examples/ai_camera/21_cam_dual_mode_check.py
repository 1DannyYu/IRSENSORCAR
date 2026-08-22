#!/usr/bin/env python3
"""IMX500 capture-mode and exposure experiments for the fused patrol (no motors).

Two questions have to be answered before the patrol is worth writing, and both
are camera-only.

**1. Can one camera do inference and 2028x1520 stills?** The patrol needs
on-sensor detection to avoid the chairs the sonar cannot see, and sharp
high-resolution stills for COLMAP. Three ways to get both:

  single   one high-resolution config; if the CNN output tensor still arrives at
           main=2028x1520 there is no mode switching at all
  switch   preview/inference config + ``switch_mode_and_capture_file`` for the
           still, which switches back on its own
  restart  ``stop()`` / ``configure()`` / ``start()`` around the still — the
           fallback if a mode switch drops the loaded network

Per mode it reports whether inference frames carry a tensor, the fused verdict
from :mod:`carbot.vision_avoid` (so the ported detection adapter is validated
against the same hardware that verified `examples/20`), the captured image
dimensions, a full SfM-usability report from :mod:`carbot.frame_quality`, and
how long the capture and the inference-resume took.

**2. Which auto-exposure setting produces SfM-usable frames?** A measured
capture came out at mean brightness 70/255 with 4.9% of pixels crushed to black,
because a bright window dragged the interior down. Dark pixels carry no
keypoints, so exposure decides how much of the frame COLMAP can use. The sweep
compares EV compensation against shadow-priority and centre/spot metering, and
reports the shutter each one settles on — brightness bought with a long shutter
trades the blur straight back in once the car is only briefly settled.

No motors, no servos, no GPIO — safe to run over SSH:

    PYTHONPATH=src python3 examples/ai_camera/21_cam_dual_mode_check.py
    PYTHONPATH=src python3 examples/ai_camera/21_cam_dual_mode_check.py --check exposure
    PYTHONPATH=src python3 examples/ai_camera/21_cam_dual_mode_check.py --check modes --mode single

The sonar is not read here; the fused verdict uses a simulated clear reading so
the printed decision reflects vision alone.
"""

from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from carbot.frame_quality import FrameQuality, assess_file, repeatable_keypoints_between_files
from carbot.vision_avoid import ObstaclePolicy, detections_from_metadata, fuse

DEFAULT_MODEL = "/usr/share/imx500-models/imx500_network_ssd_mobilenetv2_fpnlite_320x320_pp.rpk"
STILL_SIZE = (2028, 1520)
PREVIEW_SIZE = (640, 480)
SIMULATED_CLEAR_SONAR_CM = 100.0
MODES = ("single", "switch", "restart")
# 'default' measured a second time at the end of the sweep, as a control for
# run-to-run variation in the same scene.
CONTROL_NAME = "default (control)"


@dataclass
class ModeResult:
    """One mode's measurements, or the error that stopped it."""

    name: str
    frames_read: int = 0
    frames_with_tensor: int = 0
    detections: int = 0
    verdict: str = "-"
    capture_bytes: int = 0
    quality: FrameQuality | None = None
    capture_s: float = 0.0
    resume_s: float | None = None
    notes: list[str] = field(default_factory=list)
    error: str | None = None

    @property
    def inference_ok(self) -> bool:
        return self.frames_with_tensor > 0

    @property
    def capture_size(self) -> tuple[int, int] | None:
        return (self.quality.width, self.quality.height) if self.quality else None

    @property
    def still_ok(self) -> bool:
        return self.capture_size == STILL_SIZE


def _assess_capture(path: Path) -> FrameQuality | None:
    """Full SfM-usability report for a capture, or None when OpenCV is absent.

    Global sharpness alone is misleading here: the first measured capture scored
    a blur-like 40 while its furniture tiles scored 127-139 — the low number came
    from a blank wall filling half the frame, not from soft focus. See
    :mod:`carbot.frame_quality`.
    """
    try:
        return assess_file(str(path))
    except (RuntimeError, ValueError):
        return None


def _read_inference(
    picam2,
    imx500,
    intrinsics,
    frames: int,
    policy: ObstaclePolicy,
    result: ModeResult,
) -> None:
    """Read ``frames`` metadata frames and fold them into ``result``."""
    frame_size = tuple(picam2.camera_configuration()["main"]["size"])
    for _ in range(frames):
        metadata = picam2.capture_metadata()
        result.frames_read += 1
        detections = detections_from_metadata(metadata, imx500, intrinsics, picam2, policy)
        if imx500.get_outputs(metadata, add_batch=True) is not None:
            result.frames_with_tensor += 1
        if detections:
            result.detections += len(detections)
            verdict = fuse(SIMULATED_CLEAR_SONAR_CM, detections, frame_size, policy)
            result.verdict = verdict.reason


def _wait_for_inference(picam2, imx500, timeout_s: float) -> float | None:
    """Seconds until a frame carries a CNN tensor again, or None on timeout."""
    start = time.monotonic()
    while time.monotonic() - start < timeout_s:
        metadata = picam2.capture_metadata()
        if imx500.get_outputs(metadata, add_batch=True) is not None:
            return time.monotonic() - start
    return None


def _record_capture(result: ModeResult, path: Path, elapsed: float) -> None:
    result.capture_s = elapsed
    if not path.exists():
        result.error = "capture produced no file"
        return
    result.capture_bytes = path.stat().st_size
    result.quality = _assess_capture(path)
    if result.quality is None:
        result.notes.append("OpenCV unavailable — no frame-quality report")


def _run_single(picam2, imx500, intrinsics, args, policy, out_dir) -> ModeResult:
    """One config at 2028x1520: no mode switch if the tensor still arrives."""
    result = ModeResult("single")
    config = picam2.create_preview_configuration(
        main={"size": STILL_SIZE},
        controls={"FrameRate": intrinsics.inference_rate},
        buffer_count=4,
    )
    if picam2.started:
        picam2.stop()
    picam2.configure(config)
    picam2.start()
    time.sleep(args.settle)
    result.notes.append(f"main={tuple(picam2.camera_configuration()['main']['size'])}")

    _read_inference(picam2, imx500, intrinsics, args.frames, policy, result)
    path = out_dir / "mode-single.jpg"
    start = time.monotonic()
    picam2.capture_file(str(path))
    _record_capture(result, path, time.monotonic() - start)
    # No switch happened, so inference cannot have been interrupted.
    result.resume_s = 0.0
    return result


def _run_switch(picam2, imx500, intrinsics, args, policy, out_dir) -> ModeResult:
    """Preview/inference config, with switch_mode_and_capture_file for the still."""
    result = ModeResult("switch")
    preview = picam2.create_preview_configuration(
        main={"size": PREVIEW_SIZE},
        controls={"FrameRate": intrinsics.inference_rate},
        buffer_count=12,
    )
    still = picam2.create_still_configuration(main={"size": STILL_SIZE})
    if picam2.started:
        picam2.stop()
    picam2.configure(preview)
    picam2.start()
    time.sleep(args.settle)

    _read_inference(picam2, imx500, intrinsics, args.frames, policy, result)
    path = out_dir / "mode-switch.jpg"
    start = time.monotonic()
    picam2.switch_mode_and_capture_file(still, str(path))
    _record_capture(result, path, time.monotonic() - start)
    result.resume_s = _wait_for_inference(picam2, imx500, args.resume_timeout)
    if result.resume_s is None:
        result.notes.append(f"no tensor within {args.resume_timeout:.0f}s of switching back")
    return result


def _run_restart(picam2, imx500, intrinsics, args, policy, out_dir) -> ModeResult:
    """Full stop/configure/start around the still — the heaviest fallback."""
    result = ModeResult("restart")
    preview = picam2.create_preview_configuration(
        main={"size": PREVIEW_SIZE},
        controls={"FrameRate": intrinsics.inference_rate},
        buffer_count=12,
    )
    still = picam2.create_still_configuration(main={"size": STILL_SIZE})
    if picam2.started:
        picam2.stop()
    picam2.configure(preview)
    picam2.start()
    time.sleep(args.settle)

    _read_inference(picam2, imx500, intrinsics, args.frames, policy, result)
    path = out_dir / "mode-restart.jpg"
    start = time.monotonic()
    picam2.stop()
    picam2.configure(still)
    picam2.start()
    time.sleep(args.settle)
    picam2.capture_file(str(path))
    picam2.stop()
    picam2.configure(preview)
    picam2.start()
    _record_capture(result, path, time.monotonic() - start)
    result.resume_s = _wait_for_inference(picam2, imx500, args.resume_timeout)
    if result.resume_s is None:
        result.notes.append(f"no tensor within {args.resume_timeout:.0f}s of restarting")
    return result


RUNNERS = {"single": _run_single, "switch": _run_switch, "restart": _run_restart}


def _print_table(results: list[ModeResult]) -> None:
    header = (
        f"{'mode':8s} {'tensor':>7s} {'dets':>5s} {'still':>11s} {'sharp':>6s} "
        f"{'bright':>7s} {'textured':>9s} {'capture':>8s} {'resume':>7s}"
    )
    print(header)
    print("-" * len(header))
    for r in results:
        if r.error and not r.frames_read:
            print(f"{r.name:8s} ERROR: {r.error}")
            continue
        q = r.quality
        tensor = f"{r.frames_with_tensor}/{r.frames_read}"
        size = "x".join(str(v) for v in r.capture_size) if r.capture_size else "-"
        sharp = f"{q.sharpness:.0f}" if q else "-"
        bright = f"{q.mean_brightness:.0f}" if q else "-"
        textured = f"{q.textured_tiles}/{q.total_tiles}" if q else "-"
        resume = f"{r.resume_s:.2f}s" if r.resume_s is not None else "TIMEOUT"
        print(
            f"{r.name:8s} {tensor:>7s} {r.detections:5d} {size:>11s} {sharp:>6s} "
            f"{bright:>7s} {textured:>9s} {r.capture_s:7.2f}s {resume:>7s}"
        )
        if q and q.problems:
            for problem in q.problems:
                print(f"{'':8s} frame: {problem}")
        for note in r.notes:
            print(f"{'':8s} note: {note}")
        if r.error:
            print(f"{'':8s} error: {r.error}")


def _recommend(results: list[ModeResult]) -> str:
    usable = [r for r in results if r.inference_ok and r.still_ok and r.resume_s is not None]
    for name in MODES:  # cheapest first
        for r in usable:
            if r.name == name:
                return f"Use mode '{name}' in the fused patrol."
    return (
        "No mode delivered both inference and a 2028x1520 still. Do not write the "
        "fused patrol yet — inspect the per-mode errors above."
    )


# --------------------------------------------------------------- exposure sweep


@dataclass
class ExposureResult:
    """One auto-exposure setting's effect on SfM frame quality."""

    name: str
    controls: dict[str, Any]
    quality: FrameQuality | None = None
    repeatable: int | None = None
    exposure_time_us: int | None = None
    analogue_gain: float | None = None
    error: str | None = None

    @property
    def shutter_ms(self) -> float | None:
        return self.exposure_time_us / 1000.0 if self.exposure_time_us else None


def _exposure_settings(metering, constraint) -> tuple[ExposureResult, ...]:
    """The settings to sweep, cheapest intervention first.

    Two different fixes are in play for the measured problem — a dark interior
    against a blown-out window. ``ExposureValue`` raises the whole frame and will
    clip the window further; ``AeConstraintMode.Shadows`` and centre/spot
    metering instead stop the bright window from dragging the interior down.

    The list ends by repeating ``default`` as ``CONTROL_NAME``. Without that
    control there is no way to tell a real difference between settings from
    run-to-run variation in the same scene, and the differences here turned out
    to be small enough that the distinction decides the conclusion.
    """
    return (
        ExposureResult("default", {}),
        ExposureResult("ev+0.5", {"ExposureValue": 0.5}),
        ExposureResult("ev+1.0", {"ExposureValue": 1.0}),
        ExposureResult("ev+1.5", {"ExposureValue": 1.5}),
        ExposureResult("shadows", {"AeConstraintMode": constraint.Shadows}),
        ExposureResult(
            "shadows+ev1.0", {"AeConstraintMode": constraint.Shadows, "ExposureValue": 1.0}
        ),
        ExposureResult("centre-metered", {"AeMeteringMode": metering.CentreWeighted}),
        ExposureResult("spot-metered", {"AeMeteringMode": metering.Spot}),
        # Every setting above settles on the same shutter because FrameRate caps
        # the frame duration, so AE can only add brightness as gain — and gain is
        # the noise that inflates raw keypoint counts. Lifting the cap lets it
        # spend time instead, which is free for a car that stops before shooting.
        ExposureResult("long-shutter", {"FrameDurationLimits": (100_000, 100_000)}),
        ExposureResult(
            "long-shutter+spot",
            {"FrameDurationLimits": (100_000, 100_000), "AeMeteringMode": metering.Spot},
        ),
        ExposureResult(CONTROL_NAME, {}),
    )


def _capture_with_metadata(picam2, path: Path) -> dict[str, Any]:
    """Capture a still and return the metadata of that exact frame.

    ``capture_request`` is used instead of ``capture_file`` so the reported
    ExposureTime and AnalogueGain belong to the saved image rather than to
    whatever frame happened to follow it.
    """
    request = picam2.capture_request()
    try:
        request.save("main", str(path))
        return request.get_metadata()
    finally:
        request.release()  # a leaked request starves the buffer pool


def _run_exposure_sweep(picam2, imx500, intrinsics, args, out_dir) -> list[ExposureResult]:
    """Sweep AE settings in the patrol's own configuration and measure each.

    Runs in mode 'single' (2028x1520 with inference) because that is what the
    patrol will use — an exposure tuned in a different mode would not transfer.
    """
    from libcamera import controls as libcamera_controls

    config = picam2.create_preview_configuration(
        main={"size": STILL_SIZE},
        controls={"FrameRate": intrinsics.inference_rate},
        buffer_count=4,
    )
    if picam2.started:
        picam2.stop()
    picam2.configure(config)
    picam2.start()
    time.sleep(args.settle)

    # The frame duration this configuration started with, so a setting that
    # widens it cannot leak into the next one — which would silently corrupt the
    # run-to-run control at the end of the sweep.
    baseline_frame_duration = picam2.capture_metadata().get("FrameDuration")
    defaults: dict[str, Any] = {
        "ExposureValue": 0.0,
        "AeConstraintMode": libcamera_controls.AeConstraintModeEnum.Normal,
        "AeMeteringMode": libcamera_controls.AeMeteringModeEnum.CentreWeighted,
    }
    if baseline_frame_duration:
        defaults["FrameDurationLimits"] = (baseline_frame_duration, baseline_frame_duration)
        print(f"   baseline frame duration {baseline_frame_duration} us")

    settings = _exposure_settings(
        libcamera_controls.AeMeteringModeEnum, libcamera_controls.AeConstraintModeEnum
    )
    for result in settings:
        print(f"   {result.name} ...", end="", flush=True)
        try:
            # libcamera keeps controls until they are overwritten, so every
            # setting starts from the same restored baseline.
            picam2.set_controls(defaults)
            _settle_ae(picam2, args.ae_settle)
            if result.controls:
                picam2.set_controls(result.controls)
                _settle_ae(picam2, args.ae_settle)

            stem = "".join(c if c.isalnum() or c in "-." else "_" for c in result.name)
            first = out_dir / f"exposure-{stem}-a.jpg"
            second = out_dir / f"exposure-{stem}-b.jpg"
            metadata = _capture_with_metadata(picam2, first)
            # A second capture of the same static scene: keypoints that match
            # across both are the ones a reconstruction can use. Raw counts rise
            # with sensor noise, so they cannot rank these settings on their own.
            _capture_with_metadata(picam2, second)
            result.exposure_time_us = metadata.get("ExposureTime")
            result.analogue_gain = metadata.get("AnalogueGain")
            result.quality = _assess_capture(first)
            result.repeatable = _count_repeatable(first, second)
        except Exception as exc:  # noqa: BLE001 — one bad setting must not end the sweep
            result.error = f"{type(exc).__name__}: {exc}"
            print(f" failed: {result.error}")
            continue
        q = result.quality
        if q:
            print(
                f" bright={q.mean_brightness:.0f} keypoints={q.keypoints} "
                f"repeatable={result.repeatable}"
            )
        else:
            print(" done (no quality report)")
    return list(settings)


def _count_repeatable(first: Path, second: Path) -> int | None:
    try:
        return repeatable_keypoints_between_files(str(first), str(second))
    except (RuntimeError, ValueError):
        return None


def _settle_ae(picam2, seconds: float) -> None:
    """Let auto-exposure converge, then drain the frames it converged over."""
    time.sleep(seconds)
    for _ in range(4):
        picam2.capture_metadata()


def _print_exposure_table(results: list[ExposureResult], blur_limit_ms: float) -> None:
    header = (
        f"{'setting':17s} {'bright':>7s} {'dark':>6s} {'clip':>6s} {'textured':>9s} "
        f"{'keypts':>7s} {'repeat':>7s} {'shutter':>8s} {'gain':>6s}"
    )
    print(header)
    print("-" * len(header))
    for r in results:
        if not r.quality:
            print(f"{r.name:17s} ERROR: {r.error or 'no measurement'}")
            continue
        q = r.quality
        shutter = f"{r.shutter_ms:.1f}ms" if r.shutter_ms else "-"
        gain = f"{r.analogue_gain:.1f}" if r.analogue_gain else "-"
        repeat = f"{r.repeatable}" if r.repeatable is not None else "-"
        print(
            f"{r.name:17s} {q.mean_brightness:7.0f} {q.dark_fraction:5.1%} "
            f"{q.clipped_fraction:5.1%} {q.textured_tiles:4d}/{q.total_tiles:<4d} "
            f"{q.keypoints:7d} {repeat:>7s} {shutter:>8s} {gain:>6s}"
        )

    shutters = {r.shutter_ms for r in results if r.shutter_ms}
    if len(shutters) == 1:
        pinned = shutters.pop()
        print(
            f"\nEvery setting settled on the same {pinned:.1f} ms shutter, so auto-exposure "
            f"is frame-duration limited and buys brightness with gain, not time. The shutter "
            f"is a property of the FrameRate control here, not of these settings — it cannot "
            f"rank them."
        )
        if pinned > blur_limit_ms:
            print(
                f"That {pinned:.1f} ms exceeds --blur-limit-ms={blur_limit_ms:.0f}; whether it "
                f"actually blurs depends on how well the chassis settles, which only a moving "
                f"test can answer."
            )
    elif any(r.shutter_ms and r.shutter_ms > blur_limit_ms for r in results):
        print(
            f"\nSettings above {blur_limit_ms:.0f} ms shutter carry a blur risk once the car "
            f"is only briefly settled."
        )


def _recommend_exposure(results: list[ExposureResult], blur_limit_ms: float) -> str:
    """Rank on repeatable keypoints, and only when the spread beats the control.

    Raw keypoint counts rise with the sensor noise that high gain introduces, so
    they favour exactly the settings that give a reconstruction least to work
    with. Repeatable matches are the honest measure — but a difference between
    two settings only means something if it is larger than the difference between
    two runs of the *same* setting, which is what ``CONTROL_NAME`` measures.
    """
    measured = [r for r in results if r.quality and r.repeatable is not None]
    if not measured:
        return "The exposure sweep produced no measurements — inspect the errors above."

    baseline = next((r for r in measured if r.name == "default"), None)
    control = next((r for r in measured if r.name == CONTROL_NAME), None)
    candidates = [r for r in measured if r.name != CONTROL_NAME]
    viable = [
        r for r in candidates if r.quality.clipped_fraction <= r.quality.policy.max_clipped_fraction
    ]
    if not viable:
        return (
            "No exposure setting stayed within the clipping limit. Inspect the table "
            "before writing the sweep."
        )

    spread = max(r.repeatable for r in viable) - min(r.repeatable for r in viable)
    lines: list[str] = []

    if baseline and control:
        noise = abs(control.repeatable - baseline.repeatable)
        lines.append(
            f"Run-to-run control: the same 'default' setting measured "
            f"{baseline.repeatable} then {control.repeatable} repeatable keypoints "
            f"(±{noise}). Spread across all settings: {spread}."
        )
        if spread <= noise:
            lines.append(
                "The spread does not exceed that control, so no exposure setting is "
                "measurably better than the default here. Keep auto-exposure as it is and "
                "do not add AE overrides to the patrol — the frames' limiting factor is "
                "scene texture and camera aim, not exposure."
            )
            _append_noise_note(lines, viable)
            return "\n".join(lines)

    # A real difference: prefer the fewest-noise route to the best repeatability.
    margin = max(1, spread // 10)
    best_score = max(r.repeatable for r in viable)
    contenders = [r for r in viable if r.repeatable >= best_score - margin]
    best = min(contenders, key=lambda r: (r.analogue_gain or 0.0, r.quality.clipped_fraction))
    lines.append(f"Use exposure setting '{best.name}' ({best.controls or 'no overrides'}).")
    if len(contenders) > 1:
        lines.append(
            f"  Tied within {margin} repeatable keypoints with "
            f"{', '.join(r.name for r in contenders if r.name != best.name)}; picked for the "
            f"lowest gain ({best.analogue_gain:.1f}) and least clipping "
            f"({best.quality.clipped_fraction:.1%})."
        )
    if baseline and best.name != "default":
        lines.append(
            f"  vs default: repeatable {baseline.repeatable} -> {best.repeatable} "
            f"({best.repeatable - baseline.repeatable:+d}), raw keypoints "
            f"{baseline.quality.keypoints} -> {best.quality.keypoints}, gain "
            f"{baseline.analogue_gain:.1f} -> {best.analogue_gain:.1f}."
        )
    _append_noise_note(lines, viable)
    if best.shutter_ms and best.shutter_ms > blur_limit_ms:
        lines.append(
            f"  Confirm the {best.shutter_ms:.1f} ms shutter is sharp enough once the car "
            f"stops: capture during the first supervised patrol run and re-check repeatability."
        )
    return "\n".join(lines)


def _append_noise_note(lines: list[str], viable: list[ExposureResult]) -> None:
    """Point out where raw keypoint counts and repeatable ones disagree."""
    noisiest = max(viable, key=lambda r: r.quality.keypoints)
    best_repeat = max(viable, key=lambda r: r.repeatable)
    if noisiest.name != best_repeat.name:
        lines.append(
            f"  Note '{noisiest.name}' reports the most raw keypoints "
            f"({noisiest.quality.keypoints}) at gain {noisiest.analogue_gain:.1f} but not the "
            f"most repeatable ones ({noisiest.repeatable} vs {best_repeat.repeatable}) — that "
            f"gap is sensor noise a raw count cannot distinguish from texture."
        )


def main() -> int:
    parser = argparse.ArgumentParser(description="IMX500 inference + high-res still mode check")
    parser.add_argument("--model", default=DEFAULT_MODEL)
    parser.add_argument(
        "--check",
        choices=("modes", "exposure", "all"),
        default="all",
        help="which experiment to run",
    )
    parser.add_argument(
        "--mode",
        choices=(*MODES, "all"),
        default="all",
        help="which capture modes the 'modes' check compares",
    )
    parser.add_argument("--frames", type=int, default=5, help="inference reads per mode")
    parser.add_argument(
        "--settle", type=float, default=1.0, help="seconds to let exposure settle before a capture"
    )
    parser.add_argument(
        "--ae-settle",
        type=float,
        default=2.0,
        help="seconds to let auto-exposure converge after changing controls",
    )
    parser.add_argument(
        "--blur-limit-ms",
        type=float,
        default=33.0,
        help="shutter longer than this is flagged as a blur risk",
    )
    parser.add_argument(
        "--resume-timeout",
        type=float,
        default=10.0,
        help="seconds to wait for the CNN tensor after a mode change",
    )
    parser.add_argument("--threshold", type=float, default=0.30)
    parser.add_argument("--out-dir", type=Path, default=Path("/tmp/dual-mode-check"))
    args = parser.parse_args()

    from picamera2 import Picamera2
    from picamera2.devices import IMX500
    from picamera2.devices.imx500 import NetworkIntrinsics

    policy = ObstaclePolicy(confidence_threshold=args.threshold)
    args.out_dir.mkdir(parents=True, exist_ok=True)

    imx500 = IMX500(args.model)
    intrinsics = imx500.network_intrinsics
    if not intrinsics:
        intrinsics = NetworkIntrinsics()
        intrinsics.task = "object detection"
    if intrinsics.task != "object detection":
        print(f"Model is not an object-detection network (task={intrinsics.task})", file=sys.stderr)
        return 1
    intrinsics.update_with_defaults()
    imx500.show_network_fw_progress_bar()

    modes = MODES if args.mode == "all" else (args.mode,)
    run_modes = args.check in ("modes", "all")
    run_exposure = args.check in ("exposure", "all")
    print(f"model={args.model.split('/')[-1]}  still={STILL_SIZE[0]}x{STILL_SIZE[1]}")
    if run_modes:
        print(f"Mode check: {', '.join(modes)}, {args.frames} inference reads each")
        if args.mode == "all":
            print(
                "Modes run cheapest-first; a later mode failing after an earlier one "
                "passed may mean the mode change itself is destructive."
            )
    if run_exposure:
        print(f"Exposure sweep: 8 settings in mode 'single', {args.ae_settle:.0f}s AE settle each")
    print("=" * 78)

    # One Picamera2 for every experiment: reopening re-uploads the network firmware.
    picam2 = Picamera2(imx500.camera_num)
    results: list[ModeResult] = []
    exposures: list[ExposureResult] = []
    try:
        if run_modes:
            for name in modes:
                print(f"\n-- {name} --")
                try:
                    result = RUNNERS[name](picam2, imx500, intrinsics, args, policy, args.out_dir)
                # Recording "mode X raised ..." IS this script's result, and one
                # unusable mode must not hide whether the other two work.
                except Exception as exc:  # noqa: BLE001
                    result = ModeResult(name, error=f"{type(exc).__name__}: {exc}")
                    print(f"   failed: {result.error}")
                else:
                    print(
                        f"   tensor {result.frames_with_tensor}/{result.frames_read} frames, "
                        f"{result.detections} detections, verdict: {result.verdict}"
                    )
                results.append(result)
        if run_exposure:
            print("\n-- exposure sweep --")
            exposures = _run_exposure_sweep(picam2, imx500, intrinsics, args, args.out_dir)
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        if picam2.started:
            picam2.stop()
        picam2.close()

    if results:
        print("\n" + "=" * 78)
        _print_table(results)
        print("-" * 78)
        # The keypoint grid localises a feature desert to a part of the frame,
        # which says where to aim the sweep — a whole-frame number cannot.
        best = next((r for r in results if r.quality), None)
        if best and best.quality:
            print(f"ORB keypoints per tile ({best.name}), 3 rows x 4 columns:")
            for row in best.quality.tile_keypoints:
                print("  " + " ".join(f"{count:7d}" for count in row))
        print(_recommend(results))

    if exposures:
        print("\n" + "=" * 78)
        _print_exposure_table(exposures, args.blur_limit_ms)
        print("-" * 78)
        print(_recommend_exposure(exposures, args.blur_limit_ms))

    print(f"\nCaptures under {args.out_dir}")
    modes_ok = not run_modes or any(r.inference_ok and r.still_ok for r in results)
    exposure_ok = not run_exposure or any(r.quality for r in exposures)
    return 0 if modes_ok and exposure_ok else 1


if __name__ == "__main__":
    sys.exit(main())

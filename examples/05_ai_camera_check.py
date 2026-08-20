#!/usr/bin/env python3
"""Verify the Raspberry Pi AI Camera (IMX500) is detected and captures frames.

Run this ON the Raspberry Pi, not on the Mac:

    python3 examples/05_ai_camera_check.py           # detection only
    python3 examples/05_ai_camera_check.py --photo   # also capture a test still
    python3 examples/05_ai_camera_check.py --inference  # also run a 5 s on-sensor inference pass

Checks performed:
1. A libcamera camera tool (`rpicam-hello` / `libcamera-hello`) is installed
2. The tool lists at least one camera and an IMX500 sensor is among them
3. IMX500 pre-trained model files are present (informational)
4. Picamera2 can open the camera and capture a still image (only with --photo)
5. On-sensor inference runs (only with --inference; needs the
   `rpicam-apps-imx500-postprocess` apt package — the script prints the install
   command if it is missing, models alone are not enough)

Exit code 0 = every hard check passed.

Notes:
- Prefer the system interpreter. Picamera2 ships as an apt package
  (`sudo apt install python3-picamera2`), not in this project's uv venv.
- If the camera was just plugged in, reboot the Pi once before testing.
- If no camera is listed, check the ribbon cable and camera port
  (the AI Camera uses the two-lane CSI connector next to the HDMI ports).
"""

from __future__ import annotations

import argparse
import shutil
import subprocess
import sys
import time
from pathlib import Path

CAMERA_TOOLS = ("rpicam-hello", "libcamera-hello")
IMX500_NET_DIR = Path("/usr/share/imx500-models")
# The postprocess package installs json configs here (Debian trixie / rpicam-apps 1.12):
IMX500_ASSET_DIRS = (Path("/usr/share/rpi-camera-assets"), Path("/usr/share/rpicam-apps/imx500"))
IMX500_PP_LIB = Path("/usr/lib/aarch64-linux-gnu/rpicam-apps-postproc/imx500-postproc.so")
PHOTO_PATH = Path("/tmp/ai-camera-check.jpg")

failures: list[str] = []


def check(label: str, ok: bool, detail: str = "") -> None:
    """Print a PASS/FAIL line and remember failures for the summary."""
    status = "PASS" if ok else "FAIL"
    suffix = f" — {detail}" if detail else ""
    print(f"[{status}] {label}{suffix}")
    if not ok:
        failures.append(label)


def camera_listing() -> str | None:
    """Return combined stdout+stderr of the first available camera tool, or None."""
    for tool in CAMERA_TOOLS:
        if shutil.which(tool) is None:
            continue
        try:
            result = subprocess.run(
                [tool, "--list-cameras"], capture_output=True, text=True, timeout=30, check=False
            )
            return f"{result.stdout}\n{result.stderr}"
        except subprocess.TimeoutExpired:
            return None
    return None


def check_tool_and_camera() -> None:
    listing = camera_listing()
    if listing is None:
        check(
            "camera tool installed",
            False,
            "neither rpicam-hello nor libcamera-hello found — install with: "
            "sudo apt install rpicam-apps",
        )
        return

    lowered = listing.lower()
    check("camera tool runs", listing.strip() != "", "got output from the tool")
    if "no cameras available" in lowered:
        check(
            "at least one camera detected",
            False,
            "the tool reports 'no cameras available' — check the ribbon cable and reboot",
        )
        return

    has_imx500 = "imx500" in lowered
    check(
        "IMX500 (AI Camera) detected",
        has_imx500,
        "found in the camera list" if has_imx500 else "camera(s) present but no IMX500 in the list",
    )


def check_model_files() -> None:
    if not IMX500_NET_DIR.is_dir():
        print(f"[INFO] IMX500 model directory not found: {IMX500_NET_DIR}")
        print("       The camera still works; models are only needed for on-sensor inference.")
        return
    files = sorted(IMX500_NET_DIR.glob("*.rpk"))
    print(f"[INFO] IMX500 models available: {len(files)} file(s) in {IMX500_NET_DIR}")
    for entry in files[:8]:
        print(f"       - {entry.name}")


def check_inference() -> None:
    """Try an on-sensor inference pass with rpicam-hello.

    Requires the `rpicam-apps-imx500-postprocess` apt package (json config +
    postprocess library). Models alone are not enough. Installs need sudo, so if
    the package is missing we print the install command instead of failing the
    camera itself.
    """
    json_files: list[Path] = []
    for d in IMX500_ASSET_DIRS:
        if d.is_dir():
            json_files += sorted(d.glob("*.json"))
    lib_ok = IMX500_PP_LIB.exists()

    if not json_files or not lib_ok:
        print("[INFO] IMX500 on-sensor inference needs the postprocess package:")
        print("       sudo apt install rpicam-apps-imx500-postprocess")
        print("       Then reboot once and re-run with --inference.")
        return

    tool = shutil.which("rpicam-hello")
    if tool is None:
        print("[INFO] rpicam-hello not found — cannot run the inference pass.")
        return

    # Prefer an object-detection config; avoid face/pose demos that need extra
    # models or OpenCV pipelines not relevant here.
    def _prefer(f: Path) -> tuple[int, str]:
        name = f.name.lower()
        score = 0
        for keyword in ("mobilenet", "ssd", "object_detection", "efficientdet"):
            if keyword in name:
                score += 1
        return (score, name)

    pp_file = max(json_files, key=_prefer)
    print(
        f"[INFO] running an inference pass ({IMX500_PP_LIB.name}): {tool} "
        f"--post-process-file {pp_file}"
    )
    print("       First run uploads the network firmware to the IMX500 and can take a few minutes.")
    try:
        result = subprocess.run(
            [tool, "--post-process-file", str(pp_file), "--timeout", "120000"],
            capture_output=True,
            text=True,
            timeout=300,
            check=False,
        )
    except subprocess.TimeoutExpired:
        check("inference pass", False, "timed out after 300 s")
        return

    combined = (result.stdout + result.stderr).lower()
    crashed = result.returncode != 0 and "error" in combined
    check(
        "inference pass",
        not crashed,
        f"exit {result.returncode}" if result.returncode else "rpicam-hello ran for 120 s",
    )


def capture_still(picamera2_cls: type) -> None:
    try:
        camera = picamera2_cls()
        camera.configure(camera.create_still_configuration())
        camera.start()
        time.sleep(1.5)  # let auto-exposure converge
        camera.capture_file(str(PHOTO_PATH))
        camera.stop()
        ok = PHOTO_PATH.exists() and PHOTO_PATH.stat().st_size > 0
        check(
            "still capture",
            ok,
            f"wrote {PHOTO_PATH} ({PHOTO_PATH.stat().st_size} bytes)"
            if ok
            else "capture produced no file",
        )
    except Exception as exc:  # noqa: BLE001 - report any backend error
        check("still capture", False, str(exc))


def check_picamera2(photo: bool) -> None:
    try:
        from picamera2 import Picamera2
    except ImportError:
        check(
            "picamera2 import",
            False,
            "run with the system python3 (picamera2 is an apt package, not a uv dependency)",
        )
        return

    try:
        cameras = Picamera2.global_camera_info()
    except Exception as exc:  # noqa: BLE001 - report any backend error
        check("picamera2 enumerates cameras", False, str(exc))
        return

    if not cameras:
        check("picamera2 sees a camera", False, "no cameras reported")
        return

    models = [c.get("Model", "?") for c in cameras]
    check("picamera2 sees a camera", True, ", ".join(models))
    check(
        "IMX500 visible to picamera2",
        any("imx500" in m.lower() for m in models),
        f"models: {', '.join(models)}",
    )

    if photo:
        capture_still(Picamera2)


def main() -> int:
    parser = argparse.ArgumentParser(description="Check the Raspberry Pi AI Camera (IMX500).")
    parser.add_argument(
        "--photo",
        action="store_true",
        help="capture a test still image to /tmp/ai-camera-check.jpg",
    )
    parser.add_argument(
        "--inference",
        action="store_true",
        help="run a 5 s on-sensor inference pass (needs rpicam-apps-imx500-postprocess)",
    )
    args = parser.parse_args()

    print("Raspberry Pi AI Camera check")
    print("=" * 30)
    check_tool_and_camera()
    check_model_files()
    check_picamera2(args.photo)
    if args.inference:
        check_inference()

    print()
    if failures:
        print(f"FAILED — {len(failures)} check(s) failed:")
        for item in failures:
            print(f"  - {item}")
        print("If the camera was just connected, reboot the Pi and try again.")
        return 1
    print("All checks passed.")
    return 0


if __name__ == "__main__":
    sys.exit(main())

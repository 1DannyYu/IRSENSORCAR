# Camera Inferences

This directory is a convenient place to find task material on the Mac. Executable code that runs
on the Pi must live under `examples/`; see the repository `CONVENTIONS.md` and
`docs/setup/mac-to-raspberry-pi-access.md`.

> **Note:** This directory is tracked by Git. Disposable photos and benchmark records belong in
> ignored `scratch/` under CONVENTIONS §7. The existing tracked image remains pending a separate
> retention decision; see
> `docs/progress/2026-08-19-tasks-directory-conventions.md`。

## Script

[`examples/35_cam_object_id_check.py`](../../examples/35_cam_object_id_check.py)

Run on the Pi:

```bash
PYTHONPATH=src python3 examples/35_cam_object_id_check.py
```

This measures IMX500 on-sensor object detection using SSD MobileNetV2 FPN-Lite; YOLO is not
installed on this Pi. It reports camera cold-start time, per-frame latency, and confidence rather
than mistaking the 120-second `05_ai_camera_check.py --inference` soak duration for one inference.

## Contents

- Disposable captures and benchmark logs belong under ignored `scratch/`.
- `2026-08-17-front-object-detection-check.jpg` is a 2026-08-17 `carpi` test image containing a
  drink can, banana, and orange. It was used to evaluate object-detection software.

# Scripts Overview

This folder contains one-off tools, validators, generators, and workstation-only utilities used to prepare, verify, and process project data.

According to [CONVENTIONS.md](../CONVENTIONS.md), this directory is meant for first-party code that does not belong in the main package but is still part of the project workflow.

## Script categories

### 1. Map and route generation
- `generate_task1_map.py` — Generates the printable Task-1 map as multi-page A4 PDFs and emits the tag placement JSON.
- `annotate_tag_map.py` — Overlays tag IDs and orientations onto a corrected route map for visual checking.
- `generate_apriltag_sheet.py` — Produces a printable AprilTag sheet for physical tagging of the map.

### 3. Ground-view and calibration helpers
- `generate_ground_view_target.py` — Prints a calibration target rectangle for ground-view camera calibration.
- `pick_ground_view_corners.py` — Lets an operator click the calibration rectangle corners and turns them into the exact `--corners` argument.

### 4. Structure-from-motion and scale anchoring
- `run_colmap_sfm.py` — Runs a COLMAP reconstruction pipeline using `pycolmap` on a set of overlapping photos.
- `anchor_sfm_scale.py` — Anchors a recovered SfM model to metric scale using a known physical AprilTag target.

### 5. Hardware check wrappers
- `map1-phase1-ir-check.sh` — Phase 1 of the Map1 test plan: stationary 4-channel IR tracing sensor readout with this build's verified pins and inversion. No motors move.

### 6. Special-purpose production helpers
- `generate_ground_view_target.py` — Creates a physical paper target for calibration and measurement.
- `generate_apriltag_sheet.py` — Creates physical tag sheets for map placement.

---

## Script-by-script summary

### `anchor_sfm_scale.py`
Main purpose:
- Converts a COLMAP reconstruction from arbitrary units into real-world metric units.

What it does:
- loads a sparse reconstruction and image set
- detects AprilTags in registered images
- estimates metres-per-unit from known tag geometry
- writes `scale.json` and a metric trajectory CSV

This is used to make a room or map reconstruction meaningful in actual real-world dimensions.

### `annotate_tag_map.py`
Main purpose:
- Draws each tag on top of the corrected map image for human inspection.

What it does:
- reads a tag-map JSON with `x_m`, `y_m`, and `yaw_deg`
- overlays tag footprints, north arrows, and IDs
- writes a visual annotated map image

This is useful for checking whether tag positions are physically placed correctly.

This is a quality-control script for the website data.

### `generate_apriltag_sheet.py`
Main purpose:
- Prints an A4 sheet of AprilTags for physical placement on the map.

What it does:
- generates tag images at exact scale
- adds a ruler verification bar
- writes a printable PDF and optional tag-map template JSON

This is used before placing the physical tags on the robot route map.

### `generate_ground_view_target.py`
Main purpose:
- Produces a printable calibration target for ground-view geometry work.

What it does:
- draws a rectangle with tick marks and corner labels
- includes a ruler-scale warning
- writes a PDF for printing at actual size

This helps build a target whose real dimensions are known to the camera calibration workflow.

### `generate_task1_map.py`
Main purpose:
- Generates the complete Task-1 route map in printable, metric-correct form.

What it does:
- draws the route geometry, roundabout, and tag positions at millimetre scale
- splits the map into A4 sheets
- writes the PDF and tag-map JSON

This is one of the key map-production scripts for the project.

### `map1-phase1-ir-check.sh`
Main purpose:
- Runs the stationary IR sensor check that gates every Map1 line-following test.

What it does:
- `cd`s to the repository root so it works from any directory
- calls `examples/36_ir_tracing_check.py` with this build's verified pins (`24,25,22,23`) and inversion (`0,1,2,3`)
- prints 30 readings at 0.3 s intervals

Safe to run over SSH — nothing moves. Re-run it after touching the sensitivity potentiometers, since channel polarity is not stable across retunes.

### `pick_ground_view_corners.py`
Main purpose:
- Converts manual corner selection into the exact `--corners` string used by the calibration example.

What it does:
- opens an image in an OpenCV window
- prompts the user to click the four rectangle corners in TL, TR, BR, BL order
- prints the exact pixel coordinates for the calibration script

This reduces human error in the calibration setup.

### `run_colmap_sfm.py`
Main purpose:
- Runs a complete COLMAP sparse reconstruction pipeline using `pycolmap`.

What it does:
- reads a folder of overlapping images
- extracts features
- matches images exhaustively
- reconstructs the 3D scene
- writes a sparse model and summary JSON

This is the scene-reconstruction step before metric scaling and map anchoring.

---

## Typical workflow

A common project flow is:

1. Capture overlapping room or route images.
2. Run `run_colmap_sfm.py` to build a reconstruction.
3. Run `anchor_sfm_scale.py` to get real-world metric scale.
4. Use `generate_task1_map.py` or `annotate_tag_map.py` to prepare printable maps and tag placement layouts.

This folder is not the robot runtime code; it is the project’s preparation, generation, and validation toolkit.

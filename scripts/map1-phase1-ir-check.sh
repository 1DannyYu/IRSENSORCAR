#!/bin/bash
# Map1 test, phase 1: stationary IR tracing sensor check.
#
# No motors move — safe to run over SSH. Pins and inversion match the verified
# wiring in docs/hardware/ir-tracing-sensor.md; re-run this after touching the
# sensitivity potentiometers, since their polarity is not stable across retunes.
set -euo pipefail

# Run from the repository root regardless of where the script is invoked from.
cd "$(dirname "$0")/.."

echo "=== Map1 Test Phase 1: IR Sensor Check ==="
echo "Hold the sensor 1-3 cm above the track"
echo "Press Ctrl+C to stop"
echo ""
PYTHONPATH=src python3 examples/36_ir_tracing_check.py \
    --pins 24,25,22,23 --invert 0,1,2,3 --count 30 --interval 0.3

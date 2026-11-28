#!/usr/bin/env bash
# The oracle, run in place of the agent: install the reference controller and the camera calibration it reads (a copy
# of the cell's documents in ~/cell/calibration: cameras.json and the empty-cell shots) into the submission folder.
#
# The reference controller is derived from the controller Claude Opus 5.5 wrote for this task in the benchmark run
# c5d336ba (rebuilt from that run's transcript): latency and grip-zone width made explicit parameters, and its catch
# timing and aim re-tuned on the development simulator for the final cell. See PR_DESCRIPTION / tests/review.
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
SUB=/home/user/submission
mkdir -p "$SUB"
cp "$HERE/ref/controller.py" "$HERE/ref/vision.py" "$HERE/ref/run.sh" "$SUB/"
cp /home/user/cell/calibration/* "$SUB/"
ls -la "$SUB"

#!/usr/bin/env bash
# The oracle, run in place of the agent: install the reference controller (solution/controller/: wrist-camera belt
# tracking with latency compensation, turnover handling, inspection-camera in-hand measurement, fiducial kit location
# and a force-guided peck-search insertion with a height check). It uses only the data sheet's numbers and what the
# cell reports on the wire.
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
SUB=/home/user/submission
mkdir -p "$SUB"
cp "$HERE"/controller/*.py "$HERE/controller/run.sh" "$SUB/"
ls -la "$SUB"

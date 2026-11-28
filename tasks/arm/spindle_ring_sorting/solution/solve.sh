#!/usr/bin/env bash
# The oracle, run in place of the agent: install the reference controller (solution/controller/: survey with the wrist
# camera, read the stacks from the overview camera, plan, then pick / carry / thread every ring with a force-guided
# spiral search on the spindle's tip, learning the camera's aim from where the rings went in).
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
SUB=/home/user/submission
mkdir -p "$SUB"
cp "$HERE/controller/controller.py" "$HERE/controller/run.sh" "$SUB/"
ls -la "$SUB"

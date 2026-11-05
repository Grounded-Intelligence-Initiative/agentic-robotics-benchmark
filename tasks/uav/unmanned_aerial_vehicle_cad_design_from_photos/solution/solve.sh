#!/usr/bin/env bash
set -euo pipefail
TASK_HOME="${ALE_HOME:-/home/user}"
install -d -m 0755 "$TASK_HOME/submission"
cp reference/drone.FCStd "$TASK_HOME/submission/drone.FCStd"
chown -R "$(basename "$TASK_HOME"):$(basename "$TASK_HOME")" "$TASK_HOME/submission"

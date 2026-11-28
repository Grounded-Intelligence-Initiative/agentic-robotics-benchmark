#!/usr/bin/env bash
set -euo pipefail
TASK_HOME="${ALE_HOME:-/home/user}"
TASK_USER="$(basename "$TASK_HOME")"
cp -r payload/. "$TASK_HOME/"
for entry in payload/*; do
  chown -R "$TASK_USER:$TASK_USER" "$TASK_HOME/$(basename "$entry")"
done

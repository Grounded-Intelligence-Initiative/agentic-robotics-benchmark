#!/usr/bin/env bash
set -euo pipefail
TASK_HOME="${ALE_HOME:-/home/user}"
TASK_USER="$(basename "$TASK_HOME")"
mkdir -p /opt/arb
cp -r /opt/task-assets/payload/. /opt/arb/
cp -r payload/. /opt/arb/
chown -R root:root /opt/arb
chmod -R a+rX,go-w /opt/arb
cp -r /opt/arb/. "$TASK_HOME/"
for entry in /opt/arb/*; do
  chown -R "$TASK_USER:$TASK_USER" "$TASK_HOME/$(basename "$entry")"
done
mkdir -p "$TASK_HOME/submission"
chown "$TASK_USER:$TASK_USER" "$TASK_HOME/submission"

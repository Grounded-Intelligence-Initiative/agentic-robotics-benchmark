#!/usr/bin/env bash
# Setup (root): stage the practice grader and the practice episodes into the agent home.
set -euo pipefail
umask 022
cd "$(dirname "$0")"
HOME_DIR="${ALE_HOME:-/home/user}"
AGENT_USER="$(basename "$HOME_DIR")"
mkdir -p "$HOME_DIR/practice_grader" "$HOME_DIR/submission"
cp payload/practice_grader/* "$HOME_DIR/practice_grader/"
if [ -d /opt/task-assets/payload/practice ]; then
  cp -r /opt/task-assets/payload/practice "$HOME_DIR/practice"
fi
chown -R "$AGENT_USER:$AGENT_USER" "$HOME_DIR/practice_grader" "$HOME_DIR/submission"
if [ -d "$HOME_DIR/practice" ]; then
  chown -R "$AGENT_USER:$AGENT_USER" "$HOME_DIR/practice"
  find "$HOME_DIR/practice" -type d -exec chmod 0755 {} + ; find "$HOME_DIR/practice" -type f -exec chmod 0644 {} +
fi

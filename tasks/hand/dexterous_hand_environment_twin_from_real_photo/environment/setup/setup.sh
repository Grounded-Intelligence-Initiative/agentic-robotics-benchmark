#!/usr/bin/env bash
# Setup: trusted root, cwd = this staged folder, before the agent phase.
#
# Stage the givens (the photo and the camera sheet, the site brief, the twin contract, the hand card, the pick sheet,
# the pick instrument, check_twin.py with the public gate and standard simulation) into the
# agent home, owned by the agent. Nothing hidden is in this folder: the truth lives in tests/ (absent during the agent
# phase). The photo comes from the image's task assets when the image has them (the published task), else from
# payload/ (development). There is no warm-up: nothing here needs a GPU.
set -uo pipefail
HOME_DIR="${ALE_HOME:-/home/user}"
AGENT_USER="$(basename "$HOME_DIR")"
cp -r payload/. "$HOME_DIR/" || { echo "[setup] staging the payload failed" >&2; exit 1; }
if [ -d /opt/task-assets/payload ]; then
  cp -r /opt/task-assets/payload/. "$HOME_DIR/" || { echo "[setup] staging the task assets failed" >&2; exit 1; }
fi
for entry in payload/* payload/.[!.]* /opt/task-assets/payload/*; do
  [ -e "$entry" ] || continue
  chown -R "$AGENT_USER:$AGENT_USER" "$HOME_DIR/$(basename "$entry")"
done
mkdir -p "$HOME_DIR/submission"
chown "$AGENT_USER:$AGENT_USER" "$HOME_DIR/submission"
chmod +x "$HOME_DIR/check_twin.py" 2>/dev/null
if [ ! -f "$HOME_DIR/photo.png" ]; then
  echo "[setup] photo.png is missing from the givens" >&2
  exit 1
fi
echo "[setup] givens staged in $HOME_DIR (owner $AGENT_USER)" >&2
exit 0

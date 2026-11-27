#!/usr/bin/env bash
set -euo pipefail
HOME_DIR="${ALE_HOME:-/home/user}"
AGENT_USER="$(basename "$HOME_DIR")"
cp -r /opt/public-kit/. "$HOME_DIR/"
# A single passive sensor sample permits protocol and image-decoding checks.
# No action, score, model, or simulator endpoint is exported.
PYTHONPATH=/opt/station MUJOCO_GL=egl /opt/physics/bin/python - "$HOME_DIR/sensor_sample.json" <<'SAMPLE'
import json, sys
from workcell import ArenaRound
arena = ArenaRound(17)
try:
    with open(sys.argv[1], "w") as f:
        json.dump(arena.observation(), f)
finally:
    arena.close()
SAMPLE
for entry in /opt/public-kit/* /opt/public-kit/.[!.]*; do
  [ -e "$entry" ] || continue
  chown -R "$AGENT_USER:$AGENT_USER" "$HOME_DIR/$(basename "$entry")"
done
chown "$AGENT_USER:$AGENT_USER" "$HOME_DIR/sensor_sample.json"

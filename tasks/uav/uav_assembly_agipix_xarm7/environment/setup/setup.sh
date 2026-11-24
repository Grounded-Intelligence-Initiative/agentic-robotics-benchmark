#!/usr/bin/env bash
# Setup: trusted root, cwd = this staged folder (/opt/ale/setup, root-only while the agent works), before the agent
# phase. Two jobs:
# (1) The givens into the agent home, owned by the agent: first the image's task assets (the kit, the xArm 7 model, the
#     picture manual and the cameras' scenery, render/; environment/Dockerfile), then payload/ (the connection client
#     and runner, the graded episode's client, the cell and provenance documents). The grader never reads these copies:
#     verification is separate (task.toml), and tests/test.sh builds the grader's own root-owned, read-only copy of the
#     same givens in the fresh verifier sandbox, from the image's task assets and tests/setup_payload/, a byte copy of
#     payload/ here (change both together, then tests/given_hashes.json).
# (2) The graded episode's cell service (online/service.py; dual mode, experimental): the dedicated account `arbcell`
#     (created here so that the image stays as main builds it); the cell's copy online/cell (a byte copy of the cell's
#     part of tests/) installed at /opt/arb-online/cell, root:arbcell 0750 / 0640, which the agent's account cannot
#     read; then the service, which reads the hidden graded episode (online/episode.json, root-only here) as root,
#     builds the episode's workcell from the image's task assets, drops to `arbcell` and serves
#     /run/arb-online/cell.sock. The episode's record goes to /var/lib/arb-online (arbcell's, 0700: the agent can
#     neither read nor write it; a declared artifact, scored by tests/verify.py in the separate verifier). Dual mode: at
#     the earlier of the graded start (before it is charged) and the start window's close the service's root supervisor
#     freezes the agent's program (/home/user/submission -> /var/lib/arb-online/submission_snapshot), the copy the
#     verifier grades as the program item. Fixed argv, a clean environment, no hidden value outside this folder and the
#     root:arbcell folder. The command returns once the service serves (or fails setup).
# There is no practice workcell: the service serves the one graded episode and nothing else. Dynamic-only: packages
# live in environment/Dockerfile.
set -euo pipefail
umask 022
HOME_DIR="${ALE_HOME:-/home/user}"
AGENT_USER="$(basename "$HOME_DIR")"
LAB_USER=arbcell
SERVICE_DIR=/opt/arb-online
PY=/opt/venv/bin/python3
cp -r /opt/task-assets/payload/. "$HOME_DIR/"
cp -r payload/. "$HOME_DIR/"
for entry in /opt/task-assets/payload/* payload/*; do
  [ -e "$entry" ] || continue
  chown -R "$AGENT_USER:$AGENT_USER" "$HOME_DIR/$(basename "$entry")"
done
mkdir -p "$HOME_DIR/submission"
chown "$AGENT_USER:$AGENT_USER" "$HOME_DIR/submission"
echo "[setup] givens staged in $HOME_DIR (owner $AGENT_USER)" >&2

# (2) the graded episode's cell service
for f in online/service.py online/online.json online/episode.json online/cell/grader_config.json \
         online/cell/cell.py online/cell/server.py online/cell/given_hashes.json; do
  [ -f "$f" ] || { echo "[setup] online/ is incomplete: $f is missing" >&2; exit 1; }
done
id -u "$LAB_USER" >/dev/null 2>&1 || \
  useradd --system --user-group --no-create-home --home-dir /nonexistent --shell /usr/sbin/nologin "$LAB_USER"
id -nG "$AGENT_USER" | tr ' ' '\n' | grep -qx "$LAB_USER" && { echo "[setup] $AGENT_USER is in group $LAB_USER" >&2; exit 1; }
chmod 0700 online
chmod 0600 online/episode.json
rm -rf "$SERVICE_DIR"
install -d -o root -g "$LAB_USER" -m 0750 "$SERVICE_DIR"
cp -r online/cell "$SERVICE_DIR/cell"
find "$SERVICE_DIR/cell" -name '__pycache__' -type d -prune -exec rm -rf {} +
chown -R "root:$LAB_USER" "$SERVICE_DIR/cell"
find "$SERVICE_DIR/cell" -type d -exec chmod 0750 {} +
find "$SERVICE_DIR/cell" -type f -exec chmod 0640 {} +
install -d -o root -g root -m 0755 /run/arb-online
install -d -o "$LAB_USER" -g "$LAB_USER" -m 0700 /var/lib/arb-online
install -d -o "$LAB_USER" -g "$LAB_USER" -m 0700 /var/lib/arb-online-work
umask 077
# Returns once the service serves (the daemon is detached, its stdio on /dev/null) or fails setup. The renderer and
# thread settings are the verify stage's (tests/test.sh); the service sets them again for its worker from online.json.
if ! env -i PATH=/opt/venv/bin:/usr/local/bin:/usr/bin:/bin LANG=C.UTF-8 HOME=/nonexistent \
    MUJOCO_GL=osmesa PYOPENGL_PLATFORM=osmesa OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 LP_NUM_THREADS=8 \
    "$PY" -I -B "$PWD/online/service.py" --config "$PWD/online/online.json" </dev/null; then
  echo "[setup] the graded episode's cell service did not come up; its logs:" >&2
  tail -n 40 /var/lib/arb-online/service.log /var/lib/arb-online/supervisor.log >&2 || true
  exit 1
fi
echo "[setup] the graded episode's cell service is up (/run/arb-online/cell.sock)" >&2

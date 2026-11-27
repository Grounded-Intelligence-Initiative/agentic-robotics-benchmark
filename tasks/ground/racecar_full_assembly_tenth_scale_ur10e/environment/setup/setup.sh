#!/usr/bin/env bash
# Setup: trusted root, cwd = this staged folder (/opt/ale/setup, root-only while the agent works), before the agent
# phase. Two jobs:
# (1) The givens (as on main): the kit list, the units' STL files, the drawings, the picture manual, the cell documents,
#     the robot models and the cell library, and the cameras' scenery, staged into the agent home, owned by the agent.
#     The scoring stage never reads these copies: it runs in a separate sandbox (task.toml) and builds its own
#     root-owned read-only copy of the same givens from the image's task assets and a byte copy of payload/ here.
# (2) The graded episode's service (dual mode, online/): the dedicated lab account `arbcell` (created here, so the
#     image stays cached); the offline grader's cell code (online/cell, a byte copy of the grader's tests/ packages) at
#     /opt/arb-online/cell and the cell's hidden kit (online/fetch_kit.py: downloaded while setup still has network,
#     with the grader's pin, grader_config.json hidden_gt) at /opt/arb-online/gt, both root-owned and readable by the
#     lab account only; then online/graded_server.py, which reads the hidden graded episode (online/graded_episode.json,
#     an episode of the fixed hidden set) and the hidden bench plan as root, drops to `arbcell` and serves
#     /run/arb-graded/graded.sock. The episode's record goes to /var/lib/arb-online (the lab account's, 0700: the agent
#     can neither read nor write it; a declared artifact, scored by tests/verify.py in the separate verifier). At the
#     earlier of the graded start (before it is charged) and the preparation window's close, the service's root
#     supervisor freezes the agent's program (/home/user/submission -> /var/lib/arb-online/submission_snapshot), the
#     copy the verifier grades as the program item. The command returns once the service serves (or fails setup).
#     There is no practice facility.
# Dynamic-only: packages live in environment/Dockerfile.
set -euo pipefail
HOME_DIR="${ALE_HOME:-/home/user}"
AGENT_USER="$(basename "$HOME_DIR")"
ASSETS=/opt/task-assets/payload
# the large givens and the cameras' scenery (render/) come from the image (environment/Dockerfile, task assets)
[ -d "$ASSETS" ] || { echo "[setup] $ASSETS missing: the image was built without its task assets" >&2; exit 1; }
[ -d payload ] || { echo "[setup] payload/ missing next to setup.sh" >&2; exit 1; }
cp -r payload/. "$HOME_DIR/"
cp -r "$ASSETS/." "$HOME_DIR/"
for entry in payload/* payload/.[!.]* "$ASSETS"/*; do
  [ -e "$entry" ] || continue
  chown -R "$AGENT_USER:$AGENT_USER" "$HOME_DIR/$(basename "$entry")"
done
mkdir -p "$HOME_DIR/submission" && chown "$AGENT_USER:$AGENT_USER" "$HOME_DIR/submission"
echo "[setup] givens staged in $HOME_DIR (owner $AGENT_USER)" >&2

# (2) the graded episode's service
LAB_USER=arbcell
SETUP_DIR="$(pwd -P)"
umask 022
if ! id -u "$LAB_USER" >/dev/null 2>&1; then
  useradd --system --user-group --no-create-home --home-dir /nonexistent --shell /usr/sbin/nologin "$LAB_USER"
fi
chmod 0700 online
chmod 0600 online/*.json online/*.sha256
install -d -o root -g root -m 0755 /opt/arb-online
rm -rf /opt/arb-online/cell
cp -r online/cell /opt/arb-online/cell
chown -R "root:$LAB_USER" /opt/arb-online/cell
chmod -R u=rwX,g=rX,o= /opt/arb-online/cell
HOME=/root PYTHONNOUSERSITE=1 /opt/venv/bin/python3 -I -B "$SETUP_DIR/online/fetch_kit.py" \
  --grader-config "$SETUP_DIR/online/grader_config.json" --dest /opt/arb-online/gt \
  --receipt /opt/arb-online/gt.receipt.json --group "$LAB_USER"
install -d -o root -g root -m 0755 /run/arb-graded
install -d -o "$LAB_USER" -g "$LAB_USER" -m 0700 /var/lib/arb-online
umask 077
# Returns once the service serves (the daemon is detached, its stdio on /dev/null) or fails setup.
env -i PATH=/opt/venv/bin:/usr/local/bin:/usr/bin:/bin LANG=C.UTF-8 HOME=/nonexistent \
  /opt/venv/bin/python3 -I -B "$SETUP_DIR/online/graded_server.py" --config "$SETUP_DIR/online/graded_config.json" </dev/null
echo "[setup] the graded episode's service is up (/run/arb-graded/graded.sock)" >&2

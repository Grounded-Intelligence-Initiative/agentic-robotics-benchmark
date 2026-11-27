#!/usr/bin/env bash
# Setup: trusted root, cwd = this staged folder (/opt/ale/setup, root-only while the agent works), before the agent
# phase. Two jobs:
# (1) The givens (main's, unchanged): the kit, the manual, the cell documents, the Panda and the cell library, into the
#     agent home, owned by the agent. The grader never reads these copies: verification is separate (task.toml), and
#     tests/test.sh builds the grader's own ROOT-OWNED read-only copy of the same givens in the fresh verifier sandbox,
#     from the image's task assets and tests/setup_payload/, a byte copy of payload/ here (change both together, then
#     tests/given_hashes.json). Dynamic-only: packages live in environment/Dockerfile.
# (2) The graded episode's cell service (dual mode, experimental; online/service.py): the dedicated lab account
#     `arbcell` (created here, so the task image stays main's), the cell installed where only root and that account
#     can read it (/opt/arb-online-cell, root:arbcell 0750 / 0640: online/cell, byte copies of the verifier's cell
#     code, the hidden kit, the hidden bench plan and the grader configuration) and the cell's collision-piece cache
#     (/var/cache/arb-online-cell, the account's own, 0700), then the service: it reads the hidden graded episode's seed
#     (online/episode.json, root-only here) as root, drops to `arbcell`, builds the episode's cell (its first build
#     computes the collision pieces: a few minutes) and serves /run/arb-online/cell.sock; the episode's record goes to
#     /var/lib/arb-online (the lab account's, 0700: the agent can neither read nor write it; a declared artifact,
#     scored by tests/verify.py in the separate verifier). At the earlier of the graded start (before it is charged) and the start window's close, the service's
#     root supervisor freezes the agent's program (/home/user/submission -> /var/lib/arb-online/submission_snapshot),
#     the copy the verifier grades as the offline item. Fixed argv, a clean environment, no hidden value outside this
#     folder and the lab account's install. The command returns once the service serves (or fails setup).
# There is no practice cell: the service serves the one graded episode and nothing else.
set -euo pipefail
HOME_DIR="${ALE_HOME:-/home/user}"
AGENT_USER="$(basename "$HOME_DIR")"
cp -r payload/. "$HOME_DIR/"
# the large givens (the manual, the parts' STL files, the Panda model) and the cameras' scenery (render/) come from the
# image (environment/Dockerfile, task assets)
if [ -d /opt/task-assets/payload ]; then
  cp -r /opt/task-assets/payload/. "$HOME_DIR/"
fi
for entry in payload/* payload/.[!.]* /opt/task-assets/payload/*; do
  [ -e "$entry" ] || continue
  chown -R "$AGENT_USER:$AGENT_USER" "$HOME_DIR/$(basename "$entry")"
done
mkdir -p "$HOME_DIR/submission" && chown "$AGENT_USER:$AGENT_USER" "$HOME_DIR/submission"
echo "[setup] givens staged in $HOME_DIR (owner $AGENT_USER)" >&2

# (2) the graded episode's cell service
LAB_USER=arbcell
CELL_DIR=/opt/arb-online-cell
export HOME=/root PYTHONNOUSERSITE=1
unset PYTHONPATH PYTHONSTARTUP
chmod 0700 online
chmod 0600 online/episode.json
id -u "$LAB_USER" >/dev/null 2>&1 || \
  useradd --system --user-group --no-create-home --home-dir /nonexistent --shell /usr/sbin/nologin "$LAB_USER"
rm -rf "$CELL_DIR"
install -d -o root -g "$LAB_USER" -m 0750 "$CELL_DIR"
cp -r online/cell/. "$CELL_DIR/"
chown -R "root:$LAB_USER" "$CELL_DIR"
chmod -R u=rwX,g=rX,o= "$CELL_DIR"
find "$CELL_DIR" -name __pycache__ -prune -exec rm -rf {} +
install -d -o root -g root -m 0755 /run/arb-online
install -d -o "$LAB_USER" -g "$LAB_USER" -m 0700 /var/lib/arb-online
install -d -o "$LAB_USER" -g "$LAB_USER" -m 0700 /var/cache/arb-online-cell
umask 077
# Returns once the service serves (the daemon is detached, its stdio on /dev/null) or fails setup.
env -i PATH=/opt/venv/bin:/usr/local/bin:/usr/bin:/bin LANG=C.UTF-8 HOME=/nonexistent \
  /opt/venv/bin/python3 -I -B "$PWD/online/service.py" --config "$PWD/online/online.json" </dev/null
echo "[setup] the graded episode's cell service is up (/run/arb-online/cell.sock)" >&2

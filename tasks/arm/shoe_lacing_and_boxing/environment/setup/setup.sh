#!/usr/bin/env bash
# Setup: trusted root, cwd = this staged folder (/opt/ale/setup, root-only), before the agent phase.
#  (1) Stages the givens (the cell documents, the cell library, the practice scripts and the example program) into the
#      agent home, owned by the agent.
#  (2) Builds the graded service's root-owned copy of the same givens at /opt/arb-online (payload/ and the image's robot
#      model, /opt/task-assets/panda; the service checks it file by file against online/given_hashes.json, a byte copy
#      of tests/given_hashes.json) and its private folder /opt/arb-online/private (online/reference/, the maintainers'
#      reference controller, readable by the service account only).
#  (3) Creates the record directory /var/lib/arb-online (the service account's, 0700; a declared artifact) and starts
#      the graded service (online/server.py): it reads the graded episode (online/episode.json) as root, drops to the
#      dedicated account `arbonline` and serves the cell on /run/arb-online/cell.sock (fixed argv, clean environment).
# Dynamic-only: packages, the robot model and the service account live in environment/Dockerfile.
set -euo pipefail
umask 022
export HOME=/root PYTHONNOUSERSITE=1
unset PYTHONPATH PYTHONSTARTUP
HOME_DIR="${ALE_HOME:-/home/user}"
AGENT_USER="$(basename "$HOME_DIR")"
SERVICE_USER=arbonline

# (1) cp -r, NOT -a: with -a the `payload/.` form also copies the payload directory's own root ownership onto the agent
# home, locking the agent out of its own workspace.
cp -r payload/. "$HOME_DIR/"
if [ -d /opt/task-assets/panda ]; then
  mkdir -p "$HOME_DIR/given"
  cp -r /opt/task-assets/panda "$HOME_DIR/given/panda"
fi
for entry in payload/* payload/.[!.]*; do
  [ -e "$entry" ] || continue
  chown -R "$AGENT_USER:$AGENT_USER" "$HOME_DIR/$(basename "$entry")"
done
mkdir -p "$HOME_DIR/submission" && chown "$AGENT_USER:$AGENT_USER" "$HOME_DIR/submission"

# (2)
id "$SERVICE_USER" >/dev/null 2>&1 || { echo "[setup] the service account $SERVICE_USER is missing from the image" >&2; exit 1; }
[ -d /opt/task-assets/panda ] || { echo "[setup] /opt/task-assets/panda missing: the image was built without its robot model" >&2; exit 1; }
rm -rf /opt/arb-online
install -d -o root -g root -m 0755 /opt/arb-online
cp -r payload/. /opt/arb-online/
cp -r /opt/task-assets/panda /opt/arb-online/given/panda
chown -R root:root /opt/arb-online
chmod -R u+rwX,go+rX,go-w /opt/arb-online
install -d -o root -g "$SERVICE_USER" -m 0750 /opt/arb-online/private
install -o root -g "$SERVICE_USER" -m 0640 online/reference/*.py /opt/arb-online/private/

# (3)
install -d -o "$SERVICE_USER" -g "$SERVICE_USER" -m 0700 /var/lib/arb-online
umask 077
# Returns once the service serves (the daemon is detached, stdio on /dev/null) or fails setup.
env -i PATH=/opt/venv/bin:/usr/local/bin:/usr/bin:/bin LANG=C.UTF-8 HOME=/nonexistent \
  python3 -I -B "$PWD/online/server.py" --config "$PWD/online/online.json" </dev/null >/dev/null
echo "[setup] givens staged in $HOME_DIR (owner $AGENT_USER); the graded service is up" >&2

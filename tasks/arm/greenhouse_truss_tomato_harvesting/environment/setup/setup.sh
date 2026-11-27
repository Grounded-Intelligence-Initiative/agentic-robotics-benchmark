#!/usr/bin/env bash
# Setup: trusted root, cwd = this staged folder, before the agent phase. Stages the givens (the growing protocol, the
# tool's data sheet, the cart's interface document and numbers, the robot's model, the client library, the
# commissioning launcher and the example program) into the agent home, owned by the agent, and starts the
# commissioning service (below). The grader never reads these copies: verification is separate (task.toml), and a
# program under test is given nothing but its own submission folder.
# Dynamic-only: packages, the robot's model and the commissioning service's program live in environment/Dockerfile.
set -euo pipefail
HOME_DIR="${ALE_HOME:-/home/user}"
AGENT_USER="$(basename "$HOME_DIR")"
# cp -r, NOT -a: with -a the `payload/.` form also copies the payload directory's own root ownership onto the agent
# home, locking the agent out of its own workspace.
cp -r payload/. "$HOME_DIR/"
for entry in payload/* payload/.[!.]*; do
  [ -e "$entry" ] || continue
  chown -R "$AGENT_USER:$AGENT_USER" "$HOME_DIR/$(basename "$entry")"
done
mkdir -p "$HOME_DIR/submission" && chown "$AGENT_USER:$AGENT_USER" "$HOME_DIR/submission"
echo "[setup] givens staged in $HOME_DIR (owner $AGENT_USER)" >&2

# The commissioning service: the cell on its practice row sections (never a graded row). The image installs it under
# /opt/rowcell-commission for the account `rowcell` only; setup starts it now, to run through the agent's session,
# under a loop that restarts it should it ever exit. The agent reaches it through the socket
# /run/rowcell/commission.sock (given/lib/rowcell_client.py, ~/commission/run_session.py): the cell's public actions
# on a fixed number of sessions, one at a time; nothing of where anything is, no yield. A session starts Isaac Sim on
# the sandbox's GPU; the first one also compiles the renderer's shaders (a few minutes).
SERVICE=/opt/rowcell-commission
[ -f "$SERVICE/commissiond.py" ] || { echo "[setup] the image lacks the commissioning service ($SERVICE)" >&2; exit 1; }
install -d -o rowcell -g rowcell -m 755 /run/rowcell
rm -f /run/rowcell/commission.sock
setsid runuser -u rowcell -- env HOME=/var/lib/rowcell ROWCELL_STATE_DIR="$SERVICE/state" \
    OMNI_KIT_ACCEPT_EULA=YES ACCEPT_EULA=Y PRIVACY_CONSENT=Y OPENBLAS_CORETYPE=Haswell OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
    NVIDIA_DRIVER_CAPABILITIES="${NVIDIA_DRIVER_CAPABILITIES:-}" NVIDIA_VISIBLE_DEVICES="${NVIDIA_VISIBLE_DEVICES:-}" \
    /bin/bash -c 'while :; do /opt/venv/bin/python3 -I -s /opt/rowcell-commission/commissiond.py >> /var/lib/rowcell/commission.log 2>&1; sleep 2; done' \
    < /dev/null > /dev/null 2>&1 &
for _ in $(seq 1 60); do [ -S /run/rowcell/commission.sock ] && break; sleep 0.5; done
[ -S /run/rowcell/commission.sock ] || { echo "[setup] the commissioning service did not come up" >&2; exit 1; }
echo "[setup] commissioning service listening on /run/rowcell/commission.sock" >&2

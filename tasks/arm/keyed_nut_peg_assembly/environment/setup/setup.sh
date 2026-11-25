#!/usr/bin/env bash
# Setup — runs as trusted root, cwd = this staged folder (/opt/ale/setup, root-only), before the agent
# phase: stage the agent-visible payload (the cell's data sheet, the commissioning client and runner, the
# wire helper) into the agent home, and start the commissioning service.
set -euo pipefail
umask 022
export HOME=/root PYTHONNOUSERSITE=1
unset PYTHONPATH PYTHONSTARTUP

HOME_DIR="${ALE_HOME:-/home/user}"
AGENT_USER="$(basename "$HOME_DIR")"

# cp -r, NOT -a: with -a the `payload/.` form also copies the payload directory's own root ownership
# onto the agent home, locking the agent out of its own workspace.
cp -r payload/. "$HOME_DIR/"
for entry in payload/* payload/.[!.]*; do
  [ -e "$entry" ] || continue
  chown -R "$AGENT_USER:$AGENT_USER" "$HOME_DIR/$(basename "$entry")"
done
mkdir -p "$HOME_DIR/submission" && chown "$AGENT_USER:$AGENT_USER" "$HOME_DIR/submission"

# The dual mode (experiment): the graded episode's service (online/graded_service.py), ONE episode of the grader's
# hidden set (online/episode.json) that the agent plays itself through ~/commission/client.py's connect_graded(). It
# runs the grader's own cell: online/cell/ holds byte copies of the grader's simulator, cameras and credit rule, which
# the worker checks against online/service.json's digests and imports as root. Started as root, it reads the episode,
# builds it, then serves as its own account `keyedgrade` (environment/Dockerfile: not the commissioning service's
# `keyedcell`) through /run/keyedcell-graded/graded.sock and writes the episode's record, and the submission folder
# frozen at the graded start (or at the close of the start window), to /var/lib/arb-online (owner keyedgrade, 0700),
# which task.toml declares an artifact and tests/verify.py grades. The command returns once the service serves.
# Practice ends when the graded episode starts (O13): the start writes /run/keyedcell-graded/flags/started, and the
# commissioning service below ends its episodes and refuses new ones once that flag exists; checked here: the
# commissioning account sees the flag's directory and the flag is not set.
GRADED="$PWD/online"
id -u keyedgrade > /dev/null 2>&1 || { echo "[setup] the image lacks the account keyedgrade" >&2; exit 1; }
chown -R root:root "$GRADED"
chmod -R go-rwx "$GRADED"
env -i PATH=/opt/venv/bin:/usr/bin:/bin LANG=C.UTF-8 HOME=/nonexistent \
    /opt/venv/bin/python3 -I -B "$GRADED/graded_service.py" --config "$GRADED/service.json" < /dev/null > /dev/null 2>&1 \
  || { echo "[setup] the graded episode's service did not come up (/var/lib/arb-online/service.log)" >&2; exit 1; }
[ -S /run/keyedcell-graded/graded.sock ] || { echo "[setup] the graded episode's service has no socket" >&2; exit 1; }
runuser -u keyedcell -- test -d /run/keyedcell-graded/flags -a -x /run/keyedcell-graded/flags \
    -a ! -e /run/keyedcell-graded/flags/started \
  || { echo "[setup] the practice flag's directory is not as the commissioning service needs it" >&2; exit 1; }
echo "[setup] graded episode's service listening on /run/keyedcell-graded/graded.sock" >&2

# The commissioning service: the cell on its development kits (never a graded kit). The image installs
# it under /opt/keyedcell-commission for the account `keyedcell` only (the agent account can neither read
# nor list it); setup starts it now, to run through the agent's session, under a loop that restarts it
# should it ever exit. The agent reaches it through the socket /run/keyedcell/commission.sock
# (~/commission/client.py, ~/commission/run_kit.py): reset(kit), step(action), close(), a fixed number of
# episodes; no ground truth, no score, no completion verdict. It runs the cell with the settings the
# grader's cell runs with (the image's rendering and thread settings). A separate verifier never runs
# this script, so no service runs where the controller is graded.
SERVICE=/opt/keyedcell-commission
[ -f "$SERVICE/commissiond.py" ] || { echo "[setup] the image lacks the commissioning service ($SERVICE)" >&2; exit 1; }
install -d -o keyedcell -g keyedcell -m 755 /run/keyedcell
rm -f /run/keyedcell/commission.sock
setsid runuser -u keyedcell -- env -i HOME="$SERVICE/state" TMPDIR="$SERVICE/state" \
    PATH=/opt/venv/bin:/usr/bin:/bin LANG=C.UTF-8 MUJOCO_GL=egl PYOPENGL_PLATFORM=egl OMP_NUM_THREADS=1 \
    LP_NUM_THREADS=2 MESA_SHADER_CACHE_DISABLE=true PYTHONNOUSERSITE=1 \
    /bin/bash -c 'cd /opt/keyedcell-commission/state && while :; do /opt/venv/bin/python3 -B -s /opt/keyedcell-commission/commissiond.py; sleep 2; done' \
    < /dev/null > /dev/null 2>&1 &
for _ in $(seq 1 60); do [ -S /run/keyedcell/commission.sock ] && break; sleep 0.5; done
[ -S /run/keyedcell/commission.sock ] || { echo "[setup] the commissioning service did not come up" >&2; exit 1; }
echo "[setup] commissioning service listening on /run/keyedcell/commission.sock" >&2

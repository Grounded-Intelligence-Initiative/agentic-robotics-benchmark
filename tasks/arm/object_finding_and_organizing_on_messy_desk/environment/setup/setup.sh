#!/usr/bin/env bash
# Setup — runs as trusted root, cwd = this staged folder (/opt/ale/setup, root-only while the agent works),
# before the agent phase.
#
# (1) Put the agent-visible material (the desk lab, its client, the graded episode's client, the task card,
#     the example controller) into the agent home, owned by the agent. Everything under payload/ is staged,
#     whatever it is called.
# (2) Start the graded episode service (online/server.py): it reads the hidden episode (online/episode.json)
#     as root, drops to the dedicated `arbonline` account and serves /run/arb-online/online.sock (fixed
#     argv, clean environment). On this first start it warms its simulator's shader cache, so this script
#     returns only once the service serves (up to about 25 minutes on a cold GPU), or fails setup.
# Package installation and fixed downloads belong in environment/Dockerfile, never here.
set -euo pipefail
umask 022
export HOME=/root PYTHONNOUSERSITE=1
unset PYTHONPATH PYTHONSTARTUP

HOME_DIR="${ALE_HOME:-/home/user}"
AGENT_USER="$(basename "$HOME_DIR")"

# cp -r, NOT -a: with -a the `payload/.` form also copies the payload directory's own
# root ownership onto the agent home, locking the agent out of its own workspace.
cp -r payload/. "$HOME_DIR/"

# chown every top-level entry that came from payload/ (recursively) and nothing else: the
# home itself keeps the ownership the image gave it.
for entry in payload/* payload/.[!.]*; do
  [ -e "$entry" ] || continue
  chown -R "$AGENT_USER:$AGENT_USER" "$HOME_DIR/$(basename "$entry")"
done

umask 077
env -i PATH=/opt/venv/bin:/usr/local/bin:/usr/bin:/bin LANG=C.UTF-8 HOME=/nonexistent \
  python3 -I -B "$PWD/online/server.py" --config "$PWD/online/online.json" </dev/null >/dev/null

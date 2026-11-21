#!/usr/bin/env bash
# Setup: runs as trusted root, cwd = this staged folder (/opt/ale/setup, root-only), before the agent phase.
# (1) Stage the agent-visible payload into the agent home. (2) Start the lab service that holds the practice
# cell: it loads lab/pallet_cell.py and lab/practice.json as root, drops to the dedicated `alelab` account and
# serves /run/ale-lab/lab.sock (fixed argv, clean environment). The simulator's source and the practice
# cartons stay in this root-only folder: the agent meets the cell only through the lab's observations.
# The lab installs its socket filter (AF_UNIX only) where the kernel takes it and serves without it where it does
# not (lab.json require_seccomp = false): it holds the practice cell, nothing that is scored, and the sandbox has
# no network either way.
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

umask 077
# Returns once the lab serves (the daemon is detached, stdio on /dev/null) or fails setup.
env -i PATH=/opt/venv/bin:/usr/local/bin:/usr/bin:/bin LANG=C.UTF-8 HOME=/nonexistent \
  python3 -I -B "$PWD/lab/alelab/server.py" --config "$PWD/lab/lab.json" </dev/null >/dev/null 2>&1

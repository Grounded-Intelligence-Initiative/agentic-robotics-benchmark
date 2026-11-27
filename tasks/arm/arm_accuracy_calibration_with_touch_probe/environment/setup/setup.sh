#!/usr/bin/env bash
# Setup: runs as trusted root, cwd = this staged folder (/opt/ale/setup, root-only), before the agent phase.
# (1) Stage the agent-visible payload into the agent home. (2) Start the lab service that holds the units: it
# loads lab/calib_plugin.py with lab/calib_cell.py, lab/cell_camera.py, lab/cell_data.py, lab/arm_model.py,
# lab/submission_format.py and lab/units.json as root, drops to the dedicated `alelab` account
# and serves /run/ale-lab/lab.sock (fixed argv, clean environment). The cell's simulator and the units' seeds
# stay in this root-only folder: the agent meets a unit only through the lab's observations.
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

# Every unit's acceptance conditions are public: written here from the hidden cell (the conditions depend on
# the unit's seed, which stays in this folder), one file per unit, for the agent to predict on.
mkdir -p "$HOME_DIR/conditions"
env -i PATH=/opt/venv/bin:/usr/local/bin:/usr/bin:/bin LANG=C.UTF-8 HOME=/nonexistent OMP_NUM_THREADS=1 \
  python3 -I -B "$PWD/lab/write_conditions.py" "$PWD/lab" "$HOME_DIR/conditions"
chown -R "$AGENT_USER:$AGENT_USER" "$HOME_DIR/conditions"

umask 077
# Returns once the lab serves (the daemon is detached, stdio on /dev/null) or fails setup.
env -i PATH=/opt/venv/bin:/usr/local/bin:/usr/bin:/bin LANG=C.UTF-8 HOME=/nonexistent \
  python3 -I -B "$PWD/lab/alelab/server.py" --config "$PWD/lab/lab.json" </dev/null >/dev/null 2>&1

#!/usr/bin/env bash
# Setup: trusted root, cwd = this staged folder (/opt/ale/setup, root-only while the agent works), before the agent
# phase. Two jobs:
# (1) The givens: payload/ into the agent home, owned by the agent, plus what the image holds: the robot model
#     (/opt/arb-assets/unitree_g1 -> ~/given/robot/unitree_g1) and the survey photos (/opt/arb-assets/survey ->
#     ~/given/cell/survey). tests/setup_payload/ is a byte copy of payload/ (change both together, then
#     tests/given_hashes.json).
# (2) The graded episode's cell service (online/service.py, the online variant): it reads the hidden graded episode
#     (online/episode.json) as root, builds the episode's house cell (online/house_cell, a byte copy of the verifier's
#     tests/house_cell), drops to the dedicated `arbcell` account and serves /run/arb-online/cell.sock; the episode's
#     record goes to /var/lib/arb-online (the lab account's, 0700: the agent can neither read nor write it; a declared
#     artifact, scored by tests/verify.py in the separate verifier). Dual mode: at the graded start, before it is
#     charged, the service's root supervisor freezes the agent's program (/home/user/submission ->
#     /var/lib/arb-online/submission_snapshot), the copy the verifier grades as the offline item. Fixed argv, a clean
#     environment, no hidden value outside this folder. The command returns once the service serves (or fails setup).
# There is no practice cell: the service serves the one graded episode and nothing else.
set -euo pipefail
umask 022
export HOME=/root PYTHONNOUSERSITE=1
unset PYTHONPATH PYTHONSTARTUP
HOME_DIR="${ALE_HOME:-/home/user}"
AGENT_USER="$(basename "$HOME_DIR")"
G1_DIR="${ARB_G1_DIR:-/opt/arb-assets/unitree_g1}"
SURVEY_DIR="${ARB_SURVEY_DIR:-/opt/arb-assets/survey}"
LAB_USER=arbcell

# cp -r, NOT -a: with -a the `payload/.` form also copies the payload directory's own root ownership onto the agent
# home, locking the agent out of its own workspace. Every top-level entry that came from payload/ is chowned
# (recursively), nothing else: the home itself keeps the ownership the image gave it.
cp -r payload/. "$HOME_DIR/"
for entry in payload/* payload/.[!.]*; do
  [ -e "$entry" ] || continue
  chown -R "$AGENT_USER:$AGENT_USER" "$HOME_DIR/$(basename "$entry")"
done
[ -d "$G1_DIR" ] || { echo "[setup] $G1_DIR missing: the image was built without the robot model" >&2; exit 1; }
[ -d "$SURVEY_DIR" ] || { echo "[setup] $SURVEY_DIR missing: the image was built without the survey photos" >&2; exit 1; }
mkdir -p "$HOME_DIR/given/robot"
rm -rf "$HOME_DIR/given/robot/unitree_g1" "$HOME_DIR/given/cell/survey"
cp -r "$G1_DIR" "$HOME_DIR/given/robot/unitree_g1"
cp -r "$SURVEY_DIR" "$HOME_DIR/given/cell/survey"
chown -R "$AGENT_USER:$AGENT_USER" "$HOME_DIR/given"
mkdir -p "$HOME_DIR/submission" && chown "$AGENT_USER:$AGENT_USER" "$HOME_DIR/submission"
echo "[setup] givens staged in $HOME_DIR (owner $AGENT_USER): $(find "$HOME_DIR/given/cell/survey" -type f | wc -l) survey files" >&2

# (2) the graded episode's cell service
id -u "$LAB_USER" >/dev/null 2>&1 || { echo "[setup] the account $LAB_USER is missing from the image" >&2; exit 1; }
install -d -o root -g root -m 0755 /run/arb-online
install -d -o "$LAB_USER" -g "$LAB_USER" -m 0700 /var/lib/arb-online
chmod 0700 online
chmod 0600 online/episode.json
umask 077
# Returns once the service serves (the daemon is detached, its stdio on /dev/null) or fails setup.
env -i PATH=/opt/venv/bin:/usr/local/bin:/usr/bin:/bin LANG=C.UTF-8 HOME=/nonexistent \
  /opt/venv/bin/python3 -I -B "$PWD/online/service.py" --config "$PWD/online/online.json" </dev/null
echo "[setup] the graded episode's cell service is up (/run/arb-online/cell.sock)" >&2

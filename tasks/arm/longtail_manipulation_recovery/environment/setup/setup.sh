#!/usr/bin/env bash
# Setup (root): stage the agent's kit (practice environment, practice failure states, practice grader, the graded
# episode's client and runner) into its home, and start the graded episode's service (dual mode, below).
set -euo pipefail
umask 077
export HOME=/root PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1
unset PYTHONPATH PYTHONSTARTUP
cd "$(dirname "$0")"
HOME_DIR="${ALE_HOME:-/home/user}"
AGENT_USER="$(basename "$HOME_DIR")"
(cd payload && find . -type d | LC_ALL=C sort | while read -r d; do mkdir -p "$HOME_DIR/$d"; done)
(cd payload && find . -type f | LC_ALL=C sort | while read -r f; do cp "$f" "$HOME_DIR/$f"; done)
for entry in payload/*; do
  [ -e "$entry" ] || continue
  tgt="$HOME_DIR/$(basename "$entry")"
  chown -R "$AGENT_USER:$AGENT_USER" "$tgt"
  find "$tgt" -type d -exec chmod 0755 {} + ; find "$tgt" -type f -exec chmod 0644 {} +
  find "$tgt" -exec touch -h -d '2026-01-01 00:00:00 UTC' {} +
done
mkdir -p "$HOME_DIR/submission" && chown "$AGENT_USER:$AGENT_USER" "$HOME_DIR/submission"

# The graded episode's service (dual mode, experiment; online/graded_service.py): ONE failure state of the grader's
# hidden set (online/episode.json and online/state/: byte copies of the state's tests/hidden/ files), which the agent
# plays itself through ~/graded/client.py, on the grader's own kitchen process (online/code/: byte copies of
# tests/sim_worker.py, kitchen_sim.py and wire_rules.py) in the image's second RoboCasa tree with the graded kitchens'
# assets (/opt/longtail-graded/robocasa, the account longgrade's: environment/Dockerfile). Setup copies the code and
# the state's files into /opt/longtail-graded (root-owned, group longgrade, readable by that group only), then starts
# the service as root; it checks every file against online/service.json's digests and serves as `longgrade` through
# /run/longgrade/graded.sock, and writes the episode's record to /var/lib/arb-online (owner longgrade, 0700), which
# task.toml declares an artifact and tests/verify.py scores; at the graded start (or when the start window closes) its
# root supervisor freezes /home/user/submission into that directory. The kitchen itself is built when the agent starts
# the episode (it holds about 6 GB). The command returns once the service serves.
G=/opt/longtail-graded
id -u longgrade > /dev/null 2>&1 || { echo "[setup] the image lacks the account longgrade" >&2; exit 1; }
[ -f "$G/robocasa/robocasa/__init__.py" ] || { echo "[setup] the image lacks the graded kitchens' RoboCasa tree" >&2; exit 1; }
for d in code state; do
  rm -rf "${G:?}/$d"
  install -d -o root -g longgrade -m 750 "$G/$d"
  for f in online/"$d"/*; do install -o root -g longgrade -m 640 "$f" "$G/$d/"; done
done
install -d -o longgrade -g longgrade -m 700 "$G/home"
chown -R root:root online
chmod -R go-rwx online
env -i PATH=/opt/venv/bin:/usr/bin:/bin LANG=C.UTF-8 HOME=/nonexistent \
    /opt/venv/bin/python3 -I -B online/graded_service.py --config online/service.json < /dev/null > /dev/null 2>&1 \
  || { echo "[setup] the graded episode's service did not come up (/var/lib/arb-online/service.log)" >&2; exit 1; }
[ -S /run/longgrade/graded.sock ] || { echo "[setup] the graded episode's service has no socket" >&2; exit 1; }

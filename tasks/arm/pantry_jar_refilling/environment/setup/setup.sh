#!/usr/bin/env bash
# Setup — runs as trusted root, cwd = this staged folder (/opt/ale/setup, root-only), before the agent phase:
# stage the agent-visible payload (the simulator, the practice units, the practice grader, the wire skeleton, the
# session-time script, the graded episode's client) into the agent home, write the session's time stamp, and start the
# graded episode's service (dual mode). The only hidden material in this folder is online/episode.json (the graded
# episode), which only that service's worker reads.
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

# The session's time stamp (session_time.sh in the agent's home reads it): written now, a few minutes before the
# agent's session starts, in a folder that is root's, so that the agent's account can read it and cannot change or
# replace it (a root-owned file in the agent's own home could be renamed away and written anew).
install -d -o root -g root -m 755 /run/session
date -u +%s > /run/session/start
chown root:root /run/session/start && chmod 0444 /run/session/start

# The dual mode (experiment, never merged into main): the graded episode's service (online/graded_service.py), ONE
# episode of the grader's hidden set (online/episode.json) that the agent plays itself through ~/graded/client.py's
# connect_graded(). It runs the grader's own simulator: the payload's jar_sim.py, jar_render.py, jar_look.json,
# jar_env.py and episode.py (byte copies of the grader's), copied from this root-only stage into online/cell/; the
# worker checks them against online/service.json's digests and imports them as root. Started as root, it reads the
# episode, builds it, then serves as its own account `jargrade` (created here, so that the task image stays main's)
# through /run/jarcell-graded/graded.sock and writes the episode's record, and the submission folder frozen at the
# graded start (or at the close of the start window), to /var/lib/arb-online (owner jargrade, 0700), which task.toml
# declares an artifact and tests/verify.py grades. It starts after the session's time stamp, so that session_time.sh
# never shows more of the start window than the service grants. The command returns once the service serves.
GRADED="$PWD/online"
id -u jargrade > /dev/null 2>&1 || useradd --system --user-group --no-create-home --home-dir /nonexistent \
    --shell /usr/sbin/nologin jargrade
rm -rf "$GRADED/cell"
install -d -o root -g root -m 700 "$GRADED/cell"
cp payload/jar_sim.py payload/jar_render.py payload/jar_look.json payload/jar_env.py payload/episode.py "$GRADED/cell/"
chown -R root:root "$GRADED"
chmod -R go-rwx "$GRADED"
env -i PATH=/opt/venv/bin:/usr/bin:/bin LANG=C.UTF-8 HOME=/nonexistent \
    /opt/venv/bin/python3 -I -B "$GRADED/graded_service.py" --config "$GRADED/service.json" < /dev/null > /dev/null 2>&1 \
  || { echo "[setup] the graded episode's service did not come up (/var/lib/arb-online/service.log)" >&2; exit 1; }
[ -S /run/jarcell-graded/graded.sock ] || { echo "[setup] the graded episode's service has no socket" >&2; exit 1; }
echo "[setup] graded episode's service listening on /run/jarcell-graded/graded.sock" >&2

#!/usr/bin/env bash
# Setup — runs as trusted root, cwd = this staged folder (/opt/ale/setup, root-only), before the agent phase:
# stage the agent-visible payload (the equipment's documents, the commissioning client and runner, the graded
# episode's client and runner, the wire skeleton, the session-time script) into the agent home, write the session's
# time stamp, and start the graded episode's service and the commissioning service. Dual mode (experiment): online/
# holds the graded episode (online/episode.json: one hidden episode, its unit's properties and its seed), which
# stays root-only here while the agent works; the rest of this folder holds nothing hidden.
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

# The session's time stamp (session_time.sh in the agent's home reads it): written now, moments before the agent's
# session starts, in a folder that is root's, so that the agent's account can read it and cannot change or replace it.
install -d -o root -g root -m 755 /run/session
date -u +%s > /run/session/start
chown root:root /run/session/start && chmod 0444 /run/session/start

# The graded episode's service (dual mode, experiment; online/graded_service.py): ONE episode of the grader's hidden
# set (online/episode.json), which the agent plays itself through ~/graded/client.py. It runs the grader's own
# simulator: the image's byte copies under /opt/labbench-commission (lab_sim.py, lab_env.py, box_transport_env.py,
# lab_render.py, lab_look.json) and online/episode.py, copied here into online/sim/, root-only, before any other
# account runs (the worker checks them against online/service.json's digests and imports them as root). Started as
# root, it builds the episode, then serves as its own account `labgrade` (environment/Dockerfile) through
# /run/labgrade/graded.sock and writes the episode's record to /var/lib/arb-online (owner labgrade, 0700), which
# task.toml declares an artifact and tests/verify.py scores; at the graded start (or when the start window closes) its
# root supervisor freezes /home/user/submission into that directory. The command returns once the service serves.
# The commissioning service closes when the graded episode starts (O13): the start writes /run/labgrade/flags/started,
# and the commissioning service below ends its benches and refuses every request once that flag exists; checked here:
# the commissioning account sees the flag's directory and the flag is not set.
GRADED="$PWD/online"
id -u labgrade > /dev/null 2>&1 || { echo "[setup] the image lacks the account labgrade" >&2; exit 1; }
rm -rf "$GRADED/sim"
install -d -o root -g root -m 700 "$GRADED/sim"
for f in lab_sim.py lab_env.py box_transport_env.py lab_render.py lab_look.json; do
  cp "/opt/labbench-commission/$f" "$GRADED/sim/$f"
done
cp "$GRADED/episode.py" "$GRADED/sim/episode.py"
chown -R root:root "$GRADED"
chmod -R go-rwx "$GRADED"
env -i PATH=/opt/venv/bin:/usr/bin:/bin LANG=C.UTF-8 HOME=/nonexistent \
    /opt/venv/bin/python3 -I -B "$GRADED/graded_service.py" --config "$GRADED/service.json" < /dev/null > /dev/null 2>&1 \
  || { echo "[setup] the graded episode's service did not come up (/var/lib/arb-online/service.log)" >&2; exit 1; }
[ -S /run/labgrade/graded.sock ] || { echo "[setup] the graded episode's service has no socket" >&2; exit 1; }
runuser -u labbench -- test -d /run/labgrade/flags -a -x /run/labgrade/flags -a ! -e /run/labgrade/flags/started \
  || { echo "[setup] the flag's directory is not as the commissioning service needs it" >&2; exit 1; }

# The commissioning service (environment/commission_service, installed by the image at /opt/labbench-commission,
# readable by its own account only): started here, as that account, with a fixed environment and PATH (no
# agent-writable directory on it), detached, restarted if it ever exits. It is not started in the grading sandbox:
# this script does not run there. It closes for the rest of the session when the graded episode starts (above).
SERVICE=/opt/labbench-commission
install -d -o labbench -g labbench -m 755 /run/labbench
rm -f /run/labbench/commission.sock
setsid runuser -u labbench -- /usr/bin/env -i HOME="$SERVICE/state" TMPDIR="$SERVICE/state" \
  PATH=/opt/venv/bin:/usr/bin:/bin LANG=C.UTF-8 MUJOCO_GL=osmesa PYOPENGL_PLATFORM=osmesa OMP_NUM_THREADS=1 \
  LP_NUM_THREADS=2 MESA_SHADER_CACHE_DISABLE=true PYTHONDONTWRITEBYTECODE=1 \
  /bin/bash -c 'cd /opt/labbench-commission/state && while :; do /opt/venv/bin/python3 -B -s /opt/labbench-commission/commissiond.py; /usr/bin/sleep 2; done' \
  < /dev/null > /dev/null 2>&1 &
for _ in $(seq 1 60); do [ -S /run/labbench/commission.sock ] && break; sleep 0.5; done
[ -S /run/labbench/commission.sock ] || { echo "[setup] the commissioning service did not come up" >&2; exit 1; }

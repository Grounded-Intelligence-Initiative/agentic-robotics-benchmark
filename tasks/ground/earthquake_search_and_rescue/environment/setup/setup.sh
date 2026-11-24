#!/usr/bin/env bash
# Setup: trusted root, cwd = this staged folder (/opt/ale/setup, root-only while the agent works), before the agent
# phase. Stages the givens (the robot's documents, the client library, the example program, the training-ground launcher
# with the grader's submission checks, the session-time script) into the agent home, owned by the agent, writes the
# session's time stamp, and starts the graded episode's service and the training ground (below). The grader never reads
# these copies: verification is separate (task.toml), and tests/test.sh builds the grader's own ROOT-OWNED read-only
# copy of the same givens in the fresh verifier sandbox, from tests/setup_payload/ (a byte copy of payload/ here: change
# both together, then tests/given_hashes.json).
# Dynamic-only: packages, the training ground's program and the graded service's account live in environment/Dockerfile.
set -euo pipefail
HOME_DIR="${ALE_HOME:-/home/user}"
AGENT_USER="$(basename "$HOME_DIR")"
cp -r payload/. "$HOME_DIR/"
for entry in payload/* payload/.[!.]*; do
  [ -e "$entry" ] || continue
  chown -R "$AGENT_USER:$AGENT_USER" "$HOME_DIR/$(basename "$entry")"
done
mkdir -p "$HOME_DIR/submission" && chown "$AGENT_USER:$AGENT_USER" "$HOME_DIR/submission"
echo "[setup] givens staged in $HOME_DIR (owner $AGENT_USER)" >&2

# The session's time stamp (session_time.sh in the agent's home reads it): written now, moments before the agent's
# session starts and before the graded episode's service below starts its start window, in a folder that is root's, so
# that the agent's account can read it and cannot change or replace it.
/usr/bin/install -d -o root -g root -m 755 /run/session
/bin/date -u +%s > /run/session/start
/bin/chown root:root /run/session/start && /bin/chmod 0444 /run/session/start

# The dual mode (experiment): the graded episode's service (online/graded_service.py), ONE mission of the grader's hidden
# set (online/episode.json) that the agent drives itself through rubblecell.client.connect_graded(). It runs the
# grader's own cell: the image's byte copy of tests/rubblecell/ (/opt/rubblecell-training/rubblecell/), copied here,
# root-only, before any other account runs (the worker checks it against online/service.json's digests and imports it
# as root). Started as root, it reads the episode and builds its site, then serves as its own account `rubblegrade`
# (environment/Dockerfile: not the training ground's `rubblecell`) through /run/rubblecell-graded/graded.sock and
# writes the episode's record to /var/lib/arb-online (owner rubblegrade, 0700), which task.toml declares an artifact and
# tests/verify.py scores. At the earlier of the graded start and the close of its start window, its root supervisor
# freezes the program: /home/user/submission is copied into /var/lib/arb-online/submission_snapshot, the copy the
# verifier grades as the program item. The command returns once the service serves. Practice ends when the graded
# episode starts (O13): the start writes /run/rubblecell-graded/flags/started, and the training ground below ends its
# missions and refuses new ones once that flag exists; checked here: the training ground's account sees the flag's
# directory and the flag is not set.
GRADED="$PWD/online"
id -u rubblegrade > /dev/null 2>&1 || { echo "[setup] the image lacks the account rubblegrade" >&2; exit 1; }
rm -rf "$GRADED/rubblecell"
/usr/bin/install -d -o root -g root -m 700 "$GRADED/rubblecell"
cp /opt/rubblecell-training/rubblecell/*.py /opt/rubblecell-training/rubblecell/*.json "$GRADED/rubblecell/"
chown -R root:root "$GRADED"
chmod -R go-rwx "$GRADED"
/usr/bin/env -i PATH=/opt/venv/bin:/usr/bin:/bin LANG=C.UTF-8 HOME=/nonexistent \
    /opt/venv/bin/python3 -I -B "$GRADED/graded_service.py" --config "$GRADED/service.json" < /dev/null > /dev/null 2>&1 \
  || { echo "[setup] the graded episode's service did not come up (/var/lib/arb-online/service.log)" >&2; exit 1; }
[ -S /run/rubblecell-graded/graded.sock ] || { echo "[setup] the graded episode's service has no socket" >&2; exit 1; }
/usr/sbin/runuser -u rubblecell -- /usr/bin/test -d /run/rubblecell-graded/flags -a -x /run/rubblecell-graded/flags \
    -a ! -e /run/rubblecell-graded/flags/started \
  || { echo "[setup] the practice flag's directory is not as the training ground needs it" >&2; exit 1; }
echo "[setup] graded episode's service listening on /run/rubblecell-graded/graded.sock" >&2

# The training ground: the image installs it under /opt/rubblecell-training for the account `rubblecell` only (the
# agent account can neither read nor list it); setup starts it now, to run through the agent's session, under a loop
# that restarts it should it ever exit. The agent reaches it through /run/rubblecell/training.sock
# (rubblecell.client.connect_training, ~/training/run_mission.py): the robot's public calls, reset(site) and each
# mission's log (the robot's own: no truth of the site, no score; also kept, world-readable, in /run/rubblecell/logs/).
# It closes for the rest of the session once the graded episode starts (O13: the flag above).
# Every program in the restart loop is named by its absolute path: /usr/local/bin is the agent account's in the base
# image and comes first on PATH.
SERVICE=/opt/rubblecell-training
[ -f "$SERVICE/trainingd.py" ] || { echo "[setup] the image lacks the training ground ($SERVICE)" >&2; exit 1; }
/usr/bin/install -d -o rubblecell -g rubblecell -m 755 /run/rubblecell
/bin/rm -f /run/rubblecell/training.sock
/usr/bin/setsid /usr/sbin/runuser -u rubblecell -- /usr/bin/env -i PATH=/usr/bin:/bin HOME="$SERVICE" MUJOCO_GL=osmesa \
    PYOPENGL_PLATFORM=osmesa OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 LP_NUM_THREADS=2 OPENBLAS_CORETYPE=Haswell \
    OPENBLAS_L2_SIZE=256 NPY_DISABLE_CPU_FEATURES="X86_V4 AVX512_ICL AVX512_SPR" RUBBLECELL_LOOK_DIR=/opt/rubblecell-look \
    /bin/bash -c 'while :; do /opt/venv/bin/python3 -s /opt/rubblecell-training/trainingd.py; /bin/sleep 2; done' \
    < /dev/null > /dev/null 2>&1 &
for _ in $(/usr/bin/seq 1 60); do [ -S /run/rubblecell/training.sock ] && break; /bin/sleep 0.5; done
[ -S /run/rubblecell/training.sock ] || { echo "[setup] the training ground did not come up" >&2; exit 1; }
echo "[setup] training ground listening on /run/rubblecell/training.sock" >&2

#!/usr/bin/env bash
# Setup: trusted root, cwd = this staged folder, before the agent phase. Three jobs:
#   1. the givens: the robot's manual and API, the station's handbook, telemetry dictionary, lost-item register and
#      rehearsal guide, the station map, the telemetry sample, the client library, the example program and the
#      rehearsal tool, staged into the agent home, owned by the agent. The pictures of the givens (the figures, the
#      catalogue's photographs and the map views) are task assets: they come from the image (/opt/task-assets/payload,
#      environment/Dockerfile). The grader never reads these copies: verification is separate (task.toml), and
#      tests/test.sh builds the grader's own ROOT-OWNED read-only copy of the same givens in the fresh verifier
#      sandbox, from the image's task assets and tests/setup_payload/ (a byte copy of payload/ here: change both
#      together, then tests/given_hashes.json);
#   2. the rehearsal facility (below), which closes when the graded episode starts (REHEARSAL_CLOSED_FLAG);
#   3. the graded episode's service (the dual mode): online/graded_service.py, started as root from this root-only
#      folder once its data folder is installed (online/data -> /opt/arb-graded, the service account's, mode 0700: the
#      offline grader's own cell, a byte copy of tests/cabincell with tests/grader_config.json, tests/reference and the
#      station map; the drawn site's package; the hidden episode file; the oracle token's digest). Its worker reads the
#      hidden episode as root, drops to `arbgraded` irreversibly (with the task assets' group), builds the graded site's
#      cell and serves it on /run/arb-graded/graded.sock (a root-owned directory). The episode's record goes to
#      /var/lib/arb-online (the service account's, mode 0700, in the root-owned /var/lib: the agent can neither read nor
#      write it), which task.toml declares as an artifact. When the graded episode starts (before the start is
#      charged), or when its preparation window closes without a start, the service's root supervisor freezes the
#      agent's program: a copy of ~/submission in /var/lib/arb-online/submission_snapshot, which the separate verifier
#      grades as main grades a submission.
# Dynamic-only: packages, the rehearsal facility's program, the service account and the task assets live in
# environment/Dockerfile.
set -euo pipefail
HOME_DIR="${ALE_HOME:-/home/user}"
AGENT_USER="$(basename "$HOME_DIR")"
ASSETS=/opt/task-assets/payload
[ -d "$ASSETS/given" ] || { echo "[setup] $ASSETS/given missing: the image was built without its task assets" >&2; exit 1; }
cp -r payload/. "$HOME_DIR/"
cp -r "$ASSETS/." "$HOME_DIR/"
for entry in payload/* payload/.[!.]* "$ASSETS"/*; do
  [ -e "$entry" ] || continue
  chown -R "$AGENT_USER:$AGENT_USER" "$HOME_DIR/$(basename "$entry")"
done
chmod -R u+rwX,go+rX,go-w "$HOME_DIR/given"     # the image's copy is root-only (750 / 640): the agent's copy is not
mkdir -p "$HOME_DIR/submission" && chown "$AGENT_USER:$AGENT_USER" "$HOME_DIR/submission"
echo "[setup] givens staged in $HOME_DIR (owner $AGENT_USER)" >&2

# The rehearsal facility: the image installs it under /opt/rehearsal for the account
# `rehearsal` only (mode 700: the agent account can neither read nor list it); setup starts it now, to run through
# the agent's session, under a loop that restarts it should it ever exit (its allowance survives a restart). The agent
# reaches it through /run/rehearsal/rehearsal.sock (~/rehearsal/rehearse.py): the robot's public calls, reset(card)
# and each run's observation sheet; /run/rehearsal/status.json and /run/rehearsal/debriefs/ are world-readable. The
# cell's asset root is the image's one copy (/opt/task-assets, the root tests/test.sh names for the grader), checked
# here first against the pins of the facility's copy of the cell (a broken image fails the setup, never a rehearsal
# run without its model files; the cell checks the same again at every build); its numerics are pinned as the grader
# pins them (runtime key cell.recipe.env, tests/test.sh): one BLAS / OpenMP thread, the CPU paths of numpy and OpenBLAS
# (the forced kernel always with OPENBLAS_L2_SIZE=256, as tests/test.sh sets it), 4 rasteriser threads (two runs may be
# open at a time on the sandbox's 8 CPUs). These variables are set here only, not in the image's ENV: the agent's shell
# does not show them. REHEARSAL_CLOSED_FLAG: the file the graded episode's service creates when the graded episode
# starts; the facility then ends an open run and refuses new ones (practice ends when the graded episode starts).
SERVICE=/opt/rehearsal
ASSET_ROOT=/opt/task-assets
GRADED_DIR=/run/arb-graded
CLOSED_FLAG="$GRADED_DIR/flags/practice_closed"
[ -f "$SERVICE/practiced.py" ] || { echo "[setup] the image lacks the rehearsal facility ($SERVICE)" >&2; exit 1; }
runuser -u rehearsal -- env CABINCELL_ASSET_ROOT="$ASSET_ROOT" /opt/venv/bin/python3 -I -B "$SERVICE/cabincell/assetroot.py" \
    robot station robot_author > /dev/null \
  || { echo "[setup] the task assets at $ASSET_ROOT are missing, altered or not the pinned ones" >&2; exit 1; }
install -d -o rehearsal -g rehearsal -m 755 /run/rehearsal
rm -f /run/rehearsal/rehearsal.sock
setsid runuser -u rehearsal -- env HOME="$SERVICE" CABINCELL_ASSET_ROOT="$ASSET_ROOT" MUJOCO_GL=osmesa PYOPENGL_PLATFORM=osmesa \
    PYTHONDONTWRITEBYTECODE=1 OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 LP_NUM_THREADS=4 \
    OPENBLAS_CORETYPE=Haswell OPENBLAS_L2_SIZE=256 NPY_DISABLE_CPU_FEATURES=X86_V4,AVX512_ICL \
    REHEARSAL_CLOSED_FLAG="$CLOSED_FLAG" \
    /bin/bash -c 'exec >> /opt/rehearsal/state/practiced.out 2>&1; while :; do /opt/venv/bin/python3 -B -s /opt/rehearsal/practiced.py; sleep 2; done' \
    < /dev/null > /dev/null 2>&1 &
for _ in $(seq 1 120); do [ -S /run/rehearsal/rehearsal.sock ] && break; sleep 0.5; done
[ -S /run/rehearsal/rehearsal.sock ] || { echo "[setup] the rehearsal facility did not come up" >&2; exit 1; }
echo "[setup] rehearsal facility listening on /run/rehearsal/rehearsal.sock" >&2

# ---- the graded episode's service (dual mode) ------------------------------------------------------------------------
# A clean environment (env -i: none of the stage's variables; the service sets its own public constants from its
# config), stdio closed by the daemon itself; the command returns once the service serves (its worker has built the
# graded site's cell) or failed. The service's own logs (service.log, supervisor.log) sit next to the record.
GRADED_USER=arbgraded
GRADED_DATA=/opt/arb-graded
RECORD_DIR=/var/lib/arb-online
id -u "$GRADED_USER" >/dev/null                      # the account comes with the image
for f in online/graded_service.py online/service.json online/data/episode.json online/data/oracle_token.sha256 \
         online/data/cell/cabincell/server.py online/data/cell/grader_config.json online/data/cell/reference/reference.py; do
  [ -f "$f" ] || { echo "[setup] online/ is incomplete: $f is missing" >&2; exit 1; }
done
if [ -s "$GRADED_DIR/supervisor.pid" ]; then         # a second run of this script: stop the previous service first
  OLD="$(cat "$GRADED_DIR/supervisor.pid")"
  if [ "$(cat "/proc/$OLD/comm" 2>/dev/null)" = "arb-graded-sup" ]; then
    kill -TERM "$OLD" 2>/dev/null || true
    sleep 2
  fi
fi
rm -rf "$GRADED_DATA"
install -d -o "$GRADED_USER" -g "$GRADED_USER" -m 0700 "$GRADED_DATA"
cp -r online/data/. "$GRADED_DATA/"
find "$GRADED_DATA" -name '__pycache__' -type d -prune -exec rm -rf {} +
chown -R "$GRADED_USER:$GRADED_USER" "$GRADED_DATA"
chmod -R u+rwX,go-rwx "$GRADED_DATA"
install -d -o root -g root -m 0755 "$GRADED_DIR"
install -d -o "$GRADED_USER" -g "$GRADED_USER" -m 0700 "$RECORD_DIR"
# the agent account reaches none of it (a proof before the service holds anything)
! runuser -u "$AGENT_USER" -- ls "$GRADED_DATA" >/dev/null 2>&1 \
  && ! runuser -u "$AGENT_USER" -- ls "$RECORD_DIR" >/dev/null 2>&1 \
  && ! runuser -u "$AGENT_USER" -- cat "$GRADED_DATA/episode.json" >/dev/null 2>&1 \
  || { echo "[setup] the agent account can reach the graded episode's data or record" >&2; exit 1; }
if ! env -i PATH=/opt/venv/bin:/usr/local/bin:/usr/bin:/bin LANG=C.UTF-8 HOME=/nonexistent \
    /opt/venv/bin/python3 -I -B "$PWD/online/graded_service.py" --config "$PWD/online/service.json" </dev/null; then
  echo "[setup] the graded episode's service did not come up; its logs:" >&2
  tail -n 40 "$RECORD_DIR/service.log" "$RECORD_DIR/supervisor.log" >&2 || true
  exit 1
fi
[ -S "$GRADED_DIR/graded.sock" ] || { echo "[setup] the graded episode's socket is missing" >&2; exit 1; }
echo "[setup] graded episode's service listening on $GRADED_DIR/graded.sock" >&2

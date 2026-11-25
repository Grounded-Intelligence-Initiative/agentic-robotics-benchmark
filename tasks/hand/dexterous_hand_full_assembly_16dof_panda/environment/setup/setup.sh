#!/usr/bin/env bash
# Setup: trusted root, cwd = this staged folder (/opt/ale/setup, root-only while the agent works), before the agent
# phase. Two jobs:
# (1) The givens: the kit, the manual, the cell documents, the Panda and the cell library into the agent home, owned by
#     the agent. The grader never reads these copies: verification is separate (task.toml), and tests/test.sh builds the
#     grader's own ROOT-OWNED read-only copy of the same givens in the fresh verifier sandbox, from the image's task
#     assets and tests/setup_payload/, a byte copy of payload/ here (change both together, then
#     tests/given_hashes.json).
# (2) The graded episode's cell service (online/service.py; dual mode, experimental): it installs the cell's code
#     (online/cell, a byte copy of the verifier's cell in tests/) and the cell's kit (downloaded here, with the network
#     the engine opens for the setup, as tests/verify.py downloads it at verify start: the tarball pinned in
#     grader_config.json, its sha256 and its file count checked) under /opt/arb-online, root-owned and readable only by
#     the dedicated `arbcell` account created below; then it reads the hidden graded episode (online/episode.json) as
#     root, builds the episode's cell, drops to `arbcell` and serves /run/arb-online/cell.sock. The episode's record goes
#     to /var/lib/arb-online (arbcell's, 0700: the agent can neither read nor write it; a declared artifact, scored by
#     tests/verify.py in the separate verifier). Dual mode: at the earlier of the graded start (before it is charged)
#     and the start window's close, the service's root supervisor freezes the agent's program (/home/user/submission
#     -> /var/lib/arb-online/submission_snapshot), the copy the verifier grades as the offline item. Fixed argv, a clean
#     environment, no hidden value outside this folder. The command returns once the service serves (or fails setup).
# There is no practice cell: the service serves the one graded episode and nothing else.
# Dynamic-only: packages live in environment/Dockerfile (unchanged from main's image).
set -euo pipefail
umask 022
export HOME=/root PYTHONNOUSERSITE=1
unset PYTHONPATH PYTHONSTARTUP
HOME_DIR="${ALE_HOME:-/home/user}"
AGENT_USER="$(basename "$HOME_DIR")"
LAB_USER=arbcell

# (1) the givens
cp -r payload/. "$HOME_DIR/"
# the large givens (the manual, the parts' STL files, the Panda model) and the cameras' scenery (render/) come from the
# image (environment/Dockerfile, task assets)
if [ -d /opt/task-assets/payload ]; then
  cp -r /opt/task-assets/payload/. "$HOME_DIR/"
fi
for entry in payload/* payload/.[!.]* /opt/task-assets/payload/*; do
  [ -e "$entry" ] || continue
  chown -R "$AGENT_USER:$AGENT_USER" "$HOME_DIR/$(basename "$entry")"
done
mkdir -p "$HOME_DIR/submission" && chown "$AGENT_USER:$AGENT_USER" "$HOME_DIR/submission"
echo "[setup] givens staged in $HOME_DIR (owner $AGENT_USER)" >&2

# (2) the graded episode's cell service
# the dedicated account (not root, not the agent; no home, no shell), created here so that the task image stays main's
id -u "$LAB_USER" >/dev/null 2>&1 || \
  useradd --system --user-group --no-create-home --home-dir /nonexistent --shell /usr/sbin/nologin "$LAB_USER"
chmod 0700 online
chmod 0600 online/episode.json
umask 077
# Returns once the service serves (the daemon is detached, its stdio on /dev/null) or fails setup. The numerics pin is
# tests/test.sh's (the cell's numbers the same on every x86-64 CPU), set before the interpreter starts.
env -i PATH=/opt/venv/bin:/usr/local/bin:/usr/bin:/bin LANG=C.UTF-8 HOME=/nonexistent \
  OPENBLAS_CORETYPE=Haswell OPENBLAS_L2_SIZE=256 NPY_DISABLE_CPU_FEATURES="X86_V4 AVX512_ICL AVX512_SPR" \
  /opt/venv/bin/python3 -I -B "$PWD/online/service.py" --config "$PWD/online/online.json" </dev/null
echo "[setup] the graded episode's cell service is up (/run/arb-online/cell.sock)" >&2

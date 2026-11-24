#!/usr/bin/env bash
# Setup: trusted root, cwd = this staged folder (/opt/ale/setup, root-only while the agent works), before the agent
# phase. Two jobs:
# (1) The givens (the packing list, the units' STL files, the drawings, the trim sheet, the water density chart and the
#     propeller data sheet, the picture manual, the station scope and the work-order example, the cell documents, the
#     robot models and the cell library) and the cameras' scenery into the agent home, owned by the agent. The scoring
#     stage never reads these copies: it runs in a separate sandbox (task.toml) and builds its own root-owned read-only
#     copy of the same givens from the image's task assets and a byte copy of payload/ here (tests/setup_payload/;
#     change both together, then tests/given_hashes.json).
# (2) The graded episode's cell service (online/service.py; dual mode, experimental): the dedicated account `arbcell`
#     (created here so that the image stays as main builds it); the cell's copy online/cell (a byte copy of the cell's
#     part of tests/) installed at /opt/arb-online/cell and the cell's hidden kit fetched into /opt/arb-online/gt
#     (online/fetch_kit.py: the tarball the offline verifier fetches, pinned by sha256; setup has network, the agent
#     phase has none), both root:arbcell 0750 / 0640, which the agent's account cannot read; then the service, which
#     reads the hidden graded episode (online/episode.json, root-only here) as root, builds the episode's cell, drops to
#     `arbcell` and serves /run/arb-online/cell.sock. The episode's record goes to /var/lib/arb-online (arbcell's, 0700:
#     the agent can neither read nor write it; a declared artifact, scored by tests/verify.py in the separate
#     verifier). Dual mode: at the earlier of the graded start (before it is charged) and the start window's close the
#     service's root supervisor freezes the agent's program (/home/user/submission ->
#     /var/lib/arb-online/submission_snapshot), the copy the verifier grades as the program item. Fixed argv, a clean
#     environment, no hidden value outside this folder and the two root:arbcell folders. The command returns once the
#     service serves (or fails setup).
# There is no practice cell: the service serves the one graded episode and nothing else. Dynamic-only: packages live in
# environment/Dockerfile.
set -euo pipefail
umask 022
HOME_DIR="${ALE_HOME:-/home/user}"
AGENT_USER="$(basename "$HOME_DIR")"
ASSETS=/opt/task-assets/payload
LAB_USER=arbcell
SERVICE_DIR=/opt/arb-online
PY=/opt/venv/bin/python3
# the large givens and the cameras' scenery (render/) come from the image (environment/Dockerfile, task assets)
[ -d "$ASSETS" ] || { echo "[setup] $ASSETS missing: the image was built without its task assets" >&2; exit 1; }
[ -d payload ] || { echo "[setup] payload/ missing next to setup.sh" >&2; exit 1; }
cp -r payload/. "$HOME_DIR/"
cp -r "$ASSETS/." "$HOME_DIR/"
for entry in payload/* payload/.[!.]* "$ASSETS"/*; do
  [ -e "$entry" ] || continue
  chown -R "$AGENT_USER:$AGENT_USER" "$HOME_DIR/$(basename "$entry")"
done
mkdir -p "$HOME_DIR/submission" && chown "$AGENT_USER:$AGENT_USER" "$HOME_DIR/submission"
echo "[setup] givens staged in $HOME_DIR (owner $AGENT_USER)" >&2

# (2) the graded episode's cell service
for f in online/service.py online/online.json online/episode.json online/fetch_kit.py online/cell/grader_config.json \
         online/cell/arm/assembly_rov/server.py; do
  [ -f "$f" ] || { echo "[setup] online/ is incomplete: $f is missing" >&2; exit 1; }
done
id -u "$LAB_USER" >/dev/null 2>&1 || \
  useradd --system --user-group --no-create-home --home-dir /nonexistent --shell /usr/sbin/nologin "$LAB_USER"
id -nG "$AGENT_USER" | tr ' ' '\n' | grep -qx "$LAB_USER" && { echo "[setup] $AGENT_USER is in group $LAB_USER" >&2; exit 1; }
chmod 0700 online
chmod 0600 online/episode.json
rm -rf "$SERVICE_DIR"
install -d -o root -g "$LAB_USER" -m 0750 "$SERVICE_DIR"
cp -r online/cell "$SERVICE_DIR/cell"
find "$SERVICE_DIR/cell" -name '__pycache__' -type d -prune -exec rm -rf {} +
chown -R "root:$LAB_USER" "$SERVICE_DIR/cell"
find "$SERVICE_DIR/cell" -type d -exec chmod 0750 {} +
find "$SERVICE_DIR/cell" -type f -exec chmod 0640 {} +
env -i PATH=/opt/venv/bin:/usr/local/bin:/usr/bin:/bin LANG=C.UTF-8 HOME=/root \
  ${ARB_HIDDEN_GT_TARBALL:+ARB_HIDDEN_GT_TARBALL="$ARB_HIDDEN_GT_TARBALL"} \
  "$PY" -I -B online/fetch_kit.py "$PWD/online/cell/grader_config.json" "$SERVICE_DIR/gt" "$LAB_USER" </dev/null
install -d -o root -g root -m 0755 /run/arb-online
install -d -o "$LAB_USER" -g "$LAB_USER" -m 0700 /var/lib/arb-online
install -d -o "$LAB_USER" -g "$LAB_USER" -m 0700 /var/lib/arb-online-work
umask 077
# Returns once the service serves (the daemon is detached, its stdio on /dev/null) or fails setup. The numerics pins
# are the image's (and tests/test.sh's); the service sets them again for its worker from online.json.
if ! env -i PATH=/opt/venv/bin:/usr/local/bin:/usr/bin:/bin LANG=C.UTF-8 HOME=/nonexistent \
    OPENBLAS_CORETYPE=Haswell OPENBLAS_L2_SIZE=256 NPY_DISABLE_CPU_FEATURES="X86_V4 AVX512_ICL AVX512_SPR" \
    "$PY" -I -B "$PWD/online/service.py" --config "$PWD/online/online.json" </dev/null; then
  echo "[setup] the graded episode's cell service did not come up; its logs:" >&2
  tail -n 40 /var/lib/arb-online/service.log /var/lib/arb-online/supervisor.log >&2 || true
  exit 1
fi
echo "[setup] the graded episode's cell service is up (/run/arb-online/cell.sock)" >&2

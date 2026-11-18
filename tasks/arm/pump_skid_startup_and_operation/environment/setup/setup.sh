#!/usr/bin/env bash
# Setup: trusted root, cwd = this staged folder, before the agent phase. Stages the givens (the cell documents, the
# client library, the instrument catalog, the commissioning launcher and the example program) into the agent home, owned
# by the agent, and starts the graded episode's service and the commissioning service (below). The grader
# never reads these copies: verification is separate (task.toml), and tests/test.sh builds the grader's own ROOT-OWNED read-only
# copy of the same givens in the fresh verifier sandbox, from tests/setup_payload/ (a byte copy of payload/ here: change
# both together, then tests/given_hashes.json) and the image's robot model (/opt/task-assets/panda).
# Dynamic-only: packages, the robot model and the commissioning service's program live in environment/Dockerfile.
set -euo pipefail
HOME_DIR="${ALE_HOME:-/home/user}"
AGENT_USER="$(basename "$HOME_DIR")"
cp -r payload/. "$HOME_DIR/"
if [ -d /opt/task-assets/panda ]; then
  mkdir -p "$HOME_DIR/given"
  cp -r /opt/task-assets/panda "$HOME_DIR/given/panda"
fi
for entry in payload/* payload/.[!.]*; do
  [ -e "$entry" ] || continue
  chown -R "$AGENT_USER:$AGENT_USER" "$HOME_DIR/$(basename "$entry")"
done
mkdir -p "$HOME_DIR/submission" && chown "$AGENT_USER:$AGENT_USER" "$HOME_DIR/submission"
echo "[setup] givens staged in $HOME_DIR (owner $AGENT_USER)" >&2

# The online variant: the graded episode's service (online/graded_service.py), ONE episode of the grader's hidden set
# (online/episode.json) that the agent plays itself through skidcell.client.connect_graded(). It runs the grader's own
# cell: the image's byte copy of tests/skidcell/ (/opt/skidcell-commission/skidcell/), copied here, root-only, before
# any other account runs (the worker checks it against online/service.json's digests and imports it as root). Started
# as root, it reads the episode, then serves as its own account `skidgrade` (created here: a system account without a
# home or a login shell, not the commissioning service's `skidcell`, which serves the agent's commissioning shifts)
# through /run/skidcell-graded/graded.sock and writes the episode's record to /var/lib/arb-online (owner skidgrade,
# 0700), which task.toml declares an artifact and tests/verify.py scores. The command returns once the service serves.
GRADED="$PWD/online"
id -u skidgrade > /dev/null 2>&1 || useradd --system --user-group --no-create-home --home-dir /nonexistent \
    --shell /usr/sbin/nologin skidgrade
rm -rf "$GRADED/skidcell"
install -d -o root -g root -m 700 "$GRADED/skidcell"
cp /opt/skidcell-commission/skidcell/*.py "$GRADED/skidcell/"
chown -R root:root "$GRADED"
chmod -R go-rwx "$GRADED"
env -i PATH=/opt/venv/bin:/usr/local/bin:/usr/bin:/bin LANG=C.UTF-8 HOME=/nonexistent \
    /opt/venv/bin/python3 -I -B "$GRADED/graded_service.py" --config "$GRADED/service.json" < /dev/null > /dev/null 2>&1 \
  || { echo "[setup] the graded episode's service did not come up (/var/lib/arb-online/service.log)" >&2; exit 1; }
[ -S /run/skidcell-graded/graded.sock ] || { echo "[setup] the graded episode's service has no socket" >&2; exit 1; }
echo "[setup] graded episode's service listening on /run/skidcell-graded/graded.sock" >&2

# The commissioning service (v0.6), the series' cell on its development sites (sister sites, never a graded one): the
# image installs it under /opt/skidcell-commission for the account `skidcell` only (the agent account can neither read
# nor list it); setup starts it now, to run through the agent's session, under a loop that restarts it should it ever
# exit. The agent reaches it through the socket /run/skidcell/commission.sock (skidcell.client.connect_commissioning,
# ~/commission/run_shift.py): the cell's public calls and reset(site), with the graded episode's budgets and a fixed
# number of shifts; no ground truth, no score, no debrief. Its numerics are pinned as the grader pins them.
SERVICE=/opt/skidcell-commission
[ -f "$SERVICE/commissiond.py" ] || { echo "[setup] the image lacks the commissioning service ($SERVICE)" >&2; exit 1; }
install -d -o skidcell -g skidcell -m 755 /run/skidcell
rm -f /run/skidcell/commission.sock
setsid runuser -u skidcell -- env HOME="$SERVICE" MUJOCO_GL=osmesa PYOPENGL_PLATFORM=osmesa OMP_NUM_THREADS=2 \
    OPENBLAS_NUM_THREADS=2 OPENBLAS_CORETYPE=Haswell NPY_DISABLE_CPU_FEATURES="X86_V4 AVX512_ICL AVX512_SPR" \
    /bin/bash -c 'while :; do /opt/venv/bin/python3 -s /opt/skidcell-commission/commissiond.py; sleep 2; done' \
    < /dev/null > /dev/null 2>&1 &
for _ in $(seq 1 60); do [ -S /run/skidcell/commission.sock ] && break; sleep 0.5; done
[ -S /run/skidcell/commission.sock ] || { echo "[setup] the commissioning service did not come up" >&2; exit 1; }
echo "[setup] commissioning service listening on /run/skidcell/commission.sock" >&2

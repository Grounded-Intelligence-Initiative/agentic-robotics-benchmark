#!/usr/bin/env bash
# Setup: trusted root, cwd = this staged folder, before the agent phase (the engine bounds it at 1800 s).
#
# 1. Stages the agent-visible material (payload/: the holder manual and model, the commissioning client and launcher,
#    the example controller) into the agent home, owned by the agent. Everything under payload/ is staged, whatever it
#    is called.
# 2. Starts the commissioning service (the image installs it under /opt/endosim-commission for the account `endosim`
#    only) and waits until its renderer, an Isaac Sim process on the sandbox's GPU, is up and has passed its canary.
#    On a cold cache that start compiles the RTX pipelines and takes minutes. The service then records the
#    development set into /var/lib/endocam/dev_procedures, which the agent reads as ~/given/dev_procedures, during the
#    first minutes of the agent's session, and serves commissioning sessions on /run/endocam/commission.sock.
#
# A sandbox whose GPU cannot render (no RTX GPU, or a host driver without its graphics libraries) fails here, as an
# environment error, before any agent time is spent. Dynamic-only: packages and the service's program live in the
# image. The verifier sandbox never runs this script: the grader starts a renderer of its own.
set -euo pipefail

HOME_DIR="${ALE_HOME:-/home/user}"
AGENT_USER="$(basename "$HOME_DIR")"

# cp -r, NOT -a: with -a the `payload/.` form also copies the payload directory's own root ownership onto the agent
# home, locking the agent out of its own workspace.
cp -r payload/. "$HOME_DIR/"
for entry in payload/* payload/.[!.]*; do
  [ -e "$entry" ] || continue
  chown -R "$AGENT_USER:$AGENT_USER" "$HOME_DIR/$(basename "$entry")"
done
mkdir -p "$HOME_DIR/submission" && chown "$AGENT_USER:$AGENT_USER" "$HOME_DIR/submission"
echo "[setup] givens staged in $HOME_DIR (owner $AGENT_USER)" >&2

SERVICE=/opt/endosim-commission
STATE=/var/lib/endocam
[ -f "$SERVICE/commissiond.py" ] || { echo "[setup] the image lacks the commissioning service ($SERVICE)" >&2; exit 1; }
# The service's state: traversable but not listable by others; the development set inside it is world-readable.
install -d -o endosim -g endosim -m 711 "$STATE"
install -d -o endosim -g endosim -m 700 "$STATE/home" "$STATE/tmp" "$STATE/optix"
install -d -o endosim -g endosim -m 755 "$STATE/dev_procedures" /run/endocam
rm -f /run/endocam/commission.sock "$STATE/ready" "$STATE/failed"
ln -sfn "$STATE/dev_procedures" "$HOME_DIR/given/dev_procedures"
chown -h "$AGENT_USER:$AGENT_USER" "$HOME_DIR/given/dev_procedures"

setsid runuser -u endosim -- env HOME="$STATE/home" TMPDIR="$STATE/tmp" OPTIX_CACHE_PATH="$STATE/optix" \
    OMNI_KIT_ACCEPT_EULA=YES ACCEPT_EULA=Y PRIVACY_CONSENT=Y PYTHONDONTWRITEBYTECODE=1 \
    /bin/bash -c 'umask 022; cd /var/lib/endocam; while :; do /opt/venv/bin/python3 -B -s /opt/endosim-commission/commissiond.py; sleep 5; done' \
    < /dev/null > /dev/null 2>&1 &

for _ in $(seq 1 750); do
  [ -f "$STATE/ready" ] && break
  [ -f "$STATE/failed" ] && { echo "[setup] the commissioning service could not start its renderer:" >&2; cat "$STATE/failed" >&2; exit 1; }
  sleep 2
done
[ -f "$STATE/ready" ] || { echo "[setup] the commissioning service's renderer did not come up in 1500 s" >&2; exit 1; }
[ -S /run/endocam/commission.sock ] || { echo "[setup] the commissioning socket is missing" >&2; exit 1; }
echo "[setup] commissioning service ready on /run/endocam/commission.sock; the development set is being recorded" >&2

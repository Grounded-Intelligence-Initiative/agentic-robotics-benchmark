#!/usr/bin/env bash
# Setup: trusted root, cwd = this staged folder, before the agent phase. Stages the givens into the agent home as
# /home/user/given, READ-ONLY to the agent (owned by root, no write bit for anyone else): the small givens from
# payload/given/ here (the contract and its checker, the documents, the procedure, the lab's notes, the unit's dossier)
# and the large ones from the image's task assets (/opt/task-assets/payload/given/: the maker's design files, the CAD
# export, the procedure's figures). The agent writes only /home/user/submission (and its own working files anywhere
# in its home). Neither this folder nor the image holds any of the unit's ground truth, which lives in tests/ (absent
# during the agent phase). The scoring stage never reads these copies (separate verification, task.toml).
# Dynamic-only: packages live in environment/Dockerfile.
set -euo pipefail
HOME_DIR="${ALE_HOME:-/home/user}"
AGENT_USER="$(basename "$HOME_DIR")"
ASSETS=/opt/task-assets/payload
[ -d "$ASSETS/given" ] || { echo "[setup] $ASSETS/given missing: the image was built without its task assets" >&2; exit 1; }
[ -d payload/given ] || { echo "[setup] payload/given missing next to setup.sh" >&2; exit 1; }
GIVEN="$HOME_DIR/given"
rm -rf "$GIVEN"
mkdir -p "$GIVEN"
cp -r "$ASSETS/given/." "$GIVEN/"
cp -r payload/given/. "$GIVEN/"
chown -R root:root "$GIVEN"
chmod -R u+rwX,go+rX,go-w "$GIVEN"
mkdir -p "$HOME_DIR/submission" && chown "$AGENT_USER:$AGENT_USER" "$HOME_DIR/submission"
echo "[setup] givens staged read-only in $GIVEN; the submission folder is $HOME_DIR/submission (owner $AGENT_USER)" >&2

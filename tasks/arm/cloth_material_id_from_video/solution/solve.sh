#!/usr/bin/env bash
# Privileged reference: writes the true materials of the fixed hidden fabrics (solution/ref/params.json, the task's
# defined answer) as the submission.
set -euo pipefail
cd "$(dirname "$0")"
mkdir -p /home/user/submission
cp ref/params.json /home/user/submission/params.json
chmod 0644 /home/user/submission/params.json

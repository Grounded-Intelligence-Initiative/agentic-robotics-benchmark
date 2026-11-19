#!/usr/bin/env bash
# ALE's author-only oracle stage submits the public-observation demonstration.
set -euo pipefail
cd "$(dirname "$0")"
mkdir -p /home/user/submission
cp oracle/{run.sh,oracle.py,survey.py,registration.py,walk.py} /home/user/submission/
chmod 755 /home/user/submission/run.sh

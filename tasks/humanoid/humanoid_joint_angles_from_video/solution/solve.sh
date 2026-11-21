#!/usr/bin/env bash
# Privileged reference: writes the true joint angles of every test recording (solution/ref, the task's defined
# answer) as the submission.
set -euo pipefail
cd "$(dirname "$0")"
mkdir -p /home/user/submission
cp ref/T*.csv /home/user/submission/
chmod 0644 /home/user/submission/T*.csv

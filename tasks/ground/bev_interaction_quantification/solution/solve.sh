#!/usr/bin/env bash
# Privileged reference: writes the hidden answers themselves (solution/ref, the task's defined answer) as the
# submission.
set -euo pipefail
cd "$(dirname "$0")"
mkdir -p /home/user/submission
cp ref/answers.csv /home/user/submission/answers.csv
chmod 0644 /home/user/submission/answers.csv

#!/usr/bin/env bash
# Privileged reference: writes the lost logs themselves, resampled at the frames of each test video (solution/ref,
# the task's defined answer), as the submission.
set -euo pipefail
cd "$(dirname "$0")"
mkdir -p /home/user/submission
cp ref/T*.csv /home/user/submission/
chmod 0644 /home/user/submission/T*.csv

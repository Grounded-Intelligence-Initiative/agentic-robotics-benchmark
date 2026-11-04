#!/usr/bin/env bash
set -euo pipefail
task_home="${ALE_HOME:-/home/user}"
cp -r payload/. "$task_home/"
chown -R "$(basename "$task_home"):$(basename "$task_home")" "$task_home/practice_grader" "$task_home/square_nut_env.py"

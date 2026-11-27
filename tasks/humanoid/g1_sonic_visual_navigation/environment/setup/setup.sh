#!/usr/bin/env bash
set -euo pipefail
home_dir="${ALE_HOME:-/home/user}"
account="$(basename "$home_dir")"
install -o "$account" -g "$account" -m 0644 payload/practice.py payload/practice_wire.py "$home_dir/"
install -o "$account" -g "$account" -m 0644 payload/starter.py "$home_dir/starter.py"

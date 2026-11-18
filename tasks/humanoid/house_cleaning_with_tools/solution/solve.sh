#!/usr/bin/env bash
# Oracle (online graded episode): the task's privileged reference cleaner, run in place of the agent by `ale validate`
# as the same unprivileged account, through the agent-side client. reference/online_oracle.py connects to the graded
# episode's cell service (house_client.connect_graded), starts THE graded episode with the 256-bit oracle token (the
# service compares its sha256 with the digest it holds and, on a match, gives this episode the privileged calls
# `reference_clean` and `truth`, as the offline grader gives them to a submission carrying the token), runs
# reference/clean.py's solve(cell) on it (one frame of each public camera, so every validation exercises the agents'
# camera path, then the reference cleaner inside one call: true object poses and process states, the G1 driven through
# the cell's own controller, grip rule, safety watch and cleaning processes) and finishes the episode. Nothing is left
# in the submission folder: the verifier scores the service's record only.
set -euo pipefail
HOME_DIR="${ALE_HOME:-/home/user}"
A="$(cd "$(dirname "$0")" && pwd)/reference"
exec /opt/venv/bin/python3 -I -B "$A/online_oracle.py" "$HOME_DIR/given/lib" "$A/oracle_token"

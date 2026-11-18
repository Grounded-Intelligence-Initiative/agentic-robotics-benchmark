#!/usr/bin/env bash
# Oracle (online variant): the privileged reference assembler plays the graded episode, run in place of the agent by
# `ale validate` as the same unprivileged account. reference/graded_oracle.py connects to the graded episode's service
# through the agent-side client (~/given/lib/furniture_client: connect_graded), starts the episode with the 256-bit
# oracle token (the service compares its sha256 with environment/setup/online/oracle_token.sha256 and, on a match,
# allows the cell's privileged calls on the graded layout), runs reference/assemble.py's solve(cell) on it (one frame of
# each public camera, then `reference_assemble`: the reference assembler inside the cell, true part poses, the two
# robots driven through the cell's own controllers, safety watch and joints) and finishes the episode. Nothing else is
# graded: the submission folder is not read.
set -euo pipefail
A="$(cd "$(dirname "$0")" && pwd -P)/reference"
exec /opt/venv/bin/python3 -B "$A/graded_oracle.py" --token "$A/oracle_token" --program "$A/assemble.py"

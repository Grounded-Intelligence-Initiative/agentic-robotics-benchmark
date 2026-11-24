#!/usr/bin/env bash
# The reference solution. `ale validate` uploads solution/ to ~/.ale-solution and runs `bash solve.sh` there in
# place of the agent, as the same unprivileged account, under the same timeout and network.
#
# The reference is not privileged. It measures each graded unit in the one lab session the unit has, with the
# documents the agent has (~/arm_model.py, ~/cell_data.py, ~/labclient.py), and writes one submission per unit.
set -euo pipefail
cd "$(dirname "$0")"
export PYTHONDONTWRITEBYTECODE=1
exec python3 calibrate/run.py

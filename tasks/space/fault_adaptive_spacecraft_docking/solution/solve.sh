#!/usr/bin/env bash
# The task's reference solution, run in place of the agent by `ale validate`, as the same unprivileged account with
# the same timeout. It installs the PRIVILEGED reference controller (solution/privileged/oracle.py, with the copy of
# the simulator beside it) and the reference token: the grader hands a submission whose .oracle_token matches the
# digest in tests/hidden/ the simulator's current truth in every observation, and this controller plans from it alone
# (an exact replica of the plant, simulate-and-select over a fixed policy family). It shows that the hidden episodes
# can be done; the anchor is its own measured score on them. An agent gets no such block and works from the
# 16-number observation.
#
# The non-privileged reference that works from the observation alone (the task author's identifying controller) is
# solution/reference/; `bash solution/reference/install.sh [submission dir]` puts it in a submission folder for a
# re-measurement. This script does not use it.
set -euo pipefail

SUB="${ALE_HOME:-/home/user}/submission"
HERE="$(cd "$(dirname "$0")" && pwd)"
mkdir -p "$SUB"

cp "$HERE/privileged/oracle.py" "$HERE/privileged/spacecraft_docking_env.py" "$SUB/"
cp "$HERE/privileged/oracle_token" "$SUB/.oracle_token"

cat > "$SUB/run.sh" <<'SH'
#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
exec /opt/venv/bin/python3 -B oracle.py
SH
chmod +x "$SUB/run.sh"

echo "[oracle] privileged submission ready at $SUB" >&2

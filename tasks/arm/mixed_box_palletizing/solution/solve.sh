#!/usr/bin/env bash
# The reference solution. `ale validate` uploads solution/ to ~/.ale-solution and runs `bash solve.sh` there in
# place of the agent, as the same unprivileged account, under the same timeout and network.
#
# The reference is privileged: tests/verify.py adds `truth` (the hidden calibration of the order and the true
# state of every carton that has arrived) to the observations of a submission whose .oracle_token hashes to
# tests/hidden/oracle_token.sha256. It needs no lab trial and reads nothing outside solution/.
set -euo pipefail

HOME_DIR="${ALE_HOME:-/home/user}"
SUB="$HOME_DIR/submission"
mkdir -p "$SUB"

cp "$(dirname "$0")/controller/controller.py" "$SUB/controller.py"
cp "$(dirname "$0")/oracle_token" "$SUB/.oracle_token"

cat > "$SUB/run.sh" <<'SH'
#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
exec python3 controller.py
SH
chmod +x "$SUB/run.sh"

echo "[oracle] submission ready at $SUB" >&2

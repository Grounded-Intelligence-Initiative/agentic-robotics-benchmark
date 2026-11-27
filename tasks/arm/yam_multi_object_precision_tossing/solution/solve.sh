#!/usr/bin/env bash
set -euo pipefail
sub="${ALE_HOME:-/home/user}/submission"
mkdir -p "$sub"
src="$(dirname "$0")"
cp "$src"/{agent,manipulation,wood_reference,arcade_moves,grips}.py "$sub/"
cp "$src"/{fixed_motions,learned-grips,learned-games}.json "$sub/"
cat > "$sub/run.sh" <<'RUN'
#!/usr/bin/env bash
set -euo pipefail
exec python3 "$(dirname "$0")/agent.py"
RUN
chmod +x "$sub/run.sh"

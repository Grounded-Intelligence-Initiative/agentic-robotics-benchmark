#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
episode_home=${ALE_HOME:-/home/user}
test "${CLAUDE_CODE_DISABLE_AUTO_MEMORY:-}" = 1
test "${CLAUDE_CODE_DISABLE_CLAUDE_MDS:-}" = 1
test ! -e "$episode_home/survey"
test -z "$(find "$episode_home/submission" -mindepth 1 -print -quit)"
for root in "$episode_home/.claude" "$episode_home/.claude-config"; do
    for entry in projects history.jsonl agent-memory; do
        if [ -e "$root/$entry" ]; then
            test -d "$root/$entry" && test -z "$(find "$root/$entry" -mindepth 1 -print -quit)" || exit 1
        fi
    done
done
test -z "$(find "$episode_home" \( -name CLAUDE.md -o -name CLAUDE.local.md -o -name MEMORY.md -o -name AGENTS.md \) -print -quit)"
echo "Cold start checked: empty prior survey, submission and session history; memory loading disabled."
exec python3 independent.py --packet "$episode_home/survey"

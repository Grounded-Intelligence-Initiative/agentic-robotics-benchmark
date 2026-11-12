#!/usr/bin/env bash
# The oracle episode: leave the privileged oracle's mission program (with its token) as the submission. The
# separate verifier runs it on the hidden sites; the graded robot service answers its `truth` call because the
# token's sha256 is the one in tests/hidden/oracle_token.sha256.
set -euo pipefail
cd "$(dirname "$0")"
mkdir -p /home/user/submission
cp oracle/run.sh oracle/oracle.py oracle/client.py oracle/token.txt /home/user/submission/
chmod 755 /home/user/submission/run.sh

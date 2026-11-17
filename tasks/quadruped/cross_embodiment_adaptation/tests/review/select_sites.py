"""Author-only fixed-suite run in the separate-verifier layout; never selects seeds.

python3 select_sites.py --program /path/to/oracle --out /evidence/fixed-suite.json
Requires root, staged tests and the authorized grader API. Parent owns official ALE validation.
"""
import argparse
import json
import os
from pathlib import Path
import pwd
import sys

STAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(STAGE))


def main():
    import grader as G
    parser = argparse.ArgumentParser()
    parser.add_argument('--program', required=True)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    seeds = json.loads((STAGE/'hidden/sites.json').read_text())['seeds']
    account = pwd.getpwnam(G.agent_user())
    G.secure_exec_path()
    for path, mode in ((G.WORK, 0o700), (G.RUN, 0o755)):
        path.mkdir(parents=True, exist_ok=True)
        os.chmod(path, mode)
    problem = G.copy_submission(args.program, G.RUN/'program')
    if problem:
        raise RuntimeError(problem)
    G.stage_public_client()
    token = (G.HIDDEN/'oracle_token.sha256').read_text().split()[0]
    config = json.loads((STAGE/'grader_config.json').read_text())
    rows = []
    for order, seed in enumerate(seeds):
        result = G.run_mission(order, seed, account.pw_uid, account.pw_gid, config, token,
                               attempt=0, oracle_authorized=True)
        rows.append({**result, 'seed': seed})
        print(json.dumps(rows[-1]), flush=True)
        args.out.write_text(json.dumps(dict(version=3, fixed_seeds=seeds, rows=rows), indent=2)+'\n')


if __name__ == '__main__':
    main()

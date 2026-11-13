"""The hidden set's pre-registered selection rule (tests/hidden/selection.json), re-runnable by maintainers.

Candidates: k = 1, 2, ..., 16, seed_k = int.from_bytes(sha256(b"earthquake_search_and_rescue/hidden/v1/<k>")[:8]) >> 1.
The hidden set is the first EPISODES candidates, in k order, on which the frozen reference program completes the
mission (score 1.0: located, kit and returned) in a privileged cell. Nothing runs this script; it documents the rule.

usage: python3 tests/review/select_set.py [--episodes 4] [--candidates 16]    (from the task folder, MUJOCO_GL set)
"""
import argparse
import hashlib
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

from rubblecell.server import RubbleCellServer  # noqa: E402


def candidates(n):
    return [int.from_bytes(hashlib.sha256(b"earthquake_search_and_rescue/hidden/v1/%d" % k).digest()[:8], "big") >> 1
            for k in range(1, n + 1)]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--episodes", type=int, default=4)
    ap.add_argument("--candidates", type=int, default=16)
    a = ap.parse_args()
    cfg = json.load(open(os.path.join(os.path.dirname(HERE), "grader_config.json")))
    chosen, runs = [], []
    for k, seed in enumerate(candidates(a.candidates), 1):
        srv = RubbleCellServer(config=cfg, mode="hidden", privileged=True, reference_dir=os.path.join(os.path.dirname(HERE), "reference"))
        srv.reset(seed, "hidden")
        srv.handle("reference_run", {})
        s = srv.finalize()
        runs.append({"k": k, "seed": seed, "score": s["episode_score"], "parts": s["parts"], "lost": s["lost"]})
        print(json.dumps(runs[-1]), flush=True)
        if s["episode_score"] >= 1.0 - 1e-9:
            chosen.append(seed)
        if len(chosen) >= a.episodes:
            break
    print(json.dumps({"seeds": chosen, "runs": runs}, indent=1))


if __name__ == "__main__":
    main()

"""Maintainers' local run of the grader's episode loop (srv_grade.run_episodes) over the real wire, as the current
user, without the verify stage's privilege drop and seccomp launcher (which need root). For selection and checks:

    SRV_PANDA_DIR=... python3 review/run_local.py SUBMISSION_DIR UNITS_JSON EPISODES_JSON [--truth] [--only i,j] [--out F]

Prints one JSON record per episode and a summary (mean credit, probe pairs)."""

import argparse
import json
import os
import subprocess
import sys

TESTS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, TESTS)

import srv_grade as G  # noqa: E402
from robotics_grader import Solver, SolverAborted  # noqa: E402
import nonet_exec  # noqa: E402


class LocalSolver(Solver):
    lowest_priority = False

    def _spawn(self):
        env = dict(nonet_exec.controller_env())
        env["HOME"] = os.path.expanduser("~")
        return subprocess.Popen(["bash", self.entry], cwd=self.submission, stdin=subprocess.PIPE,
                                stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, universal_newlines=True,
                                bufsize=1, start_new_session=True, env=env)

    def _check_processes(self):
        return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("submission")
    ap.add_argument("units")
    ap.add_argument("episodes")
    ap.add_argument("--truth", action="store_true")
    ap.add_argument("--only")
    ap.add_argument("--out")
    a = ap.parse_args()
    units = json.load(open(a.units))["units"]
    eps = [(n, str(e["unit"]), int(e["seed"])) for n, e in enumerate(json.load(open(a.episodes))["episodes"])]
    if a.only:
        keep = set(int(x) for x in a.only.split(","))
        eps = [e for e in eps if e[0] in keep]
    clock = G.Clock()
    solver = clock.timed(LocalSolver, step_timeout=30.0, submission=os.path.abspath(a.submission),
                         clear_account=False)
    cfg = dict(G.CONFIG, grader_budget_s=9000)
    try:
        records, lock = G.run_episodes(solver, eps, units, a.truth, log=lambda m: print(m, file=sys.stderr, flush=True),
                                       config=cfg, clock=clock)
    finally:
        try:
            solver.close()
        except Exception:  # noqa: BLE001
            pass
    out = []
    for r in records:
        r = {k: v for k, v in r.items() if k != "seed"}
        out.append(r)
        print(json.dumps(r, default=str))
    mean = sum(r["credit"] for r in records) / max(1, len(records))
    summ = {"mean_credit": mean, "lock": lock.rewards(), "controller_s": round(clock.s, 1)}
    print(json.dumps(summ, default=str))
    if a.out:
        json.dump({"records": out, "summary": summ}, open(a.out, "w"), indent=1, default=str)


if __name__ == "__main__":
    main()

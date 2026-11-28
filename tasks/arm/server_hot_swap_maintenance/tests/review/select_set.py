"""Maintainers' tool: choose the fixed hidden set from the candidate runs (review/make_sets.py cand, then
review/run_local.py with the reference over the real wire, one record per candidate) and write

    tests/units.json                     the chosen units' values (u0 .. u9), in full
    tests/hidden/episodes.json           the graded episodes, in a salted shuffled order
    tests/hidden/episode_selection.json  the record: the rule, every candidate's result, the frozen files' sha256

Rule (fixed before the runs): candidates are taken in index order; a candidate is kept when the reference completed
it (credit 1) in its run over the wire; the first N kept form the set. The set therefore holds only work orders the
reference completes; it says nothing of the others' difficulty.

    python3 review/select_set.py CAND_UNITS CAND_EPISODES RESULTS_DIR TESTS_DIR [N]
"""

import hashlib
import json
import os
import random
import sys

ORDER_SALT = "srv-order-51b7"
FROZEN = ("srv_sim.py", "srv_env.py", "episode.py", "srv_grade.py")


def sha(path):
    with open(path, "rb") as f:
        return hashlib.sha256(f.read()).hexdigest()


def main():
    cu, ce, rd, tests = sys.argv[1:5]
    n = int(sys.argv[5]) if len(sys.argv) > 5 else 10
    units = json.load(open(cu))["units"]
    cands = json.load(open(ce))["episodes"]
    record, kept = [], []
    for c in cands:
        path = os.path.join(rd, "c%d.json" % c["index"])
        try:
            res = json.load(open(path))["records"][0]
        except (OSError, ValueError, KeyError, IndexError):
            record.append({"index": c["index"], "result": "not run"})
            continue
        sc = res.get("score") or {}
        ok = float(res.get("credit", 0.0)) == 1.0
        record.append({"index": c["index"], "credit": res.get("credit"), "steps": res.get("steps"),
                       "end": sc.get("end"), "incident": sc.get("incident"), "failed_slots": sc.get("failed_slots"),
                       "failed_fan": sc.get("failed_fan"), "kept": bool(ok and len(kept) < n)})
        if ok and len(kept) < n:
            kept.append(c)
    if len(kept) < n:
        sys.exit("only %d candidates kept" % len(kept))
    out_units, eps = {}, []
    for i, c in enumerate(kept):
        uid = "u%d" % i
        out_units[uid] = units[c["unit"]]
        eps.append({"unit": uid, "seed": int(c["seed"]), "candidate": int(c["index"])})
    random.Random(ORDER_SALT).shuffle(eps)
    json.dump({"units": out_units}, open(os.path.join(tests, "units.json"), "w"), indent=1, sort_keys=True)
    json.dump({"episodes": [{"unit": e["unit"], "seed": e["seed"]} for e in eps]},
              open(os.path.join(tests, "hidden", "episodes.json"), "w"), indent=1)
    json.dump({"rule": __doc__.split("Rule (fixed before the runs): ")[1].split("\n\n")[0].replace("\n", " "),
               "n": n, "order_salt": ORDER_SALT,
               "frozen_sha256": {f: sha(os.path.join(tests, f)) for f in FROZEN},
               "graded_order": [{"unit": e["unit"], "candidate": e["candidate"]} for e in eps],
               "candidates": record}, open(os.path.join(tests, "hidden", "episode_selection.json"), "w"), indent=1)
    print("kept candidates", [c["index"] for c in kept])


if __name__ == "__main__":
    main()

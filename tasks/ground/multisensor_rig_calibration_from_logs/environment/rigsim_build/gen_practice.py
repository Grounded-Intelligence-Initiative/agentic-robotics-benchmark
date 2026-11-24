"""Builds the agent-visible practice rigs and their truth folders at image build time.

Runs only in the Dockerfile's `practice` build stage, beside a byte-identical copy of the
grader's generator (rigsim.py). The final image receives the generated folders, never the
generator.

    python3 gen_practice.py <out_root>
      -> <out_root>/practice/rig_{a,b,c}/          (what a hidden rig looks like)
         <out_root>/practice_truth/rig_{a,b,c}/    (ground_truth.yaml, faults.json, heldout.npz)
"""

import json
import multiprocessing
import os
import sys

os.environ.setdefault("OMP_NUM_THREADS", "1")
os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rigsim  # noqa: E402

SEEDS = {"rig_a": [2026, 11, 22, 1], "rig_b": [2026, 11, 22, 2], "rig_c": [2026, 11, 22, 3]}


def build(args):
    name, root = args
    rig = rigsim.make_rig(SEEDS[name])
    records = rigsim.write_rig(rig, name, os.path.join(root, "practice", name))
    tdir = os.path.join(root, "practice_truth", name)
    os.makedirs(tdir, exist_ok=True)
    rigsim._dump_yaml(rigsim.truth_dict(rig), os.path.join(tdir, "ground_truth.yaml"))
    with open(os.path.join(tdir, "faults.json"), "w") as fh:
        json.dump(records, fh, indent=1)
    rigsim.save_heldout(rigsim.heldout(rig), os.path.join(tdir, "heldout.npz"))
    return name, len(records)


if __name__ == "__main__":
    root = sys.argv[1]
    with multiprocessing.Pool(3) as pool:
        for name, n in pool.map(build, [(k, root) for k in sorted(SEEDS)]):
            print("[practice] %s written, %d fault(s)" % (name, n), flush=True)

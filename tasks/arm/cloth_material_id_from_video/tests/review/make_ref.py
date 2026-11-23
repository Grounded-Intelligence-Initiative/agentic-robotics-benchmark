"""Maintainers' tool (never run by the grader): builds the hidden reference of cloth_material_id_from_video.

Writes, from the true materials below and a fixed seed:
  tests/ref/fabrics.json              true material, mass and the hidden tests (with targets) of A, B, C
  solution/ref/params.json            the true materials in the submission format (the oracle's artifact)
  environment/gen/fabrics.json        materials and masses of P, A, B, C for the build-time video renderer
  environment/setup/payload/data/fabrics/P/gt_params.json   P's ground truth, shown to the agent
Every test is checked: its target is reached by planning with the true material (oracle quality 1.0) and its
outcome curve is monotone with a usable slope around the target.

usage (from the task folder, with MuJoCo 3.16 and numpy):  python3 tests/review/make_ref.py [--workers 6]
"""
import argparse
import json
import os
import sys
from concurrent.futures import ProcessPoolExecutor

import numpy as np

TASK = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, os.path.join(TASK, "environment", "setup", "payload"))
import cphys as W  # noqa: E402
import ctasks as T  # noqa: E402

AREA = W.SIZE_X * W.SIZE_Y
# The true materials (MuJoCo flex shell parameters) and areal densities (kg/m^2). P is the practice fabric.
TRUE = {
    "P": (dict(young=8.0e4, poisson=0.30, thickness=4.0e-3, damping=0.003, mu=0.45), 0.14),
    "A": (dict(young=2.0e5, poisson=0.25, thickness=1.5e-3, damping=0.0015, mu=0.28), 0.06),
    "B": (dict(young=3.0e5, poisson=0.30, thickness=8.0e-3, damping=0.006, mu=0.62), 0.34),
    "C": (dict(young=6.0e3, poisson=0.40, thickness=4.0e-3, damping=0.004, mu=0.85), 0.20),
}
SEED = 20261122
N_PER_FAMILY = 2
TARGETS = {"hang": (0.012, 0.050), "slide": (0.020, 0.065), "drape": (-0.15, -0.02)}
MIN_SLOPE = {"hang": 0.15, "slide": 0.08, "drape": 0.15}   # |dy/du| a target segment needs


def draw_test(fam, rng):
    if fam == "hang":
        return dict(fam=fam, pose=[float(rng.uniform(-0.15, 0.05)), float(rng.uniform(-0.10, 0.10)),
                                   float(rng.uniform(-np.pi, np.pi))],
                    corner=[int(rng.choice([-1, 1])), int(rng.choice([-1, 1]))])
    if fam == "slide":
        return dict(fam=fam, pose=[float(rng.uniform(0.0, 0.10)), float(rng.uniform(-0.12, 0.12)),
                                   float(rng.uniform(-0.35, 0.35))],
                    side=-1, d_release=float(rng.uniform(0.09, 0.14)))
    return dict(fam=fam, y=float(rng.uniform(-0.12, 0.12)), hang_x=float(rng.uniform(0.03, 0.06)))


def check_test(args):
    """Curve of the true world over the control range, a target on a monotone stretch, the oracle's quality."""
    key, test, seed = args
    mat, rho = TRUE[key]
    mass = rho * AREA
    rng = np.random.default_rng(seed)
    lo, hi = T.U_RANGE[test["fam"]]
    us = np.linspace(lo, hi, 9)
    ys = [T.outcome(mat, mass, test, u) for u in us]
    if any(y is None for y in ys):
        return key, test, None, "unstable"
    ys = np.array(ys)
    tlo, thi = TARGETS[test["fam"]]
    slope = np.diff(ys) / np.diff(us)
    ok = []
    for i in range(len(us) - 1):
        a, b = min(ys[i], ys[i + 1]), max(ys[i], ys[i + 1])
        lo_t, hi_t = max(a, tlo), min(b, thi)
        if abs(slope[i]) > MIN_SLOPE[test["fam"]] and np.sign(slope[i]) == np.sign(ys[-1] - ys[0]) and hi_t - lo_t > 0.002:
            ok.append((i, lo_t, hi_t))
    if not ok:
        return key, test, None, "no monotone target segment"
    i, lo_t, hi_t = ok[int(rng.integers(len(ok)))]
    test = dict(test, target=float(rng.uniform(lo_t + 0.0005, hi_t - 0.0005)))
    q, info = T.run_test(mat, mat, mass, test)
    if q < 1.0:
        return key, test, None, "oracle quality %.3f (%s)" % (q, info)
    test["oracle_u"] = float(info["u"])
    return key, test, dict(curve_u=us.round(4).tolist(), curve_y=ys.round(5).tolist()), "ok"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--workers", type=int, default=6)
    ap.add_argument("--cache", help="jsonl of checked candidates to reuse and extend (outside the task folder)")
    a = ap.parse_args()
    cache = {}
    if a.cache and os.path.exists(a.cache):
        for line in open(a.cache):
            r = json.loads(line)
            cache[r["id"]] = r
    rng = np.random.default_rng(SEED)
    want = {k: {f: [] for f in T.TOL} for k in ("A", "B", "C")}
    candidates = []
    for k in want:
        for fam in T.TOL:
            for _ in range(N_PER_FAMILY + 2):          # spares in case a draw has no usable target
                candidates.append((k, draw_test(fam, rng), int(rng.integers(2 ** 63))))
    def cid(c):
        return json.dumps([c[0], c[1], c[2], MIN_SLOPE[c[1]["fam"]], TARGETS[c[1]["fam"]]], sort_keys=True)
    todo = [c for c in candidates if cid(c) not in cache]
    results = {}
    with ProcessPoolExecutor(a.workers) as ex:
        for c, r in zip(todo, ex.map(check_test, todo)):
            results[cid(c)] = r
            if a.cache:
                with open(a.cache, "a") as f:
                    f.write(json.dumps({"id": cid(c), "r": [r[0], r[1], r[2], r[3]]}) + "\n")
    for c in candidates:
        k, test, curve, why = results[cid(c)] if cid(c) in results else cache[cid(c)]["r"]
        print(k, test["fam"], why, {kk: v for kk, v in test.items() if kk != "fam"}, flush=True)
        if curve is not None and len(want[k][test["fam"]]) < N_PER_FAMILY:
            want[k][test["fam"]].append(test)
    for k in want:
        for fam, ts in want[k].items():
            assert len(ts) == N_PER_FAMILY, "fabric %s family %s has %d usable tests" % (k, fam, len(ts))
    ref = {"fabrics": {k: {"material": TRUE[k][0], "mass_kg": TRUE[k][1] * AREA,
                           "tests": [t for fam in T.TOL for t in want[k][fam]]} for k in want},
           "seed": SEED}
    os.makedirs(os.path.join(TASK, "tests", "ref"), exist_ok=True)
    json.dump(ref, open(os.path.join(TASK, "tests", "ref", "fabrics.json"), "w"), indent=1)
    os.makedirs(os.path.join(TASK, "solution", "ref"), exist_ok=True)
    json.dump({"fabrics": {k: TRUE[k][0] for k in ("A", "B", "C")}},
              open(os.path.join(TASK, "solution", "ref", "params.json"), "w"), indent=1)
    os.makedirs(os.path.join(TASK, "environment", "gen"), exist_ok=True)
    json.dump({"fabrics": {k: {"material": m, "mass_kg": rho * AREA} for k, (m, rho) in TRUE.items()}},
              open(os.path.join(TASK, "environment", "gen", "fabrics.json"), "w"), indent=1)
    pdir = os.path.join(TASK, "environment", "setup", "payload", "data", "fabrics", "P")
    os.makedirs(pdir, exist_ok=True)
    json.dump({"fabrics": {"P": TRUE["P"][0]}}, open(os.path.join(pdir, "gt_params.json"), "w"), indent=1)
    print("written")


if __name__ == "__main__":
    main()

"""Maintainers' tool (never run by the grader): builds the hidden reference of cloth_material_id_from_video.

The real fabrics are creal materials (tests/creal.py: a finer sheet in air with speed-dependent friction); the robot
plans in cphys. So the task's defined answer for each fabric is not its real material but the cphys material that
transfers best: the REFERENCE, calibrated here against the real fabric on calibration tests that are drawn
separately from the hidden tests (same families and distributions, other poses, corners, directions, controls).

Stages (each cached in --cache, so a rerun only computes what changed):
  1. curves   the real outcome y_real(u) on a grid of the control range, for every hidden-test candidate and every
              calibration test (creal);
  2. tests    hidden tests: a target on a monotone stretch of the real curve, two per family per fabric;
  3. calibrate the reference cphys material per fabric: Nelder-Mead (log young, poisson, log thickness, log damping,
              mu) from the real material, minimising sum over calibration tests and grid points of
              ((y_cphys(u) - y_real(u)) / tolerance)^2 (each term capped at 16);
  4. score    the reference on the hidden tests exactly as the grader does (plan in cphys, execute in creal); the
              mean is the anchor (tests/anchor.json is written by hand from this number).

Writes tests/ref/fabrics.json (real material, mass, hidden tests), solution/ref/params.json (the reference of A, B,
C), environment/gen/fabrics.json (real materials and masses of P, A, B, C, for the renderer) and P's gt_params.json
(P's reference, shown to the agent).

usage (from the task folder, MuJoCo 3.16 + numpy + scipy): python3 tests/review/make_ref.py --workers 6 --cache FILE
"""
import argparse
import json
import threading
import os
import sys
from concurrent.futures import ProcessPoolExecutor

import numpy as np

TASK = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, os.path.join(TASK, "tests"))
import cphys as W  # noqa: E402
import creal as R  # noqa: E402
import ctasks as T  # noqa: E402

AREA = W.SIZE_X * W.SIZE_Y
# The real materials (creal parameters) and areal densities (kg/m^2). P is the practice fabric.
TRUE = {
    "P": (dict(young=8.0e4, poisson=0.30, thickness=4.0e-3, damping=0.003, mu=0.45), 0.14),
    "A": (dict(young=2.0e5, poisson=0.25, thickness=1.5e-3, damping=0.0015, mu=0.28), 0.06),
    "B": (dict(young=3.0e5, poisson=0.30, thickness=8.0e-3, damping=0.006, mu=0.62), 0.34),
    "C": (dict(young=6.0e3, poisson=0.40, thickness=4.0e-3, damping=0.004, mu=0.85), 0.20),
}
SEED = 20261123
N_PER_FAMILY = 2
N_CAL = 2
GRID = 7
CAL_IDX = (1, 3, 5)          # grid points the calibration compares (of GRID)
TARGETS = {"hang": (0.012, 0.050), "slide": (0.030, 0.110), "drape": (-0.15, -0.02)}
MIN_SLOPE = {"hang": 0.15, "slide": 0.04, "drape": 0.15}
CACHE = {}
CACHE_PATH = None
LOCK = threading.Lock()


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


def _key(*a):
    return json.dumps(a, sort_keys=True)


def sim_job(args):
    world, mat, mass, test, u = args
    return T.outcome(mat, mass, test, u, R if world == "real" else None)


def run_sims(pool, jobs):
    """jobs: list of (world, mat, mass, test, u); cached by value."""
    todo = [j for j in jobs if _key(*j) not in CACHE]
    for j, y in zip(todo, pool.map(sim_job, todo, chunksize=1)):
        with LOCK:
            CACHE[_key(*j)] = y
            if CACHE_PATH:
                with open(CACHE_PATH, "a") as f:
                    f.write(json.dumps({"k": _key(*j), "y": y}) + "\n")
    return [CACHE[_key(*j)] for j in jobs]


def grid(fam):
    lo, hi = T.U_RANGE[fam]
    return [float(u) for u in np.linspace(lo, hi, GRID)]


def pick_target(test, ys, rng):
    us, ys = np.array(grid(test["fam"])), np.array(ys, float)
    tlo, thi = TARGETS[test["fam"]]
    slope = np.diff(ys) / np.diff(us)
    ok = []
    for i in range(len(us) - 1):
        a, b = min(ys[i], ys[i + 1]), max(ys[i], ys[i + 1])
        lo_t, hi_t = max(a, tlo), min(b, thi)
        if abs(slope[i]) > MIN_SLOPE[test["fam"]] and np.sign(slope[i]) == np.sign(ys[-1] - ys[0]) and hi_t - lo_t > 0.002:
            ok.append((lo_t, hi_t))
    if not ok:
        return None
    lo_t, hi_t = ok[int(rng.integers(len(ok)))]
    return float(rng.uniform(lo_t + 0.0005, hi_t - 0.0005))


def theta_to_mat(th):
    return W.clamp_material(dict(young=10 ** th[0], poisson=th[1], thickness=10 ** th[2], damping=10 ** th[3], mu=th[4]))


def mat_to_theta(m):
    return np.array([np.log10(m["young"]), m["poisson"], np.log10(m["thickness"]), np.log10(max(m["damping"], 1e-5)), m["mu"]])


def calibrate(pool, key, cal_tests, real_curves, log):
    mass = TRUE[key][1] * AREA

    def loss(th):
        mat = theta_to_mat(th)
        jobs = [("cphys", mat, mass, t, grid(t["fam"])[k]) for t in cal_tests for k in CAL_IDX]
        ys = run_sims(pool, jobs)
        tot, i = 0.0, 0
        for t, yr in zip(cal_tests, real_curves):
            for k in CAL_IDX:
                yc = ys[i]
                i += 1
                e = 4.0 if yc is None or yr[k] is None else min(4.0, abs(yc - yr[k]) / T.TOL[t["fam"]])
                tot += e * e
        log("  %s loss %.2f at %s" % (key, tot, {k: float("%.4g" % v) for k, v in mat.items()}))
        return tot

    from scipy.optimize import minimize
    th0 = mat_to_theta(TRUE[key][0])
    step = np.array([0.25, 0.08, 0.15, 0.4, 0.06])
    simplex = [th0] + [th0 + np.eye(5)[i] * step[i] for i in range(5)]
    r = minimize(loss, th0, method="Nelder-Mead", options=dict(initial_simplex=np.array(simplex), maxfev=70,
                                                               xatol=1e-3, fatol=0.5))
    return theta_to_mat(r.x), float(r.fun)


def score_job(args):
    ref_mat, real_mat, mass, test = args
    return T.run_test(ref_mat, real_mat, mass, test, R)


def main():
    global CACHE_PATH
    ap = argparse.ArgumentParser()
    ap.add_argument("--workers", type=int, default=6)
    ap.add_argument("--cache")
    a = ap.parse_args()
    CACHE_PATH = a.cache
    if a.cache and os.path.exists(a.cache):
        for line in open(a.cache):
            r = json.loads(line)
            CACHE[r["k"]] = r["y"]
    logf = open(os.path.join(os.path.dirname(a.cache or "."), "make_ref_stages.log"), "a")

    def log(s):
        print(s, flush=True)
        logf.write(s + "\n")
        logf.flush()

    rng = np.random.default_rng(SEED)
    cands = {k: [] for k in TRUE}
    cal = {k: [] for k in TRUE}
    for k in TRUE:
        for fam in T.TOL:
            if k != "P":
                cands[k] += [(draw_test(fam, rng), int(rng.integers(2 ** 63))) for _ in range(N_PER_FAMILY + 1)]
            cal[k] += [draw_test(fam, rng) for _ in range(N_CAL)]
    with ProcessPoolExecutor(a.workers) as pool:
        # 1. real curves
        jobs = []
        for k in TRUE:
            mat, mass = TRUE[k][0], TRUE[k][1] * AREA
            for t, _ in cands[k]:
                jobs += [("real", mat, mass, t, u) for u in grid(t["fam"])]
            for t in cal[k]:
                jobs += [("real", mat, mass, t, u) for u in grid(t["fam"])]
        log("stage 1: %d real simulations (%d cached)" % (len(jobs), sum(_key(*j) in CACHE for j in jobs)))
        run_sims(pool, jobs)

        def real_curve(k, t):
            mat, mass = TRUE[k][0], TRUE[k][1] * AREA
            return [CACHE[_key("real", mat, mass, t, u)] for u in grid(t["fam"])]

        # 2. hidden tests
        hidden = {}
        for k in ("A", "B", "C"):
            hidden[k] = []
            for fam in T.TOL:
                got = 0
                for t, seed in cands[k]:
                    if t["fam"] != fam or got == N_PER_FAMILY:
                        continue
                    ys = real_curve(k, t)
                    if any(y is None for y in ys):
                        continue
                    tgt = pick_target(t, ys, np.random.default_rng(seed))
                    if tgt is None:
                        log("  %s %s candidate without a usable target: %s" % (k, fam, np.round(ys, 4).tolist()))
                        continue
                    hidden[k].append(dict(t, target=tgt))
                    got += 1
                assert got == N_PER_FAMILY, (k, fam, got)
        # 3. calibrate the reference
        refs = {}
        from concurrent.futures import ThreadPoolExecutor

        def cal_one(k):
            r, fl = calibrate(pool, k, cal[k], [real_curve(k, t) for t in cal[k]], log)
            log("stage 3: reference %s loss %.2f: %s" % (k, fl, r))
            return k, r
        with ThreadPoolExecutor(len(TRUE)) as th:
            refs = dict(th.map(cal_one, list(TRUE)))
        # 4. score the reference on the hidden tests, as the grader does
        per = {}
        for k in ("A", "B", "C"):
            mass = TRUE[k][1] * AREA
            res = list(pool.map(score_job, [(refs[k], TRUE[k][0], mass, t) for t in hidden[k]]))
            qs = [q for q, _ in res]
            fam_q = {}
            for q, t in zip(qs, hidden[k]):
                fam_q.setdefault(t["fam"], []).append(q)
            per[k] = float(np.mean([np.mean(v) for v in fam_q.values()]))
            for (q, info), t in zip(res, hidden[k]):
                log("  %s %s q=%.3f %s" % (k, t["fam"], q, {kk: (round(v, 4) if isinstance(v, float) else v) for kk, v in info.items()}))
        log("stage 4: reference material_transfer_score %.4f per fabric %s" % (np.mean(list(per.values())), per))
    ref = {"fabrics": {k: {"material": TRUE[k][0], "mass_kg": TRUE[k][1] * AREA, "tests": hidden[k],
                           "calibration_tests": cal[k], "reference": refs[k]} for k in ("A", "B", "C")},
           "seed": SEED, "reference_score": per}
    os.makedirs(os.path.join(TASK, "tests", "ref"), exist_ok=True)
    json.dump(ref, open(os.path.join(TASK, "tests", "ref", "fabrics.json"), "w"), indent=1)
    os.makedirs(os.path.join(TASK, "solution", "ref"), exist_ok=True)
    json.dump({"fabrics": {k: refs[k] for k in ("A", "B", "C")}},
              open(os.path.join(TASK, "solution", "ref", "params.json"), "w"), indent=1)
    os.makedirs(os.path.join(TASK, "environment", "gen"), exist_ok=True)
    json.dump({"fabrics": {k: {"material": m, "mass_kg": rho * AREA} for k, (m, rho) in TRUE.items()}},
              open(os.path.join(TASK, "environment", "gen", "fabrics.json"), "w"), indent=1)
    pdir = os.path.join(TASK, "environment", "setup", "payload", "data", "fabrics", "P")
    os.makedirs(pdir, exist_ok=True)
    json.dump({"fabrics": {"P": refs["P"]}}, open(os.path.join(pdir, "gt_params.json"), "w"), indent=1)
    log("written")


if __name__ == "__main__":
    main()

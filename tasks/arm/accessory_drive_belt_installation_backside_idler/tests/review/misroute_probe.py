"""Grader probe (maintainers' tool; never run by the grader): the acceptance on belts laid on the drive by hand
(kinematically, cell internals), the misroutes the I gate is for, on the hidden engines, through the grader's own path
(BeltCellServer.finalize: switch-off, settle, run-in, inspection, score).

    python3 misroute_probe.py <panda dir> [seed ...]          # default: tests/hidden/seeds.json

Placements (each settled 2 s with the arms at home before finish):
  correct  : C, ALT, I (back), WP, T (back), the tensioner where the chain closes (control: expected 1.0)
  over_I   : the belt over I (its ribs on I's top, I inside the loop)
  front_I  : the belt not round I, its top run passing in front of I's front flange
Expected: over_I and front_I score 0 with the pointer able to rest inside the window (over_I does).
"""
import math
import os
import sys

import mujoco
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from beltcell import drive as dr  # noqa: E402

V = dr.VARIANTS[dr.CELL_VARIANT]
T = V["tensioner"]
SENSE = dr.opening_sense(T)


def circles(drive, th, l, sflip=(), drop=()):
    out = []
    for c in drive.circles(th, l):
        if c.name in drop:
            continue
        if c.name in sflip:
            c = dr.Circle(c.name, c.c, c.rho, -c.s)
        out.append(c)
    return out


def _gap(cs, n, l):
    path, tot, _ = dr.loop_path(cs)
    P = np.vstack([path, path])
    pts, g = dr.resample_closed(P, n, l)
    t0 = path[1] - path[0]
    t0 /= np.linalg.norm(t0)
    return float(g @ t0), path, pts


def solve_theta(drive, n, l, **kw):
    lo, hi = T["theta_closed"], T["theta_closed"] + SENSE * math.radians(50.0)
    glo, ghi = _gap(circles(drive, lo, l, **kw), n, l)[0], _gap(circles(drive, hi, l, **kw), n, l)[0]
    if glo * ghi > 0:
        return None
    for _ in range(50):
        mid = 0.5 * (lo + hi)
        gm = _gap(circles(drive, mid, l, **kw), n, l)[0]
        if gm * glo > 0:
            lo, glo = mid, gm
        else:
            hi = mid
    return 0.5 * (lo + hi)


def sag_path(path, extra, i0, i1):
    """replace the straight run path[i0:i1] by a parabola-like sag of `extra` length (outwards = to its right)"""
    a, b = path[i0], path[i1]
    L = np.linalg.norm(b - a)
    u = (b - a) / L
    nrm = np.array([u[1], -u[0]])        # right of the running direction: outside a CCW loop
    # find the sag depth h of y = 4h s(1-s) whose arc length is L + extra
    def arc(h):
        s = np.linspace(0, 1, 400)
        y = 4 * h * s * (1 - s)
        return float(np.sum(np.hypot(np.diff(s * L), np.diff(y))))
    lo, hi = 0.0, 0.5 * L
    for _ in range(60):
        mid = 0.5 * (lo + hi)
        if arc(mid) < L + extra:
            lo = mid
        else:
            hi = mid
    h = 0.5 * (lo + hi)
    s = np.linspace(0, 1, max(2, i1 - i0))
    seg = a[None, :] + s[:, None] * (b - a)[None, :] + (4 * h * s * (1 - s))[:, None] * nrm[None, :]
    return np.vstack([path[:i0], seg, path[i1:]]), h


def place(srv, kind="correct", depth_fn=None):
    sc = srv.scene
    d = sc.data
    n, l = sc.belt.n, sc.belt.l
    info = {"kind": kind}
    kw = {}
    if kind == "over_I":
        kw = {"sflip": ("I",)}
    elif kind == "front_I":
        kw = {"drop": ("I",)}
    if kind == "T_rib":
        th = T["theta_closed"]
        cs = circles(sc.drive, th, l, sflip=("T",))
        path, tot, _ = dr.loop_path(cs, step=0.0005)
        extra = n * l - tot
        info["extra_mm"] = round(1000 * extra, 1)
        # the C>ALT span: the run from C's exit to ALT's entry (the longest straight run)
        seg = np.linalg.norm(np.diff(path, axis=0), axis=1)
        # straight runs: consecutive collinear samples; pick the longest run of equal direction
        dirs = np.arctan2(np.diff(path[:, 1]), np.diff(path[:, 0]))
        best, k0 = (0, 0, 0), 0
        for k in range(1, len(dirs) + 1):
            if k == len(dirs) or abs((dirs[k] - dirs[k0] + math.pi) % (2 * math.pi) - math.pi) > 1e-6:
                if k - k0 > best[0]:
                    best = (k - k0, k0, k)
                k0 = k
        _, i0, i1 = best
        path, h = sag_path(path, extra, i0, i1)
        info["sag_mm"] = round(1000 * h, 1)
        pts, g = dr.resample_closed(np.vstack([path, path]), n, l)
        theta = th
    else:
        theta = solve_theta(sc.drive, n, l, **kw)
        if theta is None:
            info["error"] = "no closing angle in the tensioner travel"
            return info
        g, path, pts = _gap(circles(sc.drive, theta, l, **kw), n, l)
    info["theta_place_deg"] = round(math.degrees(theta), 2)
    Ic = np.asarray(V["pulleys"]["I"]["c"])
    rI = V["pulleys"]["I"]["r"] + dr.TPULLEY["flange_h"]
    P3 = []
    for x, z in pts:
        dd = dr.BELT_PLANE_D
        if kind == "front_I":
            # in front of I's front flange (depth 0.043 + half the belt width + 2 mm), ramped over 40 mm
            r = math.hypot(x - Ic[0], z - Ic[1])
            dfront = dr.TPULLEY["d_front1"] + 0.5 * dr.BELT["width"] + 0.002
            if r < rI + 0.040:
                f = min(1.0, max(0.0, (rI + 0.040 - r) / 0.040))
                dd = dr.BELT_PLANE_D + f * (dfront - dr.BELT_PLANE_D)
        if depth_fn is not None:
            dd = depth_fn(x, z, dd)
        P3.append(sc.E_to_world(x, z, dd))
    P3 = np.asarray(P3)
    nvec = np.tile(sc.R_E[:, 1], (n, 1))
    d.qpos[sc.drive.t_qadr] = SENSE * (theta - T["theta_closed"])
    d.qvel[sc.drive.t_dof] = 0.0
    sc.belt.set_points(d, P3, nvec)
    mujoco.mj_forward(sc.model, d)
    info["closure_gap_mm"] = round(1000 * float(np.linalg.norm(g)) if np.ndim(g) else 1000 * abs(g), 2)
    return info


def main():
    import json
    from beltcell.server import BeltCellServer
    tests = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    panda = sys.argv[1]
    seeds = [int(s) for s in sys.argv[2:]] or json.load(open(os.path.join(tests, "hidden", "seeds.json")))["seeds"]
    cfg = json.load(open(os.path.join(tests, "grader_config.json")))
    for s in seeds:
        for kind in ("correct", "over_I", "front_I"):
            srv = BeltCellServer(os.path.abspath(panda), config=cfg, render=False, log=lambda m: None)
            srv.reset(s)
            info = place(srv, kind)
            for _ in range(200):
                srv.physics_tick()
            srv.done, srv.done_reason = True, "probe"
            summ = srv.finalize()
            ins = (summ.get("run_in") or {}).get("inspect") or {}
            print(json.dumps({"seed": s, "kind": kind, "score": summ["episode_score"], "damage": summ["damage"],
                              "theta_deg": (ins.get("tensioner") or {}).get("theta_deg"),
                              "in_window": (summ.get("items") or {}).get("in_window"),
                              "I": {k: (ins.get("I") or {}).get(k) for k in ("wind", "cover", "ok")},
                              "failed": [k for k, v in (summ.get("items") or {}).items() if not v], "placed": info}), flush=True)


if __name__ == "__main__":
    main()

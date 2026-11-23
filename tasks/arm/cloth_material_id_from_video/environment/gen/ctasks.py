"""ctasks: the manipulation tasks the grader scores (cloth_material_id_from_video).

Byte-identical copies are used by the hidden grader and shipped to you. It holds no hidden values: the grader's
hidden tests only choose poses, corners, directions and targets for these functions.

Every test starts with the fabric lying flat at rest at a pose (x, y, yaw) on the table and has one control value u
that the robot chooses by planning: `plan(material, mass, test)` simulates the test in a world made of the given
material and searches u so that the predicted outcome equals the test's target. The grader plans with YOUR
material and then executes the planned u once in the world made of the TRUE material; the score of the test
depends only on how far the real outcome lands from the target (`test_quality`).

Families (outcome y, control u):
  hang   grasp a corner, lift, hold, lower the clamp to height u, settle;
         y = height of the fabric's lowest point above the table.                tolerance 3 mm
  slide  grasp the middle of a short edge, lift it 15 mm, drag it along the table, reaching speed u, let go while
         moving; y = how far the fabric's leading point ends up beyond the release point.   tolerance 6 mm
  drape  grasp a short edge, hang the fabric beyond the table's front edge, lay it back over the edge until the
         clamp is at x = u on the table (the clamp keeps holding); y = height of the lowest point of the part
         hanging over the edge (negative: below the table top).                tolerance 5 mm
"""
import numpy as np

import cphys as W

TOL = {"hang": 0.003, "slide": 0.006, "drape": 0.005}
U_RANGE = {"hang": (0.24, 0.52), "slide": (0.12, 0.90), "drape": (0.02, 0.20)}
PLAN_ITERS = 8


# ------------------------------------------------------------------------------------------------------------------
# programs
# ------------------------------------------------------------------------------------------------------------------
def prog_hang(test, u):
    """test: pose, corner [sx, sy]. Returns (traj, first recorded step of the settled window)."""
    pose = test["pose"]
    p, yaw = W.corner_grasp(pose, *test["corner"])
    top = np.array([p[0], p[1], 0.45])
    low = np.array([p[0], p[1], float(u)])
    keys = [(0.0, p, yaw, 0, 0), (0.1, p, yaw, 0, 1), (1.2, top, yaw, 0, 1), (2.4, top, yaw, 0, 1),
            (3.4, low, yaw, 0, 1), (5.4, low, yaw, 0, 1)]
    C = W.program_to_traj(keys)
    return C, len(C) - int(0.2 / W.DT)


def fling_traj(p0, yaw, direction, v, d_acc, d_release, lift=0.015):
    """Grasp at p0, lift by `lift`, accelerate along `direction` (world yaw) to speed v over d_acc, keep the speed,
    open the jaws d_release from the start while still moving, then rise and slow down. Returns (traj, release point)."""
    u = np.array([np.cos(direction), np.sin(direction), 0.0])
    q = W.clamp_quat(yaw, 0.0)
    rows = []

    def add(p, c):
        rows.append(np.r_[p, q, c])

    for _ in range(int(0.1 / W.DT)):
        add(p0, 0)
    for _ in range(int(0.1 / W.DT)):
        add(p0, 1)
    pl = np.asarray(p0, float) + [0, 0, lift]
    n = int(0.3 / W.DT)
    for i in range(n):
        s = (i + 1) / n
        s = s * s * s * (10 - 15 * s + 6 * s * s)
        add(p0 + s * (pl - p0), 1)
    a = v * v / (2 * d_acc)
    t, x = 0.0, 0.0
    while x < d_release:
        t += W.DT
        x = 0.5 * a * t * t if t < v / a else d_acc + v * (t - v / a)
        add(pl + u * x, 1)
    pr = pl + u * x
    for i in range(int(2.0 / W.DT)):
        tt = (i + 1) * W.DT
        add(pr + u * v * 0.1 * (1 - np.exp(-tt / 0.1)) + [0, 0, 0.3 * min(1.0, tt / 0.5)], 0)
    return np.array(rows), pr


def prog_slide(test, u):
    """test: pose, side (+1/-1 short edge), d_release. The drag runs along the fabric's long axis, away from it."""
    pose = test["pose"]
    p, yaw = W.edge_grasp(pose, test["side"])
    direction = pose[2] + (0.0 if test["side"] > 0 else np.pi)
    C, pr = fling_traj(p, yaw, direction, float(u), 0.06, test["d_release"])
    return C, pr, direction


def prog_drape(test, u):
    """test: y (fabric centre y; the fabric starts with its long axis along x, back edge towards -x), hang_x."""
    pose = (W.EDGE_X - 0.22, test["y"], 0.0)
    p, yaw = W.edge_grasp(pose, -1)
    above = np.array([W.EDGE_X + test["hang_x"], test["y"], 0.22])
    land = np.array([float(u), test["y"], 0.006])
    keys = [(0.0, p, yaw, 0, 0), (0.1, p, yaw, 0, 1), (0.6, p + [0, 0, 0.12], yaw, 0, 1), (1.6, above, yaw, 0, 1),
            (2.4, above, yaw, 0, 1), (4.0, land, yaw, 0, 1), (5.6, land, yaw, 0, 1)]
    C = W.program_to_traj(keys)
    return C, len(C) - int(0.2 / W.DT)


# ------------------------------------------------------------------------------------------------------------------
# outcomes
# ------------------------------------------------------------------------------------------------------------------
def outcome(material, mass, test, u):
    """Simulate `test` with control u in the world made of `material`. Returns y (float) or None if the
    simulation diverged."""
    fam = test["fam"]
    if fam == "hang":
        C, s0 = prog_hang(test, u)
        sim = W.Sim(material, mass)
        sim.place_flat(test["pose"])
        fr = sim.run(C, record_every=10, start=s0)
        if sim.unstable or not len(fr):
            return None
        return float(np.mean([f[:, 2].min() for f in fr]))
    if fam == "slide":
        C, pr, direction = prog_slide(test, u)
        sim = W.Sim(material, mass)
        sim.place_flat(test["pose"])
        fr = sim.run(C, record_every=len(C) - 1)
        if sim.unstable or len(fr) < 2:
            return None
        d = np.array([np.cos(direction), np.sin(direction)])
        return float((fr[-1][:, :2] @ d).max() - pr[:2] @ d)
    if fam == "drape":
        C, s0 = prog_drape(test, u)
        sim = W.Sim(material, mass)
        sim.place_flat((W.EDGE_X - 0.22, test["y"], 0.0))
        fr = sim.run(C, record_every=10, start=s0)
        if sim.unstable or not len(fr):
            return None
        return float(np.mean([f[:, 2].min() for f in fr]))
    raise ValueError("unknown family %r" % fam)


def plan(material, mass, test, iters=PLAN_ITERS):
    """Choose u in U_RANGE[fam] so that outcome(material, mass, test, u) is as close as possible to test["target"]:
    the two ends of the range, then Illinois regula falsi. Returns (u, predicted y)."""
    fam, target = test["fam"], float(test["target"])
    lo, hi = U_RANGE[fam]
    tol = 0.25 * TOL[fam]
    pts = []

    def f(u):
        y = outcome(material, mass, test, u)
        pts.append((u, y))
        return y

    ylo, yhi = f(lo), f(hi)
    if ylo is None or yhi is None:
        good = [(u, y) for u, y in pts if y is not None]
        if not good:
            return 0.5 * (lo + hi), None
        u, y = min(good, key=lambda p: abs(p[1] - target))
        return u, y
    a, fa, b, fb = lo, ylo - target, hi, yhi - target
    if fa * fb > 0:                                   # target outside what this model can reach: the closer end
        return (lo, ylo) if abs(fa) <= abs(fb) else (hi, yhi)
    side = 0
    for _ in range(iters):
        if abs(fa) <= tol:
            return a, fa + target
        if abs(fb) <= tol:
            return b, fb + target
        c = (a * fb - b * fa) / (fb - fa)
        yc = f(c)
        if yc is None:
            c = 0.5 * (a + b)
            yc = f(c)
            if yc is None:
                break
        fc = yc - target
        if fc * fb > 0:
            b, fb = c, fc
            if side == -1:
                fa *= 0.5
            side = -1
        else:
            a, fa = c, fc
            if side == +1:
                fb *= 0.5
            side = +1
    u, y = min([(u, y) for u, y in pts if y is not None], key=lambda p: abs(p[1] - target))
    return u, y


def test_quality(test, y_true):
    """1 within the family tolerance of the target, falling linearly to 0 at twice the tolerance."""
    if y_true is None or not np.isfinite(y_true):
        return 0.0
    e = abs(y_true - float(test["target"]))
    return float(np.clip(2.0 - e / TOL[test["fam"]], 0.0, 1.0))


def run_test(material, true_material, mass, test):
    """Plan with `material`, execute once with `true_material`. Returns (quality, info)."""
    u, y_pred = plan(material, mass, test)
    y_true = outcome(true_material, mass, test, u)
    return test_quality(test, y_true), {"u": u, "predicted": y_pred, "real": y_true, "target": test["target"]}

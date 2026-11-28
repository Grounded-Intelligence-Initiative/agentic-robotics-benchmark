"""Least thumb normal force that holds the specimen against a field (quasi-static, from the true contacts).

The specimen is one rigid body; the contacts are the three pads at their true points (relative to the specimen's
centre of mass), normals along the specimen's squeeze axis (the thumb +u on one face, the fingers -u on the other),
the index and middle normal forces equal (the reference commands equal shares). Each contact's force is a
non-negative combination of generators on the boundary of the elliptic friction cone with torsion
(||(t1 / mu, t2 / mu, tau / gamma)|| <= N): 8 tangential, 2 torsional and 16 mixed generators, an inscribed polytope
(conservative). Equilibrium of forces and moments about the centre of mass with the field force m g_hand; the thumb's
normal force is minimised. least_thumb_force returns (N_T, N_I, N_M), or infinities if no squeeze holds.
"""
import numpy as np
from scipy.optimize import linprog

_TH = np.linspace(0, 2 * np.pi, 8, endpoint=False)


def generators(mu, gam):
    G = []
    for th in _TH:
        G.append((1.0, mu * np.cos(th), mu * np.sin(th), 0.0))
    for s in (1, -1):
        G.append((1.0, 0.0, 0.0, s * gam))
    for th in _TH:
        for s in (1, -1):
            G.append((1.0, mu * np.cos(th) / np.sqrt(2), mu * np.sin(th) / np.sqrt(2), s * gam / np.sqrt(2)))
    return np.array(G)


def least_thumb_force(r_T, r_I, r_M, u, field_force, mu, gam):
    """r_*: contact points relative to the centre of mass (hand frame, m); u: squeeze axis (unit); field_force:
    m * g_hand (N). Returns (N_T_min, N_I, N_M) or (inf, inf, inf)."""
    u = np.asarray(u, float) / np.linalg.norm(u)
    e1 = np.cross(u, [0, 0, 1.0])
    if np.linalg.norm(e1) < 1e-6:
        e1 = np.cross(u, [0, 1.0, 0])
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(u, e1)
    Gn = generators(mu, gam)
    cols = []
    owner = []
    for k, (r, nsgn) in enumerate(((r_T, 1.0), (r_I, -1.0), (r_M, -1.0))):
        n = nsgn * u
        for g in Gn:
            f = g[0] * n + g[1] * e1 + g[2] * e2
            tq = np.cross(r, f) + g[3] * n
            cols.append(np.concatenate([f, tq]))
            owner.append(k)
    A = np.array(cols).T                       # 6 x ncol
    owner = np.array(owner)
    ncol = A.shape[1]
    # equalities: wrench balance; N_I == N_M
    Aeq = np.vstack([A, (owner == 1).astype(float) - (owner == 2).astype(float)])
    beq = np.concatenate([-np.concatenate([field_force, np.zeros(3)]), [0.0]])
    c = (owner == 0).astype(float)
    res = linprog(c, A_eq=Aeq, b_eq=beq, bounds=[(0, None)] * ncol, method="highs")
    if res.status != 0:
        return float("inf"), float("inf"), float("inf")
    x = res.x
    return float(x[owner == 0].sum()), float(x[owner == 1].sum()), float(x[owner == 2].sum())

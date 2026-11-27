"""The rigid rest on a level plate: quasi-static toppling of the true unit held at the plan's pelvis attitude and
lowered (topple, _hull2d and _inside_or_edge): a corner touches within 1e-7 m; each rotation is exact to the next
contact; at most 200 steps; about the first contact point, then the contact edge, in the sense gravity tips it, until
the centre of mass's projection lies inside the hull of the touching corners; over 25 deg of rotation, balanced on a
point, no further contact or out of steps = a fall (the posture scores 0)."""
from __future__ import annotations

import math

import numpy as np

from .util import DEG, axis_rot

CONTACT_TOL = 1e-7        # m: a corner within this of the plate touches (printed in the acceptance sheet)
MAX_STEPS = 200           # printed in the acceptance sheet
MAX_ROT = 25 * DEG        # over 25 deg of rotation = a fall (printed in the acceptance sheet)
# the other thresholds below (an edge shorter than 1e-9 m, a centre within 1e-12 m of a point, a lever below 1e-15, a
# rotation below 1e-13 rad) are numerical guards chosen for this grader


def topple(points, com, max_rot=MAX_ROT):
    """points (n, 3) and com (3,) in the world with the body held at its planned attitude. Returns (R, t, ok, info):
    x_rest = R x + t; info {"rot_deg", "contacts"} or {"why"}"""
    P = np.array(points, float); c = np.array(com, float)
    R = np.eye(3); t = np.zeros(3)
    shift = -P[:, 2].min(); P[:, 2] += shift; c[2] += shift; t[2] += shift
    contacts = [int(np.argmin(P[:, 2]))]
    total = 0.0
    zhat = np.array([0.0, 0.0, 1.0])
    for it in range(MAX_STEPS):
        on = [i for i in range(len(P)) if P[i, 2] < CONTACT_TOL]
        if len(on) >= 3:
            pts = P[on][:, :2]
            hull = _hull2d(pts)
            inside, edge = _inside_or_edge(hull, c[:2], pts)
            if inside:
                return R, t, True, {"rot_deg": total / DEG, "contacts": len(on)}
            contacts = [on[edge[0]], on[edge[1]]]
        elif len(on) == 2:
            contacts = on
        else:
            contacts = [on[0]] if on else [int(np.argmin(P[:, 2]))]
        if len(contacts) == 2:
            p, q = P[contacts[0]], P[contacts[1]]
            e = q - p; L = np.linalg.norm(e[:2])
            if L < 1e-9:
                contacts = [contacts[0]]
            else:
                e = np.array([e[0], e[1], 0.0]) / L
                tt = (c - p) @ e / L
                if tt < 0.0:
                    contacts = [contacts[0]]
                elif tt > 1.0:
                    contacts = [contacts[1]]
        if len(contacts) == 1:
            p = P[contacts[0]]
            dh = c - p; dh[2] = 0.0
            if np.linalg.norm(dh) < 1e-12:
                return R, t, False, {"why": "balanced on a point"}
            a = np.cross(zhat, dh / np.linalg.norm(dh))
        else:
            p = P[contacts[0]]
            a = e.copy()
            if np.cross(a, c - p)[2] > 0:
                a = -a
        best = None
        for i in range(len(P)):
            if i in contacts:
                continue
            v = P[i] - p
            w = np.cross(a, v)[2]
            vz = v[2]
            if w < -1e-15:
                th = math.atan2(vz, -w)
                if th > 1e-13 and (best is None or th < best[0]):
                    best = (th, i)
        if best is None:
            return R, t, False, {"why": "no further contact"}
        th = best[0]
        total += th
        if total > max_rot:
            return R, t, False, {"why": "rotation beyond limit", "rot_deg": total / DEG}
        Ra = axis_rot(a, th)
        P = (P - p) @ Ra.T + p
        c = Ra @ (c - p) + p
        R = Ra @ R; t = Ra @ (t - p) + p
        P[:, 2] -= P[:, 2].min()
    return R, t, False, {"why": "iterations"}


def _hull2d(pts):
    idx = sorted(range(len(pts)), key=lambda i: (pts[i][0], pts[i][1]))

    def cross(o, a, b):
        return (pts[a][0] - pts[o][0]) * (pts[b][1] - pts[o][1]) - (pts[a][1] - pts[o][1]) * (pts[b][0] - pts[o][0])
    lower, upper = [], []
    for i in idx:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], i) <= 0:
            lower.pop()
        lower.append(i)
    for i in reversed(idx):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], i) <= 0:
            upper.pop()
        upper.append(i)
    return lower[:-1] + upper[:-1]


def _inside_or_edge(hull, c, pts):
    """inside the counter-clockwise hull, or the edge (i, j) the centroid -> c segment crosses"""
    n = len(hull)
    if n < 3:
        return False, (hull[0], hull[-1])
    worst = None
    cen = pts[hull].mean(0)
    for k in range(n):
        i, j = hull[k], hull[(k + 1) % n]
        a, b = pts[i], pts[j]
        cr = (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
        if cr < 0:
            d1 = (b[0] - a[0]) * (cen[1] - a[1]) - (b[1] - a[1]) * (cen[0] - a[0])
            s = d1 / (d1 - cr) if d1 != cr else 0.0
            xpt = cen + s * (c - cen)
            u = ((xpt - a) @ (b - a)) / max((b - a) @ (b - a), 1e-30)
            score = -abs(u - 0.5)
            if worst is None or score > worst[0]:
                worst = (score, (i, j))
    if worst is None:
        return True, None
    return False, worst[1]


def feet_frame_coords(c, centres, xfeet):
    """horizontal coordinates of point c in the feet frame: origin the midpoint of the two tread centres, x the mean
    foot x axis projected on the plate and normalised, z up"""
    o = 0.5 * (centres["left"] + centres["right"])
    x = np.array(xfeet, float).copy(); x[2] = 0.0; x /= np.linalg.norm(x)
    y = np.cross([0.0, 0.0, 1.0], x)
    d = c - o
    return np.array([d @ x, d @ y])

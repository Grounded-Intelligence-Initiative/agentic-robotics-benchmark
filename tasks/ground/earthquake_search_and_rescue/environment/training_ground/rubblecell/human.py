"""People-shaped things of the site (cell internals): the survivor (warm, alive, dusty clothes) and the decoys a camera
could take for one (a shop mannequin, a heap of clothes). A figure is a list of primitive parts (capsules, ellipsoids,
spheres) in a lying pose, each tagged with what it is made of (skin, hair, shirt, trousers, shoes, plastic) so the
thermal camera can give each its temperature.

Local frame of a lying figure: x along the body (head at +x), y to its left, z up; the floor at z = 0.
"""

import math

import numpy as np

__all__ = ["figure_parts", "SKIN", "SHIRT", "TROUSERS"]

SKIN = [(0.87, 0.70, 0.58), (0.76, 0.57, 0.44), (0.58, 0.41, 0.30), (0.40, 0.28, 0.20)]
HAIR = [(0.10, 0.08, 0.06), (0.30, 0.20, 0.12), (0.55, 0.45, 0.30), (0.25, 0.25, 0.25)]
SHIRT = [(0.72, 0.18, 0.15), (0.20, 0.35, 0.62), (0.85, 0.80, 0.55), (0.25, 0.48, 0.30), (0.55, 0.55, 0.58)]
TROUSERS = [(0.20, 0.24, 0.35), (0.30, 0.28, 0.25), (0.15, 0.15, 0.16), (0.45, 0.40, 0.30)]


def _rot(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def _seg(a, b, r, what, name):
    """A capsule part from a to b (local coordinates)."""
    a = np.asarray(a, float)
    b = np.asarray(b, float)
    return {"type": "capsule", "from": a, "to": b, "r": float(r), "what": what, "name": name}


def _ell(c, radii, yaw, what, name, pitch=0.0):
    return {"type": "ellipsoid", "pos": np.asarray(c, float), "size": tuple(float(v) for v in radii), "yaw": float(yaw),
            "pitch": float(pitch), "what": what, "name": name}


def figure_parts(pose="back", scale=1.0, rng=None, mannequin=False):
    """Primitive parts of a lying adult (height ~1.72 m x scale) in pose "back" (face up), "side" (on the left side,
    knees bent) or "prone" (face down, head turned). Returns a list of part dicts (local frame)."""
    s = float(scale)
    P = []
    if pose == "side":
        z = 0.14 * s            # body axis height lying on the side (half the chest depth... shoulder width / 2)
        hip = np.array([0.0, 0.0, z])
        chest = np.array([0.46 * s, 0.0, z + 0.01])
        head = np.array([0.74 * s, 0.03 * s, 0.11 * s])
        P.append(_ell((hip + chest) / 2.0, (0.30 * s, 0.11 * s, 0.17 * s), 0.0, "shirt", "torso"))
        P.append(_ell(hip + np.array([-0.04 * s, 0, 0]), (0.13 * s, 0.12 * s, 0.17 * s), 0.0, "trousers", "pelvis"))
        P.append(_ell(head, (0.11 * s, 0.085 * s, 0.10 * s), 0.0, "skin", "head"))
        P.append(_ell(head + np.array([0.02 * s, -0.01 * s, 0.03 * s]), (0.095 * s, 0.08 * s, 0.08 * s), 0.0, "hair", "hair"))
        P.append(_seg(chest + np.array([0.05 * s, 0, 0]), head - np.array([0.08 * s, 0, 0]), 0.05 * s, "skin", "neck"))
        # legs: knees bent forward (+y)
        knee_u = hip + np.array([-0.30 * s, 0.22 * s, -0.02 * s])
        knee_l = hip + np.array([-0.30 * s, 0.20 * s, 0.10 * s])
        P.append(_seg(hip + np.array([-0.05 * s, 0.02 * s, -0.04 * s]), knee_u, 0.075 * s, "trousers", "thigh_l"))
        P.append(_seg(hip + np.array([-0.05 * s, 0.02 * s, 0.06 * s]), knee_l, 0.075 * s, "trousers", "thigh_r"))
        P.append(_seg(knee_u, knee_u + np.array([-0.40 * s, -0.02 * s, 0.0]), 0.055 * s, "trousers", "shin_l"))
        P.append(_seg(knee_l, knee_l + np.array([-0.40 * s, 0.0, 0.0]), 0.055 * s, "trousers", "shin_r"))
        P.append(_ell(knee_u + np.array([-0.44 * s, 0.03 * s, 0.0]), (0.06 * s, 0.12 * s, 0.045 * s), math.pi / 2, "shoes", "foot_l"))
        P.append(_ell(knee_l + np.array([-0.44 * s, 0.03 * s, 0.0]), (0.06 * s, 0.12 * s, 0.045 * s), math.pi / 2, "shoes", "foot_r"))
        # arms: lower arm under the head, upper arm reaching forward
        sh_u = chest + np.array([0.02 * s, 0.0, 0.16 * s])
        sh_l = chest + np.array([0.02 * s, 0.0, -0.10 * s])
        el_u = sh_u + np.array([-0.05 * s, 0.26 * s, -0.08 * s])
        P.append(_seg(sh_u, el_u, 0.045 * s, "shirt", "uarm_r"))
        hand_u = el_u + np.array([0.18 * s, 0.18 * s, -0.06 * s])
        P.append(_seg(el_u, hand_u, 0.038 * s, "skin" if not mannequin else "shirt", "farm_r"))
        P.append(_ell(hand_u + np.array([0.05 * s, 0.03 * s, 0]), (0.08 * s, 0.045 * s, 0.025 * s), 0.6, "skin", "hand_r"))
        el_l = sh_l + np.array([0.20 * s, 0.10 * s, 0.0])
        P.append(_seg(sh_l + np.array([0, 0, 0.06 * s]), el_l, 0.045 * s, "shirt", "uarm_l"))
        hand_l = el_l + np.array([0.22 * s, -0.04 * s, 0.02 * s])
        P.append(_seg(el_l, hand_l, 0.038 * s, "skin", "farm_l"))
        P.append(_ell(hand_l + np.array([0.06 * s, 0, 0]), (0.08 * s, 0.045 * s, 0.025 * s), 0.0, "skin", "hand_l"))
        heads = [head]
        hands = [hand_u + np.array([0.05 * s, 0.03 * s, 0]), hand_l + np.array([0.06 * s, 0, 0])]
    else:
        prone = pose == "prone"
        zt = 0.10 * s
        hip = np.array([0.0, 0.0, zt])
        chest = np.array([0.46 * s, 0.0, zt + 0.01])
        head = np.array([0.76 * s, 0.0, 0.10 * s])
        P.append(_ell((hip + chest) / 2.0, (0.30 * s, 0.18 * s, 0.10 * s), 0.0, "shirt", "torso"))
        P.append(_ell(hip + np.array([-0.04 * s, 0, 0]), (0.13 * s, 0.17 * s, 0.10 * s), 0.0, "trousers", "pelvis"))
        P.append(_ell(head, (0.11 * s, 0.085 * s, 0.10 * s), 0.0, "skin" if not prone else "hair", "head"))
        P.append(_ell(head + np.array([0.03 * s, 0, (0.02 if not prone else 0.04) * s]),
                      (0.095 * s, 0.085 * s, 0.075 * s), 0.0, "hair", "hair"))
        P.append(_seg(chest + np.array([0.06 * s, 0, 0]), head - np.array([0.08 * s, 0, 0]), 0.05 * s, "skin", "neck"))
        spread = 0.10 * s
        for side, sg in (("l", 1.0), ("r", -1.0)):
            h0 = hip + np.array([-0.06 * s, sg * 0.09 * s, -0.02 * s])
            knee = h0 + np.array([-0.42 * s, sg * spread, 0.0])
            P.append(_seg(h0, knee, 0.075 * s, "trousers", "thigh_" + side))
            ank = knee + np.array([-0.40 * s, sg * 0.03 * s, -0.01 * s])
            P.append(_seg(knee, ank, 0.055 * s, "trousers", "shin_" + side))
            P.append(_ell(ank + np.array([-0.03 * s, 0, 0.06 * s]), (0.05 * s, 0.045 * s, 0.11 * s), 0.0, "shoes", "foot_" + side))
        hands = []
        sh_l = chest + np.array([0.02 * s, 0.19 * s, 0.0])
        sh_r = chest + np.array([0.02 * s, -0.19 * s, 0.0])
        # left arm above the head (reaching), right arm along the body
        el_l = sh_l + np.array([0.26 * s, 0.10 * s, 0.0])
        hd_l = el_l + np.array([0.24 * s, 0.02 * s, 0.0])
        P.append(_seg(sh_l, el_l, 0.045 * s, "shirt", "uarm_l"))
        P.append(_seg(el_l, hd_l, 0.038 * s, "skin" if not mannequin else "shirt", "farm_l"))
        P.append(_ell(hd_l + np.array([0.07 * s, 0, 0]), (0.08 * s, 0.045 * s, 0.025 * s), 0.0, "skin", "hand_l"))
        el_r = sh_r + np.array([-0.26 * s, -0.06 * s, 0.0])
        hd_r = el_r + np.array([-0.22 * s, 0.02 * s, 0.0])
        P.append(_seg(sh_r, el_r, 0.045 * s, "shirt", "uarm_r"))
        P.append(_seg(el_r, hd_r, 0.038 * s, "skin" if not mannequin else "shirt", "farm_r"))
        P.append(_ell(hd_r + np.array([-0.07 * s, 0, 0]), (0.08 * s, 0.045 * s, 0.025 * s), 0.0, "skin", "hand_r"))
        heads = [head]
        hands = [hd_l + np.array([0.07 * s, 0, 0]), hd_r + np.array([-0.07 * s, 0, 0])]
    if mannequin:
        for p in P:
            if p["what"] in ("skin", "hair"):
                p["what"] = "plastic"
    return P, heads, hands


def place_parts(parts, x, y, yaw, z0=0.0):
    """World placement of a figure's parts: rotate by yaw about z, translate to (x, y, z0)."""
    R = _rot(yaw)
    t = np.array([x, y, z0])
    out = []
    for p in parts:
        q = dict(p)
        if p["type"] == "capsule":
            q["from"] = R @ p["from"] + t
            q["to"] = R @ p["to"] + t
        else:
            q["pos"] = R @ p["pos"] + t
            q["yaw"] = p["yaw"] + yaw
        out.append(q)
    return out

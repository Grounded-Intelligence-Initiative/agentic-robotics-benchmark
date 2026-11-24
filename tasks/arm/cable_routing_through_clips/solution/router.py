"""Privileged reference controller for cable_routing_through_clips (serves the closed-loop wire).

It is NOT a perception system: tests/verify.py adds the cell's true state (`obs["privileged"]`: the cable's centre
line, the plug's pose, the cable's radius and length) to the observations of a submission whose .oracle_token hashes
to tests/hidden/oracle_token.sha256, and of no other. This controller routes and mates from that state; it shows that
the hidden episodes can be completed through the wire, and its measured credit is the anchor.

Routing one fixture, the way a technician does it by hand:

1. Pick the cable up at the arc length (from the connector) it needs to reach a point a few
   centimetres past the fixture, measured along the route from where the cable is already held.
2. Carry it there high enough that the cable hangs clear, so the stretch from the last fixed
   point runs straight over the fixture.
3. Clip: lower it along that line and pull a little, so the cable drops between the pins.
   Post: lower it on the outer side of the post, then swing the grip around the post so the
   cable wraps it, and pull.
4. Let go, lift, go on with the next fixture.

Then the plug: grip it behind its mating part, carry it to the socket's mouth, put it down flat there and take a
fresh grip (a cable bent at the plug turns it in the fingers during the carry), and slide it home along the board,
correcting its line from the observation each step, until the socket reports it mated.

All decisions (which fixture, which arc length) are kept in relative terms and every target is
recomputed from the observation in hand, so a step asked twice with the same `t` is answered
from each request's own observation and never advances anything.
"""

import copy
import json
import math
import sys

import numpy as np

REACH = 0.06            # grasp this far past the fixture
SLACK = 0.010           # extra cable length when picking
PULL = 0.007            # extra pull while wrapping a post
TAUT_N = 1.5            # connector load that means the cable is taut
TENSION_N = 5.0         # back off once the connector load cell reads this
LOW_N = 1.2             # pull out while the load is below this (slack)
CARRY_Z = 0.035
HIGH_Z = 0.06            # carry height on the taut sphere
GRASP_Z = 0.0015
LAY_Z = 0.004
HOVER_Z = 0.06
CONNECTOR_X = -0.39       # the connector's x on every board of this cell
PLUG_GRIP = 0.030       # grip the plug this far behind its nose
PLUG_Z = 0.0014         # tool centre point height that holds the plug flat on the board
PRE_IN = 0.025          # put the plug down with its nose this far before the mouth
PUSH = 0.003            # per step while sliding it home


def _unit(v):
    n = float(np.linalg.norm(v))
    return v / n if n > 1e-12 else v


def _arc(P):
    return np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(P[:, :2], axis=0), axis=1))])


def _at_arc(P, s):
    a = _arc(P)
    s = min(max(s, 0.0), a[-1])
    i = int(np.clip(np.searchsorted(a, s) - 1, 0, len(P) - 2))
    f = (s - a[i]) / max(a[i + 1] - a[i], 1e-9)
    p = P[i] + f * (P[i + 1] - P[i])
    tang = _unit(P[i + 1, :2] - P[i, :2])
    return p, tang


def _turn(d, side, angle=1.0):
    """Direction d turned toward a post's side (a final post still needs a bend to wrap)."""
    a = angle if side == "left" else -angle
    c, s = math.cos(a), math.sin(a)
    return np.array([c * d[0] - s * d[1], s * d[0] + c * d[1]])


def route_points(obs):
    """Route geometry from the observation: connector exit, then per fixture its passage point
    and the directions in and out."""
    R = obs["privileged"]["cable_radius"]
    q = np.array(obs["connector"]["pos"], float)
    sock = np.array(obs["socket"]["pos"], float)
    pts = []
    fx = obs["fixtures"]
    # first pass: passage points (clip centre; post: tangent point on the route side)
    for i, f in enumerate(fx):
        c = np.array(f["pos"], float)
        if f["type"] == "clip":
            pts.append(c)
        else:
            pts.append(None)
    # posts need the neighbouring points; resolve iteratively using clip/connector neighbours
    for _ in range(3):
        for i, f in enumerate(fx):
            if f["type"] != "post":
                continue
            prev = q if i == 0 else (pts[i - 1] if pts[i - 1] is not None else np.array(fx[i - 1]["pos"], float))
            nxt = pts[i + 1] if i + 1 < len(fx) and pts[i + 1] is not None else (
                np.array(fx[i + 1]["pos"], float) if i + 1 < len(fx) else sock)
            c = np.array(f["pos"], float)
            r = f["radius"] + R
            d_in = _unit(c - prev)
            d_out = _unit(nxt - c) if nxt is not None else _turn(d_in, f["side"])
            # corner point: on the bisector away from the post
            bis = _unit(d_in - d_out) if np.linalg.norm(d_in - d_out) > 1e-6 else np.array([-d_in[1], d_in[0]])
            pts[i] = c + r * bis
    return q, pts


PIN_R = 0.0025
SEATED_Z = 0.013


def _crosses(P, a, b, zmax):
    s = 0.0
    for j in range(len(P) - 1):
        p, q = P[j, :2], P[j + 1, :2]
        r, w = q - p, b - a
        den = r[0] * w[1] - r[1] * w[0]
        ln = float(np.linalg.norm(r))
        if abs(den) > 1e-12:
            t = ((a[0] - p[0]) * w[1] - (a[1] - p[1]) * w[0]) / den
            u = ((a[0] - p[0]) * r[1] - (a[1] - p[1]) * r[0]) / den
            if 0.0 <= t <= 1.0 and 0.0 <= u <= 1.0:
                z = P[j, 2] + t * (P[j + 1, 2] - P[j, 2])
                if z < zmax:
                    return s + t * ln
        s += ln
    return None


def fixture_status(obs):
    """Which fixtures the observed cable satisfies, in route order (the cell's own check)."""
    P = np.array(obs["privileged"]["cable"], float)
    R = obs["privileged"]["cable_radius"]
    arc = _arc(P)
    ok = []
    last = -1.0
    for f in obs["fixtures"]:
        c = np.array(f["pos"], float)
        if f["type"] == "clip":
            a = f["axis"]
            n = np.array([-math.sin(a), math.cos(a)]) * (f["gap"] / 2 + PIN_R)
            sx = _crosses(P, c - n, c + n, SEATED_Z + 0.002)
        else:
            dist = np.linalg.norm(P[:, :2] - c, axis=1)
            j = int(np.argmin(dist))
            jj = min(max(j, 1), len(P) - 2)
            tang = P[jj + 1, :2] - P[jj - 1, :2]
            cross = tang[0] * (c[1] - P[j, 1]) - tang[1] * (c[0] - P[j, 0])
            side_ok = (cross > 0) if f["side"] == "left" else (cross < 0)
            sx = float(arc[j]) if (dist[j] <= f["radius"] + R + 0.008 and side_ok) else None
        good = sx is not None and sx > last
        ok.append(good)
        if good:
            last = sx
    return ok


class Router(object):
    def __init__(self):
        self.reset()

    def reset(self):
        self.state = {"k": 0, "phase": "plan", "n": 0, "s": 0.0, "yaw": 0.0, "pa": 0.0}
        self.last_t = None
        self.saved = None

    def act(self, t, obs):
        # A repeated t is answered from the state saved before that t. Only an observation of the real scene moves
        # the state on: the connector always sits at x = CONNECTOR_X on this board, and a shifted copy of the scene
        # moves it, so the reference keeps its state independent of the order in which the copies arrive.
        real = abs(float(obs["connector"]["pos"][0]) - CONNECTOR_X) < 1e-6
        if t == self.last_t:
            st = copy.deepcopy(self.saved)
        else:
            self.saved = copy.deepcopy(self.state)
            st = copy.deepcopy(self.state)
            self.last_t = t
        action = self._policy(st, obs)
        if real:
            self.state = st
        return action

    # ----------------------------------------------------------------------------------
    def _geometry(self, st, obs):
        k = st["k"]
        fx = obs["fixtures"]
        q, pts = route_points(obs)
        prev = q if k == 0 else pts[k - 1]
        T = pts[k]
        d_in = _unit(T - prev)
        if k + 1 < len(fx):
            d_out = _unit(pts[k + 1] - T)
        else:
            d_out = _unit(np.array(obs["socket"]["pos"], float) - T)
        return fx[k], prev, T, d_in, d_out

    def _held_arc(self, st, obs, P):
        """Arc length at which the cable passes the last fixed point."""
        k = st["k"]
        if k == 0:
            return 0.0
        q, pts = route_points(obs)
        a = _arc(P)
        j = int(np.argmin(np.linalg.norm(P[:, :2] - pts[k - 1], axis=1)))
        return float(a[j])

    def _policy(self, st, obs):
        g = obs["gripper"]
        gp = np.array(g["pos"], float)
        P = np.array(obs["privileged"]["cable"], float)
        fx = obs["fixtures"]
        R = obs["privileged"]["cable_radius"]
        load = obs["connector_force"]
        if st["k"] >= len(fx):
            return self._plug(st, obs)
        f, prev, T, d_in, d_out = self._geometry(st, obs)
        ph = st["phase"]

        def align(direction):
            a = math.atan2(direction[1], direction[0])
            cur = st["yaw"]
            return cur + ((a - cur + math.pi / 2) % math.pi - math.pi / 2)

        def out(pos, grip):
            return {"pos": [float(pos[0]), float(pos[1]), float(pos[2])], "yaw": st["yaw"], "grip": grip}

        if f["type"] == "clip":
            land = T + REACH * d_in                     # where the grip lands: straight on past the clip
        else:
            # straight on past the corner, a little outward so the cable clears the post
            side = 1.0 if f["side"] == "left" else -1.0
            outward = -side * np.array([-d_in[1], d_in[0]])
            land = T + REACH * d_in + 0.004 * outward

        if ph == "plan":
            s0 = self._held_arc(st, obs, P)
            need = float(np.linalg.norm(land - prev))
            st["s0"] = s0
            st["s"] = s0 + need + SLACK
            st["rad"] = need + SLACK - 0.003          # taut radius around the last fixed point
            st["phase"], st["n"] = "approach", 0
            ph = "approach"

        gpt, tang = _at_arc(P, st["s"])
        prev3 = np.array([prev[0], prev[1], R])

        def on_sphere(direction_xy, z):
            """Point at height z on the sphere of the taut radius around the last fixed point."""
            h = math.sqrt(max(st["rad"] ** 2 - (z - R) ** 2, 1e-6))
            dvec = _unit(direction_xy)
            return np.array([prev[0] + h * dvec[0], prev[1] + h * dvec[1], z])

        if ph == "approach":
            st["yaw"] = align(tang)
            if np.linalg.norm(gp[:2] - gpt[:2]) < 0.004 and abs(gp[2] - HOVER_Z) < 0.01:
                st["phase"], st["n"] = "descend", 0
            return out([gpt[0], gpt[1], HOVER_Z], 0.0)
        if ph == "descend":
            st["yaw"] = align(tang)
            st["n"] += 1
            if gp[2] < GRASP_Z + 0.002 or st["n"] > 25:
                st["phase"], st["n"] = "close", 0
            return out([gpt[0], gpt[1], GRASP_Z], 0.0)
        if ph == "close":
            st["n"] += 1
            if st["n"] >= 6:
                st["phase"], st["n"] = "lift", 0
            return out([gp[0], gp[1], GRASP_Z], 1.0)
        if ph == "lift":
            # straight up, but never further from the last fixed point than the free cable allows
            st["n"] += 1
            z = min(HIGH_Z, gp[2] + 0.02)
            v = gp[:2] - prev
            h = math.sqrt(max(st["rad"] ** 2 - (z - R) ** 2, 1e-6))
            xy = gp[:2] if np.linalg.norm(v) <= h else prev + _unit(v) * h
            st["yaw"] = align(gp[:2] - prev)
            if gp[2] > HIGH_Z - 0.005 or st["n"] > 25:
                st["phase"], st["n"] = "swing", 0
            return out([xy[0], xy[1], z], 1.0)
        if ph == "swing":
            # around the last fixed point at the taut radius, toward the landing direction
            st["n"] += 1
            a_cur = math.atan2(gp[1] - prev[1], gp[0] - prev[0])
            a_tgt = math.atan2(land[1] - prev[1], land[0] - prev[0])
            da = (a_tgt - a_cur + math.pi) % (2 * math.pi) - math.pi
            a_next = a_cur + max(-0.12, min(0.12, da))
            if load > TENSION_N:
                st["rad"] -= 0.004
            elif load < LOW_N:
                st["rad"] += 0.001
            tgt = on_sphere(np.array([math.cos(a_next), math.sin(a_next)]), HIGH_Z)
            st["yaw"] = align(tgt[:2] - prev)
            if abs(da) < 0.01 or st["n"] > 60:
                st["phase"], st["n"] = "lower", 0
            return out(tgt, 1.0)
        if ph == "lower":
            # down the sphere: the taut cable comes down straight onto the fixture
            st["n"] += 1
            if load > TENSION_N:
                st["rad"] -= 0.004
            elif load < LOW_N:
                st["rad"] += 0.002
            a = min(1.0, st["n"] / 14.0)
            z = HIGH_Z + a * (LAY_Z - HIGH_Z)
            tgt = on_sphere(land - prev, z)
            st["yaw"] = align(land - prev)
            if st["n"] >= 18:
                st["phase"], st["n"] = ("release" if f["type"] == "clip" else "sweep"), 0
            return out(tgt, 1.0)
        if ph == "sweep":
            # swing around the post corner from d_in to d_out at REACH, keeping the cable taut
            a0 = math.atan2(land[1] - T[1], land[0] - T[0])
            a1 = math.atan2(d_out[1], d_out[0])
            da = (a1 - a0 + math.pi) % (2 * math.pi) - math.pi
            st["n"] += 1
            frac = min(1.0, st["n"] / 20.0)
            ang = a0 + frac * da
            if load > TENSION_N:
                st["pr"] = st.get("pr", 0.0) - 0.004
            elif load < 0.6 * TAUT_N:
                st["pr"] = min(st.get("pr", 0.0) + 0.001, 0.03)
            rad = REACH + st.get("pr", 0.0)
            xy = T + rad * np.array([math.cos(ang), math.sin(ang)])
            st["yaw"] = align(np.array([math.cos(ang), math.sin(ang)]))
            if st["n"] >= 26:
                st["phase"], st["n"], st["pr"] = "release", 0, 0.0
            return out([xy[0], xy[1], LAY_Z + 0.002], 1.0)
        if ph == "release":
            st["n"] += 1
            if st["n"] >= 6:
                st["phase"], st["n"] = "retreat", 0
            return out([gp[0], gp[1], gp[2]], 0.0)
        if ph == "retreat":
            st["n"] += 1
            if gp[2] > HOVER_Z - 0.01 or st["n"] > 15:
                st["phase"], st["n"] = "check", 0
            return out([gp[0], gp[1], HOVER_Z], 0.0)
        if ph == "check":
            # let the cable settle, then verify every fixture so far and redo the first one missed
            st["n"] += 1
            if st["n"] < 5:
                return out([gp[0], gp[1], HOVER_Z], 0.0)
            status = fixture_status(obs)
            first_bad = next((i for i in range(st["k"] + 1) if not status[i]), None)
            tries = st.setdefault("tries", {})
            if first_bad is None:
                st["k"] += 1
            else:
                key = str(first_bad)
                tries[key] = tries.get(key, 0) + 1
                if tries[key] >= 3:
                    st["k"] += 1                      # give up on it and go on
                else:
                    st["k"] = first_bad
            st["phase"], st["n"] = "plan", 0
            return out([gp[0], gp[1], HOVER_Z], 0.0)
        return out(gp, 0.0)


def _plug_frame(obs):
    sock = np.array(obs["socket"]["pos"], float)
    sh = float(obs["socket"]["heading"])
    u = np.array([math.cos(sh), math.sin(sh)])
    return sock, sh, u, np.array([-u[1], u[0]])


def _plug_pose(obs):
    p = obs["privileged"]["plug"]
    return np.array(p["nose"], float), float(p["heading"]), float(p["tilt"])


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def _clip(v, lim):
    return max(-lim, min(lim, v))


def _plug_policy(self, st, obs):
    g = obs["gripper"]
    gp = np.array(g["pos"], float)
    gy = float(g["yaw"])
    sock, sh, u, lat_dir = _plug_frame(obs)
    nose, ph_, tilt = _plug_pose(obs)
    ph = st["phase"]
    if ph in ("plan", "check"):                    # just arrived from the route
        ph = st["phase"] = "p_up"
        st["n"] = 0
        st["ptries"] = st.get("ptries", 0)

    def out(pos, yaw, grip, done=False):
        a = {"pos": [float(pos[0]), float(pos[1]), float(pos[2])], "yaw": float(yaw), "grip": grip}
        if done:
            a["done"] = True
        return a

    def grip_point():
        return nose[:2] - PLUG_GRIP * np.array([math.cos(ph_), math.sin(ph_)])

    def tcp_for(nose_xy, plug_heading):
        """Tool centre point and yaw that put the held plug's nose at nose_xy on plug_heading."""
        gyaw = plug_heading - st["dyaw"]
        c, s_ = math.cos(gyaw), math.sin(gyaw)
        o = st["off"]
        return nose_xy - np.array([o[0] * c - o[1] * s_, o[0] * s_ + o[1] * c]), gyaw

    if obs["socket"]["mated"] and ph not in ("p_free", "p_away"):
        ph = st["phase"] = "p_free"
        st["n"] = 0
    SAFE = (0.40, 0.28)
    if ph == "p_up" and st["n"] == 0 and (abs(grip_point()[0]) > SAFE[0] or abs(grip_point()[1]) > SAFE[1]) \
            and st.get("fetches", 0) < 2:
        st["fetches"] = st.get("fetches", 0) + 1
        ph = st["phase"] = "f_up"
    if ph.startswith("f_"):                          # the plug lies by the rim: drag the cable end in first
        P = np.array(obs["privileged"]["cable"], float)
        a_ = _arc(P)
        pick = None
        for back in (0.05, 0.08, 0.11, 0.14, 0.17, 0.20):
            p_, tang = _at_arc(P, a_[-1] - back)
            if abs(p_[0]) < SAFE[0] + 0.02 and abs(p_[1]) < SAFE[1] + 0.02:
                pick = (p_, tang)
                break
        if pick is None:
            pick = _at_arc(P, a_[-1] - 0.2)
        p_, tang = pick
        yaw = math.atan2(tang[1], tang[0])
        drop = sock - 0.10 * u
        st["n"] += 1
        if ph == "f_up":
            if st["n"] > 12:
                st["phase"], st["n"] = "f_down", 0
            return out([p_[0], p_[1], 0.04], yaw, 0.0)
        if ph == "f_down":
            if st["n"] > 8:
                st["phase"], st["n"] = "f_close", 0
            return out([p_[0], p_[1], 0.0015], yaw, 0.0)
        if ph == "f_close":
            if st["n"] >= 6:
                st["phase"], st["n"] = "f_drag", 0
            return out([gp[0], gp[1], 0.0015], gy, 1.0)
        if ph == "f_drag":
            if np.linalg.norm(gp[:2] - drop) < 0.01 or st["n"] > 30 or obs["connector_force"] > 8.0:
                st["phase"], st["n"] = "f_open", 0
            return out([drop[0], drop[1], 0.03], gy, 1.0)
        if ph == "f_open":
            if st["n"] >= 5:
                st["phase"], st["n"] = "p_up", 0
            return out([gp[0], gp[1], 0.004], gy, 0.0)
    if ph == "p_up":                                 # up and over the plug's grip point
        st["n"] += 1
        gpt = grip_point()
        if gp[2] < 0.02 and st["n"] < 8:
            return out([gp[0], gp[1], 0.04], gy, 0.0)
        if np.linalg.norm(gp[:2] - gpt) < 0.003 or st["n"] > 40:
            st["phase"], st["n"] = "p_down", 0
        return out([gpt[0], gpt[1], 0.04], ph_, 0.0)
    if ph == "p_down":
        st["n"] += 1
        gpt = grip_point()
        if gp[2] < PLUG_Z + 0.0015 or st["n"] > 12:
            st["phase"], st["n"] = "p_close", 0
        return out([gpt[0], gpt[1], PLUG_Z], ph_, 0.0)
    if ph == "p_close":
        st["n"] += 1
        if st["n"] >= 6:
            c, s_ = math.cos(gy), math.sin(gy)
            rel = nose[:2] - gp[:2]
            st["off"] = [rel[0] * c + rel[1] * s_, -rel[0] * s_ + rel[1] * c]
            st["dyaw"] = _wrap(ph_ - gy)
            st["phase"], st["n"] = "p_lift", 0
        return out([gp[0], gp[1], PLUG_Z], gy, 1.0)
    if ph == "p_lift":
        st["n"] += 1
        if st["n"] >= 5:
            st["phase"], st["n"] = "p_carry", 0
        return out([gp[0], gp[1], 0.03], gy, 1.0)
    if ph == "p_carry":
        st["n"] += 1
        tgt, gyaw = tcp_for(sock - PRE_IN * u, sh)
        if (np.linalg.norm(gp[:2] - tgt) < 0.003 and abs(_wrap(gy - gyaw)) < 0.03) or st["n"] > 40:
            st["phase"], st["n"] = "p_lower", 0
        return out([tgt[0], tgt[1], 0.03], gyaw, 1.0)
    if ph == "p_lower":
        st["n"] += 1
        tgt, gyaw = tcp_for(sock - PRE_IN * u, sh)
        if st["n"] >= 8:
            st["phase"], st["n"] = "p_push", 0
        return out([tgt[0], tgt[1], PLUG_Z], gyaw, 1.0)
    # from here the grip offset is measured afresh each step (the plug may turn in the fingers)
    c, s_ = math.cos(gy), math.sin(gy)
    rel = nose[:2] - gp[:2]
    st["off"] = [rel[0] * c + rel[1] * s_, -rel[0] * s_ + rel[1] * c]
    st["dyaw"] = _wrap(ph_ - gy)
    if np.linalg.norm(rel) > 0.06:                   # the plug is no longer in the fingers: fetch it again
        st["ptries"] += 1
        st["phase"], st["n"] = ("p_up" if st["ptries"] < 4 else "p_free"), 0
        return out([gp[0], gp[1], 0.04], gy, 0.0)
    if ph == "p_push":                               # the nose along the socket's line, a few mm further each step
        st["n"] += 1
        depth = float(np.dot(nose[:2] - sock, u))
        if obs["connector_force"] < 10.0:
            st["goal"] = min(max(st.get("goal", depth), depth) + 0.004, 0.017)
        goal = st.get("goal", depth)
        tgt, gyaw = tcp_for(sock + goal * u, sh)
        if st["n"] > 35:
            st["ptries"] += 1
            st["goal"] = depth - 0.02
            st["phase"], st["n"] = ("p_back" if st["ptries"] < 4 else "p_free"), 0
        return out([tgt[0], tgt[1], PLUG_Z], gyaw, 1.0)
    if ph == "p_back":                               # draw it back out along the line and push again
        st["n"] += 1
        tgt, gyaw = tcp_for(sock + st["goal"] * u, sh)
        if st["n"] >= 6:
            st["phase"], st["n"] = "p_push", 0
        return out([tgt[0], tgt[1], PLUG_Z], gyaw, 1.0)
    if ph == "p_free":
        st["n"] += 1
        if st["n"] >= 5:
            st["phase"], st["n"] = "p_away", 0
        return out([gp[0], gp[1], gp[2]], gy, 0.0)
    if ph == "p_away":
        st["n"] += 1
        return out([gp[0], gp[1], HOVER_Z], gy, 0.0, done=gp[2] > HOVER_Z - 0.01 or st["n"] > 15)
    return out(gp, gy, 0.0)


Router._plug = _plug_policy


def main():
    router = Router()
    print("READY", flush=True)
    for line in sys.stdin:
        line = line.strip()
        if not line:
            continue
        try:
            req = json.loads(line)
        except ValueError:
            print(json.dumps({"error": "bad json"}), flush=True)
            continue
        kind = req.get("type")
        if kind == "reset":
            router.reset()
            print(json.dumps({"ok": True}), flush=True)
        elif kind == "act":
            try:
                print(json.dumps({"action": router.act(req["t"], req["obs"])}), flush=True)
            except Exception as exc:  # noqa: BLE001
                print(json.dumps({"error": str(exc)[:200]}), flush=True)
        elif kind == "close":
            print(json.dumps({"ok": True}), flush=True)
            return
        else:
            print(json.dumps({"error": "unknown request"}), flush=True)


if __name__ == "__main__":
    main()

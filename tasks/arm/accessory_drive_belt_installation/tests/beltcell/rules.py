"""The cell's rules, applied after every 10 ms control tick (cell internals). Nothing here is reported on the wire.

* Grip (the one documented rule): a closing gripper (from a narrowing command until its fingers' commanded ramp
  reaches the commanded width) holds what lies between its finger pads, at the pose of closing:
  a belt span pinched across its thickness (the link becomes welded to the hand), or the round service lever (two
  point constraints on the lever's axis, so the lever may turn between the pads). Opening the fingers lets go. The
  hold slips when the held thing pulls with more than 30 N, or twists a held belt with more than 1.0 N m, for 50 ms.
  Open fingers and the backs of closed fingers otherwise touch the belt without friction (a span can be pushed).
  The belt meets a finger pad on its flat face (the pads' small ridges do not touch the belt or a held part).
* Safety: any Panda link or hand, finger pads excepted, pressing on anything (the other arm included) with more than
  30 N for 30 ms stops both arms: the episode ends and the engine scores 0.
* Damage: the tensioner arm moving faster than its damper's rated 60 deg/s (a snap-back, or a violent opening); the
  belt touching the base plate (dropped: scrap).

Contact bits (contype, conaffinity): world/drive (1, 62); free belt (2, 13); left arm (4, 43); right arm (8, 23); a belt
link or the lever held by the left hand (16, 9), by the right hand (32, 5): a held part never touches the hand that
holds it, and keeps touching everything else.
"""

import math

import mujoco
import numpy as np

from . import drive as dr
from .physics import CLS_BELT, CLS_LINK, CLS_PAD, CLS_WORLD

__all__ = ["Rules", "GRIP", "SAFETY", "DAMAGE", "BITS"]

GRIP = {"pad_x": 0.0085, "pad_z": (0.0944, 0.1114), "slack": 0.003, "belt_angle_deg": 30.0, "lever_angle_deg": 30.0,
        "slip_force_N": 30.0, "slip_torque_Nm": 1.0, "slip_ticks": 5, "lever_pin_dz": 0.005}
SAFETY = {"link_N": 30.0, "ticks": 3}
DAMAGE = {"arm_speed_dps": 60.0, "belt_floor_z": 0.004}
# contact bits (contype, conaffinity); a pair collides when either type bit meets the other's affinity. The Menagerie
# fingertip pads are a flat block (pad_1) with four 1.5 mm ridges in front of it (6 x 4 x 6-7 mm boxes, 4 mm apart):
# the belt touches the pads' flat faces only (ridge_L / ridge_R never meet the belt or a held part): the belt's
# discrete contact spheres (r 2.2 mm, 5.2 mm apart across its width) would otherwise nest in the 4 mm gaps between the
# ridges, which a real 20 mm belt cannot do (cell 0.2.0). The ridges still touch everything else.
BITS = {"world": (1, 254), "belt": (2, 13), "L": (4, 171), "R": (8, 87), "held_L": (16, 9), "held_R": (32, 5),
        "ridge_L": (64, 137), "ridge_R": (128, 69)}
RIDGE_HALF_X_MAX = 0.005           # finger boxes narrower than this are the pads' ridges (pad_1 is 8.5 mm half-wide)


class Rules(object):
    def __init__(self, scene):
        self.sc = scene
        m = scene.model
        self.held = {s: None for s in scene.arms}          # ("belt", link index) | ("lever",)
        self.closing = {s: False for s in scene.arms}
        self.slip = {s: 0 for s in scene.arms}
        self.stopped = False
        self.stop_what = None
        self.damage = None
        self.events = []                                    # the grader's own record (never on the wire)
        self._press = {}
        self.peak = {"lever_N": {s: 0.0 for s in scene.arms}, "grip_N": {s: 0.0 for s in scene.arms}}
        self.grip_eq = {s: m.equality("grip/" + s).id for s in scene.arms}
        self.pin_eq = {s: [m.equality("pin/%s/%s" % (s, k)).id for k in ("a", "b")] for s in scene.arms}
        self.lever_geom = scene.drive.lever_geom
        self._set_bits()

    # ------------------------------------------------------------------ contact bits
    def _set_bits(self):
        sc = self.sc
        m = sc.model
        for g in range(m.ngeom):
            if int(m.geom_contype[g]) == 0 and int(m.geom_conaffinity[g]) == 0:
                continue
            cls = int(sc.geom_class[g])
            if cls == CLS_BELT:
                m.geom_contype[g], m.geom_conaffinity[g] = BITS["belt"]
            elif cls in (CLS_LINK, CLS_PAD):
                side = "L" if int(sc.geom_arm[g]) == 1 else "R"
                ridge = cls == CLS_PAD and float(m.geom_size[g][0]) < RIDGE_HALF_X_MAX
                m.geom_contype[g], m.geom_conaffinity[g] = BITS[("ridge_" if ridge else "") + side]
            else:
                m.geom_contype[g], m.geom_conaffinity[g] = BITS["world"]

    def _ev(self, e):
        self.events.append((round(float(self.sc.data.time), 3), e))

    # ------------------------------------------------------------------ grip
    def command_width(self, side, w, current):
        self.closing[side] = (w < current - 0.002) or w <= 0.001
        if self.held[side] is not None and w > current + 0.002:
            self.release(side, "opened")

    def _hand(self, side):
        d = self.sc.data
        b = self.sc.hand_body[side]
        return d.xpos[b].copy(), d.xmat[b].reshape(3, 3).copy()

    def find_grippable(self, side):
        sc = self.sc
        d = sc.data
        p_h, R_h = self._hand(side)
        w = sc.arms[side].width()
        z0, z1 = GRIP["pad_z"]
        sl = GRIP["slack"]
        # belt links: centre between the pads, thickness across the closing axis
        C = sc.belt.centres(d)
        F = sc.belt.frames(d)
        v = (C - p_h) @ R_h
        ok = (np.abs(v[:, 0]) <= GRIP["pad_x"] + sl) & (v[:, 2] >= z0 - sl) & (v[:, 2] <= z1 + sl) & (np.abs(v[:, 1]) <= 0.5 * w + 0.001)
        best = None
        ca = math.cos(math.radians(GRIP["belt_angle_deg"]))
        for i in np.nonzero(ok)[0]:
            if self._held_link(int(i)):
                continue
            if abs(float(F[i][:, 1] @ R_h[:, 1])) < ca:
                continue
            score = abs(float(v[i, 1])) + 0.2 * abs(float(v[i, 0]))
            if best is None or score < best[0]:
                best = (score, ("belt", int(i)))
        if best is not None:
            return best[1]
        # the lever: its axis passes between the pads, across the closing axis
        if self.held["L" if side == "R" else "R"] == ("lever",):
            return None
        a_tip = d.site_xpos[sc.drive.lever_site].copy()
        ax = d.xmat[sc.drive.arm_body].reshape(3, 3) @ np.array([0.0, -1.0, 0.0])        # lever axis, out of the face
        lever_len = sc.drive.T["lever_d"][1] - sc.drive.T["lever_d"][0]
        tcp = p_h + R_h[:, 2] * 0.1034
        s = float(np.clip((tcp - a_tip) @ ax, -lever_len, 0.0))
        q = a_tip + ax * s
        vq = R_h.T @ (q - p_h)
        r_lev = float(sc.drive.T["lever_radius"])
        if abs(vq[0]) <= GRIP["pad_x"] + sl and z0 - sl <= vq[2] <= z1 + sl and abs(vq[1]) <= 0.5 * w - r_lev + 0.002 \
                and abs(float(ax @ R_h[:, 1])) <= math.sin(math.radians(GRIP["lever_angle_deg"])) and w >= 2 * r_lev - 0.002:
            return ("lever", q, ax)
        return None

    def _held_link(self, i):
        return any(h is not None and h[0] == "belt" and h[1] == i for h in self.held.values())

    def _geoms_of_body(self, b):
        m = self.sc.model
        return [g for g in range(m.ngeom) if int(m.geom_bodyid[g]) == int(b) and (int(m.geom_contype[g]) or int(m.geom_conaffinity[g]))]

    def attach(self, side, hit):
        sc = self.sc
        m, d = sc.model, sc.data
        hb = sc.hand_body[side]
        if hit[0] == "belt":
            i = hit[1]
            b = int(sc.belt.body[i])
            e = self.grip_eq[side]
            R1 = d.xmat[hb].reshape(3, 3)
            p = R1.T @ (d.xpos[b] - d.xpos[hb])
            q = np.zeros(4)
            mujoco.mju_mulQuat(q, np.array([d.xquat[hb][0], -d.xquat[hb][1], -d.xquat[hb][2], -d.xquat[hb][3]]), d.xquat[b])
            m.eq_obj2id[e] = b
            m.eq_data[e, 0:3] = 0.0
            m.eq_data[e, 3:6] = p
            m.eq_data[e, 6:10] = q / np.linalg.norm(q)
            m.eq_data[e, 10] = 1.0
            d.eq_active[e] = 1
            for g in self._geoms_of_body(b):
                if int(sc.geom_class[g]) == CLS_BELT:
                    m.geom_contype[g], m.geom_conaffinity[g] = BITS["held_" + side]
            sc.arms[side].hold_width(max(0.0, dr.BELT["thick"] - 0.0005))
            self.held[side] = ("belt", i)
            self._ev("grip:%s:belt%03d" % (side, i))
        else:
            _, qpt, ax = hit
            arm_b = sc.drive.arm_body
            R1 = d.xmat[hb].reshape(3, 3)
            R2 = d.xmat[arm_b].reshape(3, 3)
            for k, e in enumerate(self.pin_eq[side]):
                pw = qpt + ax * ((-1) ** k) * GRIP["lever_pin_dz"]
                m.eq_obj2id[e] = arm_b
                m.eq_data[e, 0:3] = R1.T @ (pw - d.xpos[hb])
                m.eq_data[e, 3:6] = R2.T @ (pw - d.xpos[arm_b])
                d.eq_active[e] = 1
            m.geom_contype[self.lever_geom], m.geom_conaffinity[self.lever_geom] = BITS["held_" + side]
            sc.arms[side].hold_width(2 * float(sc.drive.T["lever_radius"]) - 0.0005)
            self.held[side] = ("lever",)
            self._ev("grip:%s:lever" % side)
        self.closing[side] = False
        self.slip[side] = 0

    def release(self, side, why):
        sc = self.sc
        m, d = sc.model, sc.data
        h = self.held[side]
        if h is None:
            return
        if h[0] == "belt":
            d.eq_active[self.grip_eq[side]] = 0
            for g in self._geoms_of_body(int(sc.belt.body[h[1]])):
                if int(sc.geom_class[g]) == CLS_BELT:
                    m.geom_contype[g], m.geom_conaffinity[g] = BITS["belt"]
        else:
            for e in self.pin_eq[side]:
                d.eq_active[e] = 0
            m.geom_contype[self.lever_geom], m.geom_conaffinity[self.lever_geom] = BITS["world"]
        sc.arms[side].hold_width(None)
        self.held[side] = None
        self._ev("%s:%s" % ("slipped" if why == "slip" else "released", side))

    def _eq_rows(self, ids):
        d = self.sc.data
        return [i for i in range(d.nefc) if d.efc_type[i] == mujoco.mjtConstraint.mjCNSTR_EQUALITY and int(d.efc_id[i]) in ids]

    def grip_load(self, side):
        """(force N, torque N m) the hold carries (weld: force rows / torque rows; lever pins: their resultant)."""
        h = self.held[side]
        d = self.sc.data
        if h is None:
            return 0.0, 0.0
        if h[0] == "belt":
            rows = self._eq_rows([self.grip_eq[side]])
            f = np.asarray(d.efc_force[rows], float)
            return (float(np.linalg.norm(f[:3])) if len(f) >= 3 else 0.0), (float(np.linalg.norm(f[3:6])) if len(f) >= 6 else 0.0)
        tot = np.zeros(3)
        for e in self.pin_eq[side]:
            rows = self._eq_rows([e])
            if len(rows) >= 3:
                tot += np.asarray(d.efc_force[rows[:3]], float)
        return float(np.linalg.norm(tot)), 0.0

    def _grip_tick(self, side):
        h = self.held[side]
        if h is None:
            if self.closing[side]:
                hit = self.find_grippable(side)
                if hit is not None:
                    self.attach(side, hit)
                else:
                    a = self.sc.arms[side]
                    if a.width_cmd <= a.width_goal + 1e-9:
                        # the fingers have finished closing (their commanded ramp has reached the commanded width): a
                        # gripper that is no longer closing takes nothing that comes between its pads later
                        self.closing[side] = False
            return
        f, t = self.grip_load(side)
        key = "lever_N" if h[0] == "lever" else "grip_N"
        self.peak[key][side] = max(self.peak[key][side], f)
        lim_t = GRIP["slip_torque_Nm"] if h[0] == "belt" else float("inf")
        if f > GRIP["slip_force_N"] or t > lim_t:
            self.slip[side] += 1
            if self.slip[side] >= GRIP["slip_ticks"]:
                self.release(side, "slip")
        else:
            self.slip[side] = 0

    # ------------------------------------------------------------------ safety and damage
    def _safety_tick(self):
        sc = self.sc
        m, d = sc.model, sc.data
        gc, ga = sc.geom_class, sc.geom_arm
        now = {}
        f6 = np.zeros(6)
        for i in range(d.ncon):
            c = d.contact[i]
            a, b = int(c.geom1), int(c.geom2)
            ca, cb = int(gc[a]), int(gc[b])
            arm_a, arm_b = int(ga[a]), int(ga[b])
            link = None
            if ca == CLS_LINK and (cb in (CLS_WORLD, CLS_BELT) or (arm_b and arm_b != arm_a)):
                link, other = a, b
            elif cb == CLS_LINK and (ca in (CLS_WORLD, CLS_BELT) or (arm_a and arm_a != arm_b)):
                link, other = b, a
            elif arm_a and arm_b and arm_a != arm_b:              # pad against the other arm
                link, other = a, b
            if link is None:
                continue
            mujoco.mj_contactForce(m, d, i, f6)
            key = (int(m.geom_bodyid[link]), int(m.geom_bodyid[other]))
            now[key] = max(now.get(key, 0.0), abs(float(f6[0])))
        press = {}
        for key, fn in now.items():
            if fn > SAFETY["link_N"]:
                cnt, peak = self._press.get(key, (0, 0.0))
                press[key] = (cnt + 1, max(peak, fn))
        self._press = press
        for key, (cnt, peak) in press.items():
            if cnt >= SAFETY["ticks"]:
                self.stopped = True
                self.stop_what = "link_contact:%s:%s:%.0fN" % (m.body(key[0]).name, m.body(key[1]).name, peak)
                self._ev("stop:" + self.stop_what)
                return

    def _damage_tick(self):
        sc = self.sc
        d = sc.data
        if self.damage is None:
            w = abs(math.degrees(sc.drive.omega(d)))
            if w > DAMAGE["arm_speed_dps"]:
                self.damage = "tensioner_overspeed:%.0fdps" % w
                self._ev("damage:" + self.damage)
            elif float(np.min(sc.belt.points(d)[:, 2])) < DAMAGE["belt_floor_z"]:
                self.damage = "belt_dropped"
                self._ev("damage:" + self.damage)

    def after_physics(self):
        for side in self.held:
            self._grip_tick(side)
        self._safety_tick()
        self._damage_tick()

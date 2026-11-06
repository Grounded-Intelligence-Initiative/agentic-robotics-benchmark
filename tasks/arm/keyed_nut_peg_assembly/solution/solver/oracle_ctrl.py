"""Privileged reference controller: drives the task from the ground-truth scene state.

Input per step: the public observation plus `gt` = {"nut_pos", "nut_quat", "pegs",
"target_peg", "groove"} (world frame, the nut origin is the hole axis on its bottom face, handle along the nut's
local +x). Output: the public 8-number action. Stateful (one instance per episode).

The gripper points down throughout. Its yaw (world angle of its closing axis) is tracked unwrapped;
for a top-down Panda grasp, joint 7 is about pi/4 - (yaw - pi/2 - q1) with q1 the base azimuth of the
gripper, so grasp side and turning direction are chosen to keep joint 7 inside its limits.
"""

import math

import numpy as np

TABLE_Z = 0.80
ROBOT_X = -0.56
COLLAR_TOP = TABLE_Z + 0.010
PEG_TOP = COLLAR_TOP + 0.065
HANDLE_OFF = 0.068            # handle grasp point from the hole axis along the nut's +x
SECTOR = math.pi / 12          # groove sector (15 deg)
GROOVE_TOP = 0.0395           # nut bottom above the collar when resting on the pin
STALLS = (0.0245, 0.0095, 0.0)   # nut bottom above the collar at the two turns and when seated
GRASP_Z = TABLE_Z + 0.036     # gripper site height while grasping the handle
Q7_LIMIT = 2.7                # planning bound on |joint 7| (hard limit 2.897)
YAW_STEP = 0.12               # rad per step


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def quat_mat(q):
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def down_rot(yaw):
    """Gripper pointing down with its closing axis (local x) at world angle `yaw`."""
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]) @ np.diag([1.0, -1.0, -1.0])


def rotvec(Rerr):
    c = max(-1.0, min(1.0, (np.trace(Rerr) - 1) / 2))
    th = math.acos(c)
    if th < 1e-9:
        return np.zeros(3)
    v = np.array([Rerr[2, 1] - Rerr[1, 2], Rerr[0, 2] - Rerr[2, 0], Rerr[1, 0] - Rerr[0, 1]])
    return v * th / (2 * math.sin(th)) if th < math.pi - 1e-6 else v


def azimuth(x, y):
    return math.atan2(y, x - ROBOT_X)


def q7_of(yaw_unwrapped, q1):
    return math.pi / 4 - (yaw_unwrapped - math.pi / 2 - q1)


class Oracle(object):

    def __init__(self):
        self.phase = "plan"
        self.k = 0
        self.yaw = None          # unwrapped gripper yaw, tracked
        self.integ = np.zeros(3)
        self.h_best, self.k_best = 1.0, 0

    def _track(self, obs):
        R = quat_mat(obs["ee_quat"])
        cur = math.atan2(R[1, 0], R[0, 0])
        if self.yaw is None:
            self.yaw = cur
        else:
            self.yaw += wrap(cur - wrap(self.yaw))

    def _move(self, obs, pos, yaw_goal, grip, gain=1.0, free_tilt=False, Rn=None):
        ee = np.asarray(obs["ee_pos"], float)
        R = quat_mat(obs["ee_quat"])
        dp = (np.asarray(pos, float) - ee) * gain
        step = max(-YAW_STEP, min(YAW_STEP, yaw_goal - self.yaw))
        if free_tilt:
            # on the shaft: turn about the vertical and rotate the gripper so that the nut stands upright
            zn = Rn[:, 2]
            dr = 0.7 * np.cross(zn, np.array([0.0, 0.0, 1.0])) + np.array([0.0, 0.0, step])
        else:
            dr = rotvec(down_rot(wrap(self.yaw + step)) @ R.T)
        return list(dp) + list(dr) + [grip, 0]

    def _plan(self, npos, nyaw, peg, groove):
        """Nut yaws: entering (pin centred in the entrance channel), after turn 1, after turn 2; then pick the
        grasp side and unwrapping that keep joint 7 inside its limits over the whole sequence."""
        px, py, pyaw = peg
        e = int(groove["entrance"])
        k1, k2 = [int(k) for k in groove["turns"]]
        psi0 = pyaw - (e + 1) * SECTOR                  # pin (peg +x) at the channel centre, nut frame angle 15(e+1)
        nyaws = [psi0, psi0 - k1 * SECTOR, psi0 - (k1 + k2) * SECTOR]
        q1g = azimuth(npos[0], npos[1])
        q1p = azimuth(px, py)
        best = None
        for side in (math.pi / 2, -math.pi / 2):
            g0 = wrap(nyaw + side)
            for kg in (-1, 0, 1):
                yg = g0 + 2 * math.pi * kg
                d0 = wrap(nyaws[0] - nyaw)
                for kd in (-1, 0, 1):
                    y0 = yg + d0 + 2 * math.pi * kd
                    seq = [y0 + (n - nyaws[0]) for n in nyaws]
                    worst = max([abs(q7_of(yg, q1g)), abs(q7_of(yg, q1p))] + [abs(q7_of(y, q1p)) for y in seq])
                    cost = abs(y0 - yg) + 0.5 * abs(yg - self.yaw) + (100.0 + worst if worst > Q7_LIMIT else 0.0)
                    if best is None or cost < best[0]:
                        best = (cost, yg, seq)
        _c, self.grasp_yaw, self.yaw_seq = best
        self.nyaw_seq = nyaws
        self.yaw_off = self.grasp_yaw - nyaw            # gripper yaw minus nut yaw while held (unwrapped)

    def act(self, obs, gt):
        self._track(obs)
        ee = np.asarray(obs["ee_pos"], float)
        npos = np.asarray(gt["nut_pos"], float)
        Rn = quat_mat(gt["nut_quat"])
        nyaw = math.atan2(Rn[1, 0], Rn[0, 0])
        peg = gt["pegs"][gt["target_peg"]]
        px, py = peg[0], peg[1]
        self.k += 1
        if self.phase == "plan":
            self._plan(npos, nyaw, peg, gt["groove"])
            h = npos + Rn @ np.array([HANDLE_OFF, 0.0, 0.0])
            self.handle = np.array([h[0], h[1], GRASP_Z])
            self.phase, self.k = "above", 0
        if self.phase == "above":
            tgt = self.handle + [0, 0, 0.10]
            a = self._move(obs, tgt, self.grasp_yaw, -1.0)
            if np.linalg.norm(tgt - ee) < 0.004 and abs(self.yaw - self.grasp_yaw) < 0.02:
                self.phase, self.k = "down", 0
            return a
        if self.phase == "down":
            a = self._move(obs, self.handle, self.grasp_yaw, -1.0)
            if np.linalg.norm(self.handle - ee) < 0.003 or self.k > 40:
                self.phase, self.k = "close", 0
            return a
        if self.phase == "close":
            a = self._move(obs, self.handle, self.grasp_yaw, 1.0)
            if self.k > 8:
                self.phase, self.k = "lift", 0
            return a
        hold_off = ee - npos                     # gripper site minus nut origin, world frame
        h = npos[2] - COLLAR_TOP                 # nut bottom above the collar
        exy = np.array([px - npos[0], py - npos[1], 0.0])
        if self.phase == "lift":
            tgt = np.array([ee[0], ee[1], PEG_TOP + 0.06 + hold_off[2]])
            a = self._move(obs, tgt, self.grasp_yaw, 1.0)
            if ee[2] > tgt[2] - 0.01:
                self.phase, self.k, self.stage = "carry", 0, 0
            return a
        # nut yaw goal of the current stage; the gripper yaw follows it (unwrapped)
        ygoal = self.yaw_seq[self.stage]
        nerr = wrap(self.nyaw_seq[self.stage] - nyaw)
        goal = ygoal if abs(ygoal - self.yaw) > 0.3 else self.yaw + nerr
        if self.phase == "carry":
            if np.linalg.norm(exy) < 0.02:
                self.integ = np.clip(self.integ + 0.3 * exy, -0.02, 0.02)
            tgt = np.array([px, py, PEG_TOP + 0.03]) + hold_off + self.integ
            a = self._move(obs, tgt, goal, 1.0)
            if np.linalg.norm(exy) < 0.0015 and abs(nerr) < math.radians(2.0) and self.k > 5:
                self.phase, self.k = "descend", 0
            return a
        if self.phase == "descend":
            stop = STALLS[self.stage]
            if h > GROOVE_TOP + 0.002:
                # still above the pin: centre the hole on the shaft
                self.integ = np.clip(self.integ + 0.3 * exy, -0.02, 0.02)
                tgt = np.array([px, py, COLLAR_TOP + stop - 0.004]) + hold_off + self.integ
                a = self._move(obs, ee + np.clip(tgt - ee, -0.004, 0.004), goal, 1.0)
            else:
                # on the shaft: a small step down, a gentle pull towards the shaft axis, and a yaw dither
                # when the descent has stalled short of the next stop
                if self.k == 1 or h < self.h_best - 0.0005:
                    self.h_best, self.k_best = h, self.k
                dither = 0.0
                if self.k - self.k_best > 8:
                    dither = math.radians(4.0) * math.sin(0.9 * (self.k - self.k_best))
                a = self._move(obs, ee + 0.4 * exy + np.array([0.0, 0.0, -0.003]), goal + dither, 1.0,
                               free_tilt=True, Rn=Rn)
            if h - stop < 0.0015:
                if self.stage == 2:
                    self.phase, self.k = "release", 0
                else:
                    self.stage += 1
                    self.phase, self.k = "turn", 0
            elif self.k > 200:
                self.phase, self.k = "release", 0
            return a
        if self.phase == "turn":
            # rotate the held nut about the peg axis by the stage's yaw change, pressing lightly down
            d = max(-0.06, min(0.06, nerr))
            c, sn = math.cos(d), math.sin(d)
            rel = ee[:2] - np.array([px, py])
            arc = np.array([px, py]) + np.array([c * rel[0] - sn * rel[1], sn * rel[0] + c * rel[1]])
            tgt = np.array([arc[0], arc[1], ee[2] - 0.001])
            a = self._move(obs, tgt, self.yaw + d, 1.0, free_tilt=True, Rn=Rn)
            if abs(nerr) < math.radians(1.5) and self.k > 3:
                self.phase, self.k = "descend", 0
            elif self.k > 150:
                self.phase, self.k = "release", 0
            return a
        if self.phase == "release":
            a = self._move(obs, ee, self.yaw, -1.0, gain=0.0)
            if self.k > 6:
                self.phase, self.k = "retreat", 0
            return a
        return [0.0, 0.0, 0.01, 0.0, 0.0, 0.0, -1.0, 0]

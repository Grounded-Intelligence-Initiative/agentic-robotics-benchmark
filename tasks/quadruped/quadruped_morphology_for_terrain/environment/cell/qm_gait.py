"""QM-24 standard gait: the one controller every QM-24 robot runs (pure Python + numpy, no simulator).

The site's robots all run this controller; the grader and the practice cell run exactly this file on your design, and
you cannot change it. It reads the design (robot.xml, through qm_design) and, every control period (50 Hz):
the joint angles, the IMU's gravity direction and angular rate (body frame), and the terrain scanner, which reports
the ground height at any point ahead of or around the robot. The scanner sees the ground only: it does not see
anything overhead (pipes, cable trays).

What it does
- Static crawl: one leg swings at a time, in the order RL, FL, RR, FR; each leg is in stance DUTY (90 %) of the
  cycle, so between two swings all four feet are down. During that four-foot dwell the body is shifted away from
  the leg that swings next (SHIFT_X of the mean mount x fore-aft, SHIFT_Y of the foot half-spacing sideways), so the
  centre of mass sits over the three stance feet. A leg without an abduction actuator cannot move its foot
  sideways, so the sideways part of the shift needs abduction actuators on the legs.
- Gait sizes from the design. R = the shortest leg reach (thigh + shank) of the four legs, h = stance_height:
      step length   s  = min(STEP_H h, STEP_R sqrt((0.92 R)^2 - h^2))
      swing height  sh = min(SWING_H h, h - max|L1 - L2| - 0.03)     (clearance above the highest ground on the path)
      period        T  = PERIOD_0 + PERIOD_H h                         (s)
  so the nominal speed is s / (DUTY T); gait_sizes() prints them.
- Foot placement from the scanner: the body rides h above the mean ground under its four feet; each stance foot is
  held on the ground under it; a swinging foot lifts from where it was, clears the highest ground between lift-off
  and landing by sh, and lands on a spot kept a foot radius plus EDGE_MARGIN clear of any step edge.
- Attitude feedback: an integral (K_ATT_I) on the IMU's pitch and roll lengthens or shortens legs (within +-0.25 h)
  to hold the body level. Heading feedback: the integrated yaw rate shortens the steps on one side.
- Inverse kinematics, closed form: knee from the law of cosines on the commanded hip-to-foot distance (clipped to
  [|L1-L2| + 0.01, 0.98 (L1+L2)]), knee direction from knee_front / knee_rear; abduction (when the leg has an
  abduction actuator) places the foot at its lateral target; without one the foot stays in the hip-flange plane.
- Start: 0.8 s standing at the stance, then the gait amplitude ramps up over 1.0 s.
The joint targets go to each actuator's servo (kp, kd from the catalog; the target is interpolated over the control
period), limited by the actuator's torque and its torque-speed line (no torque at the rated speed).
"""

import math

import numpy as np

from qm_design import FRONT, LEGS, SIDE, joint_names, leg_geometry

DT = 0.02  # control period (s)
DUTY = 0.90                     # each leg is in stance 90 % of the cycle; one leg swings at a time
SWING_ORDER = ("RL", "FL", "RR", "FR")  # swing windows start at 0, 1/4, 1/2, 3/4 of the cycle
SHIFT_X, SHIFT_Y = 0.15, 0.20   # body shift away from the next swinging leg: fractions of the mount x / foot half-spacing
K_ATT_I = 1.0                   # leg-length integral that levels the body (1/s)
K_YAW, K_YAWD, YAW_CLIP = 0.3, 0.0, 0.4
STAND_S, RAMP_S = 0.8, 1.0
STEP_H, STEP_R = 0.55, 1.8      # step length: min(STEP_H h, STEP_R sqrt((0.92 R)^2 - h^2))
SWING_H = 0.45                  # swing clearance: min(SWING_H h, h - max|L1 - L2| - 0.03)
PERIOD_0, PERIOD_H = 0.8, 4.0   # cycle period: PERIOD_0 + PERIOD_H h (s)
EDGE_MARGIN = 0.02              # landing spots keep a foot radius plus this from a height change


def gait_sizes(design, catalog):
    geo = leg_geometry(design, catalog)
    R = min(g["L1"] + g["L2"] for g in geo.values())
    h = float(design["params"]["stance_height"])
    s = min(STEP_H * h, STEP_R * math.sqrt(max((0.92 * R) ** 2 - h * h, 0.0)))
    dl = max(abs(g["L1"] - g["L2"]) for g in geo.values())
    sh = max(min(SWING_H * h, h - dl - 0.03), 0.0)
    T = PERIOD_0 + PERIOD_H * h
    return {"reach": R, "stance_height": h, "step": s, "swing": sh, "period": T, "speed": s / (DUTY * T)}


def planar_ik(x, z, L1, L2, kdir):
    d = math.hypot(x, z)
    d = min(max(d, abs(L1 - L2) + 0.01), 0.98 * (L1 + L2))
    c2 = (d * d - L1 * L1 - L2 * L2) / (2 * L1 * L2)
    q2 = kdir * math.acos(min(max(c2, -1.0), 1.0))
    phi = math.atan2(-x, -z)
    q1 = phi - math.atan2(L2 * math.sin(q2), L1 + L2 * math.cos(q2))
    return q1, q2


def planar_fk(q1, q2, L1, L2):
    return (-L1 * math.sin(q1) - L2 * math.sin(q1 + q2), -L1 * math.cos(q1) - L2 * math.cos(q1 + q2))


def smoothstep(v):
    v = min(max(v, 0.0), 1.0)
    return v * v * (3 - 2 * v)


class StandardGait(object):
    def __init__(self, design, catalog):
        self.design = design
        self.geo = leg_geometry(design, catalog)
        self.names = joint_names(design)
        self.size = gait_sizes(design, catalog)
        self.kdir = {leg: design["params"]["knee_front" if FRONT[leg] > 0 else "knee_rear"] for leg in LEGS}
        self.half_w = float(np.mean([abs(self.geo[leg]["mount"][1] + self.geo[leg]["w"]) for leg in LEGS]))
        self.mount_x = float(np.mean([abs(self.geo[leg]["mount"][0]) for leg in LEGS]))
        self.offset = {leg: k / 4.0 for k, leg in enumerate(SWING_ORDER)}
        self.reset()

    def reset(self):
        self.ext_i = np.zeros(4)
        self.yaw = 0.0
        self.prev_sw = np.zeros(4, bool)
        self.g_prev = np.zeros(4)
        self.land = np.zeros(4)

    def leg_targets(self, leg, x, y, z):
        """Foot target (body frame, relative to the leg's mount) -> joint angles for this leg."""
        g = self.geo[leg]
        if g["has_abd"]:
            zp = -math.sqrt(max(y * y + z * z - g["w"] ** 2, 1e-6))
            a = math.atan2(z, y) - math.atan2(zp, g["w"])
            a = (a + math.pi) % (2 * math.pi) - math.pi
            q1, q2 = planar_ik(x, zp, g["L1"], g["L2"], self.kdir[leg])
            return [a, q1, q2]
        q1, q2 = planar_ik(x, z, g["L1"], g["L2"], self.kdir[leg])
        return [q1, q2]

    def shift(self, cyc):
        """Foot offset (x, y) that moves the body away from the leg about to swing (or swinging), so the centre of
        mass sits inside the three stance feet; the move happens in the four-foot dwell before each lift-off."""
        sw = 1.0 - DUTY
        k = int(cyc * 4) % 4
        local = cyc * 4 - math.floor(cyc * 4)          # position inside this quarter: swing, then dwell
        cur = SWING_ORDER[k]
        nxt = SWING_ORDER[(k + 1) % 4]
        a = np.array([FRONT[cur] * SHIFT_X * self.mount_x, SIDE[cur] * SHIFT_Y * self.half_w])
        b = np.array([FRONT[nxt] * SHIFT_X * self.mount_x, SIDE[nxt] * SHIFT_Y * self.half_w])
        u = smoothstep((local - 4 * sw) / max(1.0 - 4 * sw, 1e-6))
        return a + (b - a) * u

    def landing_x(self, height, x0, fy, r):
        """A landing x near x0 that keeps the foot clear of step edges."""
        m = r + EDGE_MARGIN
        for dx in (0.0, -0.01, 0.01, -0.02, 0.02, -0.03, 0.03, -0.04, 0.04, -0.05):
            hs = [height(x0 + dx + e, fy) for e in (-m, -m / 2, 0.0, m / 2, m)]
            if max(hs) - min(hs) < 0.005:
                return x0 + dx
        return x0

    def foot_position(self, leg, qleg):
        """Measured joint angles of one leg -> foot centre in the body frame, relative to the leg's mount."""
        g = self.geo[leg]
        a, q1, q2 = (qleg[0], qleg[1], qleg[2]) if g["has_abd"] else (0.0, qleg[0], qleg[1])
        x, zp = planar_fk(q1, q2, g["L1"], g["L2"])
        return np.array([x, g["w"] * math.cos(a) - zp * math.sin(a), g["w"] * math.sin(a) + zp * math.cos(a)])

    def act(self, t_index, obs):
        """obs: {"q": [n] joint angles (joint_names order), "gravity": [3], "gyro": [3] (body frame), "height": a
        function (x, y) -> terrain height (m) relative to the body origin at a point (x, y) of the body's
        heading-aligned level frame (the terrain scanner)}. Returns joint targets in joint_names order."""
        t = t_index * DT
        sz = self.size
        h, s, sh, T = sz["stance_height"], sz["step"], sz["swing"], sz["period"]
        sw = 1.0 - DUTY
        g = np.asarray(obs["gravity"], float)
        w = np.asarray(obs["gyro"], float)
        qm = np.asarray(obs["q"], float)
        height = obs["height"]
        pitch_up = math.asin(max(-1.0, min(1.0, -g[0])))
        roll_left = math.asin(max(-1.0, min(1.0, -g[1])))
        self.yaw += w[2] * DT
        amp = 0.0 if t < STAND_S else min(1.0, (t - STAND_S) / RAMP_S)
        cyc = max(t - STAND_S, 0.0) / T
        shx, shy = (self.shift(cyc) * amp) if t >= STAND_S else (0.0, 0.0)
        plan, k = [], 0
        for i, leg in enumerate(LEGS):
            geo = self.geo[leg]
            nj = 3 if geo["has_abd"] else 2
            foot = geo["mount"] + self.foot_position(leg, qm[k:k + nj])   # measured, body frame
            k += nj
            mount_l = geo["mount"]
            fy = mount_l[1] + geo["w"] + shy
            ph = ((cyc - self.offset[leg]) % 1.0) if t >= STAND_S else 1.0
            sl = s * amp * float(np.clip(1.0 + SIDE[leg] * (K_YAW * self.yaw + K_YAWD * w[2]), 1.0 - YAW_CLIP, 1.0 + YAW_CLIP))
            swinging = ph < sw
            if swinging:
                u = ph / sw
                if not self.prev_sw[i]:   # lift-off: the ground left behind, and a landing spot clear of edges
                    self.g_prev[i] = height(foot[0], foot[1])
                    self.land[i] = self.landing_x(height, mount_l[0] + shx + sl / 2, fy, geo["r"]) - mount_l[0] - shx
                v = smoothstep((u - 0.15) / 0.7)
                x = -sl / 2 + (self.land[i] + sl / 2) * v
                g_land = height(mount_l[0] + shx + self.land[i], fy)
                ground = self.g_prev[i] + (g_land - self.g_prev[i]) * smoothstep(u)
                xs = mount_l[0] + shx + np.linspace(x - geo["r"], self.land[i] + geo["r"] + 0.02, 6)
                path = max(height(xx, fy) for xx in xs)
                plan.append((leg, geo, mount_l, fy, x, ground, (u, self.g_prev[i], g_land, path)))
            else:
                if t >= STAND_S:
                    p = (ph - sw) / DUTY
                    x = self.land[i] - (self.land[i] + sl / 2) * p
                else:
                    x = 0.0
                ground = height(foot[0], foot[1])
                plan.append((leg, geo, mount_l, fy, x, ground, None))
            self.prev_sw[i] = swinging
        gbar = float(np.mean([pl[5] for pl in plan]))
        out = []
        for i, (leg, geo, mount_l, fy, x, ground, swing) in enumerate(plan):
            mx, my = abs(geo["mount"][0]), abs(geo["mount"][1])
            err = -(FRONT[leg] * pitch_up * mx + SIDE[leg] * roll_left * my)
            self.ext_i[i] = float(np.clip(self.ext_i[i] + K_ATT_I * err * DT, -0.25 * h, 0.25 * h))
            off = -gbar - h - self.ext_i[i]   # foot-centre height relative to the body = ground + off
            if swing is not None:
                u, g0, g1, path = swing
                apex = max(g0, path) + off + amp * sh
                e = math.sin(math.pi * u) ** 0.6
                base = (g0 if u <= 0.5 else g1) + off
                z = base + (apex - base) * e
            else:
                z = ground + off
            vh = np.array([x + shx + mount_l[0], fy, z])
            vb = vh - geo["mount"]
            out += self.leg_targets(leg, vb[0], vb[1], vb[2])
        return np.array(out)

"""QM-24 walkway simulator (hidden: the grader's and the practice cell's; never in the agent's environment).

build_mjcf(design, catalog, terrain) -> MJCF text of the grader's own model: the robot rebuilt from the catalog (the
agent's robot.xml is never handed to MuJoCo), the walkway heightfield, walls and overhead pipes.
make_terrain(rng, knobs) -> one instance of the terrain family. run_episode(...) -> outcome dict.

Collision classes (contype/conaffinity): ground+walls ct=1; pipes ct=2; feet ca=1; body module and pod ca=1|2;
leg links and actuators ca=2 (they pass through the ground but hit pipes). No robot self-collision.
Any contact of the body module, the pod or a leg part (with anything) is a fall. So is a tilt over FALL_TILT_DEG or
body-underside clearance under FALL_CLEARANCE. A fall ends the run; progress is the best forward position of the
body reached at least FALL_LAG_S before the fall. NaN / runaway speed / flight / solver warnings are a fault: 0.
"""

import math

import numpy as np

import mujoco

from qm_design import FRONT, LEGS, SIDE, joint_names, leg_geometry
import qm_gait

TIMESTEP = 0.002
SUBSTEPS = 10  # control at 50 Hz
GROUND_FRICTION = 0.5
SOLREF = 0.015
FALL_TILT_DEG = 50.0
FALL_CLEARANCE = 0.025
FALL_LAG_S = 0.5
HF_X0, HF_X1, HF_RES = -1.0, 4.5, 0.01
CORRIDOR = 1.0  # walkway width between the walls (m)

# The difficulty knobs of the terrain family (the grader's grader_config.json may override any of them).
DEFAULT_KNOBS = {
    "track_m": 3.0,
    "time_limit_s": 60.0,
    "n_up": [2, 3],            # steps up in the first flight
    "rise_m": [0.04, 0.12],    # step rise
    "tread_m": [0.25, 0.38],   # step tread
    "rubble_n": [6, 14],       # rubble blocks on the landing and the run-out
    "rubble_h": [0.01, 0.035],
    "rubble_size": [0.04, 0.12],
    "ramp_deg": [6.0, 14.0],   # ramp down after the landing
    "ramp_len": [0.30, 0.50],
    "n_up2": [1, 2],           # a second, short flight
    "pipe_n": [0, 2],          # overhead pipes across the walkway
    "pipe_clear_m": [0.30, 0.40],  # pipe underside above the highest ground under it
    "pipe_r": 0.03,
    "obs_noise": [0.004, 0.015],  # IMU noise: gravity direction, rate (rad/s), 1 sigma
}


def _u(rng, lo_hi):
    lo, hi = lo_hi
    return float(rng.uniform(lo, hi))


def _i(rng, lo_hi):
    lo, hi = lo_hi
    return int(rng.integers(lo, hi + 1))


def make_terrain(rng, knobs=None):
    k = dict(DEFAULT_KNOBS)
    k.update(knobs or {})
    nx = int(round((HF_X1 - HF_X0) / HF_RES)) + 1
    ny = int(round(CORRIDOR / HF_RES)) + 1
    xs = HF_X0 + HF_RES * np.arange(nx)
    ys = -CORRIDOR / 2 + HF_RES * np.arange(ny)
    prof = np.zeros(nx)
    x = 0.45 + _u(rng, (0.0, 0.15))
    level = 0.0
    feats = []
    for _ in range(_i(rng, k["n_up"])):
        level += _u(rng, k["rise_m"])
        prof[xs >= x] = level
        feats.append(("step", x, level))
        x += _u(rng, k["tread_m"])
    land0 = x - 0.05
    x += 0.30 + _u(rng, (0.0, 0.1))  # landing
    ang = math.radians(_u(rng, k["ramp_deg"]))
    rl = _u(rng, k["ramp_len"])
    m = (xs >= x) & (xs < x + rl)
    prof[m] = level - (xs[m] - x) * math.tan(ang)
    level -= rl * math.tan(ang)
    prof[xs >= x + rl] = level
    feats.append(("ramp", x, -math.degrees(ang)))
    x += rl + 0.15 + _u(rng, (0.0, 0.1))
    for _ in range(_i(rng, k["n_up2"])):
        level += _u(rng, k["rise_m"])
        prof[xs >= x] = level
        feats.append(("step", x, level))
        x += _u(rng, k["tread_m"])
    run_out0 = x
    hf = np.repeat(prof[None, :], ny, axis=0)  # rows = y, cols = x
    # rubble: on the landing and the run-out, never on a step edge
    for _ in range(_i(rng, k["rubble_n"])):
        zone = (land0, land0 + 0.35) if rng.uniform() < 0.5 else (run_out0, max(run_out0 + 0.2, k["track_m"] + 0.3))
        cx, cy = _u(rng, zone), _u(rng, (-0.38, 0.38))
        sx, sy, hh = _u(rng, k["rubble_size"]), _u(rng, k["rubble_size"]), _u(rng, k["rubble_h"])
        mx = (xs > cx - sx / 2) & (xs < cx + sx / 2)
        my = (ys > cy - sy / 2) & (ys < cy + sy / 2)
        if not mx.any() or not my.any():
            continue
        block = hf[np.ix_(my, mx)]
        if block.max() - block.min() > 1e-6:  # straddles a step or the ramp: skip
            continue
        hf[np.ix_(my, mx)] = block + hh
    pipes = []
    for _ in range(_i(rng, k["pipe_n"])):
        px = _u(rng, (0.9, k["track_m"] - 0.2))
        near = (xs > px - 0.25) & (xs < px + 0.25)
        ground = float(hf[:, near].max())
        pipes.append((px, ground + _u(rng, k["pipe_clear_m"]) + k["pipe_r"]))
    return {"hf": hf, "nx": nx, "ny": ny, "pipes": pipes, "features": feats, "knobs": k,
            "noise": (_u(rng, (0, k["obs_noise"][0])), _u(rng, (0, k["obs_noise"][1]))),
            "noise_seed": int(rng.integers(0, 2 ** 31))}


def ground_height(terrain, x, y=0.0):
    j = int(round((x - HF_X0) / HF_RES))
    i = int(round((y + CORRIDOR / 2) / HF_RES))
    j = min(max(j, 0), terrain["nx"] - 1)
    i = min(max(i, 0), terrain["ny"] - 1)
    return float(terrain["hf"][i, j])


def _fmt(v):
    return " ".join("%.6g" % x for x in v)


def build_mjcf(design, catalog, terrain):
    acts, links, feet = catalog["actuators"], catalog["links"], catalog["feet"]
    body = catalog["bodies"][design["body"]]
    rng_j = catalog["joint_ranges"]
    hf = terrain["hf"]
    zmin, zmax = float(hf.min()), float(hf.max())
    elev = max(zmax - zmin, 0.01)
    terrain["hf_base"], terrain["hf_elev"] = zmin, elev
    p = design["params"]
    out = ['<mujoco model="qm24_walkway">',
           '<compiler angle="radian" autolimits="true"/>',
           '<option timestep="%g" integrator="implicitfast" cone="pyramidal"/>' % TIMESTEP,
           '<visual><global offwidth="1280" offheight="960"/><quality shadowsize="4096"/></visual>',
           '<asset>',
           '<texture name="grid" type="2d" builtin="checker" rgb1="0.42 0.40 0.36" rgb2="0.36 0.34 0.31" width="512" height="512"/>',
           '<material name="ground" texture="grid" texrepeat="10 2" texuniform="false" reflectance="0"/>',
           '<hfield name="walk" nrow="%d" ncol="%d" size="%g %g %g %g"/>' % (
               terrain["ny"], terrain["nx"], (HF_X1 - HF_X0) / 2, CORRIDOR / 2, elev, 0.05),
           '</asset>',
           '<default><geom solref="%g 1" solimp="0.9 0.95 0.001"/></default>' % SOLREF,
           '<worldbody>',
           '<light pos="1.5 -1.5 3" dir="-0.3 0.4 -1" diffuse="0.8 0.8 0.8" castshadow="true"/>',
           '<light pos="1.5 1.5 3" dir="0 0 -1" diffuse="0.35 0.35 0.35" castshadow="false"/>',
           '<geom name="walk" type="hfield" hfield="walk" pos="%g 0 %g" material="ground" contype="1" conaffinity="0" friction="%g 0.005 0.0001" group="0"/>' % (
               (HF_X0 + HF_X1) / 2, zmin, GROUND_FRICTION)]
    for s in (1, -1):
        out.append('<geom name="wall%s" type="box" size="%g 0.02 0.35" pos="%g %g %g" rgba="0.55 0.57 0.6 0.35" contype="1" conaffinity="0" group="0"/>' % (
            "L" if s > 0 else "R", (HF_X1 - HF_X0) / 2, (HF_X0 + HF_X1) / 2, s * (CORRIDOR / 2 + 0.02), zmin + 0.3))
    for n, (px, pz) in enumerate(terrain["pipes"]):
        out.append('<geom name="pipe%d" type="cylinder" size="%g %g" pos="%g 0 %g" euler="1.5708 0 0" rgba="0.75 0.45 0.15 1" contype="2" conaffinity="0" group="2"/>' % (
            n, terrain["knobs"]["pipe_r"], CORRIDOR / 2, px, pz))
    out.append('<geom name="goal" type="box" size="0.01 %g 0.002" pos="%g 0 %g" rgba="0.1 0.7 0.2 1" contype="0" conaffinity="0" group="3"/>' % (
        CORRIDOR / 2, terrain["knobs"]["track_m"], ground_height(terrain, terrain["knobs"]["track_m"]) + 0.001))
    sx, sy, sz = body["size"]
    out.append('<body name="torso" pos="0 0 1">')
    out.append('<freejoint name="root"/>')
    out.append('<geom name="torso" type="box" size="%g %g %g" mass="%g" rgba="0.2 0.35 0.6 1" contype="0" conaffinity="3" group="1"/>' % (
        sx / 2, sy / 2, sz / 2, body["mass"]))
    out.append('<geom name="pod" type="box" size="0.07 0.05 0.03" pos="0 0 %g" mass="%g" rgba="0.85 0.75 0.2 1" contype="0" conaffinity="3" group="1"/>' % (
        sz / 2 + 0.03, catalog["limits"]["payload_kg"]))
    out.append('<site name="imu" pos="0 0 0"/>')
    closes = 0
    for leg in LEGS:
        L = design["legs"][leg]
        side = SIDE[leg]
        mount = L["mount"]
        depth = 0

        def act_body(name, aid, pos, joint, axis, rng, lateral):
            a = acts[aid]
            out.append('<body name="%s" pos="%s">' % (name, _fmt(pos)))
            out.append('<joint name="%s" type="hinge" axis="%s" range="%g %g" armature="%g" damping="0.01" frictionloss="0.01"/>' % (
                joint, axis, rng[0], rng[1], a["armature"]))
            if lateral:
                fr = "0 0 0 0 %g 0" % (side * a["length"])
            else:
                fr = "0 %g 0 0 %g 0" % (-a["length"] / 2, a["length"] / 2)
            out.append('<geom type="cylinder" fromto="%s" size="%g" mass="%g" rgba="0.15 0.15 0.17 1" contype="0" conaffinity="2" group="1"/>' % (
                fr, a["radius"], a["mass"]))

        if L["abd"]:
            act_body(leg + "_abd", L["abd"], mount, leg + "_abd", "1 0 0", rng_j["abd"], True)
            depth += 1
            act_body(leg + "_hip", L["hip"], [0, side * acts[L["abd"]]["length"], 0], leg + "_hip", "0 1 0", rng_j["hip"], True)
        else:
            act_body(leg + "_hip", L["hip"], mount, leg + "_hip", "0 1 0", rng_j["hip"], True)
        depth += 1
        pos = [0, side * acts[L["hip"]]["length"], 0]
        for k, lid in enumerate(L["thigh"]):
            ln = links[lid]
            out.append('<body name="%s_thigh%d" pos="%s">' % (leg, k + 1, _fmt(pos)))
            out.append('<geom type="capsule" fromto="0 0 0 0 0 %g" size="%g" mass="%g" rgba="0.7 0.7 0.72 1" contype="0" conaffinity="2" group="1"/>' % (
                -ln["length"], ln["radius"], ln["mass"]))
            depth += 1
            pos = [0, 0, -ln["length"]]
        act_body(leg + "_knee", L["knee"], pos, leg + "_knee", "0 1 0", rng_j["knee"], False)
        depth += 1
        pos = [0, 0, 0]
        for k, lid in enumerate(L["shank"]):
            ln = links[lid]
            out.append('<body name="%s_shank%d" pos="%s">' % (leg, k + 1, _fmt(pos)))
            out.append('<geom type="capsule" fromto="0 0 0 0 0 %g" size="%g" mass="%g" rgba="0.7 0.7 0.72 1" contype="0" conaffinity="2" group="1"/>' % (
                -ln["length"], ln["radius"], ln["mass"]))
            depth += 1
            pos = [0, 0, -ln["length"]]
        ft = feet[L["foot"]]
        out.append('<body name="%s_foot" pos="%s">' % (leg, _fmt(pos)))
        out.append('<geom name="%s_foot" type="sphere" size="%g" mass="%g" friction="%g 0.005 0.0001" rgba="0.1 0.1 0.1 1" contype="0" conaffinity="1" condim="3" group="1"/>' % (
            leg, ft["radius"], ft["mass"], ft["friction"]))
        out.append('</body>')
        out.append('</body>' * depth)
    out.append('</body>')
    out.append('<camera name="side" mode="fixed" pos="1.5 -3.2 1.0" xyaxes="1 0 0 0 0.3 1"/>')
    out.append('</worldbody>')
    out.append('<actuator>')
    for leg in LEGS:
        L = design["legs"][leg]
        for role in ("abd", "hip", "knee"):
            if role == "abd" and not L["abd"]:
                continue
            tq = acts[L[role]]["torque"]
            out.append('<motor name="%s_%s" joint="%s_%s" ctrlrange="%g %g"/>' % (leg, role, leg, role, -tq, tq))
    out.append('</actuator>')
    out.append('</mujoco>')
    return "\n".join(out)


class Episode(object):
    def __init__(self, design, catalog, terrain):
        self.design, self.catalog, self.terrain = design, catalog, terrain
        self.xml = build_mjcf(design, catalog, terrain)
        self.m = mujoco.MjModel.from_xml_string(self.xml)
        hf = terrain["hf"]
        self.m.hfield_data[:] = ((hf - terrain["hf_base"]) / terrain["hf_elev"]).astype(np.float32).ravel()
        self.d = mujoco.MjData(self.m)
        self.names = joint_names(design)
        self.qadr = np.array([self.m.jnt_qposadr[self.m.joint(n).id] for n in self.names])
        self.vadr = np.array([self.m.jnt_dofadr[self.m.joint(n).id] for n in self.names])
        roles = [n.split("_")[1] for n in self.names]
        legs = [n.split("_")[0] for n in self.names]
        A = [catalog["actuators"][design["legs"][lg][r]] for lg, r in zip(legs, roles)]
        self.kp = np.array([a["kp"] for a in A])
        self.kd = np.array([a["kd"] for a in A])
        self.tmax = np.array([a["torque"] for a in A])
        self.wmax = np.array([a["speed"] for a in A])
        self.gait = qm_gait.StandardGait(design, catalog)
        self.torso = self.m.body("torso").id
        self.robot_geoms = set(g for g in range(self.m.ngeom) if self.m.geom_bodyid[g] != 0)
        self.foot_gid = {leg: self.m.geom(leg + "_foot").id for leg in LEGS}
        self.foot_geoms = set(self.foot_gid.values())
        self.foot_index = {self.foot_gid[leg]: j for j, leg in enumerate(LEGS)}
        self.touch = np.zeros(4, bool)
        self.body_geoms = set([self.m.geom("torso").id, self.m.geom("pod").id])
        self.sat = 0.0
        self._init_pose()

    def _init_pose(self):
        m, d, g = self.m, self.d, self.gait
        mujoco.mj_resetData(m, d)
        h = g.size["stance_height"]
        geo = g.geo
        rfoot = max(geo[leg]["r"] for leg in LEGS)
        q = []
        for leg in LEGS:
            q += g.leg_targets(leg, 0.0, geo[leg]["w"], -h)
        q = np.array(q)
        self.q_cmd = q.copy()
        gz = max(ground_height(self.terrain, geo[leg]["mount"][0], geo[leg]["mount"][1] + geo[leg]["w"]) for leg in LEGS)
        d.qpos[0:3] = [0.0, 0.0, gz + h + rfoot + 0.005]
        d.qpos[3:7] = [1, 0, 0, 0]
        d.qpos[self.qadr] = q
        mujoco.mj_forward(m, d)

    def torques(self, q_cmd, qd_cmd):
        d = self.d
        q, qd = d.qpos[self.qadr], d.qvel[self.vadr]
        tau = self.kp * (q_cmd - q) + self.kd * (qd_cmd - qd)
        hi = self.tmax * np.clip(1.0 - qd / self.wmax, 0.0, 1.0)
        lo = -self.tmax * np.clip(1.0 + qd / self.wmax, 0.0, 1.0)
        out = np.clip(tau, lo, hi)
        self.sat += float(np.mean(np.abs(out - tau) > 1e-9))
        return out

    def observe(self, rng, noise):
        d = self.d
        R = d.xmat[self.torso].reshape(3, 3)
        g = R.T @ np.array([0.0, 0.0, -1.0])
        g = g + rng.normal(0, noise[0], 3)
        g = g / np.linalg.norm(g)
        w = d.qvel[3:6] + rng.normal(0, noise[1], 3)
        q = d.qpos[self.qadr] + rng.normal(0, 0.002, len(self.qadr))
        contact = [int(c) for c in self.touch]
        # the terrain scanner: ground height relative to the body origin, in the heading-aligned level frame
        px, py, pz = (float(v) for v in d.xpos[self.torso])
        yaw = math.atan2(R[1, 0], R[0, 0])
        cy, sy = math.cos(yaw), math.sin(yaw)
        terr = self.terrain

        def height(x, y):
            return ground_height(terr, px + cy * x - sy * y, py + sy * x + cy * y) - pz

        return {"q": q, "contact": contact, "gravity": g, "gyro": w, "height": height}

    def sense_contacts(self):
        """Foot switches: a foot counts as touching when it touched the ground at any physics step of the period."""
        d = self.d
        for i in range(d.ncon):
            c = d.contact[i]
            for gg in (c.geom1, c.geom2):
                j = self.foot_index.get(gg)
                if j is not None:
                    self.touch[j] = True

    def clearance(self):
        m, d = self.m, self.d
        sx, sy, sz = self.catalog["bodies"][self.design["body"]]["size"]
        R = d.xmat[self.torso].reshape(3, 3)
        p = d.xpos[self.torso]
        best = 1e9
        grp = np.array([1, 0, 0, 0, 0, 0], np.uint8)
        gid = np.zeros(1, np.int32)
        for cx, cy in ((0, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)):
            pt = p + R @ np.array([cx * sx / 2, cy * sy / 2, -sz / 2])
            dist = mujoco.mj_ray(m, d, pt, np.array([0.0, 0.0, -1.0]), grp, 1, -1, gid)
            if dist >= 0:
                best = min(best, dist)
        return best

    def check(self):
        """None, ('fall', reason) or ('fault', reason)."""
        m, d = self.m, self.d
        if not np.all(np.isfinite(d.qpos)) or not np.all(np.isfinite(d.qvel)):
            return ("fault", "simulation diverged")
        if np.linalg.norm(d.qvel[0:3]) > 4.0:
            return ("fault", "runaway body speed")
        for w in (mujoco.mjtWarning.mjWARN_BADQACC, mujoco.mjtWarning.mjWARN_CONTACTFULL, mujoco.mjtWarning.mjWARN_CNSTRFULL):
            if d.warning[w].number > 0:
                return ("fault", "solver warning")
        for i in range(d.ncon):
            c = d.contact[i]
            for gg in (c.geom1, c.geom2):
                if gg in self.body_geoms:
                    return ("fall", "body contact")
                if gg in self.robot_geoms and gg not in self.foot_geoms:
                    return ("fall", "leg hit an overhead pipe")
        R = d.xmat[self.torso].reshape(3, 3)
        if R[2, 2] < math.cos(math.radians(FALL_TILT_DEG)):
            return ("fall", "tipped over")
        cl = self.clearance()
        if cl < FALL_CLEARANCE:
            return ("fall", "body too low")
        if cl > self.gait.size["reach"] + 0.25:
            return ("fault", "flight")
        return None

    def run(self, seed=0, frame_cb=None, settle_check_s=0.0):
        rng = np.random.default_rng(self.terrain["noise_seed"] + seed)
        noise = self.terrain["noise"]
        k = self.terrain["knobs"]
        L, tl = k["track_m"], k["time_limit_s"]
        nsteps = int(round(tl / (TIMESTEP * SUBSTEPS)))
        lag = int(round(FALL_LAG_S / (TIMESTEP * SUBSTEPS)))
        self.gait.reset()
        xs = []
        obs_prev = self.observe(rng, noise)
        outcome, reason, t_end = "timeout", "time limit", tl
        q_cmd = self.q_cmd
        q_prev = self.q_cmd.copy()
        for step in range(nsteps):
            obs = self.observe(rng, noise)
            q_next = self.gait.act(step, obs_prev)  # one control period of latency, in simulated time
            obs_prev = obs
            self.touch[:] = False
            # each actuator's servo interpolates its target over the control period (rate = target change / period)
            qd_cmd = (q_cmd - q_prev) / (TIMESTEP * SUBSTEPS)
            for j in range(SUBSTEPS):
                qc = q_prev + (q_cmd - q_prev) * (j + 1) / SUBSTEPS
                self.d.ctrl[:] = self.torques(qc, qd_cmd)
                mujoco.mj_step(self.m, self.d)
                self.sense_contacts()
            q_prev, q_cmd = q_cmd, q_next
            x = float(self.d.xpos[self.torso][0])
            xs.append(x)
            if frame_cb is not None:
                frame_cb(self, step)
            ev = self.check()
            if ev is not None:
                outcome, reason, t_end = ev[0], ev[1], (step + 1) * TIMESTEP * SUBSTEPS
                break
            if x >= L:
                outcome, reason, t_end = "goal", "reached the goal", (step + 1) * TIMESTEP * SUBSTEPS
                break
        if outcome == "fault":
            prog = 0.0
        elif outcome == "fall":
            prog = max(xs[: max(len(xs) - lag, 1)])
            prog = max(0.0, min(prog, xs[0] if len(xs) <= lag else prog))
            prog = min(max(prog / L, 0.0), 1.0)
        else:
            prog = min(max(max(xs) / L, 0.0), 1.0)
        n = max(len(xs), 1)
        return {"progress": prog, "outcome": outcome, "reason": reason, "time_s": round(t_end, 2),
                "x_final": xs[-1] if xs else 0.0, "torque_saturation": self.sat / (n * SUBSTEPS)}

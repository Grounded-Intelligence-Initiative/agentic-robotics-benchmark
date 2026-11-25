"""The cell: the Panda (MuJoCo Menagerie, gravity compensated, joint position servos) with a deployable pusher rod and a
wrist force/torque sensor, the block tower on its turntable, a staging post, three RGB-D cameras; the controller (10 ms
ticks: straight-line Cartesian goals with slerped orientation solved by damped least squares onto the joint servos, or
joint goals; a force-controlled gripper; the turntable's trapezoidal profile) and the referee (removals, placements,
collapse, the episode's score).

Referee (score section of config.py): a block of the tower whose centre has moved more than `removed_beyond_m` from
its reference position (turntable frame) has been REMOVED; the removal is legal when its level is below the second
highest complete level at that moment. A removed block that comes to rest, not touching the fingers, in a free slot of
the current top level (or of a new level when the top level is complete) within the placement tolerances is PLACED
and becomes part of the tower. Any block of the tower more than `fall_dz_m` below its reference height, or tilted more
than `fall_tilt_deg`, COLLAPSES the tower (permanent). Score = 0 when collapsed, else (placed blocks whose removal was
legal and that are still in their slot after the final settle) / n_cap, capped at 1.
"""

import math
import os

import mujoco
import numpy as np

from . import tower as TW
from .config import CONFIG, PHYSICS_DT, SUBSTEPS, TICK_S, level_yaw

ARM_J = ["joint%d" % i for i in range(1, 8)]


class MoveError(Exception):
    pass


def topdown_mat(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, s, 0.0], [s, -c, 0.0], [0.0, 0.0, -1.0]])


def quat_to_mat(q):
    R = np.zeros(9)
    mujoco.mju_quat2Mat(R, np.asarray(q, float) / np.linalg.norm(q))
    return R.reshape(3, 3)


def mat_to_quat(R):
    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, np.ascontiguousarray(R, dtype=float).ravel())
    return q


def _lookat_xyaxes(pos, target, up=(0.0, 0.0, 1.0)):
    f = np.asarray(target, float) - np.asarray(pos, float)
    f /= np.linalg.norm(f)
    u = np.asarray(up, float)
    if abs(np.dot(f, u)) > 0.98:
        u = np.array([1.0, 0.0, 0.0])
    x = np.cross(f, u)
    x /= np.linalg.norm(x)
    y = np.cross(x, f)
    return list(x) + list(y)


def build_spec(tower, panda_dir):
    spec = mujoco.MjSpec.from_file(os.path.join(panda_dir, "panda.xml"))
    spec.modelname = "block_tower_cell"
    o = spec.option
    o.timestep = PHYSICS_DT
    o.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    o.cone = mujoco.mjtCone.mjCONE_ELLIPTIC
    o.impratio = 10.0
    o.iterations = 100
    o.ls_iterations = 30
    o.tolerance = 1e-10
    spec.visual.quality.shadowsize = 0
    spec.visual.global_.offwidth = 640
    spec.visual.global_.offheight = 480
    spec.visual.map.znear = 0.01
    spec.stat.extent = 2.0
    spec.visual.headlight.ambient = [0.2, 0.2, 0.2]
    spec.visual.headlight.diffuse = [0.3, 0.3, 0.3]
    spec.visual.headlight.specular = [0.0, 0.0, 0.0]
    for b in spec.bodies:
        if b.name != "world":
            b.gravcomp = 1.0
    rob = CONFIG["robot"]
    tcp = rob["tcp_offset_m"]
    pu = rob["pusher"]
    hand = spec.body("hand")
    hand.add_site(name="tcp", pos=[0, 0, tcp], size=[0.002, 0, 0], group=4)
    hand.add_site(name="ft", pos=[0, 0, 0], size=[0.002, 0, 0], group=4)
    zr = tcp - pu["dz_above_tcp_m"]
    hand.add_site(name="rod_tip", pos=[pu["x0_m"] + pu["length_m"], 0, zr], size=[0.002, 0, 0], group=4)
    rod = dict(rgba=[0.15, 0.15, 0.17, 1], friction=[0.2, 0.002, 0.0001], solref=[0.002, 1.0],
               solimp=[0.99, 0.999, 0.0002, 0.5, 2.0])
    hand.add_geom(name="rod_post", type=mujoco.mjtGeom.mjGEOM_BOX, pos=[pu["x0_m"] + 0.004, 0, (0.066 + zr) / 2],
                  size=[0.004, 0.006, (zr - 0.066) / 2 + 0.003], **rod)
    hand.add_geom(name="rod", type=mujoco.mjtGeom.mjGEOM_CAPSULE,
                  fromto=[pu["x0_m"], 0, zr, pu["x0_m"] + pu["length_m"] - pu["radius_m"], 0, zr],
                  size=[pu["radius_m"], 0, 0], **rod)
    spec.add_sensor(name="ft_force", type=mujoco.mjtSensor.mjSENS_FORCE, objtype=mujoco.mjtObj.mjOBJ_SITE, objname="ft")
    spec.add_sensor(name="ft_torque", type=mujoco.mjtSensor.mjSENS_TORQUE, objtype=mujoco.mjtObj.mjOBJ_SITE, objname="ft")
    wc = CONFIG["cameras"]["wrist"]
    hand.add_camera(name="wrist", pos=list(wc["hand_pos"]),
                    xyaxes=_lookat_xyaxes(wc["hand_pos"], wc["hand_look_at"], up=(1.0, 0.0, 0.0)), fovy=wc["fovy_deg"])
    wb = spec.worldbody
    wb.add_light(name="sun", pos=[0.4, 0.0, 2.5], dir=[0, 0, -1], diffuse=[0.5, 0.5, 0.5],
                 type=mujoco.mjtLightType.mjLIGHT_DIRECTIONAL, castshadow=0)
    wb.add_light(name="fill", pos=[-1.0, -1.0, 2.0], dir=[0.4, 0.4, -1], diffuse=[0.3, 0.3, 0.3],
                 type=mujoco.mjtLightType.mjLIGHT_DIRECTIONAL, castshadow=0)
    wb.add_geom(name="floor", type=mujoco.mjtGeom.mjGEOM_PLANE, size=[1.5, 1.5, 0.01], rgba=[0.45, 0.46, 0.48, 1])
    sp = CONFIG["staging_post"]
    wb.add_geom(name="staging_post", type=mujoco.mjtGeom.mjGEOM_BOX,
                pos=[sp["xy"][0], sp["xy"][1], sp["top_z"] / 2], size=[sp["half_xy"][0], sp["half_xy"][1], sp["top_z"] / 2],
                rgba=[0.25, 0.35, 0.3, 1], friction=[0.6, 0.002, 0.0001], margin=TW.MARGIN)
    TW.add_tower(spec, tower)
    for name in ("left", "right"):
        cc = CONFIG["cameras"][name]
        wb.add_camera(name=name, pos=list(cc["pos"]), xyaxes=_lookat_xyaxes(cc["pos"], cc["target"]), fovy=cc["fovy_deg"])
    return spec


class Cell(object):
    def __init__(self, panda_dir, tower, duration_s=None, settle_s=2.0):
        self.tower = tower
        self.seed = int(tower["seed"])
        self.spec = build_spec(tower, panda_dir)
        self.m = self.spec.compile()
        m = self.m
        self.d = mujoco.MjData(m)
        self.kd = mujoco.MjData(m)
        d = self.d
        self.duration = float(duration_s if duration_s is not None else CONFIG["episode"]["duration_s"])
        self.qadr = np.array([m.jnt_qposadr[m.joint(j).id] for j in ARM_J])
        self.dadr = np.array([m.jnt_dofadr[m.joint(j).id] for j in ARM_J])
        rng_ = m.jnt_range[[m.joint(j).id for j in ARM_J]]
        self.lo = rng_[:, 0] + 0.02
        self.hi = rng_[:, 1] - 0.02
        self.fadr = [m.jnt_qposadr[m.joint(j).id] for j in ("finger_joint1", "finger_joint2")]
        self.site = {n: m.site(n).id for n in ("tcp", "rod_tip", "ft")}
        self.n = len(tower["blocks"])
        self.ids = [m.body("blk%d" % i).id for i in range(self.n)]
        self.idset = set(self.ids)
        self.robot_bodies = set(m.body(n).id for n in ["link%d" % i for i in range(8)] + ["hand", "left_finger", "right_finger"])
        self.finger_bodies = (m.body("left_finger").id, m.body("right_finger").id)
        self.rod_geoms = [m.geom("rod_post").id, m.geom("rod").id]
        self.tadr = m.jnt_qposadr[m.joint("turntable_j").id]
        self.tdadr = m.jnt_dofadr[m.joint("turntable_j").id]
        self.tact = m.actuator("turntable").id
        self.sens_f = m.sensor_adr[m.sensor("ft_force").id]
        self.sens_t = m.sensor_adr[m.sensor("ft_torque").id]
        self.cam = {m.camera(i).name: i for i in range(m.ncam)}
        rob = CONFIG["robot"]
        self.q_home = np.array(rob["home_q"], float)
        d.qpos[self.qadr] = self.q_home
        d.ctrl[:7] = self.q_home
        self.q_cmd = self.q_home.copy()
        self.q_nom = self.q_home.copy()
        m.actuator_forcelimited[:7] = 0
        self.grip_open(0.08)
        d.qpos[self.fadr] = 0.04
        self.pusher(False)
        self.goal = None
        self.ref_pos = self.ref_quat = None
        self.hold = 0
        self.ik_err = (0.0, 0.0)
        self.reached = True
        self.out_of_reach = False
        self.table_cmd = 0.0
        self.table_goal = 0.0
        self.table_v = 0.0
        self.t = 0.0
        self.ticks = 0
        self.done = False
        self.done_reason = None
        self.frames_used = 0
        self.frame_budget = int(CONFIG["budgets"]["frames_per_episode"])
        self.stats = {"frames": 0, "removals": 0, "illegal_removals": 0, "placements": 0, "max_contact_N": 0.0}
        self.ft_rng = np.random.default_rng([self.seed, 77])
        self.ft_bias = self.ft_rng.normal(0.0, 0.05, 3)
        mujoco.mj_forward(m, d)
        for _ in range(int(round(settle_s / PHYSICS_DT))):
            mujoco.mj_step(m, d)
        # referee state
        sc = CONFIG["score"]
        self.sc = sc
        self.state = ["tower"] * self.n
        self.level = [b["level"] for b in tower["blocks"]]
        self.slot = [b["slot"] for b in tower["blocks"]]
        self.ref = self.table_frame(d.xpos[self.ids])
        self.legal = [False] * self.n
        self.placed = [False] * self.n
        self.collapsed = False
        self.collapse_info = None
        self.events = []
        self._tally = None

    # ------------------------------------------------------------------ frames
    def table_angle(self):
        return float(self.d.qpos[self.tadr])

    def table_frame(self, p):
        th = self.table_angle()
        ax = CONFIG["tower"]["turntable_axis_xy"]
        c, s = math.cos(-th), math.sin(-th)
        p = np.asarray(p, float)
        dx, dy = p[..., 0] - ax[0], p[..., 1] - ax[1]
        return np.stack([c * dx - s * dy, s * dx + c * dy, p[..., 2]], axis=-1)

    def site_pose(self, name, d=None):
        d = d or self.d
        s = self.site[name]
        return d.site_xpos[s].copy(), d.site_xmat[s].reshape(3, 3).copy()

    # ------------------------------------------------------------------ tools
    def pusher(self, on):
        m = self.m
        for g in self.rod_geoms:
            m.geom_contype[g] = 1 if on else 0
            m.geom_conaffinity[g] = 1 if on else 0
            m.geom_rgba[g, 3] = 1.0 if on else 0.0
        b = m.geom_bodyid[self.rod_geoms[0]]
        m.body_contype[b] = 1
        m.body_conaffinity[b] = 1
        self.pusher_on = bool(on)

    def grip_open(self, width=0.08):
        m = self.m
        width = float(min(max(width, 0.0), 0.08))
        m.actuator_gainprm[7, 0] = 0.01568627451
        m.actuator_biasprm[7, :3] = [0, -100, -10]
        m.actuator_ctrlrange[7] = [0, 255]
        self.d.ctrl[7] = 255.0 * width / 0.08
        self.grip_mode = ("open", width)

    def grip_close(self, force=20.0):
        m = self.m
        force = float(min(max(force, 1.0), CONFIG["robot"]["grip_force_max_N"]))
        m.actuator_gainprm[7, 0] = 1.0
        m.actuator_biasprm[7, :3] = [0, 0, -10]
        m.actuator_ctrlrange[7] = [-200, 255]
        self.d.ctrl[7] = -force
        self.grip_mode = ("close", force)

    def opening(self):
        return float(self.d.qpos[self.fadr[0]] + self.d.qpos[self.fadr[1]])

    def wrench(self, noisy=True):
        """Force / torque the flange exerts on the hand assembly (hand, fingers, pusher), world frame. An upward push
        on the hand from outside reads as -z; the hand's own weight is in it (tare it)."""
        _, R = self.site_pose("ft")
        f = R @ self.d.sensordata[self.sens_f:self.sens_f + 3]
        tq = R @ self.d.sensordata[self.sens_t:self.sens_t + 3]
        if noisy:
            rob = CONFIG["robot"]
            f = f + self.ft_bias + self.ft_rng.normal(0.0, rob["ft_noise_N"], 3)
            tq = tq + self.ft_rng.normal(0.0, rob["ft_torque_noise_Nm"], 3)
        return f, tq

    # ------------------------------------------------------------------ IK and goals
    def ik(self, frame, pos, R_t, q0, iters=3, q_nom=None):
        m, kd = self.m, self.kd
        s = self.site[frame]
        q = np.array(q0, float)
        qn = self.q_nom if q_nom is None else q_nom
        ep = er = 0.0
        for _ in range(iters):
            kd.qpos[self.qadr] = q
            mujoco.mj_kinematics(m, kd)
            mujoco.mj_comPos(m, kd)
            p = kd.site_xpos[s]
            R = kd.site_xmat[s].reshape(3, 3)
            e_p = pos - p
            e_r = 0.5 * (np.cross(R[:, 0], R_t[:, 0]) + np.cross(R[:, 1], R_t[:, 1]) + np.cross(R[:, 2], R_t[:, 2]))
            jp = np.zeros((3, m.nv))
            jr = np.zeros((3, m.nv))
            mujoco.mj_jacSite(m, kd, jp, jr, s)
            J = np.vstack([jp[:, self.dadr], jr[:, self.dadr]])
            e = np.concatenate([e_p, e_r])
            JJt = J @ J.T + 1e-4 * np.eye(6)
            dq = J.T @ np.linalg.solve(JJt, e)
            Jp = J.T @ np.linalg.solve(JJt, np.eye(6))
            dq += (np.eye(7) - Jp @ J) @ (0.05 * (qn - q))
            q = np.clip(q + dq, self.lo, self.hi)
            ep, er = float(np.linalg.norm(e_p)), float(np.linalg.norm(e_r))
        return q, ep, er

    def solve_ik(self, frame, pos, R, seed, iters=150):
        q = np.array(seed, float)
        for _ in range(iters):
            q, ep, er = self.ik(frame, np.asarray(pos, float), R, q, iters=3, q_nom=np.array(seed, float))
        return q, ep, er

    def set_cart_goal(self, frame, pos, quat, speed, rot_speed=1.0):
        kd = self.kd
        kd.qpos[self.qadr] = self.q_cmd
        mujoco.mj_kinematics(self.m, kd)
        self.ref_pos = kd.site_xpos[self.site[frame]].copy()
        self.ref_quat = mat_to_quat(kd.site_xmat[self.site[frame]].reshape(3, 3))
        q_t = np.asarray(quat, float) / np.linalg.norm(quat)
        if np.dot(self.ref_quat, q_t) < 0:
            q_t = -q_t
        rob = CONFIG["robot"]
        self.goal = {"kind": "cart", "frame": frame, "pos": np.asarray(pos, float), "quat": q_t,
                     "R": quat_to_mat(q_t), "speed": float(min(max(speed, 0.001), rob["max_speed_m_s"])),
                     "rot_speed": float(min(max(rot_speed, 0.01), rob["max_rot_rad_s"]))}
        self.hold = 0
        self.reached = False
        self.out_of_reach = False

    def set_joint_goal(self, q, speed):
        self.goal = {"kind": "joint", "q": np.clip(np.asarray(q, float), self.lo, self.hi),
                     "speed": float(min(max(speed, 0.01), 2.0))}
        self.hold = 0
        self.reached = False
        self.out_of_reach = False

    def _advance(self):
        g = self.goal
        if g is None or self.reached:
            return
        if g["kind"] == "joint":
            dq = g["q"] - self.q_cmd
            mx = float(np.max(np.abs(dq)))
            if mx < 1e-9:
                self.hold += 1
                if self.hold >= 20:
                    self.reached = True
                return
            self.q_cmd = self.q_cmd + dq * min(1.0, g["speed"] * TICK_S / mx)
            return
        dp = g["pos"] - self.ref_pos
        dist = float(np.linalg.norm(dp))
        step = g["speed"] * TICK_S
        self.ref_pos = g["pos"].copy() if dist <= step else self.ref_pos + dp * (step / dist)
        ang = 2 * math.acos(min(1.0, abs(float(np.dot(self.ref_quat, g["quat"])))))
        if ang > 1e-9:
            f = min(1.0, g["rot_speed"] * TICK_S / ang)
            qn = self.ref_quat + f * (g["quat"] - self.ref_quat)
            self.ref_quat = qn / np.linalg.norm(qn)
        at_goal = dist <= step and ang < 1e-3
        R_ref = g["R"] if at_goal else quat_to_mat(self.ref_quat)
        q, ep, er = self.ik(g["frame"], self.ref_pos, R_ref, self.q_cmd)
        dq = q - self.q_cmd
        cap = CONFIG["robot"]["joint_speed_cap_rad_s"] * TICK_S
        mx = float(np.max(np.abs(dq)))
        if mx > cap:
            dq *= cap / mx
        self.q_cmd = self.q_cmd + dq
        self.ik_err = (ep, er)
        if at_goal:
            self.hold += 1
            if (ep < 3e-4 and er < 3e-3 and self.hold >= 15) or self.hold >= 300:
                self.reached = True
                self.out_of_reach = not (ep < 1.5e-3 and er < 1e-2)

    def _table(self):
        a = CONFIG["tower"]["turntable_accel_rad_s2"]
        vmax = CONFIG["tower"]["turntable_speed_rad_s"]
        dist = self.table_goal - self.table_cmd
        if abs(dist) < 1e-12:
            self.table_v = 0.0
            return
        sgn = 1.0 if dist > 0 else -1.0
        v = min(abs(self.table_v) + a * TICK_S, vmax, math.sqrt(2 * a * abs(dist)) + 1e-3)
        stp = min(abs(dist), v * TICK_S)
        self.table_v = sgn * v
        self.table_cmd += sgn * stp

    def table_moving(self):
        return abs(self.table_goal - self.table_cmd) > 1e-9 or abs(float(self.d.qvel[self.tdadr])) > 1e-3

    # ------------------------------------------------------------------ ticking
    def tick(self):
        if self.done:
            return
        self._advance()
        self._table()
        d = self.d
        d.ctrl[:7] = self.q_cmd
        d.ctrl[self.tact] = self.table_cmd
        for _ in range(SUBSTEPS):
            mujoco.mj_step(self.m, d)
        self.ticks += 1
        self.t = round(self.ticks * TICK_S, 6)
        if self.ticks % 10 == 0:
            self.referee()
        if self.t >= self.duration - 1e-9 and not self.done:
            self.end("episode_end")

    def run_ticks(self, n):
        for _ in range(int(n)):
            if self.done:
                break
            self.tick()

    def end(self, reason):
        if not self.done:
            self.done = True
            self.done_reason = reason

    # ------------------------------------------------------------------ referee
    def _tilt(self, b):
        z = float(self.d.xmat[b][8])
        t = math.acos(max(-1.0, min(1.0, z)))
        return min(t, math.pi - t)

    def _finger_contact(self, b):
        m, d = self.m, self.d
        for k in range(d.ncon):
            c = d.contact[k]
            b1, b2 = m.geom_bodyid[c.geom1], m.geom_bodyid[c.geom2]
            if (b1 == b and b2 in self.robot_bodies) or (b2 == b and b1 in self.robot_bodies):
                return True
        return False

    def members(self, k):
        return [i for i in range(self.n) if self.state[i] == "tower" and self.level[i] == k]

    def complete_levels(self):
        lv = {}
        for i in range(self.n):
            if self.state[i] == "tower":
                lv[self.level[i]] = lv.get(self.level[i], 0) + 1
        return sorted(k for k, c in lv.items() if c == 3)

    def legal_below(self):
        comp = self.complete_levels()
        return comp[-2] if len(comp) >= 2 else -1

    def referee(self, check_place=None):
        if self.collapsed:
            return
        sc = self.sc
        P = self.table_frame(self.d.xpos[self.ids])
        fall_t = math.radians(sc["fall_tilt_deg"])
        for i in range(self.n):
            if self.state[i] != "tower":
                continue
            b = self.ids[i]
            if self.ref[i][2] - P[i][2] > sc["fall_dz_m"] or self._tilt(b) > fall_t:
                self.collapsed = True
                self.collapse_info = {"t": self.t, "block": i, "level": self.level[i],
                                      "dz_m": round(float(self.ref[i][2] - P[i][2]), 4),
                                      "tilt_deg": round(math.degrees(self._tilt(b)), 2)}
                self.events.append(("collapse", self.t, i))
                return
            if np.linalg.norm(P[i][:2] - self.ref[i][:2]) > sc["removed_beyond_m"]:
                lb = self.legal_below()
                self.legal[i] = self.level[i] < lb
                self.state[i] = "out"
                self.placed[i] = False
                self.stats["removals"] += 1
                if not self.legal[i]:
                    self.stats["illegal_removals"] += 1
                self.events.append(("removed", self.t, i, self.level[i], self.legal[i]))
        if check_place or (check_place is None and self.ticks % 50 == 0):
            for i in range(self.n):
                if self.state[i] == "out":
                    self._try_place(i, P)

    def _try_place(self, i, P):
        sc = self.sc
        b = self.ids[i]
        if np.linalg.norm(self.d.cvel[b][3:]) > sc["rest_speed_m_s"]:
            return False
        if self._finger_contact(b):
            return False
        top = max(self.level[j] for j in range(self.n) if self.state[j] == "tower")
        k = top + 1 if len(self.members(top)) == 3 else top
        below = self.members(k - 1)
        if len(below) != 3:
            return False
        h = self.tower["blocks"][i]["h"]
        pb = P[below]
        # level k-1's axis (table frame) and centre
        th = self.table_angle()
        axes = []
        for j in below:
            R = self.d.xmat[self.ids[j]].reshape(3, 3)
            a = R[:, 0].copy()
            a2 = np.array([math.cos(-th) * a[0] - math.sin(-th) * a[1], math.sin(-th) * a[0] + math.cos(-th) * a[1]])
            a2 /= max(1e-9, np.linalg.norm(a2))
            if axes and np.dot(a2, axes[0]) < 0:
                a2 = -a2
            axes.append(a2)
        axb = np.mean(axes, axis=0)
        axb /= np.linalg.norm(axb)
        cen = pb[:, :2].mean(0)
        ztop = max(P[j][2] + self.tower["blocks"][j]["h"] / 2 for j in below)
        R = self.d.xmat[b].reshape(3, 3)
        a = R[:, 0]
        ai = np.array([math.cos(-th) * a[0] - math.sin(-th) * a[1], math.sin(-th) * a[0] + math.cos(-th) * a[1]])
        ai /= max(1e-9, np.linalg.norm(ai))
        # the new level runs across the level below: its axis is level k-1's lateral direction
        if abs(float(np.dot(ai, axb))) > math.sin(math.radians(sc["place_yaw_tol_deg"])):
            return False
        if abs(P[i][2] - (ztop + h / 2 + 2 * TW.MARGIN)) > sc["place_z_tol_m"]:
            return False
        rel = P[i][:2] - cen
        axial = float(np.dot(rel, axb))           # along level k-1's axis = across the new block
        along = float(np.dot(rel, np.array([-axb[1], axb[0]])))
        if abs(along) > sc["place_axial_tol_m"]:
            return False
        pitch = CONFIG["tower"]["slot_pitch_m"]
        s_f = axial / pitch
        s = int(round(s_f))
        if s not in (-1, 0, 1) or abs(axial - s * pitch) > sc["place_lateral_tol_m"]:
            return False
        slot = s + 1
        if any(self.slot[j] == slot for j in self.members(k)):
            return False
        self.state[i] = "tower"
        self.level[i] = k
        self.slot[i] = slot
        self.placed[i] = True
        self.ref[i] = P[i].copy()
        self.stats["placements"] += 1
        self.events.append(("placed", self.t, i, k, slot, self.legal[i]))
        return True

    # ------------------------------------------------------------------ privileged views
    def loads_and_gaps(self):
        """Per block: (normal force from blocks above, vertical gap to the lowest block resting above)."""
        m, d = self.m, self.d
        idx = {b: i for i, b in enumerate(self.ids)}
        top = np.zeros(self.n)
        f6 = np.zeros(6)
        for k in range(d.ncon):
            c = d.contact[k]
            if c.efc_address < 0:
                continue
            b1, b2 = m.geom_bodyid[c.geom1], m.geom_bodyid[c.geom2]
            if b1 not in idx or b2 not in idx or abs(c.frame[2]) < 0.9:
                continue
            mujoco.mj_contactForce(m, d, k, f6)
            lo = b1 if d.xpos[b1][2] < d.xpos[b2][2] else b2
            top[idx[lo]] += abs(f6[0])
        gaps = np.full(self.n, 1.0)
        for i in range(self.n):
            if self.state[i] != "tower":
                continue
            ab = self.members(self.level[i] + 1)
            if not ab:
                continue
            t_i = d.xpos[self.ids[i]][2] + self.tower["blocks"][i]["h"] / 2
            bot = min(d.xpos[self.ids[j]][2] - self.tower["blocks"][j]["h"] / 2 for j in ab)
            gaps[i] = bot - t_i - 2 * TW.MARGIN
        return top, gaps

    def contacts_above(self, i):
        m, d = self.m, self.d
        b = self.ids[i]
        f6 = np.zeros(6)
        tot = 0.0
        pos = np.zeros(3)
        for k in range(d.ncon):
            con = d.contact[k]
            if con.efc_address < 0:
                continue
            b1, b2 = m.geom_bodyid[con.geom1], m.geom_bodyid[con.geom2]
            if b not in (b1, b2):
                continue
            o = b2 if b1 == b else b1
            if o not in self.idset or abs(con.frame[2]) < 0.9 or d.xpos[o][2] < d.xpos[b][2] + 0.005:
                continue
            mujoco.mj_contactForce(m, d, k, f6)
            tot += abs(f6[0])
            pos += abs(f6[0]) * con.pos
        return tot, (pos / tot if tot > 0 else None)

    # ------------------------------------------------------------------ score
    def tally(self):
        if self._tally is not None:
            return self._tally
        # final settle: the arm holds still, the turntable stops where it is
        self.goal = None
        self.table_goal = self.table_cmd
        if not self.collapsed:
            for _ in range(int(round(self.sc["final_settle_s"] / TICK_S))):
                self._table()
                self.d.ctrl[:7] = self.q_cmd
                self.d.ctrl[self.tact] = self.table_cmd
                for _ in range(SUBSTEPS):
                    mujoco.mj_step(self.m, self.d)
                self.ticks += 1
                if self.ticks % 10 == 0:
                    self.referee(check_place=False)
            if not self.collapsed:
                P = self.table_frame(self.d.xpos[self.ids])
                for i in range(self.n):
                    if self.state[i] == "out":
                        self._try_place(i, P)
                self.referee(check_place=False)
        counted = [i for i in range(self.n) if self.state[i] == "tower" and self.placed[i] and self.legal[i]]
        placed_any = [i for i in range(self.n) if self.state[i] == "tower" and self.placed[i]]
        ncap = CONFIG["score"]["n_cap"]
        score = 0.0 if self.collapsed else min(1.0, len(counted) / float(ncap))
        top = max(self.level[j] for j in range(self.n) if self.state[j] == "tower")
        self._tally = {"tower_score": score, "relocated": len(counted), "placed_total": len(placed_any),
                       "collapsed": bool(self.collapsed), "collapse": self.collapse_info,
                       "removals": self.stats["removals"], "illegal_removals": self.stats["illegal_removals"],
                       "top_level": int(top), "n_cap": ncap}
        return self._tally

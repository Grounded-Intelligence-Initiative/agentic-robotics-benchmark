"""Phase-A prototype cell: Panda (Menagerie) + pusher rod + wrist F/T + block tower + staging plate.

Control: 10 ms ticks (10 physics steps of 1 ms). Cartesian goals for the `tcp` (pad centre) or `rod_tip` site, hand
pointing straight down with a yaw; linear interpolation at a capped speed; damped-least-squares IK onto joint position
servos (Menagerie gains). Gripper: position servo when opening, constant squeeze force when closing.
"""
import math
import os

import mujoco
import numpy as np

import tower as T

TICK = 0.01
SUB = 10
ROD_LEN = 0.12          # rod from hand x = 0.02 to 0.14
ROD_X0 = 0.02
ROD_DZ = 0.014          # rod axis this much above the tcp
TCP_Z = 0.1034          # pad centre below the hand origin
GRIP_FORCE = 20.0       # N squeeze (tendon force)
STAGE_XY = np.array([0.42, 0.28])
STAGE_TOP = 0.115
STAGE_YAW = 0.0         # plate axes: block placed along world x
ARM_J = ["joint%d" % i for i in range(1, 8)]
Q_HOME = np.array([0.0, -0.5, 0.0, -2.3, 0.0, 1.8, 0.785])


class MoveFailed(Exception):
    pass


def topdown_mat(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, s, 0.0], [s, -c, 0.0], [0.0, 0.0, -1.0]])      # columns: x=(c,s,0), y=(s,-c,0), z=(0,0,-1)


def hand_mat(yaw, pitch=0.0):
    """Hand pointing down, yawed; then tilted by `pitch` about its own y axis (positive: fingertips toward -x_hand)."""
    R0 = topdown_mat(yaw)
    c, s = math.cos(-pitch), math.sin(-pitch)
    Ry = np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])
    return R0 @ Ry


def mat_quat(R):
    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, np.ascontiguousarray(R).ravel())
    return q


def build(tower, panda_dir, phys=T.PHYS):
    spec = T.build_spec(tower, panda_dir=panda_dir, phys=phys)
    spec.option.enableflags |= mujoco.mjtEnableBit.mjENBL_SLEEP
    hand = spec.body("hand")
    hand.add_site(name="tcp", pos=[0, 0, TCP_Z], size=[0.002, 0, 0], group=4)
    hand.add_site(name="ft", pos=[0, 0, 0], size=[0.002, 0, 0], group=4)
    hand.add_site(name="rod_tip", pos=[ROD_X0 + ROD_LEN, 0, TCP_Z - ROD_DZ], size=[0.002, 0, 0], group=4)
    rod_mat = dict(rgba=[0.15, 0.15, 0.17, 1], friction=[0.2, 0.002, 0.0001], solref=[0.002, 1.0],
                   solimp=[0.99, 0.999, 0.0002, 0.5, 2.0])
    # post from the palm bottom (z 0.066) down to the rod, then the rod along +x
    zr = TCP_Z - ROD_DZ
    hand.add_geom(name="rod_post", type=mujoco.mjtGeom.mjGEOM_BOX, pos=[ROD_X0 + 0.004, 0, (0.066 + zr) / 2],
                  size=[0.004, 0.006, (zr - 0.066) / 2 + 0.003], **rod_mat)
    hand.add_geom(name="rod", type=mujoco.mjtGeom.mjGEOM_CAPSULE, fromto=[ROD_X0, 0, zr, ROD_X0 + ROD_LEN - 0.003, 0, zr],
                  size=[0.003, 0, 0], **rod_mat)
    spec.add_sensor(name="ft_force", type=mujoco.mjtSensor.mjSENS_FORCE, objtype=mujoco.mjtObj.mjOBJ_SITE, objname="ft")
    spec.add_sensor(name="ft_torque", type=mujoco.mjtSensor.mjSENS_TORQUE, objtype=mujoco.mjtObj.mjOBJ_SITE, objname="ft")
    wb = spec.worldbody
    # staging plate with a fence on its -y and -x edges
    sx, sy = STAGE_XY
    wb.add_geom(name="stage", type=mujoco.mjtGeom.mjGEOM_BOX, pos=[sx, sy, STAGE_TOP / 2], size=[0.015, 0.012, STAGE_TOP / 2],
                rgba=[0.25, 0.35, 0.3, 1], friction=[0.6, 0.002, 0.0001], margin=T.MARGIN)
    # the fingers: high-friction pads
    for g in spec.geoms:
        pass
    return spec


class Cell(object):
    def __init__(self, tower, panda_dir, settle_s=2.0):
        self.tower = tower
        self.spec = build(tower, panda_dir)
        self.m = self.spec.compile()
        m = self.m
        self.d = mujoco.MjData(m)
        self.kd = mujoco.MjData(m)
        d = self.d
        self.qadr = np.array([m.jnt_qposadr[m.joint(j).id] for j in ARM_J])
        self.dadr = np.array([m.jnt_dofadr[m.joint(j).id] for j in ARM_J])
        self.lo = m.jnt_range[[m.joint(j).id for j in ARM_J], 0] + 0.02
        self.hi = m.jnt_range[[m.joint(j).id for j in ARM_J], 1] - 0.02
        self.fadr = [m.jnt_qposadr[m.joint(j).id] for j in ("finger_joint1", "finger_joint2")]
        self.site = {n: m.site(n).id for n in ("tcp", "rod_tip", "ft")}
        self.ids = T.block_ids(m, len(tower["blocks"]))
        self.idset = set(self.ids)
        self.robot_bodies = set(m.body(n).id for n in ["link%d" % i for i in range(8)] + ["hand", "left_finger", "right_finger"])
        self.finger_bodies = (m.body("left_finger").id, m.body("right_finger").id)
        self.tadr = m.jnt_qposadr[m.joint("turntable_j").id]
        self.tact = m.actuator("turntable").id
        self.table_cmd = 0.0
        self.rod_geoms = [m.geom("rod_post").id, m.geom("rod").id]
        self.sens_f = m.sensor_adr[m.sensor("ft_force").id]
        self.sens_t = m.sensor_adr[m.sensor("ft_torque").id]
        d.qpos[self.qadr] = Q_HOME
        d.ctrl[:7] = Q_HOME
        self.q_cmd = Q_HOME.copy()
        self.q_nom = Q_HOME.copy()
        m.actuator_forcelimited[:7] = 0
        self.grip_open(0.08)
        self.pusher(False)
        d.qpos[self.fadr] = 0.04
        mujoco.mj_forward(m, d)
        self.t = 0.0
        T.settle(m, d, settle_s)
        self.t = 0.0
        self.p_settled = d.xpos[self.ids].copy()
        self.q_settled = d.xquat[self.ids].copy()
        self.max_contact = 0.0

    # ------------------------------------------------------------------ gripper
    def pusher(self, on):
        """Deploy (True) or stow (False) the pusher rod: stowed, it neither collides nor shows."""
        m = self.m
        for g in self.rod_geoms:
            m.geom_contype[g] = 1 if on else 0
            m.geom_conaffinity[g] = 1 if on else 0
            m.geom_rgba[g, 3] = 1.0 if on else 0.0
        b = m.geom_bodyid[self.rod_geoms[0]]
        m.body_contype[b] = 1
        m.body_conaffinity[b] = 1
        self.pusher_on = bool(on)

    TABLE_W = 0.6           # rad/s
    TABLE_A = 0.6           # rad/s^2

    def table_angle(self):
        return float(self.d.qpos[self.tadr])

    def rotate_table(self, angle):
        """Turn the turntable to `angle` (rad, absolute) with a trapezoidal profile; blocks until settled."""
        a0 = self.table_cmd
        dist = angle - a0
        if abs(dist) < 1e-9:
            return
        sgn = 1.0 if dist > 0 else -1.0
        v = 0.0
        x = 0.0
        D = abs(dist)
        while x < D - 1e-9:
            v = min(v + self.TABLE_A * TICK, self.TABLE_W, math.sqrt(max(0.0, 2 * self.TABLE_A * (D - x))) + 1e-3)
            x = min(D, x + v * TICK)
            self.table_cmd = a0 + sgn * x
            self.tick()
        self.wait(0.5)

    def grip_open(self, width=0.08):
        m = self.m
        m.actuator_gainprm[7, 0] = 0.01568627451
        m.actuator_biasprm[7, :3] = [0, -100, -10]
        self.d.ctrl[7] = 255.0 * min(width, 0.08) / 0.08

    def grip_close(self, force=GRIP_FORCE):
        m = self.m
        m.actuator_gainprm[7, 0] = 1.0
        m.actuator_biasprm[7, :3] = [0, 0, -10]
        m.actuator_ctrlrange[7] = [-200, 255]
        self.d.ctrl[7] = -force

    def opening(self):
        return float(self.d.qpos[self.fadr[0]] + self.d.qpos[self.fadr[1]])

    # ------------------------------------------------------------------ kinematics
    def site_pose(self, name, d=None):
        d = d or self.d
        s = self.site[name]
        return d.site_xpos[s].copy(), d.site_xmat[s].reshape(3, 3).copy()

    def ik(self, frame, pos, R_t, q0, iters=3):
        m, kd = self.m, self.kd
        s = self.site[frame]
        q = q0.copy()
        for _ in range(iters):
            kd.qpos[self.qadr] = q
            mujoco.mj_kinematics(m, kd)
            mujoco.mj_comPos(m, kd)
            p = kd.site_xpos[s]
            R = kd.site_xmat[s].reshape(3, 3)
            ep = pos - p
            er = 0.5 * (np.cross(R[:, 0], R_t[:, 0]) + np.cross(R[:, 1], R_t[:, 1]) + np.cross(R[:, 2], R_t[:, 2]))
            jp = np.zeros((3, m.nv))
            jr = np.zeros((3, m.nv))
            mujoco.mj_jacSite(m, kd, jp, jr, s)
            J = np.vstack([jp[:, self.dadr], jr[:, self.dadr]])
            e = np.concatenate([ep, er])
            JJt = J @ J.T + 1e-4 * np.eye(6)
            dq = J.T @ np.linalg.solve(JJt, e)
            Jp = J.T @ np.linalg.solve(JJt, np.eye(6))
            dq += (np.eye(7) - Jp @ J) @ (0.05 * (self.q_nom - q) + self._avoid(kd))
            q = np.clip(q + dq, self.lo, self.hi)
        return q, float(np.linalg.norm(ep)), float(np.linalg.norm(er))

    AVOID_BODIES = ("link3", "link4", "link5", "link6")
    AVOID_R = 0.20
    AVOID_GAIN = 0.5

    def _avoid(self, kd):
        """Null-space push of the elbow/forearm bodies away from the tower's vertical axis (horizontally)."""
        m = self.m
        out = np.zeros(7)
        if not getattr(self, "avoid_on", False):
            return out
        jp = np.zeros((3, m.nv))
        for nm in self.AVOID_BODIES:
            b = m.body(nm).id
            p = kd.xpos[b]
            dxy = p[:2] - T.TOWER_XY
            r = float(np.linalg.norm(dxy))
            if r >= self.AVOID_R or r < 1e-6:
                continue
            u = np.array([dxy[0] / r, dxy[1] / r, 0.0])
            mujoco.mj_jacBody(m, kd, jp, None, b)
            out += self.AVOID_GAIN * (self.AVOID_R - r) * (jp[:, self.dadr].T @ u)
        return out

    # ------------------------------------------------------------------ stepping
    def tick(self):
        self.d.ctrl[:7] = self.q_cmd
        self.d.ctrl[self.tact] = self.table_cmd
        for _ in range(SUB):
            mujoco.mj_step(self.m, self.d)
        self.t += TICK
        self._contacts()

    def _contacts(self):
        m, d = self.m, self.d
        worst = 0.0
        f6 = np.zeros(6)
        for k in range(d.ncon):
            c = d.contact[k]
            if c.efc_address < 0:
                continue
            b1, b2 = m.geom_bodyid[c.geom1], m.geom_bodyid[c.geom2]
            r1, r2 = b1 in self.robot_bodies, b2 in self.robot_bodies
            if r1 == r2:
                continue
            mujoco.mj_contactForce(m, d, k, f6)
            worst = max(worst, abs(f6[0]))
        self.contact_now = worst
        self.max_contact = max(self.max_contact, worst)

    def wrench(self):
        """Force on the hand subtree (world frame), torque: raw sensor values rotated to world."""
        _, R = self.site_pose("ft")
        f = self.d.sensordata[self.sens_f:self.sens_f + 3]
        return R @ f

    def move(self, frame, pos, yaw, speed=0.15, max_t=20.0, stop=None, rot_speed=1.0):
        """Straight-line move of `frame` to pos with orientation slerped to the goal (yaw: hand-down yaw, or a
        rotation matrix). stop(cell)->bool aborts. Raises MoveFailed when the goal is not reached in max_t."""
        pos = np.asarray(pos, float)
        R_t = topdown_mat(yaw) if np.isscalar(yaw) else np.asarray(yaw, float)
        q_t = mat_quat(R_t)
        kd = self.kd
        kd.qpos[self.qadr] = self.q_cmd
        mujoco.mj_kinematics(self.m, kd)
        ref = kd.site_xpos[self.site[frame]].copy()
        q_ref = mat_quat(kd.site_xmat[self.site[frame]].reshape(3, 3))
        if np.dot(q_ref, q_t) < 0:
            q_t = -q_t
        n_max = int(max_t / TICK)
        ep = er = 0.0
        for i in range(n_max + 1):
            if i == n_max:
                if ep < 1.5e-3 and er < 1e-2:
                    break           # close enough: the joints cannot null the last fraction of a mm here
                raise MoveFailed("move of %s to %s did not converge (ik err %.1f mm / %.3f rad)" % (frame, np.round(pos, 3), ep * 1000, er))
            dp = pos - ref
            dist = float(np.linalg.norm(dp))
            step = speed * TICK
            ref = pos.copy() if dist <= step else ref + dp * (step / dist)
            ang = 2 * math.acos(min(1.0, abs(float(np.dot(q_ref, q_t)))))
            if ang > 1e-6:
                f = min(1.0, rot_speed * TICK / ang)
                qn = q_ref + f * (q_t - q_ref)
                q_ref = qn / np.linalg.norm(qn)
            R_ref = np.zeros(9)
            mujoco.mju_quat2Mat(R_ref, q_ref)
            q, ep, er = self.ik(frame, ref, R_ref.reshape(3, 3), self.q_cmd)
            dq = q - self.q_cmd
            cap = 1.5 * TICK
            mx = float(np.max(np.abs(dq)))
            if mx > cap:
                dq *= cap / mx
            self.q_cmd = self.q_cmd + dq
            self.tick()
            if stop is not None and stop(self):
                return "stopped"
            if dist <= step and ang < 1e-3 and ep < 3e-4 and er < 3e-3:
                break
        # let the servos converge
        for _ in range(15):
            q, ep, er = self.ik(frame, pos, R_t, self.q_cmd)
            self.q_cmd = q
            self.tick()
            if stop is not None and stop(self):
                return "stopped"
        self.ik_err = (ep, er)
        p, _ = self.site_pose(frame)
        return float(np.linalg.norm(p - pos))

    def solve(self, frame, pos, R, seed, iters=80):
        q = np.asarray(seed, float).copy()
        old = self.q_nom
        self.q_nom = q.copy()
        try:
            for _ in range(iters):
                q, ep, er = self.ik(frame, np.asarray(pos, float), R, q, iters=3)
        finally:
            self.q_nom = old
        return q, ep, er

    def reconfigure(self, frame, pos, yaw, q7, speed=1.0):
        """Joint-space move to an IK solution of (pos, yaw) whose joint 7 starts near q7; keeps that posture
        as the null-space target afterwards."""
        R = topdown_mat(yaw) if np.isscalar(yaw) else np.asarray(yaw, float)
        seed = self.q_cmd.copy()
        seed[6] = q7
        q, ep, er = self.solve(frame, pos, R, seed)
        if ep > 1e-3 or er > 1e-2:
            raise MoveFailed("reconfigure: no solution near q7=%.2f (%.1f mm / %.3f)" % (q7, ep * 1000, er))
        self.pose_q(q, speed=speed)
        self.q_nom = q.copy()
        return q

    def wait(self, s):
        for _ in range(int(round(s / TICK))):
            self.tick()

    def pose_q(self, q, speed=1.0):
        q = np.asarray(q, float)
        while True:
            dq = q - self.q_cmd
            mx = float(np.max(np.abs(dq)))
            if mx < 1e-6:
                break
            self.q_cmd = self.q_cmd + dq * min(1.0, speed * TICK / mx)
            self.tick()
        self.wait(0.2)

    # ------------------------------------------------------------------ truth (privileged)
    def block_pose(self, i):
        b = self.ids[i]
        return self.d.xpos[b].copy(), self.d.xmat[b].reshape(3, 3).copy()

    def disturbance(self, exclude=()):
        p = self.d.xpos[self.ids] - self.p_settled
        mask = np.ones(len(self.ids), bool)
        mask[list(exclude)] = False
        return float(np.linalg.norm(p[mask], axis=1).max()) if mask.any() else 0.0

"""The program's own kinematic model of the two arms: the given Panda MJCF (~/given/panda/panda.xml) placed twice at the
bases describe() gives, a tcp site 0.1034 m along each hand's +z (cell_manual), and one keep-out slab standing in for the
engine's front (the drive, its tensioner lever and the peg reach at most 0.10 m out of the face). Used for forward
kinematics, inverse kinematics, the arm-arm clearance and the arm-slab clearance of planned joint transits.
Public inputs only: describe(), the given Panda model, the estimated engine pose."""
import math
import os

import mujoco
import numpy as np

TCP_Z = 0.1034
SIDES = {"L": "left", "R": "right"}
# the keep-out slab in the engine frame E (x, z, d): the drive's extent and its depth out of the face
SLAB_E = {"x": (-0.42, 0.34), "z": (-0.13, 0.40), "d": (-0.05, 0.095)}


def quat_to_mat(q):
    m = np.zeros(9)
    mujoco.mju_quat2Mat(m, np.asarray(q, float))
    return m.reshape(3, 3)


def mat_to_quat(R):
    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, np.asarray(R, float).reshape(9))
    return q


def rot_axis(axis, ang):
    a = np.asarray(axis, float)
    a = a / np.linalg.norm(a)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(ang) * K + (1 - math.cos(ang)) * K @ K


def hand_R(approach_w, closing_w):
    z = np.asarray(approach_w, float)
    z = z / np.linalg.norm(z)
    y = np.asarray(closing_w, float)
    y = y - (y @ z) * z
    y = y / np.linalg.norm(y)
    return np.column_stack([np.cross(y, z), y, z])


def rotvec(Rm):
    """axis * angle of the rotation matrix Rm"""
    c = max(-1.0, min(1.0, 0.5 * (float(np.trace(Rm)) - 1.0)))
    ang = math.acos(c)
    if ang < 1e-9:
        return np.zeros(3)
    v = np.array([Rm[2, 1] - Rm[1, 2], Rm[0, 2] - Rm[2, 0], Rm[1, 0] - Rm[0, 1]])
    n = np.linalg.norm(v)
    if n < 1e-6:
        M = 0.5 * (Rm + np.eye(3))
        v = M[:, int(np.argmax(np.diag(M)))]
        n = np.linalg.norm(v)
    return v / n * ang


class Kin(object):
    def __init__(self, info, panda_dir):
        spec = mujoco.MjSpec()
        spec.compiler.degree = False
        spec.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
        arms = info["robots"]["arms"]
        self.home = {}
        for side, name in SIDES.items():
            child = mujoco.MjSpec.from_file(os.path.join(panda_dir, "panda.xml"))
            for k in list(child.keys):
                child.delete(k)
            for a in list(child.actuators):
                child.delete(a)
            for t in list(child.tendons):
                child.delete(t)
            for e in list(child.equalities):
                child.delete(e)
            child.body("hand").add_site(name="tcp", pos=[0.0, 0.0, TCP_Z])
            fr = spec.worldbody.add_frame(pos=list(arms[name]["base_pos"]))
            fr.attach_body(child.body("link0"), side + "/", "")
            self.home[side] = np.asarray(arms[name]["home_q"], float)
        sl = spec.worldbody.add_body(name="slab", mocap=True)
        hx = 0.5 * (SLAB_E["x"][1] - SLAB_E["x"][0])
        hz = 0.5 * (SLAB_E["z"][1] - SLAB_E["z"][0])
        hd = 0.5 * (SLAB_E["d"][1] - SLAB_E["d"][0])
        sl.add_geom(name="slab", type=mujoco.mjtGeom.mjGEOM_BOX, size=[hx, hd, hz], group=5)
        self.m = spec.compile()
        self.d = mujoco.MjData(self.m)
        m = self.m
        self.jadr = {s: np.array([m.jnt_qposadr[m.joint("%s/joint%d" % (s, i + 1)).id] for i in range(7)]) for s in SIDES}
        self.dadr = {s: np.array([m.jnt_dofadr[m.joint("%s/joint%d" % (s, i + 1)).id] for i in range(7)]) for s in SIDES}
        self.fadr = {s: [m.jnt_qposadr[m.joint("%s/finger_joint%d" % (s, i)).id] for i in (1, 2)] for s in SIDES}
        self.rng = {s: np.array([m.jnt_range[m.joint("%s/joint%d" % (s, i + 1)).id] for i in range(7)]) for s in SIDES}
        self.site = {s: m.site(s + "/tcp").id for s in SIDES}
        self.hand = {s: m.body(s + "/hand").id for s in SIDES}
        self.geoms = {}
        for s in SIDES:
            ids = [b for b in range(m.nbody) if m.body(b).name.startswith(s + "/")]
            self.geoms[s] = np.array([g for g in range(m.ngeom) if int(m.geom_bodyid[g]) in ids and int(m.geom_group[g]) == 3])
        self.slab_g = m.geom("slab").id
        self.slab_b = m.body("slab").id
        self.pts_b = {s: [m.body("%s/%s" % (s, n)).id for n in ("link3", "link4", "link5", "link6", "link7", "hand")] for s in SIDES}
        self._ft = np.zeros(6)
        self.q = {s: self.home[s].copy() for s in SIDES}
        self.width = {s: 0.08 for s in SIDES}
        self.E = None

    # ------------------------------------------------------------------ state
    def set_engine(self, E_pos, R_E):
        """the slab at the estimated engine pose (E frame: x right, z up, y into the face)"""
        self.E = (np.asarray(E_pos, float), np.asarray(R_E, float))
        cx = 0.5 * (SLAB_E["x"][0] + SLAB_E["x"][1])
        cz = 0.5 * (SLAB_E["z"][0] + SLAB_E["z"][1])
        cd = 0.5 * (SLAB_E["d"][0] + SLAB_E["d"][1])
        p = self.E[0] + self.E[1] @ np.array([cx, -cd, cz])
        mid = self.m.body_mocapid[self.slab_b]
        self.d.mocap_pos[mid] = p
        self.d.mocap_quat[mid] = mat_to_quat(self.E[1])

    def load(self, q=None, width=None):
        for s in SIDES:
            if q is not None and s in q:
                self.q[s] = np.asarray(q[s], float)
            if width is not None and s in width:
                self.width[s] = float(width[s])
        for s in SIDES:
            self.d.qpos[self.jadr[s]] = self.q[s]
            for a in self.fadr[s]:
                self.d.qpos[a] = 0.5 * self.width[s]
        mujoco.mj_kinematics(self.m, self.d)

    def fk(self, side, q):
        qq = dict(self.q)
        qq[side] = np.asarray(q, float)
        self.d.qpos[self.jadr[side]] = qq[side]
        mujoco.mj_kinematics(self.m, self.d)
        p = self.d.site_xpos[self.site[side]].copy()
        R = self.d.site_xmat[self.site[side]].reshape(3, 3).copy()
        self.d.qpos[self.jadr[side]] = self.q[side]
        return p, R

    def hand_pose(self, side, q=None):
        if q is not None:
            self.d.qpos[self.jadr[side]] = q
            mujoco.mj_kinematics(self.m, self.d)
        b = self.hand[side]
        out = self.d.xpos[b].copy(), self.d.xmat[b].reshape(3, 3).copy()
        if q is not None:
            self.d.qpos[self.jadr[side]] = self.q[side]
            mujoco.mj_kinematics(self.m, self.d)
        return out

    def link_points(self, side, q):
        self.d.qpos[self.jadr[side]] = q
        mujoco.mj_kinematics(self.m, self.d)
        P = self.d.xpos[self.pts_b[side]].copy()
        hb = self.pts_b[side][-1]
        tip = self.d.xpos[hb] + self.d.xmat[hb].reshape(3, 3)[:, 2] * 0.112
        self.d.qpos[self.jadr[side]] = self.q[side]
        return np.vstack([P, tip])

    # ------------------------------------------------------------------ clearance
    def _min_dist(self, ga, gb, cap):
        m, d = self.m, self.d
        pa, pb = d.geom_xpos[ga], d.geom_xpos[gb]
        D = np.linalg.norm(pa[:, None, :] - pb[None, :, :], axis=2) - m.geom_rbound[ga][:, None] - m.geom_rbound[gb][None, :]
        best = cap
        self.last_pair = None
        for i, j in zip(*np.nonzero(D < cap)):
            dist = mujoco.mj_geomDistance(m, d, int(ga[i]), int(gb[j]), cap, self._ft)
            if dist < best:
                best = dist
                self.last_pair = (m.body(int(m.geom_bodyid[ga[i]])).name, m.body(int(m.geom_bodyid[gb[j]])).name)
        return best

    def clearance(self, q=None, cap=0.15):
        """arm-arm clearance at q {side: q} (missing sides at the loaded state)"""
        if q:
            for s, v in q.items():
                self.d.qpos[self.jadr[s]] = v
        mujoco.mj_kinematics(self.m, self.d)
        c = self._min_dist(self.geoms["L"], self.geoms["R"], cap)
        if q:
            for s in q:
                self.d.qpos[self.jadr[s]] = self.q[s]
        return c

    def slab_clearance(self, side, q, cap=0.10):
        self.d.qpos[self.jadr[side]] = q
        mujoco.mj_kinematics(self.m, self.d)
        c = self._min_dist(self.geoms[side], np.array([self.slab_g]), cap)
        self.d.qpos[self.jadr[side]] = self.q[side]
        return c

    # ------------------------------------------------------------------ inverse kinematics
    def _ik1(self, side, pos, R, q0, q_ref, iters=150, null_gain=0.05):
        m, d = self.m, self.d
        q = np.array(q0, float)
        lo, hi = self.rng[side][:, 0] + 0.02, self.rng[side][:, 1] - 0.02
        jp = np.zeros((3, m.nv))
        jr = np.zeros((3, m.nv))
        dofs = self.dadr[side]
        sid = self.site[side]
        for it in range(iters):
            d.qpos[self.jadr[side]] = q
            mujoco.mj_kinematics(m, d)
            mujoco.mj_comPos(m, d)
            p = d.site_xpos[sid]
            Rc = d.site_xmat[sid].reshape(3, 3)
            ep = pos - p
            er = rotvec(R @ Rc.T)
            if np.linalg.norm(ep) < 2e-4 and np.linalg.norm(er) < 2e-3:
                break
            mujoco.mj_jacSite(m, d, jp, jr, sid)
            J = np.vstack([jp[:, dofs], jr[:, dofs]])
            e = np.concatenate([ep, 0.5 * er])
            lam = 0.02
            JJ = J @ J.T + lam * lam * np.eye(6)
            dq = J.T @ np.linalg.solve(JJ, e)
            Nn = np.eye(7) - J.T @ np.linalg.solve(JJ, J)
            # null space: towards q_ref, and away from any joint limit closer than 0.35 rad
            mid = 0.5 * (lo + hi)
            span = hi - lo
            push = np.where(q - lo < 0.35, 0.35 - (q - lo), 0.0) - np.where(hi - q < 0.35, 0.35 - (hi - q), 0.0)
            dq += Nn @ (null_gain * (q_ref - q) + 0.3 * push)
            nrm = np.linalg.norm(dq)
            if nrm > 0.3:
                dq *= 0.3 / nrm
            q = np.clip(q + dq, lo, hi)
        d.qpos[self.jadr[side]] = q
        mujoco.mj_kinematics(m, d)
        p = d.site_xpos[sid].copy()
        Rc = d.site_xmat[sid].reshape(3, 3).copy()
        d.qpos[self.jadr[side]] = self.q[side]
        return q, float(np.linalg.norm(pos - p)), float(np.linalg.norm(rotvec(R @ Rc.T)))

    def margin(self, side, q):
        """the smallest distance of q to its joint limits (rad)"""
        q = np.asarray(q, float)
        return float(np.min(np.minimum(q - self.rng[side][:, 0], self.rng[side][:, 1] - q)))

    def ik(self, side, pos, R, q_ref=None, seeds=6, rng_seed=0):
        """(q, pos err, rot err, ok): DLS from q_ref, then from home and perturbed seeds; the solution closest to q_ref
        among those within 1 mm / 0.5 deg"""
        pos = np.asarray(pos, float)
        q_ref = self.q[side] if q_ref is None else np.asarray(q_ref, float)
        rs = np.random.default_rng(rng_seed)
        starts = [q_ref, self.home[side]] + [q_ref + rs.normal(0.0, 0.4, 7) for _ in range(seeds)]
        best = None
        for q0 in starts:
            q, ep, er = self._ik1(side, pos, R, q0, q_ref)
            ok = ep < 1e-3 and er < math.radians(0.5)
            mg = self.margin(side, q)
            key = (0 if ok else 1, 0 if mg >= 0.15 else 1, float(np.linalg.norm(q - q_ref)) if ok else ep + 0.1 * er)
            if best is None or key < best[0]:
                best = (key, q, ep, er, ok)
            if ok and mg >= 0.15 and np.linalg.norm(q - q_ref) < 0.5:
                break
        return best[1], best[2], best[3], best[4]

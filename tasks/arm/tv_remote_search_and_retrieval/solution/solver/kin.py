"""Kinematics for the reference controller: forward kinematics and damped least-squares inverse
kinematics of the rail-mounted Panda, on a private copy of the room's model."""

import math

import mujoco
import numpy as np

import remote_env as E


def rot_from_axes(z_axis, x_axis):
    """Rotation whose z column is `z_axis` (the hand's approach) and whose x column is `x_axis`
    made orthogonal to it (the fingers' closing direction)."""
    z = np.asarray(z_axis, float)
    z = z / np.linalg.norm(z)
    x = np.asarray(x_axis, float)
    x = x - z * (x @ z)
    x = x / np.linalg.norm(x)
    return np.column_stack([x, np.cross(z, x), z])


class Kin(object):
    def __init__(self, model=None):
        self.model = model if model is not None else mujoco.MjModel.from_xml_string(E.build_xml())
        self.data = mujoco.MjData(self.model)
        m = self.model
        J = mujoco.mjtObj.mjOBJ_JOINT
        names = ["rail"] + ["joint{}".format(i) for i in range(1, 8)]
        self.qadr = [m.jnt_qposadr[mujoco.mj_name2id(m, J, n)] for n in names]
        self.dadr = [m.jnt_dofadr[mujoco.mj_name2id(m, J, n)] for n in names]
        self.site = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, "grip_site")
        self.cam = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_CAMERA, "wrist")
        self.lo = np.array([E.RAIL_RANGE[0]] + [l for l, _ in E.ARM_LIMITS])
        self.hi = np.array([E.RAIL_RANGE[1]] + [h for _, h in E.ARM_LIMITS])
        self.home = np.array((E.RAIL_HOME,) + E.ARM_HOME)
        B = mujoco.mjtObj.mjOBJ_BODY
        ped = mujoco.mj_name2id(m, B, "pedestal")
        self.robot = set()
        for b in range(m.nbody):
            p = b
            while p != 0:
                if p == ped:
                    self.robot.add(b)
                    break
                p = m.body_parentid[p]
        self.finger_adr = [m.jnt_qposadr[mujoco.mj_name2id(m, J, n)] for n in ("finger_joint1", "finger_joint2")]
        self.drawer_adr = {n: m.jnt_qposadr[mujoco.mj_name2id(m, J, "drawer_" + n)] for n in E.DRAWERS}
        self.free_adr = {n: m.jnt_qposadr[mujoco.mj_name2id(m, J, n + "_free")]
                         for n in ("remote", "hanger", "pad_left", "pad_right", "hat", "box")}
        self.absent = set()
        G = mujoco.mjtObj.mjOBJ_GEOM
        self.rail_pair = {mujoco.mj_name2id(m, G, 'rail_beam'), mujoco.mj_name2id(m, G, 'pedestal')}
        self.fingers = {b for b in self.robot if 'finger' in m.body(b).name}
        for g in range(m.ngeom):
            if m.geom_bodyid[g] in self.robot:
                m.geom_margin[g] = 0.04          # so that near misses are reported too
        for name in ("remote", "hanger", "hat"):      # (the hat is in a drawer: met only there)
            self.park(name)
        self.rot = np.eye(3)
        self.shift = np.zeros(3)

    # -- where the furniture stands, and the frame the caller works in ---------------------
    def set_layout(self, sofa_pose, cabinet_pose):
        """Put the settee and the chest of this private model where the episode has them."""
        m = self.model
        B = mujoco.mjtObj.mjOBJ_BODY
        self.layout = {"sofa": (E.SOFA_PIVOT, tuple(sofa_pose)), "cabinet": (E.CAB_PIVOT, tuple(cabinet_pose))}
        for name, (pivot, pose) in self.layout.items():
            origin = (0.0, 0.0) if name == "sofa" else pivot
            (x, y), yaw = E.place(pivot, pose, origin)
            b = mujoco.mj_name2id(m, B, name)
            m.body_pos[b] = (x, y, 0.0)
            m.body_quat[b] = (math.cos(0.5 * yaw), 0.0, 0.0, math.sin(0.5 * yaw))
        half = 0.5 * E.PAD_SIZE[0]
        loose = [("pad_" + side, "sofa", (E.SOFA_X0 - E.PAD_OVERHANG + half, E.PAD_Y[side])) for side in ("left", "right")]
        loose.append(("box", "cabinet", getattr(self, "box_xy", (E.CAB_X0 + E.BOX_X, E.CAB_CY))))
        for name, piece, xy in loose:
            pivot, pose = self.layout[piece]
            (x, y), yaw = E.place(pivot, pose, xy, getattr(self, "box_yaw", 0.0) if name == "box" else 0.0)
            adr = self.free_adr[name]
            m.qpos0[adr:adr + 2] = (x, y)
            m.qpos0[adr + 3:adr + 7] = (math.cos(0.5 * yaw), 0.0, 0.0, math.sin(0.5 * yaw))
            if name not in self.absent:
                self.data.qpos[adr:adr + 7] = m.qpos0[adr:adr + 7]

    def use(self, piece):
        """Work in the nominal layout's coordinates of `piece` ("sofa" or "cabinet"): `fk`
        returns poses in them and `ik` takes poses in them. None: the world's."""
        if piece is None:
            self.rot, self.shift = np.eye(3), np.zeros(3)
            return
        pivot, pose = self.layout[piece]
        (x, y), yaw = E.place(pivot, pose, (0.0, 0.0))
        c, s = math.cos(yaw), math.sin(yaw)
        self.rot = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
        self.shift = np.array([x, y, 0.0])

    def to_world(self, points):
        return np.asarray(points, float) @ self.rot.T + self.shift

    def to_frame(self, points):
        return (np.asarray(points, float) - self.shift) @ self.rot

    def park(self, name):
        """Leave a loose thing out of the collision checks."""
        adr = self.free_adr[name]
        self.data.qpos[adr:adr + 3] = (5.0 + len(self.absent), 5.0, 5.0)
        self.absent.add(name)

    def unpark(self, name):
        adr = self.free_adr[name]
        self.data.qpos[adr:adr + 3] = self.model.qpos0[adr:adr + 3]
        self.absent.discard(name)

    def set_drawer(self, name, opening):
        self.data.qpos[self.drawer_adr[name]] = opening

    def why(self, q, opening=0.08, clearance=0.0):
        """The names of the first pair that makes `collides` true at `q` (for diagnosis)."""
        m = self.model
        pair = self._hit(q, opening, clearance)
        return None if pair is None else (m.geom(pair[0]).name, m.geom(pair[1]).name)

    def collides(self, q, opening=0.08, clearance=0.0):
        """True when the robot at `q` touches the room (as this model knows it) or itself."""
        return self._hit(q, opening, clearance) is not None

    def _hit(self, q, opening, clearance):
        m, d = self.model, self.data
        for adr, v in zip(self.qadr, q):
            d.qpos[adr] = v
        d.qpos[self.finger_adr[0]] = 0.5 * opening
        d.qpos[self.finger_adr[1]] = -0.5 * opening
        mujoco.mj_kinematics(m, d)
        mujoco.mj_comPos(m, d)
        mujoco.mj_collision(m, d)
        for i in range(d.ncon):
            c = d.contact[i]
            if c.dist > clearance:
                continue
            b1, b2 = m.geom_bodyid[c.geom1], m.geom_bodyid[c.geom2]
            if b1 in self.fingers and b2 in self.fingers:
                continue                      # closed fingers touch each other
            if {int(c.geom1), int(c.geom2)} == self.rail_pair:
                continue                      # the pedestal rides on the rail
            if (b1 in self.robot) != (b2 in self.robot):
                return int(c.geom1), int(c.geom2)
            if c.dist <= 0.0 and abs(b1 - b2) > 2:
                return int(c.geom1), int(c.geom2)        # the arm against itself
        return None

    def fk(self, q):
        """Pose of the grip site in the frame in use."""
        p, r = self._fk(q)
        return self.rot.T @ (p - self.shift), self.rot.T @ r

    def _fk(self, q):
        d = self.data
        for adr, v in zip(self.qadr, q):
            d.qpos[adr] = v
        mujoco.mj_kinematics(self.model, d)
        mujoco.mj_comPos(self.model, d)
        return d.site_xpos[self.site].copy(), d.site_xmat[self.site].reshape(3, 3).copy()

    def ik(self, pos, rot, q0, rail=None, iters=200, posture=None, tol=1e-4):
        """Joint vector (rail + 7) that puts the grip site at (`pos`, `rot`), starting from `q0`.
        With `rail` given the rail stays there; otherwise it moves like a joint. Returns
        (q, position error, rotation error)."""
        m, d = self.model, self.data
        pos = self.rot @ np.asarray(pos, float) + self.shift
        if rot is not None:
            rot = self.rot @ np.asarray(rot, float)
        q = np.array(q0, float)
        if rail is not None:
            q[0] = rail
        posture = self.home if posture is None else np.asarray(posture, float)
        free = np.arange(8) if rail is None else np.arange(1, 8)
        jacp = np.zeros((3, m.nv))
        jacr = np.zeros((3, m.nv))
        cols = [self.dadr[i] for i in free]
        ep = er = 1.0
        for _ in range(iters):
            p, r = self._fk(q)
            if rot is None:
                rot = r.copy()
            dp = np.asarray(pos) - p
            dr_mat = np.asarray(rot) @ r.T
            quat = np.empty(4)
            mujoco.mju_mat2Quat(quat, dr_mat.reshape(-1))
            dr = np.empty(3)
            mujoco.mju_quat2Vel(dr, quat, 1.0)
            ep, er = np.linalg.norm(dp), np.linalg.norm(dr)
            if ep < tol and er < 1e-3:
                break
            mujoco.mj_jacSite(m, d, jacp, jacr, self.site)
            jac = np.vstack([jacp[:, cols], jacr[:, cols]])
            err = np.concatenate([dp, dr])
            lam = 1e-3
            jj = jac @ jac.T + lam * np.eye(6)
            dq = jac.T @ np.linalg.solve(jj, err)
            null = np.eye(len(free)) - jac.T @ np.linalg.solve(jj, jac)
            dq += null @ (0.05 * (posture[free] - q[free]))
            step = np.clip(dq, -0.2, 0.2)
            q[free] = np.clip(q[free] + step, self.lo[free] + 0.01, self.hi[free] - 0.01)
        return q, ep, er

    def camera_pose(self, q):
        self._fk(q)
        d = self.data
        pose = np.eye(4)
        pose[:3, :3] = d.cam_xmat[self.cam].reshape(3, 3) @ np.diag([1.0, -1.0, -1.0])
        pose[:3, 3] = d.cam_xpos[self.cam]
        return pose

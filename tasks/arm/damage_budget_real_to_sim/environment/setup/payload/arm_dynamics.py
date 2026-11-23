"""Public Cartesian arm and passive-pitch vacuum interface; reads MJCF parameters."""
import math

import mujoco
import numpy as np

HOME = (-1.9, -1.7, 1.)
LOWER = (-2.6, -2.6, .17)
UPPER = (2.6, 2.6, 1.5)
STEP = .14
SUBSTEPS = 20
ACTION_EPS = 1e-12


def bounded_action(action):
    """Validate one wire action; snap only floating-point spill at the endpoints."""
    if not isinstance(action, list) or len(action) != 4:
        raise ValueError("wrong action shape or range")
    bounded = []
    for x in action:
        if (isinstance(x, bool) or not isinstance(x, (int, float))
                or (isinstance(x, float) and not math.isfinite(x))):
            raise ValueError("finite numeric action list required")
        if abs(x) - 1 > ACTION_EPS:
            raise ValueError("wrong action shape or range")
        bounded.append(math.copysign(1., x) if abs(x) > 1 else x)
    return bounded


def parameter(model, name):
    field = model.numeric(name)
    if field.size[0] != 1 or not np.isfinite(field.data[0]) or field.data[0] <= 0:
        raise ValueError(f"expected a positive scalar MJCF numeric: {name}")
    return float(field.data[0])


def pose_row(data):
    agent, obj = data.body("agent"), data.body("object")
    axis = obj.xmat.reshape(3, 3)[:, 0]
    pitch = math.atan2(-axis[2], math.hypot(axis[0], axis[1]))
    return np.r_[agent.xpos, obj.xpos, pitch]


def drive_matrix(model):
    """Map world XY slide travel to the two signed motor coordinates in the MJCF."""
    rows = np.zeros((2, 2))
    xy = model.dof_jntid[:2].tolist()
    for i in range(2):
        if model.actuator_trntype[i] != mujoco.mjtTrn.mjTRN_TENDON:
            raise ValueError("expected two fixed-tendon position servos")
        tendon = model.actuator_trnid[i, 0]
        start, count = model.tendon_adr[tendon], model.tendon_num[tendon]
        if count != 2:
            raise ValueError("each crossed belt must span both planar slides")
        for j in range(start, start+count):
            if model.wrap_type[j] != mujoco.mjtWrap.mjWRAP_JOINT or model.wrap_objid[j] not in xy:
                raise ValueError("expected a fixed tendon on the two planar slides")
            rows[i, xy.index(model.wrap_objid[j])] = model.wrap_prm[j]*model.actuator_gear[i, 0]
    if not np.isfinite(rows).all() or abs(np.linalg.det(rows)) < 1e-8:
        raise ValueError("singular planar drive")
    return rows


class ArmDynamics:
    """MuJoCo resolves the contacts, passive hinge, and loaded weld wrench."""

    def __init__(self, model, data):
        self.model, self.data = model, data
        self.obj, self.agent, self.tool = (data.body(n) for n in ("object", "agent", "tool"))
        self.fragile_id = model.geom("fragile").id
        self.obj_dof = model.body_dofadr[self.obj.id]
        self.capacity = parameter(model, "cup_capacity_n")
        self.proof = parameter(model, "cap_proof_n")
        self.impact = parameter(model, "impact_limit_n")
        self.scuff_limit = parameter(model, "scuff_limit_n")
        self.to_motor = drive_matrix(model)
        self.to_world = np.linalg.inv(self.to_motor)
        self.command = np.array(HOME)
        data.qpos[:3] = self.command
        self.set_command(self.command)
        data.eq_active[0] = False
        self.broken = self.scuffed = False
        self.break_cause = None
        self.seal_losses = 0
        self.peak_contact = 0.
        self.peak_fixture = 0.
        self.fixture_geoms = {i for i in range(model.ngeom)
                              if model.geom(i).name.startswith(("fixture_", "tray"))}
        mujoco.mj_forward(model, data)

    @property
    def held(self):
        return bool(self.data.eq_active[0])

    def set_command(self, xyz):
        self.command = np.clip(xyz, LOWER, UPPER)
        self.data.ctrl[:2] = self.to_motor @ self.command[:2]
        self.data.ctrl[2] = self.command[2]

    def _seal(self, pressure):
        if pressure <= 0. or self.broken or self.held:
            return
        m, d = self.model, self.data
        obj_rot = self.obj.xmat.reshape(3, 3)
        tool_rot = self.tool.xmat.reshape(3, 3)
        offset = self.tool.xpos - self.obj.xpos
        x, y, z = obj_rot.T @ offset
        if (abs(x) <= .59 and abs(y) <= .087 and .109 <= z <= .150
                and tool_rot[:, 2] @ obj_rot[:, 2] >= .996
                and np.linalg.norm(d.qvel[:3] - d.qvel[self.obj_dof:self.obj_dof+3]) < .19):
            # No pose snapping: retain the current relative frame at seal time.
            patch = np.array([x, y, .08])
            m.eq_data[0, :3] = patch
            m.eq_data[0, 3:6] = tool_rot.T @ (obj_rot @ patch - offset)
            inverse = np.empty(4)
            mujoco.mju_negQuat(inverse, self.tool.xquat)
            mujoco.mju_mulQuat(m.eq_data[0, 6:10], inverse, self.obj.xquat)
            d.eq_active[0] = True

    def step(self, action):
        action = bounded_action(action)
        m, d = self.model, self.data
        delta = np.asarray(action[:3])*STEP
        delta[:2] = self.to_world @ delta[:2]
        self.set_command(self.command + delta)
        pressure = max(0., float(action[3]))
        d.qfrc_applied[:3] = (0, 0, 5.0031)
        if pressure <= 0.:
            d.eq_active[0] = False
        self._seal(pressure)
        force = np.zeros(6)
        for _ in range(SUBSTEPS):
            if self.held and pressure*self.capacity > self.proof:
                self._fracture("vacuum_overload")
            mujoco.mj_step(m, d)
            if self.held:
                rows = d.efc_force[(d.efc_type == mujoco.mjtConstraint.mjCNSTR_EQUALITY)
                                   & (d.efc_id == 0)]
                # Resolve the measured reaction in the current cup frame.
                pull = -(self.tool.xmat.reshape(3, 3).T @ rows[:3])
                if np.hypot(max(0., pull[2]), np.linalg.norm(pull[:2])/.85) > pressure*self.capacity:
                    d.eq_active[0] = False
                    self.seal_losses += 1
            if not self.broken:
                for index, contact in enumerate(d.contact):
                    if self.fragile_id not in (contact.geom1, contact.geom2):
                        continue
                    mujoco.mj_contactForce(m, d, index, force)
                    load = float(np.linalg.norm(force[:3]))
                    self.peak_contact = max(self.peak_contact, load)
                    if self.fixture_geoms.intersection((contact.geom1, contact.geom2)):
                        self.peak_fixture = max(self.peak_fixture, load)
                    if load > self.scuff_limit:
                        self.scuffed = True
                    if load > self.impact:
                        self._fracture("impact")
                        break
        mujoco.mj_forward(m, d)

    def _fracture(self, cause):
        self.broken = True
        self.break_cause = cause
        self.data.eq_active[0] = False

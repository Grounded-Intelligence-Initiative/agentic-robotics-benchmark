"""Tool-catching cell (development copy): a Wonik Allegro right hand on a servoed 6-DoF wrist catches tools tossed
to it, then regrasps them in the hand to a functional grasp.

Frames and units: world frame, z up, metres, radians, seconds, quaternions (w, x, y, z).
Hand frame H = the Allegro palm body frame: +X palm normal (the grasp side), +Y towards the index finger (and the
thumb), +Z along the fingers.
"""

import math
import os

import numpy as np

import mujoco

# ------------------------------------------------------------------------------------------------
# Difficulty knobs (documented for maintainers in tests/review/knobs.md; tune after frontier measurement)
KNOBS = dict(
    latency_steps=4,                # control steps between an observation and the step its reply takes effect
    flight_time=(0.5, 0.7),         # s, release to arrival in the catch zone
    release_dist=(1.9, 2.3),        # m, thrower's release point along +x
    spin_endover=7.0,               # rad/s, max end-over-end spin
    spin_roll=3.0,                  # rad/s, max roll about the tool axis
    scale=(0.95, 1.05),             # tool size factor
    head_mass=(0.6, 1.6),           # density factor of the non-handle parts
    friction=(0.6, 1.0),            # tool-hand sliding friction
    zone_frac=0.30,                 # fraction of the handle length the requested grip zone covers
    announce_t=2.0,                 # s after the throw starts: the grip zone is announced
    zone_rule="far",                # "far": the zone whose centre is farther from where the hand holds the tool at
                                    # the announcement (a regrasp is always needed); "random": the throw's own draw
    handle_margin=0.02,             # m, grasp-location ramp outside the requested zone
    axis_tol_deg=(20.0, 45.0),      # working-end-up: full credit below, none above
    palm_tol_deg=(30.0, 55.0),      # tool axis across the palm (along the hand's Y)
    steady_tol=(0.01, 0.03),        # m, tool displacement in the hand during the hold window: full / none
    catch_credit=0.1,               # credit of a throw that is caught and held to the end
)

HERE = os.path.dirname(os.path.abspath(__file__))
ALLEGRO_XML = os.environ.get(
    "ALLEGRO_XML", os.path.join(HERE, "..", "downloads", "mujoco_menagerie", "wonik_allegro", "right_hand.xml"))

TIMESTEP = 0.001
SUBSTEPS = 20                     # -> 50 Hz control
CONTROL_DT = TIMESTEP * SUBSTEPS
LATENCY = KNOBS["latency_steps"]                       # an action takes effect LATENCY control steps after the observation it answers
THROW_STEPS = 450                 # 9 s per throw
HOLD_STEPS = 100                  # the last 2 s: the steadiness window
GRAV = 9.81
TOOL_SOLREF = 0.004               # s, contact time constant of the tool's geoms (stiff: no sinking at impact)
FINGER_KP = 2.0                   # N m / rad
FINGER_TAU = 0.8                  # N m, joint torque limit
FINGER_DAMPING = 0.03             # N m s / rad (fast catching hand; Menagerie default 0.1)

# wrist servo limits (commanded pose is rate limited, then tracked by a stiff impedance)
V_LIN, A_LIN = 2.0, 20.0
V_ANG, A_ANG = 8.0, 80.0
WS_LO = np.array([-0.45, -0.45, 0.70])
WS_HI = np.array([0.45, 0.45, 1.45])

HOME_POS = np.array([0.0, 0.0, 1.00])
# palm up, fingers towards the thrower (+x): X_H = +z, Z_H = +x, Y_H = -y
HOME_R = np.array([[0.0, 0.0, 1.0], [0.0, -1.0, 0.0], [1.0, 0.0, 0.0]])

# ------------------------------------------------------------------------------------------------
# Tools: nominal drawings. Tool frame: origin at the butt of the handle, +x along the axis to the working end,
# +y the head's long axis for the hammer / the blade width for the spatula / the jaw opening for the wrench.
# parts: (name, type, size (MuJoCo half sizes), pos, material, density kind, is_handle)
TOOLS = {
    "hammer": dict(
        parts=[("handle", "capsule", (0.016, 0.13), (0.13, 0, 0), "wood", 700.0, True),
               ("neck", "box", (0.012, 0.011, 0.011), (0.272, 0, 0), "steel", 7800.0, False),
               ("head", "box", (0.011, 0.042, 0.011), (0.295, 0.0, 0), "steel", 7800.0, False)],
        handle=(0.03, 0.20), length=0.316),
    "screwdriver": dict(
        parts=[("handle", "capsule", (0.018, 0.055), (0.07, 0, 0), "plastic_red", 1100.0, True),
               ("shaft", "capsule", (0.004, 0.08), (0.20, 0, 0), "steel", 7800.0, False),
               ("tip", "box", (0.006, 0.006, 0.0025), (0.285, 0, 0), "steel", 7800.0, False)],
        handle=(0.03, 0.11), length=0.29),
    "wrench": dict(
        parts=[("handle", "box", (0.065, 0.014, 0.006), (0.065, 0, 0), "vinyl_blue", 2000.0, True),
               ("shaft", "box", (0.06, 0.009, 0.004), (0.19, 0, 0), "steel", 7800.0, False),
               ("jaw", "box", (0.018, 0.022, 0.005), (0.268, 0, 0), "steel", 7800.0, False)],
        handle=(0.01, 0.13), length=0.285),
    "spatula": dict(
        parts=[("handle", "capsule", (0.014, 0.08), (0.10, 0, 0), "rubber_green", 1100.0, True),
               ("neck", "box", (0.03, 0.007, 0.003), (0.21, 0, 0), "steel", 7800.0, False),
               ("blade", "box", (0.05, 0.04, 0.003), (0.29, 0, 0), "nylon", 1400.0, False)],
        handle=(0.03, 0.17), length=0.34),
}
TOOL_NAMES = tuple(TOOLS)
# The grip zone each throw asks for, announced in the observation only from ANNOUNCE_T on (after any catch):
# "end" = the butt-side part of the handle, "choke" = the working-end side part. Each covers ZONE_FRAC of the handle.
ZONES = ("end", "choke")


def zone_range(tool, zone):
    h0, h1 = TOOLS[tool]["handle"]
    w = KNOBS["zone_frac"] * (h1 - h0)
    return (h0, h0 + w) if zone == "end" else (h1 - w, h1)
MATERIALS = {"wood": (0.62, 0.45, 0.28, 1), "steel": (0.55, 0.56, 0.58, 1), "plastic_red": (0.55, 0.16, 0.14, 1),
             "plastic_black": (0.12, 0.12, 0.12, 1), "nylon": (0.78, 0.74, 0.64, 1),
             "rubber_green": (0.22, 0.42, 0.24, 1), "vinyl_blue": (0.20, 0.30, 0.58, 1)}

CAMS = {  # name: (pos, look-at, fovy); both fixed to the cell's frame and calibrated
    "side": ((1.0, -2.0, 1.40), (1.05, 0.0, 1.30), 68.0),
    "head": ((-0.60, 0.35, 1.80), (1.0, 0.0, 1.0), 62.0),
}
IMG_W, IMG_H = 192, 144


def look_at(pos, target, up=(0, 0, 1)):
    pos, target = np.asarray(pos, float), np.asarray(target, float)
    f = target - pos
    f /= np.linalg.norm(f)
    z = -f
    x = np.cross(np.array(up, float), z)
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    return np.stack([x, y, z], axis=1)


def quat_from_mat(R):
    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, np.asarray(R, float).reshape(9))
    return q


def mat_from_quat(q):
    R = np.zeros(9)
    mujoco.mju_quat2Mat(R, np.asarray(q, float))
    return R.reshape(3, 3)


def rotvec_to_mat(v):
    v = np.asarray(v, float)
    th = np.linalg.norm(v)
    if th < 1e-12:
        return np.eye(3)
    k = v / th
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(th) * K + (1 - math.cos(th)) * K @ K


def mat_to_rotvec(R):
    q = quat_from_mat(R)
    if q[0] < 0:
        q = -q
    s = np.linalg.norm(q[1:])
    if s < 1e-12:
        return np.zeros(3)
    return q[1:] / s * 2 * math.atan2(s, q[0])


# ------------------------------------------------------------------------------------------------
def sample_throw(rng, tool=None, strata=None):
    """strata: optional {"T": u, "spin": u} in [0, 1) replacing the uniform draws of the flight time and of the
    end-over-end spin (a run stratifies them; the distribution of a single throw is unchanged)."""
    strata = strata or {}
    """Hidden per-throw parameters."""
    drawn = TOOL_NAMES[int(rng.integers(len(TOOL_NAMES)))]
    tool = drawn if tool is None else tool
    scale = float(rng.uniform(*KNOBS["scale"]))
    head_mass_scale = float(rng.uniform(*KNOBS["head_mass"]))
    friction = float(rng.uniform(*KNOBS["friction"]))
    arrive = HOME_POS + np.array([rng.uniform(0.0, 0.15), rng.uniform(-0.15, 0.15), rng.uniform(0.02, 0.12)])
    release = np.array([rng.uniform(*KNOBS["release_dist"]), rng.uniform(-0.35, 0.35), rng.uniform(1.05, 1.35)])
    T = float(rng.uniform(*KNOBS["flight_time"]))
    if "T" in strata:
        lo, hi = KNOBS["flight_time"]
        T = lo + strata["T"] * (hi - lo)
    v0 = (arrive - release - 0.5 * np.array([0, 0, -GRAV]) * T * T) / T
    # tool axis roughly horizontal and across the throw direction, random which end leads
    d = (arrive - release)
    d[2] = 0
    d /= np.linalg.norm(d)
    side = np.cross([0, 0, 1], d)
    yaw = rng.uniform(-0.6, 0.6)
    ax = math.cos(yaw) * side + math.sin(yaw) * d
    if rng.random() < 0.5:
        ax = -ax
    ax = ax + np.array([0, 0, rng.uniform(-0.25, 0.25)])
    ax /= np.linalg.norm(ax)
    y0 = np.cross([0, 0, 1], ax)
    y0 /= np.linalg.norm(y0)
    roll = rng.uniform(-math.pi, math.pi)
    y = math.cos(roll) * y0 + math.sin(roll) * np.cross(ax, y0)
    R0 = np.stack([ax, y, np.cross(ax, y)], axis=1)
    # spin: mostly end-over-end about the throw-side axis, plus some roll about the tool axis
    spin_axis = np.cross(ax, [0, 0, 1])
    if np.linalg.norm(spin_axis) < 1e-6:
        spin_axis = y
    spin_axis /= np.linalg.norm(spin_axis)
    s_end = rng.uniform(-1, 1)
    if "spin" in strata:
        s_end = 2.0 * strata["spin"] - 1.0
    w = spin_axis * s_end * KNOBS["spin_endover"] + ax * rng.uniform(-1, 1) * KNOBS["spin_roll"]
    t_release = float(rng.uniform(0.3, 0.9))
    zone = ZONES[int(rng.integers(len(ZONES)))]
    return dict(zone=zone, tool=tool, scale=scale, head_mass_scale=head_mass_scale, friction=friction, release=release,
                v0=v0, R0=R0, w0=w, T=T, t_release=t_release, arrive=arrive)


def build_model(throw):
    spec = mujoco.MjSpec.from_string(f"""
<mujoco model="tool_catch">
  <option timestep="{TIMESTEP}" integrator="implicitfast" cone="elliptic" impratio="10" gravity="0 0 {-GRAV}"/>
  <visual><headlight ambient="0.32 0.32 0.32" diffuse="0.25 0.25 0.25"/><global offwidth="1280" offheight="960"/><quality shadowsize="2048"/></visual>
  <asset>
    <texture name="grid" type="2d" builtin="checker" rgb1="0.42 0.42 0.40" rgb2="0.46 0.46 0.44" width="512" height="512"/>
    <texture name="sky" type="skybox" builtin="gradient" rgb1="0.82 0.82 0.80" rgb2="0.62 0.62 0.60" width="256" height="256"/>
    <material name="floor" texture="grid" texrepeat="8 8"/>
  </asset>
  <worldbody>
    <light pos="0.5 -0.6 3.0" dir="-0.1 0.2 -1" diffuse="0.7 0.7 0.7" castshadow="true"/>
    <light pos="-1 1 2.5" dir="0.3 -0.3 -1" diffuse="0.35 0.35 0.35" castshadow="false"/>
    <light pos="2 0 2.8" dir="0 0 -1" diffuse="0.4 0.4 0.4" castshadow="false"/>
    <geom name="floor" type="plane" size="4 4 0.1" material="floor" rgba="1 1 1 1"/>
    <geom name="wall_back" type="box" pos="-1.6 0 1.5" size="0.02 4 1.5" rgba="0.72 0.72 0.70 1" contype="0" conaffinity="0"/>
    <geom name="wall_far" type="box" pos="3.4 0 1.5" size="0.02 4 1.5" rgba="0.76 0.76 0.74 1" contype="0" conaffinity="0"/>
    <geom name="wall_left" type="box" pos="1 3.0 1.5" size="4 0.02 1.5" rgba="0.74 0.74 0.72 1" contype="0" conaffinity="0"/>
    <geom name="wall_right" type="box" pos="1 -3.0 1.5" size="4 0.02 1.5" rgba="0.74 0.74 0.72 1" contype="0" conaffinity="0"/>
    <body name="wrist_target" mocap="true" pos="{HOME_POS[0]} {HOME_POS[1]} {HOME_POS[2]}"/>
    <body name="wrist" pos="{HOME_POS[0]} {HOME_POS[1]} {HOME_POS[2]}" gravcomp="1">
      <freejoint name="wrist_free"/>
      <inertial pos="0 0 -0.12" mass="1.2" diaginertia="0.004 0.004 0.002"/>
      <geom name="forearm" type="cylinder" size="0.04 0.07" pos="0 0 -0.165" rgba="0.75 0.75 0.78 1" contype="0" conaffinity="0"/>
      <site name="mount" pos="0 0 0"/>
    </body>
  </worldbody>
  <equality>
    <weld name="wrist_servo" body1="wrist_target" body2="wrist" solref="0.01 1" solimp="0.95 0.99 0.001"/>
  </equality>
</mujoco>""")
    # cameras
    for name, (pos, tgt, fovy) in CAMS.items():
        R = look_at(pos, tgt)
        spec.worldbody.add_camera(name=name, pos=list(pos), quat=list(quat_from_mat(R)), fovy=fovy)
    # the hand: palm frame = wrist frame rotated so that the palm body's +z continues the forearm
    hand = mujoco.MjSpec.from_file(ALLEGRO_XML)
    palm = hand.body("palm")
    palm.quat = [1, 0, 0, 0]
    for b in hand.bodies:
        b.gravcomp = 1.0
    mount = spec.site("mount")
    mount.attach_body(palm, "h_", "")
    ft = spec.body("h_palm").add_site(name="ft", pos=[0, 0, 0])
    spec.add_sensor(name="ft_force", type=mujoco.mjtSensor.mjSENS_FORCE, objtype=mujoco.mjtObj.mjOBJ_SITE, objname="ft")
    spec.add_sensor(name="ft_torque", type=mujoco.mjtSensor.mjSENS_TORQUE, objtype=mujoco.mjtObj.mjOBJ_SITE, objname="ft")
    # tool
    tool = TOOLS[throw["tool"]]
    s = throw["scale"]
    tb = spec.worldbody.add_body(name="tool", pos=list(throw["release"]), quat=list(quat_from_mat(throw["R0"])))
    tb.add_freejoint(name="tool_free")
    for (pname, gtype, size, pos, mat, dens, is_handle) in tool["parts"]:
        d = dens * (throw["head_mass_scale"] if not is_handle else 1.0)
        g = tb.add_geom(name=f"tool_{pname}", type=getattr(mujoco.mjtGeom, f"mjGEOM_{gtype.upper()}"),
                        size=[v * s for v in size] + [0] * (3 - len(size)), pos=[v * s for v in pos],
                        rgba=list(MATERIALS[mat]), density=d, friction=[throw["friction"], 0.01, 0.001],
                        condim=4, priority=1, solref=[TOOL_SOLREF, 1.0],
                        solimp=[0.95, 0.99, 0.001, 0.5, 2])
        if gtype in ("capsule", "cylinder"):
            # MuJoCo capsules/cylinders run along their local z: turn onto the tool's x axis
            if pname == "ring":
                g.quat = [1, 0, 0, 0]
            else:
                g.quat = list(quat_from_mat(np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]], float)))
    m = spec.compile()
    hand_dofs = [m.jnt_dofadr[m.actuator_trnid[i, 0]] for i in range(m.nu)]
    m.dof_damping[hand_dofs] = FINGER_DAMPING
    m.actuator_gainprm[:, 0] = FINGER_KP
    m.actuator_biasprm[:, 1] = -FINGER_KP
    m.actuator_forcelimited[:] = 1
    m.actuator_forcerange[:] = [-FINGER_TAU, FINGER_TAU]
    return spec, m


class Cell:
    def __init__(self, throw, render=False):
        self.throw = throw
        self.spec, self.m = build_model(throw)
        m = self.m
        self.d = mujoco.MjData(m)
        self.tool_bid = m.body("tool").id
        self.tool_jnt = m.joint("tool_free").id
        self.tool_qadr = m.jnt_qposadr[self.tool_jnt]
        self.tool_dadr = m.jnt_dofadr[self.tool_jnt]
        self.palm_bid = m.body("h_palm").id
        self.wrist_bid = m.body("wrist").id
        self.mocap = m.body("wrist_target").mocapid[0]
        self.finger_act = np.arange(m.nu)
        self.hand_bodies = set(i for i in range(m.nbody) if m.body(i).name.startswith("h_"))
        self.floor_gid = m.geom("floor").id
        d = self.d
        # wrist home
        R_wrist = HOME_R  # palm frame == wrist frame orientation
        self.set_wrist_pose(HOME_POS - HOME_R @ np.array([0, 0, -0.095]) * 0, HOME_R)
        mujoco.mj_forward(m, d)
        self.cmd_pos = d.mocap_pos[self.mocap].copy()
        self.cmd_R = mat_from_quat(d.mocap_quat[self.mocap])
        self.tgt_pos, self.tgt_R = self.cmd_pos.copy(), self.cmd_R.copy()
        self.v_lin = np.zeros(3)
        self.w_ang = np.zeros(3)
        self.finger_tgt = np.zeros(m.nu)
        self.step_i = 0
        self.released = False
        self.hold_tool()
        mujoco.mj_forward(m, d)

    def set_wrist_pose(self, pos, R):
        d, m = self.d, self.m
        q = quat_from_mat(R)
        d.mocap_pos[self.mocap] = pos
        d.mocap_quat[self.mocap] = q
        adr = m.jnt_qposadr[m.joint("wrist_free").id]
        d.qpos[adr:adr + 3] = pos
        d.qpos[adr + 3:adr + 7] = q

    def hold_tool(self):
        d = self.d
        a = self.tool_qadr
        d.qpos[a:a + 3] = self.throw["release"]
        d.qpos[a + 3:a + 7] = quat_from_mat(self.throw["R0"])
        d.qvel[self.tool_dadr:self.tool_dadr + 6] = 0

    def palm_pose(self):
        return self.d.xpos[self.palm_bid].copy(), self.d.xmat[self.palm_bid].reshape(3, 3).copy()

    def tool_com_state(self):
        d = self.d
        R = d.xmat[self.tool_bid].reshape(3, 3)
        w = R @ d.qvel[self.tool_dadr + 3:self.tool_dadr + 6]
        com = d.xipos[self.tool_bid].copy()
        v = d.qvel[self.tool_dadr:self.tool_dadr + 3] + np.cross(w, com - d.xpos[self.tool_bid])
        return com, v, R.copy(), w

    def tool_pose(self):
        return self.d.xpos[self.tool_bid].copy(), self.d.xmat[self.tool_bid].reshape(3, 3).copy()

    def command(self, pos, R, fingers):
        self.tgt_pos = np.clip(np.asarray(pos, float), WS_LO, WS_HI)
        self.tgt_R = np.asarray(R, float)
        self.finger_tgt = np.clip(np.asarray(fingers, float), self.m.actuator_ctrlrange[:, 0],
                                  self.m.actuator_ctrlrange[:, 1])

    def _servo_substep(self, dt):
        # second-order rate limiter on position
        e = self.tgt_pos - self.cmd_pos
        dist = np.linalg.norm(e)
        if dist > 1e-9:
            vdes = e / dist * min(V_LIN, math.sqrt(2 * A_LIN * dist))
        else:
            vdes = np.zeros(3)
        dv = vdes - self.v_lin
        n = np.linalg.norm(dv)
        if n > A_LIN * dt:
            dv *= A_LIN * dt / n
        self.v_lin = self.v_lin + dv
        self.cmd_pos = self.cmd_pos + self.v_lin * dt
        # orientation
        rv = mat_to_rotvec(self.tgt_R @ self.cmd_R.T)  # world-frame rotation from cmd to tgt
        ang = np.linalg.norm(rv)
        wdes = rv / ang * min(V_ANG, math.sqrt(2 * A_ANG * ang)) if ang > 1e-9 else np.zeros(3)
        dw = wdes - self.w_ang
        n = np.linalg.norm(dw)
        if n > A_ANG * dt:
            dw *= A_ANG * dt / n
        self.w_ang = self.w_ang + dw
        self.cmd_R = rotvec_to_mat(self.w_ang * dt) @ self.cmd_R
        self.d.mocap_pos[self.mocap] = self.cmd_pos
        self.d.mocap_quat[self.mocap] = quat_from_mat(self.cmd_R)

    def step(self):
        m, d = self.m, self.d
        t = self.step_i * CONTROL_DT
        d.ctrl[:] = self.finger_tgt
        for _ in range(SUBSTEPS):
            if not self.released:
                if d.time >= self.throw["t_release"]:
                    self.released = True
                    # v0 is the velocity of the centre of mass; the free joint takes that of the body origin
                    r = d.xipos[self.tool_bid] - d.xpos[self.tool_bid]
                    d.qvel[self.tool_dadr:self.tool_dadr + 3] = self.throw["v0"] - np.cross(self.throw["w0"], r)
                    d.qvel[self.tool_dadr + 3:self.tool_dadr + 6] = self.throw["R0"].T @ self.throw["w0"]
                else:
                    self.hold_tool()
            self._servo_substep(TIMESTEP)
            mujoco.mj_step(m, d)
        self.step_i += 1

    def contacts(self):
        """(hand body id, tool geom name, force normal) for every hand-tool contact."""
        m, d = self.m, self.d
        out = []
        f6 = np.zeros(6)
        for i in range(d.ncon):
            c = d.contact[i]
            b1, b2 = m.geom_bodyid[c.geom1], m.geom_bodyid[c.geom2]
            if b1 == self.tool_bid and b2 in self.hand_bodies:
                tg, hb = c.geom1, b2
            elif b2 == self.tool_bid and b1 in self.hand_bodies:
                tg, hb = c.geom2, b1
            else:
                continue
            mujoco.mj_contactForce(m, d, i, f6)
            out.append((hb, m.geom(tg).name, f6[0], c.pos.copy()))
        return out

    def tool_on_floor(self):
        m, d = self.m, self.d
        for i in range(d.ncon):
            c = d.contact[i]
            g = (c.geom1, c.geom2)
            if self.floor_gid in g and (m.geom_bodyid[g[0]] == self.tool_bid or m.geom_bodyid[g[1]] == self.tool_bid):
                return True
        return self.tool_pose()[0][2] < 0.4


# ------------------------------------------------------------------------------------------------
# Credit of one throw
GRASP_POINT_H = np.array([0.03, 0.0, -0.01])      # the power-grasp point in the hand frame


def ramp(x, full, zero):
    """1 at or below `full`, 0 at or above `zero`, linear between."""
    if x <= full:
        return 1.0
    if x >= zero:
        return 0.0
    return (zero - x) / (zero - full)


class ThrowJudge:
    """Records the hold window and scores a throw."""

    def __init__(self, cell):
        self.cell = cell
        self.rel = []          # tool pose in the hand frame over the hold window
        self.ncontact_bodies = []
        self.dropped = False

    def observe(self, step_i):
        c = self.cell
        if c.tool_on_floor():
            self.dropped = True
        if step_i >= THROW_STEPS - HOLD_STEPS:
            p, R = c.palm_pose()
            tp, tR = c.tool_pose()
            self.rel.append((R.T @ (tp - p), R.T @ tR, tR[:, 0].copy(), R.copy(), p.copy()))
            self.ncontact_bodies.append(len(set(b for b, _, _, _ in c.contacts())))

    def score(self):
        c = self.cell
        tool = TOOLS[c.throw["tool"]]
        s = c.throw["scale"]
        if self.dropped or not self.rel:
            return dict(score=0.0, caught=False)
        held = np.mean([n >= 3 for n in self.ncontact_bodies])
        if held < 0.9:
            return dict(score=0.0, caught=False, held_frac=float(held))
        k = KNOBS
        rel_p = np.array([r[0] for r in self.rel])
        # grasp location along the tool (final), in nominal tool units
        p_t, R_t = self.rel[-1][0], self.rel[-1][1]
        along = float((GRASP_POINT_H - p_t) @ R_t[:, 0]) / s
        h0, h1 = zone_range(c.throw["tool"], c.throw["zone"])
        out = max(h0 - along, along - h1, 0.0)
        handle = ramp(out, 0.0, k["handle_margin"])
        up = math.degrees(math.acos(np.clip(self.rel[-1][2][2], -1, 1)))
        axis = ramp(up, *k["axis_tol_deg"])
        across = math.degrees(math.acos(np.clip(abs(R_t[1, 0]), 0, 1)))
        palm = ramp(across, *k["palm_tol_deg"])
        disp = float(np.max(np.linalg.norm(rel_p - rel_p[0], axis=1)))
        wrist = float(np.max(np.linalg.norm(np.array([r[4] for r in self.rel]) - self.rel[0][4], axis=1)))
        steady = ramp(max(disp, wrist), *k["steady_tol"])
        q = handle * axis * palm * steady
        return dict(score=k["catch_credit"] + (1 - k["catch_credit"]) * q, caught=True, along=along, handle=handle,
                    axis=axis, palm=palm, steady=steady, up_deg=up, across_deg=across, disp=disp, wrist=wrist)


# ------------------------------------------------------------------------------------------------
# The controller-facing episode: a sequence of throws, observations in, actions out.
FINGER_JOINTS = ("ffj0", "ffj1", "ffj2", "ffj3", "mfj0", "mfj1", "mfj2", "mfj3",
                 "rfj0", "rfj1", "rfj2", "rfj3", "thj0", "thj1", "thj2", "thj3")
TOUCH_BODIES = ("palm", "ff_base", "ff_proximal", "ff_medial", "ff_distal", "ff_tip",
                "mf_base", "mf_proximal", "mf_medial", "mf_distal", "mf_tip",
                "rf_base", "rf_proximal", "rf_medial", "rf_distal", "rf_tip",
                "th_base", "th_proximal", "th_medial", "th_distal", "th_tip")
N_THROWS = 12


class ActionError(ValueError):
    pass


def parse_action(a, nu=16):
    """action = {"wrist_pos": [3], "wrist_rotvec": [3], "fingers": [16]} (world frame; radians)."""
    try:
        pos = np.asarray(a["wrist_pos"], float).reshape(3)
        rv = np.asarray(a["wrist_rotvec"], float).reshape(3)
        f = np.asarray(a["fingers"], float).reshape(nu)
    except Exception as e:  # noqa: BLE001
        raise ActionError(f"malformed action: {e}") from None
    if not (np.all(np.isfinite(pos)) and np.all(np.isfinite(rv)) and np.all(np.isfinite(f))):
        raise ActionError("non-finite action")
    return pos, rotvec_to_mat(rv), f


class Episode:
    """N_THROWS throws drawn from `seed`. reset() -> obs; step(action) -> (obs, done, info)."""

    def __init__(self, seed, n_throws=N_THROWS, render=True):
        self.rng = np.random.default_rng(seed)
        # a run brings every tool equally often (when the run length allows), in random order
        if n_throws % len(TOOL_NAMES) == 0:
            order = [TOOL_NAMES[i] for i in self.rng.permutation(np.repeat(np.arange(len(TOOL_NAMES)),
                                                                           n_throws // len(TOOL_NAMES)))]
        else:
            order = [None] * n_throws
        # flight time and end-over-end spin stratified over the run (one draw per 1/n quantile bin, shuffled)
        n = n_throws
        uT = (self.rng.permutation(n) + self.rng.random(n)) / n
        uS = (self.rng.permutation(n) + self.rng.random(n)) / n
        self.throws = [sample_throw(self.rng, tool, {"T": float(uT[i]), "spin": float(uS[i])})
                       for i, tool in enumerate(order)]
        self.render_on = render
        self.results = []
        self.k = 0
        self.cell = None
        self.renderer = None

    # -- throw lifecycle
    def _start_throw(self, k):
        if self.renderer is not None:
            self.renderer.close()
            self.renderer = None
        self.cell = Cell(self.throws[k])
        self.judge = ThrowJudge(self.cell)
        m = self.cell.m
        m.light_castshadow[:] = 0
        m.vis.quality.offsamples = 0
        if self.render_on:
            self.renderer = mujoco.Renderer(m, IMG_H, IMG_W)
        hold = (self.cell.cmd_pos.copy(), self.cell.cmd_R.copy(), np.zeros(m.nu))
        self.queue = [hold] * LATENCY
        self.step_in_throw = 0
        self.announced = False
        c = self.cell
        self.act_ids = [m.jnt_qposadr[m.joint("h_" + j).id] for j in FINGER_JOINTS]
        self.act_dofs = [m.jnt_dofadr[m.joint("h_" + j).id] for j in FINGER_JOINTS]
        self.touch_ids = [m.body("h_" + b).id for b in TOUCH_BODIES]

    def _announce(self):
        """Fix the grip zone this throw asks for. With zone_rule "far" and the tool in the hand, the zone farther from
        where the hand holds it now (so a regrasp is always needed); otherwise the throw's own draw."""
        c = self.cell
        self.announced = True
        if KNOBS["zone_rule"] != "far" or c.tool_on_floor():
            return
        if len(set(b for b, _, _, _ in c.contacts())) < 2:
            return
        p, R = c.palm_pose()
        tp, tR = c.tool_pose()
        along = float((p + R @ GRASP_POINT_H - tp) @ tR[:, 0]) / c.throw["scale"]
        far = max(ZONES, key=lambda z: abs(along - 0.5 * sum(zone_range(c.throw["tool"], z))))
        c.throw["zone"] = far

    def reset(self):
        self.k = 0
        self.results = []
        self._start_throw(0)
        return self.observe()

    def observe(self):
        c = self.cell
        m, d = c.m, c.d
        p, R = c.palm_pose()
        t = self.step_in_throw * CONTROL_DT
        touch = np.zeros(len(TOUCH_BODIES))
        f6 = np.zeros(6)
        idx = {b: i for i, b in enumerate(self.touch_ids)}
        for i in range(d.ncon):
            con = d.contact[i]
            for g in (con.geom1, con.geom2):
                b = m.geom_bodyid[g]
                if b in idx:
                    mujoco.mj_contactForce(m, d, i, f6)
                    touch[idx[b]] += abs(f6[0])
        wv = np.zeros(6)
        mujoco.mj_objectVelocity(m, d, mujoco.mjtObj.mjOBJ_BODY, c.palm_bid, wv, 0)
        obs = {
            "throw": self.k, "step": self.step_in_throw, "t": round(t, 4), "tool": c.throw["tool"],
            "zone": c.throw["zone"] if self.announced else None,
            "wrist_pos": p.tolist(), "wrist_quat": quat_from_mat(R).tolist(),
            "wrist_linvel": wv[3:].tolist(), "wrist_angvel": wv[:3].tolist(),
            "finger_q": d.qpos[self.act_ids].tolist(), "finger_qd": d.qvel[self.act_dofs].tolist(),
            "finger_tau": d.actuator_force.tolist(),
            "touch": touch.tolist(),
            "ft_force": d.sensor("ft_force").data.tolist(), "ft_torque": d.sensor("ft_torque").data.tolist(),
        }
        if self.renderer is not None:
            for name in CAMS:
                self.renderer.update_scene(d, name)
                obs["img_" + name] = self.renderer.render().copy()
        return obs

    def step(self, action):
        c = self.cell
        pos, R, f = parse_action(action, c.m.nu)
        self.queue.append((pos, R, f))
        c.command(*self.queue.pop(0))
        c.step()
        self.judge.observe(self.step_in_throw)
        self.step_in_throw += 1
        if not self.announced and self.step_in_throw * CONTROL_DT >= KNOBS["announce_t"] - 1e-9:
            self._announce()
        info = {}
        if self.judge.dropped or self.step_in_throw >= THROW_STEPS:
            r = self.judge.score()
            r.update(tool=c.throw["tool"], zone=c.throw["zone"])
            self.results.append(r)
            info["throw_result"] = r
            self.k += 1
            if self.k >= len(self.throws):
                return None, True, info
            self._start_throw(self.k)
        return self.observe(), False, info

    def close(self):
        if self.renderer is not None:
            self.renderer.close()
            self.renderer = None


def render_backgrounds():
    """The calibration shots: each camera's view of the empty cell (no tool, no robot)."""
    th = sample_throw(np.random.default_rng(0))
    th["release"] = np.array([0.0, 0.0, -5.0])
    c = Cell(th)
    m, d = c.m, c.d
    m.light_castshadow[:] = 0
    m.vis.quality.offsamples = 0
    for g in range(m.ngeom):
        if m.geom_bodyid[g] in c.hand_bodies or m.geom_bodyid[g] == c.wrist_bid:
            m.geom_rgba[g, 3] = 0
    m.mat_rgba[:, 3] = 0 if False else m.mat_rgba[:, 3]
    hidden = [g for g in range(m.ngeom) if m.geom_bodyid[g] in c.hand_bodies or m.geom_bodyid[g] == c.wrist_bid]
    m.geom_matid[hidden] = -1
    r = mujoco.Renderer(m, IMG_H, IMG_W)
    out = {}
    for n in CAMS:
        r.update_scene(d, n)
        out[n] = r.render().copy()
    r.close()
    return out

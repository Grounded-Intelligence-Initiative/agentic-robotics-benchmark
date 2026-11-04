"""Keyed nut-on-peg assembly: the task's robot environment (scene, robot, controller, sensors, credit).

This file is byte-identical in your dev sim, in the lab service and in the grader. What differs
between them is only the NUT that is loaded (a nut spec, see `NUT_SPEC_KEYS`). Your dev sim ships
one practice nut (`practice_nut.json`); the lab and the grader load the hidden nuts N0-N3, whose
specs you never see.

Frames and units
----------------
World frame: the table top is the plane z = 0.80 and spans x, y in [-0.40, 0.40]. The Panda base
stands at (-0.56, 0, 0.80) facing +x. Metres, radians, seconds. Quaternions are (w, x, y, z).

Main entry points
-----------------
    env = NutEnv(nut_spec)                  # nut_spec = json.load(open("practice_nut.json"))
    obs = env.reset(seed, peg_bands)        # any int seed: layout + sensor noise; peg_bands in 1..3
    obs, done, info = env.step(action)      # 10 Hz
    env.episode_score()                     # credit of the episode so far (see `episode_score`)
    env.gt()                                # ground truth (dev use only)

Action (a list of 8 numbers):
    [dx, dy, dz, drx, dry, drz, grip, cams]
    dx..dz:   target displacement of the gripper site in the world frame (m; the vector is clipped
              to length MAX_DPOS = 0.02);
    drx..drz: a world-frame rotation vector (rad; clipped to length MAX_DROT = 0.15) applied to the
              current gripper orientation; an operational-space controller tracks the target;
    grip:     -1 = fully open (80 mm) ... +1 = fully closed; linear in between;
    cams:     integer bit mask of the cameras whose RGB image the NEXT observation carries
              (1 = left, 2 = right, 4 = wrist; 0 = none). Images cost wall time.

Scene
-----
Three pegs stand on the table. Each peg: a 50 x 50 x 10 mm collar (base), on it a 22 x 22 mm
square shaft 90 mm tall, and one key ridge on one face of the shaft (6 mm wide,
protruding 6 mm, from the collar top up to 11.5 mm above it). The pegs look alike except for 1, 2 or 3 dark bands
near the shaft top. Every episode draws the peg positions and yaws anew.

The nut: a 60 x 60 x 24 mm block with a 24 x 24 mm square through-hole (1 mm clearance per side
on the shaft) and a handle bar (50 mm long, 16 mm wide, 16 mm tall, centred 12 mm above the nut's
bottom face) along the nut's local +x axis. A keyway slot (8 mm wide, 7 mm deep) is cut into ONE
face of the hole, in the LOWER 12 mm of the nut only; from above the hole looks plain square.
The keyway's direction relative to the handle (`key_deg` in the nut spec: 0, 90, 180 or 270
degrees counter-clockwise about the nut's z axis, 0 = towards the handle) differs between nuts.
A nut slides down to the collar only when its keyway faces the peg's ridge; otherwise it comes to
rest on top of the ridge, about 11.5 mm above the collar. Every episode places the nut flat on the
table at a fresh position and yaw: the yaw is the nut's azimuth seen from the robot base
(atan2(y - 0, x + 0.56)) plus 135 degrees plus a uniform offset within +-60 degrees.

Nut spec (a dict): {"id": str, "key_deg": 0|90|180|270, "mass": kg (0.05-0.15), "friction":
sliding friction of the nut surfaces (0.6-1.0), "rgba": [r, g, b, 1]}.
"""

import json
import math
import os

import numpy as np

import mujoco

# ----------------------------------------------------------------------------------------------
# Public constants
# ----------------------------------------------------------------------------------------------
TIMESTEP = 0.002
SUBSTEPS = 50                    # -> 10 Hz control
CONTROL_DT = TIMESTEP * SUBSTEPS
MAX_STEPS = 900                  # control steps per episode (90 s simulated)
MAX_DPOS = 0.02
MAX_DROT = 0.15
TABLE_Z = 0.80
TABLE_HALF = 0.40
ROBOT_BASE = np.array([-0.56, 0.0, TABLE_Z])
WORKSPACE_LO = np.array([-0.45, -0.45, TABLE_Z + 0.002])
WORKSPACE_HI = np.array([0.45, 0.45, TABLE_Z + 0.55])

# geometry (m)
HOLE = 0.012          # hole half width
NUT_OUT = 0.030       # nut outer half width
NUT_T = 0.024         # nut thickness
SLOT_W = 0.004        # keyway half width
SLOT_X = 0.019        # keyway bottom, from the hole axis
HANDLE_LEN = 0.050
HANDLE_W = 0.008      # half width
HANDLE_Z = (0.004, 0.020)
PEG = 0.011           # shaft half width
PEG_H = 0.090
COLLAR = 0.025        # collar half width
COLLAR_H = 0.010
RIDGE_X = 0.017
RIDGE_W = 0.003
RIDGE_H = 0.0115

# layout ranges (uniform per episode)
PEG_REGION = ((-0.02, 0.15), (-0.21, 0.21))
PEG_MIN_SPACING = 0.11
NUT_REGION = ((-0.20, -0.07), (-0.20, 0.20))
NUT_PEG_MIN_DIST = 0.13
NUT_YAW_CENTER = math.radians(135.0)   # nut yaw = azimuth of the nut from the robot base + this +- spread
NUT_YAW_SPREAD = math.radians(60.0)

# credit
LIFT_HEIGHT = 0.03           # nut bottom above the table while held (stage 1)
ENGAGE_XY = 0.005            # hole axis within this distance of the target shaft axis (stage 2)
SEAT_TOL = 0.003             # nut bottom within this height of the collar top (stage 3)
SEAT_TILT = math.radians(5.0)
RELEASE_OPEN = 0.06          # finger opening (m) counted as released
RELEASE_STEPS = 10

GRIP_KP = 3000.0             # finger position gain (N/m); the finger force is capped at 20 N
RGB_NOISE = 2.0              # 8-bit sigma
WRENCH_NOISE = 0.3           # N (torque: 0.03 N m)
WRENCH_BIAS = 0.5            # N per axis, fixed per episode (torque: 0.05 N m)
INSTAB_ROBOT_QVEL = 8.0      # rad/s: an episode with faster robot joints ends as a fault
INSTAB_NUT_VEL = 5.0         # m/s
HOME_Q = np.array([0.0, -0.35, 0.0, -2.35, 0.0, 2.0, math.pi / 4])
NULLSPACE_Q = np.array([0.0, -0.3, 0.0, -2.2, 0.0, 1.9, math.pi / 4])

CAMERAS = {
    # name: (position, look-at target, fovy deg, width, height); fixed cameras never move
    "left": ((-0.30, 0.55, 1.45), (0.05, 0.0, 0.80), 50.0, 320, 240),
    "right": ((-0.30, -0.55, 1.45), (0.05, 0.0, 0.80), 50.0, 320, 240),
}
WRIST = {"fovy": 75.0, "width": 256, "height": 192}
CAMERA_NAMES = ("left", "right", "wrist")
NUT_SPEC_KEYS = ("id", "key_deg", "mass", "friction", "rgba")
OBS_KEYS = ("t", "nut", "peg_bands", "q", "dq", "ee_pos", "ee_quat", "gripper", "wrench", "T_world_wrist",
            "left", "right", "wrist")


_SCENE_OPT = mujoco.MjvOption()
_SCENE_OPT.sitegroup[:] = 0          # sites are frames, not objects: never drawn


class EnvError(Exception):
    pass


class ActionError(ValueError):
    pass


# ----------------------------------------------------------------------------------------------
# Helpers
# ----------------------------------------------------------------------------------------------
def look_at(pos, target, up=(0, 0, 1)):
    """Rotation whose columns are the MuJoCo camera axes (x right, y up, z backwards)."""
    pos, target = np.asarray(pos, float), np.asarray(target, float)
    f = target - pos
    f = f / np.linalg.norm(f)
    z = -f
    x = np.cross(np.array(up, dtype=float), z)
    x = x / np.linalg.norm(x)
    y = np.cross(z, x)
    return np.stack([x, y, z], axis=1)


def quat_from_mat(R):
    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, np.asarray(R, dtype=float).reshape(9))
    return q


def mat_from_quat(q):
    R = np.zeros(9)
    mujoco.mju_quat2Mat(R, np.asarray(q, dtype=float))
    return R.reshape(3, 3)


def axis_angle_mat(v):
    v = np.asarray(v, dtype=float)
    th = float(np.linalg.norm(v))
    if th < 1e-12:
        return np.eye(3)
    k = v / th
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(th) * K + (1 - math.cos(th)) * K @ K


def wrap_angle(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def yaw_quat(yaw):
    return np.array([math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)])


def robosuite_assets():
    """robosuite's model assets (Panda + gripper) installed in the image; robosuite is not imported."""
    import importlib.util
    spec = importlib.util.find_spec("robosuite")
    if spec is None or not spec.submodule_search_locations:
        raise EnvError("robosuite assets not installed")
    return os.path.join(list(spec.submodule_search_locations)[0], "models", "assets")


def camera_intrinsics(fovy, width, height):
    f = (height / 2.0) / math.tan(math.radians(fovy) / 2.0)
    return np.array([[f, 0, width / 2.0], [0, f, height / 2.0], [0, 0, 1.0]])


def fixed_camera_matrices(name):
    """(K, T_world_cam) of a fixed camera, OpenCV convention (x right, y down, z forward)."""
    pos, tgt, fovy, w, h = CAMERAS[name]
    R = look_at(pos, tgt) @ np.diag([1.0, -1.0, -1.0])
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = pos
    return camera_intrinsics(fovy, w, h), T


def parse_action(a):
    """Validate an action against the public format; returns (move[6], grip, render tuple)."""
    if not isinstance(a, (list, tuple)) or len(a) != 8:
        raise ActionError("action must be a list of 8 numbers")
    out = []
    for v in a:
        if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(float(v)) or abs(v) > 1e3:
            raise ActionError("action must be a list of 8 numbers")
        out.append(float(v))
    cams = out[7]
    if cams != int(cams) or not 0 <= cams <= 7:
        raise ActionError("action must be a list of 8 numbers")
    cams = int(cams)
    render = tuple(n for i, n in enumerate(CAMERA_NAMES) if cams & (1 << i))
    return np.array(out[:6]), float(np.clip(out[6], -1.0, 1.0)), render


def encode_image(img):
    """The JSON form of an image: raw RGB bytes, row-major."""
    import base64
    img = np.ascontiguousarray(img, dtype=np.uint8)
    return {"dtype": "uint8", "shape": list(img.shape), "b64": base64.b64encode(img.tobytes()).decode("ascii")}


def decode_image(d):
    import base64
    return np.frombuffer(base64.b64decode(d["b64"]), np.uint8).reshape(d["shape"])


def to_wire(obs):
    """Observation with numpy arrays -> JSON-ready dict (images via `encode_image`)."""
    out = {}
    for k, v in obs.items():
        if k in CAMERA_NAMES:
            out[k] = encode_image(v)
        elif isinstance(v, np.ndarray):
            out[k] = v.tolist()
        else:
            out[k] = v
    return out


def check_nut_spec(spec):
    if not isinstance(spec, dict) or any(k not in spec for k in NUT_SPEC_KEYS):
        raise EnvError("nut spec needs keys " + ", ".join(NUT_SPEC_KEYS))
    if int(spec["key_deg"]) not in (0, 90, 180, 270):
        raise EnvError("key_deg must be 0, 90, 180 or 270")
    return spec


# ----------------------------------------------------------------------------------------------
# Model assembly
# ----------------------------------------------------------------------------------------------
def _box(x0, x1, y0, y1, z0, z1, attrs):
    return '<geom type="box" pos="{:.6f} {:.6f} {:.6f}" size="{:.6f} {:.6f} {:.6f}" {}/>'.format(
        (x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2, (x1 - x0) / 2, (y1 - y0) / 2, (z1 - z0) / 2, attrs)


def _rot90(boxes, deg):
    c, s = round(math.cos(math.radians(deg))), round(math.sin(math.radians(deg)))
    out = []
    for (x0, x1, y0, y1, z0, z1) in boxes:
        pts = [(c * x - s * y, s * x + c * y) for x in (x0, x1) for y in (y0, y1)]
        xs, ys = [p[0] for p in pts], [p[1] for p in pts]
        out.append((min(xs), max(xs), min(ys), max(ys), z0, z1))
    return out


def nut_body_xml(spec):
    """The nut as an MJCF body (origin: hole axis on the bottom face; handle along local +x)."""
    hz = NUT_T / 2
    walls = [(-NUT_OUT, -HOLE, -NUT_OUT, NUT_OUT, 0, NUT_T),
             (-HOLE, HOLE, HOLE, NUT_OUT, 0, NUT_T),
             (-HOLE, HOLE, -NUT_OUT, -HOLE, 0, NUT_T),
             (HOLE, NUT_OUT, -NUT_OUT, NUT_OUT, hz, NUT_T),
             (SLOT_X, NUT_OUT, -NUT_OUT, NUT_OUT, 0, hz),
             (HOLE, SLOT_X, SLOT_W, NUT_OUT, 0, hz),
             (HOLE, SLOT_X, -NUT_OUT, -SLOT_W, 0, hz)]
    rgba = " ".join("{:.3f}".format(float(c)) for c in spec["rgba"])
    attrs = 'rgba="{}" condim="4" friction="{:.4f} 0.01 0.0001"'.format(rgba, float(spec["friction"]))
    g = "".join(_box(*b, attrs=attrs) for b in _rot90(walls, int(spec["key_deg"])))
    g += _box(NUT_OUT, NUT_OUT + HANDLE_LEN, -HANDLE_W, HANDLE_W, HANDLE_Z[0], HANDLE_Z[1], attrs=attrs)
    m = float(spec["mass"])
    return ('<body name="nut"><freejoint name="nut_free"/>'
            '<inertial pos="0.010 0 0.012" mass="{m:.5f}" diaginertia="{a:.3e} {b:.3e} {c:.3e}"/>{g}</body>').format(
        m=m, a=m * 3.5e-4, b=m * 6.5e-4, c=m * 9.0e-4, g=g)


def peg_body_xml(i, bands):
    shaft = 'rgba="0.62 0.64 0.67 1"'
    g = _box(-COLLAR, COLLAR, -COLLAR, COLLAR, 0, COLLAR_H, 'rgba="0.30 0.31 0.34 1"')
    g += _box(-PEG, PEG, -PEG, PEG, COLLAR_H, COLLAR_H + PEG_H, shaft)
    g += _box(PEG, RIDGE_X, -RIDGE_W, RIDGE_W, COLLAR_H, COLLAR_H + RIDGE_H, shaft)
    top = COLLAR_H + PEG_H
    e = PEG + 0.0004
    for k in range(bands):
        z = top - 0.010 - 0.013 * k
        g += _box(-e, e, -e, e, z - 0.0035, z + 0.0035,
                  'rgba="0.07 0.07 0.09 1" contype="0" conaffinity="0" group="1" mass="0"')
    return '<body name="peg{}">{}</body>'.format(i, g)


SCENE_XML = """<mujoco model="keyed_nut">
  <compiler angle="radian" autolimits="true"/>
  <option timestep="{dt}" integrator="implicitfast" cone="elliptic" impratio="10"/>
  <visual><global offwidth="320" offheight="240"/><quality shadowsize="2048"/>
    <headlight ambient="0.25 0.25 0.25" diffuse="0.45 0.45 0.45"/></visual>
  <asset>
    <texture name="floor" type="2d" builtin="checker" rgb1="0.42 0.40 0.38" rgb2="0.36 0.34 0.33" width="256" height="256"/>
    <material name="floormat" texture="floor" texrepeat="6 6"/>
    <texture name="wood" type="2d" builtin="flat" mark="random" random="0.08" rgb1="0.58 0.46 0.33" rgb2="0.5 0.4 0.3" width="512" height="512"/>
    <material name="tablemat" texture="wood" texrepeat="3 3" specular="0.1"/>
    <texture name="wall" type="2d" builtin="gradient" rgb1="0.82 0.84 0.86" rgb2="0.65 0.67 0.70" width="128" height="128"/>
    <material name="wallmat" texture="wall"/>
  </asset>
  <default><geom solref="0.004 1" solimp="0.95 0.99 0.001"/></default>
  <worldbody>
    <light pos="0.2 0.5 2.4" dir="-0.1 -0.2 -1" diffuse="0.55 0.55 0.52" castshadow="true"/>
    <light pos="-0.6 -0.6 2.2" dir="0.3 0.3 -1" diffuse="0.25 0.25 0.27" castshadow="false"/>
    <geom name="floor" type="plane" size="3 3 0.1" material="floormat"/>
    <geom name="wall_back" type="box" pos="1.3 0 1.2" size="0.02 2.5 1.2" material="wallmat" contype="0" conaffinity="0"/>
    <geom name="table" type="box" pos="0 0 {tz2}" size="{th} {th} {tz2}" material="tablemat" friction="0.7 0.005 0.0001"/>
    <geom name="pedestal" type="box" pos="{rx} 0 {tz2}" size="0.12 0.12 {tz2}" rgba="0.25 0.25 0.27 1" contype="0" conaffinity="0" group="1"/>
  </worldbody>
</mujoco>"""


def build_model(spec, layout):
    """Compile table + Panda + 3 pegs + nut for a layout {pegs: [(x, y, yaw)]*3, nut: (x, y, yaw)}."""
    root = mujoco.MjSpec.from_string(SCENE_XML.format(dt=TIMESTEP, tz2=TABLE_Z / 2, th=TABLE_HALF, rx=ROBOT_BASE[0]))
    wb = root.worldbody
    for name, (pos, tgt, fovy, _w, _h) in CAMERAS.items():
        c = wb.add_camera()
        c.name = name
        c.pos = list(pos)
        c.quat = list(quat_from_mat(look_at(pos, tgt)))
        c.fovy = fovy
    assets = robosuite_assets()
    robot = mujoco.MjSpec.from_file(os.path.join(assets, "robots", "panda", "robot.xml"))
    grip = mujoco.MjSpec.from_file(os.path.join(assets, "grippers", "panda_gripper.xml"))
    for cam in list(robot.cameras):
        if cam.name == "robotview":
            robot.delete(cam)
    robot.body("right_hand").add_frame().attach_body(grip.body("right_gripper"), "g_", "")
    wb.add_frame(pos=list(ROBOT_BASE)).attach_body(robot.body("base"), "r_", "")
    objs = mujoco.MjSpec.from_string(
        '<mujoco><compiler angle="radian" autolimits="true"/>'
        '<worldbody>{}{}</worldbody></mujoco>'.format(
            "".join(peg_body_xml(i, i + 1) for i in range(3)), nut_body_xml(spec)))
    for i, (x, y, yaw) in enumerate(layout["pegs"]):
        wb.add_frame(pos=[x, y, TABLE_Z], quat=list(yaw_quat(yaw))).attach_body(objs.body("peg%d" % i), "o_", "")
    x, y, yaw = layout["nut"]
    wb.add_frame(pos=[x, y, TABLE_Z + 0.0005], quat=list(yaw_quat(yaw))).attach_body(objs.body("nut"), "o_", "")
    for act in root.actuators:
        if act.name.startswith("r_g_gripper_finger_joint"):
            act.gainprm[0] = GRIP_KP
            act.biasprm[1] = -GRIP_KP
    m = root.compile()
    m.cam_fovy[m.camera("r_eye_in_hand").id] = WRIST["fovy"]
    return m


def sample_layout(rng):
    """Peg positions/yaws (peg i has i+1 bands) and the nut placement; rejection-sampled."""
    for _ in range(10000):
        pegs = []
        while len(pegs) < 3:
            x = rng.uniform(*PEG_REGION[0])
            y = rng.uniform(*PEG_REGION[1])
            if all(math.hypot(x - p[0], y - p[1]) >= PEG_MIN_SPACING for p in pegs):
                pegs.append((float(x), float(y), float(rng.uniform(-math.pi, math.pi))))
        nx, ny = rng.uniform(*NUT_REGION[0]), rng.uniform(*NUT_REGION[1])
        az = math.atan2(ny - ROBOT_BASE[1], nx - ROBOT_BASE[0])
        nyaw = float(wrap_angle(az + NUT_YAW_CENTER + rng.uniform(-NUT_YAW_SPREAD, NUT_YAW_SPREAD)))
        hx, hy = nx + 0.055 * math.cos(nyaw), ny + 0.055 * math.sin(nyaw)    # handle centre
        if all(math.hypot(nx - p[0], ny - p[1]) >= NUT_PEG_MIN_DIST and math.hypot(hx - p[0], hy - p[1]) >= 0.09
               for p in pegs) and abs(hy) < 0.3 and hx > -0.25:
            order = rng.permutation(3)
            return {"pegs": [pegs[i] for i in order], "nut": (float(nx), float(ny), nyaw)}
    raise EnvError("layout sampling failed")


# ----------------------------------------------------------------------------------------------
# Environment
# ----------------------------------------------------------------------------------------------
class NutEnv(object):

    def __init__(self, nut_spec, render=True, max_steps=MAX_STEPS, early_end=True):
        self.spec = check_nut_spec(nut_spec)
        self.nut_id = str(nut_spec["id"])
        self.early_end = early_end
        self.render_enabled = render
        self.max_steps = int(max_steps)
        self.model = None
        self._renderers = {}

    # ------------------------------------------------------------------ episode
    def reset(self, seed, peg_bands):
        if int(peg_bands) not in (1, 2, 3):
            raise EnvError("peg_bands must be 1, 2 or 3")
        self.peg_bands = int(peg_bands)
        self.seed = int(seed) & (2 ** 64 - 1)
        place = np.random.default_rng([self.seed & 0xFFFFFFFF, self.seed >> 32, 7])
        self.rng = np.random.default_rng([self.seed & 0xFFFFFFFF, self.seed >> 32, 11])
        self.layout = sample_layout(place)
        self.close()
        self.model = build_model(self.spec, self.layout)
        self.data = mujoco.MjData(self.model)
        self.fk = mujoco.MjData(self.model)
        self._index()
        self._init_state()
        self.wrench_bias = np.concatenate([self.rng.uniform(-WRENCH_BIAS, WRENCH_BIAS, 3),
                                           self.rng.uniform(-0.1 * WRENCH_BIAS, 0.1 * WRENCH_BIAS, 3)])
        self.t = 0
        self.done = False
        self.fault = None
        self.credit = 0.0
        self.release_count = 0
        self.render_next = CAMERA_NAMES
        return self.observation()

    def _index(self):
        m = self.model
        self.jq = np.array([m.joint("r_joint%d" % i).qposadr[0] for i in range(1, 8)])
        self.jv = np.array([m.joint("r_joint%d" % i).dofadr[0] for i in range(1, 8)])
        self.act = np.array([m.actuator("r_torq_j%d" % i).id for i in range(1, 8)])
        self.gact = [m.actuator("r_g_gripper_finger_joint1").id, m.actuator("r_g_gripper_finger_joint2").id]
        self.gq = [m.joint("r_g_finger_joint1").qposadr[0], m.joint("r_g_finger_joint2").qposadr[0]]
        self.site = m.site("r_g_grip_site").id
        self.ft_force = m.sensor("r_g_force_ee").adr[0]
        self.ft_torque = m.sensor("r_g_torque_ee").adr[0]
        self.ft_site = m.site("r_g_ft_frame").id
        self.wrist_cam = m.camera("r_eye_in_hand").id
        self.nut_body = m.body("o_nut").id
        nj = m.joint("o_nut_free")
        self.nut_qadr, self.nut_dadr = int(nj.qposadr[0]), int(nj.dofadr[0])
        self.nut_geoms = set(int(i) for i in np.where(m.geom_bodyid == self.nut_body)[0])
        fingers = set()
        for b in range(m.nbody):
            nm = m.body(b).name
            if nm.startswith("r_g_") and ("finger" in nm):
                fingers.add(b)
        self.finger_geoms = set(int(i) for i in np.where(np.isin(m.geom_bodyid, list(fingers)))[0])
        self.target_peg = self.peg_bands - 1          # peg body i carries i + 1 bands
        self.peg_poses = [tuple(p) for p in self.layout["pegs"]]
        self._jp, self._jr = np.zeros((3, m.nv)), np.zeros((3, m.nv))
        self._M = np.zeros((m.nv, m.nv))

    def _init_state(self):
        m, d = self.model, self.data
        mujoco.mj_resetData(m, d)
        d.qpos[self.jq] = HOME_Q
        d.qpos[self.gq[0]], d.qpos[self.gq[1]] = 0.04, -0.04
        mujoco.mj_forward(m, d)
        self.grip_target = 0.04
        self.q_null = NULLSPACE_Q.copy()
        q_hold = d.qpos[self.jq].copy()
        for _ in range(150):
            tau = 200.0 * (q_hold - d.qpos[self.jq]) - 20.0 * d.qvel[self.jv] + d.qfrc_bias[self.jv]
            d.ctrl[self.act] = tau
            d.ctrl[self.gact] = [0.04, -0.04]
            mujoco.mj_step(m, d)
        self.target_pos = d.site_xpos[self.site].copy()
        self.target_R = d.site_xmat[self.site].reshape(3, 3).copy()

    # ------------------------------------------------------------------ control
    def _osc_torque(self):
        m, d = self.model, self.data
        jp, jr = self._jp, self._jr
        mujoco.mj_jacSite(m, d, jp, jr, self.site)
        J = np.vstack([jp, jr])[:, self.jv]
        mujoco.mj_fullM(m, d, self._M)
        Mq = self._M[np.ix_(self.jv, self.jv)]
        R = d.site_xmat[self.site].reshape(3, 3)
        ep = self.target_pos - d.site_xpos[self.site]
        Re = self.target_R @ R.T
        eo = 0.5 * np.array([Re[2, 1] - Re[1, 2], Re[0, 2] - Re[2, 0], Re[1, 0] - Re[0, 1]])
        v = J @ d.qvel[self.jv]
        kp, ko = 400.0, 200.0
        acc = np.concatenate([kp * ep - 2.0 * math.sqrt(kp) * v[:3], ko * eo - 2.0 * math.sqrt(ko) * v[3:]])
        Minv = np.linalg.inv(Mq)
        Lam = np.linalg.inv(J @ Minv @ J.T + 1e-2 * np.eye(6))
        tau = J.T @ (Lam @ acc) + d.qfrc_bias[self.jv]
        Jbar = Minv @ J.T @ Lam
        N = np.eye(7) - J.T @ Jbar.T
        qn = d.qpos[self.jq]
        dq = d.qvel[self.jv]
        tau += N @ (5.0 * (self.q_null - qn) - 6.0 * dq)
        tau -= 0.5 * dq
        over = np.abs(dq) - 1.5
        tau -= np.where(over > 0, 30.0 * over * np.sign(dq), 0.0)
        return tau

    def step(self, action):
        """Apply one 10 Hz action. Returns (obs, done, info). Raises ActionError on a malformed action."""
        if self.done:
            raise EnvError("episode already finished")
        mv, g, render = parse_action(action)
        d = self.data
        dp = mv[:3]
        n = np.linalg.norm(dp)
        if n > MAX_DPOS:
            dp = dp * (MAX_DPOS / n)
        dr = mv[3:]
        n = np.linalg.norm(dr)
        if n > MAX_DROT:
            dr = dr * (MAX_DROT / n)
        cur_p = d.site_xpos[self.site].copy()
        cur_R = d.site_xmat[self.site].reshape(3, 3).copy()
        self.target_pos = np.clip(cur_p + dp, WORKSPACE_LO, WORKSPACE_HI)
        self.target_R = axis_angle_mat(dr) @ cur_R
        self.grip_target = 0.04 * (1.0 - g) / 2.0
        self.render_next = render
        self._physics_step()
        self.t += 1
        self._post_step()
        return self.observation(), self.done, self.info()

    def _physics_step(self):
        m, d = self.model, self.data
        gw = [self.grip_target, -self.grip_target]
        for _ in range(SUBSTEPS):
            d.ctrl[self.act] = self._osc_torque()
            d.ctrl[self.gact] = gw
            mujoco.mj_step(m, d)
        if not np.all(np.isfinite(d.qpos)) or not np.all(np.isfinite(d.qvel)):
            self.fault = "non-finite state"

    # ------------------------------------------------------------------ credit
    def nut_state(self):
        """(position of the nut origin = hole axis on its bottom face, rotation matrix)."""
        d = self.data
        return d.xpos[self.nut_body].copy(), d.xmat[self.nut_body].reshape(3, 3).copy()

    def finger_contacts(self):
        """Number of distinct finger geoms touching the nut."""
        d = self.data
        touching = set()
        for i in range(d.ncon):
            c = d.contact[i]
            g1, g2 = int(c.geom1), int(c.geom2)
            if g1 in self.nut_geoms and g2 in self.finger_geoms:
                touching.add(g2)
            elif g2 in self.nut_geoms and g1 in self.finger_geoms:
                touching.add(g1)
        return len(touching)

    def finger_bodies_touching(self):
        d, m = self.data, self.model
        bodies = set()
        for i in range(d.ncon):
            c = d.contact[i]
            g1, g2 = int(c.geom1), int(c.geom2)
            if g1 in self.nut_geoms and g2 in self.finger_geoms:
                bodies.add(int(m.geom_bodyid[g2]))
            elif g2 in self.nut_geoms and g1 in self.finger_geoms:
                bodies.add(int(m.geom_bodyid[g1]))
        return bodies

    def stage_now(self):
        """Instantaneous stage: 0 none, 1 lifted while held, 2 engaged on the target peg, 3 seated on it."""
        p, R = self.nut_state()
        px, py, pyaw = self.peg_poses[self.target_peg]
        dxy = math.hypot(p[0] - px, p[1] - py)
        tilt = math.acos(max(-1.0, min(1.0, R[2, 2])))
        top = TABLE_Z + COLLAR_H + PEG_H
        if dxy <= ENGAGE_XY and p[2] < top - 0.005 and p[2] > TABLE_Z:
            if p[2] - (TABLE_Z + COLLAR_H) <= SEAT_TOL and tilt <= SEAT_TILT:
                return 3
            return 2
        held = len(self.finger_bodies_touching()) >= 2
        if held and p[2] - TABLE_Z >= LIFT_HEIGHT:
            return 1
        return 0

    def _post_step(self):
        d = self.data
        if self.fault is None:
            if np.max(np.abs(d.qvel[self.jv])) > INSTAB_ROBOT_QVEL:
                self.fault = "robot joint speed"
            elif np.linalg.norm(d.qvel[self.nut_dadr:self.nut_dadr + 3]) > INSTAB_NUT_VEL:
                self.fault = "nut speed"
        if self.fault is not None:
            self.done = True
            return
        st = self.stage_now()
        if st >= 1:
            self.credit = max(self.credit, 0.25)
        if st >= 2:
            self.credit = max(self.credit, 0.5)
        opening = float(d.qpos[self.gq[0]] - d.qpos[self.gq[1]])
        if st == 3 and opening >= RELEASE_OPEN and self.finger_contacts() == 0:
            self.release_count += 1
        else:
            self.release_count = 0
        if self.release_count >= RELEASE_STEPS:
            self.credit = 1.0
        if (self.early_end and self.credit >= 1.0) or self.t >= self.max_steps:
            self.done = True

    def episode_score(self):
        """Credit of the episode (0 after a fault), the best stage reached:
        0.25 the nut was lifted LIFT_HEIGHT above the table while both fingers held it;
        0.5  the nut's hole was around the TARGET peg's shaft (axis within ENGAGE_XY, nut below the shaft top);
        1.0  the nut sat on the target peg's collar (within SEAT_TOL, tilt below 5 degrees) for RELEASE_STEPS
             consecutive steps with the fingers open at least RELEASE_OPEN and not touching the nut."""
        if self.fault is not None:
            return 0.0
        return float(self.credit)

    def info(self):
        return {"t": self.t, "fault": self.fault}

    # ------------------------------------------------------------------ sensing
    def _renderer(self, w, h):
        r = self._renderers.get((w, h))
        if r is None:
            r = mujoco.Renderer(self.model, h, w)
            self._renderers[(w, h)] = r
        return r

    def close(self):
        """Free the episode's renderer resources."""
        for r in self._renderers.values():
            try:
                r.close()
            except Exception:  # noqa: BLE001 - a renderer that failed to free must not stop the caller
                pass
        self._renderers = {}

    def render(self, cam):
        if cam == "wrist":
            w, h, camid = WRIST["width"], WRIST["height"], self.wrist_cam
        else:
            _p, _t, _f, w, h = CAMERAS[cam]
            camid = self.model.camera(cam).id
        r = self._renderer(w, h)
        r.update_scene(self.data, camera=camid, scene_option=_SCENE_OPT)
        rgb = r.render()
        noise = self.rng.normal(0.0, RGB_NOISE, rgb.shape)
        return np.clip(rgb.astype(np.float32) + noise, 0, 255).astype(np.uint8)

    def wrist_camera_matrices(self):
        """(K, T_world_cam) of the wrist camera now, OpenCV convention."""
        d = self.data
        R = d.cam_xmat[self.wrist_cam].reshape(3, 3) @ np.diag([1.0, -1.0, -1.0])
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = d.cam_xpos[self.wrist_cam]
        return camera_intrinsics(WRIST["fovy"], WRIST["width"], WRIST["height"]), T

    def observation(self):
        d = self.data
        R = d.site_xmat[self.site].reshape(3, 3)
        Rft = d.site_xmat[self.ft_site].reshape(3, 3)
        f = Rft @ d.sensordata[self.ft_force:self.ft_force + 3] + self.wrench_bias[:3] + self.rng.normal(0, WRENCH_NOISE, 3)
        tq = (Rft @ d.sensordata[self.ft_torque:self.ft_torque + 3] + self.wrench_bias[3:]
              + self.rng.normal(0, WRENCH_NOISE * 0.1, 3))
        obs = {"t": int(self.t),
               "nut": self.nut_id,
               "peg_bands": self.peg_bands,
               "q": np.round(d.qpos[self.jq], 6),
               "dq": np.round(d.qvel[self.jv], 6),
               "ee_pos": np.round(d.site_xpos[self.site], 6),
               "ee_quat": np.round(quat_from_mat(R), 6),
               "gripper": round(float(d.qpos[self.gq[0]] - d.qpos[self.gq[1]]), 6),
               "wrench": np.round(np.concatenate([f, tq]), 4),
               "T_world_wrist": np.round(self.wrist_camera_matrices()[1], 6)}
        empty = np.zeros((0, 0, 3), np.uint8)
        for cam in CAMERA_NAMES:
            obs[cam] = self.render(cam) if (self.render_enabled and cam in self.render_next) else empty
        return obs

    def perturbed_observation(self, obs, rng):
        """A probe copy for the grader's lock: small joint-reading offsets (up to 0.001 rad) with the
        gripper pose and wrist-camera pose recomputed consistently, and a small wrench offset."""
        q = np.asarray(obs["q"], float) + np.array([rng.uniform(-0.001, 0.001) for _ in range(7)])
        f = self.fk
        f.qpos[:] = self.data.qpos
        f.qpos[self.jq] = q
        mujoco.mj_kinematics(self.model, f)
        mujoco.mj_camlight(self.model, f)
        out = dict(obs)
        out["q"] = np.round(q, 6)
        out["ee_pos"] = np.round(f.site_xpos[self.site], 6)
        out["ee_quat"] = np.round(quat_from_mat(f.site_xmat[self.site].reshape(3, 3)), 6)
        R = f.cam_xmat[self.wrist_cam].reshape(3, 3) @ np.diag([1.0, -1.0, -1.0])
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = f.cam_xpos[self.wrist_cam]
        out["T_world_wrist"] = np.round(T, 6)
        w = np.asarray(obs["wrench"], float).copy()
        w[:3] += np.array([rng.uniform(-0.2, 0.2) for _ in range(3)])
        out["wrench"] = np.round(w, 4)
        return out

    # ------------------------------------------------------------------ ground truth (dev only)
    def gt(self):
        p, R = self.nut_state()
        return {"nut_pos": p.tolist(), "nut_quat": quat_from_mat(R).tolist(), "key_deg": int(self.spec["key_deg"]),
                "pegs": [list(pp) for pp in self.peg_poses], "target_peg": self.target_peg,
                "stage": self.stage_now(), "credit": self.credit, "t": self.t, "fault": self.fault}


def load_spec(path):
    with open(path) as fh:
        return check_nut_spec(json.load(fh))

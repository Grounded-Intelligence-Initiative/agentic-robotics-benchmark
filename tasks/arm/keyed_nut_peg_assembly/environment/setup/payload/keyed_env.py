"""Bayonet nut kit: the task's robot environment (scene, robot, controller, sensors, safety stops).

This file is byte-identical in your dev sim and in the grader. What differs between them is only the
KIT that is loaded: three nut specs (see `NUT_SPEC_KEYS`) and the work order. Your dev sim ships one
practice kit (`practice_kit.json`, `load_kit`), and `sample_kit` draws more; every graded episode loads
its own kit, drawn with `sample_kit` from a seed you never see. This file has no ground-truth accessor
and judges no assembly: the grader judges every nut by the rule the task instruction states, with its
own code, on the state this simulation reaches.

Frames and units
----------------
World frame: the table top is the plane z = 0.80 and spans x, y in [-0.40, 0.40]. The Panda base
stands at (-0.56, 0, 0.80) facing +x. Metres, radians, seconds. Quaternions are (w, x, y, z).

Main entry points
-----------------
    kit = load_kit("practice_kit.json")     # or sample_kit(numpy.random.default_rng(seed))
    env = NutEnv(kit["nuts"])               # three nut specs
    obs = env.reset(seed, kit["order"])     # any int seed: layout + sensor noise; order[i] = bands of nut i's peg
    obs, done, info = env.step(action)      # 10 Hz; info = {"t", "fault", "ruined"} (see Safety stops)

Action (a list of 8 numbers):
    [dx, dy, dz, drx, dry, drz, grip, cams]
    dx..dz:   target displacement of the gripper site in the world frame (m; the vector is clipped
              to length MAX_DPOS = 0.02);
    drx..drz: a world-frame rotation vector (rad; clipped to length MAX_DROT = 0.15) applied to the
              current gripper orientation; an operational-space controller tracks the target;
    grip:     -1 = fully open (80 mm) ... +1 = fully closed; linear in between;
    cams:     integer bit mask of the cameras whose RGB image the NEXT observation carries
              (1 = left, 2 = right, 4 = wrist; 0 = none). Images cost wall time (and, when graded,
              controller budget: see the task instruction).

Scene
-----
Three pegs stand on the table. Each peg: a 50 x 50 x 10 mm collar (base), on it a round shaft of
radius 11 mm, 65 mm tall, carrying one pin: a small block on the shaft surface (3 mm wide, 4 mm
tall, reaching 17.2 mm from the shaft axis) centred 37.5 mm above the collar top, on the peg's
local +x side. The pegs look alike except for 1, 2 or 3 dark bands near the shaft top. Every
episode draws the peg positions and yaws anew.

The nut: a ring 45 mm tall (outer radius 28 mm, a 24-sided polygon) whose hole (inscribed radius
12.7 mm) fits over the shaft, with a handle bar (16 mm wide, 16 mm tall, from 25 to 41 mm above the
nut's bottom face) reaching 98 mm from the hole axis along the nut's local +x axis. The ring is
built from 24 sectors (15 degrees each; sector s spans [15 s, 15 (s + 1)] degrees counter-clockwise
from the handle) x 9 layers (5 mm each, layer 0 at the bottom). In each cell the inner part of the
wall (radius 12.7 to 19 mm) is either solid or cut away; the cut cells form a GROOVE in the hole
wall, through which the pin must travel for the nut to come down. The top layer is always solid,
so the outside of a nut does not show its groove, but the groove is real geometry and can be seen
where the hole is open to view: looking down into the hole at a steep angle shows the far side of
the hole wall below the top layer, where every cut cell is a recess about 6 mm deep (a step at its
bottom edge), and the entrances are openings in the bottom face. The groove family (identical for
every nut):

  * channels are 2 sectors (30 degrees) wide;
  * an ENTRANCE riser: layers 0-1 at sectors (e, e + 1);
  * a first TURN: layers 1-2, running k1 sectors from sector e (k1 = 2..4 sectors either way:
    counter-clockwise for k1 > 0), then a riser in layers 2-4 at the new sector;
  * a second TURN: layers 4-5, k2 sectors (2..4, either way), then a riser in layers 5-7;
  * optional DEAD ENDS: a turn channel can also continue 1-3 sectors the other way and stop;
  * optional FALSE ENTRANCES (0 to 2): risers in layers 0-2 at other sectors that lead nowhere.

The pin enters the groove from below. With the nut resting on the pin, the nut's bottom is 39.5
mm above the collar. The nut comes down to 24.5 mm above the collar once the pin is in the
entrance riser (or a false entrance), to 9.5 mm once it has passed the first turn and riser, and
to the collar once it has passed the second. Passing a turn means rotating the nut about the shaft
while the pin is in the turn channel; the nut turns by -15 degrees per sector the pin advances
counter-clockwise in the nut frame. Which entrance, which turns and which decoys a nut has differs
between nuts (the nut spec's `groove`). A kit has NUTS = 3 nuts. Every episode places them flat on
the table at fresh positions (NUT_REGION, at least NUT_MIN_SPACING apart, handles clear of the other
nuts) and yaws: each yaw is the nut's azimuth seen from the robot base (atan2(y - 0, x + 0.56)) plus
135 degrees plus a uniform offset within +-60 degrees. The work order assigns each nut a different
peg; nut i goes on the peg with order[i] bands (`work_order` names the nuts by colour, `colour_name`).

Nut spec (a dict): {"id": str, "groove": {"entrance": 0..23, "turns": [k1, k2], "dead_ends":
[n1, n2] (sectors, 0 = none), "false_entrances": [sectors]}, "mass": kg (0.05-0.15), "friction":
sliding friction of the nut surfaces (0.6-1.0), "rgba": [r, g, b, 1]}.

Safety stops (they end the episode, in your dev sim as in the grader)
--------------------------------------------------------------------
  * an unstable simulation: any arm joint faster than INSTAB_ROBOT_QVEL = 8 rad/s, a nut faster than
    INSTAB_NUT_VEL = 5 m/s, or a non-finite state (`info["fault"]` names it);
  * a ruined nut: while a nut is on its own peg (its hole axis within ENGAGE_XY = 5 mm of that peg's
    shaft axis, below the shaft top), the centre of the pin lies inside a solid cell of the nut's
    wall, more than FORCE_ANGLE_TOL = 5 degrees sideways or FORCE_Z_TOL = 1.5 mm up or down from
    every cut cell, for FORCE_STEPS = 3 consecutive steps (`pin_in_wall`); the groove walls are solid,
    so this only happens when the pin is forced through one. `info["ruined"]` lists the ruined nuts.
With `early_end=False` (a development option) a ruined nut is recorded but the episode goes on.
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
NUTS = 3                         # nuts in a kit (one per peg)
CYCLE_STEPS = 800                # cycle time per nut: 80 s simulated
MAX_STEPS = NUTS * CYCLE_STEPS   # control steps per episode (one kit, 240 s simulated)
MAX_DPOS = 0.02
MAX_DROT = 0.15
TABLE_Z = 0.80
TABLE_HALF = 0.40
ROBOT_BASE = np.array([-0.56, 0.0, TABLE_Z])
WORKSPACE_LO = np.array([-0.45, -0.45, TABLE_Z + 0.002])
WORKSPACE_HI = np.array([0.45, 0.45, TABLE_Z + 0.55])

# geometry (m)
NSEC = 24             # angular sectors of the nut ring (15 degrees each)
LAYER = 0.005         # layer height of the nut ring
NLAYER = 9            # nut height 45 mm
R_HOLE = 0.0128       # hole radius (inscribed 12.7 mm on the 24-gon; shaft 11 mm)
R_GROOVE = 0.019      # groove floor radius
R_NUT = 0.028         # nut outer radius
NUT_H = NLAYER * LAYER
HANDLE_LEN = 0.070
HANDLE_W = 0.008      # half width
HANDLE_Z = (0.025, 0.041)
SHAFT_R = 0.011
PEG_H = 0.065
COLLAR = 0.025        # collar half width
COLLAR_H = 0.010
PIN_Z = 0.0375        # pin centre above the collar top
PIN_R = 0.0172        # pin tip radius
PIN_W = 0.0015        # pin half width (tangential)
PIN_HH = 0.002        # pin half height
# groove schedule (layers): entrance channel 0-1, first turn 1-2, riser 2-4, second turn 4-5, riser 5-7
TURN_LAYERS = ((1, 2), (4, 5))
RISERS = ((0, 1), (2, 4), (5, 7))

# layout ranges (uniform per episode)
PEG_REGION = ((-0.02, 0.15), (-0.21, 0.21))
PEG_MIN_SPACING = 0.13
NUT_REGION = ((-0.24, -0.06), (-0.24, 0.24))
NUT_PEG_MIN_DIST = 0.13
NUT_MIN_SPACING = 0.12       # between nut centres
HANDLE_CLEAR = 0.07          # handle centre from any other nut centre
NUT_YAW_CENTER = math.radians(135.0)   # nut yaw = azimuth of the nut from the robot base + this +- spread
NUT_YAW_SPREAD = math.radians(60.0)

# safety stops
ENGAGE_XY = 0.005            # a nut is on its peg: hole axis within this distance of that peg's shaft axis
GROOVE_TOP = 0.0395          # nut bottom above the collar when it rests on the pin (pin top)
FORCE_ANGLE_TOL = 5.0        # deg: pin centre this far inside a solid wall cell (sideways) breaks the nut
FORCE_Z_TOL = 0.0015         # m: ... or this far (up or down)
FORCE_STEPS = 3              # consecutive control steps

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
NUT_SPEC_KEYS = ("id", "groove", "mass", "friction", "rgba")
OBS_KEYS = ("t", "order", "q", "dq", "ee_pos", "ee_quat", "gripper", "wrench", "T_world_wrist",
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
    g = spec["groove"]
    if (not isinstance(g, dict) or int(g["entrance"]) not in range(NSEC) or len(g["turns"]) != 2
            or any(not 1 <= abs(int(k)) <= 6 for k in g["turns"])):
        raise EnvError("groove needs entrance 0-23 and two turns of 1-6 sectors")
    return spec


def groove_cells(groove):
    """The (sector, layer) cells cut out of the ring's inner part. Channels are 2 sectors wide;
    sector s spans [15 s, 15 (s + 1)] degrees counter-clockwise from the handle (nut +x)."""
    cells = set()

    def chan(sec, layers):
        for lay in layers:
            cells.update({(sec % NSEC, lay), ((sec + 1) % NSEC, lay)})

    s = int(groove["entrance"])
    for i, (lo, hi) in enumerate(RISERS):
        chan(s, range(lo, hi + 1))
        if i < 2:
            k = int(groove["turns"][i])
            step = 1 if k > 0 else -1
            for j in range(0, k + step, step):
                chan(s + j, TURN_LAYERS[i])
            for j in range(1, int(groove.get("dead_ends", [0, 0])[i]) + 1):    # dead end, opposite way
                chan(s - step * j, TURN_LAYERS[i])
            s = s + k
    for f in groove.get("false_entrances", []):                                # blind risers
        chan(int(f), range(0, TURN_LAYERS[0][1] + 1))
    return cells


# ----------------------------------------------------------------------------------------------
# Model assembly
# ----------------------------------------------------------------------------------------------
def _box(x0, x1, y0, y1, z0, z1, attrs):
    return '<geom type="box" pos="{:.6f} {:.6f} {:.6f}" size="{:.6f} {:.6f} {:.6f}" {}/>'.format(
        (x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2, (x1 - x0) / 2, (y1 - y0) / 2, (z1 - z0) / 2, attrs)


def nut_body_xml(spec):
    """The nut as an MJCF body (origin: hole axis on the bottom face; handle along local +x). The ring is
    NSEC x NLAYER cells, each an inner block (R_HOLE..R_GROOVE) and an outer block (R_GROOVE..R_NUT);
    groove cells have no inner block."""
    cells = groove_cells(spec["groove"])
    rgba = " ".join("{:.3f}".format(float(c)) for c in spec["rgba"])
    attrs = 'rgba="{}" condim="4" solref="0.002 1" friction="{:.4f} 0.01 0.0001"'.format(rgba, float(spec["friction"]))
    da = 2 * math.pi / NSEC
    g = []
    for sec in range(NSEC):
        a = (sec + 0.5) * da
        c, sn = math.cos(a), math.sin(a)
        quat = "{:.6f} 0 0 {:.6f}".format(math.cos(a / 2), math.sin(a / 2))
        for lay in range(NLAYER):
            z = (lay + 0.5) * LAYER
            for (r0, r1, inner) in ((R_HOLE, R_GROOVE, True), (R_GROOVE, R_NUT, False)):
                if inner and (sec, lay) in cells:
                    continue
                rc = (r0 + r1) / 2
                ht = r1 * math.tan(da / 2) + 0.0002
                g.append('<geom type="box" pos="{:.6f} {:.6f} {:.6f}" quat="{}" size="{:.6f} {:.6f} {:.6f}" {}/>'.format(
                    rc * c, rc * sn, z, quat, (r1 - r0) / 2, ht, LAYER / 2, attrs))
    g.append(_box(R_NUT - 0.002, R_NUT + HANDLE_LEN, -HANDLE_W, HANDLE_W, HANDLE_Z[0], HANDLE_Z[1], attrs))
    m = float(spec["mass"])
    return ('<body name="nut"><freejoint name="nut_free"/>'
            '<inertial pos="0.015 0 0.018" mass="{m:.5f}" diaginertia="{a:.3e} {b:.3e} {c:.3e}"/>{g}</body>').format(
        m=m, a=m * 6.0e-4, b=m * 9.0e-4, c=m * 1.1e-3, g="".join(g))


def peg_body_xml(i, bands):
    shaft = 'rgba="0.62 0.64 0.67 1"'
    g = _box(-COLLAR, COLLAR, -COLLAR, COLLAR, 0, COLLAR_H, 'rgba="0.30 0.31 0.34 1"')
    g += '<geom type="cylinder" pos="0 0 {:.4f}" size="{:.4f} {:.4f}" {}/>'.format(
        COLLAR_H + PEG_H / 2, SHAFT_R, PEG_H / 2, shaft)
    g += _box(SHAFT_R - 0.0005, PIN_R, -PIN_W, PIN_W, COLLAR_H + PIN_Z - PIN_HH, COLLAR_H + PIN_Z + PIN_HH, shaft)
    top = COLLAR_H + PEG_H
    for k in range(bands):
        z = top - 0.010 - 0.013 * k
        g += ('<geom type="cylinder" pos="0 0 {:.4f}" size="{:.4f} 0.0035" rgba="0.07 0.07 0.09 1" contype="0" '
              'conaffinity="0" group="1" mass="0"/>').format(z, SHAFT_R + 0.0004)
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


def build_model(specs, layout):
    """Compile table + Panda + 3 pegs + the nuts for a layout {pegs: [(x, y, yaw)]*3, nuts: [(x, y, yaw)]}."""
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
            "".join(peg_body_xml(i, i + 1) for i in range(3)),
            "".join(nut_body_xml(sp).replace('<body name="nut">', '<body name="nut%d">' % i, 1)
                    .replace('name="nut_free"', 'name="nut%d_free"' % i, 1) for i, sp in enumerate(specs))))
    for i, (x, y, yaw) in enumerate(layout["pegs"]):
        wb.add_frame(pos=[x, y, TABLE_Z], quat=list(yaw_quat(yaw))).attach_body(objs.body("peg%d" % i), "o_", "")
    for i, (x, y, yaw) in enumerate(layout["nuts"]):
        wb.add_frame(pos=[x, y, TABLE_Z + 0.0005], quat=list(yaw_quat(yaw))).attach_body(objs.body("nut%d" % i), "o_", "")
    for act in root.actuators:
        if act.name.startswith("r_g_gripper_finger_joint"):
            act.gainprm[0] = GRIP_KP
            act.biasprm[1] = -GRIP_KP
    m = root.compile()
    m.cam_fovy[m.camera("r_eye_in_hand").id] = WRIST["fovy"]
    return m


def sample_layout(rng, nuts=NUTS):
    """Peg positions/yaws (peg i has i + 1 bands) and the nut placements; rejection-sampled."""
    for _ in range(20000):
        pegs = []
        while len(pegs) < 3:
            x = rng.uniform(*PEG_REGION[0])
            y = rng.uniform(*PEG_REGION[1])
            if all(math.hypot(x - p[0], y - p[1]) >= PEG_MIN_SPACING for p in pegs):
                pegs.append((float(x), float(y), float(rng.uniform(-math.pi, math.pi))))
        placed = []
        for _k in range(200):
            if len(placed) == nuts:
                break
            nx, ny = rng.uniform(*NUT_REGION[0]), rng.uniform(*NUT_REGION[1])
            az = math.atan2(ny - ROBOT_BASE[1], nx - ROBOT_BASE[0])
            nyaw = float(wrap_angle(az + NUT_YAW_CENTER + rng.uniform(-NUT_YAW_SPREAD, NUT_YAW_SPREAD)))
            hx, hy = nx + 0.063 * math.cos(nyaw), ny + 0.063 * math.sin(nyaw)    # handle centre
            ok = all(math.hypot(nx - p[0], ny - p[1]) >= NUT_PEG_MIN_DIST and math.hypot(hx - p[0], hy - p[1]) >= 0.09
                     for p in pegs) and abs(hy) < 0.3 and hx > -0.28
            for (ox, oy, oyaw) in placed:
                ohx, ohy = ox + 0.063 * math.cos(oyaw), oy + 0.063 * math.sin(oyaw)
                ok = ok and math.hypot(nx - ox, ny - oy) >= NUT_MIN_SPACING \
                    and math.hypot(hx - ox, hy - oy) >= HANDLE_CLEAR and math.hypot(ohx - nx, ohy - ny) >= HANDLE_CLEAR \
                    and math.hypot(hx - ohx, hy - ohy) >= 0.05
            if ok:
                placed.append((float(nx), float(ny), nyaw))
        if len(placed) == nuts:
            order = rng.permutation(3)
            return {"pegs": [pegs[i] for i in order], "nuts": placed}
    raise EnvError("layout sampling failed")


# ----------------------------------------------------------------------------------------------
# Environment
# ----------------------------------------------------------------------------------------------
class NutEnv(object):
    """One kit: NUTS nuts on the table, each to be assembled on its own peg (the work order)."""

    def __init__(self, nut_specs, render=True, max_steps=MAX_STEPS, early_end=True):
        """early_end: end the episode at the first ruined nut (a fault always ends it)."""
        if isinstance(nut_specs, dict):
            nut_specs = [nut_specs]
        self.specs = [check_nut_spec(sp) for sp in nut_specs]
        self.n = len(self.specs)
        self.early_end = early_end
        self.render_enabled = render
        self.max_steps = int(max_steps)
        self.model = None
        self._renderers = {}

    # ------------------------------------------------------------------ episode
    def reset(self, seed, order):
        """order[i] = band count (1-3) of the peg nut i goes on; with three nuts, a permutation of 1, 2, 3."""
        order = [int(b) for b in order]
        if len(order) != self.n or any(b not in (1, 2, 3) for b in order) or len(set(order)) != self.n:
            raise EnvError("order must give each nut a different peg (band count 1-3)")
        self.order = order
        self.seed = int(seed) & (2 ** 64 - 1)
        place = np.random.default_rng([self.seed & 0xFFFFFFFF, self.seed >> 32, 7])
        self.rng = np.random.default_rng([self.seed & 0xFFFFFFFF, self.seed >> 32, 11])
        self.layout = sample_layout(place, self.n)
        self.close()
        self.model = build_model(self.specs, self.layout)
        self.data = mujoco.MjData(self.model)
        self.fk = mujoco.MjData(self.model)
        self._index()
        self._init_state()
        self.wrench_bias = np.concatenate([self.rng.uniform(-WRENCH_BIAS, WRENCH_BIAS, 3),
                                           self.rng.uniform(-0.1 * WRENCH_BIAS, 0.1 * WRENCH_BIAS, 3)])
        self.t = 0
        self.done = False
        self.fault = None
        self.broken = [False] * self.n
        self.force_count = [0] * self.n
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
        self.nut_body, self.nut_qadr, self.nut_dadr, self.nut_geoms = [], [], [], []
        for i in range(self.n):
            b = m.body("o_nut%d" % i).id
            j = m.joint("o_nut%d_free" % i)
            self.nut_body.append(b)
            self.nut_qadr.append(int(j.qposadr[0]))
            self.nut_dadr.append(int(j.dofadr[0]))
            self.nut_geoms.append(set(int(g) for g in np.where(m.geom_bodyid == b)[0]))
        fingers = set()
        for b in range(m.nbody):
            nm = m.body(b).name
            if nm.startswith("r_g_") and ("finger" in nm):
                fingers.add(b)
        self.finger_geoms = set(int(i) for i in np.where(np.isin(m.geom_bodyid, list(fingers)))[0])
        self.target_peg = [b - 1 for b in self.order]           # peg body i carries i + 1 bands
        self.peg_poses = [tuple(p) for p in self.layout["pegs"]]
        self.cells = [groove_cells(sp["groove"]) for sp in self.specs]
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
    def nut_state(self, i=0):
        """(position of nut i's origin = hole axis on its bottom face, rotation matrix)."""
        d = self.data
        b = self.nut_body[i]
        return d.xpos[b].copy(), d.xmat[b].reshape(3, 3).copy()

    def _finger_touch(self, i):
        """(finger geoms, finger bodies) touching nut i."""
        d, m = self.data, self.model
        geoms, bodies = set(), set()
        ng = self.nut_geoms[i]
        for k in range(d.ncon):
            c = d.contact[k]
            g1, g2 = int(c.geom1), int(c.geom2)
            if g1 in ng and g2 in self.finger_geoms:
                geoms.add(g2)
                bodies.add(int(m.geom_bodyid[g2]))
            elif g2 in ng and g1 in self.finger_geoms:
                geoms.add(g1)
                bodies.add(int(m.geom_bodyid[g1]))
        return geoms, bodies

    def pin_in_wall(self, i=0):
        """True if the centre of nut i's target peg's pin lies inside a solid cell of the nut's wall, more than
        FORCE_ANGLE_TOL sideways or FORCE_Z_TOL up/down from every groove cell of the same layer/sector."""
        p, R = self.nut_state(i)
        px, py, pyaw = self.peg_poses[self.target_peg[i]]
        rc = (SHAFT_R + PIN_R) / 2
        pin = np.array([px + rc * math.cos(pyaw), py + rc * math.sin(pyaw), TABLE_Z + COLLAR_H + PIN_Z])
        q = R.T @ (pin - p)
        z = float(q[2])
        if not 0.0 < z < NUT_H or math.hypot(q[0], q[1]) > R_GROOVE:
            return False
        ang = math.degrees(math.atan2(q[1], q[0])) % 360.0
        da = 360.0 / NSEC
        for dz in (0.0, FORCE_Z_TOL, -FORCE_Z_TOL):
            lay = int(math.floor((z + dz) / LAYER))
            for dang in (0.0, FORCE_ANGLE_TOL, -FORCE_ANGLE_TOL):
                sec = int(math.floor(((ang + dang) % 360.0) / da))
                if (sec, lay) in self.cells[i] or lay < 0:
                    return False
        return True

    def on_peg(self, i=0):
        """True while nut i is on its own peg: its hole axis within ENGAGE_XY of that peg's shaft axis, the nut
        below the shaft top and above the table (the condition under which `pin_in_wall` can ruin it)."""
        p, _R = self.nut_state(i)
        px, py, _pyaw = self.peg_poses[self.target_peg[i]]
        top = TABLE_Z + COLLAR_H + PEG_H
        return math.hypot(p[0] - px, p[1] - py) <= ENGAGE_XY and p[2] < top - 0.005 and p[2] > TABLE_Z

    def _post_step(self):
        d = self.data
        if self.fault is None:
            if np.max(np.abs(d.qvel[self.jv])) > INSTAB_ROBOT_QVEL:
                self.fault = "robot joint speed"
            elif any(np.linalg.norm(d.qvel[a:a + 3]) > INSTAB_NUT_VEL for a in self.nut_dadr):
                self.fault = "nut speed"
        if self.fault is not None:
            self.done = True
            return
        for i in range(self.n):
            if not self.broken[i] and self.on_peg(i):
                self.force_count[i] = self.force_count[i] + 1 if self.pin_in_wall(i) else 0
                if self.force_count[i] >= FORCE_STEPS:
                    self.broken[i] = True
        if (self.early_end and any(self.broken)) or self.t >= self.max_steps:
            self.done = True

    def info(self):
        return {"t": self.t, "fault": self.fault, "ruined": [i for i in range(self.n) if self.broken[i]]}

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

    def work_order(self):
        """[{"colour": name, "peg_bands": b}] per nut (nut i first)."""
        return [{"colour": colour_name(sp["rgba"]), "peg_bands": b} for sp, b in zip(self.specs, self.order)]

    def observation(self):
        d = self.data
        R = d.site_xmat[self.site].reshape(3, 3)
        Rft = d.site_xmat[self.ft_site].reshape(3, 3)
        f = Rft @ d.sensordata[self.ft_force:self.ft_force + 3] + self.wrench_bias[:3] + self.rng.normal(0, WRENCH_NOISE, 3)
        tq = (Rft @ d.sensordata[self.ft_torque:self.ft_torque + 3] + self.wrench_bias[3:]
              + self.rng.normal(0, WRENCH_NOISE * 0.1, 3))
        obs = {"t": int(self.t),
               "order": self.work_order(),
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


NUT_COLOURS = {"blue": [0.16, 0.36, 0.78, 1], "green": [0.20, 0.62, 0.30, 1], "red": [0.78, 0.18, 0.16, 1],
               "yellow": [0.86, 0.74, 0.16, 1], "orange": [0.85, 0.45, 0.12, 1], "purple": [0.55, 0.25, 0.70, 1],
               "teal": [0.15, 0.65, 0.68, 1], "pink": [0.80, 0.35, 0.55, 1]}


def colour_name(rgba):
    """The NUT_COLOURS name of an rgba (nearest)."""
    return min(NUT_COLOURS, key=lambda k: sum((a - b) ** 2 for a, b in zip(NUT_COLOURS[k][:3], rgba[:3])))


def sample_nut_spec(rng, nut_id="N", colour=None):
    """A random nut from the task's nut family (the hidden graded nuts are drawn with this exact function):
    entrance uniform over the 24 sectors; each turn 2, 3 or 4 sectors, either way; each turn channel continues
    into a dead end of 0-3 sectors the other way; 0, 1 or 2 false entrances at sectors at least 2 sectors clear
    of every cell of the true path and of each other; mass uniform 0.05-0.15 kg; friction uniform 0.6-1.0;
    colour from NUT_COLOURS (uniform, unless `colour` names one). `rng` is a numpy Generator."""
    g = {"entrance": int(rng.integers(NSEC)),
         "turns": [int(rng.choice([2, 3, 4])) * int(rng.choice([-1, 1])) for _ in range(2)],
         "dead_ends": [int(rng.integers(4)) for _ in range(2)],
         "false_entrances": []}
    # sectors the true path and its first dead end occupy in the bottom layers (entrance riser and first turn)
    busy = {sec for (sec, lay) in groove_cells(g) if lay <= TURN_LAYERS[0][1]}
    for _ in range(int(rng.integers(3))):
        free = [f for f in range(NSEC)
                if all(min((f + d - b) % NSEC, (b - f - d) % NSEC) >= 2 for b in busy for d in (0, 1))]
        if not free:
            break
        f = int(rng.choice(free))
        g["false_entrances"].append(f)
        busy.update({f % NSEC, (f + 1) % NSEC})
    return check_nut_spec({"id": nut_id, "groove": g, "mass": round(float(rng.uniform(0.05, 0.15)), 4),
                           "friction": round(float(rng.uniform(0.6, 1.0)), 4),
                           "rgba": list(NUT_COLOURS[colour] if colour else
                                        NUT_COLOURS[sorted(NUT_COLOURS)[int(rng.integers(len(NUT_COLOURS)))]])})


def sample_kit(rng, kit_id="K"):
    """A random kit as graded (the hidden kits are drawn with this exact function): NUTS nuts drawn with
    `sample_nut_spec`, of NUTS different colours chosen uniformly, and the work order, a uniformly random
    assignment of the nuts to the pegs with 1, 2 and 3 bands. Returns {"nuts": [specs], "order": [bands]}."""
    names = sorted(NUT_COLOURS)
    cols = [names[int(i)] for i in rng.choice(len(names), NUTS, replace=False)]
    nuts = [sample_nut_spec(rng, "%s-%d" % (kit_id, i), colour=c) for i, c in enumerate(cols)]
    order = [int(b) + 1 for b in rng.permutation(3)][:NUTS]
    return {"nuts": nuts, "order": order}


def load_spec(path):
    with open(path) as fh:
        return check_nut_spec(json.load(fh))


def load_kit(path):
    """A kit file: {"nuts": [3 nut specs], "order": [band count per nut]}."""
    with open(path) as fh:
        kit = json.load(fh)
    return {"nuts": [check_nut_spec(sp) for sp in kit["nuts"]], "order": [int(b) for b in kit["order"]]}

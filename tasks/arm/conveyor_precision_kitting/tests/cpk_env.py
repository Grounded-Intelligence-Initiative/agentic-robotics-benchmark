"""Conveyor precision kitting cell: a Franka Panda picks parts off a moving belt and seats each in its keyed
precision pocket of a kit tray; parts lying the wrong way up go through a turnover station first.

Where this file runs: in the grader (tests/cpk_env.py) and in the practice-cell service of the agent's sandbox (a
byte copy readable by the service's account only). The agent never reads it.

The robot and its tracking law follow tasks/arm/keyed_nut_peg_assembly (robosuite 1.5.2's Panda assets, an
operational-space controller at 10 Hz, the hand pointing down with a yaw); the belt, the parts, the kit, the
turnover station, the sensors and the credit are this task's own.

Frames and units: world frame, the table top is z = 0.80; the Panda base stands at (-0.56, 0, 0.80) facing +x.
Metres, radians, seconds, newtons. Quaternions are (w, x, y, z).

    env = CellEnv()
    obs = env.reset(seed)                 # one shift
    obs, done, info = env.step(action)

Action (7 numbers): [dx, dy, dz, dyaw, grip, cams, turn]
    dx..dz: displacement of the grip point, world frame (m; the vector is clipped to length MAX_DPOS), applied
            relative to where the grip point is when the command is executed;
    dyaw:   rotation of the hand about the world vertical (rad; clipped to MAX_DYAW); the hand always points down;
    grip:   -1 = open (80 mm) ... +1 = closed, linear in between (finger force capped at 20 N);
    cams:   which images the next observation carries, a bit mask: 1 overview, 2 wrist, 4 inspection camera;
            every image taken costs CAM_COST_S of the cell's time (the step lasts that much longer);
    turn:   > 0.5 asks the turnover station to cycle.
The motion and gripper part of an action is executed LATENCY control steps after the observation it answers (the
cell's command pipeline); cams and turn act at once.
"""

import math
import os
from collections import deque

import numpy as np

import mujoco

# ----------------------------------------------------------------------------------------------
# Constants
#
# Difficulty knobs (each documented to the agent in datasheet.md / cell_data.py; change both together):
#   LATENCY_RANGE       command pipeline delay per shift (control steps), hidden     2-3
#   ARM_GAIN/DAMPING    tracking stiffness x / damping ratio per shift, hidden       0.75-1.25 / 0.75-1.10
#   GRAVITY_COMP_ERR    gravity compensation error per shift, hidden                 +-3 %
#   CAM_COST_S          cell time an image costs                                     0.05 s
#   BELT_SPEED/WOBBLE   belt speed range per shift, periodic variation               30-55 mm/s, +-8 %
#   BELT_DRIFT          slow drift of the belt speed over the first 400 s            +-10 %
#   LIGHT_RANGE/TINT    lamp brightness and colour cast per shift                    0.7-1.3, +-6 % per channel
#   FRICTION_RANGE      part/tray friction per shift                                 0.25-0.55
#   RELEASE_GAP/FIRST   spacing of the parts on the belt                             44-54 s, first 20-26 s
#   RELEASES            parts per shift (6 needed + spares)                          8
#   FLIP_P              share of parts lying upside down                             0.5
#   CLEAR/BOSS_CLEAR    pocket clearance per side, key boss clearance                0.6 mm, 0.9 mm
#   CHAMFER             lead-in chamfer on the pocket walls                          1.0 mm
#   DAMAGE_FORCE        part-kit force limit                                         12 N for 2 steps
#   COLOUR_TOL          shade tolerance per part (variant pairs differ by ~0.02-0.03)  0.02
#   CAM_TOL             camera mount tolerances (wrist / inspection)                 1.0 mm 0.3 deg / 0.8 mm 0.3 deg
#   KIT_JITTER          kit tray placement per shift                                 12 mm, 5 deg
#   POCKET_TOL          each pocket's position on its tray (relative to the fiducials)  1.2 mm per axis
# The practice budget (2 shifts) is in practice_service/practiced.py; the reply budget (0.1 s) in
# tests/grader_config.json.
# ----------------------------------------------------------------------------------------------
TIMESTEP = 0.002
SUBSTEPS = 50                     # -> a control step lasts 0.1 s (plus CAM_COST_S per image taken)
CONTROL_DT = TIMESTEP * SUBSTEPS
LATENCY = 2                       # nominal command pipeline delay (control steps); a shift draws LATENCY_RANGE
LATENCY_RANGE = (2, 3)            # v4: the shift's real delay, fixed within the shift, never reported
ARM_GAIN_RANGE = (0.75, 1.25)     # v4: the shift's tracking stiffness, relative to the nominal 400 / 200
ARM_DAMPING_RANGE = (0.75, 1.10)  # v4: the shift's damping ratio (nominal 1.0, critically damped)
GRAVITY_COMP_ERR = 0.03           # v4: the shift's gravity-compensation error, relative (a steady sag or lift)
CAM_COST_S = 0.05                 # cell time per image taken
CAM_COST_SUBSTEPS = int(round(CAM_COST_S / TIMESTEP))
MAX_DPOS = 0.03
MAX_DYAW = 0.15
TABLE_Z = 0.80
TABLE_HALF = (0.40, 0.68)
ROBOT_BASE = np.array([-0.56, 0.0, TABLE_Z])
WORKSPACE_LO = np.array([-0.38, -0.50, TABLE_Z + 0.004])
WORKSPACE_HI = np.array([0.30, 0.42, TABLE_Z + 0.40])

# ---- parts: 20 mm thick plates, outline in the part frame (origin at the outline's bounding-box centre, x along the
# length); the key is cut through the full height on the +x end.
H_PART = 0.020
# name: (length, width, key type, key dims, rgb, mass)
#   corner (nx, ny): a notch nx long, ny wide at the (+x, +y) corner
#   slot (w, d):     a slot w wide, d deep, centred in the +x end
#   tab (nx, ny):    both +x corners notched nx by ny (the +x end is a narrower tab)
VARIANTS = {
    "A1": (0.044, 0.024, "corner", (0.010, 0.008), (0.20, 0.36, 0.78), 0.045),
    "A2": (0.052, 0.024, "corner", (0.010, 0.008), (0.22, 0.37, 0.76), 0.052),
    "B1": (0.040, 0.028, "slot", (0.010, 0.008), (0.24, 0.58, 0.30), 0.044),
    "B2": (0.040, 0.028, "corner", (0.010, 0.008), (0.26, 0.60, 0.28), 0.046),
    "C1": (0.038, 0.030, "corner", (0.009, 0.008), (0.86, 0.47, 0.15), 0.048),
    "C2": (0.046, 0.026, "tab", (0.008, 0.006), (0.84, 0.49, 0.17), 0.045),
}
VARIANT_NAMES = ("A1", "A2", "B1", "B2", "C1", "C2")
PART_FRICTION = 0.35
MARK_R = 0.0035                   # datum mark on the top face: radius, and centre inset from the -x end
MARK_INSET = 0.008
COLOUR_TOL = 0.02                 # each RGB component of a part, per part

# ---- kit tray: plate with six keyed pockets; pocket frame = the frame of its part when seated
KIT_PLATE = (0.125, 0.082, 0.005)        # half sizes; plate top = pocket floor
CLEAR = 0.0006                           # clearance per side between a part and its pocket walls
BOSS_CLEAR = 0.0009                      # ... and between the part's key feature and the pocket's key boss
KEY_H = 0.0025                           # key boss height above the floor (below the walls' lead-in)
POCKET_DEPTH = 0.005                     # wall height above the floor
WALL_T = 0.004                           # wall thickness
CHAMFER = 0.0010                         # 45 degree lead-in on the walls' inner top edges
# (variant, x, y, yaw) in the kit frame
POCKETS = (
    ("A1", -0.075, 0.040, 0.0),
    ("B2", 0.000, 0.040, 0.0),
    ("C1", 0.075, 0.040, math.pi / 2),
    ("A2", -0.075, -0.040, math.pi),
    ("B1", 0.000, -0.040, 0.0),
    ("C2", 0.075, -0.040, math.pi),
)
FIDUCIALS = ((-0.110, 0.068), (0.110, 0.068), (-0.110, -0.068))   # dark dots on the plate, kit frame
FIDUCIAL_R = 0.006
KIT_NOMINAL = (-0.155, -0.27, math.pi / 2)
KIT_JITTER = (0.012, 0.012, math.radians(5.0))
POCKET_TOL = 0.0012                      # each tray's pockets sit within this (per axis) of their drawing position,
                                         # relative to the tray's fiducials (a pool of trays, each made and worn alike)
KIT_FRICTION = 0.35

# ---- belt (runs towards -y)
BELT_X = 0.10
BELT_Z = TABLE_Z + 0.040                 # belt top
BELT_HALF_W = 0.080
RAIL_GAP = 0.088                         # rail inner faces at BELT_X +- this
RAIL_H = 0.020
RELEASE_Y = 0.62                         # where the feeder puts parts on the belt
PICK_ZONE = (0.40, -0.40)                # parts are reachable between these y; past the end they are lost
BELT_SPEED = (0.030, 0.055)              # m/s, per shift
BELT_WOBBLE = 0.08                       # relative, slow sinusoid
BELT_DRIFT = 0.10                        # v4: relative drift over the first BELT_DRIFT_S of a shift (either way)
BELT_DRIFT_S = 400.0
BELT_WOBBLE_PERIOD = (40.0, 80.0)
STRIPE_PITCH = 0.10
LATERAL = 0.030                          # part centre within +- this of the belt centre line
FLIP_P = 0.5                             # chance a part lies upside down
RELEASES = 8                             # parts per shift: one of every variant plus two spares
RELEASE_FIRST = (20.0, 26.0)
RELEASE_GAP = (44.0, 54.0)
SHIFT_GRACE = 60.0                       # s after the last part leaves the pick zone

# ---- turnover station: two nests side by side along y; a cycle turns the input nest's part over (180 degrees
# about the x axis through the station centre, at the parts' mid height) onto the output nest
STATION = (-0.12, 0.28)
NEST_OFF = 0.046
NEST_IN = (0.032, 0.026)                 # inner half sizes
NEST_DEPTH = 0.004
NEST_CHAMFER = 0.0015
STATION_BASE_H = 0.010                   # nest floor above the table
TURN_STEPS = (2, 16, 6)                  # clamp, turn, return (control steps)
GUARD_R = 0.12                           # the station cycles only with the hand outside this radius ...
GUARD_Z = 0.10                           # ... or this high above the nest floor

# ---- inspection camera (looks up)
UPCAM_POS = (-0.10, 0.04, TABLE_Z + 0.012)
UPCAM_FOVY = 40.0

# ---- credit and damage
SEAT_XY = 0.0012
SEAT_Z = 0.0006
SEAT_TILT = math.radians(2.0)
SEAT_YAW = math.radians(3.0)
REST_SPEED = 0.01
DAMAGE_FORCE = 12.0                      # N: part-kit contact force averaged over a control step
DAMAGE_STEPS = 2

# ---- robot, sensors
GRIP_KP = 3000.0
FINGER_COUPLING = (0.005, 1.0)
PAD_FRICTION = 1.5
INSTAB_ROBOT_QVEL = 8.0
INSTAB_PART_VEL = 6.0
HOME_Q = np.array([0.0, -0.35, 0.0, -2.35, 0.0, 2.0, math.pi / 4])
NULLSPACE_Q = np.array([0.0, -0.3, 0.0, -2.2, 0.0, 1.9, math.pi / 4])
WRENCH_NOISE = (0.20, 0.015)
WRENCH_BIAS = (0.5, 0.04)
WRENCH_DRIFT = (0.25 / 60.0, 0.01 / 60.0)

OVERVIEW_CAM = ((0.02, 0.0, 1.75), (-0.02, 0.0, 0.80), 58.0)
IMG = {"overview": (160, 120), "wrist": (160, 120), "inspect": (96, 96)}   # (width, height)
WRIST_FOVY = 75.0
CAMERA_NAMES = ("overview", "wrist", "inspect")
CAM_BITS = {"overview": 1, "wrist": 2, "inspect": 4}
CAM_TOL = {                                    # the real cameras against their drawing, per shift
    "overview": {"pos": 0.010, "rot_deg": 1.5},
    "wrist": {"pos": 0.0010, "rot_deg": 0.3},
    "inspect": {"pos": 0.0008, "rot_deg": 0.3},
}
LIGHT_RANGE = (0.70, 1.30)
LIGHT_TINT = 0.06                 # v4: per-channel colour cast of the lamps (relative)
FRICTION_RANGE = (0.25, 0.55)     # v4: the shift's part-tray friction (parts and tray surfaces)



class EnvError(Exception):
    pass


class ActionError(ValueError):
    pass


# ----------------------------------------------------------------------------------------------
# Helpers
# ----------------------------------------------------------------------------------------------
def look_at(pos, target, up=(0, 0, 1)):
    pos, target = np.asarray(pos, float), np.asarray(target, float)
    f = target - pos
    f = f / np.linalg.norm(f)
    z = -f
    x = np.cross(np.array(up, dtype=float), z)
    if np.linalg.norm(x) < 1e-9:
        x = np.array([0.0, -1.0, 0.0]) if z[2] > 0 else np.array([1.0, 0.0, 0.0])
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


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def axis_angle_mat(v):
    v = np.asarray(v, float)
    th = float(np.linalg.norm(v))
    if th < 1e-12:
        return np.eye(3)
    k = v / th
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(th) * K + (1 - math.cos(th)) * K @ K


def wrap_angle(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def yaw_of_mat(R):
    return math.atan2(R[1, 0], R[0, 0])


def robosuite_assets():
    import importlib.util
    spec = importlib.util.find_spec("robosuite")
    if spec is None or not spec.submodule_search_locations:
        raise EnvError("robosuite assets not installed")
    return os.path.join(list(spec.submodule_search_locations)[0], "models", "assets")


def parse_action(a):
    if not isinstance(a, (list, tuple)) or len(a) != 7:
        raise ActionError("action must be a list of 7 numbers")
    out = []
    for v in a:
        if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(float(v)) or abs(v) > 1e3:
            raise ActionError("action must be a list of 7 finite numbers")
        out.append(float(v))
    cams = int(round(out[5]))
    if cams < 0 or cams > 7:
        raise ActionError("the camera mask must be an integer from 0 to 7")
    return np.array(out[:3]), out[3], float(np.clip(out[4], -1.0, 1.0)), cams, out[6] > 0.5


def encode_image(img):
    import base64
    img = np.ascontiguousarray(img, dtype=np.uint8)
    return {"dtype": "uint8", "shape": list(img.shape), "b64": base64.b64encode(img.tobytes()).decode("ascii")}


def decode_image(d):
    import base64
    return np.frombuffer(base64.b64decode(d["b64"]), np.uint8).reshape(d["shape"])


def to_wire(obs):
    out = {}
    for k, v in obs.items():
        if k in CAMERA_NAMES:
            out[k] = encode_image(v)
        elif isinstance(v, np.ndarray):
            out[k] = v.tolist()
        else:
            out[k] = v
    return out


def part_rects(name):
    """The part's outline as boxes: list of (cx, cy, hx, hy) in the part frame."""
    L, W, key, (a, b), _rgb, _m = VARIANTS[name]
    x0, x1, y0, y1 = -L / 2, L / 2, -W / 2, W / 2
    if key == "corner":
        rs = [(x0, x1 - a, y0, y1), (x1 - a, x1, y0, y1 - b)]
    elif key == "slot":
        rs = [(x0, x1 - b, y0, y1), (x1 - b, x1, y0, -a / 2), (x1 - b, x1, a / 2, y1)]
    elif key == "tab":
        rs = [(x0, x1 - a, y0, y1), (x1 - a, x1, y0 + b, y1 - b)]
    else:
        raise EnvError(key)
    return [((u0 + u1) / 2, (v0 + v1) / 2, (u1 - u0) / 2, (v1 - v0) / 2) for u0, u1, v0, v1 in rs]


def boss_rects(name, c=BOSS_CLEAR, cw=CLEAR):
    """The key bosses of the variant's pocket: (cx, cy, hx, hy) in the pocket frame (the notch minus clearance,
    reaching the pocket walls)."""
    L, W, key, (a, b), _rgb, _m = VARIANTS[name]
    X, Y = L / 2 + cw, W / 2 + cw                # the bosses reach the walls
    if key == "corner":
        rs = [(L / 2 - a + c, X, W / 2 - b + c, Y)]
    elif key == "slot":
        rs = [(L / 2 - b + c, X, -a / 2 + c, a / 2 - c)]
    elif key == "tab":
        rs = [(L / 2 - a + c, X, W / 2 - b + c, Y), (L / 2 - a + c, X, -Y, -W / 2 + b - c)]
    else:
        raise EnvError(key)
    return [((u0 + u1) / 2, (v0 + v1) / 2, (u1 - u0) / 2, (v1 - v0) / 2) for u0, u1, v0, v1 in rs]


def walls_xml(prefix, cx, cy, yaw, ix, iy, depth, chamfer, z0, rgba, friction):
    """A rectangular walled recess: inner half sizes ix, iy, walls `depth` tall from z0, WALL_T thick, with a 45 degree
    chamfer `chamfer` on the inner top edges. Geoms in the frame of the body they are written into: the recess
    centre at (cx, cy), turned by yaw."""
    out = []
    c, s = math.cos(yaw), math.sin(yaw)
    q_yaw = (math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2))

    def put(name, lx, ly, lz, hx, hy, hz, quat_local=None):
        x, y = cx + c * lx - s * ly, cy + s * lx + c * ly
        q = q_yaw
        if quat_local is not None:
            q = tuple(quat_mul(q_yaw, quat_local))
        out.append('<geom name="{}" type="box" pos="{:.5f} {:.5f} {:.5f}" size="{:.5f} {:.5f} {:.5f}" '
                   'quat="{:.6f} {:.6f} {:.6f} {:.6f}" rgba="{}" friction="{} 0.005 0.0001"/>'.format(
                       name, x, y, lz, hx, hy, hz, q[0], q[1], q[2], q[3], rgba, friction))

    lo_h = depth - chamfer
    r45 = math.sqrt(0.5)
    for k, (sx, sy) in enumerate(((1, 0), (-1, 0), (0, 1), (0, -1))):
        if sx != 0:
            a, span = ix, iy + WALL_T
            put("%s_w%dl" % (prefix, k), sx * (a + WALL_T / 2), 0.0, z0 + lo_h / 2, WALL_T / 2, span, lo_h / 2)
            put("%s_w%du" % (prefix, k), sx * (a + chamfer + (WALL_T - chamfer) / 2), 0.0, z0 + lo_h + chamfer / 2,
                (WALL_T - chamfer) / 2, span, chamfer / 2)
            if chamfer > 0:
                # square bar turned 45 degrees about y; its inner face is the chamfer
                put("%s_w%dc" % (prefix, k), sx * (a + chamfer), 0.0, z0 + lo_h, chamfer * r45, iy, chamfer * r45,
                    (math.cos(math.pi / 8), 0.0, math.sin(math.pi / 8), 0.0))
        else:
            a, span = iy, ix
            put("%s_w%dl" % (prefix, k), 0.0, sy * (a + WALL_T / 2), z0 + lo_h / 2, span, WALL_T / 2, lo_h / 2)
            put("%s_w%du" % (prefix, k), 0.0, sy * (a + chamfer + (WALL_T - chamfer) / 2), z0 + lo_h + chamfer / 2,
                span, (WALL_T - chamfer) / 2, chamfer / 2)
            if chamfer > 0:
                put("%s_w%dc" % (prefix, k), 0.0, sy * (a + chamfer), z0 + lo_h, ix, chamfer * r45, chamfer * r45,
                    (math.cos(math.pi / 8), math.sin(math.pi / 8), 0.0, 0.0))
    return out


def quat_mul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return (w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2)


def pocket_world(kit_pose, k, offsets=None):
    """(x, y, yaw) of pocket k's frame in the world, for a kit at kit_pose (x, y, yaw); offsets[k] = the tray's own
    (dx, dy) of that pocket in the kit frame."""
    kx, ky, kyaw = kit_pose
    _v, px, py, pyaw = POCKETS[k]
    if offsets is not None:
        px, py = px + offsets[k][0], py + offsets[k][1]
    c, s = math.cos(kyaw), math.sin(kyaw)
    return kx + c * px - s * py, ky + s * px + c * py, wrap_angle(kyaw + pyaw)


# ----------------------------------------------------------------------------------------------
# Scene
# ----------------------------------------------------------------------------------------------
SCENE_XML = """<mujoco model="conveyor_kitting_cell">
  <compiler angle="radian" autolimits="true"/>
  <option timestep="{dt}" integrator="implicitfast" cone="elliptic" impratio="10"/>
  <visual><global offwidth="640" offheight="480"/><quality shadowsize="2048"/>
    <headlight ambient="0.28 0.28 0.28" diffuse="0.35 0.35 0.35"/></visual>
  <asset>
    <texture name="floor" type="2d" builtin="flat" mark="random" random="0.12" rgb1="0.46 0.47 0.46" rgb2="0.40 0.40 0.40" width="256" height="256"/>
    <material name="floormat" texture="floor" texrepeat="8 8" specular="0.05"/>
    <texture name="wood" type="2d" builtin="flat" mark="random" random="0.04" rgb1="0.58 0.55 0.50" rgb2="0.5 0.47 0.43" width="256" height="256"/>
    <material name="tablemat" texture="wood" texrepeat="3 3" specular="0.1"/>
    <material name="alu" rgba="0.70 0.72 0.74 1" shininess="0.5"/>
    <material name="belt" rgba="0.16 0.16 0.17 1" specular="0.05"/>
  </asset>
  <default><geom solref="0.004 1" solimp="0.95 0.99 0.001"/></default>
  <worldbody>
    <light name="lamp" pos="0.1 0.1 2.3" dir="-0.05 -0.05 -1" diffuse="0.45 0.45 0.43" castshadow="true"/>
    <light name="fill" pos="-0.7 -0.6 2.0" dir="0.3 0.3 -1" diffuse="0.22 0.22 0.24" castshadow="false"/>
    <geom name="floor" type="plane" size="4 4 0.1" material="floormat" contype="1" conaffinity="1"/>
    <geom name="table" type="box" pos="0 0 {tz2}" size="{thx} {thy} {tz2}" material="tablemat" friction="0.8 0.005 0.0001"/>
    <geom name="pedestal" type="box" pos="{rx} 0 {tz2}" size="0.12 0.12 {tz2}" rgba="0.25 0.25 0.27 1" contype="0" conaffinity="0"/>
    <!-- the hall around the cell: visual only, outside the cameras' views -->
    <geom name="wall_a" type="box" pos="1.9 0 1.5" size="0.02 3.0 1.5" rgba="0.78 0.79 0.80 1" contype="0" conaffinity="0"/>
    <geom name="wall_b" type="box" pos="-2.0 0 1.5" size="0.02 3.0 1.5" rgba="0.74 0.75 0.76 1" contype="0" conaffinity="0"/>
    <geom name="wall_c" type="box" pos="0 2.6 1.5" size="3.0 0.02 1.5" rgba="0.76 0.77 0.78 1" contype="0" conaffinity="0"/>
    <geom name="wall_d" type="box" pos="0 -2.6 1.5" size="3.0 0.02 1.5" rgba="0.76 0.77 0.78 1" contype="0" conaffinity="0"/>
    <geom name="floor_line1" type="box" pos="0.0 1.05 0.001" size="1.4 0.03 0.001" rgba="0.86 0.86 0.84 1" contype="0" conaffinity="0"/>
    <geom name="floor_line2" type="box" pos="0.0 -1.05 0.001" size="1.4 0.03 0.001" rgba="0.86 0.86 0.84 1" contype="0" conaffinity="0"/>
    <geom name="cabinet" type="box" pos="-1.1 -1.0 0.9" size="0.25 0.2 0.9" rgba="0.80 0.81 0.79 1" contype="0" conaffinity="0"/>
    <geom name="post1" type="box" pos="0.62 0.72 1.05" size="0.02 0.02 1.05" rgba="0.70 0.72 0.74 1" contype="0" conaffinity="0"/>
    <geom name="post2" type="box" pos="0.62 -0.72 1.05" size="0.02 0.02 1.05" rgba="0.70 0.72 0.74 1" contype="0" conaffinity="0"/>
    <geom name="beam1" type="box" pos="0.62 0 2.1" size="0.02 0.72 0.02" rgba="0.70 0.72 0.74 1" contype="0" conaffinity="0"/>
    <geom name="cam_mount" type="box" pos="0.02 0 1.78" size="0.03 0.05 0.02" rgba="0.15 0.15 0.16 1" contype="0" conaffinity="0"/>
  </worldbody>
</mujoco>"""


def build_model(layout):
    """The cell for one shift: layout = {"kit": (x, y, yaw), "parts": [variant per release], "rgba": [...],
    "light": gain}."""
    root = mujoco.MjSpec.from_string(SCENE_XML.format(
        dt=TIMESTEP, tz2=TABLE_Z / 2, thx=TABLE_HALF[0], thy=TABLE_HALF[1], rx=ROBOT_BASE[0]))
    wb = root.worldbody
    assets = robosuite_assets()
    robot = mujoco.MjSpec.from_file(os.path.join(assets, "robots", "panda", "robot.xml"))
    grip = mujoco.MjSpec.from_file(os.path.join(assets, "grippers", "panda_gripper.xml"))
    for cam in list(robot.cameras):
        if cam.name == "robotview":
            robot.delete(cam)
    robot.body("right_hand").add_frame().attach_body(grip.body("right_gripper"), "g_", "")
    wb.add_frame(pos=list(ROBOT_BASE)).attach_body(robot.body("base"), "r_", "")

    g = []
    # ---- belt: frame, rails, the moving belt (slide joint, kinematic) and its visual stripes (mocap)
    bl = 0.75
    g.append('<geom name="belt_frame" type="box" pos="{} 0 {:.4f}" size="0.095 {} {:.4f}" rgba="0.70 0.72 0.74 1" '
             'contype="0" conaffinity="0"/>'.format(BELT_X, (TABLE_Z + BELT_Z - 0.01) / 2, bl - 0.03,
                                                   (BELT_Z - 0.01 - TABLE_Z) / 2))
    for k, sx in enumerate((1, -1)):
        g.append('<geom name="rail{}" type="box" pos="{:.4f} 0 {:.4f}" size="0.004 {} {:.4f}" rgba="0.70 0.72 0.74 1" '
                 'friction="0.3 0.005 0.0001"/>'.format(k, BELT_X + sx * (RAIL_GAP + 0.004), BELT_Z + RAIL_H / 2 - 0.004,
                                                       bl - 0.05, RAIL_H / 2 + 0.004))
    g.append('<body name="belt" pos="{} 0 {:.4f}"><joint name="belt_slide" type="slide" axis="0 1 0" armature="1000" '
             'damping="0"/><geom name="belt_g" type="box" size="{} {} 0.005" rgba="0.16 0.16 0.17 1" '
             'friction="0.9 0.005 0.0001"/></body>'.format(BELT_X, BELT_Z - 0.005, BELT_HALF_W, bl))
    stripes = "".join('<geom name="stripe{}" type="box" pos="0 {:.3f} 0" size="{} 0.004 0.0003" rgba="0.30 0.30 0.31 1" '
                      'contype="0" conaffinity="0"/>'.format(i, -bl + 0.05 + i * STRIPE_PITCH, BELT_HALF_W - 0.004)
                      for i in range(int(2 * bl / STRIPE_PITCH) - 1))
    g.append('<body name="stripes" mocap="true" pos="{} 0 {:.5f}">{}</body>'.format(BELT_X, BELT_Z + 0.0003, stripes))
    g.append('<geom name="hood" type="box" pos="{} {:.3f} {:.3f}" size="0.11 0.12 0.06" rgba="0.34 0.36 0.38 1" '
             'contype="0" conaffinity="0"/>'.format(BELT_X, RELEASE_Y + 0.02, BELT_Z + 0.075))
    g.append('<geom name="chute" type="box" pos="{} {:.3f} {:.3f}" size="0.10 0.05 0.03" rgba="0.30 0.30 0.32 1" '
             'contype="0" conaffinity="0"/>'.format(BELT_X, -bl - 0.02, BELT_Z - 0.03))

    # ---- kit tray
    kx, ky, kyaw = layout["kit"]
    kg = ['<geom name="kit_plate" type="box" pos="0 0 {:.4f}" size="{} {} {}" rgba="0.60 0.61 0.62 1" '
          'friction="{} 0.005 0.0001"/>'.format(KIT_PLATE[2], KIT_PLATE[0], KIT_PLATE[1], KIT_PLATE[2], layout.get("friction", KIT_FRICTION))]
    floor = 2 * KIT_PLATE[2]
    for k, (v, px, py, pyaw) in enumerate(POCKETS):
        px, py = px + layout["pocket_off"][k][0], py + layout["pocket_off"][k][1]
        L, W = VARIANTS[v][0], VARIANTS[v][1]
        kg += walls_xml("kit_p%d" % k, px, py, pyaw, L / 2 + CLEAR, W / 2 + CLEAR, POCKET_DEPTH, CHAMFER, floor,
                        "0.42 0.43 0.46 1", layout.get("friction", KIT_FRICTION))
        c, s = math.cos(pyaw), math.sin(pyaw)
        for j, (bx, by, hx, hy) in enumerate(boss_rects(v)):
            x, y = px + c * bx - s * by, py + s * bx + c * by
            kg.append('<geom name="kit_p{}_key{}" type="box" pos="{:.5f} {:.5f} {:.5f}" size="{:.5f} {:.5f} {:.5f}" '
                      'quat="{:.6f} 0 0 {:.6f}" rgba="0.42 0.43 0.46 1" friction="{} 0.005 0.0001"/>'.format(
                          k, j, x, y, floor + KEY_H / 2, hx, hy, KEY_H / 2, math.cos(pyaw / 2),
                          math.sin(pyaw / 2), layout.get("friction", KIT_FRICTION)))
    for j, (fx, fy) in enumerate(FIDUCIALS):
        kg.append('<geom name="kit_fid{}" type="cylinder" pos="{} {} {:.5f}" size="{} 0.0002" rgba="0.08 0.08 0.09 1" '
                  'contype="0" conaffinity="0"/>'.format(j, fx, fy, floor + 0.0002, FIDUCIAL_R))
    g.append('<body name="kit" pos="{:.5f} {:.5f} {:.4f}" quat="{:.6f} 0 0 {:.6f}">{}</body>'.format(
        kx, ky, TABLE_Z, math.cos(kyaw / 2), math.sin(kyaw / 2), "".join(kg)))

    # ---- turnover station
    sx_, sy_ = STATION
    sg = ['<geom name="st_base" type="box" pos="0 0 {:.4f}" size="0.045 0.085 {:.4f}" rgba="0.55 0.57 0.60 1" '
          'friction="0.5 0.005 0.0001"/>'.format(STATION_BASE_H / 2, STATION_BASE_H / 2)]
    for k, oy in enumerate((NEST_OFF, -NEST_OFF)):
        sg += walls_xml("st_n%d" % k, 0.0, oy, 0.0, NEST_IN[0], NEST_IN[1], NEST_DEPTH, NEST_CHAMFER, STATION_BASE_H,
                        "0.36 0.38 0.42 1", 0.5)
    g.append('<body name="station" pos="{} {} {}">{}</body>'.format(sx_, sy_, TABLE_Z, "".join(sg)))
    hz = TABLE_Z + STATION_BASE_H + H_PART / 2
    g.append('<body name="flipper" pos="{} {} {:.4f}"><joint name="flip_hinge" type="hinge" axis="1 0 0" damping="1"/>'
             '<geom name="flipper_arm" type="box" pos="0.040 {:.3f} 0" size="0.006 0.028 0.004" rgba="0.80 0.62 0.20 1" '
             'contype="0" conaffinity="0" mass="0.1"/>'
             '<geom name="flipper_hub" type="cylinder" pos="0.040 0 0" quat="0.7071 0 0.7071 0" size="0.008 0.008" '
             'rgba="0.3 0.3 0.32 1" contype="0" conaffinity="0" mass="0.05"/></body>'.format(
                 sx_, sy_, hz, NEST_OFF))

    # ---- inspection camera housing (looks up)
    ux, uy, uz = UPCAM_POS
    g.append('<geom name="upcam_body" type="cylinder" pos="{} {} {:.4f}" size="0.022 {:.4f}" rgba="0.12 0.12 0.13 1" '
             'contype="0" conaffinity="0"/>'.format(ux, uy, (TABLE_Z + uz - 0.004) / 2, (uz - 0.004 - TABLE_Z) / 2))

    # ---- parts (parked until released)
    pg = []
    for i, v in enumerate(layout["parts"]):
        rgba = layout["rgba"][i]
        geoms = "".join('<geom name="part{}_g{}" type="box" pos="{:.5f} {:.5f} 0" size="{:.5f} {:.5f} {:.5f}" rgba="{}" '
                        'friction="{} 0.005 0.0001" condim="4"/>'.format(
                            i, j, cx, cy, hx, hy, H_PART / 2, " ".join("%.3f" % u for u in rgba), layout.get("friction", PART_FRICTION))
                        for j, (cx, cy, hx, hy) in enumerate(part_rects(v)))
        L, W = VARIANTS[v][0], VARIANTS[v][1]
        dens = VARIANTS[v][5] / (L * W * H_PART)
        # the datum mark: a dark dot on the top face only, near the -x end (a part lying upside down shows none)
        mark = ('<geom name="part{}_mark" type="cylinder" pos="{:.4f} 0 {:.5f}" size="{} 0.0002" rgba="0.10 0.10 0.11 1" '
                'contype="0" conaffinity="0" mass="0"/>'.format(i, -L / 2 + MARK_INSET, H_PART / 2 + 0.0001, MARK_R))
        pg.append('<body name="part{}" pos="{} {} 0.05"><freejoint name="part{}_free"/>{}</body>'.format(
            i, 1.5 + 0.1 * i, 1.5, i, geoms.replace('condim="4"', 'condim="4" density="%.1f"' % dens) + mark))
    # static geoms (belt frame, rails, hood, chute, camera housing) go into one fixed body
    g = ['<body name="statics">' + "".join(x for x in g if x.startswith("<geom")) + "</body>"] + \
        [x for x in g if not x.startswith("<geom")]
    objs = mujoco.MjSpec.from_string('<mujoco><compiler angle="radian" autolimits="true"/><worldbody>{}</worldbody>'
                                     '</mujoco>'.format("".join(g) + "".join(pg)))
    for b in list(objs.worldbody.bodies):
        wb.add_frame().attach_body(b, "o_", "")

    # cameras
    pos, tgt, fovy = OVERVIEW_CAM
    c = wb.add_camera()
    c.name = "overview"
    c.pos = list(pos)
    c.quat = list(quat_from_mat(look_at(pos, tgt)))
    c.fovy = fovy
    c = wb.add_camera()
    c.name = "inspect"
    c.pos = list(UPCAM_POS)
    c.quat = list(quat_from_mat(look_at(UPCAM_POS, (UPCAM_POS[0], UPCAM_POS[1], UPCAM_POS[2] + 1.0), up=(1, 0, 0))))
    c.fovy = UPCAM_FOVY

    for act in root.actuators:
        if act.name.startswith("r_g_gripper_finger_joint"):
            act.gainprm[0] = GRIP_KP
            act.biasprm[1] = -GRIP_KP
    eq = root.add_equality()
    eq.type = mujoco.mjtEq.mjEQ_JOINT
    eq.name = "finger_coupling"
    eq.name1 = "r_g_finger_joint1"
    eq.name2 = "r_g_finger_joint2"
    eq.data[:5] = [0.0, -1.0, 0.0, 0.0, 0.0]
    eq.solref = list(FINGER_COUPLING)
    m = root.compile()
    m.cam_fovy[m.camera("r_eye_in_hand").id] = WRIST_FOVY
    m.site_rgba[:, 3] = 0.0
    m.light_castshadow[:] = 0
    m.vis.quality.offsamples = 0
    # near/far clipping planes are relative to the model's extent (the hall makes it ~12 m): keep close views
    m.vis.map.znear = 0.001
    m.vis.map.zfar = 1.0
    for name in ("r_g_finger1_pad_collision", "r_g_finger2_pad_collision"):
        m.geom_friction[m.geom(name).id, 0] = PAD_FRICTION
    gain = float(layout["light"])
    tint = np.asarray(layout.get("tint", (1.0, 1.0, 1.0)), float)
    for name in ("lamp", "fill"):
        lid = m.light(name).id
        m.light_diffuse[lid] = m.light_diffuse[lid] * gain * tint
    m.vis.headlight.diffuse[:] = np.asarray(m.vis.headlight.diffuse) * tint
    m.vis.headlight.ambient[:] = np.asarray(m.vis.headlight.ambient) * tint
    # the robot's collision meshes are never drawn (only its visual meshes)
    for gid in range(m.ngeom):
        bname = m.body(m.geom_bodyid[gid]).name
        if bname.startswith("r_") and m.geom_group[gid] == 0:
            m.geom_rgba[gid, 3] = 0.0
    return m


def sample_layout(seed, releases=RELEASES):
    """Everything a shift draws from its seed, except the sensor noise."""
    s = [seed & 0xFFFFFFFF, seed >> 32]
    r = np.random.default_rng(s + [11])
    kit = (KIT_NOMINAL[0] + r.uniform(-KIT_JITTER[0], KIT_JITTER[0]),
           KIT_NOMINAL[1] + r.uniform(-KIT_JITTER[1], KIT_JITTER[1]),
           KIT_NOMINAL[2] + r.uniform(-KIT_JITTER[2], KIT_JITTER[2]))
    parts = list(VARIANT_NAMES)
    while len(parts) < releases:
        parts.append(VARIANT_NAMES[int(r.integers(len(VARIANT_NAMES)))])
    parts = [parts[i] for i in r.permutation(len(parts))]
    rgba = []
    for v in parts:
        base = np.array(VARIANTS[v][4])
        rgba.append(list(np.clip(base + r.uniform(-COLOUR_TOL, COLOUR_TOL, 3), 0, 1)) + [1.0])
    light = float(r.uniform(*LIGHT_RANGE))
    speed = float(r.uniform(*BELT_SPEED))
    wob_period = float(r.uniform(*BELT_WOBBLE_PERIOD))
    wob_phase = float(r.uniform(0, 2 * math.pi))
    t_rel, t = [], float(r.uniform(*RELEASE_FIRST))
    for _ in parts:
        t_rel.append(t)
        t += float(r.uniform(*RELEASE_GAP))
    lateral = [float(r.uniform(-LATERAL, LATERAL)) for _ in parts]
    yaw = [float(r.uniform(-math.pi, math.pi)) for _ in parts]
    flipped = [bool(r.random() < FLIP_P) for _ in parts]
    rc = np.random.default_rng(s + [13])
    cams = {}
    for name in CAMERA_NAMES:
        tol = CAM_TOL[name]
        ax = rc.normal(size=3)
        cams[name] = {"dpos": list(rc.uniform(-tol["pos"], tol["pos"], 3)),
                      "drot": list(ax / np.linalg.norm(ax) * math.radians(rc.uniform(-tol["rot_deg"], tol["rot_deg"])))}
    rp = np.random.default_rng(s + [29])
    pocket_off = [tuple(float(u) for u in rp.uniform(-POCKET_TOL, POCKET_TOL, 2)) for _ in POCKETS]
    rv = np.random.default_rng(s + [31])          # v4: the shift's hidden variation of the machine itself
    latency = int(rv.integers(LATENCY_RANGE[0], LATENCY_RANGE[1] + 1))
    arm = {"gain": float(rv.uniform(*ARM_GAIN_RANGE)), "zeta": float(rv.uniform(*ARM_DAMPING_RANGE)),
           "gcomp": float(rv.uniform(-GRAVITY_COMP_ERR, GRAVITY_COMP_ERR))}
    drift = float(rv.uniform(-BELT_DRIFT, BELT_DRIFT))
    tint = [float(u) for u in 1.0 + rv.uniform(-LIGHT_TINT, LIGHT_TINT, 3)]
    friction = float(rv.uniform(*FRICTION_RANGE))
    return {"kit": kit, "pocket_off": pocket_off, "parts": parts, "rgba": rgba, "light": light, "speed": speed,
            "wobble": (wob_period, wob_phase), "latency": latency, "arm": arm, "drift": drift, "tint": tint,
            "friction": friction,
            "release_t": t_rel, "lateral": lateral, "yaw": yaw, "flipped": flipped, "cams": cams}


class CellEnv(object):
    """One shift of the conveyor kitting cell."""

    def __init__(self, render=True, releases=RELEASES, tolerances=True):
        self.render_enabled = render
        self.releases = int(releases)
        self.tolerances = bool(tolerances)
        self.model = None
        self._renderers = {}

    # ------------------------------------------------------------------ episode
    def reset(self, seed, bench=False):
        self.seed = int(seed) & (2 ** 64 - 1)
        s = [self.seed & 0xFFFFFFFF, self.seed >> 32]
        self.layout = sample_layout(self.seed, self.releases)
        self.bench = bool(bench)
        self.noise_rng = np.random.default_rng(s + [17])
        self.close()
        self.model = build_model(self.layout)
        if self.tolerances:
            self._apply_camera_tolerances()
        self.data = mujoco.MjData(self.model)
        self.fk = mujoco.MjData(self.model)
        self._index()
        self._init_state()
        wr = np.random.default_rng(s + [19])
        self.wrench_bias = np.concatenate([wr.uniform(-WRENCH_BIAS[0], WRENCH_BIAS[0], 3),
                                           wr.uniform(-WRENCH_BIAS[1], WRENCH_BIAS[1], 3)])
        self.wrench_drift = np.concatenate([wr.uniform(-WRENCH_DRIFT[0], WRENCH_DRIFT[0], 3),
                                            wr.uniform(-WRENCH_DRIFT[1], WRENCH_DRIFT[1], 3)])
        self.t = 0
        self.time = 0.0
        self.belt_travel = 0.0
        self.pending = deque([(np.zeros(3), 0.0, -1.0)] * int(self.layout["latency"]))
        n = len(self.layout["parts"])
        self.status = ["parked"] * n              # parked -> released -> lost
        self.damaged = [False] * n
        self.force_count = [0] * n
        self.max_force = [0.0] * n
        self.next_release = 0 if not self.bench else n
        last = self.layout["release_t"][-1]
        self.t_end = last + (RELEASE_Y - PICK_ZONE[1]) / (self.layout["speed"] * (1 - BELT_WOBBLE - BELT_DRIFT)) + SHIFT_GRACE
        self.station_busy = False
        self.station_cycles = 0
        self.station_refused = 0
        self.station_part = None
        self.done = False
        self.fault = None
        self.events = []
        self.last_cams = 7
        return self.observation(7)

    def _apply_camera_tolerances(self):
        m = self.model
        for name in CAMERA_NAMES:
            cid = m.camera("r_eye_in_hand" if name == "wrist" else name).id
            d = self.layout["cams"][name]
            m.cam_pos[cid] = m.cam_pos[cid] + np.array(d["dpos"])
            m.cam_quat[cid] = quat_from_mat(axis_angle_mat(d["drot"]) @ mat_from_quat(m.cam_quat[cid])) \
                if name != "wrist" else quat_from_mat(mat_from_quat(m.cam_quat[cid]) @ axis_angle_mat(d["drot"]))

    def _index(self):
        m = self.model
        self.jq = np.array([m.joint("r_joint%d" % i).qposadr[0] for i in range(1, 8)])
        self.jv = np.array([m.joint("r_joint%d" % i).dofadr[0] for i in range(1, 8)])
        self.act = np.array([m.actuator("r_torq_j%d" % i).id for i in range(1, 8)])
        self.gact = [m.actuator("r_g_gripper_finger_joint1").id, m.actuator("r_g_gripper_finger_joint2").id]
        self.gq = [m.joint("r_g_finger_joint1").qposadr[0], m.joint("r_g_finger_joint2").qposadr[0]]
        self.site = m.site("r_g_grip_site").id
        self.ft_site = m.site("r_g_ft_frame").id
        self.ft_force = m.sensor("r_g_force_ee").adr[0]
        self.ft_torque = m.sensor("r_g_torque_ee").adr[0]
        self.belt_q = m.joint("o_belt_slide").qposadr[0]
        self.belt_v = m.joint("o_belt_slide").dofadr[0]
        self.stripes_mocap = m.body_mocapid[m.body("o_stripes").id]
        self.flip_q = m.joint("o_flip_hinge").qposadr[0]
        self.flip_v = m.joint("o_flip_hinge").dofadr[0]
        n = len(self.layout["parts"])
        self.part_bid = [m.body("o_part%d" % i).id for i in range(n)]
        self.part_qadr = [int(m.joint("o_part%d_free" % i).qposadr[0]) for i in range(n)]
        self.part_dadr = [int(m.joint("o_part%d_free" % i).dofadr[0]) for i in range(n)]
        self.part_geoms = [np.where(m.geom_bodyid == b)[0] for b in self.part_bid]
        self.geom_part = -np.ones(m.ngeom, dtype=int)
        for i, gs in enumerate(self.part_geoms):
            self.geom_part[gs] = i
        kit_b = m.body("o_kit").id
        self.geom_kit = (m.geom_bodyid == kit_b)
        fingers = [b for b in range(m.nbody) if m.body(b).name.startswith("r_g_") and "finger" in m.body(b).name]
        self.geom_finger = np.isin(m.geom_bodyid, fingers)
        self.part_contype = {i: (m.geom_contype[gs].copy(), m.geom_conaffinity[gs].copy())
                             for i, gs in enumerate(self.part_geoms)}
        self.kit_pose = tuple(self.layout["kit"])
        self.pockets = [pocket_world(self.kit_pose, k, self.layout["pocket_off"]) for k in range(len(POCKETS))]
        self.floor_z = TABLE_Z + 2 * KIT_PLATE[2]
        self.nest_floor = TABLE_Z + STATION_BASE_H
        self.hinge = np.array([STATION[0], STATION[1], self.nest_floor + H_PART / 2])
        self._jp, self._jr = np.zeros((3, m.nv)), np.zeros((3, m.nv))
        self._M = np.zeros((m.nv, m.nv))

    def _init_state(self):
        m, d = self.model, self.data
        mujoco.mj_resetData(m, d)
        d.qpos[self.jq] = HOME_Q
        d.qpos[self.gq[0]], d.qpos[self.gq[1]] = 0.04, -0.04
        for i in range(len(self.part_bid)):
            self._park(i)
        mujoco.mj_forward(m, d)
        self.q_stale = d.qpos.copy()
        self.grip_target = 0.04
        q_hold = d.qpos[self.jq].copy()
        tare = []
        for k in range(150):
            d.ctrl[self.act] = 200.0 * (q_hold - d.qpos[self.jq]) - 20.0 * d.qvel[self.jv] + d.qfrc_bias[self.jv]
            d.ctrl[self.gact] = [0.04, -0.04]
            self.q_stale[:] = d.qpos
            mujoco.mj_step(m, d)
            if k >= 100:
                tare.append(self._raw_wrench())
        self.tare = np.mean(tare, axis=0)
        self.target_pos = d.site_xpos[self.site].copy()
        R = d.site_xmat[self.site].reshape(3, 3).copy()
        self.R_down = rot_z(-yaw_of_mat(R)) @ R
        self.target_yaw = yaw_of_mat(R)

    def _park(self, i):
        d = self.data
        a, v = self.part_qadr[i], self.part_dadr[i]
        d.qpos[a:a + 3] = [1.5 + 0.1 * i, 1.5, 0.05]
        d.qpos[a + 3:a + 7] = [1, 0, 0, 0]
        d.qvel[v:v + 6] = 0.0

    def set_part_pose(self, i, pos, R, vel=(0, 0, 0)):
        d = self.data
        a, v = self.part_qadr[i], self.part_dadr[i]
        d.qpos[a:a + 3] = pos
        d.qpos[a + 3:a + 7] = quat_from_mat(R)
        d.qvel[v:v + 6] = 0.0
        d.qvel[v:v + 3] = vel

    def belt_speed(self, t=None):
        t = self.time if t is None else t
        per, ph = self.layout["wobble"]
        drift = self.layout["drift"] * min(t / BELT_DRIFT_S, 1.0)
        return self.layout["speed"] * (1.0 + BELT_WOBBLE * math.sin(2 * math.pi * t / per + ph)) * (1.0 + drift)

    def _release(self, i):
        lay = self.layout
        R = rot_z(lay["yaw"][i])
        if lay["flipped"][i]:
            R = R @ rot_x(math.pi)
        self.set_part_pose(i, [BELT_X + lay["lateral"][i], RELEASE_Y, BELT_Z + H_PART / 2 + 0.0004], R,
                           (0.0, -self.belt_speed(), 0.0))
        self.status[i] = "released"
        self.events.append({"t": self.t, "time": round(self.time, 2), "event": "release", "part": i})

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
        arm = self.layout["arm"]
        kp, ko, z = 400.0 * arm["gain"], 200.0 * arm["gain"], arm["zeta"]
        acc = np.concatenate([kp * ep - 2.0 * z * math.sqrt(kp) * v[:3], ko * eo - 2.0 * z * math.sqrt(ko) * v[3:]])
        Minv = np.linalg.inv(Mq)
        Lam = np.linalg.inv(J @ Minv @ J.T + 1e-2 * np.eye(6))
        tau = J.T @ (Lam @ acc) + (1.0 + arm["gcomp"]) * d.qfrc_bias[self.jv]
        Jbar = Minv @ J.T @ Lam
        N = np.eye(7) - J.T @ Jbar.T
        qn = d.qpos[self.jq]
        dq = d.qvel[self.jv]
        tau += N @ (5.0 * (NULLSPACE_Q - qn) - 6.0 * dq)
        tau -= 0.5 * dq
        over = np.abs(dq) - 1.5
        tau -= np.where(over > 0, 30.0 * over * np.sign(dq), 0.0)
        return tau

    @property
    def target_R(self):
        return rot_z(self.target_yaw) @ self.R_down

    def step(self, action):
        if self.done:
            raise EnvError("shift already finished")
        dp_new, dyaw_new, g_new, cams, turn = parse_action(action)
        self.pending.append((dp_new, dyaw_new, g_new))
        dp, dyaw, g = self.pending.popleft()
        d = self.data
        n = np.linalg.norm(dp)
        if n > MAX_DPOS:
            dp = dp * (MAX_DPOS / n)
        dyaw = float(np.clip(dyaw, -MAX_DYAW, MAX_DYAW))
        cur_p = d.site_xpos[self.site].copy()
        self.target_pos = np.clip(cur_p + dp, WORKSPACE_LO, WORKSPACE_HI)
        self.target_yaw = wrap_angle(self.ee_yaw() + dyaw)
        self.grip_target = 0.04 * (1.0 - g) / 2.0
        if turn:
            self._station_request()
        nsub = SUBSTEPS + CAM_COST_SUBSTEPS * sum(1 for b in (1, 2, 4) if cams & b)
        self._physics(nsub)
        self.t += 1
        self._post_step()
        self.last_cams = cams
        return self.observation(cams), self.done, self.info()

    def _physics(self, nsub):
        m, d = self.model, self.data
        gw = [self.grip_target, -self.grip_target]
        near = self._parts_near_kit()
        fsum = np.zeros(len(self.part_bid))
        for _ in range(nsub):
            while self.next_release < len(self.part_bid) and self.time >= self.layout["release_t"][self.next_release]:
                self._release(self.next_release)
                self.next_release += 1
            vb = self.belt_speed()
            d.qpos[self.belt_q] = 0.0
            d.qvel[self.belt_v] = -vb
            if self.station_busy:
                self._station_kinematics()
            d.ctrl[self.act] = self._osc_torque()
            d.ctrl[self.gact] = gw
            self.q_stale[:] = d.qpos
            mujoco.mj_step(m, d)
            self.time += TIMESTEP
            self.belt_travel += vb * TIMESTEP
            if near:
                fsum += self._part_kit_force()
        d.qpos[self.belt_q] = 0.0
        d.mocap_pos[self.stripes_mocap, 1] = -(self.belt_travel % STRIPE_PITCH)
        mujoco.mj_kinematics(m, d)
        self.step_force = fsum / nsub
        if not np.all(np.isfinite(d.qpos)) or not np.all(np.isfinite(d.qvel)):
            self.fault = "non-finite state"

    def _parts_near_kit(self):
        d = self.data
        kx, ky, _ = self.kit_pose
        for b in self.part_bid:
            p = d.xpos[b]
            if abs(p[0] - kx) < 0.20 and abs(p[1] - ky) < 0.16 and p[2] < self.floor_z + 0.05:
                return True
        return False

    def _part_kit_force(self):
        d = self.data
        n = d.ncon
        out = np.zeros(len(self.part_bid))
        if n == 0:
            return out
        geom = d.contact.geom[:n]
        adr = d.contact.efc_address[:n]
        ok = adr >= 0
        g1, g2 = geom[:, 0], geom[:, 1]
        p1, p2 = self.geom_part[g1], self.geom_part[g2]
        k1, k2 = self.geom_kit[g1], self.geom_kit[g2]
        sel1 = ok & (p1 >= 0) & k2
        sel2 = ok & (p2 >= 0) & k1
        fn = d.efc_force[np.where(ok, adr, 0)]
        if sel1.any():
            np.add.at(out, p1[sel1], fn[sel1])
        if sel2.any():
            np.add.at(out, p2[sel2], fn[sel2])
        return out

    # ------------------------------------------------------------------ turnover station
    def _part_in_nest(self, i, nest):
        d = self.data
        p = d.xpos[self.part_bid[i]]
        cy = STATION[1] + (NEST_OFF if nest == 0 else -NEST_OFF)
        return (abs(p[0] - STATION[0]) < NEST_IN[0] and abs(p[1] - cy) < NEST_IN[1]
                and p[2] < self.nest_floor + H_PART + 0.004)

    def _station_request(self):
        d = self.data
        ok = not self.station_busy
        inp = [i for i in range(len(self.part_bid)) if self.status[i] == "released" and self._part_in_nest(i, 0)]
        outp = [i for i in range(len(self.part_bid)) if self.status[i] == "released" and self._part_in_nest(i, 1)]
        hp = d.site_xpos[self.site]
        clear = (math.hypot(hp[0] - STATION[0], hp[1] - STATION[1]) > GUARD_R or hp[2] > self.nest_floor + GUARD_Z)
        if ok and len(inp) == 1 and not outp and clear:
            i = inp[0]
            v = d.qvel[self.part_dadr[i]:self.part_dadr[i] + 3]
            if np.linalg.norm(v) < 0.02 and not self.touching_fingers(i):
                self.station_busy = True
                self.station_part = i
                self.station_t0 = self.time
                p, R = self.part_pose(i)
                self.station_p0, self.station_R0 = p, R
                gs = self.part_geoms[i]
                self.model.geom_contype[gs] = 0
                self.model.geom_conaffinity[gs] = 0
                self.events.append({"t": self.t, "time": round(self.time, 2), "event": "turn", "part": i})
                return
        if not self.station_busy:
            self.station_refused += 1

    def _station_kinematics(self):
        d = self.data
        c, tu, r = (k * CONTROL_DT for k in TURN_STEPS)
        e = self.time - self.station_t0
        i = self.station_part
        if e < c:
            phi = 0.0
        elif e < c + tu:
            x = (e - c) / tu
            phi = math.pi * (0.5 - 0.5 * math.cos(math.pi * x))
        else:
            phi = math.pi
        if i is not None:
            if phi < math.pi:
                Rx = rot_x(phi)
                self.set_part_pose(i, self.hinge + Rx @ (self.station_p0 - self.hinge), Rx @ self.station_R0)
            else:
                Rx = rot_x(math.pi)
                p = self.hinge + Rx @ (self.station_p0 - self.hinge)
                p[2] = max(p[2], self.nest_floor + H_PART / 2) + 0.0015
                self.set_part_pose(i, p, Rx @ self.station_R0)
                gs = self.part_geoms[i]
                ct, ca = self.part_contype[i]
                self.model.geom_contype[gs] = ct
                self.model.geom_conaffinity[gs] = ca
                self.station_part = None
                self.station_cycles += 1
        if e >= c + tu:
            x = min(1.0, (e - c - tu) / r)
            phi = math.pi * (1.0 - x)
            if x >= 1.0:
                self.station_busy = False
        d.qpos[self.flip_q] = phi
        d.qvel[self.flip_v] = 0.0

    # ------------------------------------------------------------------ credit
    def ee_yaw(self):
        return yaw_of_mat(self.data.site_xmat[self.site].reshape(3, 3) @ self.R_down.T)

    def part_pose(self, i):
        d = self.data
        b = self.part_bid[i]
        return d.xpos[b].copy(), d.xmat[b].reshape(3, 3).copy()

    def touching_fingers(self, i):
        d = self.data
        n = d.ncon
        if n == 0:
            return False
        geom = d.contact.geom[:n]
        g1, g2 = geom[:, 0], geom[:, 1]
        return bool(np.any((self.geom_part[g1] == i) & self.geom_finger[g2]) or
                    np.any((self.geom_part[g2] == i) & self.geom_finger[g1]))

    def seated_in(self, i, k):
        """Whether part i sits fully seated, key aligned, in pocket k (geometry only)."""
        v, _px, _py, _pyaw = POCKETS[k]
        if self.layout["parts"][i] != v:
            return False
        px, py, pyaw = self.pockets[k]
        p, R = self.part_pose(i)
        if math.hypot(p[0] - px, p[1] - py) > SEAT_XY:
            return False
        if abs(p[2] - (self.floor_z + H_PART / 2)) > SEAT_Z:
            return False
        if R[2, 2] < math.cos(SEAT_TILT):
            return False
        if abs(wrap_angle(math.atan2(R[1, 0], R[0, 0]) - pyaw)) > SEAT_YAW:
            return False
        vel = self.data.qvel[self.part_dadr[i]:self.part_dadr[i] + 3]
        return float(np.linalg.norm(vel)) < REST_SPEED

    def inspect_kit(self):
        """The kit inspection: per pocket 'seated', 'damaged' (a damaged part seated), 'unseated' (its part in the
        pocket area but not seated), or 'empty'."""
        out = []
        for k in range(len(POCKETS)):
            px, py, _ = self.pockets[k]
            st = "empty"
            for i in range(len(self.part_bid)):
                if self.status[i] != "released":
                    continue
                p, _R = self.part_pose(i)
                if math.hypot(p[0] - px, p[1] - py) > 0.03 or p[2] > self.floor_z + 0.04:
                    continue
                if self.seated_in(i, k) and not self.touching_fingers(i):
                    st = "damaged" if self.damaged[i] else "seated"
                    break
                st = "unseated"
            out.append(st)
        return out

    def credit(self):
        return sum(1 for s in self.inspect_kit() if s == "seated")

    def _post_step(self):
        d = self.data
        for i in range(len(self.part_bid)):
            if self.status[i] != "released":
                continue
            p = d.xpos[self.part_bid[i]]
            on_belt = abs(p[0] - BELT_X) < RAIL_GAP + 0.01 and BELT_Z - 0.01 < p[2] < BELT_Z + 0.03
            if (on_belt and p[1] < PICK_ZONE[1]) or p[2] < TABLE_Z - 0.05:
                self.status[i] = "lost"
                self._park(i)
                self.events.append({"t": self.t, "time": round(self.time, 2), "event": "lost", "part": i})
        f = self.step_force
        for i in range(len(self.part_bid)):
            self.max_force[i] = max(self.max_force[i], float(f[i]))
            self.force_count[i] = self.force_count[i] + 1 if f[i] > DAMAGE_FORCE else 0
            if self.force_count[i] >= DAMAGE_STEPS and not self.damaged[i]:
                self.damaged[i] = True
                self.events.append({"t": self.t, "time": round(self.time, 2), "event": "damaged", "part": i})
        if self.fault is None:
            if np.max(np.abs(d.qvel[self.jv])) > INSTAB_ROBOT_QVEL:
                self.fault = "robot joint speed"
            elif any(np.linalg.norm(d.qvel[a:a + 3]) > INSTAB_PART_VEL for a in self.part_dadr):
                self.fault = "part speed"
        if self.fault is not None or (not self.bench and self.time >= self.t_end):
            self.done = True

    def info(self):
        return {"t": self.t, "time": round(self.time, 3), "fault": self.fault, "station_busy": int(self.station_busy)}

    # ------------------------------------------------------------------ sensing
    def close(self):
        for r in self._renderers.values():
            try:
                r.close()
            except Exception:  # noqa: BLE001
                pass
        self._renderers = {}

    def render(self, cam, data=None):
        w, h = IMG[cam]
        if cam not in self._renderers:
            self._renderers[cam] = mujoco.Renderer(self.model, h, w)
        name = {"overview": "overview", "wrist": "r_eye_in_hand", "inspect": "inspect"}[cam]
        r = self._renderers[cam]
        r.update_scene(self.data if data is None else data, name)
        return r.render().copy()

    def _raw_wrench(self):
        d = self.data
        Rft = d.site_xmat[self.ft_site].reshape(3, 3)
        return np.concatenate([Rft @ d.sensordata[self.ft_force:self.ft_force + 3],
                               Rft @ d.sensordata[self.ft_torque:self.ft_torque + 3]])

    def wrench(self):
        zero = self.wrench_bias + self.wrench_drift * self.time
        w = -(self._raw_wrench() - self.tare) + zero      # the load the tool receives: pressing down reads +z
        w[:3] += self.noise_rng.normal(0, WRENCH_NOISE[0], 3)
        w[3:] += self.noise_rng.normal(0, WRENCH_NOISE[1], 3)
        return np.round(w, 4)

    def observation(self, cams):
        d = self.data
        R = d.site_xmat[self.site].reshape(3, 3)
        obs = {"t": int(self.t),
               "time": round(float(self.time), 4),
               "q": np.round(d.qpos[self.jq], 6),
               "dq": np.round(d.qvel[self.jv], 5),
               "ee_pos": np.round(d.site_xpos[self.site], 6),
               "ee_quat": np.round(quat_from_mat(R), 6),
               "ee_yaw": round(float(self.ee_yaw()), 6),
               "gripper": round(float(d.qpos[self.gq[0]] - d.qpos[self.gq[1]]), 6),
               "wrench": self.wrench() if self.model is not None else np.zeros(6),
               "station_busy": int(self.station_busy),
               "station_cycles": int(self.station_cycles)}
        for cam in CAMERA_NAMES:
            if self.render_enabled and (cams & CAM_BITS[cam]):
                obs[cam] = self.render(cam)
            else:
                obs[cam] = np.zeros((0, 0, 3), np.uint8)
        return obs

    def perturbed_observation(self, obs, rng):
        """A probe copy for the grader's lock: the observation of a slightly different arm pose (joint readings offset
        by up to 0.004 rad; hand pose and the images that were taken are those of the offset pose, computed from the
        joint positions before the last physics step, as the real observation's are) and a slightly different wrench."""
        dq = np.array([rng.uniform(-0.004, 0.004) for _ in range(7)])
        f = self.fk
        f.qpos[:] = self.q_stale
        f.qpos[self.jq] = self.q_stale[self.jq] + dq
        f.mocap_pos[:] = self.data.mocap_pos
        mujoco.mj_kinematics(self.model, f)
        mujoco.mj_comPos(self.model, f)
        mujoco.mj_camlight(self.model, f)
        R = f.site_xmat[self.site].reshape(3, 3)
        out = dict(obs)
        if self.render_enabled:
            for cam in CAMERA_NAMES:
                if np.asarray(obs[cam]).size:
                    out[cam] = self.render(cam, f)
        out["q"] = np.round(np.asarray(obs["q"], float) + dq, 6)
        out["ee_pos"] = np.round(f.site_xpos[self.site], 6)
        out["ee_quat"] = np.round(quat_from_mat(R), 6)
        out["ee_yaw"] = round(float(yaw_of_mat(R @ self.R_down.T)), 6)
        w = np.asarray(obs["wrench"], float) + np.array([rng.uniform(-0.05, 0.05) for _ in range(6)]) * \
            np.array([1, 1, 1, 0.1, 0.1, 0.1])
        out["wrench"] = np.round(w, 4)
        return out

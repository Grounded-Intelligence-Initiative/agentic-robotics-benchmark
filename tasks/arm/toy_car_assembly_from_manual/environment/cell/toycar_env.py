"""Toy car assembly cell: a Franka Panda builds a children's wooden toy car from loose kit parts.

Where this file runs: in the grader (tests/toycar_env.py) and, as a byte copy (environment/cell/toycar_env.py), in
the practice cell of the agent's sandbox (/opt/toycell, readable by the practice cell's own account only, building
the practice kit only). The agent never reads it. The graded kit's numbers are not in this file: they are in the kit
file the grader holds (tests/kit.json), and the agent gets only that kit's picture manual.

The robot, its hand and its tracking law follow tasks/arm/keyed_nut_peg_assembly (robosuite 1.5.2's Panda assets,
an operational-space controller at 10 Hz); the scene, the kit, the latches and the roll test are this task's own.

Frames and units: world frame, metres, radians, seconds; quaternions (w, x, y, z). The table top is z = 0.80.
The Panda base stands at (-0.56, 0, 0.80) facing +x. The assembly stand is a fixed post on the table at
STAND_XY whose top is STAND_H above the table.

    env = ToyCarEnv(kit)                 # kit: a dict or a path to a kit JSON file
    obs = env.reset(seed)
    obs, done, info = env.step(action)   # 10 Hz

Action (a list of 6 numbers): [dx, dy, dz, dyaw, dtilt, grip]
    dx..dz: displacement of the grip point's target, world frame (m; the vector is clipped to MAX_DPOS);
    dyaw:   rotation of the hand about the world vertical (rad, clipped to MAX_DYAW);
    dtilt:  change of the hand's tilt (rad, clipped to MAX_DTILT), kept in [-TILT_MAX, TILT_MAX]. Tilt 0 is the
            hand pointing straight down; tilting turns the hand about its finger-closing axis, so at tilt +-pi/2 it
            points horizontally (at yaw 0: tilt +pi/2 points along +y, tilt -pi/2 along -y);
    grip:   -1 = open (80 mm between the fingers) ... +1 = closed, linear in between.

Latching: a part that is within the tolerances of a free mate pose of its type, and is let go (the grip command is
<= 0, or the part touches neither finger), snaps onto that pose and stays there (a press fit). Parts of the other model in the box fit the same
mates. An episode ends when every mate is filled, after MAX_STEPS steps, or on a fault. Then the roll test:
the latched car is set on the table in a separate clean simulation and pushed (see `roll_test`).
"""

import json
import math
import os
import re

import numpy as np

import mujoco

# ----------------------------------------------------------------------------------------------
# Public constants
# ----------------------------------------------------------------------------------------------
TIMESTEP = 0.002
SUBSTEPS = 50                    # -> 10 Hz control
CONTROL_DT = TIMESTEP * SUBSTEPS
MAX_STEPS = 3000                 # 300 s of cell time per episode
MAX_DPOS = 0.02
MAX_DYAW = 0.15
MAX_DTILT = 0.15
TILT_MAX = math.pi / 2
TABLE_Z = 0.80
TABLE_HALF = (0.45, 0.60)
ROBOT_BASE = np.array([-0.56, 0.0, TABLE_Z])
STAND_XY = (0.06, 0.0)           # nominal; each episode moves the stand (STAND_JITTER)
STAND_JITTER = (0.025, 0.025, math.radians(8.0))
STAND_H = 0.10                   # the stand's top above the table
STAND_TOP = (0.030, 0.016)       # half sizes of the stand's top plate (smaller than any chassis)
LAYOUT_X = (-0.22, 0.22)         # where the loose parts lie (their centres)
LAYOUT_Y = 0.33
LAYOUT_R = (0.36, 0.74)          # distance from the robot base
WORKSPACE_LO = np.array([-0.40, -0.42, TABLE_Z + 0.004])
WORKSPACE_HI = np.array([0.32, 0.42, TABLE_Z + 0.45])

# The overview camera (nominal pose; each episode re-seats it: OVERVIEW_JITTER) looks down; image right is world
# -y, image up is world +x.
OVERVIEW = {"pos": (-0.03, 0.0, 1.62), "xaxis": (0.0, -1.0, 0.0), "yaxis": (1.0, 0.0, 0.0), "fovy": 48.0,
            "width": 224, "height": 224}
OVERVIEW_JITTER = {"pos": (0.04, 0.04, 0.04), "tilt": math.radians(4.0), "roll": math.radians(6.0)}
WRIST = {"fovy": 75.0, "width": 96, "height": 96}
CAMERA_NAMES = ("overview", "wrist")

# Tolerances of the latches (position in m, angle in rad)
LATCH = {
    "chassis": {"pos": 0.006, "ang": math.radians(8.0)},
    "axle": {"pos": 0.005, "ang": math.radians(8.0)},
    "wheel": {"pos": 0.005, "ang": math.radians(10.0)},
    "cabin": {"pos": 0.005, "ang": math.radians(8.0)},
}
LATCH_Z = 0.006                  # resting parts (chassis, cabin): centre at most this above the mate

# Press fits: an axle into its notch and a wheel onto its axle end. Near its seat the part meets a detent lip
# PRESS[...]["lip"] short of home; it passes only when pushed through with at least F_SEAT, held for YIELD_S; a push
# that reaches F_DAMAGE before the lip gives (a blow) damages the part, which then never seats. Past the lip the
# part slides home and seats (latches) once within HOME_TOL of home, with no need to let go. Until then a lead-in
# centres it (the notch's chamfer, the wheel's bore on the axle stub): a part let go at the lip stays there, unseated.
PRESS = {"axle": {"lip": 0.004, "top": 0.016, "lat": 0.001, "ang": math.radians(3.0)},
         "wheel": {"lip": 0.003, "lat": 0.003, "ang": math.radians(6.0)}}
N_DETENT_BARS, N_DETENT_COLLARS = 2, 4
LATCHED_MASS_SCALE = float(os.environ.get("TC_LMS", "1"))   # >1 makes latched parts solid to the fingers (the reference then needs another axle press)
AXLE_ALONG_TOL = 0.004           # how far off centre (along the notch) an axle may sit and still seat
F_SEAT = 10.0                    # N
F_DAMAGE = 30.0                  # N
YIELD_S = 0.030                  # s the push must hold F_SEAT..F_DAMAGE before the lip gives
DAMAGE_S = 0.020                 # s at F_DAMAGE or more that break the part
HOME_TOL = 0.0006                # m


# Paint of each part type, shared by both models in the box (the per-episode look jitters it slightly)
PAINT = {
    "chassis": (0.74, 0.13, 0.10),
    "axle": (0.90, 0.80, 0.60),
    "wheel": (0.07, 0.07, 0.08),
    "cabin": (0.16, 0.36, 0.78),
}
HUB_RGB = (0.30, 0.30, 0.32)
WOOD_DENSITY = 650.0

GRIP_KP = 3000.0
FINGER_COUPLING = (0.005, 1.0)
INSTAB_ROBOT_QVEL = 8.0
INSTAB_PART_VEL = 6.0
HOME_Q = np.array([0.0, -0.35, 0.0, -2.35, 0.0, 2.0, math.pi / 4])
NULLSPACE_Q = np.array([0.0, -0.3, 0.0, -2.2, 0.0, 1.9, math.pi / 4])

ROLL_PUSH_SPEED = 0.6            # m/s along the car's length
ROLL_TIME = 2.0                  # s
ROLL_MIN_TRAVEL = 0.25           # m
ROLL_MAX_TILT = math.radians(15.0)


class EnvError(Exception):
    pass


class ActionError(ValueError):
    pass


# ----------------------------------------------------------------------------------------------
# Small helpers
# ----------------------------------------------------------------------------------------------
def look_at(pos, target, up=(0, 0, 1)):
    """Rotation whose columns are the MuJoCo camera axes (x right, y up, z backwards)."""
    pos, target = np.asarray(pos, float), np.asarray(target, float)
    f = target - pos
    f = f / np.linalg.norm(f)
    z = -f
    up = np.asarray(up, float)
    if abs(np.dot(up, z)) > 0.99:
        up = np.array([1.0, 0.0, 0.0])
    x = np.cross(up, z)
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


def rot_axis(axis, a):
    axis = np.asarray(axis, float) / np.linalg.norm(axis)
    K = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    return np.eye(3) + math.sin(a) * K + (1 - math.cos(a)) * (K @ K)


def wrap_angle(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def yaw_quat(yaw):
    return np.array([math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)])


def yaw_of_mat(R):
    return math.atan2(R[1, 0], R[0, 0])


def axis_angle_error(a, b):
    """Angle between two lines (directions a and b, either sign)."""
    c = abs(float(np.dot(a, b)) / (np.linalg.norm(a) * np.linalg.norm(b)))
    return math.acos(min(1.0, c))


def yaw_error_pi(a, b):
    d = (a - b) % math.pi
    return min(d, math.pi - d)


def robosuite_assets():
    import importlib.util
    spec = importlib.util.find_spec("robosuite")
    if spec is None or not spec.submodule_search_locations:
        raise EnvError("robosuite assets not installed")
    return os.path.join(list(spec.submodule_search_locations)[0], "models", "assets")


def parse_action(a):
    if not isinstance(a, (list, tuple)) or len(a) != 6:
        raise ActionError("action must be a list of 6 numbers")
    out = []
    for v in a:
        if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(float(v)) or abs(v) > 1e3:
            raise ActionError("action must be a list of 6 numbers")
        out.append(float(v))
    return np.array(out[:3]), out[3], out[4], float(np.clip(out[5], -1.0, 1.0))


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


def load_kit(kit):
    if isinstance(kit, str):
        with open(kit) as f:
            kit = json.load(f)
    need = {"chassis": ("length", "width", "base_t", "block_h", "notch_x", "notch_w"),
            "axle": ("length", "dia"), "wheel": ("dia", "thick"), "cabin": ("length", "width", "height")}
    for part, keys in need.items():
        for k in keys:
            float(kit[part][k])
    return kit


# ----------------------------------------------------------------------------------------------
# Procedural textures (deterministic; no image files)
# ----------------------------------------------------------------------------------------------
_TEX_CACHE = {}


def _smooth_noise(rng, h, w, cells):
    g = rng.random((cells + 1, cells + 1))
    ys = np.linspace(0, cells, h, endpoint=False)
    xs = np.linspace(0, cells, w, endpoint=False)
    y0, x0 = ys.astype(int), xs.astype(int)
    fy, fx = (ys - y0)[:, None], (xs - x0)[None, :]
    fy, fx = fy * fy * (3 - 2 * fy), fx * fx * (3 - 2 * fx)
    a = g[y0][:, x0]
    b = g[y0][:, x0 + 1]
    c = g[y0 + 1][:, x0]
    d = g[y0 + 1][:, x0 + 1]
    return a * (1 - fx) * (1 - fy) + b * fx * (1 - fy) + c * (1 - fx) * fy + d * fx * fy


def _fbm(rng, h, w, octaves=((4, 1.0), (8, 0.5), (16, 0.25), (32, 0.12))):
    out = np.zeros((h, w))
    for cells, amp in octaves:
        out += amp * _smooth_noise(rng, h, w, cells)
    return out / sum(a for _c, a in octaves)


def wood_texture(seed, size, base, dark, rings=22.0, stretch=6.0):
    """A plank of wood grain: rings of a stretched, noise-warped distance field."""
    rng = np.random.default_rng(seed)
    h = w = size
    yy, xx = np.mgrid[0:h, 0:w] / float(size)
    warp = _fbm(rng, h, w)
    r = np.sqrt(((yy - 0.37) * stretch) ** 2 + ((xx - 0.5) * 0.35) ** 2) + 0.9 * warp
    grain = 0.5 + 0.5 * np.sin(2 * math.pi * rings * r)
    grain = grain ** 3
    fine = _fbm(rng, h, w, ((64, 1.0), (128, 0.6)))
    t = np.clip(0.75 * grain + 0.25 * fine, 0, 1)[..., None]
    img = np.asarray(base)[None, None, :] * (1 - t) + np.asarray(dark)[None, None, :] * t
    return np.clip(img * 255, 0, 255).astype(np.uint8)


def plaster_texture(seed, size, base):
    rng = np.random.default_rng(seed)
    n = _fbm(rng, size, size, ((8, 1.0), (32, 0.5), (96, 0.35)))
    img = np.asarray(base)[None, None, :] * (0.92 + 0.12 * n[..., None])
    return np.clip(img * 255, 0, 255).astype(np.uint8)


def planks_texture(seed, size, base, dark, planks=4):
    rng = np.random.default_rng(seed)
    img = np.zeros((size, size, 3), np.uint8)
    ph = size // planks
    for k in range(planks):
        tile = wood_texture(seed * 31 + k, size, base * (0.9 + 0.2 * rng.random()), dark, rings=10.0, stretch=3.0)
        img[k * ph:(k + 1) * ph] = tile[:ph]
        img[k * ph:k * ph + 2] = (np.asarray(dark) * 0.6 * 255).astype(np.uint8)
    return img


def _textures():
    if not _TEX_CACHE:
        _TEX_CACHE["table"] = wood_texture(3, 512, np.array([0.47, 0.30, 0.17]), np.array([0.28, 0.16, 0.08]))
        _TEX_CACHE["floor"] = planks_texture(5, 512, np.array([0.62, 0.48, 0.33]), np.array([0.40, 0.29, 0.18]))
        _TEX_CACHE["wall"] = plaster_texture(7, 256, np.array([0.86, 0.84, 0.79]))
        _TEX_CACHE["birch"] = wood_texture(11, 128, np.array([0.92, 0.82, 0.62]), np.array([0.80, 0.66, 0.45]),
                                           rings=8.0, stretch=2.0)
    return _TEX_CACHE


# ----------------------------------------------------------------------------------------------
# The episode's parts
# ----------------------------------------------------------------------------------------------
def kit_parts(kit):
    """The kit's own parts: (type, dims) in a fixed order."""
    c, a, w, b = kit["chassis"], kit["axle"], kit["wheel"], kit["cabin"]
    out = [("chassis", dict(c))]
    out += [("axle", dict(a)) for _ in range(2)]
    out += [("wheel", dict(w)) for _ in range(4)]
    out.append(("cabin", dict(b)))
    return out


# The toy family: every episode draws its own car within these ranges (whole millimetres), then the sibling sets
# relative to it. Fixed for the whole family: base plate 8 mm, blocks 10 mm, notches 11 mm wide, axles 10 mm thick.
FAMILY = {
    "chassis_length": (120, 146), "chassis_width": (48, 60), "end_block": (12, 20),
    "wheel_dia": (40, 50), "wheel_thick": (12, 16), "wheel_gap": (2, 5),
    "cabin_length": (44, 64), "cabin_width": (34, 44), "cabin_height": (26, 36),     # cabin width <= length - 10
}


def sample_kit(rng):
    """One car of the family (metres), drawn from rng: every number a whole millimetre."""
    F = FAMILY
    mm = lambda key: int(rng.integers(F[key][0], F[key][1] + 1))
    L, W = mm("chassis_length"), mm("chassis_width")
    notch_w, base_t, block_h, adia = 11, 8, 10, 10    # notch 1 mm wider than the axle: a precision fit
    notch_x = L // 2 - mm("end_block") - notch_w // 2
    middle = 2 * notch_x - notch_w                       # the block between the notches carries the cabin
    cab_len = int(rng.integers(F["cabin_length"][0], min(F["cabin_length"][1], middle - 8) + 1))
    thick = mm("wheel_thick")
    axle_len = W + 2 * thick + 2 * mm("wheel_gap")       # the wheels' outer faces flush with the axle's ends
    kit = {
        "model": "episode",
        "chassis": {"length": L, "width": W, "base_t": base_t, "block_h": block_h, "notch_x": notch_x,
                    "notch_w": notch_w},
        "axle": {"length": axle_len, "dia": adia},
        "wheel": {"dia": mm("wheel_dia"), "thick": thick},
        "cabin": {"length": cab_len, "width": int(rng.integers(F["cabin_width"][0], min(F["cabin_width"][1], cab_len - 10) + 1)),
                  "height": mm("cabin_height")},
    }
    for part, d in kit.items():
        if isinstance(d, dict):
            for k in d:
                d[k] = d[k] / 1000.0
    return kit


SIBLING_SETS = (("axle", "cabin"), ("axle", "wheel"), ("wheel", "cabin"), ("axle", "wheel", "cabin"))
SIBLING_COUNT = {"axle": 2, "wheel": 4, "cabin": 1}
SIBLING_SCALE = 1.0              # difficulty knob: how far a sibling part's key size is from the kit's (1.0 = 8-30 mm)


def other_model_parts(kit, rng):
    """The sibling model's parts in the box: full sets of two or three part types (its 2 axles, its 4 wheels, its
    cabin), one combination drawn per episode, so that every such type lies on the table as many times in the
    sibling's size as in the kit's and counting tells nothing. A sibling part differs from the kit's own in its key
    dimension (axle length, wheel diameter, cabin length) and is the same within a set."""
    types = SIBLING_SETS[int(rng.integers(len(SIBLING_SETS)))]
    out = []
    for typ in types:
        d = dict(kit[typ])
        if typ == "axle":
            if rng.random() < 0.6:
                d["length"] = kit["axle"]["length"] + SIBLING_SCALE * rng.uniform(0.016, 0.030)
            else:
                d["length"] = kit["axle"]["length"] - SIBLING_SCALE * rng.uniform(0.016, 0.022)
        elif typ == "wheel":
            if rng.random() < 0.6:
                d["dia"] = kit["wheel"]["dia"] + SIBLING_SCALE * rng.uniform(0.009, 0.016)
            else:
                d["dia"] = max(2 * (kit["chassis"]["base_t"] + kit["axle"]["dia"] / 2) + 0.004,
                               kit["wheel"]["dia"] - SIBLING_SCALE * rng.uniform(0.008, 0.010))
        else:
            if rng.random() < 0.5:
                d["length"] = kit["cabin"]["length"] + SIBLING_SCALE * rng.uniform(0.012, 0.018)
                d["height"] = kit["cabin"]["height"] - SIBLING_SCALE * rng.uniform(0.006, 0.010)
            else:
                d["length"] = kit["cabin"]["length"] - SIBLING_SCALE * rng.uniform(0.010, 0.014)
                d["width"] = kit["cabin"]["width"] - rng.uniform(0.0, 0.004)
                d["height"] = kit["cabin"]["height"] + SIBLING_SCALE * rng.uniform(0.008, 0.012)
        out += [(typ, dict(d)) for _ in range(SIBLING_COUNT[typ])]
    return out


def part_footprint(typ, d):
    """Half extents (along the part's own x, y) of a part lying in its rest pose, and its rest height."""
    if typ == "chassis":
        return d["length"] / 2, d["width"] / 2, 0.0
    if typ == "axle":
        return d["dia"] / 2, d["length"] / 2, d["dia"] / 2
    if typ == "wheel":
        return d["dia"] / 2, d["dia"] / 2, d["thick"] / 2
    return d["length"] / 2, d["width"] / 2, 0.0


def part_geoms(typ, d, rgb, prefix, coll=""):
    """MJCF geoms of a part in its body frame. Body frames: chassis and cabin at the centre of the bottom face,
    axle at its centre with its axis along local y, wheel at its centre with its axis along local z."""
    rgba = "%.3f %.3f %.3f 1" % tuple(rgb)
    g = []
    fr = 'friction="0.9 0.01 0.0005"'
    if typ == "chassis":
        L, W, bt, bh, nx, nw = (d[k] for k in ("length", "width", "base_t", "block_h", "notch_x", "notch_w"))
        g.append('<geom name="{p}base" type="box" pos="0 0 {z}" size="{x} {y} {h}" rgba="{c}" {f}/>'.format(
            p=prefix, z=bt / 2, x=L / 2, y=W / 2, h=bt / 2, c=rgba, f=fr))
        edges = [-L / 2, -nx - nw / 2, -nx + nw / 2, nx - nw / 2, nx + nw / 2, L / 2]
        for k, (x0, x1) in enumerate([(edges[0], edges[1]), (edges[2], edges[3]), (edges[4], edges[5])]):
            g.append('<geom name="{p}blk{k}" type="box" pos="{cx} 0 {z}" size="{x} {y} {h}" rgba="{c}" {f}/>'.format(
                p=prefix, k=k, cx=(x0 + x1) / 2, z=bt + bh / 2, x=(x1 - x0) / 2, y=W / 2, h=bh / 2, c=rgba, f=fr))
    elif typ == "axle":
        g.append('<geom name="{p}rod" type="cylinder" quat="0.7071068 0.7071068 0 0" size="{r} {h}" '
                 'rgba="{c}" {f} condim="6" contype="5" conaffinity="5"/>'.format(
                     p=prefix, r=d["dia"] / 2, h=d["length"] / 2, c=rgba, f='friction="0.9 0.02 0.002"'))
    elif typ == "wheel":
        r, t = d["dia"] / 2, d["thick"] / 2
        g.append('<geom name="{p}tire" type="cylinder" size="{r} {h}" rgba="{c}" {f} condim="6" contype="3" '
                 'conaffinity="3"/>'.format(
            p=prefix, r=r, h=t, c=rgba, f='friction="1.0 0.02 0.002"'))
        g.append('<geom name="{p}hub" type="cylinder" size="{r} {h}" rgba="{c} 1" contype="0" conaffinity="0" '
                 'mass="0"/>'.format(p=prefix, r=0.42 * r, h=t + 0.0006, c="%.3f %.3f %.3f" % HUB_RGB))
        g.append('<geom name="{p}mark" type="box" pos="{x} 0 0" size="{s} 0.0025 {h}" rgba="0.85 0.85 0.85 1" '
                 'contype="0" conaffinity="0" mass="0"/>'.format(p=prefix, x=0.7 * r, s=0.12 * r, h=t + 0.0004))
    else:
        L, W, H = d["length"], d["width"], d["height"]
        g.append('<geom name="{p}box" type="box" pos="0 0 {z}" size="{x} {y} {h}" rgba="{c}" {f}/>'.format(
            p=prefix, z=H / 2, x=L / 2, y=W / 2, h=H / 2, c=rgba, f=fr))
        g.append('<geom name="{p}roof" type="box" pos="0 0 {z}" size="{x} {y} 0.0006" rgba="0.95 0.95 0.93 1" '
                 'contype="0" conaffinity="0" mass="0"/>'.format(p=prefix, z=H + 0.0006, x=L / 2 - 0.003,
                                                                  y=W / 2 - 0.003))
        win = "0.16 0.22 0.30 1"
        g.append('<geom name="{p}winl" type="box" pos="0 {y} {z}" size="{x} 0.0006 {h}" rgba="{c}" contype="0" '
                 'conaffinity="0" mass="0"/>'.format(p=prefix, y=W / 2, z=0.62 * H, x=0.33 * L, h=0.2 * H, c=win))
        g.append('<geom name="{p}winr" type="box" pos="0 {y} {z}" size="{x} 0.0006 {h}" rgba="{c}" contype="0" '
                 'conaffinity="0" mass="0"/>'.format(p=prefix, y=-W / 2, z=0.62 * H, x=0.33 * L, h=0.2 * H, c=win))
        g.append('<geom name="{p}winf" type="box" pos="{x} 0 {z}" size="0.0006 {y} {h}" rgba="{c}" contype="0" '
                 'conaffinity="0" mass="0"/>'.format(p=prefix, x=L / 2, z=0.62 * H, y=0.36 * W, h=0.2 * H, c=win))
    out = []
    for x in g:
        if 'contype="0"' not in x:
            if coll:                                         # the caller's collision bits replace the part's own
                x = re.sub(r' contype="\d+" conaffinity="\d+"', "", x)
            x = x.replace("<geom ", '<geom density="%g" %s ' % (WOOD_DENSITY, coll))
        out.append(x)
    return "".join(out)


# ----------------------------------------------------------------------------------------------
# Scene
# ----------------------------------------------------------------------------------------------
SCENE_XML = """<mujoco model="toy_car_cell">
  <compiler angle="radian" autolimits="true"/>
  <option timestep="{dt}" integrator="implicitfast" cone="elliptic" impratio="10"/>
  <statistic extent="1.2" center="0 0 0.9"/>
  <visual><global offwidth="1280" offheight="960"/><quality shadowsize="4096"/><map shadowclip="0.8" shadowscale="0.5"/>
    <headlight ambient="0.32 0.32 0.32" diffuse="0.35 0.35 0.35"/></visual>
  <worldbody>
    <light name="lamp" pos="0.25 0.45 1.75" dir="-0.3 -0.45 -1" diffuse="0.55 0.52 0.46" castshadow="true"/>
    <light name="fill" pos="-0.9 -0.6 2.2" dir="0.4 0.25 -1" diffuse="0.28 0.28 0.30" castshadow="false"/>
    <light name="sun" directional="true" pos="0.6 0.9 2.5" dir="-0.25 -0.35 -1" diffuse="0.25 0.25 0.23" castshadow="true" active="false"/>
    <geom name="floor" type="plane" size="4 4 0.1" material="floormat"/>
    <geom name="wall_back" type="box" pos="1.6 0 1.4" size="0.02 3 1.4" material="wallmat" contype="0" conaffinity="0"/>
    <geom name="wall_front" type="box" pos="-1.6 0 1.4" size="0.02 3 1.4" material="wallmat" contype="0" conaffinity="0"/>
    <geom name="skirting" type="box" pos="1.57 0 0.05" size="0.01 3 0.05" rgba="0.92 0.91 0.88 1" contype="0" conaffinity="0"/>
    <geom name="wall_left" type="box" pos="0 2.2 1.4" size="3 0.02 1.4" material="wallmat" contype="0" conaffinity="0"/>
    <geom name="wall_right" type="box" pos="0 -2.2 1.4" size="3 0.02 1.4" material="wallmat" contype="0" conaffinity="0"/>
    <geom name="table" type="box" pos="0 0 {tz}" size="{thx} {thy} 0.02" material="tablemat" friction="0.8 0.005 0.0001"/>
    <geom name="leg1" type="box" pos="{lx} {ly} {lz}" size="0.025 0.025 {lh}" material="tablemat" contype="0" conaffinity="0"/>
    <geom name="leg2" type="box" pos="{lx} -{ly} {lz}" size="0.025 0.025 {lh}" material="tablemat" contype="0" conaffinity="0"/>
    <geom name="leg3" type="box" pos="-{lx} {ly} {lz}" size="0.025 0.025 {lh}" material="tablemat" contype="0" conaffinity="0"/>
    <geom name="leg4" type="box" pos="-{lx} -{ly} {lz}" size="0.025 0.025 {lh}" material="tablemat" contype="0" conaffinity="0"/>
    <geom name="pedestal" type="box" pos="{rx} 0 {pz}" size="0.11 0.11 {ph}" rgba="0.22 0.22 0.24 1"/>
    <body name="stand" pos="{sx} {sy} {tt}" euler="0 0 {syaw}">
      <geom name="stand_post" type="box" pos="0 0 {spz}" size="0.012 0.012 {sph}" rgba="0.55 0.56 0.58 1"/>
      <geom name="stand_foot" type="box" pos="0 0 {sfz}" size="0.045 0.045 0.004" rgba="0.20 0.20 0.22 1"/>
      <geom name="stand_top" type="box" pos="0 0 {stz}" size="{stx} {sty} 0.003" rgba="0.55 0.56 0.58 1"/>
    </body>
    <body name="lamp_body" pos="0.30 0.52 {tt}">
      <geom type="cylinder" size="0.07 0.008" pos="0 0 0.008" rgba="0.12 0.12 0.12 1" contype="0" conaffinity="0"/>
      <geom type="capsule" fromto="0 0 0.01 -0.05 -0.06 0.36" size="0.011" rgba="0.30 0.30 0.32 1" contype="0" conaffinity="0"/>
      <geom type="capsule" fromto="-0.05 -0.06 0.36 -0.14 -0.16 0.42" size="0.010" rgba="0.30 0.30 0.32 1" contype="0" conaffinity="0"/>
      <geom type="cylinder" size="0.06 0.05" pos="-0.17 -0.19 0.40" euler="0.5 -0.3 0" rgba="0.85 0.80 0.55 1" contype="0" conaffinity="0"/>
    </body>
  </worldbody>
</mujoco>"""


def _add_textures(spec):
    tex = _textures()
    for name, img in tex.items():
        t = spec.add_texture(name=name, type=mujoco.mjtTexture.mjTEXTURE_2D, width=img.shape[1], height=img.shape[0],
                             nchannel=3)
        t.data = img.tobytes()
    for mname, tname, rep in (("tablemat", "table", (2, 2)), ("floormat", "floor", (4, 4)),
                              ("wallmat", "wall", (3, 2)), ("birch", "birch", (1, 1))):
        m = spec.add_material(name=mname)
        m.textures[mujoco.mjtTextureRole.mjTEXROLE_RGB] = tname
        m.texrepeat = list(rep)
        m.specular = 0.15
        m.shininess = 0.3


def build_model(parts, look, robot=True):
    """parts: list of (type, dims, rgb). Bodies: o_part<i>, geoms o_part<i>_*."""
    xml = SCENE_XML.format(
        dt=TIMESTEP, tz=TABLE_Z - 0.02, thx=TABLE_HALF[0], thy=TABLE_HALF[1],
        lx=TABLE_HALF[0] - 0.05, ly=TABLE_HALF[1] - 0.05, lz=(TABLE_Z - 0.04) / 2, lh=(TABLE_Z - 0.04) / 2,
        rx=ROBOT_BASE[0], pz=TABLE_Z + 0.002, ph=0.002,
        sx=look["stand"][0], sy=look["stand"][1], syaw=look["stand"][2], spz=STAND_H / 2, sph=STAND_H / 2 - 0.003,
        sfz=0.004, stz=STAND_H - 0.003, stx=STAND_TOP[0], sty=STAND_TOP[1], tt=TABLE_Z)
    root = mujoco.MjSpec.from_string(xml)
    _add_textures(root)
    wb = root.worldbody
    c = wb.add_camera()
    c.name = "overview"
    c.pos = list(look["ov_pos"])
    c.quat = list(quat_from_mat(look["ov_R"]))
    c.fovy = OVERVIEW["fovy"]
    c2 = wb.add_camera()
    c2.name = "demo"
    c2.pos = [0.62, -0.62, 1.30]
    c2.quat = list(quat_from_mat(look_at((0.62, -0.62, 1.30), (-0.02, 0.02, 0.84))))
    c2.fovy = 42.0
    if robot:
        assets = robosuite_assets()
        rob = mujoco.MjSpec.from_file(os.path.join(assets, "robots", "panda", "robot.xml"))
        grip = mujoco.MjSpec.from_file(os.path.join(assets, "grippers", "panda_gripper.xml"))
        for cam in list(rob.cameras):
            if cam.name == "robotview":
                rob.delete(cam)
        rob.body("right_hand").add_frame().attach_body(grip.body("right_gripper"), "g_", "")
        wb.add_frame(pos=list(ROBOT_BASE + np.array([0, 0, 0.004]))).attach_body(rob.body("base"), "r_", "")
    # the press fits' detents: mocap bodies parked out of the way until their mate exists (see ToyCarEnv._detents)
    dk = look.get("detent_sizes")
    if dk is not None:
        for k in range(N_DETENT_BARS):
            b = wb.add_body(name="detent_bar%d" % k, pos=[3.0, 3.0 + 0.1 * k, 0.05], mocap=True)
            g = b.add_geom(name="detent_bar%d_g" % k, type=mujoco.mjtGeom.mjGEOM_BOX, size=list(dk["bar"]),
                           rgba=[0.5, 0.5, 0.5, 0.0])
            g.contype, g.conaffinity, g.priority = 4, 4, 1
            g.solref, g.solimp = [0.002, 1.0], [0.99, 0.999, 0.0005, 0.5, 2.0]
        for k in range(N_DETENT_COLLARS):
            b = wb.add_body(name="detent_collar%d" % k, pos=[3.2, 3.0 + 0.1 * k, 0.05], mocap=True)
            g = b.add_geom(name="detent_collar%d_g" % k, type=mujoco.mjtGeom.mjGEOM_CYLINDER,
                           size=list(dk["collar"]), rgba=[0.5, 0.5, 0.5, 0.0])
            g.contype, g.conaffinity, g.priority = 2, 2, 1
            g.solref, g.solimp = [0.002, 1.0], [0.99, 0.999, 0.0005, 0.5, 2.0]
    bodies = []
    for i, (typ, d, rgb) in enumerate(parts):
        bodies.append('<body name="part{i}" pos="{x} 1.5 0.9"><freejoint name="part{i}_free"/>{g}</body>'.format(
            i=i, x=1.0 + 0.15 * i, g=part_geoms(typ, d, rgb, "part%d_" % i)))
    objs = mujoco.MjSpec.from_string(
        '<mujoco><compiler angle="radian" autolimits="true"/>'
        '<worldbody>{}</worldbody></mujoco>'.format("".join(bodies)))
    for i in range(len(parts)):
        wb.add_frame().attach_body(objs.body("part%d" % i), "o_", "")
    # Wheels pass over axles (the axle runs through the wheel's bore): no contact between the two types.
    for i, (ti, _d, _c) in enumerate(parts):
        for j, (tj, _d2, _c2) in enumerate(parts):
            if ti == "wheel" and tj == "axle":
                ex = root.add_exclude()
                ex.bodyname1 = "o_part%d" % i
                ex.bodyname2 = "o_part%d" % j
    if robot:
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
    if robot:
        m.cam_fovy[m.camera("r_eye_in_hand").id] = WRIST["fovy"]
        # the robot model's collision meshes (group 0) overlap its visual meshes: draw only the visual ones
        for i in range(m.ngeom):
            if m.geom(i).name.startswith(("r_", "g_", "r_g_")) or m.body(m.geom_bodyid[i]).name.startswith("r_"):
                if m.geom_group[i] == 0:
                    m.geom_rgba[i, 3] = 0.0
    m.site_rgba[:, 3] = 0.0
    g = float(look["light"])
    for name in ("lamp", "fill"):
        lid = m.light(name).id
        m.light_diffuse[lid] = m.light_diffuse[lid] * g
    return m


def cheap_render_settings(m):
    """The graded cameras: no shadows, no multisampling (the CPU renderer stays fast)."""
    m.light_castshadow[:] = 0
    m.vis.quality.offsamples = 0


# ----------------------------------------------------------------------------------------------
# Mates
# ----------------------------------------------------------------------------------------------
def chassis_pose_on_stand(stand):
    return np.array([stand[0], stand[1], TABLE_Z + STAND_H]), float(stand[2])


def sample_overview(rng):
    """The overview camera as re-seated for an episode: the nominal pose moved by up to OVERVIEW_JITTER."""
    j = OVERVIEW_JITTER
    pos = np.array(OVERVIEW["pos"], float) + np.array([rng.uniform(-a, a) for a in j["pos"]])
    cx, cy = np.array(OVERVIEW["xaxis"], float), np.array(OVERVIEW["yaxis"], float)
    R = np.stack([cx, cy, np.cross(cx, cy)], axis=1)
    ang = rng.uniform(0, 2 * math.pi)
    tilt_axis = math.cos(ang) * R[:, 0] + math.sin(ang) * R[:, 1]
    R = rot_axis(tilt_axis, rng.uniform(0.3, 1.0) * j["tilt"]) @ rot_axis(R[:, 2], rng.uniform(-j["roll"], j["roll"])) @ R
    return pos, R


def sample_stand(rng):
    """(x, y, yaw) of the assembly stand for an episode."""
    jx, jy, jyaw = STAND_JITTER
    return (STAND_XY[0] + float(rng.uniform(-jx, jx)), STAND_XY[1] + float(rng.uniform(-jy, jy)),
            float(rng.uniform(-jyaw, jyaw)))


class Mate(object):
    """A place a part of type `typ` latches onto: pos (of the part's body frame), axis (the part's axis line,
    for axle and wheel), yaw (mod pi, for chassis and cabin)."""

    def __init__(self, name, typ, pos, axis=None, yaw=None, parent=None, side=0):
        self.name, self.typ = name, typ
        self.pos = np.asarray(pos, float)
        self.axis = None if axis is None else np.asarray(axis, float)
        self.yaw = yaw
        self.parent = parent      # index of the axle part a wheel mate sits on
        self.detent_body = None   # the press fit's detent (a mocap body) and its geom
        self.detent_geom = None
        self.yielded = False
        self.yield_part = None
        self.hold = 0.0
        self.over = 0.0
        self.logged = False
        self.side = side
        self.part = None          # index of the part latched here


# ----------------------------------------------------------------------------------------------
# The cell
# ----------------------------------------------------------------------------------------------
class ToyCarEnv(object):
    def __init__(self, kit=None, render=True, max_steps=MAX_STEPS, distractors=2, manual=True):
        """kit: None (the default) draws a car of the family every episode from its seed; a kit (dict or JSON path)
        builds that car every episode. manual: render the episode's picture manual, handed over in the
        observation at t = 0."""
        self.fixed_kit = load_kit(kit) if kit is not None else None
        self.kit = self.fixed_kit
        self.with_manual = bool(manual)
        self.manual = None
        self.render_enabled = render
        self.max_steps = int(max_steps)
        self.n_distractors = int(distractors)
        self.model = None
        self._renderers = {}

    # ------------------------------------------------------------------ episode
    def reset(self, seed):
        self.seed = int(seed) & (2 ** 64 - 1)
        s = [self.seed & 0xFFFFFFFF, self.seed >> 32]
        rng_parts = np.random.default_rng(s + [3])
        rng_look = np.random.default_rng(s + [5])
        rng_place = np.random.default_rng(s + [7])
        self.kit = self.fixed_kit if self.fixed_kit is not None else sample_kit(np.random.default_rng(s + [17]))
        self.manual = None
        if self.with_manual:
            import toycar_manual
            self.manual = toycar_manual.render_manual(self.kit)
        parts = [(t, d, "kit") for t, d in kit_parts(self.kit)]
        if self.n_distractors:
            parts += [(t, d, "other") for t, d in other_model_parts(self.kit, rng_parts)]
        order = rng_parts.permutation(len(parts))         # body index carries nothing about the model
        parts = [parts[i] for i in order]
        self.look = {"light": float(rng_look.uniform(0.85, 1.15))}
        self.look["detent_sizes"] = {
            "bar": (self.kit["chassis"]["notch_w"] / 2 - 0.001, self.kit["chassis"]["width"] / 2, 0.0015),
            "collar": (0.6 * self.kit["wheel"]["dia"] / 2, 0.0015)}
        rng_cell = np.random.default_rng(s + [11])
        self.look["ov_pos"], self.look["ov_R"] = sample_overview(rng_cell)
        self.look["stand"] = sample_stand(rng_cell)
        self.stand = self.look["stand"]
        self.parts = []
        for typ, d, model in parts:
            rgb = np.clip(np.array(PAINT[typ]) + rng_look.uniform(-0.03, 0.03, 3), 0, 1)
            self.parts.append({"type": typ, "dims": d, "model": model, "rgb": rgb})
        self.close()
        self.model = build_model([(p["type"], p["dims"], p["rgb"]) for p in self.parts], self.look)
        cheap_render_settings(self.model)
        self.data = mujoco.MjData(self.model)
        self.fk = mujoco.MjData(self.model)
        self._index()
        self._init_robot()
        self._layout(rng_place)
        self.t = 0
        self.done = False
        self.fault = None
        self.events = []
        self.latched = {}         # part index -> (pos, quat) held
        self._collars_used = 0
        self.contact_log = []
        self.damaged = set()      # parts forced too hard: they never seat
        self._make_mates()
        return self.observation()

    def _index(self):
        m = self.model
        self.jq = np.array([m.joint("r_joint%d" % i).qposadr[0] for i in range(1, 8)])
        self.jv = np.array([m.joint("r_joint%d" % i).dofadr[0] for i in range(1, 8)])
        self.act = np.array([m.actuator("r_torq_j%d" % i).id for i in range(1, 8)])
        self.gact = [m.actuator("r_g_gripper_finger_joint1").id, m.actuator("r_g_gripper_finger_joint2").id]
        self.gq = [m.joint("r_g_finger_joint1").qposadr[0], m.joint("r_g_finger_joint2").qposadr[0]]
        self.site = m.site("r_g_grip_site").id
        n = len(self.parts)
        self.pbody = [m.body("o_part%d" % i).id for i in range(n)]
        self.pqadr = [int(m.joint("o_part%d_free" % i).qposadr[0]) for i in range(n)]
        self.pdadr = [int(m.joint("o_part%d_free" % i).dofadr[0]) for i in range(n)]
        self.pgeoms = [set(int(g) for g in np.where(m.geom_bodyid == b)[0]) for b in self.pbody]
        fingers = set(b for b in range(m.nbody) if m.body(b).name.startswith("r_g_") and "finger" in m.body(b).name)
        self.finger_geoms = set(int(i) for i in np.where(np.isin(m.geom_bodyid, list(fingers)))[0])
        self._jp, self._jr = np.zeros((3, m.nv)), np.zeros((3, m.nv))
        self._M = np.zeros((m.nv, m.nv))

    def _init_robot(self):
        m, d = self.model, self.data
        mujoco.mj_resetData(m, d)
        d.qpos[self.jq] = HOME_Q
        d.qpos[self.gq[0]], d.qpos[self.gq[1]] = 0.04, -0.04
        for i in range(len(self.parts)):
            a = self.pqadr[i]
            d.qpos[a:a + 3] = [1.0 + 0.15 * i, 1.5, 0.05]
            d.qpos[a + 3:a + 7] = [1, 0, 0, 0]
        mujoco.mj_forward(m, d)
        self.q_stale = d.qpos.copy()
        self.grip_target = 0.04
        self.grip_cmd = -1.0
        q_hold = d.qpos[self.jq].copy()
        for _ in range(150):
            d.ctrl[self.act] = 200.0 * (q_hold - d.qpos[self.jq]) - 20.0 * d.qvel[self.jv] + d.qfrc_bias[self.jv]
            d.ctrl[self.gact] = [0.04, -0.04]
            mujoco.mj_step(m, d)
        R = d.site_xmat[self.site].reshape(3, 3).copy()
        # The hand pointing down at yaw 0. The reference is turned half a turn from the home pose's hand, so that
        # over the work area yaw 0 sits near the middle of the wrist's range (the home pose is at yaw +-pi).
        self.R_down = rot_z(math.pi) @ rot_z(-yaw_of_mat(R)) @ R
        # the finger-closing axis of the hand at yaw 0 (world), the axis tilt turns about
        self.close_axis0 = self._closing_axis_world(self.R_down)
        self.target_pos = d.site_xpos[self.site].copy()
        self.target_yaw, _tilt = self.hand_pose()
        self.target_tilt = 0.0
        self.q_stale = d.qpos.copy()

    def _closing_axis_world(self, R):
        """The finger-closing axis (world) of a hand whose grip-site frame is R: the slide axis of the fingers."""
        m = self.model
        b = m.body("r_g_leftfinger").id
        # finger joint axis in the finger body frame is local y; finger body frames are fixed in the hand frame.
        jid = m.joint("r_g_finger_joint1").id
        ax_local = m.jnt_axis[jid]
        Rb = self.data.xmat[b].reshape(3, 3)
        Rs = self.data.site_xmat[self.site].reshape(3, 3)
        ax_site = Rs.T @ (Rb @ ax_local)                  # in the grip-site frame
        return R @ ax_site

    def _layout(self, rng):
        """Loose parts on the table at random poses, wholly inside the overview camera's view, clear of the stand
        and of each other. The largest parts are placed first; a layout that does not fit is drawn again."""
        d = self.data
        sizes = []
        for p in self.parts:
            hx, hy, hz = part_footprint(p["type"], p["dims"])
            sizes.append((math.hypot(hx, hy), hz))
        order = sorted(range(len(self.parts)), key=lambda i: -sizes[i][0])
        poses = None
        for attempt in range(200):
            gap = 0.025 if attempt < 100 else 0.012
            placed = {}
            for i in order:
                rad = sizes[i][0]
                for _ in range(400):
                    x = rng.uniform(LAYOUT_X[0], LAYOUT_X[1])
                    y = rng.uniform(-LAYOUT_Y, LAYOUT_Y)
                    if not LAYOUT_R[0] <= math.hypot(x - ROBOT_BASE[0], y) <= LAYOUT_R[1]:
                        continue
                    if abs(y) + rad > LAYOUT_Y + 0.01 or x + rad > LAYOUT_X[1] + 0.04:
                        continue
                    if abs(x - self.stand[0]) < 0.12 + rad and abs(y - self.stand[1]) < 0.08 + rad:
                        continue
                    if any(math.hypot(x - q[0], y - q[1]) < rad + q[2] + gap for q in placed.values()):
                        continue
                    placed[i] = (x, y, rad)
                    break
                else:
                    break
            if len(placed) == len(self.parts):
                poses = placed
                break
        if poses is None:
            raise EnvError("could not lay out the parts")
        for i, p in enumerate(self.parts):
            x, y, _rad = poses[i]
            yaw = rng.uniform(-math.pi, math.pi)
            a, v = self.pqadr[i], self.pdadr[i]
            d.qpos[a:a + 3] = [x, y, TABLE_Z + sizes[i][1] + 0.0005]
            d.qpos[a + 3:a + 7] = yaw_quat(yaw)
            d.qvel[v:v + 6] = 0.0
        mujoco.mj_forward(self.model, d)
        # let the parts settle
        for _ in range(100):
            d.ctrl[self.act] = self._osc_torque()
            d.ctrl[self.gact] = [0.04, -0.04]
            mujoco.mj_step(self.model, d)
        self.q_stale = d.qpos.copy()

    def _make_mates(self):
        k = self.kit
        cpos, cyaw = chassis_pose_on_stand(self.stand)
        self.mates = [Mate("stand", "chassis", cpos, yaw=cyaw)]
        self.cab_mate = None

    def _chassis_mates(self, ci):
        """Once the chassis (part ci) is on the stand: the notch and cabin mates, from that chassis's own size."""
        d = self.parts[ci]["dims"]
        base = self.latched[ci][0]
        Rs = rot_z(self.stand[2])
        for k, (s, name) in enumerate(((1, "front_notch"), (-1, "rear_notch"))):
            mt = Mate(name, "axle", base + Rs @ np.array([s * d["notch_x"], 0.0, d["base_t"]]),
                      axis=Rs @ np.array([0.0, 1.0, 0.0]))
            mt.detent_body = self.model.body("detent_bar%d" % k).id
            mt.detent_geom = self.model.geom("detent_bar%d_g" % k).id
            hz = self.look["detent_sizes"]["bar"][2]
            self._place_detent(mt, mt.pos + np.array([0.0, 0.0, PRESS["axle"]["lip"] - hz]), yaw_quat(self.stand[2]))
            self.mates.append(mt)
        self.mates.append(Mate("cabin_seat", "cabin", base + np.array([0.0, 0.0, d["base_t"] + d["block_h"]]),
                               yaw=self.stand[2]))

    def _axle_mates(self, ai, mate):
        L = self.parts[ai]["dims"]["length"]
        t = self.kit["wheel"]["thick"]
        for side in (1, -1):
            mt = Mate("%s_%s_wheel" % (mate.name.split("_")[0], "left" if side > 0 else "right"),
                      "wheel", mate.pos + np.array([0.0, 0.0, self.parts[ai]["dims"]["dia"] / 2]) + mate.axis * (side * L / 2),
                      axis=mate.axis.copy(), parent=ai, side=side)    # on the axle's axis, at its end
            k = self._collars_used
            self._collars_used += 1
            mt.detent_body = self.model.body("detent_collar%d" % k).id
            mt.detent_geom = self.model.geom("detent_collar%d_g" % k).id
            out = mt.axis * side
            hz = self.look["detent_sizes"]["collar"][1]
            # the collar's outer face where the wheel's inner face is when the wheel sits at the lip
            centre = mt.pos + out * (PRESS["wheel"]["lip"] - t - hz)
            z = out
            x = np.cross(np.array([0.0, 0.0, 1.0]), z)
            x = x / np.linalg.norm(x)
            self._place_detent(mt, centre, quat_from_mat(np.stack([x, np.cross(z, x), z], axis=1)))
            self.mates.append(mt)

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
        tau += N @ (5.0 * (NULLSPACE_Q - qn) - 6.0 * dq)
        tau -= 0.5 * dq
        over = np.abs(dq) - 1.5
        tau -= np.where(over > 0, 30.0 * over * np.sign(dq), 0.0)
        return tau

    @property
    def target_R(self):
        return rot_z(self.target_yaw) @ rot_axis(self.close_axis0, self.target_tilt) @ self.R_down

    def hand_pose(self, R=None):
        """(yaw, tilt) of a hand whose grip-site frame is R (default: the current one)."""
        if R is None:
            R = self.data.site_xmat[self.site].reshape(3, 3)
        M = R @ self.R_down.T                              # = rot_z(yaw) @ rot_axis(c0, tilt)
        c = M @ self.close_axis0
        yaw = wrap_angle(math.atan2(c[1], c[0]) - math.atan2(self.close_axis0[1], self.close_axis0[0]))
        T = rot_z(-yaw) @ M
        # T is a rotation about close_axis0: its angle
        cosang = (np.trace(T) - 1.0) / 2.0
        ang = math.acos(max(-1.0, min(1.0, cosang)))
        axis = np.array([T[2, 1] - T[1, 2], T[0, 2] - T[2, 0], T[1, 0] - T[0, 1]])
        if np.dot(axis, self.close_axis0) < 0:
            ang = -ang
        return yaw, ang

    def step(self, action):
        if self.done:
            raise EnvError("episode already finished")
        dp, dyaw, dtilt, g = parse_action(action)
        d = self.data
        n = np.linalg.norm(dp)
        if n > MAX_DPOS:
            dp = dp * (MAX_DPOS / n)
        dyaw = float(np.clip(dyaw, -MAX_DYAW, MAX_DYAW))
        dtilt = float(np.clip(dtilt, -MAX_DTILT, MAX_DTILT))
        cur_p = d.site_xpos[self.site].copy()
        self.target_pos = np.clip(cur_p + dp, WORKSPACE_LO, WORKSPACE_HI)
        yaw, tilt = self.hand_pose()
        self.target_yaw = wrap_angle(yaw + dyaw)
        self.target_tilt = float(np.clip(tilt + dtilt, -TILT_MAX, TILT_MAX))
        self.grip_target = 0.04 * (1.0 - g) / 2.0
        self.grip_cmd = g
        self._physics_step()
        self.t += 1
        self._post_step()
        return self.observation(), self.done, self.info()

    def _hold_latched(self):
        d = self.data
        for i, (p, q) in self.latched.items():
            a, v = self.pqadr[i], self.pdadr[i]
            d.qpos[a:a + 3] = p
            d.qpos[a + 3:a + 7] = q
            d.qvel[v:v + 6] = 0.0

    def _physics_step(self):
        m, d = self.model, self.data
        gw = [self.grip_target, -self.grip_target]
        for k in range(SUBSTEPS):
            d.ctrl[self.act] = self._osc_torque()
            d.ctrl[self.gact] = gw
            self._hold_latched()
            self._press_fits()
            self.q_stale[:] = d.qpos
            mujoco.mj_step(m, d)
            if k % 5 == 4:
                self._try_latch()
        self._hold_latched()
        mujoco.mj_forward(m, d)
        if not np.all(np.isfinite(d.qpos)) or not np.all(np.isfinite(d.qvel)):
            self.fault = "non-finite state"

    # ------------------------------------------------------------------ latching
    def part_pose(self, i):
        d = self.data
        b = self.pbody[i]
        return d.xpos[b].copy(), d.xmat[b].reshape(3, 3).copy()

    def touching_fingers(self, i):
        d = self.data
        gs = self.pgeoms[i]
        for k in range(d.ncon):
            c = d.contact[k]
            g1, g2 = int(c.geom1), int(c.geom2)
            if (g1 in gs and g2 in self.finger_geoms) or (g2 in gs and g1 in self.finger_geoms):
                return True
        return False

    def _mate_fits(self, i, mt):
        p = self.parts[i]
        typ, dims = p["type"], p["dims"]
        pos, R = self.part_pose(i)
        tol = LATCH[typ]
        if typ in ("chassis", "cabin"):
            if R[2, 2] < math.cos(tol["ang"]):
                return None
            if math.hypot(pos[0] - mt.pos[0], pos[1] - mt.pos[1]) > tol["pos"]:
                return None
            if not (-0.004 <= pos[2] - mt.pos[2] <= LATCH_Z):
                return None
            if yaw_error_pi(yaw_of_mat(R), mt.yaw) > tol["ang"]:
                return None
            return mt.pos.copy(), quat_from_mat(rot_z(mt.yaw + (math.pi if abs(wrap_angle(yaw_of_mat(R) - mt.yaw)) > math.pi / 2 else 0.0)))
        if typ == "axle":
            target = mt.pos + np.array([0.0, 0.0, dims["dia"] / 2])
            ax = R[:, 1]
            if axis_angle_error(ax, mt.axis) > tol["ang"]:
                return None
            if math.hypot(pos[0] - target[0], pos[1] - target[1]) > tol["pos"]:
                return None
            if not (-0.004 <= pos[2] - target[2] <= LATCH_Z):
                return None
            # align the rod's axis exactly with the notch
            Rt = rot_z(self.stand[2]) if np.dot(ax, mt.axis) >= 0 else rot_z(self.stand[2] + math.pi)
            return target, quat_from_mat(Rt)
        # wheel: centre at the axle end less half its thickness, axis along the axle
        target = mt.pos - mt.axis * (mt.side * dims["thick"] / 2)
        ax = R[:, 2]
        if axis_angle_error(ax, mt.axis) > tol["ang"]:
            return None
        if np.linalg.norm(pos - target) > tol["pos"]:
            return None
        # the wheel's spin about its axis is kept
        zc = mt.axis if np.dot(ax, mt.axis) >= 0 else -mt.axis
        xc = R[:, 0] - np.dot(R[:, 0], zc) * zc
        xc = xc / np.linalg.norm(xc)
        Rt = np.stack([xc, np.cross(zc, xc), zc], axis=1)
        return target, quat_from_mat(Rt)

    def _try_latch(self):
        free = [mt for mt in self.mates if mt.part is None]
        if not free:
            return
        for i, p in enumerate(self.parts):
            if i in self.latched:
                continue
            if p["type"] in PRESS:
                continue                                   # axles and wheels seat by a press fit
            pos = self.data.xpos[self.pbody[i]]
            cands = [mt for mt in free if mt.typ == p["type"] and np.linalg.norm(pos - mt.pos) < 0.08]
            if not cands:
                continue
            for mt in cands:
                hit = self._mate_fits(i, mt)
                if hit is None:
                    continue
                if self.grip_cmd > 0.0 and self.touching_fingers(i):
                    break                                  # still held: a part latches once let go
                self._latch(i, mt, hit)
                free = [m2 for m2 in self.mates if m2.part is None]
                break

    def _press_geometry(self, i, mt):
        """(along, lateral vector, axis error) of part i against press-fit mate mt: along = distance from home
        toward the outside (up out of the notch; out along the axle), or None when far away."""
        p = self.parts[i]
        d_ = p["dims"]
        pos, R = self.part_pose(i)
        if p["type"] == "axle":
            home = mt.pos + np.array([0.0, 0.0, d_["dia"] / 2])
            out = np.array([0.0, 0.0, 1.0])
            ax = R[:, 1]
        else:
            home = mt.pos - mt.axis * (mt.side * d_["thick"] / 2)
            out = mt.axis * mt.side
            ax = R[:, 2]
        rel = pos - home
        along = float(rel @ out)
        lat = rel - along * out
        if p["type"] == "axle":
            lat = lat - float(lat @ mt.axis) * mt.axis      # the rod may sit anywhere along the notch...
            if abs(float((pos - home) @ mt.axis)) > AXLE_ALONG_TOL:   # ...within a few mm of centred
                return None
        return along, lat, axis_angle_error(ax, mt.axis), home, out, ax

    def _detent_push(self, i, mt):
        """The contact force (N) between part i and the detent of mate mt."""
        m, d = self.model, self.data
        gd = mt.detent_geom
        gs = self.pgeoms[i]
        f6 = np.zeros(6)
        push = 0.0
        for k in range(d.ncon):
            c = d.contact[k]
            g1, g2 = int(c.geom1), int(c.geom2)
            if (g1 == gd and g2 in gs) or (g2 == gd and g1 in gs):
                mujoco.mj_contactForce(m, d, k, f6)
                push += float(f6[0])
        return push

    def _engaged(self, i, mt):
        """Whether part i sits in mate mt's lead-in (on its axis, in line with it, near the lip)."""
        g = self._press_geometry(i, mt)
        if g is None:
            return False
        along, lat, aerr = g[0], g[1], g[2]
        spec = PRESS[self.parts[i]["type"]]
        top = spec.get("top", self.parts[i]["dims"].get("thick", 0.0) + 0.004)
        return -0.003 <= along <= top and np.linalg.norm(lat) <= spec["lat"] and aerr <= spec["ang"]

    def _place_detent(self, mt, pos, quat):
        mid = self.model.body_mocapid[mt.detent_body]
        self.data.mocap_pos[mid] = pos
        self.data.mocap_quat[mid] = quat

    def _park_detent(self, mt):
        mid = self.model.body_mocapid[mt.detent_body]
        self.data.mocap_pos[mid] = [3.5, 3.5 + 0.1 * mid, 0.05]

    def _press_fits(self):
        """The press fits, every physics substep: the lead-in (a part near its seat is drawn onto the seat's axis and
        aligned, like a chamfer does), the detent lip (a real contact: the mate's detent body), its giving way or the
        part's damage, and the seating once home."""
        d = self.data
        free = [k for k, mt in enumerate(self.mates) if mt.part is None and mt.typ in PRESS]
        for k in free:
            mt = self.mates[k]
            if mt.yielded:
                continue
            ps = [i for i, p in enumerate(self.parts)
                  if p["type"] == mt.typ and i not in self.latched and i not in self.damaged]
            push = max([self._detent_push(i, mt) for i in ps] or [0.0])
            if push > 1.0 and not mt.logged:                  # diagnostics only (harness), never observed
                mt.logged = True
                ip = max(ps, key=lambda j: self._detent_push(j, mt))
                g = self._press_geometry(ip, mt)
                if g is not None:
                    self.contact_log.append({"mate": mt.name, "kit": self.parts[ip]["model"] == "kit",
                                             "lat": [round(float(v) * 1000, 2) for v in g[1]],
                                             "ang": round(math.degrees(g[2]), 2),
                                             "axial": round(float((self.data.xpos[self.pbody[ip]] - g[3]) @ mt.axis) * 1000, 2)})
            if push >= F_DAMAGE:
                mt.over += TIMESTEP
                if mt.over >= DAMAGE_S:
                    i = max(ps, key=lambda j: self._detent_push(j, mt))
                    self.damaged.add(i)
                    self.events.append({"t": self.t, "part": i, "type": self.parts[i]["type"],
                                        "model": self.parts[i]["model"], "mate": mt.name, "damaged": True})
                    mt.over = 0.0
            else:
                mt.over = 0.0
            if F_SEAT <= push < F_DAMAGE:
                mt.hold += TIMESTEP
                if mt.hold >= YIELD_S:
                    ip = max(ps, key=lambda j: self._detent_push(j, mt))
                    if ip not in self.damaged and self._engaged(ip, mt):
                        mt.yielded = True                     # the lip gives: the part can go home
                        mt.yield_part = ip
                        self._park_detent(mt)
            else:
                mt.hold = 0.0
        for i, p in enumerate(self.parts):
            if p["type"] not in PRESS or i in self.latched or i in self.damaged:
                continue
            pos = d.xpos[self.pbody[i]]
            for k in free:
                mt = self.mates[k]
                if mt.typ != p["type"] or np.linalg.norm(pos - mt.pos) > 0.05:
                    continue
                g = self._press_geometry(i, mt)
                if g is None:
                    continue
                along, lat, aerr, home, out, ax = g
                spec = PRESS[p["type"]]
                top = spec.get("top", p["dims"].get("thick", 0.0) + 0.004)
                if not (-0.003 <= along <= top and np.linalg.norm(lat) <= spec["lat"] and aerr <= spec["ang"]):
                    continue
                if mt.yielded and along <= HOME_TOL and np.linalg.norm(lat) <= 0.001 and aerr <= math.radians(3.0):
                    self._latch(i, mt, self._seat_pose(i, mt, home, ax))
                    break
                # the lead-in: a fraction of the way onto the seat's axis and into line with it, every substep
                a, v = self.pqadr[i], self.pdadr[i]
                d.qpos[a:a + 3] -= 0.15 * lat
                lin = d.qvel[v:v + 3]
                lin -= (lin - float(lin @ out) * out) * 0.5
                axis_t = mt.axis * (1.0 if float(ax @ mt.axis) >= 0 else -1.0)
                if aerr > 1e-4:
                    rv = np.cross(ax, axis_t)
                    n_ = np.linalg.norm(rv)
                    if n_ > 1e-9:
                        dq = np.zeros(4)
                        mujoco.mju_axisAngle2Quat(dq, rv / n_, 0.15 * aerr)
                        qn = np.zeros(4)
                        mujoco.mju_mulQuat(qn, dq, d.qpos[a + 3:a + 7].copy())
                        d.qpos[a + 3:a + 7] = qn
                        d.qvel[v + 3:v + 6] *= 0.5
                break

    def _seat_pose(self, i, mt, home, ax):
        p = self.parts[i]
        if p["type"] == "axle":
            Rt = rot_z(self.stand[2]) if np.dot(ax, mt.axis) >= 0 else rot_z(self.stand[2] + math.pi)
            return home, quat_from_mat(Rt)
        R = self.part_pose(i)[1]
        zc = mt.axis if np.dot(ax, mt.axis) >= 0 else -mt.axis
        xc = R[:, 0] - np.dot(R[:, 0], zc) * zc
        xc = xc / np.linalg.norm(xc)
        return home, quat_from_mat(np.stack([xc, np.cross(zc, xc), zc], axis=1))

    def _latch(self, i, mt, hit):
        pos, quat = hit
        self.latched[i] = (np.array(pos), np.array(quat))
        # a latched part is fixed to the car: make it as heavy as one for contact (pushing on it meets the stand and
        # the arm, not a few grams of wood that the per-step hold then puts back)
        b = self.pbody[i]
        self.model.body_mass[b] *= LATCHED_MASS_SCALE
        self.model.body_inertia[b] *= LATCHED_MASS_SCALE
        mt.part = i
        p = self.parts[i]
        self.events.append({"t": self.t, "part": i, "type": p["type"], "model": p["model"], "mate": mt.name})
        if mt.name == "stand":
            self._chassis_mates(i)
        elif p["type"] == "axle":
            self._axle_mates(i, mt)
        self._hold_latched()
        mujoco.mj_forward(self.model, self.data)

    def _post_step(self):
        d = self.data
        if self.fault is None:
            if np.max(np.abs(d.qvel[self.jv])) > INSTAB_ROBOT_QVEL:
                self.fault = "robot joint speed"
            elif any(np.linalg.norm(d.qvel[a:a + 3]) > INSTAB_PART_VEL for a in self.pdadr):
                self.fault = "part speed"
        if self.fault is not None:
            self.done = True
            return
        self._try_latch()
        if len(self.mates) == 8 and all(mt.part is not None for mt in self.mates):
            self.done = True
        if self.t >= self.max_steps:
            self.done = True

    # ------------------------------------------------------------------ credit
    def credit(self):
        """Mates filled by the kit's own part: the step credit (0..8)."""
        n = 0
        for mt in self.mates:
            if mt.part is not None and self.parts[mt.part]["model"] == "kit":
                n += 1
        return n

    def info(self):
        return {"t": self.t, "fault": self.fault, "latched": len(self.latched), "credit": self.credit()}

    # ------------------------------------------------------------------ sensing
    def close(self):
        for r in (self._renderers or {}).values():
            try:
                r.close()
            except Exception:  # noqa: BLE001
                pass
        self._renderers = {}

    def render(self, cam, width=None, height=None):
        if cam == "overview":
            w, h, name = OVERVIEW["width"], OVERVIEW["height"], "overview"
        elif cam == "wrist":
            w, h, name = WRIST["width"], WRIST["height"], "r_eye_in_hand"
        else:
            w, h, name = width, height, cam
        w, h = width or w, height or h
        r = self._renderers.get((w, h))
        if r is None:
            r = self._renderers[(w, h)] = mujoco.Renderer(self.model, h, w)
        r.update_scene(self.data, name)
        return r.render().copy()

    def observation(self):
        d = self.data
        R = d.site_xmat[self.site].reshape(3, 3)
        yaw, tilt = self.hand_pose(R)
        obs = {"t": int(self.t),
               "q": np.round(d.qpos[self.jq], 6),
               "ee_pos": np.round(d.site_xpos[self.site], 6),
               "ee_quat": np.round(quat_from_mat(R), 6),
               "ee_yaw": round(float(yaw), 6),
               "ee_tilt": round(float(tilt), 6),
               "gripper": round(float(d.qpos[self.gq[0]] - d.qpos[self.gq[1]]), 6),
               "wrench": [round(float(v), 4) for v in self.wrench()]}
        for cam in CAMERA_NAMES:
            obs[cam] = self.render(cam) if self.render_enabled else np.zeros((0, 0, 3), np.uint8)
        if self.t == 0 and self.manual is not None:
            obs["manual"] = self.manual                   # the episode's picture manual, at t = 0 only
        return obs

    def wrench(self):
        """The wrist force/torque sensor (between the arm's flange and the hand): force (N) and torque (N m) that the
        hand, and what it holds or pushes on, exert on the flange, in the world frame. At rest in free air it reads
        the hand's own weight."""
        m, d = self.model, self.data
        sid = m.site("r_g_ft_frame").id
        R = d.site_xmat[sid].reshape(3, 3)
        fa = m.sensor_adr[m.sensor("r_g_force_ee").id]
        ta = m.sensor_adr[m.sensor("r_g_torque_ee").id]
        return np.concatenate([R @ d.sensordata[fa:fa + 3], R @ d.sensordata[ta:ta + 3]])

    def perturbed_observation(self, obs, rng):
        """A probe copy for the grader's lock: joint readings offset by up to 0.002 rad, with the hand's pose moved
        by the same offsets. The real observation's hand pose is the exact kinematics of its joint readings (the
        cell runs a forward pass after the last physics step), so the copy's is the exact kinematics of its own.
        The images are the real ones."""
        dq = np.array([rng.uniform(-0.002, 0.002) for _ in range(7)])
        f = self.fk
        f.qpos[:] = self.data.qpos
        f.qpos[self.jq] = self.data.qpos[self.jq] + dq
        mujoco.mj_kinematics(self.model, f)
        R = f.site_xmat[self.site].reshape(3, 3)
        yaw, tilt = self.hand_pose(R)
        out = dict(obs)
        out["q"] = np.round(np.asarray(obs["q"], float) + dq, 6)
        out["ee_pos"] = np.round(f.site_xpos[self.site], 6)
        out["ee_quat"] = np.round(quat_from_mat(R), 6)
        out["ee_yaw"] = round(float(yaw), 6)
        out["ee_tilt"] = round(float(tilt), 6)
        return out

    # ------------------------------------------------------------------ the roll test
    def assembled(self):
        """What is latched, in the chassis frame: list of (type, dims, model, pos, R) and the wheel/axle links."""
        chassis = [mt.part for mt in self.mates if mt.name == "stand" and mt.part is not None]
        if not chassis:
            return None
        ci = chassis[0]
        cpos = self.latched[ci][0]
        Rinv = rot_z(-self.stand[2])                       # into the stand's frame: the car's length along x
        out = []
        for mt in self.mates:
            if mt.part is None:
                continue
            p = self.parts[mt.part]
            pos, quat = self.latched[mt.part]
            out.append({"type": p["type"], "dims": p["dims"], "model": p["model"], "mate": mt.name,
                        "pos": (Rinv @ (np.asarray(pos) - cpos)).tolist(),
                        "quat": list(map(float, quat_from_mat(Rinv @ mat_from_quat(quat)))),
                        "axle": None if mt.parent is None else self.parts[mt.parent]["dims"]})
        return out

    def roll_test(self):
        return roll_test(self.assembled())


def roll_test(assembly):
    """Set the latched car on a flat table in a clean simulation, push it along its length and report whether it
    rolls: it must travel ROLL_MIN_TRAVEL within ROLL_TIME and stay upright, with its cabin on. A wheel turns on its
    axle unless it binds against the chassis (its inner face inside the chassis's side)."""
    if not assembly:
        return {"rolls": False, "travel": 0.0, "reason": "no chassis on the stand"}
    have = {a["mate"] for a in assembly}
    if "cabin_seat" not in have:
        return {"rolls": False, "travel": 0.0, "reason": "no cabin"}
    chassis = [a for a in assembly if a["type"] == "chassis"][0]
    W = chassis["dims"]["width"]
    car = ['<geom name="floor" type="plane" size="5 5 0.1" friction="0.8 0.005 0.0001"/>']
    body = []
    wheels = []
    for k, a in enumerate(assembly):
        pos = " ".join("%.6f" % v for v in a["pos"])
        quat = " ".join("%.6f" % v for v in a["quat"])
        bits = 'contype="1" conaffinity="0"'          # the car's geoms touch the floor only
        geoms = part_geoms(a["type"], a["dims"], PAINT[a["type"]], "a%d_" % k, coll=bits).replace('condim="6"', 'condim="3"')
        if a["type"] == "wheel":
            inner = abs(a["pos"][1]) - a["dims"]["thick"] / 2
            binds = inner < W / 2 + 0.0005
            joint = "" if binds else '<joint name="spin%d" type="hinge" axis="0 0 1" damping="0.000002"/>' % k
            wheels.append(k)
            body.append('<body name="w%d" pos="%s" quat="%s">%s%s</body>' % (k, pos, quat, joint, geoms))
        else:
            body.append('<body name="p%d" pos="%s" quat="%s">%s</body>' % (k, pos, quat, geoms))
    zlift = 0.06
    xml = ('<mujoco><compiler angle="radian"/><option timestep="0.001" integrator="implicitfast"/>'
           '<worldbody>{floor}'
           '<body name="car" pos="0 0 {z}"><freejoint name="car_free"/>{body}</body></worldbody></mujoco>').format(
        floor=car[0], z=zlift, body="".join(body))
    try:
        m = mujoco.MjModel.from_xml_string(xml)
    except Exception as exc:  # noqa: BLE001
        return {"rolls": False, "travel": 0.0, "reason": "model: %s" % exc}
    d = mujoco.MjData(m)
    # drop the car onto its wheels and let it settle
    for _ in range(600):
        mujoco.mj_step(m, d)
    car_id = m.body("car").id
    x0 = d.xpos[car_id].copy()
    R = d.xmat[car_id].reshape(3, 3)
    fwd = R[:, 0].copy()
    fwd[2] = 0.0
    fwd /= max(1e-9, np.linalg.norm(fwd))
    d.qvel[0:3] = ROLL_PUSH_SPEED * fwd
    # the wheels spin up with the car (rolling start)
    tilt_max = 0.0
    floor = m.geom("floor").id
    wheel_geoms = {m.geom("a%d_tire" % k).id: k for k in wheels}
    on_floor = {k: 0 for k in wheels}
    n = int(ROLL_TIME / m.opt.timestep)
    for _ in range(n):
        mujoco.mj_step(m, d)
        Rc = d.xmat[car_id].reshape(3, 3)
        tilt_max = max(tilt_max, math.acos(max(-1.0, min(1.0, Rc[2, 2]))))
        touching = set()
        for c in d.contact[:d.ncon]:
            g = int(c.geom2) if int(c.geom1) == floor else int(c.geom1)
            if g in wheel_geoms:
                touching.add(wheel_geoms[g])
        for k in touching:
            on_floor[k] += 1
        if not np.all(np.isfinite(d.qpos)):
            return {"rolls": False, "travel": 0.0, "reason": "unstable"}
    travel = float(np.dot(d.xpos[car_id] - x0, fwd))
    four = len(wheels) == 4 and all(v >= 0.9 * n for v in on_floor.values())
    rolls = travel >= ROLL_MIN_TRAVEL and tilt_max <= ROLL_MAX_TILT and four
    return {"rolls": bool(rolls), "travel": round(travel, 4), "tilt": round(math.degrees(tilt_max), 2),
            "wheels_on_floor": int(sum(v >= 0.9 * n for v in on_floor.values())),
            "reason": "" if rolls else "travelled %.3f m, tilted %.1f deg, %d of 4 wheels on the table" % (
                travel, math.degrees(tilt_max), sum(v >= 0.9 * n for v in on_floor.values()))}

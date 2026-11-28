"""Ring re-sorting cell: a Franka Panda with long parallel jaws re-sorts colour-coded spacer rings on a fixture of
seven vertical spindles so that every product variant ends on its own spindle.

Where this file runs: in the grader (tests/ring_env.py) and in the practice-cell service of the agent's sandbox (a byte
copy readable by the service's account only). The agent never reads it.

The robot and its tracking law follow tasks/arm/keyed_nut_peg_assembly and kitting_controller_from_teleop_logs
(robosuite 1.5.2's Panda assets, an operational-space controller at 10 Hz, the hand pointing down with a yaw); the
jaws, the fixture, the rings, the sensors and the credit are this task's own.

Frames and units: world frame, the table top is z = 0.80; the Panda base stands at (-0.56, 0, 0.80) facing +x.
Metres, radians, seconds, newtons. Quaternions are (w, x, y, z).

    env = RingEnv()
    obs = env.reset(seed)                 # one episode: a fresh scrambled fixture
    obs, done, info = env.step(action)    # 10 Hz

Action (6 numbers): [dx, dy, dz, dyaw, grip, cams]
    dx..dz: displacement of the jaw point (TCP), world frame (m; the vector is clipped to length MAX_DPOS);
    dyaw:   rotation of the hand about the world vertical (rad; clipped to MAX_DYAW); the hand always points down;
    grip:   -1 = open (80 mm between the pads) ... +1 = closed, linear in between (finger force capped at 20 N);
    cams:   which images the next observation carries: 0 none, 1 overview, 2 wrist, 3 both.
"""

import math
import os

import numpy as np

import mujoco

# ----------------------------------------------------------------------------------------------
# Constants
#
# Difficulty knobs (each is documented to the agent in environment/setup/payload/cell/datasheet.md; change both):
#   R_BORE - R_SP        bore clearance (radial 0.75 mm)          SP_CHAMFER      lead-in at the tip (0.5 mm)
#   FIX_POS_TOL, FIX_YAW_TOL, SP_POS_TOL, SP_TILT_TOL             hidden fixture / spindle deviations
#   CAM_TOL              hidden camera mounting errors            COLOUR_TOL, VARIANT_RGB  colour similarity
#   WRENCH_NOISE/BIAS/DRIFT  force/torque sensor                  DAMAGE_FORCE, DAMAGE_STEPS  damage limit
#   VARIANTS, RINGS_PER_VARIANT, NSP, CAPACITY, scramble min_moves   puzzle size
#   VARIANT_MASS         hidden property (plain vs steel-cored rings of one colour; only weighing tells)
#   MAX_STEPS            takt per episode                         (practice budget: practice_service EPISODES)
# ----------------------------------------------------------------------------------------------
TIMESTEP = 0.002
SUBSTEPS = 50
CONTROL_DT = TIMESTEP * SUBSTEPS
MAX_STEPS = 10000
MAX_DPOS = 0.02
MAX_DYAW = 0.15
TABLE_Z = 0.80
TABLE_HALF = (0.40, 0.45)
ROBOT_BASE = np.array([-0.56, 0.0, TABLE_Z])
WORKSPACE_LO = np.array([-0.30, -0.40, TABLE_Z + 0.004])
WORKSPACE_HI = np.array([0.30, 0.40, TABLE_Z + 0.40])

# Rings (anodised aluminium spacer rings): bore, outside, height; collision = NSEG box segments + feet + rim spheres
NSEG = 16
R_BORE = 0.0080
R_OUT = 0.0200
RING_H = 0.016
FOOT_R = 0.0015
FOOT_PROTRUDE = 0.0002
RING_MASS = 0.045
RING_FRICTION = 0.35          # ring against the steel spindle and other rings (nominal)
BORE_RADIAL = (0.00075, 0.00075)  # per ring: bore radius minus spindle radius, uniform in this range (m)
FRICTION_RANGE = (0.35, 0.35)     # per episode: ring-spindle / ring-ring friction coefficient
VARIANTS = ("A", "B", "C", "D", "E", "F")
VARIANT_RGB = {               # nominal anodising colours (linear RGB of the renderer): pairs look the same
    "A": (0.10, 0.22, 0.72),  # blue, plain
    "B": (0.10, 0.22, 0.72),  # blue, steel-cored
    "C": (0.72, 0.10, 0.10),  # red, plain
    "D": (0.72, 0.10, 0.10),  # red, steel-cored
    "E": (0.14, 0.52, 0.22),  # green, plain
    "F": (0.14, 0.52, 0.22),  # green, steel-cored
}
VARIANT_MASS = {"A": 0.045, "B": 0.110, "C": 0.045, "D": 0.110, "E": 0.045, "F": 0.110}   # kg
COLOUR_TOL = 0.03
RINGS_PER_VARIANT = 4

# Fixture: base plate with NSP spindles in one row along the fixture's y axis
NSP = 7
CAPACITY = 4
SP_PITCH = 0.068
R_SP = 0.00725                # 14.5 mm spindle in a 16.0 mm bore: 1.5 mm diametral clearance
SP_LEN = 0.074                # above the plate top
SP_CHAMFER = 0.0003           # 45 degree chamfer at the tip
TIP_LEN = 0.0075              # the tip's collision mesh (the shaft below it is a capsule)
PLATE = (0.045, 0.262, 0.006)  # half sizes
FIXTURE_NOMINAL = (0.07, 0.0)  # plate centre (x, y), fixture yaw 0
FIX_POS_TOL = 0.004
FIX_YAW_TOL = math.radians(0.8)
SP_POS_TOL = 0.0015
SP_TILT_TOL = math.radians(1.0)
# The pallet the fixture sits on is bumped a few times per episode (a worn stop in the conveyor station): while the
# hand is clear of the fixture with its jaws open and empty, the whole fixture (plate, spindles and the rings on them) shifts within BUMP_STEPS
# control steps by BUMP_MM (random direction) and turns by up to BUMP_YAW. Its total offset from where the episode
# started stays within BUMP_MAX.
BUMPS = (2, 3)
BUMP_WINDOW = (300, 3600)
BUMP_MM = (0.006, 0.015)
BUMP_YAW = math.radians(1.0)
BUMP_MAX = 0.020
BUMP_STEPS = 10

# Jaws (custom long jaws on the Panda's fingers): pad centre = TCP
JAW_TCP = 0.1604              # TCP along the hand axis from the gripper base
PAD_FRICTION = 1.0

# Credit and damage
ON_AXIS_TOL = 0.0025          # ring centre from the spindle axis (bore clearance is 0.75 mm)
ON_TILT_TOL = math.radians(8.0)
REST_SPEED = 0.03
DAMAGE_FORCE = 25.0           # N: ring-spindle contact force (sum of normal forces)
DAMAGE_STEPS = 3              # consecutive control steps over DAMAGE_FORCE
SORTED_STEPS = 20             # the episode ends once the fixture has stayed sorted this many steps

GRIP_KP = 3000.0
FINGER_COUPLING = (0.005, 1.0)
INSTAB_ROBOT_QVEL = 8.0
INSTAB_RING_VEL = 8.0
HOME_Q = np.array([0.0, -0.35, 0.0, -2.35, 0.0, 2.0, math.pi / 4])
NULLSPACE_Q = np.array([0.0, -0.3, 0.0, -2.2, 0.0, 1.9, math.pi / 4])

OVERVIEW_CAM = ((0.52, -0.42, 1.22), (0.06, 0.0, 0.84), 40.0)
OVERVIEW_W, OVERVIEW_H = 240, 180
WRIST_FOVY = 75.0
WRIST_W, WRIST_H = 128, 96
CAMERA_NAMES = ("overview", "wrist")
CAM_TOL = {                   # the real cameras against their published calibration, per episode
    "overview": {"pos": 0.004, "rot_deg": 0.4, "focal": 0.01},
    "wrist": {"pos": 0.002, "rot_deg": 0.6, "focal": 0.01},
}
WRENCH_NOISE = (0.25, 0.02)
WRENCH_BIAS = (0.6, 0.05)
WRENCH_DRIFT = (0.25 / 60.0, 0.01 / 60.0)

# contact bit masks: 1 robot, 2 spindle, 4 ring segment, 8 ring foot/rim, 16 table/plate
CT_ROBOT, CT_SPINDLE, CT_SEG, CT_FOOT, CT_STATIC = 1, 2, 4, 8, 16


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


def yaw_quat(yaw):
    return np.array([math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)])


def yaw_of_mat(R):
    return math.atan2(R[1, 0], R[0, 0])


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
            raise ActionError("action must be a list of 6 finite numbers")
        out.append(float(v))
    cams = int(round(out[5]))
    if cams < 0 or cams > 3:
        raise ActionError("the camera mask must be 0, 1, 2 or 3")
    return np.array(out[:3]), out[3], float(np.clip(out[4], -1.0, 1.0)), cams


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


# ----------------------------------------------------------------------------------------------
# Puzzle (the arrangement of variants on spindles)
# ----------------------------------------------------------------------------------------------
def is_pure(stack):
    return len(stack) > 0 and all(v == stack[0] for v in stack)


def legal_moves(state, cap=CAPACITY):
    out = []
    for a, sa in enumerate(state):
        if not sa:
            continue
        for b, sb in enumerate(state):
            if a != b and len(sb) < cap:
                out.append((a, b))
    return out


def scramble(rng, min_moves=26, cap=CAPACITY):
    """A start: random legal moves applied to a sorted fixture until it is a fresh start (one spindle empty, the six
    others full and none of them pure) after at least min_moves moves. Returns (state, moves applied)."""
    for _ in range(200):
        empty = int(rng.integers(NSP))
        order = list(rng.permutation(len(VARIANTS)))
        state, k = [], 0
        for s in range(NSP):
            if s == empty:
                state.append([])
            else:
                state.append([int(order[k])] * RINGS_PER_VARIANT)
                k += 1
        last = None
        for n in range(400):
            moves = [mv for mv in legal_moves(state, cap) if last is None or mv != (last[1], last[0])]
            a, b = moves[int(rng.integers(len(moves)))]
            state[b].append(state[a].pop())
            last = (a, b)
            if n + 1 >= min_moves:
                sizes = sorted(len(s) for s in state)
                if sizes == [0] + [cap] * (NSP - 1) and not any(is_pure(s) for s in state):
                    return [list(s) for s in state], n + 1
    raise EnvError("could not scramble")


def heuristic(state):
    h = 0
    claimed = {}
    for s in state:
        k = 0
        while k < len(s) and s[k] == s[0]:
            k += 1
        if s:
            claimed.setdefault(s[0], []).append(k)
        h += len(s) - k
    return h


def plan(state, cap=CAPACITY, weight=2.0, max_expand=200000):
    """Weighted A* over arrangements: a list of moves (from, to) that sorts `state` (each variant on its own spindle)."""
    import heapq
    start = tuple(tuple(s) for s in state)

    def done(st):
        seen = set()
        for s in st:
            if not s:
                continue
            if not is_pure(s) or len(s) != RINGS_PER_VARIANT or s[0] in seen:
                return False
            seen.add(s[0])
        return True

    def key(st):
        return tuple(sorted(st))

    openq = [(weight * heuristic(start), 0, start, None)]
    parent = {key(start): (None, None)}
    gbest = {key(start): 0}
    n = 0
    while openq:
        f, g, st, _ = heapq.heappop(openq)
        if done(st):
            moves, k = [], key(st)
            node = st
            path = []
            while True:
                prev, mv = parent[key(node)]
                if prev is None:
                    break
                path.append(mv)
                node = prev
            return path[::-1]
        n += 1
        if n > max_expand:
            break
        for a, b in legal_moves(st, cap):
            # never break a pure full spindle, never move a ring onto a spindle whose rings would stay mixed below it
            ns = [list(s) for s in st]
            ns[b].append(ns[a].pop())
            nt = tuple(tuple(s) for s in ns)
            k = key(nt)
            if k in gbest and gbest[k] <= g + 1:
                continue
            gbest[k] = g + 1
            parent[k] = (st, (a, b))
            heapq.heappush(openq, (g + 1 + weight * heuristic(nt), g + 1, nt, None))
    raise EnvError("no plan found")


# ----------------------------------------------------------------------------------------------
# Scene
# ----------------------------------------------------------------------------------------------
SCENE_XML = """<mujoco model="ring_cell">
  <compiler angle="radian" autolimits="true"/>
  <option timestep="{dt}" integrator="implicitfast" cone="elliptic" impratio="10"/>
  <visual><global offwidth="1280" offheight="960"/><quality shadowsize="2048"/>
    <headlight ambient="0.22 0.22 0.22" diffuse="0.22 0.22 0.22"/></visual>
  <asset>
    <texture name="floor" type="2d" builtin="flat" mark="random" random="0.15" rgb1="0.50 0.50 0.48" rgb2="0.42 0.42 0.40" width="256" height="256"/>
    <material name="floormat" texture="floor" texrepeat="8 8" specular="0.05"/>
    <texture name="wallt" type="2d" builtin="gradient" rgb1="0.86 0.87 0.88" rgb2="0.70 0.72 0.74" width="128" height="128"/>
    <material name="wallmat" texture="wallt"/>
    <material name="alu" rgba="0.72 0.74 0.76 1" specular="0.5" shininess="0.6"/>
    <material name="steel" rgba="0.66 0.67 0.69 1" specular="0.8" shininess="0.8" reflectance="0.05"/>
    <material name="plate" rgba="0.24 0.25 0.27 1" specular="0.3" shininess="0.4"/>
    <texture name="mat" type="2d" builtin="flat" mark="random" random="0.06" rgb1="0.36 0.37 0.36" rgb2="0.3 0.3 0.3" width="256" height="256"/>
    <material name="tablemat" texture="mat" texrepeat="3 3" specular="0.1"/>
  </asset>
  <default><geom solref="0.004 1" solimp="0.95 0.99 0.001"/></default>
  <worldbody>
    <light name="lamp" pos="0.2 0.1 2.2" dir="-0.05 -0.05 -1" diffuse="0.45 0.45 0.43" specular="0.25 0.25 0.25" castshadow="true"/>
    <light name="fill" pos="0.9 -0.8 1.8" dir="-0.4 0.4 -0.6" diffuse="0.25 0.25 0.27" castshadow="false"/>
    <geom name="floor" type="plane" size="3 3 0.1" material="floormat" contype="16" conaffinity="0"/>
    <geom name="table" type="box" pos="0 0 {tz2}" size="{thx} {thy} {tz2}" material="tablemat" friction="0.6 0.005 0.0001" contype="16" conaffinity="1"/>
    <geom name="pedestal" type="box" pos="{rx} 0 {tz2}" size="0.12 0.12 {tz2}" rgba="0.25 0.25 0.27 1" contype="0" conaffinity="0"/>
    <geom name="wall_back" type="box" pos="1.7 0 1.5" size="0.02 3.0 1.5" material="wallmat" contype="0" conaffinity="0"/>
    <geom name="wall_side" type="box" pos="0 2.4 1.5" size="3.0 0.02 1.5" material="wallmat" contype="0" conaffinity="0"/>
    <geom name="wall_back2" type="box" pos="-2.2 0 1.5" size="0.02 3.0 1.5" material="wallmat" contype="0" conaffinity="0"/>
    <geom name="wall_side2" type="box" pos="0 -2.4 1.5" size="3.0 0.02 1.5" material="wallmat" contype="0" conaffinity="0"/>
    <geom name="post1" type="box" pos="0.62 0.62 1.05" size="0.02 0.02 1.05" material="alu" contype="0" conaffinity="0"/>
    <geom name="post2" type="box" pos="0.62 -0.62 1.05" size="0.02 0.02 1.05" material="alu" contype="0" conaffinity="0"/>
    <geom name="post3" type="box" pos="-0.75 0.62 1.05" size="0.02 0.02 1.05" material="alu" contype="0" conaffinity="0"/>
    <geom name="post4" type="box" pos="-0.75 -0.62 1.05" size="0.02 0.02 1.05" material="alu" contype="0" conaffinity="0"/>
    <geom name="beam1" type="box" pos="-0.065 0.62 2.1" size="0.705 0.02 0.02" material="alu" contype="0" conaffinity="0"/>
    <geom name="beam2" type="box" pos="-0.065 -0.62 2.1" size="0.705 0.02 0.02" material="alu" contype="0" conaffinity="0"/>
    <geom name="beam3" type="box" pos="0.62 0 2.1" size="0.02 0.62 0.02" material="alu" contype="0" conaffinity="0"/>
    <geom name="cabinet" type="box" pos="-1.05 -0.9 0.9" size="0.25 0.2 0.9" rgba="0.82 0.83 0.80 1" contype="0" conaffinity="0"/>
  </worldbody>
</mujoco>"""


def ring_mesh(nv=48):
    """Annulus (bore R_BORE, outside R_OUT, height RING_H) as a visual mesh; every face group has its own vertices
    so that the edges render sharp. Origin: axis on the bottom face."""
    verts, faces = [], []
    ang = [2 * math.pi * k / nv for k in range(nv)]

    def ring(r, z):
        base = len(verts) // 3
        for a in ang:
            verts.extend([r * math.cos(a), r * math.sin(a), z])
        return base

    for (z, up) in ((RING_H, True), (0.0, False)):           # top and bottom annuli
        o, i = ring(R_OUT, z), ring(R_BORE, z)
        for k in range(nv):
            k1 = (k + 1) % nv
            if up:
                faces.extend([o + k, o + k1, i + k1, o + k, i + k1, i + k])
            else:
                faces.extend([o + k, i + k1, o + k1, o + k, i + k, i + k1])
    for (r, out) in ((R_OUT, True), (R_BORE, False)):         # outer and bore walls
        b, t = ring(r, 0.0), ring(r, RING_H)
        for k in range(nv):
            k1 = (k + 1) % nv
            if out:
                faces.extend([b + k, b + k1, t + k1, b + k, t + k1, t + k])
            else:
                faces.extend([b + k, t + k1, b + k1, b + k, t + k, t + k1])
    return verts, faces


def tip_mesh(nv=32):
    """Spindle tip (convex): the last TIP_LEN of the spindle with its 45 degree chamfer. Origin: plate top."""
    verts = []
    for (r, z) in ((R_SP, SP_LEN - TIP_LEN), (R_SP, SP_LEN - SP_CHAMFER), (R_SP - SP_CHAMFER, SP_LEN)):
        for k in range(nv):
            a = 2 * math.pi * k / nv
            verts.extend([r * math.cos(a), r * math.sin(a), z])
    return verts


def ring_body(spec_body, idx, rgba, mass=RING_MASS, r_bore=None, friction=None):
    """Collision and visual geoms of ring idx under an MjSpec body."""
    b = spec_body
    b.add_freejoint(name="ring%d_free" % idx)
    r_bore = R_BORE if r_bore is None else r_bore
    friction = RING_FRICTION if friction is None else friction
    izz = mass / 2 * (R_OUT ** 2 + r_bore ** 2)
    ixx = mass / 12 * (3 * (R_OUT ** 2 + r_bore ** 2) + RING_H ** 2)
    b.mass = mass
    b.ipos = [0, 0, RING_H / 2]
    b.inertia = [ixx, ixx, izz]
    b.explicitinertial = True
    da = 2 * math.pi / NSEG
    half_t = R_OUT * math.tan(da / 2) + 0.0001
    rc = (r_bore + R_OUT) / 2
    for s in range(NSEG):
        a = (s + 0.5) * da
        g = b.add_geom()
        g.name = "ring%d_seg%d" % (idx, s)
        g.type = mujoco.mjtGeom.mjGEOM_BOX
        g.size = [(R_OUT - r_bore) / 2, half_t, RING_H / 2]
        g.pos = [rc * math.cos(a), rc * math.sin(a), RING_H / 2]
        g.quat = [math.cos(a / 2), 0, 0, math.sin(a / 2)]
        g.contype, g.conaffinity = CT_SEG, CT_ROBOT | CT_SPINDLE | CT_STATIC
        g.friction = [friction, 0.005, 0.0001]
        g.condim = 4
        g.group = 3
        g.rgba = [0, 0, 0, 0]
        g.mass = 0
    for k in range(3):
        a = 2 * math.pi * k / 3 + math.pi / 6
        g = b.add_geom()
        g.name = "ring%d_foot%d" % (idx, k)
        g.type = mujoco.mjtGeom.mjGEOM_SPHERE
        g.size = [FOOT_R, 0, 0]
        g.pos = [0.014 * math.cos(a), 0.014 * math.sin(a), FOOT_R - FOOT_PROTRUDE]
        g.contype, g.conaffinity = CT_FOOT, CT_SEG | CT_STATIC
        g.friction = [friction, 0.005, 0.0001]
        g.condim = 3
        g.group = 3
        g.rgba = [0, 0, 0, 0]
        g.mass = 0
    for k in range(6):
        a = 2 * math.pi * k / 6
        for z in (0.004, RING_H - 0.004):
            g = b.add_geom()
            g.name = "ring%d_rim%d_%d" % (idx, k, int(z * 1e4))
            g.type = mujoco.mjtGeom.mjGEOM_SPHERE
            g.size = [0.0035, 0, 0]
            g.pos = [(R_OUT - 0.0035) * math.cos(a), (R_OUT - 0.0035) * math.sin(a), z]
            g.contype, g.conaffinity = CT_FOOT, CT_SEG
            g.friction = [friction, 0.005, 0.0001]
            g.condim = 3
            g.group = 3
            g.rgba = [0, 0, 0, 0]
            g.mass = 0
    g = b.add_geom()
    g.name = "ring%d_vis" % idx
    g.type = mujoco.mjtGeom.mjGEOM_MESH
    g.meshname = "ring_vis"
    g.contype, g.conaffinity = 0, 0
    g.group = 1
    g.rgba = list(rgba)
    g.mass = 0
    g.material = "ring%d_mat" % idx


def build_model(layout, look):
    """Compile table + Panda with jaws + fixture + rings. layout: {"plate": (x, y, yaw), "spindles": [(x, y, tilt
    rotation vector)] * NSP (world), "rings": [(variant, spindle, level)]}; look: {"light", "rgba": [per ring]}."""
    root = mujoco.MjSpec.from_string(SCENE_XML.format(
        dt=TIMESTEP, tz2=TABLE_Z / 2, thx=TABLE_HALF[0], thy=TABLE_HALF[1], rx=ROBOT_BASE[0]))
    wb = root.worldbody
    pos, tgt, fovy = OVERVIEW_CAM
    c = wb.add_camera()
    c.name = "overview"
    c.pos = list(pos)
    c.quat = list(quat_from_mat(look_at(pos, tgt)))
    c.fovy = fovy
    for name in ("lamp", "fill"):
        pass
    # meshes
    mv, mf = ring_mesh()
    me = root.add_mesh()
    me.name = "ring_vis"
    me.uservert = mv
    me.userface = mf
    me.inertia = mujoco.mjtMeshInertia.mjMESH_INERTIA_SHELL
    mt = root.add_mesh()
    mt.name = "tip"
    mt.uservert = tip_mesh()
    for i, rgba in enumerate(look["rgba"]):
        root.add_material(name="ring%d_mat" % i, rgba=list(rgba), specular=0.35, shininess=0.5)
    # robot
    assets = robosuite_assets()
    robot = mujoco.MjSpec.from_file(os.path.join(assets, "robots", "panda", "robot.xml"))
    grip = mujoco.MjSpec.from_file(os.path.join(assets, "grippers", "panda_gripper.xml"))
    for cam in list(robot.cameras):
        if cam.name == "robotview":
            robot.delete(cam)
    for gname in ("finger1_collision", "finger2_collision", "finger1_pad_collision", "finger2_pad_collision"):
        grip.delete(grip.geom(gname))
    grip.add_material(name="jaw", rgba=[0.55, 0.56, 0.58, 1], specular=0.4, shininess=0.5)
    grip.add_material(name="padmat", rgba=[0.12, 0.12, 0.13, 1], specular=0.1)
    for fname, sgn in (("leftfinger", 1.0), ("rightfinger", -1.0)):
        fb = grip.body(fname)
        g = fb.add_geom()
        g.name = "jaw%s" % ("1" if sgn > 0 else "2")
        g.type = mujoco.mjtGeom.mjGEOM_BOX
        g.size = [0.0105, 0.003, 0.0395]
        g.pos = [0, sgn * 0.007, 0.0745]
        g.contype, g.conaffinity = CT_ROBOT, CT_ROBOT
        g.friction = [0.5, 0.005, 0.0001]
        g.material = "jaw"
        g.group = 1
        g.mass = 0.04
        p = fb.add_geom()
        p.name = "pad%s" % ("1" if sgn > 0 else "2")
        p.type = mujoco.mjtGeom.mjGEOM_BOX
        p.size = [0.0100, 0.0020, 0.0060]
        p.pos = [0, sgn * 0.0020, 0.1080]
        p.contype, p.conaffinity = CT_ROBOT, CT_ROBOT
        p.friction = [PAD_FRICTION, 0.02, 0.0001]
        p.condim = 4
        p.solref = [0.004, 1.0]
        p.material = "padmat"
        p.group = 1
        p.mass = 0.005
    grip.body("eef").pos = [0, 0, JAW_TCP]
    robot.body("right_hand").add_frame().attach_body(grip.body("right_gripper"), "g_", "")
    wb.add_frame(pos=list(ROBOT_BASE)).attach_body(robot.body("base"), "r_", "")
    # fixture
    px, py, pyaw = layout["plate"]
    pb = wb.add_body(name="fixture", pos=[px, py, TABLE_Z], quat=list(yaw_quat(pyaw)), mocap=True)
    Rf = rot_z(pyaw)
    g = pb.add_geom(name="plate", type=mujoco.mjtGeom.mjGEOM_BOX, size=list(PLATE), pos=[0, 0, PLATE[2]])
    g.contype, g.conaffinity = CT_STATIC, CT_ROBOT
    g.material = "plate"
    g.friction = [0.4, 0.005, 0.0001]
    ptop = TABLE_Z + 2 * PLATE[2]
    for k, (sx, sy, tilt) in enumerate(layout["spindles"]):
        lp = Rf.T @ np.array([sx - px, sy - py, 0.0])
        sb = pb.add_body(name="spindle%d" % k, pos=[lp[0], lp[1], 2 * PLATE[2]],
                         quat=list(quat_from_mat(Rf.T @ axis_angle_mat(tilt) @ Rf)))
        z0, z1 = R_SP - 0.004, SP_LEN - R_SP
        g = sb.add_geom(name="sp%d_shaft" % k, type=mujoco.mjtGeom.mjGEOM_CAPSULE, size=[R_SP, (z1 - z0) / 2, 0],
                        pos=[0, 0, (z0 + z1) / 2])
        g.contype, g.conaffinity = CT_SPINDLE, CT_ROBOT
        g.material = "steel"
        g.friction = [layout.get("friction", RING_FRICTION), 0.005, 0.0001]
        g = sb.add_geom(name="sp%d_tip" % k, type=mujoco.mjtGeom.mjGEOM_MESH, meshname="tip")
        g.contype, g.conaffinity = CT_SPINDLE, CT_ROBOT
        g.material = "steel"
        g.friction = [layout.get("friction", RING_FRICTION), 0.005, 0.0001]
        g = sb.add_geom(name="sp%d_collar" % k, type=mujoco.mjtGeom.mjGEOM_CYLINDER, size=[0.0125, 0.0015, 0],
                        pos=[0, 0, -0.0010])
        g.contype, g.conaffinity = 0, 0
        g.material = "steel"
    # rings
    for i in range(len(layout["rings"])):
        rb = wb.add_body(name="ring%d" % i, pos=[0.3, 0.5 + 0.05 * i, 0.9])
        ring_body(rb, i, look["rgba"][i], VARIANT_MASS[VARIANTS[layout["rings"][i][0]]],
                  layout.get("bores", [R_BORE] * 99)[i], layout.get("friction", RING_FRICTION))
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
    cid = m.camera("r_eye_in_hand").id
    m.cam_fovy[cid] = WRIST_FOVY
    for name, cname in (("overview", "overview"), ("wrist", "r_eye_in_hand")):
        err = look.get("cams", {}).get(name)
        if err is None:
            continue
        ci = m.camera(cname).id
        m.cam_pos[ci] = m.cam_pos[ci] + np.asarray(err["pos"], float)
        m.cam_quat[ci] = quat_from_mat(axis_angle_mat(err["rot"]) @ mat_from_quat(m.cam_quat[ci]))
        fov = math.radians(float(m.cam_fovy[ci]))
        m.cam_fovy[ci] = math.degrees(2 * math.atan(math.tan(fov / 2) / (1.0 + float(err["focal"]))))
    m.site_rgba[:, 3] = 0.0
    # the robot's collision meshes coincide with its visual meshes: never drawn
    for gi in range(m.ngeom):
        bname = m.body(m.geom_bodyid[gi]).name
        if bname.startswith("r_") and m.geom_group[gi] == 0 and m.geom_type[gi] == mujoco.mjtGeom.mjGEOM_MESH:
            m.geom_rgba[gi, 3] = 0.0
    gain = float(look["light"])
    for name in ("lamp", "fill"):
        lid = m.light(name).id
        m.light_diffuse[lid] = m.light_diffuse[lid] * gain
    m.vis.headlight.diffuse[:] = m.vis.headlight.diffuse * gain
    m.vis.headlight.ambient[:] = m.vis.headlight.ambient * gain
    return m


# ----------------------------------------------------------------------------------------------
# Episode draws
# ----------------------------------------------------------------------------------------------
def nominal_spindles():
    fx, fy = FIXTURE_NOMINAL
    return [(fx, fy + (k - (NSP - 1) / 2) * SP_PITCH) for k in range(NSP)]


def sample_fixture(rng):
    fx, fy = FIXTURE_NOMINAL
    px = fx + rng.uniform(-FIX_POS_TOL, FIX_POS_TOL)
    py = fy + rng.uniform(-FIX_POS_TOL, FIX_POS_TOL)
    pyaw = rng.uniform(-FIX_YAW_TOL, FIX_YAW_TOL)
    c, s = math.cos(pyaw), math.sin(pyaw)
    sp = []
    for k in range(NSP):
        ly = (k - (NSP - 1) / 2) * SP_PITCH
        lx, lyy = rng.uniform(-SP_POS_TOL, SP_POS_TOL), ly + rng.uniform(-SP_POS_TOL, SP_POS_TOL)
        az = rng.uniform(-math.pi, math.pi)
        th = SP_TILT_TOL * math.sqrt(rng.uniform(0.0, 1.0))
        tilt = [th * -math.sin(az), th * math.cos(az), 0.0]
        sp.append((px + c * lx - s * lyy, py + s * lx + c * lyy, tilt))
    return {"plate": (px, py, pyaw), "spindles": sp}


def sample_look(rng, n_rings):
    rgba = []
    return {"light": float(rng.uniform(0.75, 1.25)), "jitter": rng.uniform(-COLOUR_TOL, COLOUR_TOL, (n_rings, 3))}


# ----------------------------------------------------------------------------------------------
# The cell
# ----------------------------------------------------------------------------------------------
class RingEnv(object):
    def __init__(self, render=True, max_steps=MAX_STEPS, min_scramble=26):
        self.render_enabled = render
        self.max_steps = int(max_steps)
        self.min_scramble = int(min_scramble)
        self.model = None
        self._renderers = {}

    # ------------------------------------------------------------------ episode
    def reset(self, seed):
        self.seed = int(seed) & (2 ** 64 - 1)
        s = [self.seed & 0xFFFFFFFF, self.seed >> 32]
        self.rng = np.random.default_rng(s + [3])
        fixture = sample_fixture(np.random.default_rng(s + [5]))
        self.start, self.scramble_moves = scramble(np.random.default_rng(s + [7]), self.min_scramble)
        rings = []
        for sp, stack in enumerate(self.start):
            for lev, v in enumerate(stack):
                rings.append((int(v), sp, lev))
        self.ring_variant = [r[0] for r in rings]
        lr = np.random.default_rng(s + [11])
        light = float(lr.uniform(0.75, 1.25))
        rgba = []
        for v, _sp, _lev in rings:
            base = np.array(VARIANT_RGB[VARIANTS[v]])
            rgba.append(list(np.clip(base + lr.uniform(-COLOUR_TOL, COLOUR_TOL, 3), 0, 1)) + [1.0])
        cams = {}
        for name in CAMERA_NAMES:
            tol = CAM_TOL[name]
            cams[name] = {"pos": lr.uniform(-tol["pos"], tol["pos"], 3).tolist(),
                          "rot": lr.uniform(-math.radians(tol["rot_deg"]), math.radians(tol["rot_deg"]), 3).tolist(),
                          "focal": float(lr.uniform(-tol["focal"], tol["focal"]))}
        self.look = {"light": light, "rgba": rgba, "cams": cams}
        pr = np.random.default_rng(s + [23])
        bores = [float(R_SP + pr.uniform(*BORE_RADIAL)) for _ in rings]
        self.layout = dict(fixture, rings=rings, bores=bores, friction=float(pr.uniform(*FRICTION_RANGE)))
        self.close()
        self.model = build_model(self.layout, self.look)
        self.data = mujoco.MjData(self.model)
        self.fk = mujoco.MjData(self.model)
        self._index()
        self._init_state()
        self.t = 0
        self.done = False
        self.fault = None
        self.damaged = [False] * self.n_rings
        self.force_count = [0] * self.n_rings
        self.sorted_count = 0
        self.cams_next = 3
        wr = np.random.default_rng(s + [13])
        self.wrench_bias = np.concatenate([wr.uniform(-WRENCH_BIAS[0], WRENCH_BIAS[0], 3),
                                           wr.uniform(-WRENCH_BIAS[1], WRENCH_BIAS[1], 3)])
        self.wrench_drift = np.concatenate([wr.uniform(-WRENCH_DRIFT[0], WRENCH_DRIFT[0], 3),
                                            wr.uniform(-WRENCH_DRIFT[1], WRENCH_DRIFT[1], 3)])
        self.noise_rng = np.random.default_rng(s + [17])
        br = np.random.default_rng(s + [19])
        self.bump_rng = br
        nb = int(br.integers(BUMPS[0], BUMPS[1] + 1))
        self.bump_times = sorted(int(x) for x in br.uniform(BUMP_WINDOW[0], BUMP_WINDOW[1], nb))
        self.bump = None                      # (t0, start pos, start yaw, end pos, end yaw) while a bump runs
        self.fix_mid = self.model.body("fixture").mocapid[0]
        self.fix_pos0 = self.data.mocap_pos[self.fix_mid].copy()
        self.fix_yaw0 = float(self.layout["plate"][2])
        self.fix_yaw = self.fix_yaw0
        self.bumps_done = 0
        return self.observation()

    def _index(self):
        m = self.model
        self.n_rings = len(self.layout["rings"])
        self.jq = np.array([m.joint("r_joint%d" % i).qposadr[0] for i in range(1, 8)])
        self.jv = np.array([m.joint("r_joint%d" % i).dofadr[0] for i in range(1, 8)])
        self.act = np.array([m.actuator("r_torq_j%d" % i).id for i in range(1, 8)])
        self.gact = [m.actuator("r_g_gripper_finger_joint1").id, m.actuator("r_g_gripper_finger_joint2").id]
        self.gq = [m.joint("r_g_finger_joint1").qposadr[0], m.joint("r_g_finger_joint2").qposadr[0]]
        self.site = m.site("r_g_grip_site").id
        self.ring_bid = [m.body("ring%d" % i).id for i in range(self.n_rings)]
        self.ring_qadr = [int(m.joint("ring%d_free" % i).qposadr[0]) for i in range(self.n_rings)]
        self.ring_dadr = [int(m.joint("ring%d_free" % i).dofadr[0]) for i in range(self.n_rings)]
        self.geom_ring = np.full(m.ngeom, -1, int)
        for i, b in enumerate(self.ring_bid):
            self.geom_ring[m.geom_bodyid == b] = i
        self.sp_bid = [m.body("spindle%d" % k).id for k in range(NSP)]
        self.geom_spindle = np.full(m.ngeom, -1, int)
        for k, b in enumerate(self.sp_bid):
            self.geom_spindle[m.geom_bodyid == b] = k
        jb = [m.body(b).id for b in range(m.nbody) if m.body(b).name in ("r_g_leftfinger", "r_g_rightfinger")]
        self.finger_geoms = set(int(i) for i in np.where(np.isin(m.geom_bodyid, jb))[0])
        self.ft_force = m.sensor("r_g_force_ee").adr[0]
        self.ft_torque = m.sensor("r_g_torque_ee").adr[0]
        self.ft_site = m.site("r_g_ft_frame").id
        self._jp, self._jr = np.zeros((3, m.nv)), np.zeros((3, m.nv))
        self._M = np.zeros((m.nv, m.nv))
        self.plate_top = TABLE_Z + 2 * PLATE[2]

    def spindle_axis(self, k):
        """(foot point on the plate top, unit axis) of spindle k (world)."""
        d = self.data
        b = self.sp_bid[k]
        return d.xpos[b].copy(), d.xmat[b].reshape(3, 3)[:, 2].copy()

    def _place_rings(self):
        m, d = self.model, self.data
        for i, (_v, sp, lev) in enumerate(self.layout["rings"]):
            foot, ax = self.spindle_axis(sp)
            R = d.xmat[self.sp_bid[sp]].reshape(3, 3)
            yaw = self.rng.uniform(-math.pi, math.pi)
            Rr = R @ rot_z(yaw)
            p = foot + ax * (FOOT_PROTRUDE + 0.0002 + lev * (RING_H + FOOT_PROTRUDE + 0.0002))
            a = self.ring_qadr[i]
            d.qpos[a:a + 3] = p
            d.qpos[a + 3:a + 7] = quat_from_mat(Rr)

    def _init_state(self):
        m, d = self.model, self.data
        mujoco.mj_resetData(m, d)
        d.qpos[self.jq] = HOME_Q
        d.qpos[self.gq[0]], d.qpos[self.gq[1]] = 0.04, -0.04
        mujoco.mj_forward(m, d)
        self._place_rings()
        mujoco.mj_forward(m, d)
        self.q_stale = d.qpos.copy()
        self.grip_target = 0.04
        q_hold = d.qpos[self.jq].copy()
        for _ in range(250):
            d.ctrl[self.act] = 200.0 * (q_hold - d.qpos[self.jq]) - 20.0 * d.qvel[self.jv] + d.qfrc_bias[self.jv]
            d.ctrl[self.gact] = [0.04, -0.04]
            mujoco.mj_step(m, d)
        self.q_stale[:] = d.qpos
        self.target_pos = d.site_xpos[self.site].copy()
        R = d.site_xmat[self.site].reshape(3, 3).copy()
        self.R_down = rot_z(-yaw_of_mat(R)) @ R
        self.target_yaw = yaw_of_mat(R)

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
        return rot_z(self.target_yaw) @ self.R_down

    def step(self, action):
        if self.done:
            raise EnvError("episode already finished")
        dp, dyaw, g, cams = parse_action(action)
        d = self.data
        n = np.linalg.norm(dp)
        if n > MAX_DPOS:
            dp = dp * (MAX_DPOS / n)
        dyaw = float(np.clip(dyaw, -MAX_DYAW, MAX_DYAW))
        cur_p = d.site_xpos[self.site].copy()
        self.target_pos = np.clip(cur_p + dp, WORKSPACE_LO, WORKSPACE_HI)
        self.target_yaw = wrap_angle(self.ee_yaw() + dyaw)
        self.grip_target = 0.04 * (1.0 - g) / 2.0
        self.cams_next = cams
        self._bump_start()
        self._physics_step()
        self.t += 1
        self._post_step()
        return self.observation(), self.done, self.info()

    def _bump_start(self):
        if self.bump is not None or not self.bump_times or self.t < self.bump_times[0]:
            return
        d = self.data
        tcp = d.site_xpos[self.site]
        fp = d.mocap_pos[self.fix_mid]
        c, s_ = math.cos(self.fix_yaw), math.sin(self.fix_yaw)
        lx, ly = c * (tcp[0] - fp[0]) + s_ * (tcp[1] - fp[1]), -s_ * (tcp[0] - fp[0]) + c * (tcp[1] - fp[1])
        # clear: the jaws (and a ring they hold) are above every spindle tip, or the hand is away from the fixture
        clear = tcp[2] > TABLE_Z + 2 * PLATE[2] + SP_LEN + 0.020 or abs(lx) > PLATE[0] + 0.07 or abs(ly) > PLATE[1] + 0.07
        # ... and the jaws are open and hold nothing (the cell's interlock lets the conveyor index the neighbouring
        # pallets only then)
        opening = float(d.qpos[self.gq[0]] - d.qpos[self.gq[1]])
        if not clear or opening < 0.06 or self.touching_jaws():
            return
        self.bump_times.pop(0)
        r = self.bump_rng
        for _ in range(50):
            a = r.uniform(-math.pi, math.pi)
            mag = r.uniform(BUMP_MM[0], BUMP_MM[1])
            end = fp + np.array([mag * math.cos(a), mag * math.sin(a), 0.0])
            if np.linalg.norm(end[:2] - self.fix_pos0[:2]) <= BUMP_MAX:
                break
        yaw_end = self.fix_yaw + r.uniform(-BUMP_YAW, BUMP_YAW)
        yaw_end = self.fix_yaw0 + float(np.clip(yaw_end - self.fix_yaw0, -1.5 * BUMP_YAW, 1.5 * BUMP_YAW))
        self.bump = (self.t, fp.copy(), self.fix_yaw, end, yaw_end)
        self.bumps_done += 1

    def _bump_pose(self, frac):
        t0, p0, y0, p1, y1 = self.bump
        u = frac * frac * (3 - 2 * frac)                  # smooth start and stop
        return p0 + (p1 - p0) * u, y0 + (y1 - y0) * u

    def _physics_step(self):
        m, d = self.model, self.data
        gw = [self.grip_target, -self.grip_target]
        for i in range(SUBSTEPS):
            if self.bump is not None:
                frac = min(1.0, ((self.t - self.bump[0]) * SUBSTEPS + i + 1) / float(BUMP_STEPS * SUBSTEPS))
                p, yaw = self._bump_pose(frac)
                d.mocap_pos[self.fix_mid] = p
                d.mocap_quat[self.fix_mid] = yaw_quat(yaw)
                if frac >= 1.0:
                    self.fix_yaw = yaw
                    self.bump = None
            d.ctrl[self.act] = self._osc_torque()
            d.ctrl[self.gact] = gw
            self.q_stale[:] = d.qpos
            mujoco.mj_step(m, d)
        if not np.all(np.isfinite(d.qpos)) or not np.all(np.isfinite(d.qvel)):
            self.fault = "non-finite state"

    def ee_yaw(self):
        return yaw_of_mat(self.data.site_xmat[self.site].reshape(3, 3) @ self.R_down.T)

    # ------------------------------------------------------------------ contacts, credit
    def ring_spindle_forces(self):
        """Per ring: the sum of the normal forces of its contacts with any spindle (N)."""
        d = self.data
        out = np.zeros(self.n_rings)
        f6 = np.zeros(6)
        for k in range(d.ncon):
            c = d.contact[k]
            g1, g2 = int(c.geom1), int(c.geom2)
            r1, r2 = self.geom_ring[g1], self.geom_ring[g2]
            s1, s2 = self.geom_spindle[g1], self.geom_spindle[g2]
            if (r1 >= 0 and s2 >= 0) or (r2 >= 0 and s1 >= 0):
                mujoco.mj_contactForce(self.model, d, k, f6)
                out[r1 if r1 >= 0 else r2] += abs(f6[0])
        return out

    def touching_jaws(self):
        d = self.data
        out = set()
        for k in range(d.ncon):
            c = d.contact[k]
            g1, g2 = int(c.geom1), int(c.geom2)
            if g1 in self.finger_geoms and self.geom_ring[g2] >= 0:
                out.add(int(self.geom_ring[g2]))
            elif g2 in self.finger_geoms and self.geom_ring[g1] >= 0:
                out.add(int(self.geom_ring[g1]))
        return out

    def ring_pose(self, i):
        d = self.data
        b = self.ring_bid[i]
        return d.xpos[b].copy(), d.xmat[b].reshape(3, 3).copy()

    def ring_on(self, i):
        """The spindle ring i is threaded on (its bore around the spindle, below the tip), or -1."""
        p, R = self.ring_pose(i)
        c = p + R[:, 2] * (RING_H / 2)
        for k in range(NSP):
            foot, ax = self.spindle_axis(k)
            u = float(np.dot(c - foot, ax))
            if u < 0 or u > SP_LEN - RING_H / 2 + 0.002:
                continue
            off = np.linalg.norm(c - foot - u * ax)
            if off < ON_AXIS_TOL and abs(float(np.dot(R[:, 2], ax))) > math.cos(ON_TILT_TOL):
                return k
        return -1

    def arrangement(self):
        """Per spindle: the rings threaded on it and at rest, bottom to top; plus the rings elsewhere."""
        d = self.data
        jaws = self.touching_jaws()
        per = [[] for _ in range(NSP)]
        loose = []
        for i in range(self.n_rings):
            k = self.ring_on(i)
            v = d.qvel[self.ring_dadr[i]:self.ring_dadr[i] + 3]
            if k >= 0:
                p, R = self.ring_pose(i)
                per[k].append((float(p[2] + R[2, 2] * RING_H / 2), i, bool(i in jaws or np.linalg.norm(v) > REST_SPEED)))
            else:
                loose.append(i)
        return [[(i, moving) for _z, i, moving in sorted(s)] for s in per], loose

    def score(self):
        """(sorted rings, complete spindles): a ring counts when it rests on a spindle whose rings are all of its
        variant, none damaged, none held or moving; a spindle is complete when it holds all four rings of a
        variant that way."""
        per, _loose = self.arrangement()
        rings, complete = 0, 0
        for s in per:
            if not s:
                continue
            vs = set(self.ring_variant[i] for i, _m in s)
            if len(vs) != 1 or any(self.damaged[i] or mv for i, mv in s):
                continue
            rings += len(s)
            if len(s) == RINGS_PER_VARIANT:
                complete += 1
        return rings, complete

    def episode_score(self):
        r, c = self.score()
        return 0.5 * r / self.n_rings + 0.5 * c / len(VARIANTS)

    def _post_step(self):
        d = self.data
        if self.fault is None:
            if np.max(np.abs(d.qvel[self.jv])) > INSTAB_ROBOT_QVEL:
                self.fault = "robot joint speed"
            elif any(np.linalg.norm(d.qvel[a:a + 3]) > INSTAB_RING_VEL for a in self.ring_dadr):
                self.fault = "ring speed"
        f = self.ring_spindle_forces()
        for i in range(self.n_rings):
            self.force_count[i] = self.force_count[i] + 1 if f[i] > DAMAGE_FORCE else 0
            if self.force_count[i] >= DAMAGE_STEPS and not self.damaged[i]:
                self.damaged[i] = True
        # the cell checks the fixture every step: sorted (every variant complete on its own spindle) with no ring
        # held for SORTED_STEPS steps in a row ends the episode early
        r, c = self.score()
        self.sorted_count = self.sorted_count + 1 if c == len(VARIANTS) else 0
        if self.fault is not None or self.t >= self.max_steps or self.sorted_count >= SORTED_STEPS:
            self.done = True

    def info(self):
        return {"t": self.t, "fault": self.fault, "damaged": int(sum(self.damaged)), "bumps": self.bumps_done}

    # ------------------------------------------------------------------ sensing
    def close(self):
        for r in self._renderers.values():
            try:
                r.close()
            except Exception:  # noqa: BLE001
                pass
        self._renderers = {}

    def render(self, cam, data=None):
        w, h = (OVERVIEW_W, OVERVIEW_H) if cam == "overview" else (WRIST_W, WRIST_H)
        if cam not in self._renderers:
            self.model.light_castshadow[:] = 0
            self.model.vis.quality.offsamples = 0
            self._renderers[cam] = mujoco.Renderer(self.model, h, w)
        name = "overview" if cam == "overview" else "r_eye_in_hand"
        r = self._renderers[cam]
        r.update_scene(self.data if data is None else data, name)
        return r.render().copy()

    def wrench(self):
        d = self.data
        Rft = d.site_xmat[self.ft_site].reshape(3, 3)
        zero = self.wrench_bias + self.wrench_drift * (self.t * CONTROL_DT)
        f = Rft @ d.sensordata[self.ft_force:self.ft_force + 3] + zero[:3] + self.noise_rng.normal(0, WRENCH_NOISE[0], 3)
        tq = Rft @ d.sensordata[self.ft_torque:self.ft_torque + 3] + zero[3:] + self.noise_rng.normal(0, WRENCH_NOISE[1], 3)
        return np.round(np.concatenate([f, tq]), 4)

    def observation(self):
        d = self.data
        R = d.site_xmat[self.site].reshape(3, 3)
        obs = {"t": int(self.t),
               "q": np.round(d.qpos[self.jq], 6),
               "ee_pos": np.round(d.site_xpos[self.site], 6),
               "ee_quat": np.round(quat_from_mat(R), 6),
               "ee_yaw": round(float(self.ee_yaw()), 6),
               "gripper": round(float(d.qpos[self.gq[0]] - d.qpos[self.gq[1]]), 6),
               "wrench": self.wrench()}
        for bit, cam in ((1, "overview"), (2, "wrist")):
            if self.render_enabled and (self.cams_next & bit):
                obs[cam] = self.render(cam)
            else:
                obs[cam] = np.zeros((0, 0, 3), np.uint8)
        return obs

    def perturbed_observation(self, obs, rng):
        """A probe copy for the grader's lock: the observation of a slightly different arm pose. The joint readings
        are offset by up to 0.004 rad; the hand's pose and the images the real observation carries are those of the
        offset pose; the wrench gets a small offset. The real observation's hand pose comes from the joint positions
        before the last physics step (the simulation's own kinematics), so the copy's does too (those older
        positions plus the copy's offsets): a copy whose pose were the exact kinematics of its `q` could be told
        apart."""
        dq = np.array([rng.uniform(-0.004, 0.004) for _ in range(7)])
        f = self.fk
        f.qpos[:] = self.q_stale
        f.qpos[self.jq] = self.q_stale[self.jq] + dq
        mujoco.mj_kinematics(self.model, f)
        mujoco.mj_comPos(self.model, f)
        mujoco.mj_camlight(self.model, f)
        R = f.site_xmat[self.site].reshape(3, 3)
        out = dict(obs)
        if self.render_enabled:
            for cam in CAMERA_NAMES:
                if np.asarray(obs[cam]).shape[0] > 0:
                    out[cam] = self.render(cam, f)
        out["q"] = np.round(np.asarray(obs["q"], float) + dq, 6)
        out["ee_pos"] = np.round(f.site_xpos[self.site], 6)
        out["ee_quat"] = np.round(quat_from_mat(R), 6)
        out["ee_yaw"] = round(float(yaw_of_mat(R @ self.R_down.T)), 6)
        w = np.asarray(obs["wrench"], float).copy()
        w[:3] += np.array([rng.uniform(-0.3, 0.3) for _ in range(3)])
        w[3:] += np.array([rng.uniform(-0.02, 0.02) for _ in range(3)])
        out["wrench"] = np.round(w, 4)
        return out

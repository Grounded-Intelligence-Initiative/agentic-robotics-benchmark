"""Bayonet nut kit cell: the data sheet's numbers, the camera calibration and a few geometry helpers.

The companion of `datasheet.md`: every constant of the kit's drawing, the cell's layout, the robot's controller
interface, the sensors and the cameras, with the helpers that follow from them (camera matrices, the cells a groove
occupies, the image codec of the wire). Plain Python and NumPy; it contains no simulator. If your controller uses
it, copy it into your submission folder (nothing else of your home is available when the controller is graded).

Frames and units: world frame, metres, radians, seconds; the table top is the plane z = TABLE_Z and spans x, y in
[-TABLE_HALF, TABLE_HALF]; the Panda base stands at ROBOT_BASE facing +x. Quaternions are (w, x, y, z).
"""

import math

import numpy as np

# ----------------------------------------------------------------------------------------------
# Control and takt
# ----------------------------------------------------------------------------------------------
CONTROL_DT = 0.1                 # one action every 0.1 s (10 Hz); the arm's torque loop runs at 500 Hz
NUTS = 3                         # nuts in a kit (one per peg)
CYCLE_STEPS = 400                # takt time per nut: 40 s simulated
MAX_STEPS = NUTS * CYCLE_STEPS   # control steps per episode (one kit, 120 s simulated)
MAX_DPOS = 0.02
MAX_DROT = 0.15
TABLE_Z = 0.80
TABLE_HALF = 0.40
ROBOT_BASE = np.array([-0.56, 0.0, TABLE_Z])
WORKSPACE_LO = np.array([-0.45, -0.45, TABLE_Z + 0.002])
WORKSPACE_HI = np.array([0.45, 0.45, TABLE_Z + 0.55])

# ----------------------------------------------------------------------------------------------
# Kit drawing (m)
# ----------------------------------------------------------------------------------------------
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
NUT_COM = (0.015, 0.0, 0.018)    # the nut's centre of mass in its own frame (towards the handle, above the bottom face)
NUT_MASS_RANGE = (0.05, 0.15)    # kg
NUT_FRICTION_RANGE = (0.6, 1.0)  # sliding friction coefficient of the nut's surfaces
BAND_HALF_H = 0.0035             # a peg band: a dark ring 7 mm tall around the shaft
BAND_FIRST = 0.010               # the first band's centre below the shaft top
BAND_PITCH = 0.013               # the next bands' centres, each this much lower

# ----------------------------------------------------------------------------------------------
# Layout tolerances (uniform per episode)
# ----------------------------------------------------------------------------------------------
PEG_REGION = ((-0.02, 0.15), (-0.21, 0.21))
PEG_MIN_SPACING = 0.13
NUT_REGION = ((-0.24, -0.06), (-0.24, 0.24))
NUT_PEG_MIN_DIST = 0.13
NUT_MIN_SPACING = 0.12       # between nut centres
HANDLE_CLEAR = 0.07          # handle centre from any other nut centre
NUT_YAW_CENTER = math.radians(135.0)   # nut yaw = azimuth of the nut from the robot base + this +- spread
NUT_YAW_SPREAD = math.radians(60.0)

# ----------------------------------------------------------------------------------------------
# Safety stops
# ----------------------------------------------------------------------------------------------
ENGAGE_XY = 0.005            # a nut is on its peg: hole axis within this distance of that peg's shaft axis
GROOVE_TOP = 0.0395          # nut bottom above the collar when it rests on the pin (pin top)
FORCE_ANGLE_TOL = 5.0        # deg: pin centre this far inside a solid wall cell (sideways) breaks the nut
FORCE_Z_TOL = 0.0015         # m: ... or this far (up or down)
FORCE_STEPS = 3              # consecutive control steps
INSTAB_ROBOT_QVEL = 8.0      # rad/s: an episode with faster robot joints ends as a fault
INSTAB_NUT_VEL = 5.0         # m/s

# ----------------------------------------------------------------------------------------------
# Robot, gripper, sensors
# ----------------------------------------------------------------------------------------------
GRIP_KP = 3000.0             # finger position gain (N/m); the finger force is capped at 20 N
RGB_NOISE = 2.0              # 8-bit sigma
WRENCH_NOISE = 0.3           # N (torque: 0.03 N m)
WRENCH_BIAS = 0.5            # N per axis, fixed per episode (torque: 0.05 N m)
HOME_Q = np.array([0.0, -0.35, 0.0, -2.35, 0.0, 2.0, math.pi / 4])
NULLSPACE_Q = np.array([0.0, -0.3, 0.0, -2.2, 0.0, 1.9, math.pi / 4])
OSC_KP_POS = 400.0           # 1/s^2: task-space position gain (natural frequency 20 rad/s, critically damped)
OSC_KP_ROT = 200.0           # 1/s^2: task-space orientation gain (14.1 rad/s, critically damped)

# ----------------------------------------------------------------------------------------------
# Cameras
# ----------------------------------------------------------------------------------------------
CAMERAS = {
    # name: (position, look-at target, fovy deg, width, height); fixed cameras never move
    "left": ((-0.30, 0.55, 1.45), (0.05, 0.0, 0.80), 50.0, 320, 240),
    "right": ((-0.30, -0.55, 1.45), (0.05, 0.0, 0.80), 50.0, 320, 240),
}
WRIST = {"fovy": 75.0, "width": 256, "height": 192}
CAMERA_NAMES = ("left", "right", "wrist")
OBS_KEYS = ("t", "order", "q", "dq", "ee_pos", "ee_quat", "gripper", "wrench", "T_world_wrist",
            "left", "right", "wrist")

NUT_COLOURS = {"blue": [0.16, 0.36, 0.78, 1], "green": [0.20, 0.62, 0.30, 1], "red": [0.78, 0.18, 0.16, 1],
               "yellow": [0.86, 0.74, 0.16, 1], "orange": [0.85, 0.45, 0.12, 1], "purple": [0.55, 0.25, 0.70, 1],
               "teal": [0.15, 0.65, 0.68, 1], "pink": [0.80, 0.35, 0.55, 1]}
PEG_COLOURS = {"collar": [0.30, 0.31, 0.34, 1], "shaft_and_pin": [0.62, 0.64, 0.67, 1], "band": [0.07, 0.07, 0.09, 1]}


# ----------------------------------------------------------------------------------------------
# Helpers
# ----------------------------------------------------------------------------------------------
def look_at(pos, target, up=(0, 0, 1)):
    """Rotation whose columns are the camera axes x right, y up, z backwards (the renderer's convention)."""
    pos, target = np.asarray(pos, float), np.asarray(target, float)
    f = target - pos
    f = f / np.linalg.norm(f)
    z = -f
    x = np.cross(np.array(up, dtype=float), z)
    x = x / np.linalg.norm(x)
    y = np.cross(z, x)
    return np.stack([x, y, z], axis=1)


def camera_intrinsics(fovy, width, height):
    """Pinhole K for a vertical field of view `fovy` (degrees); the principal point is at (W/2, H/2), so the
    centre of pixel (u, v) is at (u + 0.5, v + 0.5)."""
    f = (height / 2.0) / math.tan(math.radians(fovy) / 2.0)
    return np.array([[f, 0, width / 2.0], [0, f, height / 2.0], [0, 0, 1.0]])


def fixed_camera_matrices(name):
    """(K, T_world_cam) of a fixed camera ("left" or "right"), OpenCV convention (x right, y down, z forward)."""
    pos, tgt, fovy, w, h = CAMERAS[name]
    R = look_at(pos, tgt) @ np.diag([1.0, -1.0, -1.0])
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = pos
    return camera_intrinsics(fovy, w, h), T


def wrist_camera_intrinsics():
    """K of the wrist camera; its pose is every observation's `T_world_wrist` (OpenCV convention)."""
    return camera_intrinsics(WRIST["fovy"], WRIST["width"], WRIST["height"])


def decode_image(d):
    """A wire image {"dtype": "uint8", "shape": [H, W, 3], "b64": ...} as an array (H, W, 3), RGB, row-major."""
    import base64
    return np.frombuffer(base64.b64decode(d["b64"]), np.uint8).reshape(d["shape"])


def wrap_angle(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def quat_to_mat(q):
    """Rotation matrix of a quaternion (w, x, y, z)."""
    w, x, y, z = [float(v) for v in q]
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def groove_cells(groove):
    """The (sector, layer) cells cut out of the ring's inner part, for a groove {"entrance": 0..23, "turns":
    [k1, k2], "dead_ends": [n1, n2], "false_entrances": [sectors]} of the family (data sheet, section 3).
    Channels are 2 sectors wide; sector s spans [15 s, 15 (s + 1)] degrees counter-clockwise from the handle
    (nut +x); layer 0 is the bottom one."""
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


def band_heights(bands):
    """Heights above the table of the centres of a peg's `bands` dark bands (1, 2 or 3)."""
    top = COLLAR_H + PEG_H
    return [top - BAND_FIRST - BAND_PITCH * k for k in range(bands)]

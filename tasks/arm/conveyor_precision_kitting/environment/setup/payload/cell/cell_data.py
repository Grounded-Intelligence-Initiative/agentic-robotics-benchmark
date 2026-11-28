"""The data sheet's numbers (datasheet.md), as Python. Plain Python and NumPy; no simulator.
Nominal values; where the hand-over survey found a quantity to vary from shift to shift, its range (a shift's own
value is never reported). Copy it into your submission if your controller uses it."""
import math

import numpy as np

# ---- frames
TABLE_Z = 0.80
ROBOT_BASE = (-0.56, 0.0, 0.80)
WORKSPACE_LO = (-0.38, -0.50, 0.804)
WORKSPACE_HI = (0.30, 0.42, 1.20)

# ---- robot and controller
CONTROL_DT = 0.1          # s per control step ...
CAM_COST_S = 0.05         # ... plus this per image taken
LATENCY_STEPS_RANGE = (2, 3)   # the action for observation t is executed during step t+L -> t+L+1; L fixed per shift
MAX_DPOS = 0.03
MAX_DYAW = 0.15
KP_POS_NOMINAL, KP_ROT_NOMINAL = 400.0, 200.0   # s^-2, nominally critically damped
ARM_STIFFNESS_TOL = 0.25       # relative, per shift
ARM_DAMPING_RATIO = (0.75, 1.10)
GRAVITY_COMP_TOL = 0.03        # relative, per shift
HOME_YAW = math.pi / 2
GRIPPER_OPEN = 0.08       # m between the pads at grip = -1
FINGER_FORCE_MAX = 20.0   # N per finger
PAD_BELOW_GRIP = 0.0044   # the pads reach this far below the grip point
FINGER_BELOW_GRIP = 0.0093

# ---- belt
BELT_X, BELT_Z, BELT_HALF_W = 0.10, 0.84, 0.080
RAIL_INNER = 0.088
RELEASE_Y = 0.62
PICK_ZONE = (0.40, -0.40)          # y: upstream end, downstream end (lost beyond)
BELT_SPEED = (0.030, 0.055)        # m/s at the start of a shift
BELT_WOBBLE = 0.08                 # relative, periodic (period 40-80 s)
BELT_DRIFT = 0.10                  # relative, either way, reached at BELT_DRIFT_S and kept
BELT_DRIFT_S = 400.0
FLIP_PROBABILITY = 0.5
RELEASES = 8
RELEASE_FIRST = (20.0, 26.0)
RELEASE_GAP = (44.0, 54.0)
SHIFT_GRACE = 60.0

# ---- parts: (length, width, key, key dims, nominal linear RGB, mass)
H_PART = 0.020
VARIANTS = {
    "A1": (0.044, 0.024, "corner", (0.010, 0.008), (0.20, 0.36, 0.78), 0.045),
    "A2": (0.052, 0.024, "corner", (0.010, 0.008), (0.22, 0.37, 0.76), 0.052),
    "B1": (0.040, 0.028, "slot", (0.010, 0.008), (0.24, 0.58, 0.30), 0.044),
    "B2": (0.040, 0.028, "corner", (0.010, 0.008), (0.26, 0.60, 0.28), 0.046),
    "C1": (0.038, 0.030, "corner", (0.009, 0.008), (0.86, 0.47, 0.15), 0.048),
    "C2": (0.046, 0.026, "tab", (0.008, 0.006), (0.84, 0.49, 0.17), 0.045),
}
COLOUR_TOL = 0.02
LAMP_BRIGHTNESS_TOL, LAMP_COLOUR_TOL = 0.30, 0.06   # relative, per shift
FRICTION_RANGE = (0.25, 0.55)      # part on tray, per shift (same for all parts of a shift)
MARK_DIAMETER, MARK_INSET = 0.007, 0.008     # datum mark on the top face, centred this far in from the -x end


def outline(name):
    """The part's outline in the part frame (m), counter-clockwise, as seen from above with the part upright."""
    L, W, key, (a, b), _rgb, _m = VARIANTS[name]
    x0, x1, y0, y1 = -L / 2, L / 2, -W / 2, W / 2
    if key == "corner":
        return [(x0, y0), (x1, y0), (x1, y1 - b), (x1 - a, y1 - b), (x1 - a, y1), (x0, y1)]
    if key == "slot":
        return [(x0, y0), (x1, y0), (x1, -a / 2), (x1 - b, -a / 2), (x1 - b, a / 2), (x1, a / 2), (x1, y1), (x0, y1)]
    return [(x0, y0), (x1 - a, y0), (x1 - a, y0 + b), (x1, y0 + b), (x1, y1 - b), (x1 - a, y1 - b), (x1 - a, y1),
            (x0, y1)]


# ---- kit tray
KIT_NOMINAL = (-0.155, -0.27, math.pi / 2)
KIT_JITTER = (0.012, 0.012, math.radians(5.0))
POCKET_TOL = 0.0012                 # each pocket on a tray: within this per axis of its drawing position
KIT_PLATE_HALF = (0.125, 0.082)
KIT_FLOOR_Z = 0.81
POCKETS = (                         # (variant, x, y, yaw) in the kit frame
    ("A1", -0.075, 0.040, 0.0),
    ("B2", 0.000, 0.040, 0.0),
    ("C1", 0.075, 0.040, math.pi / 2),
    ("A2", -0.075, -0.040, math.pi),
    ("B1", 0.000, -0.040, 0.0),
    ("C2", 0.075, -0.040, math.pi),
)
CLEARANCE = 0.0006                  # per side, part to pocket walls
KEY_CLEARANCE = 0.0009              # per side, key feature to key boss
KEY_HEIGHT = 0.0025
POCKET_DEPTH = 0.005
CHAMFER = 0.0010
FIDUCIALS = ((-0.110, 0.068), (0.110, 0.068), (-0.110, -0.068))
FIDUCIAL_DIAMETER = 0.012
DAMAGE_FORCE, DAMAGE_STEPS = 12.0, 2


def pocket_world(kit_pose, k):
    """(x, y, yaw) of pocket k in the world for a kit at kit_pose = (x, y, yaw)."""
    kx, ky, kyaw = kit_pose
    _v, px, py, pyaw = POCKETS[k]
    c, s = math.cos(kyaw), math.sin(kyaw)
    return kx + c * px - s * py, ky + s * px + c * py, kyaw + pyaw


# ---- turnover station
STATION = (-0.12, 0.28)
NEST_IN_CENTRE = (-0.12, 0.326)
NEST_OUT_CENTRE = (-0.12, 0.234)
NEST_HALF = (0.032, 0.026)
NEST_DEPTH = 0.004
NEST_FLOOR_Z = 0.81
TURN_AXIS = (0.28, 0.82)            # the turnover axis is parallel to x through (y, z) = this
GUARD_R, GUARD_Z = 0.12, 0.10

# ---- cameras: position, rotation (columns = camera x, y, z axes in the world; the camera looks along -z), fovy, (W, H)
OVERVIEW = {"pos": (0.02, 0.0, 1.75), "look_at": (-0.02, 0.0, 0.80), "fovy": 58.0, "size": (160, 120)}
WRIST = {"pos_in_grip_frame": (0.0, 0.05, -0.097), "R_in_grip_frame": ((-1, 0, 0), (0, 1, 0), (0, 0, -1)),
         "fovy": 75.0, "size": (160, 120)}
INSPECT = {"pos": (-0.10, 0.04, 0.812), "R": ((0, 1, 0), (1, 0, 0), (0, 0, -1)), "fovy": 40.0, "size": (96, 96)}
CAMERA_TOL = {"overview": (0.010, 1.5), "wrist": (0.0010, 0.3), "inspect": (0.0008, 0.3)}   # (m per axis, deg)


def grip_frame(ee_yaw):
    """Rotation of the grip-point frame (x = closing direction, z = straight down) for hand yaw ee_yaw."""
    c, s = math.cos(ee_yaw), math.sin(ee_yaw)
    Rz = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    return Rz @ np.diag([1.0, -1.0, -1.0])


def wrist_camera(ee_pos, ee_yaw):
    """(position, R) of the wrist camera on its drawing for the hand at ee_pos, ee_yaw."""
    Rg = grip_frame(ee_yaw)
    pos = np.asarray(ee_pos, float) + Rg @ np.array(WRIST["pos_in_grip_frame"], float)
    return pos, Rg @ np.array(WRIST["R_in_grip_frame"], float)


def look_at(pos, target):
    pos, target = np.asarray(pos, float), np.asarray(target, float)
    z = pos - target
    z = z / np.linalg.norm(z)
    x = np.cross([0.0, 0.0, 1.0], z)
    x = x / np.linalg.norm(x)
    return np.stack([x, np.cross(z, x), z], axis=1)


def pixel_ray(R, fovy, size, u, v):
    """World direction of the ray through pixel (u, v) (column, row; pixel centres at +0.5)."""
    W, H = size
    f = (H / 2.0) / math.tan(math.radians(fovy) / 2)
    return R @ np.array([(u + 0.5 - W / 2) / f, -(v + 0.5 - H / 2) / f, -1.0])

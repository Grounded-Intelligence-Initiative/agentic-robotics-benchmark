"""The camera holder, the two scopes and the view rules: the public model.

This file is the reference implementation of the holder manual: every number and formula in it is stated there, and
the benchmark's own simulator computes with this same file. It holds nothing about a procedure or the scene.

Frames. The patient frame W has its origin at the camera trocar's remote centre of motion (RCM), x to the surgeon's
right, y away from the surgeon (cranial), z up (anterior). Lengths are metres, angles radians. The camera frame C is
the usual graphics one: x right in the image, y up in the image, the camera looks along -z.

Joints q = [pan, tilt, insertion, roll]:

    R_WS = Ry(-pan) . Rx(tilt)          the scope's axis is a = R_WS . (0, 0, -1): straight down at pan = tilt = 0,
                                        towards +x for pan > 0, towards +y for tilt > 0
    p    = insertion * a                the lens, `insertion` metres past the RCM along the scope
    R_WC = R_WS . Rz(roll) . Rx(-d) . Rz(h)
                                        d = the scope's direction of view (nominally 0 or 30 degrees; a unit's actual
                                        one is within VIEW_TOL_DEG of it, version 0.4): a 30 degree scope looks
                                        30 degrees below its own axis (towards -y of the rolled scope); h = the angle at
                                        which the camera head is clamped on the scope's eyepiece (set by hand for each
                                        procedure, not reported: it turns the picture about the view direction)

Pure numpy.
"""

import math

import numpy as np

# ---------------------------------------------------------------------------------------------- the published numbers
IMAGE_W, IMAGE_H = 640, 480
DT = 0.1                      # s of simulated time per control step
SUBSTEPS = 5                  # the holder's servo ticks inside one control step

JOINTS = ("pan", "tilt", "insertion", "roll")
Q_MIN = np.array([-1.05, -0.35, 0.010, -2.80])
Q_MAX = np.array([1.05, 1.25, 0.200, 2.80])
RATE_MAX = np.array([0.50, 0.50, 0.050, 0.80])      # rad/s, rad/s, m/s, rad/s: commands are clipped to these
ACC_MAX = np.array([2.5, 2.5, 0.30, 4.0])           # the servo's acceleration limits
SERVO_TAU = 0.12              # s, first-order lag of the joint rate towards the commanded rate
Q_HOME = np.array([0.0, 0.55, 0.020, 0.0])          # where every procedure starts: withdrawn into the cannula

SCOPES = {
    "0": {"view_deg": 0.0, "fx": 480.0, "fy": 480.0, "cx": 320.0, "cy": 240.0},
    "30": {"view_deg": 30.0, "fx": 456.0, "fy": 456.0, "cx": 320.0, "cy": 240.0},
}
SHAFT_DIAMETER = 0.005        # m, the zoom band's reference shaft (rule 2 is a working distance stated for a 5 mm shaft)
SHAFT_DIAMETERS = (0.005, 0.010)   # m, the instruments' shafts: 5 or 10 mm, the whole instrument built to its size

# what differs from one procedure to the next in the camera chain and is not reported (holder_manual.md, section 4)
HEAD_ANGLE_MAX_DEG = 40.0     # the camera head is clamped on the eyepiece at any angle within +-this
LATENCY_STEPS = (1, 2, 3)     # the video chain's latency, in control steps

# the view rules (instruction.md, holder_manual.md section 6)
SAFE_ZONE = 0.30              # a tip is inside when |u - cx| <= 0.30 W and |v - cy| <= 0.30 H
ZOOM_BAND_PX = (22.0, 44.0)   # apparent shaft diameter at the tip, fx * 5 mm / depth
HORIZON_REF_DEG = 45.0        # the display's up reference r = (0, cos 45, sin 45) in W
HORIZON_LIMIT_DEG = 10.0
FINE_WINDOW_S = 1.0           # fine work: every active tip stayed within FINE_RADIUS of its mean over this window
FINE_RADIUS = 0.009
IMAGE_SPEED_LIMIT = 32.0      # px/s the camera may move the image of the working point during fine work

# the lens
LENS_CLEARANCE = 0.004        # m from the tissue at which the lens touches it
CLEAN_INSERTION = 0.025       # the lens is wiped while insertion <= this ...
CLEAN_TIME_S = 10.0           # ... for this long, continuously

# the series (version 0.4): every unit of the holder deviates from the nominal values in this file within these
# tolerances, fixed for the unit; this file computes the nominal unit (holder_manual.md, section 2)
ZERO_TOL = (0.015, 0.015, 0.006, 0.07)   # rad, rad, m, rad: a joint's reading = its true value + a fixed offset within +-this
GAIN_RANGE = (0.8, 1.2)                  # each joint moves at its (clipped) command times a fixed gain within this range
VIEW_TOL_DEG = 2.5                       # a scope's actual direction of view: its nominal one +- this

# the surgeon takes the camera back (the holder's control of the procedure ends; every step after it is not usable)
LENS_INJURY_SPEED = 0.015     # m/s: the lens reaching the tissue faster than this injures it
LENS_CONTACTS_TOLERATED = 1   # a contact after this many in one procedure ends it
CRITICAL_MIN_USABLE = 0.25    # a critical action seen with a usable view on fewer than this share of its steps ends it


def rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def scope_rotation(q):
    """R_WS: the scope's frame before roll."""
    return ry(-float(q[0])) @ rx(float(q[1]))


def camera_pose(q, scope, head=0.0, view_deg=None):
    """(R_WC, p): the camera's axes as columns in W, and the lens position, for the true joints q. `head`: the camera
    head's angle on the eyepiece, rad; `view_deg`: the scope's actual direction of view (its nominal one if None)."""
    r_ws = scope_rotation(q)
    p = float(q[2]) * (r_ws @ np.array([0.0, 0.0, -1.0]))
    d = SCOPES[scope]["view_deg"] if view_deg is None else float(view_deg)
    r_wc = r_ws @ rz(float(q[3])) @ rx(-math.radians(d)) @ rz(float(head))
    return r_wc, p


def lens_position(q):
    return float(q[2]) * (scope_rotation(q) @ np.array([0.0, 0.0, -1.0]))


def to_camera(points, pose):
    """World points (N, 3) in the camera frame (x right, y up, z towards the viewer)."""
    r_wc, p = pose
    return (np.atleast_2d(np.asarray(points, dtype=float)) - p) @ r_wc


def project(points, pose, scope):
    """(u, v, depth) per world point; depth is the distance along the view direction (<= 0: behind the lens)."""
    k = SCOPES[scope]
    pc = to_camera(points, pose)
    depth = -pc[:, 2]
    safe = np.where(np.abs(depth) < 1e-9, 1e-9, depth)
    u = k["cx"] + k["fx"] * pc[:, 0] / safe
    v = k["cy"] - k["fy"] * pc[:, 1] / safe
    return np.stack([u, v, depth], axis=1)


def horizon_reference():
    a = math.radians(HORIZON_REF_DEG)
    return np.array([0.0, math.cos(a), math.sin(a)])


def horizon_error(q, scope, head=0.0, view_deg=None):
    """Signed angle (rad) between the image's up direction and the projection of the display's up reference."""
    r_wc, _ = camera_pose(q, scope, head, view_deg)
    r = horizon_reference()
    return math.atan2(float(r @ r_wc[:, 0]), float(r @ r_wc[:, 1]))


def level_roll(pan, tilt, scope="0", head=0.0, view_deg=None):
    """The roll at which the horizon error is zero for this pan and tilt. With the camera head at 0 it is the same
    for both scopes (closed form); otherwise it is solved for (Newton's method from the 0 degree scope's answer)."""
    w = scope_rotation([pan, tilt, 0.0, 0.0]).T @ horizon_reference()
    roll = math.atan2(-w[0], w[1]) - float(head)
    d = SCOPES[scope]["view_deg"] if view_deg is None else float(view_deg)
    if float(head) == 0.0 or d == 0.0:
        return math.atan2(math.sin(roll), math.cos(roll))
    for _ in range(20):
        e = horizon_error([pan, tilt, 0.0, roll], scope, head, view_deg)
        if abs(e) < 1e-12:
            break
        de = (horizon_error([pan, tilt, 0.0, roll + 1e-6], scope, head, view_deg)
              - horizon_error([pan, tilt, 0.0, roll - 1e-6], scope, head, view_deg)) / 2e-6
        roll -= e / (de if abs(de) > 1e-6 else 1.0)
    return math.atan2(math.sin(roll), math.cos(roll))


def apparent_diameter(depth, scope):
    return SCOPES[scope]["fx"] * SHAFT_DIAMETER / max(float(depth), 1e-6)


def in_safe_zone(u, v, scope):
    k = SCOPES[scope]
    return abs(u - k["cx"]) <= SAFE_ZONE * IMAGE_W and abs(v - k["cy"]) <= SAFE_ZONE * IMAGE_H


class Holder(object):
    """The holder's joints under rate commands: clipping, the servo's lag and acceleration limit, the joint stops."""

    def __init__(self, q=None):
        self.q = np.array(Q_HOME if q is None else q, dtype=float)
        self.v = np.zeros(4)

    def tick(self, command, h):
        """One servo tick of length h towards the (already clipped) rate command; returns the joints it would reach."""
        dv = (command - self.v) * (h / SERVO_TAU)
        lim = ACC_MAX * h
        v = self.v + np.clip(dv, -lim, lim)
        q = self.q + v * h
        lo, hi = q < Q_MIN, q > Q_MAX
        q = np.clip(q, Q_MIN, Q_MAX)
        v = np.where(lo | hi, 0.0, v)
        return q, v


def clip_command(action):
    """A reply's action as four finite rates inside the limits, or ValueError."""
    a = np.asarray(action, dtype=float).reshape(-1)
    if a.shape != (4,) or not np.all(np.isfinite(a)):
        raise ValueError("an action is four finite numbers [pan_rate, tilt_rate, insertion_rate, roll_rate]")
    return np.clip(a, -RATE_MAX, RATE_MAX)

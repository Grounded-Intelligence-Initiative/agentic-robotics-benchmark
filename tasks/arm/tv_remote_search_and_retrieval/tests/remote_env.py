"""The living room of tv_remote_search_and_retrieval: a Franka Panda on a floor rail in front of a
two-seat settee and a chest of three drawers, with the loose things of a living room (a sun hat,
a storage box, a clothes hanger, a phone, a glasses case, a pouch), and a TV
remote that lies somewhere in it.

World frame: metres, radians, quaternions (w, x, y, z). x points from the robot's rail to the sofa,
y to the robot's left, z up; the floor is z = 0. The rail runs along y at x = 0.

A scene is a `Scene` = `make_scene(seed, remote)`: where the settee and the chest stand
(`sofa_pose`, `cabinet_pose`), how the hanger hangs, which drawer the hat is in and how it lies
there, where the box stands on the chest, the look of the room (fabric, wood, floor, wall and
lamp), all drawn from the seed, and where the remote is, which the caller says. A phone, a
glasses case and a pouch are in the model too; `make_scene` puts them where the caller says, or
out of the room. This file holds no rule for where the remote is in a graded
episode: that is the grader's, and you can put the remote wherever you want to practise.

The furniture's shape is the same in every scene; where it stands is not. Every constant below
that places something (SOFA_X0, PAD_Y, CAB_CY, KNOB_Y, PEG_X, ...) describes the **nominal
layout**. In a scene the settee, with its pads, is that layout turned by a small angle about the
vertical through `SOFA_PIVOT` and shifted; the chest, with its drawers, the hanger, the hat and the box,
likewise about `CAB_PIVOT` (`LAYOUT_RANGE`, `place`). An observation does not say by how much: the
cameras show it.

This file is the simulator and the judge of an episode (`Env.progress`, `Env.score`); the grader
imports its own copy, and the copy in the agent's home is for development.
"""

import base64
import hashlib
import math
import os
import struct
import xml.etree.ElementTree as ET
from dataclasses import asdict, dataclass

import numpy as np

os.environ.setdefault("MUJOCO_GL", "egl")

import mujoco  # noqa: E402

__all__ = [
    "Scene", "make_scene", "Env", "CAMERAS", "ACTION_SIZE", "CONTROL_DT",
    "MAX_STEPS", "camera_intrinsics", "decode_image", "build_xml", "place",
]



# ----------------------------------------------------------------------------------------------
# Timing
# ----------------------------------------------------------------------------------------------
SIM_DT = 0.002
SUBSTEPS = 25
CONTROL_DT = SIM_DT * SUBSTEPS          # 0.05 s: the controller runs at 20 Hz
MAX_STEPS = 3600                        # 180 s of simulated time for an episode

# ----------------------------------------------------------------------------------------------
# Robot: Panda (robosuite 1.5.2 asset files) on a pedestal that rides a rail along y
# ----------------------------------------------------------------------------------------------
BASE_Z = 0.30                           # height of the Panda's base plane above the floor
RAIL_RANGE = (-1.75, 1.10)              # y of the pedestal
RAIL_HOME = 0.15
ARM_HOME = (0.0, -0.9, 0.0, -2.3, 0.0, 1.5, 0.785)
ARM_LIMITS = (
    (-2.8973, 2.8973), (-1.7628, 1.7628), (-2.8973, 2.8973), (-3.0718, -0.0698),
    (-2.8973, 2.8973), (-0.0175, 3.7525), (-2.8973, 2.8973),
)
ARM_KP = (4500.0, 4500.0, 3500.0, 3500.0, 2000.0, 2000.0, 2000.0)
ARM_KV = (450.0, 450.0, 350.0, 350.0, 200.0, 200.0, 200.0)
ARM_FORCE = (87.0, 87.0, 87.0, 87.0, 12.0, 12.0, 12.0)
# The controller's targets are followed at no more than these speeds (rail m/s, joints rad/s).
MAX_SPEED = (0.5, 1.4, 1.4, 1.4, 1.4, 1.8, 1.8, 1.8)
GRIP_RANGE = (0.0, 0.08)                # opening between the two fingers
GRIP_SPEED = 0.10                       # m/s of opening
FINGER_FORCE = 35.0                     # N per finger
ACTION_SIZE = 10                        # [rail, q1..q7, grip, cams]

# ----------------------------------------------------------------------------------------------
# Sofa: a two-seat settee (Poly Haven "Sofa 01", CC0) with two loose seat pads. The numbers
# are the collision shapes the model is drawn around.
# ----------------------------------------------------------------------------------------------
SOFA_X0 = 0.36                          # x of the sofa's front face
SOFA_Y = (-0.32, 0.82)                  # outer faces of the two armrests
ARM_W = 0.065                           # armrest width
UNDER_GAP = 0.06                        # floor to the sofa's underside
DECK_Z = 0.25                           # top of the flat seat deck the pads lie on
SEAT_DEPTH = 0.50                       # deck, front face to backrest
PAD_SIZE = (0.46, 0.49, 0.065)          # depth (x), width (y), thickness
PAD_OVERHANG = 0.03                     # a pad's front edge stands out over the sofa's front face
PAD_MASS = 0.55
BACK_TILT = math.radians(7.0)
BACK_H = 0.55
ARMREST_TOP = 0.53
SEAT_MID_Y = 0.5 * (SOFA_Y[0] + SOFA_Y[1])
PAD_Y = {"left": 0.5 * (SEAT_MID_Y + SOFA_Y[1] - ARM_W), "right": 0.5 * (SOFA_Y[0] + ARM_W + SEAT_MID_Y)}

# ----------------------------------------------------------------------------------------------
# Side cabinet: a chest of three drawers (Poly Haven "Gothic Commode 01", CC0).
# Cabinet frame: x from its front, y from its middle, z from the floor.
# ----------------------------------------------------------------------------------------------
CAB_X0 = 0.38                           # x of the cabinet frame's origin (its front)
CAB_CY = -0.85
CAB_HALF_W = 0.285                      # outer faces of the corner posts
CAB_PANEL_W = 0.272                     # outer faces of the side panels between the posts
CAB_Y = (CAB_CY - CAB_HALF_W, CAB_CY + CAB_HALF_W)
CAB_DEPTH = 0.457
CAB_H = 0.727
CAB_UNDER = 0.108                       # floor to the underside of the carcass
CAB_LEG_X = (0.049, 0.410)              # the four legs: front and back pair, in the cabinet frame
CAB_LEG_Y = 0.2685                      # ... at -y and +y
CAB_LEG_HALF = (0.046, 0.0295)          # a leg's half sizes (its foot's), in x and y
DRAWERS = ("upper", "mid", "lower")
DRAWER_FRONT_X = (0.045, 0.069)         # a closed drawer's front panel, in the cabinet frame
DRAWER_TRAVEL = 0.36
DRAWER_INNER = (0.36, 0.47)             # usable floor of a drawer: depth (x), width (y)
DRAWER_WALL_H = 0.10                    # height of a drawer's side walls above its floor
DRAWER_FLOOR_Z = {"upper": 0.539, "mid": 0.348, "lower": 0.163}       # top of each drawer's floor
DRAWER_FRONT_Z = {"upper": (0.515, 0.669), "mid": (0.323, 0.477), "lower": (0.137, 0.291)}
# Each drawer has two round knobs: their centres in the cabinet frame (x, |y|, z).
KNOB_X = 0.020
KNOB_Y = {"upper": 0.161, "mid": 0.129, "lower": 0.129}
KNOB_Z = {"upper": 0.590, "mid": 0.400, "lower": 0.215}
KNOB_HALF = 0.011                       # a knob's head, as a cube

# ----------------------------------------------------------------------------------------------
# Where the furniture stands in an episode: the nominal layout turned about a pivot and shifted
# ----------------------------------------------------------------------------------------------
SOFA_PIVOT = (SOFA_X0, SEAT_MID_Y)      # the middle of the settee's front face
CAB_PIVOT = (CAB_X0, CAB_CY)            # the middle of the chest's front
# (shift along x, shift along y, turn about z) are drawn uniformly from these ranges, for the
# settee and for the chest independently.
LAYOUT_RANGE = ((-0.01, 0.05), (-0.05, 0.05), (-0.09, 0.09))


def place(pivot, pose, xy, yaw=0.0):
    """A point (and a direction) of the nominal layout, as it is in an episode whose piece of
    furniture stands at `pose` = (dx, dy, turn) about `pivot`. Returns ((x, y), yaw)."""
    c, s = math.cos(pose[2]), math.sin(pose[2])
    rx, ry = xy[0] - pivot[0], xy[1] - pivot[1]
    return (pivot[0] + pose[0] + c * rx - s * ry, pivot[1] + pose[1] + s * rx + c * ry), yaw + pose[2]


# ----------------------------------------------------------------------------------------------
# Clothes hanger on a peg on the cabinet's outer side (the face y = CAB_Y[0])
# ----------------------------------------------------------------------------------------------
PEG_X = CAB_X0 + 0.18
PEG_Z = 0.56
PEG_LEN = 0.07
PEG_R = 0.006
HANGER_BOARD = (0.22, 0.007, 0.016)     # half sizes: along the hanger, across its plane, in-plane height
HANGER_MASS = 0.25
HOOK_R = 0.022
HOOK_WIRE = 0.0035
HOOK_NECK_Z = 0.055
HANG_OFFSET = 0.008                     # hanger plane in front of the posts it rests against

# ----------------------------------------------------------------------------------------------
# Sun hat in one of the chest's drawers (Poly Haven "Fishermans Hat", CC0). Its crown is hollow:
# its collision shapes are a convex decomposition of the drawn mesh. Hat frame: the crown's axis
# is z, the hat rests on the plane z = 0; its brim is wavy: it touches the plane on its +y side,
# is about 1.7 cm above it at its +-x ends and 2.8 cm at its -y side.
# ----------------------------------------------------------------------------------------------
HAT_PARTS = 48                          # convex pieces of the collision shape (room/hat_col_NN.obj)
HAT_SCALE = 0.68                        # of the mesh files
HAT_BRIM = (0.139, 0.150)               # the brim's reach along the hat's x and y
HAT_CROWN_R = 0.070                     # inner radius of the crown
HAT_H = 0.105
HAT_MASS = 0.09

# ----------------------------------------------------------------------------------------------
# Storage box on the chest's top: an open wooden box. Box frame: x and y along its inner floor's
# short and long side, z = 0 the plane it stands on.
# ----------------------------------------------------------------------------------------------
BOX_INNER = (0.30, 0.30)                # inner floor: x, y
BOX_WALL_H = 0.06                       # the walls' height above the inner floor
BOX_WALL = 0.006                        # thickness of the walls and of the floor
BOX_MASS = 0.60
BOX_X = 0.172                           # its middle behind the chest's front, give or take 1.5 cm

# ----------------------------------------------------------------------------------------------
# The remote (Objaverse "Remote control" by craazyy, CC BY 4.0, scaled)
# ----------------------------------------------------------------------------------------------
REMOTE_HALF = (0.086, 0.022, 0.009)
REMOTE_MASS = 0.20

# ----------------------------------------------------------------------------------------------
# A phone, a glasses case and a pouch (Objaverse "SmartPhone" by shedmon and "Spectacles case" by
# lib.test, both CC BY 4.0, scaled; the pouch is a plain cloth-covered block):
# more small things a living room has.
# ----------------------------------------------------------------------------------------------
PHONE_HALF = (0.075, 0.037, 0.0045)
PHONE_MASS = 0.18
CASE_HALF = (0.080, 0.035, 0.017)
CASE_MASS = 0.09
POUCH_HALF = (0.095, 0.036, 0.020)
POUCH_MASS = 0.11

# ----------------------------------------------------------------------------------------------
# Cameras
# ----------------------------------------------------------------------------------------------
CAMERAS = {
    # name: (bit in the action's camera mask, width, height, vertical field of view in degrees)
    "overview": (1, 320, 240, 58.0),
    "wrist": (2, 320, 240, 70.0),
}
OVERVIEW_POS = (-0.80, -0.15, 1.80)
OVERVIEW_LOOKAT = (0.55, -0.15, 0.25)
WRIST_CAM_POS = (0.0, 0.065, 0.02)      # in the hand frame (z = approach axis, x = closing axis)
WRIST_CAM_TILT = math.radians(15.0)     # tilted toward the fingertips
DEPTH_MAX_MM = 6000

# ----------------------------------------------------------------------------------------------
# Judging
# ----------------------------------------------------------------------------------------------
SEEN_PIXELS = 60                        # remote pixels in a delivered image that count as "seen"
HOLD_STEPS = 20                         # 1 s of holding the remote clear of everything
LIFT_CLEAR = 0.08                       # the held remote's centre above the surface it lay on
PARTIAL_SEEN = 0.2                      # score for the remote seen in a delivered image
PARTIAL_REVEALED = 0.4                  # score for the remote laid open
# Where the phone, the case and the pouch are when a scene has none.
OUT_OF_ROOM = ((6.0, 6.0, 1.0, 0.0), (7.0, 6.0, 1.0, 0.0), (8.0, 6.0, 1.0, 0.0))

_MASK = (1 << 64) - 1


# ================================================================================================
# Episode specification
# ================================================================================================
@dataclass(frozen=True)
class Scene:
    seed: int
    sofa_pose: tuple        # (dx, dy, turn) of the settee from the nominal layout, about SOFA_PIVOT
    cabinet_pose: tuple     # (dx, dy, turn) of the chest, about CAB_PIVOT
    hanger_swing: float     # rad, the hanger's initial rotation about the peg
    # The hat's and the box's places are in the nominal layout: `place` gives them as they are
    # in the scene.
    hat_drawer: str         # the drawer the hat lies in
    hat_xy: tuple           # x, y of the hat's axis (the drawer closed)
    hat_yaw: float          # direction of the hat's x axis
    box_xy: tuple           # x, y of the box's middle on the chest's top
    box_yaw: float          # direction of the box's x axis
    # World frame, as they are in the scene: (x, y, z of the centre, yaw). A thing is put there
    # and let go; it comes to rest on whatever is below it.
    remote: tuple
    phone: tuple
    case: tuple
    pouch: tuple
    fabric_rgb: tuple       # tints: each multiplies a surface's texture
    pad_rgb: tuple
    wood_rgb: tuple
    cabinet_rgb: tuple
    floor_rgb: tuple
    wall_rgb: tuple
    floor_kind: int
    lamp_xy: tuple
    lamp_level: float

    def as_dict(self):
        return asdict(self)


class _Draw(object):
    """Uniform numbers from a keyed hash of a seed and a stream's name."""

    def __init__(self, seed, stream):
        self._key = struct.pack("<Q", int(seed) & _MASK)
        self._stream = stream.encode()
        self._n = 0

    def uniform(self, low=0.0, high=1.0):
        digest = hashlib.blake2b(self._stream + struct.pack("<I", self._n), key=self._key,
                                 digest_size=8).digest()
        self._n += 1
        return low + (high - low) * (struct.unpack("<Q", digest)[0] >> 11) / float(1 << 53)

    def choice(self, items):
        return items[min(int(self.uniform(0.0, len(items))), len(items) - 1)]


FABRICS = (
    (1.00, 1.00, 1.00), (0.80, 0.88, 1.00), (1.00, 0.88, 0.76), (0.82, 1.00, 0.86),
    (0.96, 0.84, 0.98), (0.78, 0.78, 0.80), (1.00, 0.80, 0.78), (0.92, 0.96, 0.74),
)
WOODS = ((1.00, 1.00, 1.00), (1.00, 0.90, 0.80), (0.82, 0.80, 0.78), (1.00, 0.94, 0.86))
FLOORS = ((1.00, 1.00, 1.00), (0.92, 0.90, 0.88), (1.00, 0.95, 0.88), (0.86, 0.88, 0.90))
WALLS = ((1.00, 1.00, 1.00), (0.92, 0.97, 1.00), (1.00, 0.95, 0.88), (0.94, 1.00, 0.94))


def _shade(rgb, factor):
    return tuple(min(1.0, max(0.0, c * factor)) for c in rgb)


def make_scene(seed, remote, phone=None, case=None, pouch=None, hanger_swing=None):
    """The scene of `seed` with the remote at `remote` = (x, y, z, yaw), world frame, as the
    furniture stands in this scene (`place` turns a point of the nominal layout into it).
    `phone`, `case` and `pouch` likewise, or None for a scene without them. The layout, the hat,
    the box, the hanger's swing (unless given) and the look are drawn from the seed."""
    look = _Draw(seed, "look")
    room = _Draw(seed, "room")
    # The hat lies in one of the drawers, against one of the drawer's side walls, the high side
    # of its brim toward the other.
    hat_drawer = room.choice(DRAWERS)
    flip = room.uniform() < 0.5
    hat_xy = (CAB_X0 + DRAWER_FRONT_X[1] + 0.5 * DRAWER_INNER[0] + room.uniform(-0.004, 0.004),
              CAB_CY + (-1.0 if flip else 1.0) * room.uniform(0.070, 0.080))
    hat_yaw = room.uniform(-0.06, 0.06) + (math.pi if flip else 0.0)
    # The box stands on the chest's top, its long side along the chest's.
    box_xy = (CAB_X0 + BOX_X + room.uniform(-0.015, 0.015), CAB_CY + room.uniform(-0.08, 0.08))
    box_yaw = room.uniform(-0.2, 0.2)
    fabric = _shade(look.choice(FABRICS), look.uniform(0.88, 1.0))
    layout = _Draw(seed, "layout")
    if hanger_swing is None:
        hanger_swing = _Draw(seed, "hanger").uniform(-0.06, 0.06)
    return Scene(
        seed=int(seed) & _MASK,
        sofa_pose=tuple(layout.uniform(lo, hi) for lo, hi in LAYOUT_RANGE),
        cabinet_pose=tuple(layout.uniform(lo, hi) for lo, hi in LAYOUT_RANGE),
        hanger_swing=float(hanger_swing),
        hat_drawer=hat_drawer,
        hat_xy=hat_xy,
        hat_yaw=hat_yaw,
        box_xy=box_xy,
        box_yaw=box_yaw,
        remote=tuple(float(v) for v in remote),
        phone=tuple(float(v) for v in (phone if phone is not None else OUT_OF_ROOM[0])),
        case=tuple(float(v) for v in (case if case is not None else OUT_OF_ROOM[1])),
        pouch=tuple(float(v) for v in (pouch if pouch is not None else OUT_OF_ROOM[2])),
        fabric_rgb=fabric,
        pad_rgb=_shade(fabric, look.uniform(0.90, 1.0)),
        wood_rgb=_shade(look.choice(WOODS), look.uniform(0.85, 1.0)),
        cabinet_rgb=_shade(look.choice(WOODS), look.uniform(0.8, 1.0)),
        floor_rgb=_shade(look.choice(FLOORS), look.uniform(0.88, 1.0)),
        wall_rgb=_shade(look.choice(WALLS), look.uniform(0.9, 1.0)),
        floor_kind=min(int(look.uniform(0.0, 2.0)), 1),
        lamp_xy=(look.uniform(-0.6, 0.4), look.uniform(-0.8, 0.8)),
        lamp_level=look.uniform(0.75, 1.0),
    )


# ================================================================================================
# The model
# ================================================================================================
def _asset_root():
    import importlib.util
    spec = importlib.util.find_spec("robosuite")
    if spec is None:
        raise RuntimeError("the robosuite asset files (Panda model) are not installed")
    return os.path.join(list(spec.submodule_search_locations)[0], "models", "assets")


def room_dir():
    """The folder with the room's meshes and textures (REMOTE_ROOM_DIR, else /opt/room, else
    `room/` beside this file)."""
    for path in (os.environ.get("REMOTE_ROOM_DIR"), "/opt/room",
                 os.path.join(os.path.dirname(os.path.abspath(__file__)), "room")):
        if path and os.path.isfile(os.path.join(path, "sofa_frame.obj")):
            return path
    raise RuntimeError("the room's meshes and textures were not found (REMOTE_ROOM_DIR)")


def _fmt(values):
    return " ".join("{:.6g}".format(float(v)) for v in values)


HIDDEN = 'group="3"'            # collision shapes: never drawn (the cameras draw groups 0 to 2)
SHOWN = 'contype="0" conaffinity="0" group="1" mass="0"'     # drawn only: no contact, no weight


def _box(name, pos, half, material=None, rgba=None, extra=""):
    if material:
        look = 'material="{}"'.format(material)
    elif rgba is not None:
        look = 'rgba="{}"'.format(_fmt(rgba))
    else:
        look = HIDDEN
    return '<geom name="{}" type="box" pos="{}" size="{}" {} {}/>'.format(
        name, _fmt(pos), _fmt(half), look, extra)


def _mesh(name, mesh, material, pos=(0, 0, 0), extra=""):
    return '<geom name="{}" type="mesh" mesh="{}" material="{}" pos="{}" {} {}/>'.format(
        name, mesh, material, _fmt(pos), SHOWN, extra)


def _panda(root):
    """The Panda's body tree with the hand attached, and its mesh assets, from the robosuite
    asset files; the actuators are this file's own."""
    arm = ET.parse(os.path.join(root, "robots", "panda", "robot.xml")).getroot()
    grip = ET.parse(os.path.join(root, "grippers", "panda_gripper.xml")).getroot()
    assets = []
    for tree, folder in ((arm, os.path.join(root, "robots", "panda")), (grip, os.path.join(root, "grippers"))):
        for item in tree.find("asset"):
            if item.tag == "mesh":
                item.set("file", os.path.join(folder, item.get("file")))
            assets.append(ET.tostring(item, encoding="unicode"))
    link0 = arm.find("worldbody").find("body").find("body")
    for body in link0.iter("body"):
        body.set("gravcomp", "1")
    hand_mount = [b for b in link0.iter("body") if b.get("name") == "right_hand"][0]
    for camera in list(hand_mount.findall("camera")):
        hand_mount.remove(camera)
    gripper = grip.find("worldbody").find("body")
    for body in gripper.iter("body"):
        body.set("gravcomp", "1")
    for geom in gripper.iter("geom"):
        if geom.get("name", "").endswith("pad_collision"):
            geom.set("size", "0.009 0.004 0.010")
            geom.set("friction", "1.5 0.05 0.0001")
            geom.set("condim", "4")
            geom.set("rgba", "0.15 0.15 0.15 1")
    for joint in gripper.iter("joint"):
        joint.set("damping", "60")
        joint.set("armature", "0.5")
        joint.set("frictionloss", "0.5")
    for site in list(gripper.iter("site")):
        site.set("rgba", "0 0 0 0")
    for site in link0.iter("site"):
        site.set("rgba", "0 0 0 0")
    tilt = WRIST_CAM_TILT
    cam = ET.SubElement(gripper, "camera")
    cam.set("name", "wrist")
    cam.set("pos", _fmt(WRIST_CAM_POS))
    cam.set("xyaxes", _fmt((1, 0, 0, 0, -math.cos(tilt), -math.sin(tilt))))
    cam.set("fovy", _fmt((CAMERAS["wrist"][3],)))
    hand_mount.append(gripper)
    return "\n".join(assets), ET.tostring(link0, encoding="unicode")


def _hook_points():
    """The hanger's wire hook in the hanger frame (x along the board, z up in its plane)."""
    # The neck stands off-centre so that the hook's seat is over the middle of the board and
    # the hanger hangs level.
    points = [(-HOOK_R, 0.0, HANGER_BOARD[2]), (-HOOK_R, 0.0, HOOK_NECK_Z)]
    for degrees in (135.0, 90.0, 45.0, 0.0, -40.0):
        a = math.radians(degrees)
        points.append((HOOK_R * math.cos(a), 0.0, HOOK_NECK_Z + HOOK_R * math.sin(a)))
    return points


# Where the peg sits in the hanger frame while the hanger hangs on it.
HOOK_SEAT = (0.0, 0.0, HOOK_NECK_Z + HOOK_R - HOOK_WIRE - PEG_R)


SOFA_PARTS = 72                         # convex pieces of the settee's collision shape (room/sofa_col_NN.obj)


def _sofa():
    """The settee: its collision shape is a convex decomposition of the drawn model, with a
    flat deck for the pads to lie on and a closed underside."""
    y0, y1 = SOFA_Y
    width = y1 - y0
    cy = 0.5 * (y0 + y1)
    depth = SEAT_DEPTH + 0.10
    inner0, inner1 = y0 + ARM_W, y1 - ARM_W
    mid = 0.5 * (inner0 + inner1)
    parts = []
    for i in range(SOFA_PARTS):
        parts.append('<geom name="sofa{i}" type="mesh" mesh="sofa_col_{i:02d}" pos="{p}" {h}/>'.format(
            i=i, p=_fmt((SOFA_X0, cy, 0)), h=HIDDEN))
    parts.append(_mesh("sofa_look", "sofa_frame", "sofa", (SOFA_X0, cy, 0)))
    # The deck under the pads (a board, drawn as it is) and the cover that closes the underside.
    deck = ((SOFA_X0 + 0.5 * SEAT_DEPTH + 0.01, mid, DECK_Z - 0.01), (0.5 * SEAT_DEPTH - 0.01, 0.5 * (inner1 - inner0), 0.01))
    under = ((SOFA_X0 + 0.5 * depth, cy, UNDER_GAP + 0.006), (0.5 * depth - 0.012, 0.5 * width - 0.03, 0.006))
    parts.append(_box("sofa_deck", deck[0], deck[1]))
    parts.append(_box("deck_look", deck[0], deck[1], material="deck", extra=SHOWN))
    parts.append(_box("sofa_under", under[0], under[1]))
    parts.append(_box("under_look", under[0], under[1], rgba=(0.12, 0.11, 0.1, 1), extra=SHOWN))
    return "\n".join(parts)


def _pad(name, yc):
    half = (0.5 * PAD_SIZE[0], 0.5 * PAD_SIZE[1], 0.5 * PAD_SIZE[2])
    x = SOFA_X0 - PAD_OVERHANG + half[0]
    z = DECK_Z + half[2]
    volume = PAD_SIZE[0] * PAD_SIZE[1] * PAD_SIZE[2]
    return ('<body name="pad_{n}" pos="{p}"><freejoint name="pad_{n}_free"/>'
            '<geom name="pad_{n}" type="box" size="{s}" density="{d}" {h} contype="5" conaffinity="6" '
            'friction="0.9 0.01 0.0001" solref="0.01 1" condim="4"/>{look}</body>').format(
                n=name, p=_fmt((x, yc, z)), s=_fmt(half), d=_fmt((PAD_MASS / volume,)), h=HIDDEN,
                look=_mesh("pad_{}_look".format(name), "cushion", "pad"))


def _cabinet():
    """The chest's carcass in the cabinet frame: collision boxes and the drawn model."""
    hw = CAB_PANEL_W
    t = 0.02
    parts = []
    # The carcass stands on four legs: its side panels end at the bottom board's underside.
    low = CAB_UNDER
    for name, ys in (("cab_side_r", -hw + 0.5 * t), ("cab_side_l", hw - 0.5 * t)):
        parts.append(_box(name, (0.5 * CAB_DEPTH + 0.01, ys, 0.5 * (CAB_H + low)), (0.5 * CAB_DEPTH - 0.01, 0.5 * t, 0.5 * (CAB_H - low))))
    for i, (xs, ys) in enumerate(((CAB_LEG_X[0], -1), (CAB_LEG_X[0], 1), (CAB_LEG_X[1], -1), (CAB_LEG_X[1], 1))):
        parts.append(_box("cab_leg{}".format(i), (xs, ys * CAB_LEG_Y, 0.5 * low), CAB_LEG_HALF + (0.5 * low,)))
    # The four corner posts stand a little proud of the side panels.
    for i, (xs, ys) in enumerate(((0.058, -1), (0.058, 1), (0.402, -1), (0.402, 1))):
        parts.append(_box("cab_post{}".format(i), (xs, ys * 0.5 * (CAB_PANEL_W + CAB_HALF_W), 0.40),
                          (0.033, 0.5 * (CAB_HALF_W - CAB_PANEL_W) + 0.004, 0.30)))
    parts.append(_box("cab_back", (CAB_DEPTH - 0.008, 0, 0.42), (0.008, hw, 0.31)))
    parts.append(_box("cab_top", (0.5 * CAB_DEPTH, 0, CAB_H - 0.014), (0.5 * CAB_DEPTH, hw + 0.028, 0.014)))
    parts.append(_box("cab_bottom", (0.5 * CAB_DEPTH + 0.01, 0, 0.12), (0.5 * CAB_DEPTH - 0.01, hw, 0.012)))
    # The front frame's rails between the drawers.
    for i, (z0, z1) in enumerate(((0.108, 0.135), (0.293, 0.321), (0.479, 0.513), (0.671, 0.70))):
        parts.append(_box("cab_rail{}".format(i), (0.035, 0, 0.5 * (z0 + z1)), (0.01, hw, 0.5 * (z1 - z0))))
    # The peg the hanger hangs on, on the outer side, pointing away from the cabinet and a little up.
    parts.append('<geom name="peg" type="capsule" fromto="{}" size="{}" rgba="0.62 0.52 0.3 1"/>'.format(
        _fmt((PEG_X - CAB_X0, -hw, PEG_Z - 0.008, PEG_X - CAB_X0, -hw - PEG_LEN, PEG_Z + 0.008)), _fmt((PEG_R,))))
    parts.append(_mesh("cab_look", "commode_body", "commode"))
    return "\n".join(parts)


def _drawer(name):
    fz0, fz1 = DRAWER_FRONT_Z[name]
    floor_z = DRAWER_FLOOR_Z[name]
    depth, width = DRAWER_INNER
    x0 = DRAWER_FRONT_X[1]
    wall = 0.008
    pre = "drawer_{}_".format(name)
    parts = ['<body name="drawer_{n}" pos="0 0 0">'.format(n=name),
             '<joint name="drawer_{n}" type="slide" axis="-1 0 0" range="0 {r}" limited="true" '
             'damping="12" frictionloss="2.5" armature="0.05"/>'.format(n=name, r=DRAWER_TRAVEL),
             '<inertial pos="{}" mass="1.2" diaginertia="0.02 0.02 0.03"/>'.format(_fmt((0.24, 0, floor_z)))]
    parts.append(_box(pre + "front", (0.5 * (DRAWER_FRONT_X[0] + x0), 0, 0.5 * (fz0 + fz1)),
                      (0.5 * (x0 - DRAWER_FRONT_X[0]), 0.5 * width + wall + 0.005, 0.5 * (fz1 - fz0))))
    parts.append(_box(pre + "floor", (x0 + 0.5 * depth, 0, floor_z - 0.004), (0.5 * depth, 0.5 * width + wall, 0.004)))
    wz = floor_z + 0.5 * DRAWER_WALL_H
    for side, ys in (("wl", 0.5 * width + 0.5 * wall), ("wr", -0.5 * width - 0.5 * wall)):
        parts.append(_box(pre + side, (x0 + 0.5 * depth, ys, wz), (0.5 * depth, 0.5 * wall, 0.5 * DRAWER_WALL_H)))
    parts.append(_box(pre + "wb", (x0 + depth + 0.5 * wall, 0, wz), (0.5 * wall, 0.5 * width + wall, 0.5 * DRAWER_WALL_H)))
    for i, ys in enumerate((-KNOB_Y[name], KNOB_Y[name])):
        parts.append(_box(pre + "knob{}".format(i), (KNOB_X, ys, KNOB_Z[name]), (KNOB_HALF, KNOB_HALF, KNOB_HALF),
                          extra='friction="1.2 0.02 0.0001" condim="4"'))
        parts.append(_box(pre + "stem{}".format(i), (0.5 * (KNOB_X + KNOB_HALF + DRAWER_FRONT_X[0]), ys, KNOB_Z[name]),
                          (0.5 * (DRAWER_FRONT_X[0] - KNOB_X - KNOB_HALF), 0.005, 0.005)))
    parts.append(_mesh(pre + "look", "commode_drawer_" + name, "commode"))
    parts.append("</body>")
    return "\n".join(parts)


def _hanger():
    bx, by, bz = HANGER_BOARD
    volume = 8.0 * bx * by * bz
    parts = ['<body name="hanger" pos="0 0 1">'
             # A little damping, as the air and the hook's rubbing give a real hanger: without it
             # the hanger would swing on its peg for ever.
             '<joint name="hanger_free" type="free" damping="0.002"/>',
             '<geom name="hanger_board" type="box" size="{}" density="{}" {} '
             'friction="0.4 0.02 0.0001" condim="4"/>'.format(_fmt(HANGER_BOARD), _fmt((0.9 * HANGER_MASS / volume,)), HIDDEN),
             _mesh("hanger_look", "hanger", "hanger")]
    points = _hook_points()
    for i in range(len(points) - 1):
        parts.append('<geom name="hanger_hook{}" type="capsule" fromto="{}" size="{}" material="steel" '
                     'density="7800" friction="0.15 0.005 0.0001"/>'.format(i, _fmt(points[i] + points[i + 1]), _fmt((HOOK_WIRE,))))
    # Two trouser clips under the board (looks only).
    for i, xs in enumerate((-0.13, 0.13)):
        parts.append(_box("hanger_clip{}".format(i), (xs, 0, -bz - 0.008), (0.012, 0.006, 0.008),
                          material="steel", extra='contype="0" conaffinity="0" mass="0.002"'))
    parts.append("</body>")
    return "\n".join(parts)


def _hat():
    parts = ['<body name="hat" pos="{}"><joint name="hat_free" type="free" damping="0.0005"/>'.format(
                 _fmt((CAB_X0 + 0.25, CAB_CY, DRAWER_FLOOR_Z["upper"] + 0.001))),
             '<inertial pos="0 0 0.04" mass="{}" diaginertia="0.0007 0.0007 0.0012"/>'.format(HAT_MASS)]
    for i in range(HAT_PARTS):
        parts.append('<geom name="hat{i}" type="mesh" mesh="hat_col_{i:02d}" {h} friction="0.7 0.01 0.0001" '
                     'condim="4"/>'.format(i=i, h=HIDDEN))
    parts.append(_mesh("hat_look", "hat", "hat_cloth"))
    parts.append("</body>")
    return "\n".join(parts)


def _storage_box():
    """The open box on the chest: a floor and four walls, drawn as they are."""
    ix, iy = 0.5 * BOX_INNER[0], 0.5 * BOX_INNER[1]
    t, h = BOX_WALL, BOX_WALL_H
    extra = 'friction="0.6 0.01 0.0001" condim="4" group="1"'
    parts = ['<body name="box" pos="{}"><freejoint name="box_free"/>'.format(_fmt((CAB_X0 + BOX_X, CAB_CY, CAB_H + 0.001))),
             '<inertial pos="0 0 0.02" mass="{}" diaginertia="0.008 0.006 0.012"/>'.format(BOX_MASS),
             _box("box_floor", (0, 0, 0.5 * t), (ix + t, iy + t, 0.5 * t), material="box", extra=extra)]
    zc = t + 0.5 * h
    for name, pos, half in (("box_w0", (-ix - 0.5 * t, 0, zc), (0.5 * t, iy + t, 0.5 * h)),
                            ("box_w1", (ix + 0.5 * t, 0, zc), (0.5 * t, iy + t, 0.5 * h)),
                            ("box_w2", (0, -iy - 0.5 * t, zc), (ix, 0.5 * t, 0.5 * h)),
                            ("box_w3", (0, iy + 0.5 * t, zc), (ix, 0.5 * t, 0.5 * h))):
        parts.append(_box(name, pos, half, material="box", extra=extra))
    parts.append("</body>")
    return "\n".join(parts)


def _remote():
    hx, hy, hz = REMOTE_HALF
    volume = 8.0 * hx * hy * hz
    return ('<body name="remote" pos="0 0 1"><freejoint name="remote_free"/>'
            '<geom name="remote" type="box" size="{}" density="{}" {} contype="9" conaffinity="10" '
            'friction="0.5 0.01 0.0001" condim="4"/>'
            '{}</body>').format(_fmt(REMOTE_HALF), _fmt((REMOTE_MASS / volume,)), HIDDEN,
                                _mesh("remote_look", "remote", "remote"))


def _clutter():
    """The phone, the glasses case and the pouch: boxes as their models are drawn. A pad is as
    soft over the phone as over the remote (those two do not touch it); they touch each other."""
    out = []
    for i, (name, half, mass, bits) in enumerate((
            ("phone", PHONE_HALF, PHONE_MASS, 'contype="9" conaffinity="10"'), ("case", CASE_HALF, CASE_MASS, ""),
            ("pouch", POUCH_HALF, POUCH_MASS, ""))):
        volume = 8.0 * half[0] * half[1] * half[2]
        if name == "pouch":             # drawn as the box it is
            out.append('<body name="pouch" pos="{x} 6 1"><freejoint name="pouch_free"/>'
                       '<geom name="pouch" type="box" size="{s}" density="{d}" {h} friction="0.6 0.01 0.0001" condim="4"/>'
                       '<geom name="pouch_look" type="box" size="{s}" material="pouch" {v}/></body>'.format(
                           x=6 + i, s=_fmt(half), d=_fmt((mass / volume,)), h=HIDDEN, v=SHOWN))
            continue
        mesh = "phone" if name == "phone" else "glasses_case"
        out.append('<body name="{n}" pos="{x} 6 1"><freejoint name="{n}_free"/>'
                   '<geom name="{n}" type="box" size="{s}" density="{d}" {h} {b} friction="0.5 0.01 0.0001" condim="4"/>'
                   '{l0}{l1}</body>'.format(n=name, x=6 + i, s=_fmt(half), d=_fmt((mass / volume,)), h=HIDDEN, b=bits,
                                            l0=_mesh(name + "_look0", mesh + "_0", mesh + "_0"),
                                            l1=_mesh(name + "_look1", mesh + "_1", mesh + "_1")))
    return "\n".join(out)


def _look_at(pos, target):
    forward = np.asarray(target, float) - np.asarray(pos, float)
    forward /= np.linalg.norm(forward)
    right = np.cross(forward, (0.0, 0.0, 1.0))
    right /= np.linalg.norm(right)
    up = np.cross(right, forward)
    return tuple(right) + tuple(up)


def build_xml():
    """The MJCF of the room. The look of an episode is set on the compiled model at reset."""
    assets, arm = _panda(_asset_root())
    actuators = ['<general name="act_rail" joint="rail" biastype="affine" gainprm="30000" '
                 'biasprm="0 -30000 -3000" forcerange="-800 800" ctrlrange="{}"/>'.format(_fmt(RAIL_RANGE))]
    for i in range(7):
        actuators.append('<general name="act_j{n}" joint="joint{n}" biastype="affine" gainprm="{kp}" '
                         'biasprm="0 -{kp} -{kv}" forcerange="-{f} {f}" ctrlrange="{lo} {hi}"/>'.format(
                             n=i + 1, kp=ARM_KP[i], kv=ARM_KV[i], f=ARM_FORCE[i], lo=ARM_LIMITS[i][0], hi=ARM_LIMITS[i][1]))
    actuators.append('<position name="act_f1" joint="finger_joint1" kp="1500" ctrlrange="0 0.04" forcerange="-{f} {f}"/>'.format(f=FINGER_FORCE))
    actuators.append('<position name="act_f2" joint="finger_joint2" kp="1500" ctrlrange="-0.04 0" forcerange="-{f} {f}"/>'.format(f=FINGER_FORCE))
    room = room_dir()
    pad = "{} {} {}".format(*PAD_SIZE)
    return """<mujoco model="living_room">
  <compiler angle="radian" autolimits="true"/>
  <option timestep="{dt}" integrator="implicitfast" cone="elliptic" impratio="20" noslip_iterations="10"/>
  <size memory="96M"/>
  <visual>
    <global offwidth="1280" offheight="960"/>
    <quality shadowsize="4096" offsamples="4"/>
    <headlight ambient="0.36 0.36 0.36" diffuse="0.28 0.28 0.28" specular="0 0 0"/>
    <map znear="0.01" zfar="12"/>
  </visual>
  <asset>
    {assets}
    <texture name="sofa" type="2d" file="{room}/sofa.png"/>
    <texture name="commode" type="2d" file="{room}/commode.png"/>
    <texture name="remote" type="2d" file="{room}/remote.png"/>
    <texture name="floor_a" type="2d" file="{room}/floor_a.png"/>
    <texture name="floor_b" type="2d" file="{room}/floor_b.png"/>
    <texture name="wall" type="2d" file="{room}/wall.png"/>
    <texture name="oak" type="2d" file="{room}/oak.png"/>
    <material name="sofa" texture="sofa" specular="0.08" shininess="0.1"/>
    <texture name="fabric" type="2d" file="{room}/fabric.png"/>
    <material name="pad" texture="fabric" specular="0.08" shininess="0.1"/>
    <material name="sofa_plain" texture="fabric" texuniform="true" texrepeat="3 3" specular="0.08" shininess="0.1"/>
    <material name="commode" texture="commode" specular="0.3" shininess="0.4"/>
    <material name="remote" texture="remote" specular="0.4" shininess="0.5"/>
    <material name="floor_a" texture="floor_a" texrepeat="4 4" specular="0.2" shininess="0.3"/>
    <material name="floor_b" texture="floor_b" texrepeat="4 4" specular="0.2" shininess="0.3"/>
    <material name="wall" texture="wall" texrepeat="2 1" specular="0" shininess="0"/>
    <material name="oak" texture="oak" texuniform="true" texrepeat="3 3" specular="0.15" shininess="0.2"/>
    <material name="deck" texture="oak" texuniform="true" texrepeat="3 3" specular="0.1" shininess="0.1"/>
    <material name="hanger" texture="oak" specular="0.3" shininess="0.4"/>
    <material name="steel" rgba="0.78 0.78 0.8 1" specular="0.8" shininess="0.8"/>
    <material name="rail" rgba="0.3 0.31 0.33 1" specular="0.4" shininess="0.5"/>
    <mesh name="sofa_frame" file="{room}/sofa_frame.obj"/>
    <mesh name="cushion" file="{room}/cushion.obj" scale="{pad}"/>
    {sofa_meshes}
    <mesh name="commode_body" file="{room}/commode_body.obj"/>
    <mesh name="commode_drawer_upper" file="{room}/commode_drawer_upper.obj"/>
    <mesh name="commode_drawer_mid" file="{room}/commode_drawer_mid.obj"/>
    <mesh name="commode_drawer_lower" file="{room}/commode_drawer_lower.obj"/>
    <mesh name="remote" file="{room}/remote.obj"/>
    <mesh name="hanger" file="{room}/hanger.obj"/>
    <texture name="hat" type="2d" file="{room}/hat.png"/>
    <material name="hat_cloth" texture="hat" specular="0.05" shininess="0.05"/>
    <mesh name="hat" file="{room}/hat.obj" scale="{hat_scale}"/>
    <material name="pouch" texture="fabric" texuniform="true" texrepeat="8 8" rgba="0.70 0.30 0.22 1" specular="0.05" shininess="0.05"/>
    <material name="box" texture="oak" texuniform="true" texrepeat="5 5" rgba="0.80 0.66 0.48 1" specular="0.1" shininess="0.1"/>
    {hat_meshes}
    {clutter_assets}
  </asset>
  <worldbody>
    <light name="lamp" pos="-0.4 0 2.6" dir="0.25 0 -1" diffuse="0.72 0.7 0.67" specular="0.1 0.1 0.1" castshadow="true"/>
    <light name="fill" pos="-1.5 1.5 2.2" dir="0.6 -0.5 -1" diffuse="0.22 0.22 0.25" specular="0 0 0" castshadow="false"/>
    <geom name="floor" type="plane" size="4 4 0.1" material="floor_a" friction="0.5 0.01 0.0001" condim="4"/>
    <geom name="wall_back" type="box" pos="{wall_x} 0 1.2" size="0.02 3 1.2" material="wall"/>
    <geom name="wall_right" type="box" pos="-0.5 -2.4 1.2" size="2 0.02 1.2" material="wall"/>
    <geom name="skirting_back" type="box" pos="{skirt_x} 0 0.05" size="0.012 3 0.05" rgba="0.93 0.93 0.91 1" contype="0" conaffinity="0"/>
    <geom name="skirting_right" type="box" pos="-0.5 -2.37 0.05" size="2 0.012 0.05" rgba="0.93 0.93 0.91 1" contype="0" conaffinity="0"/>
    <geom name="rail_beam" type="box" pos="0 {rail_mid} 0.02" size="0.07 {rail_half} 0.02" material="rail"/>
    <camera name="overview" pos="{cam_pos}" xyaxes="{cam_axes}" fovy="{cam_fovy}"/>
    <body name="pedestal" pos="0 0 0">
      <joint name="rail" type="slide" axis="0 1 0" range="{rail_lo} {rail_hi}" damping="50" armature="2"/>
      <inertial pos="0 0 0.15" mass="40" diaginertia="2 2 2"/>
      <geom name="pedestal" type="box" pos="0 0 {ped_z}" size="0.11 0.11 {ped_half}" material="rail"/>
      <body name="arm_mount" pos="0 0 {base_z}">
        {arm}
      </body>
    </body>
    <body name="sofa" pos="0 0 0">
      {sofa}
    </body>
    {pad_left}
    {pad_right}
    <body name="cabinet" pos="{cab_pos}">
      {cabinet}
      {drawer_upper}
      {drawer_mid}
      {drawer_lower}
    </body>
    {hanger}
    {hat}
    {box}
    {clutter}
    {remote}
  </worldbody>
  <contact>
    <exclude body1="pedestal" body2="link0"/>
    <exclude body1="pedestal" body2="link1"/>
  </contact>
  <actuator>
    {actuators}
  </actuator>
  <sensor>
    <force name="wrist_force" site="ft_frame"/>
    <torque name="wrist_torque" site="ft_frame"/>
  </sensor>
</mujoco>""".format(
        dt=SIM_DT, assets=assets, arm=arm, room=room, pad=pad, sofa=_sofa(), cabinet=_cabinet(),
        pad_left=_pad("left", PAD_Y["left"]), pad_right=_pad("right", PAD_Y["right"]),
        drawer_upper=_drawer("upper"), drawer_mid=_drawer("mid"), drawer_lower=_drawer("lower"),
        hanger=_hanger(), remote=_remote(), hat=_hat(), box=_storage_box(), clutter=_clutter(),
        hat_scale=_fmt((HAT_SCALE,) * 3),
        clutter_assets="\n    ".join(
            '<mesh name="{n}" file="{r}/{n}.obj"/><texture name="{n}" type="2d" file="{r}/{n}.png"/>'
            '<material name="{n}" texture="{n}" specular="0.3" shininess="0.4"/>'.format(n=n, r=room)
            for n in ("phone_0", "phone_1", "glasses_case_0", "glasses_case_1")),
        sofa_meshes="\n    ".join('<mesh name="sofa_col_{i:02d}" file="{r}/sofa_col_{i:02d}.obj"/>'.format(i=i, r=room)
                                for i in range(SOFA_PARTS)),
        hat_meshes="\n    ".join('<mesh name="hat_col_{i:02d}" file="{r}/hat_col_{i:02d}.obj" scale="{s}"/>'.format(i=i, r=room, s=_fmt((HAT_SCALE,) * 3))
                               for i in range(HAT_PARTS)),
        cab_pos=_fmt((CAB_X0, CAB_CY, 0)),
        wall_x=SOFA_X0 + 0.82, skirt_x=SOFA_X0 + 0.79,
        actuators="\n    ".join(actuators),
        rail_mid=0.5 * (RAIL_RANGE[0] + RAIL_RANGE[1]), rail_half=0.5 * (RAIL_RANGE[1] - RAIL_RANGE[0]) + 0.15,
        rail_lo=RAIL_RANGE[0], rail_hi=RAIL_RANGE[1],
        ped_z=0.04 + 0.5 * (BASE_Z - 0.04), ped_half=0.5 * (BASE_Z - 0.04), base_z=BASE_Z,
        cam_pos=_fmt(OVERVIEW_POS), cam_axes=_fmt(_look_at(OVERVIEW_POS, OVERVIEW_LOOKAT)),
        cam_fovy=CAMERAS["overview"][3])


# ================================================================================================
# Images
# ================================================================================================
def camera_intrinsics(name):
    """(fx, fy, cx, cy) of a camera's pinhole model; the centre of pixel (u, v) is (u + 0.5, v + 0.5)."""
    _, width, height, fovy = CAMERAS[name]
    f = 0.5 * height / math.tan(0.5 * math.radians(fovy))
    return f, f, 0.5 * width, 0.5 * height


def _encode(array):
    return {"dtype": str(array.dtype), "shape": list(array.shape),
            "b64": base64.b64encode(np.ascontiguousarray(array).tobytes()).decode("ascii")}


def decode_image(item):
    """An observation's image entry as a NumPy array."""
    data = base64.b64decode(item["b64"])
    return np.frombuffer(data, dtype=np.dtype(item["dtype"])).reshape(item["shape"])


def _quat_yaw(yaw):
    return np.array([math.cos(0.5 * yaw), 0.0, 0.0, math.sin(0.5 * yaw)])


# ================================================================================================
# The environment
# ================================================================================================
class Env(object):
    """One living room. `reset(spec)` starts an episode, `step(action)` advances it by one
    control step (0.05 s) and returns the next observation.

    Action: 10 numbers `[rail, q1..q7, grip, cams]`: position targets for the rail (m) and the
    seven arm joints (rad), the finger opening (m, 0 to 0.08) and a camera bit mask for the next
    observation (1 = overview, 2 = wrist). Targets are clipped to the joint ranges and followed
    at no more than `MAX_SPEED`.
    """

    def __init__(self, render=True):
        self.model = mujoco.MjModel.from_xml_string(build_xml())
        self.data = mujoco.MjData(self.model)
        self.render_enabled = bool(render)
        self._renderers = {}
        m = self.model
        self._name = lambda kind, name: mujoco.mj_name2id(m, kind, name)
        J, B, G = mujoco.mjtObj.mjOBJ_JOINT, mujoco.mjtObj.mjOBJ_BODY, mujoco.mjtObj.mjOBJ_GEOM
        self.arm_qadr = [m.jnt_qposadr[self._name(J, n)] for n in ["rail"] + ["joint{}".format(i) for i in range(1, 8)]]
        self.arm_dadr = [m.jnt_dofadr[self._name(J, n)] for n in ["rail"] + ["joint{}".format(i) for i in range(1, 8)]]
        self.finger_qadr = [m.jnt_qposadr[self._name(J, n)] for n in ("finger_joint1", "finger_joint2")]
        self.drawer_qadr = {n: m.jnt_qposadr[self._name(J, "drawer_" + n)] for n in DRAWERS}
        self.free_qadr = {n: m.jnt_qposadr[self._name(J, n + "_free")] for n in ("remote", "hanger", "pad_left", "pad_right", "hat", "box", "phone", "case", "pouch")}
        self.free_dadr = {n: m.jnt_dofadr[self._name(J, n + "_free")] for n in ("remote", "hanger", "pad_left", "pad_right", "hat")}
        self.body = {n: self._name(B, n) for n in ("remote", "hanger", "hat", "box", "pad_left", "pad_right", "drawer_upper",
                                                   "drawer_mid", "drawer_lower", "right_gripper", "pedestal",
                                                   "sofa", "cabinet")}
        self.grip_site = self._name(mujoco.mjtObj.mjOBJ_SITE, "grip_site")
        self.cam_id = {n: self._name(mujoco.mjtObj.mjOBJ_CAMERA, n) for n in CAMERAS}
        self.pad_geoms = (self._name(G, "finger1_pad_collision"), self._name(G, "finger2_pad_collision"))
        self.finger_geoms = self.pad_geoms + (self._name(G, "finger1_collision"), self._name(G, "finger2_collision"))
        self.remote_geom = self._name(G, "remote")
        self.peg_geom = self._name(G, "peg")
        self.floor_geom = self._name(G, "floor")
        self.robot_bodies = set()
        ped = self.body["pedestal"]
        for b in range(m.nbody):
            p = b
            while p != 0:
                if p == ped:
                    self.robot_bodies.add(b)
                    break
                p = m.body_parentid[p]
        self.mat = {n: self._name(mujoco.mjtObj.mjOBJ_MATERIAL, n)
                    for n in ("sofa", "sofa_plain", "pad", "oak", "deck", "commode", "floor_a", "floor_b", "wall", "hanger")}
        # Collision shapes are not drawn: the room's are in group 3 already, the robot's go there.
        for g in range(m.ngeom):
            if m.geom_bodyid[g] in self.robot_bodies and (m.geom_contype[g] or m.geom_conaffinity[g]) \
                    and m.geom(g).name != "pedestal" and not m.geom(g).name.endswith("pad_collision"):
                m.geom_group[g] = 3          # (the fingers' rubber pads are what they look like)
        self.scene_option = mujoco.MjvOption()
        self.scene_option.geomgroup[3] = 0
        self.solid = np.array([1, 0, 0, 1, 0, 0], dtype=np.uint8)      # what a ray can hit
        self.light = self._name(mujoco.mjtObj.mjOBJ_LIGHT, "lamp")
        self.spec = None
        self.steps = 0
        self.target = np.zeros(9)
        self.rng = np.random.default_rng(0)

    # ------------------------------------------------------------------------------------------
    def reset(self, spec):
        """Start the episode of the scene `spec` (a `Scene`); returns the first observation, which
        carries both cameras' images."""
        m, d = self.model, self.data
        self.spec = spec
        self.rng = np.random.default_rng([spec.seed & 0xFFFFFFFF, spec.seed >> 32, 77])
        mujoco.mj_resetData(m, d)
        self._apply_look(spec)
        q0 = (RAIL_HOME,) + ARM_HOME
        for adr, value in zip(self.arm_qadr, q0):
            d.qpos[adr] = value
        d.qpos[self.finger_qadr[0]] = 0.04
        d.qpos[self.finger_qadr[1]] = -0.04
        self.target = np.array(q0 + (GRIP_RANGE[1],))
        self._set_ctrl()
        # The furniture where this episode has it.
        sofa, cab = spec.sofa_pose, spec.cabinet_pose
        (sx, sy), _ = place(SOFA_PIVOT, sofa, (0.0, 0.0))
        m.body_pos[self.body["sofa"]] = (sx, sy, 0.0)
        m.body_quat[self.body["sofa"]] = _quat_yaw(sofa[2])
        m.body_pos[self.body["cabinet"]] = (CAB_PIVOT[0] + cab[0], CAB_PIVOT[1] + cab[1], 0.0)
        m.body_quat[self.body["cabinet"]] = _quat_yaw(cab[2])
        half = (0.5 * PAD_SIZE[0], 0.5 * PAD_SIZE[2])
        for side in ("left", "right"):
            (px, py), _ = place(SOFA_PIVOT, sofa, (SOFA_X0 - PAD_OVERHANG + half[0], PAD_Y[side]))
            adr = self.free_qadr["pad_" + side]
            d.qpos[adr:adr + 3] = (px, py, DECK_Z + half[1])
            d.qpos[adr + 3:adr + 7] = _quat_yaw(sofa[2])
        # The hanger on its peg.
        y_plane = CAB_Y[0] - HANG_OFFSET
        along = CAB_HALF_W - CAB_PANEL_W + HANG_OFFSET          # from the peg's root
        peg_z = PEG_Z - 0.008 + (along / PEG_LEN) * 0.016
        swing = spec.hanger_swing
        c, s = math.cos(swing), math.sin(swing)
        # Hanger frame: x along +world x, z up, rotated by `swing` about its normal (world y).
        seat = HOOK_SEAT
        origin = (PEG_X - (c * seat[0] + s * seat[2]), y_plane, peg_z + PEG_R + 0.0005 - (-s * seat[0] + c * seat[2]) + PEG_R)
        adr = self.free_qadr["hanger"]
        (hx, hy), _ = place(CAB_PIVOT, cab, origin[:2])
        d.qpos[adr:adr + 3] = (hx, hy, origin[2])
        turned = np.empty(4)
        mujoco.mju_mulQuat(turned, _quat_yaw(cab[2]), np.array([math.cos(0.5 * swing), 0.0, math.sin(0.5 * swing), 0.0]))
        d.qpos[adr + 3:adr + 7] = turned
        adr = self.free_qadr["hat"]
        (hx, hy), yaw = place(CAB_PIVOT, cab, spec.hat_xy, spec.hat_yaw)
        d.qpos[adr:adr + 3] = (hx, hy, DRAWER_FLOOR_Z[spec.hat_drawer] + 0.001)
        d.qpos[adr + 3:adr + 7] = _quat_yaw(yaw)
        adr = self.free_qadr["box"]
        (bx, by), yaw = place(CAB_PIVOT, cab, spec.box_xy, spec.box_yaw)
        d.qpos[adr:adr + 3] = (bx, by, CAB_H + 0.001)
        d.qpos[adr + 3:adr + 7] = _quat_yaw(yaw)
        # The remote, the phone, the glasses case and the pouch where the scene has them.
        for name, pose in (("remote", spec.remote), ("phone", spec.phone), ("case", spec.case), ("pouch", spec.pouch)):
            adr = self.free_qadr[name]
            d.qpos[adr:adr + 3] = pose[:3]
            d.qpos[adr + 3:adr + 7] = _quat_yaw(pose[3])
        self.start_z = float(spec.remote[2])
        mujoco.mj_forward(m, d)
        for _ in range(500):                       # 1 s for the loose things to settle
            mujoco.mj_step(m, d)
        self.steps = 0
        self.stopped = None
        self.progress = {"seen": False, "revealed": False, "retrieved": False}
        self._hold = 0
        self._reveal_run = 0
        self._pending = dict.fromkeys(CAMERAS, True)
        return self.observation()

    def _apply_look(self, spec):
        m = self.model

        def tint(name, rgb):
            m.mat_rgba[self.mat[name]] = tuple(rgb) + (1.0,)

        tint("sofa", spec.fabric_rgb)
        tint("pad", spec.pad_rgb)
        tint("sofa_plain", spec.fabric_rgb)
        tint("oak", spec.wood_rgb)
        tint("deck", _shade(spec.wood_rgb, 0.9))
        tint("hanger", spec.wood_rgb)
        tint("commode", spec.cabinet_rgb)
        tint("floor_a", spec.floor_rgb)
        tint("floor_b", spec.floor_rgb)
        tint("wall", spec.wall_rgb)
        m.geom_matid[self.floor_geom] = self.mat["floor_b" if spec.floor_kind else "floor_a"]
        m.light_pos[self.light] = (spec.lamp_xy[0], spec.lamp_xy[1], 2.6)
        m.light_diffuse[self.light] = tuple(c * spec.lamp_level for c in (0.72, 0.70, 0.67))

    # ------------------------------------------------------------------------------------------
    def _set_ctrl(self):
        d = self.data
        d.ctrl[0:8] = self.target[0:8]
        d.ctrl[8] = 0.5 * self.target[8]
        d.ctrl[9] = -0.5 * self.target[8]

    def clip_action(self, action):
        """The action as the simulator takes it: 9 clipped targets and the camera mask."""
        values = [float(v) for v in action]
        if len(values) != ACTION_SIZE or not all(math.isfinite(v) for v in values):
            raise ValueError("an action is {} finite numbers".format(ACTION_SIZE))
        goal = np.empty(9)
        goal[0] = min(max(values[0], RAIL_RANGE[0]), RAIL_RANGE[1])
        for i in range(7):
            goal[1 + i] = min(max(values[1 + i], ARM_LIMITS[i][0]), ARM_LIMITS[i][1])
        goal[8] = min(max(values[8], GRIP_RANGE[0]), GRIP_RANGE[1])
        mask = int(values[9])
        if mask != values[9] or not 0 <= mask <= 3:
            raise ValueError("the camera mask is an integer from 0 to 3")
        return goal, mask

    def step(self, action):
        """Advance one control step. Returns (observation, done)."""
        if self.stopped is not None:
            raise RuntimeError("the episode is over: " + self.stopped)
        goal, mask = self.clip_action(action)
        m, d = self.model, self.data
        speed = np.array(MAX_SPEED + (GRIP_SPEED,)) * SIM_DT
        for _ in range(SUBSTEPS):
            self.target += np.clip(goal - self.target, -speed, speed)
            self._set_ctrl()
            mujoco.mj_step(m, d)
        self.steps += 1
        self._pending = {name: bool(mask & CAMERAS[name][0]) for name in CAMERAS}
        if not np.all(np.isfinite(d.qpos)) or np.abs(d.qvel[self.arm_dadr[1:]]).max() > 8.0:
            self.stopped = "unstable"
            return self.observation(images=False), True
        self._judge()
        obs = self.observation()
        if self.progress["retrieved"]:
            self.stopped = "retrieved"
        elif self.steps >= MAX_STEPS:
            self.stopped = "time"
        return obs, self.stopped is not None

    # ------------------------------------------------------------------------------------------
    def gripper_opening(self):
        d = self.data
        return float(d.qpos[self.finger_qadr[0]] - d.qpos[self.finger_qadr[1]])

    def _wrist_pose(self):
        d = self.data
        cam = self.cam_id["wrist"]
        rot = d.cam_xmat[cam].reshape(3, 3) @ np.diag([1.0, -1.0, -1.0])     # OpenCV: x right, y down, z forward
        pose = np.eye(4)
        pose[:3, :3] = rot
        pose[:3, 3] = d.cam_xpos[cam]
        return pose

    def camera_pose(self, name):
        """4 x 4 pose of a camera in the world, OpenCV convention (x right, y down, z forward)."""
        d = self.data
        cam = self.cam_id[name]
        pose = np.eye(4)
        pose[:3, :3] = d.cam_xmat[cam].reshape(3, 3) @ np.diag([1.0, -1.0, -1.0])
        pose[:3, 3] = d.cam_xpos[cam]
        return pose

    def observation(self, images=True):
        m, d = self.model, self.data
        rng = self.rng
        q = np.array([d.qpos[a] for a in self.arm_qadr]) + rng.normal(0.0, 2e-4, 8)
        dq = np.array([d.qvel[a] for a in self.arm_dadr]) + rng.normal(0.0, 2e-3, 8)
        quat = np.empty(4)
        mujoco.mju_mat2Quat(quat, d.site_xmat[self.grip_site])
        rot = d.site_xmat[self.grip_site].reshape(3, 3)
        force = rot @ d.sensordata[0:3] + rng.normal(0.0, 0.15, 3)
        torque = rot @ d.sensordata[3:6] + rng.normal(0.0, 0.01, 3)
        obs = {
            "t": self.steps,
            "q": [round(float(v), 6) for v in q],
            "dq": [round(float(v), 5) for v in dq],
            "gripper": round(self.gripper_opening() + float(rng.normal(0.0, 1e-4)), 5),
            "ee_pos": [round(float(v), 5) for v in d.site_xpos[self.grip_site]],
            "ee_quat": [round(float(v), 6) for v in quat],
            "wrench": [round(float(v), 3) for v in np.concatenate([force, torque])],
            "T_world_wrist": [[round(float(v), 6) for v in row] for row in self._wrist_pose()],
            "images": {},
        }
        for name in CAMERAS:
            if images and self.render_enabled and self._pending.get(name):
                rgb, depth = self.render(name)
                obs["images"][name] = {"rgb": _encode(rgb), "depth": _encode(depth)}
                if not self.progress["seen"] and self._remote_pixels(name) >= SEEN_PIXELS:
                    self.progress["seen"] = True
        self._pending = dict.fromkeys(CAMERAS, False)
        return obs

    # ------------------------------------------------------------------------------------------
    def _renderer(self, name):
        if name not in self._renderers:
            _, width, height, _ = CAMERAS[name]
            self._renderers[name] = mujoco.Renderer(self.model, height, width)
        return self._renderers[name]

    def render(self, name, noise=True):
        """(rgb uint8 H x W x 3, depth uint16 H x W in millimetres along the optical axis; 0 = no
        reading beyond 6 m) of camera `name`."""
        renderer = self._renderer(name)
        renderer.disable_depth_rendering()
        renderer.disable_segmentation_rendering()
        renderer.update_scene(self.data, camera=self.cam_id[name], scene_option=self.scene_option)
        rgb = renderer.render().copy()
        renderer.enable_depth_rendering()
        renderer.update_scene(self.data, camera=self.cam_id[name], scene_option=self.scene_option)
        metres = renderer.render().copy()
        renderer.disable_depth_rendering()
        if noise:
            rgb = np.clip(rgb.astype(np.float32) + self.rng.normal(0.0, 1.5, rgb.shape), 0, 255).astype(np.uint8)
            metres = metres * (1.0 + self.rng.normal(0.0, 0.002, metres.shape))
        mm = np.where(metres * 1000.0 < DEPTH_MAX_MM, np.rint(metres * 1000.0), 0).astype(np.uint16)
        return rgb, mm

    def _remote_pixels(self, name):
        renderer = self._renderer(name)
        renderer.enable_segmentation_rendering()
        renderer.update_scene(self.data, camera=self.cam_id[name], scene_option=self.scene_option)
        seg = renderer.render()
        renderer.disable_segmentation_rendering()
        ids = seg[:, :, 0]
        types = seg[:, :, 1]
        remote = self.body["remote"]
        geoms = np.flatnonzero(self.model.geom_bodyid == remote)
        return int(np.count_nonzero((types == int(mujoco.mjtObj.mjOBJ_GEOM)) & np.isin(ids, geoms)))

    # ------------------------------------------------------------------------------------------
    def _contacts(self, body):
        """(set of geoms of other bodies touching `body`)."""
        m, d = self.model, self.data
        touching = set()
        for i in range(d.ncon):
            con = d.contact[i]
            if con.dist > 0.001:
                continue
            b1, b2 = m.geom_bodyid[con.geom1], m.geom_bodyid[con.geom2]
            if b1 == body and b2 != body:
                touching.add(int(con.geom2))
            elif b2 == body and b1 != body:
                touching.add(int(con.geom1))
        return touching

    def remote_exposed(self):
        """True when nothing but the robot is above the remote: three rays straight up from its
        top face, at its centre and near both ends, at least two of them free for 0.6 m."""
        m, d = self.model, self.data
        adr = self.free_qadr["remote"]
        pos = d.xpos[self.body["remote"]]
        rot = d.xmat[self.body["remote"]].reshape(3, 3)
        free = 0
        geomid = np.zeros(1, dtype=np.int32)
        for along in (-0.06, 0.0, 0.06):
            start = pos + rot @ np.array([along, 0.0, 0.0]) + np.array([0.0, 0.0, 0.012])
            hit = True
            origin = start.copy()
            for _ in range(8):
                dist = mujoco.mj_ray(m, d, origin, np.array([0.0, 0.0, 1.0]), self.solid, 1, self.body["remote"], geomid)
                if dist < 0 or (origin[2] + dist - start[2]) > 0.6:
                    hit = False
                    break
                gb = int(m.geom_bodyid[geomid[0]])
                if gb in self.robot_bodies:
                    origin = origin + np.array([0.0, 0.0, dist + 0.002])
                    continue
                break
            free += 0 if hit else 1
        del adr
        return free >= 2

    def _judge(self):
        m, d = self.model, self.data
        progress = self.progress
        # Laid open: nothing but the robot is above the remote.
        if not progress["revealed"]:
            self._reveal_run = self._reveal_run + 1 if self.remote_exposed() else 0
            if self._reveal_run >= 5:
                progress["revealed"] = True
        # Retrieved: the remote held by both finger pads, touching nothing else, lifted clear of
        # the surface it lay on at the start, with nothing but the robot above it, for HOLD_STEPS
        # in a row.
        touching = self._contacts(self.body["remote"])
        held = all(p in touching for p in self.pad_geoms)
        others = [g for g in touching if g not in self.finger_geoms]
        pos = d.xpos[self.body["remote"]]
        base = self.start_z - REMOTE_HALF[2]
        clear = pos[2] - base >= LIFT_CLEAR and self.remote_exposed()
        self._hold = self._hold + 1 if (held and not others and clear) else 0
        if self._hold >= HOLD_STEPS:
            progress["retrieved"] = True

    def score(self):
        """The episode's score from 0 to 1 as it stands."""
        p = self.progress
        if self.stopped == "unstable":
            return 0.0
        if p["retrieved"]:
            return 1.0
        if p["revealed"]:
            return PARTIAL_REVEALED
        return PARTIAL_SEEN if p["seen"] else 0.0

    def close(self):
        for renderer in self._renderers.values():
            renderer.close()
        self._renderers = {}

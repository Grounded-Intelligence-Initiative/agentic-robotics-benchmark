"""The live range: a hazardous-materials cart (cart_model.py: robosuite's Panda on its Omron
mobile base, a hook tool on the flange, a depth camera on a mast) that has to find two or three
carriers of hazardous samples (packages) in a furnished laboratory, move them to the store
across the yard and put each into the containment vessel of its class. The grader's; the agent
never has this file.

World frame: z up, metres; the floor is z = 0. The lab's inside is ROOM_X by ROOM_Y; its door
is a gap in the wall x = 0, centred on y = 0. The cart starts outside the door, facing in. The
store's inside is x in STORE_X, |y| <= STORE_HALF_Y; its door, in its wall x = STORE_X[1],
faces the lab's. Its four vessels stand along its side walls, each marked by a plate in the
colour of its class. A trolley stands in the yard between the doors, its handle (a CYAN bar)
toward the store: the cart may load it and tow it by the handle with the latch.

Along the lab's walls stand benches of drawer cabinets. A package is a case with a carrying
handle (an ORANGE bar) and a class label on its top; inside it are flasks (fixed to the case)
and a reagent that sloshes (a point mass on a short, lightly damped pendulum hung in the case).
Each lies in a closed drawer, on the floor beside or behind furniture, or on a bench's top.
Other drawers hold empty cases with handles of other colours: they are harmless. A drawer is
opened by its pull, a BRASS bar on its front, with the same latch that carries the packages. A
package is spilled, and the episode is lost for good, when
  * its reagent leans against the case by more than `tilt_limit_deg` for TILT_HOLD_S,
  * the low-passed acceleration of its centre of mass exceeds `shock_limit`, or
  * the countdown runs out before every package is secured.
A package is secured when it rests on the floor of the vessel of its class, that vessel's lid
is shut and the latch holds neither. The episode is won when every package is secured and the
lab is as it was: every drawer shut, every stool and chair where it stood, the latch empty. It
ends there; when the countdown runs out before that, it is lost, the lab put right included.

Hidden per episode (drawn from the seed, never in an observation): the furniture and where it
stands, how many packages there are, where each stands and its class, which vessel is of which
class, the harmless cases, each package's mass, centre of mass, reagent (its share of the mass,
its slosh frequency and damping), how freely it swings on the latch, the trolley's place,
heading, mass and rolling resistance, the surfaces' friction, the force/torque sensor's bias.
"""

import base64
import json
import math
import os
import zlib

import mujoco
import numpy as np

from arm_model import CONTROL_DT, PHYSICS_DT, SUBSTEPS, draw_bias, new_world  # noqa: F401
from cart_model import (  # noqa: F401
    ARM_MOUNT, BASE_BOX, BASE_HALF, CAM_MOUNT, Cart, add_box, add_cart, decode, encode, take_picture)

# --- the lab ------------------------------------------------------------------------------
ROOM_X, ROOM_Y = 7.0, 5.6                    # inside dimensions: x in [0, ROOM_X], |y| <= ROOM_Y / 2
DOOR_HALF = 0.65                             # the door: |y| < DOOR_HALF in the wall x = 0
WALL_H, WALL_T = 2.2, 0.10
CART_START = np.array([-1.6, 0.0, 0.0])      # x, y, yaw of the cart: outside the door, facing in

# --- the store ----------------------------------------------------------------------------
# The hazardous-materials store: a room across the yard from the lab's door. Its inside is
# x in STORE_X, |y| <= STORE_HALF_Y; its door is a gap |y| < STORE_DOOR_HALF in its wall
# x = STORE_X[1], facing the lab's door. Its containment vessels stand along its side walls,
# each marked by the colour of the class of package it takes.
STORE_X = (-10.4, -5.6)
STORE_HALF_Y = 2.2
STORE_DOOR_HALF = 0.9
STANDS = np.array([[-9.3, 1.97], [-7.6, 1.97], [-9.3, -1.97], [-7.6, -1.97]])   # the vessels' stands
STAND_HALF, STAND_H = 0.20, 0.30             # the stand: a square column, its top this high
CLASS_COLOURS = ([0.15, 0.30, 0.85, 1], [0.15, 0.65, 0.25, 1], [0.55, 0.20, 0.75, 1], [0.92, 0.92, 0.92, 1])
PLATE_HALF = (0.11, 0.003, 0.07)             # a stand's colour plate, on its face to the store's middle
PLATE_Z = 0.16
LABEL_HALF = (0.05, 0.035, 0.0015)           # a package's colour label, on its case's top

# --- the trolley --------------------------------------------------------------------------
# A wheeled trolley stands in the yard. Its frame: origin on the floor under its middle, x
# along its length; it rolls on four swivel casters (a planar body: its mass and how freely it
# rolls are drawn per episode and not reported). Its tray is an open box on top; its handle,
# a CYAN bar across its +x end, is what the latch takes. It stands in the yard between the two
# doors, its handle toward the store's door.
TROLLEY_HALF = (0.45, 0.28)
TRAY_TOP = 0.50                              # the tray floor's top face
RIM_H = 0.05                                 # the tray's rim, above its floor
TROLLEY_HANDLE = (0.53, 0.85, 0.20)          # the handle's axis: x in the trolley's frame, height, half length
CYAN = [0.10, 0.75, 0.85, 1]

# --- the package --------------------------------------------------------------------------
BOX_HALF = np.array([0.085, 0.055, 0.045])
BAR_Z = 0.085
BAR_HALF = 0.065
BAR_RADIUS = 0.006
SHELL_MASS = 0.5
LATCH_REACH = 0.006
LATCH_SOLREF = [0.006, 1]
ORANGE = [0.95, 0.45, 0.05, 1]
OTHER_HANDLES = ([0.1, 0.35, 0.9, 1], [0.1, 0.7, 0.2, 1], [0.75, 0.1, 0.7, 1])

# --- the vessel ---------------------------------------------------------------------------
VESSEL_INNER = 0.125
VESSEL_WALL = 0.012
VESSEL_HEIGHT = 0.20
VESSEL_FLOOR = 0.015
CLEAR_ABOVE_RIM = 0.03
# The vessel's lid: a plate hinged on the rim's edge at +x, standing open (leaning back over
# the stop) at the start; it is closed by its handle, a bar on its upper face.
LID_SIZE = 0.29                              # square, m
LID_THICK = 0.012
LID_OPEN = 1.85                              # rad: where it stands at the start, on its stop
LID_SHUT = 0.06                              # rad: below this it counts as closed
LID_MASS = 0.5
LID_HANDLE = np.array([0.25, 0.057])         # the handle's axis: along the plate from the hinge, and off its lower face
LID_HANDLE_HALF = 0.07
LID_RED = [0.85, 0.15, 0.15, 1]

# --- what a package is lost by --------------------------------------------------------------------------
TILT_HOLD_S = 0.06
TILT_LIMIT_DEG = 5.0                         # the same in every episode (the observation reports it)
SHOCK_LIMIT = 2.0                            # m/s^2, likewise
# The reagent sloshes: in effect a point mass (a share of the ballast) on a short pendulum hung
# in the case from a universal joint, its length set by the slosh frequency (L = g / w^2) and
# lightly damped. A package spills by "tilt" when the liquid's pendulum leans against the
# case's z axis by more than the limit: at rest that is the case's tilt from the vertical.
LIQUID_SHARE = (0.55, 0.80)                  # of the ballast's mass
SLOSH_HZ = (2.0, 3.5)
SLOSH_ZETA = (0.01, 0.03)
LIQUID_BOB = 0.005                           # m: the point mass's sphere (it collides with nothing)
GRAVITY = 9.81
SHOCK_TAU_S = 0.012
DRAWER_SHUT = 0.015                          # m: a drawer out by less than this counts as shut
MOVABLE = "chair"                            # the kit's stools and chairs are not fixed: they can be pushed
CHAIR_MASS = 7.0
IN_PLACE = (0.10, 0.30)                      # m, rad: a stool or chair this near where it stood is in its place

# --- the furniture ------------------------------------------------------------------------
# The shapes of the lab's furniture come from a kit of models (lab_assets/, with its
# manifest: every model's size, its parts and their colours, the surfaces things can stand
# on); what collides is each piece's bounding box. A piece's own frame: origin on the floor
# under the middle of its bounding box, its back on -y, its front on +y.

# --- the drawer cabinets ------------------------------------------------------------------
# A cabinet's own frame: origin on the floor under its middle, its back on -y (against a
# wall), its front on +y. It has one drawer, under its top, which slides out along +y.
CAB_DEPTH = 0.52
CAB_PANEL = 0.018                            # carcass panels and the drawer's front
DRAWER_PANEL = 0.012                         # the drawer's sides, back and floor
DRAWER_HEIGHT = 0.20                         # the drawer's front, top to bottom
DRAWER_TRAVEL = 0.36                         # how far it slides out, m
DRAWER_FRICTION = 3.0                        # N, dry
DRAWER_DAMPING = 15.0                        # N s/m
DRAWER_MASS = 2.5
PULL_HALF = 0.08                             # the pull: a bar of this half length and
PULL_RADIUS = 0.006                          # radius along the cabinet's x,
PULL_STANDOFF = 0.040                        # its axis this far out from the drawer's front,
PULL_DROP = 0.07                             # this far below the drawer front's top edge
BRASS = [0.90, 0.75, 0.10, 1]


def drawer_frame(top):
    """Heights of a cabinet's drawer, from the height of the cabinet's top: the drawer
    front's top edge, the drawer's inner floor, and the pull's axis."""
    front_top = top - CAB_PANEL - 0.005
    floor = front_top - DRAWER_HEIGHT + DRAWER_PANEL
    return front_top, floor, front_top - PULL_DROP


def _asset_dir():
    here = os.path.dirname(os.path.abspath(__file__))
    for d in (os.environ.get("HAZMAT_LAB_ASSETS", ""), "/opt/lab_assets",
              os.path.join(here, "lab_assets"),
              os.path.join(here, "..", "environment", "lab_assets")):
        if d and os.path.isfile(os.path.join(d, "manifest.json")):
            return d
    return None


_KIT = None


def kit():
    """The furniture kit: {name: dict(kind, size, top, surface, parts)}, and under "" its
    surface textures."""
    global _KIT
    if _KIT is None:
        d = _asset_dir()
        if d is None:
            raise RuntimeError("the furniture kit (lab_assets/manifest.json) is not installed")
        with open(os.path.join(d, "manifest.json")) as f:
            manifest = json.load(f)
        _KIT = {"": {k: [dict(file=os.path.join(d, t["file"]), tile=float(t["tile_m"][0]), rgba=t["rgba"][:3])
                         for t in v] for k, v in manifest["textures"].items()}}
        for k, m in manifest["models"].items():
            tops = sorted(m.get("surfaces", []), key=lambda q: -(q["x"][1] - q["x"][0]) * (q["y"][1] - q["y"][0]))
            _KIT[k] = dict(kind=m["kind"], size=[float(v) for v in m["size"]],
                           top=float(tops[0]["z"]) if tops else None, surface=tops[0] if tops else None,
                           parts=[dict(file=os.path.join(d, q["file"]), texture=os.path.join(d, q["texture"]),
                                       rgba=q["rgba"][:3], clear=bool(q.get("transparent"))) for q in m["parts"]])
    return _KIT


def kinds():
    out = {}
    for k, v in kit().items():
        if k:
            out.setdefault(v["kind"], []).append(k)
    return {k: sorted(v) for k, v in out.items()}


# ------------------------------------------------------------------------------------------
# the episode's draw
# ------------------------------------------------------------------------------------------
GRID = 0.05                                  # the cell of the generator's own floor plan
REACH_RADIUS = 0.62                          # the disc a cart has to get through with, m
STAND_OFF = 0.70 + ARM_MOUNT[0]              # cart's centre to a closed drawer's pull, to open it
TOP_MAX = 0.81                               # a top higher than this is beyond the arm's reach for the handle
TOP_SHARE = 0.25                             # how often the package stands on a desk or a table,
FLOOR_SHARE = 0.30                           # and how often on the floor (else: in a drawer)


def _half(piece):
    """Half extents in the world of a piece's bounding box, turned by its yaw."""
    sx, sy = piece["size"][0] / 2.0, piece["size"][1] / 2.0
    c, s = abs(math.cos(piece["yaw"])), abs(math.sin(piece["yaw"]))
    return np.array([c * sx + s * sy, s * sx + c * sy])


def _overlap(p, half, pieces, gap):
    for q in pieces:
        h = _half(q)
        if abs(p[0] - q["xy"][0]) < half[0] + h[0] + gap and abs(p[1] - q["xy"][1]) < half[1] + h[1] + gap:
            return True
    return False


def _local(piece, v):
    """A point of a piece's own frame in the world."""
    c, s = math.cos(piece["yaw"]), math.sin(piece["yaw"])
    return np.array([piece["xy"][0] + c * v[0] - s * v[1], piece["xy"][1] + s * v[0] + c * v[1]])


def _reachable(pieces):
    """A test: can a disc of REACH_RADIUS get from the cart's start to a point?"""
    ox, oy = -3.2, -(ROOM_Y + 1.0) / 2.0
    nx, ny = int(round((ROOM_X - ox + 0.5) / GRID)), int(round((ROOM_Y + 1.0) / GRID))
    X, Y = np.meshgrid(ox + (np.arange(nx) + 0.5) * GRID, oy + (np.arange(ny) + 0.5) * GRID, indexing="ij")
    r = REACH_RADIUS
    blocked = (X > -WALL_T - r) & (((np.abs(Y) > DOOR_HALF - r) & (X < r)) | (X > ROOM_X - r)
                                   | (np.abs(Y) > ROOM_Y / 2.0 - r))
    for sx, sy in STANDS:
        blocked |= (np.abs(X - sx) < STAND_HALF + r) & (np.abs(Y - sy) < STAND_HALF + r)
    for q in pieces:
        h = _half(q)
        blocked |= (np.abs(X - q["xy"][0]) < h[0] + r) & (np.abs(Y - q["xy"][1]) < h[1] + r)
    seen = np.zeros_like(blocked)
    i0, j0 = int((CART_START[0] - ox) / GRID), int((CART_START[1] - oy) / GRID)
    stack = [(i0, j0)]
    seen[i0, j0] = True
    while stack:
        i, j = stack.pop()
        for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            u, v = i + di, j + dj
            if 0 <= u < nx and 0 <= v < ny and not seen[u, v] and not blocked[u, v]:
                seen[u, v] = True
                stack.append((u, v))

    def at(p):
        i, j = int((p[0] - ox) / GRID), int((p[1] - oy) / GRID)
        return bool(0 <= i < nx and 0 <= j < ny and seen[i, j])
    return at


def _corners(xy, yaw, half):
    c, s = math.cos(yaw), math.sin(yaw)
    return [np.array([xy[0] + c * a - s * b, xy[1] + s * a + c * b])
            for a, b in ((half[0], half[1]), (-half[0], half[1]), (-half[0], -half[1]), (half[0], -half[1]))]


def _rects_meet(xy1, yaw1, half1, xy2, yaw2, half2):
    """Do two turned rectangles overlap? (Separating axes.)"""
    A, B = _corners(xy1, yaw1, half1), _corners(xy2, yaw2, half2)
    for yaw in (yaw1, yaw1 + math.pi / 2.0, yaw2, yaw2 + math.pi / 2.0):
        axis = np.array([math.cos(yaw), math.sin(yaw)])
        a, b = [float(p @ axis) for p in A], [float(p @ axis) for p in B]
        if max(a) < min(b) or max(b) < min(a):
            return False
    return True


THING_SCALE = {"classic_laptop": 0.6}


def _on_a_top(rng, pieces, things, at):
    """A place for the package on a desk or a table: near an edge a cart can stand at, with
    nothing else on the piece within 18 cm of its bounding box's corners or between it and that
    edge. None if there is none."""
    K = kit()
    cart_half = np.array([BASE_HALF[0] + 0.08, BASE_HALF[1] + 0.08])
    for k in rng.permutation(len(pieces)):
        t = pieces[int(k)]
        about = K.get(t["kind"])
        if about is None or about["kind"] not in ("desk", "low_table") or about["top"] is None or about["top"] > TOP_MAX:
            continue
        top = about["surface"]                   # taken as centred: the kit's tops are, within a few cm
        hx = min(-top["x"][0], top["x"][1]) - 0.01
        hy = min(-top["y"][0], top["y"][1]) - 0.01
        for _ in range(30):
            edge = int(rng.integers(0, 4))       # 0: front (+y), 1: -x, 2: +x, 3: back (-y)
            inset = float(rng.uniform(0.16, 0.22))
            if edge in (0, 3):
                n = np.array([0.0, 1.0 if edge == 0 else -1.0])
                v = np.array([rng.uniform(-hx + 0.16, hx - 0.16) if hx > 0.17 else 0.0, n[1] * (hy - inset)])
            else:
                n = np.array([1.0 if edge == 2 else -1.0, 0.0])
                v = np.array([n[0] * (hx - inset), rng.uniform(-hy + 0.16, hy - 0.16) if hy > 0.17 else 0.0])
            if min(hx - abs(v[0]), hy - abs(v[1])) < 0.15:
                continue
            xy = _local(t, v)
            out = _local(t, v + n) - xy                         # the edge's outward normal, in the world
            # nothing else on the piece within 32 cm, or between the package and the edge
            clear = True
            for o in things:
                rel = np.asarray(o["xy"]) - xy
                if np.linalg.norm(rel) < 0.18 + o["reach"] or (rel @ out > 0.0 and np.linalg.norm(rel) < 0.9
                                                             and abs(rel @ np.array([-out[1], out[0]])) < 0.16 + o["reach"]):
                    clear = False
            if not clear:
                continue
            # the cart's place: its front 10 cm off the edge, facing the package; and the way to it
            stand = xy + out * (inset + 0.10 + BASE_BOX[0] + BASE_HALF[0])
            yaw = math.atan2(-out[1], -out[0])
            if not at(stand + out * 0.6):
                continue
            swept = np.array([cart_half[0] + 0.3, cart_half[1]])
            centre = stand + out * 0.3 - out * BASE_BOX[0]
            if any(_rects_meet(centre, yaw, swept, q["xy"], q["yaw"], np.array(q["size"][:2]) / 2.0) for q in pieces):
                continue
            if any(c[0] < 0.05 or c[0] > ROOM_X - 0.05 or abs(c[1]) > ROOM_Y / 2.0 - 0.05
                   for c in _corners(centre, yaw, swept)):
                continue                          # not through a wall
            return dict(xy=[float(xy[0]), float(xy[1])], z=float(K[t["kind"]]["top"]),
                        yaw=float(rng.uniform(-math.pi, math.pi)), cabinet=None, on=t["kind"])
    return None


def _on_the_floor(rng, pieces, keep_clear, at):
    """A place for the package on the floor: beside or behind a piece of furniture, out of
    the cart's ways to the drawers, with room for the cart to come up to it squarely from
    some side. None if there is none."""
    cart_half = np.array([BASE_HALF[0] + 0.08, BASE_HALF[1] + 0.08])
    case = np.array([0.11, 0.11])
    for _ in range(80):
        t = pieces[int(rng.integers(0, len(pieces)))]
        h = _half(t)
        side = int(rng.integers(0, 4))
        gap = float(rng.uniform(0.32, 0.50))
        if side < 2:
            xy = np.array([t["xy"][0] + (1 if side == 0 else -1) * (h[0] + gap), t["xy"][1] + rng.uniform(-h[1], h[1])])
        else:
            xy = np.array([t["xy"][0] + rng.uniform(-h[0], h[0]), t["xy"][1] + (1 if side == 2 else -1) * (h[1] + gap)])
        if xy[0] < 0.35 or xy[0] > ROOM_X - 0.35 or abs(xy[1]) > ROOM_Y / 2.0 - 0.35:
            continue
        if _overlap(xy, case, pieces, 0.20) or _overlap(xy, case, keep_clear, 0.0):
            continue
        for k in rng.permutation(12):
            yaw = float(k) * math.pi / 6.0
            u = np.array([math.cos(yaw), math.sin(yaw)])
            stand = xy - u * (ARM_MOUNT[0] + 0.56)
            if not at(stand - u * 0.6):
                continue
            swept = np.array([cart_half[0] + 0.3, cart_half[1]])
            centre = stand - u * 0.3 + u * BASE_BOX[0]
            if any(_rects_meet(centre, yaw, swept, q["xy"], q["yaw"], np.array(q["size"][:2]) / 2.0) for q in pieces):
                continue
            if any(c[0] < 0.05 or c[0] > ROOM_X - 0.05 or abs(c[1]) > ROOM_Y / 2.0 - 0.05
                   for c in _corners(centre, yaw, swept)):
                continue
            return dict(xy=[float(xy[0]), float(xy[1])], z=0.0, yaw=float(rng.uniform(-math.pi, math.pi)),
                        cabinet=None, on="floor")
    return None


WALLS = {"right": 0.0, "far": math.pi / 2.0, "left": math.pi, "near": 3.0 * math.pi / 2.0}


def _layout(rng):
    """The furniture of one lab: (pieces, cabinets, things, rugs, keep_clear), or None when the
    draw did not fit. A lab as people leave one: benches along the walls (the drawer cabinets,
    one to three side by side under one run of worktop) with instruments and bottles at the
    back, a bench or two in the room with stools pushed back from them, steel shelving, a tool
    cart, gas bottles, an extinguisher, boxes and crates where there was room, nothing quite
    square to anything."""
    K = kit()
    pieces, keep_clear, things, rugs = [], [], [], []
    # nothing stands in the doorway
    keep_clear.append(dict(xy=np.array([0.75, 0.0]), yaw=0.0, size=[1.5, 2.2]))

    def on_wall(wall, size, skew=0.03):
        """Centre and yaw of a piece of this size standing with its back to a wall."""
        along, depth = size[0] / 2.0, size[1] / 2.0
        yaw = WALLS[wall] + rng.uniform(-skew, skew)
        off = depth + 0.03 + along * abs(math.sin(yaw - WALLS[wall]))
        if wall == "right":
            return np.array([rng.uniform(0.3 + along, ROOM_X - 0.3 - along), -ROOM_Y / 2.0 + off]), yaw
        if wall == "left":
            return np.array([rng.uniform(0.3 + along, ROOM_X - 0.3 - along), ROOM_Y / 2.0 - off]), yaw
        if wall == "far":
            return np.array([ROOM_X - off, rng.uniform(-ROOM_Y / 2.0 + 0.3 + along, ROOM_Y / 2.0 - 0.3 - along)]), yaw
        lo, hi = DOOR_HALF + 0.45 + along, ROOM_Y / 2.0 - 0.3 - along           # beside the door
        if hi <= lo:
            return None, yaw
        return np.array([off, rng.uniform(lo, hi) * rng.choice([-1.0, 1.0])]), yaw

    def fits(piece, gap, others=None):
        h = _half(piece)
        p = piece["xy"]
        if p[0] - h[0] < 0.03 or p[0] + h[0] > ROOM_X - 0.03 or abs(p[1]) + h[1] > ROOM_Y / 2.0 - 0.03:
            return False
        return not _overlap(p, h, pieces if others is None else others, gap) and not _overlap(p, h, keep_clear, 0.0)

    def add(kind, xy, yaw, gap=0.06, size=None, **more):
        """A piece at a given place, if it fits there."""
        piece = dict(kind=kind, xy=np.array(xy, float), yaw=float(yaw),
                     size=list(size if size is not None else K[kind]["size"]), **more)
        if not fits(piece, gap):
            return None
        pieces.append(piece)
        return piece

    def put(kind, walls, gap=0.08, tries=60, frontage=0.0, size=None, skew=0.03, **more):
        """A piece somewhere along one of the walls, or free in the room."""
        size = list(size if size is not None else K[kind]["size"])
        for _ in range(tries):
            wall = str(rng.choice(walls))
            if wall == "free":
                yaw = float(rng.integers(0, 4)) * math.pi / 2.0 + rng.uniform(-0.25, 0.25)
                p = np.array([rng.uniform(1.6, ROOM_X - 0.8), rng.uniform(-ROOM_Y / 2.0 + 0.8, ROOM_Y / 2.0 - 0.8)])
            else:
                p, yaw = on_wall(wall, size, skew)
                if p is None:
                    continue
            piece = dict(kind=kind, xy=p, yaw=float(yaw), size=size, **more)
            if not fits(piece, gap):
                continue
            if frontage:                         # the floor in front of it stays free
                zone = dict(xy=_local(piece, [0.0, size[1] / 2.0 + frontage / 2.0]), yaw=piece["yaw"],
                            size=[size[0] + 0.7, frontage])
                if _overlap(zone["xy"], _half(zone), pieces, 0.0):
                    continue
                keep_clear.append(zone)
            pieces.append(piece)
            return piece
        return None

    KIND = kinds()

    def pick(kind):
        return str(rng.choice(KIND[kind]))

    def on_top(parent, kind, v, yaw, z):
        xy = _local(parent, v)
        scale = THING_SCALE.get(kind, 1.0)
        things.append(dict(kind=kind, xy=[float(xy[0]), float(xy[1])], z=float(z), yaw=float(parent["yaw"] + yaw),
                           scale=scale, reach=float(scale * math.hypot(K[kind]["size"][0], K[kind]["size"][1]) / 2.0)))

    def dress(t, count, first=None):
        """Leave things on a piece's top, apart from one another; `first` goes at the back,
        nearly square to the piece."""
        top = K[t["kind"]]["surface"]
        if top is None:
            return
        (x0, x1), (y0, y1) = top["x"], top["y"]
        placed = []
        names = ([first] if first else []) + [str(k) for k in rng.permutation(KIND["thing"]) if k != first]
        for kind in names[:count]:
            scale = THING_SCALE.get(kind, 1.0)
            reach = scale * math.hypot(K[kind]["size"][0], K[kind]["size"][1]) / 2.0
            ex = ey = reach
            if kind == first:                    # square to the piece: its own extents, and a margin for the turn
                ex, ey = scale * K[kind]["size"][0] / 2.0 + 0.04, scale * K[kind]["size"][1] / 2.0 + 0.04
            if x1 - x0 < 2 * ex + 0.04 or y1 - y0 < 2 * ey + 0.04:
                continue
            for _ in range(15):
                v = np.array([rng.uniform(x0 + ex + 0.02, x1 - ex - 0.02),
                              rng.uniform(y0 + ey + 0.02, y1 - ey - 0.02)])
                if kind == first:
                    v[1] = y0 + ey + 0.01
                if all(np.linalg.norm(v - q) > reach + r + 0.05 for q, r in placed):
                    placed.append((v, reach))
                    on_top(t, kind, v, rng.uniform(-0.1, 0.1) if kind == first else rng.uniform(0, 6.28), top["z"])
                    break

    def work_place(d):
        """What belongs to a bench in the room: a stool pushed back from it, a waste canister
        beside some, and what was left on it."""
        stool = pick("chair")
        depth = K[stool]["size"][1]
        add(stool, _local(d, [rng.uniform(-0.25, 0.25), d["size"][1] / 2.0 + depth / 2.0 + rng.uniform(0.05, 0.35)]),
            d["yaw"] + math.pi + rng.uniform(-0.7, 0.7), gap=0.02)
        if rng.random() < 0.5:
            side = rng.choice([-1.0, 1.0])
            kind = pick("bin")
            add(kind, _local(d, [side * (d["size"][0] / 2.0 + 0.25), rng.uniform(-0.2, 0.3)]),
                rng.uniform(0, 6.28), gap=0.03)
        dress(d, int(rng.integers(2, 5)),
              first=str(rng.choice(["chemistry_set", "industrial_microscope", "vintage_microscope", "classic_laptop"])))

    BENCH_THINGS = ["bunsen_burner", "vintage_microscope", "industrial_microscope", "bleach_bottle", "cleaner_bottle",
                    "all_purpose_cleaner", "plastic_bottle_gallon", "first_aid_box", "alarm_clock_01",
                    "book_encyclopedia_set_01", "metal_toolbox", "clipboard", "circuit_board", "magnifying_glass_01"]

    # the middle of the room: one long bench, or two tables pushed together with stools on
    # either side
    if rng.random() < 0.85:
        yaw = float(rng.choice([0.0, 0.0, 0.0, math.pi / 2.0])) + rng.uniform(-0.15, 0.15)
        mid = np.array([rng.uniform(2.6, 3.8), rng.uniform(-0.4, 0.4)])
        if rng.random() < 0.45:
            shape = [("metal_office_desk", 0.0, 0.0, 0.0)]
        else:
            depth = K["wooden_table_02"]["size"][1]
            shape = [("wooden_table_02", 0.0, -depth / 2.0 - 0.01, math.pi), ("wooden_table_02", 0.0, depth / 2.0 + 0.01, 0.0)]
        desks = []
        for kind, vx, vy, turn in shape:
            c, sn = math.cos(yaw), math.sin(yaw)
            d = add(kind, mid + np.array([c * vx - sn * vy, sn * vx + c * vy]), yaw + turn + rng.uniform(-0.04, 0.04), gap=0.0)
            if d is not None:
                desks.append(d)
        for d in desks:
            work_place(d)

    # the benches along the walls: runs of one to three drawer cabinets side by side, square to
    # the wall, the floor before each run kept free
    cabinets = []
    wanted = int(rng.integers(4, 6))
    while len(cabinets) < wanted:
        k = min(int(rng.choice([1, 2, 2, 3])), wanted - len(cabinets))
        widths = [float(rng.uniform(0.62, 0.90)) for _ in range(k)]
        height = float(rng.uniform(0.72, 0.80))
        run = put("cabinet", ["far", "left", "right", "near"], gap=0.20, frontage=1.75, skew=0.0, tries=40,
                  size=[sum(widths) + 0.004 * (k - 1), CAB_DEPTH, height])
        if run is None:
            if k == 1:
                return None
            continue                             # a shorter run, then
        pieces.remove(run)
        x = -run["size"][0] / 2.0
        for wdt in widths:
            c = dict(kind="cabinet", xy=_local(run, [x + wdt / 2.0, 0.0]), yaw=run["yaw"], size=[wdt, CAB_DEPTH, height])
            x += wdt + 0.004
            c["turn"] = int(round(c["yaw"] / (math.pi / 2.0))) % 4
            pieces.append(c)
            cabinets.append(c)
        # what was left at the back of the run's worktop: square to it (its own extents) or
        # turned anyhow (its circle)
        width, spots = run["size"][0], []
        for _ in range(int(rng.integers(1, 2 + k))):
            kind = str(rng.choice(BENCH_THINGS + (["chemistry_set"] * 2 if width > 1.25 else [])))
            square = kind in ("chemistry_set", "book_encyclopedia_set_01", "first_aid_box") or rng.random() < 0.3
            ex = K[kind]["size"][0] / 2.0 if square else math.hypot(K[kind]["size"][0], K[kind]["size"][1]) / 2.0
            ey = K[kind]["size"][1] / 2.0 if square else ex
            if 2 * ex + 0.04 > width or 2 * ey + 0.06 > CAB_DEPTH:
                continue
            v = np.array([rng.uniform(-width / 2.0 + ex + 0.02, width / 2.0 - ex - 0.02), -CAB_DEPTH / 2.0 + ey + 0.03])
            if all(abs(v[0] - u) > ex + q + 0.04 for u, q in spots):
                spots.append((v[0], ex))
                on_top(run, kind, v, rng.uniform(-0.05, 0.05) if square else rng.uniform(0, 6.28), height)

    # more benches, mostly along the walls
    for _ in range(int(rng.integers(2, 4))):
        d = put(pick("desk"), ["free", "far", "far", "left", "right"], gap=0.10, skew=0.06, tries=40)
        if d is not None:
            work_place(d)

    # steel shelving along the walls, some side by side; a tool cart or two with things on it
    for _ in range(int(rng.integers(2, 5))):
        b = put(pick("shelves"), ["far", "left", "right", "near"], gap=0.06, tries=30, skew=0.02)
        if b is not None and rng.random() < 0.4:
            kind = pick("shelves")
            add(kind, _local(b, [rng.choice([-1.0, 1.0]) * (b["size"][0] / 2.0 + K[kind]["size"][0] / 2.0 + 0.03), 0.0]),
                b["yaw"], gap=0.0)
    for _ in range(int(rng.integers(0, 3))):
        t = put(pick("sideboard"), ["far", "left", "right", "near", "free"], gap=0.08, tries=30, skew=0.15)
        if t is not None:
            dress(t, 2)

    # what stands along the walls: gas bottles, an extinguisher, a hand truck, a tool chest,
    # a sign; and boxes, crates and a stray stool where there was room
    for _ in range(int(rng.integers(3, 6))):
        put(pick("standing"), ["far", "left", "right", "near"], gap=0.05, tries=25, skew=0.6)
    for _ in range(int(rng.integers(4, 8))):
        kind = str(rng.choice(KIND["box"] + KIND["box"] + KIND["bin"] + KIND["chair"][:1]))
        box = K[kind]["kind"] == "box"
        put(kind, ["far", "left", "right", "near", "far", "left", "right", "near", "free"] if box
            else ["far", "left", "right", "near"], gap=0.05, tries=25, skew=0.5 if box else 3.14)
    return pieces, cabinets, things, rugs, keep_clear


def draw_episode(seed):
    """Everything an episode's seed decides. `hidden` never reaches an observation."""
    rng = np.random.default_rng(int(seed))
    while True:
        drawn = _layout(rng)
        if drawn is None:
            continue
        pieces, cabinets, things, rugs, keep_clear = drawn
        at = _reachable(pieces)
        # a cart must be able to stand before every drawer, and to back away from it
        front = CAB_DEPTH / 2.0 + PULL_STANDOFF
        if all(at(_local(c, [0.0, front + STAND_OFF])) and at(_local(c, [0.0, front + STAND_OFF + 0.5]))
               for c in cabinets):
            break
    # two or three packages, each in a drawer, on the floor or on a top, apart from one
    # another; harmless cases in one or two of the other drawers
    order = [int(k) for k in rng.permutation(len(cabinets))]
    count = int(rng.integers(2, 4))
    cases, holds, spots = {}, {}, []
    for i in range(count):
        where = rng.random()
        spot = None
        for _ in range(4):
            if where < TOP_SHARE:
                spot = _on_a_top(rng, pieces, things, at)
            elif where < TOP_SHARE + FLOOR_SHARE:
                spot = _on_the_floor(rng, pieces, keep_clear, at)
            if spot is None or all(np.linalg.norm(np.array(spot["xy"]) - np.array(q["xy"])) > 1.2 for q in spots):
                break
            spot = None
        if spot is not None:
            spots.append(spot)
            cases["package{}".format(i)] = spot
        else:
            holds[order.pop(0)] = "package{}".format(i)
    for n, k in enumerate(order[:int(rng.integers(1, 3))]):
        holds[k] = "other{}".format(n)
    for k, c in enumerate(cabinets):
        c["holds"] = holds.get(k)
        if c["holds"]:
            inner = c["size"][0] - 2 * CAB_PANEL - 0.01 - 2 * DRAWER_PANEL
            v = [float(rng.uniform(-(inner / 2.0 - 0.13), inner / 2.0 - 0.13)),
                 CAB_DEPTH / 2.0 - CAB_PANEL - float(rng.uniform(0.12, 0.20))]
            xy = _local(c, v)
            cases[c["holds"]] = dict(xy=[float(xy[0]), float(xy[1])], z=drawer_frame(c["size"][2])[1],
                                     yaw=float(rng.uniform(-math.pi, math.pi)), cabinet=k)
    packages = []
    for i in range(count):
        cx = rng.uniform(0.012, 0.050) * rng.choice([-1.0, 1.0])
        packages.append(dict(cases["package{}".format(i)], ballast_mass=float(rng.uniform(0.6, 2.6)),
                             ballast_pos=[float(cx), float(rng.uniform(-0.002, 0.002)), float(rng.uniform(-0.030, 0.012))],
                             swing_damping=float(rng.uniform(0.0005, 0.004)),
                             liquid_share=float(rng.uniform(*LIQUID_SHARE)),
                             slosh_hz=float(rng.uniform(*SLOSH_HZ)),
                             slosh_zeta=float(rng.uniform(*SLOSH_ZETA))))
    # the packages' classes (one colour each, all different), the vessels' classes, the trolley
    classes = [int(c) for c in rng.permutation(len(CLASS_COLOURS))[:count]]
    for o, c in zip(packages, classes):
        o["cls"] = c
    vessel_class = [int(c) for c in rng.permutation(len(CLASS_COLOURS))]
    trolley = dict(xy=[float(rng.uniform(-3.7, -3.1)), float(rng.uniform(-0.5, 0.5))],
                   yaw=float(math.pi + rng.uniform(-0.3, 0.3)), mass=float(rng.uniform(15.0, 25.0)),
                   roll=float(rng.uniform(10.0, 30.0)), turn=float(rng.uniform(2.0, 8.0)))
    rng.choice([6.0, 8.0, 10.0]), rng.choice([3.0, 4.5, 6.0]), rng.choice([0.0, 30.0, 60.0])   # drawn and not used: a seed keeps its lab
    return dict(
        tilt_limit_deg=TILT_LIMIT_DEG,
        shock_limit=SHOCK_LIMIT,
        countdown_s=float(600.0 + 300.0 * count),            # 1200 s with two packages, 1500 s with three
        hidden=dict(
            furniture=[dict(kind=q["kind"], xy=[float(v) for v in q["xy"]], yaw=float(q["yaw"]),
                            size=[float(v) for v in q["size"]], scale=float(q.get("scale", 1.0)))
                       for q in pieces if q["kind"] != "cabinet"],
            style=dict(floor=int(rng.integers(0, 2)), wall=int(rng.integers(0, 2)), casework=int(rng.integers(0, 2)),
                       worktop=int(rng.integers(0, 2))),
            cabinets=[dict(xy=[float(v) for v in c["xy"]], turn=int(c["turn"]),
                           size=[float(v) for v in c["size"]], holds=c["holds"])
                      for c in cabinets],
            packages=packages,
            vessel_class=vessel_class,
            trolley=trolley,
            others=[cases[k] for k in sorted(cases) if k.startswith("other")],
            things=things,
            ft_bias=draw_bias(rng),
            surface_friction=float(rng.uniform(0.6, 1.0)),
            noise_seed=int(rng.integers(0, 2 ** 31)),
        ),
    )


# ------------------------------------------------------------------------------------------
# the model
# ------------------------------------------------------------------------------------------
def _case(wb, name, xy, z, yaw, handle_rgba, ballast_mass, ballast_pos, liquid=None, label=None):
    """A case with a handle bar on two posts, standing on a surface at height z. A contact
    takes the larger of its two geoms' frictions, so the surface's (and the vessel's) decides.
    With `liquid` (share, slosh_hz, slosh_zeta) that share of the ballast is the reagent: a
    point mass on a damped pendulum hung in the case from a universal joint, at rest at the
    ballast's position; the rest of the ballast (the flasks) is fixed to the case."""
    pk = wb.add_body(name=name, pos=[xy[0], xy[1], z + BOX_HALF[2] + 0.0005],
                     quat=[math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)])
    pk.add_freejoint(name=name)
    add_box(pk, name + "_case", BOX_HALF, [0, 0, 0], [0.35, 0.3, 0.22, 1], 1, 6, mass=SHELL_MASS,
         friction=0.3)
    bar = pk.add_geom(name=name + "_bar", type=mujoco.mjtGeom.mjGEOM_CAPSULE,
                      fromto=[-BAR_HALF, 0, BAR_Z, BAR_HALF, 0, BAR_Z], size=[BAR_RADIUS, 0, 0],
                      rgba=list(handle_rgba))
    bar.mass, bar.contype, bar.conaffinity, bar.group = 0.02, 0, 0, 1
    for k, sx in enumerate((-BAR_HALF, BAR_HALF)):
        add_box(pk, "{}_post{}".format(name, k), [0.004, 0.004, (BAR_Z - BOX_HALF[2]) / 2],
             [sx, 0, (BOX_HALF[2] + BAR_Z) / 2], [0.1, 0.1, 0.1, 1], 0, 0, mass=0.005)
    fixed = float(ballast_mass) * (1.0 - (liquid[0] if liquid else 0.0))
    ballast = pk.add_geom(name=name + "_ballast", type=mujoco.mjtGeom.mjGEOM_SPHERE,
                          size=[0.012, 0, 0], pos=[float(v) for v in ballast_pos], rgba=[0, 0, 0, 0])
    ballast.mass, ballast.contype, ballast.conaffinity, ballast.group = fixed, 0, 0, 2
    if liquid:
        share, hz, zeta = liquid
        mass = float(ballast_mass) * share
        w = 2.0 * math.pi * hz
        length = GRAVITY / (w * w)
        lq = pk.add_body(name=name + "_liquid", pos=[float(ballast_pos[0]), float(ballast_pos[1]),
                                                       float(ballast_pos[2]) + length])
        for axis, jn in (([1, 0, 0], "x"), ([0, 1, 0], "y")):
            j = lq.add_joint(name="{}_slosh_{}".format(name, jn), type=mujoco.mjtJoint.mjJNT_HINGE, axis=axis)
            j.damping = [2.0 * zeta * mass * length * length * w, 0.0, 0.0]
        bob = lq.add_geom(name=name + "_reagent", type=mujoco.mjtGeom.mjGEOM_SPHERE, size=[LIQUID_BOB, 0, 0],
                          pos=[0.0, 0.0, -length], rgba=[0, 0, 0, 0])
        bob.mass, bob.contype, bob.conaffinity, bob.group = mass, 0, 0, 2
    if label is not None:                         # its class's colour, on the case's top
        add_box(pk, name + "_label", LABEL_HALF, [0, 0, BOX_HALF[2] + LABEL_HALF[2]], list(label), 0, 0, mass=0.001)
    pk.add_site(name=name + "_grip", pos=[0, 0, BAR_Z], size=[0.004, 0, 0])
    return pk


def _surface(w, name, about, colours):
    """A material carrying one of the kit's surface textures (floor, wall, casework, worktop)."""
    w.add_texture(name=name, type=mujoco.mjtTexture.mjTEXTURE_2D, file=about["file"])
    mat = w.add_material(name=name)
    mat.textures[mujoco.mjtTextureRole.mjTEXROLE_RGB] = name
    mat.texuniform = True
    mat.texrepeat = [1.0 / about["tile"], 1.0 / about["tile"]]
    colours[name] = list(about["rgba"])


def _paint(g, material, colours, tint=1.0):
    """Give a box a surface material. In a picture it shows in that texture's mean colour
    (times `tint`), which is what `colours` records for it."""
    g.material = material
    mean = colours[material]
    if not isinstance(tint, float) or tint != 1.0:
        tint = [tint] * 3 if isinstance(tint, float) else list(tint)
        g.rgba = [tint[0], tint[1], tint[2], 1.0]
        colours[g.name] = [mean[i] * tint[i] for i in range(3)]
    else:
        g.rgba = [0.5, 0.5, 0.5, 1.0]            # MuJoCo's default: the material's own colour shows
        colours[g.name] = list(mean)
    return g


def _piece(w, meshes, colours, body, name, kind, xy, z, yaw, mu, size=None, scale=1.0, solid=True, loose=False):
    """One piece of furniture: its kit model is what the camera sees (each part in the one
    colour its manifest entry gives); what collides (when it is `solid`) is its bounding box,
    which is not drawn. A `loose` piece (a chair) is a free body standing on the floor: it
    moves when it is pushed."""
    K = kit()[kind]
    size = [v * scale for v in K["size"]] if size is None else size
    b = body.add_body(name=name, pos=[float(xy[0]), float(xy[1]), float(z) + (0.0005 if loose else 0.0)],
                      quat=[math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)])
    half = [size[0] / 2.0, size[1] / 2.0, size[2] / 2.0]
    if loose:
        b.add_freejoint(name=name)
        add_box(b, name + "_box", half, [0, 0, half[2]], [0.5, 0.5, 0.5, 1], 1, 6, mass=0.25 * CHAIR_MASS,
                friction=mu, group=0)
        low = b.add_geom(name=name + "_weight", type=mujoco.mjtGeom.mjGEOM_SPHERE, size=[0.02, 0, 0],
                         pos=[0, 0, 0.18], rgba=[0, 0, 0, 0])      # most of its weight sits low
        low.mass, low.contype, low.conaffinity, low.group = 0.75 * CHAIR_MASS, 0, 0, 2
    elif solid:
        add_box(b, name + "_box", half, [0, 0, half[2]], [0.5, 0.5, 0.5, 1], 4, 3, friction=mu, group=0)
    for k, part in enumerate(K["parts"]):
        skin = "{}_{}".format(kind, k)
        key = "{}_{}".format(skin, int(round(1000 * scale)))
        if skin not in meshes:
            w.add_texture(name=skin, type=mujoco.mjtTexture.mjTEXTURE_2D, file=part["texture"])
            mat = w.add_material(name=skin)
            mat.textures[mujoco.mjtTextureRole.mjTEXROLE_RGB] = skin
            if part["clear"]:
                mat.rgba = [1.0, 1.0, 1.0, 0.3]
            meshes.add(skin)
        if key not in meshes:
            mesh = w.add_mesh(name=key, file=part["file"], inertia=mujoco.mjtMeshInertia.mjMESH_INERTIA_SHELL)
            mesh.scale = [scale, scale, scale]
            meshes.add(key)
        g = b.add_geom(name="{}_m{}".format(name, k), type=mujoco.mjtGeom.mjGEOM_MESH, meshname=key, material=skin)
        g.contype, g.conaffinity, g.group = 0, 0, 1
        g.mass = 0.0
        colours[g.name] = list(part["rgba"])
    return b


def _cabinet(body, k, c, mu, colours):
    """A drawer cabinet: a carcass of panels with a fixed lower front, and under its top one
    drawer on a slide, with its pull."""
    W, D, H = c["size"]
    wood = dark = [0.5, 0.5, 0.5, 1.0]           # placeholders: every box is painted below
    a = c["turn"] * math.pi / 2.0
    name = "cabinet{}".format(k)
    b = body.add_body(name=name, pos=[c["xy"][0], c["xy"][1], 0.0], quat=[math.cos(a / 2), 0, 0, math.sin(a / 2)])
    t = CAB_PANEL
    front_top, floor, pull_z = drawer_frame(H)
    front_bottom = front_top - DRAWER_HEIGHT
    add_box(b, name + "_top", [W / 2, D / 2, t / 2], [0, 0, H - t / 2], wood, 4, 3, friction=mu)
    add_box(b, name + "_bottom", [W / 2, D / 2, t / 2], [0, 0, 0.06 + t / 2], wood, 4, 3)
    add_box(b, name + "_kick", [W / 2 - 0.02, D / 2 - 0.03, 0.03], [0, 0, 0.03], dark, 4, 3)
    add_box(b, name + "_back", [W / 2, t / 2, (H - 0.06) / 2], [0, -D / 2 + t / 2, (H + 0.06) / 2], wood, 4, 3)
    for sx in (-1, 1):
        add_box(b, "{}_side{}".format(name, sx), [t / 2, D / 2, (H - 0.06) / 2],
             [sx * (W / 2 - t / 2), 0, (H + 0.06) / 2], wood, 4, 3)
    # the fixed front below the drawer, with a groove down its middle to read as two doors
    low = front_bottom - 0.005 - 0.06 - t
    for sx in (-1, 1):
        add_box(b, "{}_door{}".format(name, sx), [W / 4 - t / 2 - 0.002, t / 2, low / 2],
             [sx * (W / 4 - t / 2 + 0.001), D / 2 - t / 2, 0.06 + t + low / 2], dark, 4, 3)
    # the drawer: front, floor, sides and back, sliding out along +y
    dr = b.add_body(name="drawer{}".format(k), pos=[0, 0, 0])
    j = dr.add_joint(name="drawer{}".format(k), type=mujoco.mjtJoint.mjJNT_SLIDE, axis=[0, 1, 0],
                     range=[0.0, DRAWER_TRAVEL])
    j.frictionloss, j.damping = DRAWER_FRICTION, [DRAWER_DAMPING, 0.0, 0.0]
    p = DRAWER_PANEL
    fw = W / 2 - t - 0.005                                   # the drawer's half width
    depth = D - t - 0.03                                     # front's inner face to the back's outside
    y0 = D / 2 - t                                           # the front's inner face
    mid = (front_top + front_bottom) / 2
    add_box(dr, name + "_dfront", [fw, t / 2, DRAWER_HEIGHT / 2], [0, D / 2 - t / 2, mid], wood, 4, 3,
         mass=0.3 * DRAWER_MASS)
    add_box(dr, name + "_dfloor", [fw, depth / 2, p / 2], [0, y0 - depth / 2, floor - p / 2], dark, 4, 3,
         mass=0.4 * DRAWER_MASS, friction=mu)
    add_box(dr, name + "_dback", [fw, p / 2, (DRAWER_HEIGHT - 0.03) / 2], [0, y0 - depth + p / 2, mid - 0.015],
         dark, 4, 3, mass=0.1 * DRAWER_MASS)
    for sx in (-1, 1):
        add_box(dr, "{}_dside{}".format(name, sx), [p / 2, depth / 2, (DRAWER_HEIGHT - 0.03) / 2],
             [sx * (fw - p / 2), y0 - depth / 2, mid - 0.015], dark, 4, 3, mass=0.1 * DRAWER_MASS)
    pull = dr.add_geom(name=name + "_pull", type=mujoco.mjtGeom.mjGEOM_CAPSULE,
                       fromto=[-PULL_HALF, D / 2 + PULL_STANDOFF, pull_z, PULL_HALF, D / 2 + PULL_STANDOFF, pull_z],
                       size=[PULL_RADIUS, 0, 0], rgba=BRASS)
    pull.mass, pull.contype, pull.conaffinity, pull.group = 0.05, 0, 0, 1
    for n, sx in enumerate((-PULL_HALF, PULL_HALF)):
        add_box(dr, "{}_pullpost{}".format(name, n), [0.004, PULL_STANDOFF / 2, 0.004],
             [sx, D / 2 + PULL_STANDOFF / 2, pull_z], [0.2, 0.2, 0.2, 1], 0, 0, mass=0.005)
    dr.add_site(name="drawer{}_grip".format(k), pos=[0, D / 2 + PULL_STANDOFF, pull_z], size=[0.004, 0, 0])
    # the episode's casework finish, the inner and lower panels a shade darker; the top is
    # the worktop
    for g in list(b.geoms) + list(dr.geoms):
        if g.type == mujoco.mjtGeom.mjGEOM_BOX and "pullpost" not in g.name:
            if g.name.endswith("_top"):
                _paint(g, "worktop", colours)
                continue
            shade = 0.8 if any(t in g.name for t in ("_kick", "_door", "_dfloor", "_dback", "_dside")) else 1.0
            _paint(g, "casework", colours, shade)
    return b


def _vessel(wb, v, stand, mu, colour=None):
    """A vessel on its stand, with its hinged lid, and the colour plate of its class on the
    stand's face to the store's middle."""
    hi, wl, ht, fl = VESSEL_INNER, VESSEL_WALL, VESSEL_HEIGHT, VESSEL_FLOOR
    vs = wb.add_body(name="vessel{}".format(v), pos=[float(stand[0]), float(stand[1]), 0])

    grey = [0.3, 0.36, 0.42, 1]
    add_box(vs, "stand{}".format(v), [STAND_HALF, STAND_HALF, STAND_H / 2], [0, 0, STAND_H / 2], [0.25, 0.25, 0.27, 1], 4, 3)
    if colour is not None:
        face = -math.copysign(STAND_HALF + PLATE_HALF[1], float(stand[1]))
        add_box(vs, "plate{}".format(v), PLATE_HALF, [0, face, PLATE_Z], list(colour), 0, 0)
    add_box(vs, "vfloor{}".format(v), [hi + wl, hi + wl, fl / 2], [0, 0, STAND_H + fl / 2], grey, 4, 3, friction=mu)
    for k, (sx, sy, bx, by) in enumerate(((1, 0, wl / 2, hi + wl), (-1, 0, wl / 2, hi + wl),
                                          (0, 1, hi + wl, wl / 2), (0, -1, hi + wl, wl / 2))):
        add_box(vs, "vwall{}_{}".format(v, k), [bx, by, ht / 2],
             [sx * (hi + wl / 2), sy * (hi + wl / 2), STAND_H + ht / 2], grey, 4, 3, friction=mu)
    # the lid: its frame has x along the hinge (the world's y) and y across the vessel's mouth
    lid = vs.add_body(name="lid{}".format(v), pos=[hi + wl, 0, STAND_H + ht + 0.004], quat=[math.cos(math.pi / 4), 0, 0, math.sin(math.pi / 4)])
    hinge = lid.add_joint(name="lid{}".format(v), type=mujoco.mjtJoint.mjJNT_HINGE, axis=[1, 0, 0], range=[0.0, LID_OPEN])
    hinge.damping, hinge.frictionloss = [0.02, 0.0, 0.0], 0.05
    add_box(lid, "lid{}_plate".format(v), [LID_SIZE / 2, LID_SIZE / 2, LID_THICK / 2], [0, LID_SIZE / 2, LID_THICK / 2],
            [0.36, 0.42, 0.48, 1], 4, 3, mass=LID_MASS)
    bar = lid.add_geom(name="lid{}_bar".format(v), type=mujoco.mjtGeom.mjGEOM_CAPSULE,
                       fromto=[-LID_HANDLE_HALF, LID_HANDLE[0], LID_HANDLE[1], LID_HANDLE_HALF, LID_HANDLE[0], LID_HANDLE[1]],
                       size=[0.006, 0, 0], rgba=LID_RED)
    bar.mass, bar.contype, bar.conaffinity, bar.group = 0.03, 0, 0, 1
    for n, sx in enumerate((-LID_HANDLE_HALF, LID_HANDLE_HALF)):
        add_box(lid, "lid{}_post{}".format(v, n), [0.004, 0.004, (LID_HANDLE[1] - LID_THICK) / 2],
                [sx, LID_HANDLE[0], (LID_HANDLE[1] + LID_THICK) / 2], [0.2, 0.2, 0.2, 1], 0, 0, mass=0.005)
    lid.add_site(name="lid{}_grip".format(v), pos=[0, LID_HANDLE[0], LID_HANDLE[1]], size=[0.004, 0, 0])


def _trolley(wb, tr, mu):
    """The trolley: a planar body (two slides and a turn, damped: it rolls on casters) with a
    frame of legs and a shelf, an open tray on top and its handle across its +x end."""
    a = tr["yaw"]
    t = wb.add_body(name="trolley", pos=[tr["xy"][0], tr["xy"][1], 0.0], quat=[math.cos(a / 2), 0, 0, math.sin(a / 2)])
    for axis, n in (([1, 0, 0], "x"), ([0, 1, 0], "y")):
        j = t.add_joint(name="trolley_" + n, type=mujoco.mjtJoint.mjJNT_SLIDE, axis=axis)
        j.damping, j.frictionloss, j.armature = [tr["roll"], 0.0, 0.0], 1.0, 0.5
    j = t.add_joint(name="trolley_turn", type=mujoco.mjtJoint.mjJNT_HINGE, axis=[0, 0, 1])
    j.damping, j.frictionloss, j.armature = [tr["turn"], 0.0, 0.0], 0.3, 0.2
    hx, hy = TROLLEY_HALF
    steel, tray = [0.55, 0.57, 0.60, 1], [0.78, 0.79, 0.80, 1]
    low, top = 0.10, TRAY_TOP - 0.03
    for i, (sx, sy) in enumerate(((1, 1), (1, -1), (-1, 1), (-1, -1))):
        add_box(t, "trolley_leg{}".format(i), [0.015, 0.015, (top - low) / 2], [sx * (hx - 0.02), sy * (hy - 0.02), (low + top) / 2],
                steel, 1, 6, mass=0.3)
        w = t.add_geom(name="trolley_wheel{}".format(i), type=mujoco.mjtGeom.mjGEOM_CYLINDER, size=[0.05, 0.02, 0],
                       pos=[sx * (hx - 0.02), sy * (hy - 0.02), 0.05], quat=[math.cos(math.pi / 4), math.sin(math.pi / 4), 0, 0],
                       rgba=[0.12, 0.12, 0.12, 1])
        w.contype, w.conaffinity, w.group, w.mass = 0, 0, 1, 0.2
    add_box(t, "trolley_shelf", [hx - 0.02, hy - 0.02, 0.012], [0, 0, 0.22], steel, 1, 6, mass=1.5)
    add_box(t, "trolley_tray", [hx, hy, 0.015], [0, 0, TRAY_TOP - 0.015], tray, 4, 3, mass=tr["mass"] - 4.0, friction=mu)
    for i, (half, pos) in enumerate((([hx, 0.01, RIM_H / 2], [0, hy - 0.01, TRAY_TOP + RIM_H / 2]),
                                     ([hx, 0.01, RIM_H / 2], [0, -(hy - 0.01), TRAY_TOP + RIM_H / 2]),
                                     ([0.01, hy - 0.02, RIM_H / 2], [hx - 0.01, 0, TRAY_TOP + RIM_H / 2]),
                                     ([0.01, hy - 0.02, RIM_H / 2], [-(hx - 0.01), 0, TRAY_TOP + RIM_H / 2]))):
        add_box(t, "trolley_rim{}".format(i), half, pos, tray, 4, 3, mass=0.2)
    # the handle: a child body whose x axis runs along the bar (the trolley's y)
    hx_, hz, hl = TROLLEY_HANDLE
    h = t.add_body(name="trolley_handle", pos=[hx_, 0.0, hz], quat=[math.cos(math.pi / 4), 0, 0, math.sin(math.pi / 4)])
    bar = h.add_geom(name="trolley_bar", type=mujoco.mjtGeom.mjGEOM_CAPSULE, fromto=[-hl, 0, 0, hl, 0, 0],
                     size=[0.012, 0, 0], rgba=CYAN)
    bar.mass, bar.contype, bar.conaffinity, bar.group = 0.2, 0, 0, 1
    for i, sy in enumerate((-1, 1)):                  # its two uprights, from the tray's end up to the bar
        add_box(t, "trolley_upright{}".format(i), [0.012, 0.012, (hz - TRAY_TOP) / 2],
                [hx - 0.012, sy * (hl - 0.02), (hz + TRAY_TOP) / 2], steel, 0, 0, mass=0.2)
        add_box(t, "trolley_arm{}".format(i), [(hx_ - hx) / 2 + 0.012, 0.012, 0.012],
                [(hx + hx_) / 2 - 0.006, sy * (hl - 0.02), hz], steel, 0, 0, mass=0.05)
    h.add_site(name="trolley_grip", pos=[0, 0, 0], size=[0.004, 0, 0])
    return t


def _store(room, colours):
    """The store's walls (its door in the wall x = STORE_X[1]) and its floor."""
    x0, x1 = STORE_X
    hy, t, h = STORE_HALF_Y, WALL_T / 2, WALL_H / 2
    cx, hx = (x0 + x1) / 2, (x1 - x0) / 2
    side = (hy - STORE_DOOR_HALF) / 2
    wall = [0.86, 0.84, 0.78, 1]
    for g in (add_box(room, "store_far", [t, hy + WALL_T, h], [x0 - t, 0, h], wall, 4, 3),
              add_box(room, "store_left", [hx + WALL_T, t, h], [cx, hy + t, h], wall, 4, 3),
              add_box(room, "store_right", [hx + WALL_T, t, h], [cx, -hy - t, h], wall, 4, 3),
              add_box(room, "store_door_l", [t, side, h], [x1 + t, STORE_DOOR_HALF + side, h], wall, 4, 3),
              add_box(room, "store_door_r", [t, side, h], [x1 + t, -STORE_DOOR_HALF - side, h], wall, 4, 3)):
        _paint(g, "wall", colours)
    _paint(add_box(room, "store_floor", [hx, hy, 0.001], [cx, 0, 0.0], wall, 0, 0), "floor", colours)


def build(ep):
    """The MuJoCo model of one episode."""
    hid = ep["hidden"]
    mu = hid["surface_friction"]
    w = new_world(ground_friction=mu)
    meshes = set()
    colours = {}                                 # geom name -> the colour it shows in a picture
    w.visual.global_.offwidth, w.visual.global_.offheight = 1280, 960
    wb = w.worldbody
    S = kit()[""]
    for role in ("floor", "wall", "casework", "worktop"):
        _surface(w, role, S[role][hid["style"][role]], colours)

    # the lab: four walls, a door in the wall x = 0
    wall = [0.86, 0.84, 0.78, 1]
    hx, hy, t, h = ROOM_X / 2, ROOM_Y / 2, WALL_T / 2, WALL_H / 2
    room = wb.add_body(name="room")
    side = (hy - DOOR_HALF) / 2
    for g in (add_box(room, "wall_far", [t, hy + WALL_T, h], [ROOM_X + t, 0, h], wall, 4, 3),
              add_box(room, "wall_left", [hx + WALL_T, t, h], [hx, hy + t, h], wall, 4, 3),
              add_box(room, "wall_right", [hx + WALL_T, t, h], [hx, -hy - t, h], wall, 4, 3),
              add_box(room, "wall_door_l", [t, side, h], [-t, DOOR_HALF + side, h], wall, 4, 3),
              add_box(room, "wall_door_r", [t, side, h], [-t, -DOOR_HALF - side, h], wall, 4, 3)):
        _paint(g, "wall", colours)
    _paint(add_box(room, "floor", [hx, hy, 0.001], [hx, 0, 0.0], wall, 0, 0), "floor", colours)
    for k, f in enumerate(hid["furniture"]):
        loose = kit()[f["kind"]]["kind"] == MOVABLE
        _piece(w, meshes, colours, wb if loose else room, "furniture{}".format(k), f["kind"], f["xy"], 0.0, f["yaw"],
               mu, size=f["size"], scale=f["scale"], loose=loose)
    for k, o in enumerate(hid["things"]):
        _piece(w, meshes, colours, room, "thing{}".format(k), o["kind"], o["xy"], o["z"], o["yaw"], mu, scale=o["scale"])
    for k, c in enumerate(hid["cabinets"]):
        _cabinet(room, k, c, mu, colours)

    # the package and the harmless cases, in their drawers
    for k, o in enumerate(hid["packages"]):
        _case(wb, "package{}".format(k), o["xy"], o["z"], o["yaw"], ORANGE, o["ballast_mass"], o["ballast_pos"],
              liquid=(o["liquid_share"], o["slosh_hz"], o["slosh_zeta"]), label=CLASS_COLOURS[o["cls"]])
    for k, o in enumerate(hid["others"]):
        _case(wb, "other{}".format(k), o["xy"], o["z"], o["yaw"], OTHER_HANDLES[k % len(OTHER_HANDLES)],
              1.2, [0.0, 0.0, 0.0])

    # the vessels on their stands, outside, each with its lid
    _store(room, colours)
    for v, stand in enumerate(STANDS):
        _vessel(wb, v, stand, mu, CLASS_COLOURS[hid["vessel_class"][v]])
    _trolley(wb, hid["trolley"], mu)

    # cameras for people to watch with (no observation uses them)
    wb.add_camera(name="overhead", pos=[ROOM_X / 2 - 1.0, 0.0, 10.5], xyaxes=[0, -1, 0, 1, 0, 0], fovy=44)
    # what the latch can close on: the handle of every case and the pull of every drawer
    holds = ["package{}".format(k) for k in range(len(hid["packages"]))] \
        + ["other{}".format(k) for k in range(len(hid["others"]))] \
        + ["drawer{}".format(k) for k in range(len(hid["cabinets"]))] + ["lid{}".format(v) for v in range(len(STANDS))] \
        + ["trolley"]
    for n in holds:
        w.add_equality(name="latch_" + n, type=mujoco.mjtEq.mjEQ_CONNECT, objtype=mujoco.mjtObj.mjOBJ_SITE,
                       name1="latch", name2=n + "_grip", active=False, solref=LATCH_SOLREF)
    add_cart(w)
    return w.compile(), holds, colours


def _tilt_deg(xmat):
    return math.degrees(math.acos(max(-1.0, min(1.0, float(xmat[8])))))


def _lean_deg(case_xmat, liquid_xmat):
    """The liquid's lean against the case: the angle between the case's z axis and the
    liquid pendulum's (from the point mass up to its pivot). At rest, the case's tilt."""
    c = float(case_xmat[2] * liquid_xmat[2] + case_xmat[5] * liquid_xmat[5] + case_xmat[8] * liquid_xmat[8])
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


# ------------------------------------------------------------------------------------------
# the episode
# ------------------------------------------------------------------------------------------
class LabEnv(object):
    """reset(seed) -> observation; step(action) -> (observation, done). `outcome` is
    "contained", "spilled:<why>" or "" while the episode runs. With `camera=False` no picture
    is drawn (the observation's `camera` stays null): faster, for work that needs none."""

    def __init__(self, camera=True):
        self.camera = camera

    def reset(self, seed):
        self.ep = draw_episode(seed)
        hid = self.ep["hidden"]
        self.model, holds, colours = build(self.ep)
        self.data = mujoco.MjData(self.model)
        m, d = self.model, self.data
        name = lambda kind, n: mujoco.mj_name2id(m, kind, n)  # noqa: E731
        self.cart = Cart(m, d, CART_START, camera=self.camera)
        for geom, rgb in colours.items():           # textured surfaces: one colour each in a picture
            gid = name(mujoco.mjtObj.mjOBJ_GEOM, geom)
            if gid >= 0:
                self.cart.colours[gid] = rgb
        count = len(hid["packages"])
        self.packages = [name(mujoco.mjtObj.mjOBJ_BODY, "package{}".format(k)) for k in range(count)]
        self.liquids = [name(mujoco.mjtObj.mjOBJ_BODY, "package{}_liquid".format(k)) for k in range(count)]
        self.pkg_dof = [int(m.jnt_dofadr[name(mujoco.mjtObj.mjOBJ_JOINT, "package{}".format(k))]) for k in range(count)]
        self.package = self.packages[0]
        self.site = dict(countdown_s=self.ep["countdown_s"], tilt_limit_deg=self.ep["tilt_limit_deg"],
                         shock_limit=self.ep["shock_limit"], vessels=STANDS, packages=count)
        # every bar the latch can close on: the body it belongs to, its axis in that body's
        # frame (middle, half length along the body's x), its site and its equality
        self.bars = []
        for n in holds:
            site = name(mujoco.mjtObj.mjOBJ_SITE, n + "_grip")
            self.bars.append(dict(name=n, body=int(m.site_bodyid[site]), site=site,
                                  mid=m.site_pos[site].copy(),
                                  half=PULL_HALF if n.startswith("drawer") else LID_HANDLE_HALF if n.startswith("lid")
                                  else TROLLEY_HANDLE[2] if n == "trolley" else BAR_HALF,
                                  eq=name(mujoco.mjtObj.mjOBJ_EQUALITY, "latch_" + n)))
            d.eq_active[self.bars[-1]["eq"]] = 0
            m.site_sameframe[site] = 0                # its position is moved to where the latch closes
        self.lid_adr = [int(m.jnt_qposadr[name(mujoco.mjtObj.mjOBJ_JOINT, "lid{}".format(v))]) for v in range(len(STANDS))]
        self.drawer_adr = [int(m.jnt_qposadr[name(mujoco.mjtObj.mjOBJ_JOINT, "drawer{}".format(k))])
                           for k in range(len(hid["cabinets"]))]
        self.secured_at = None
        self.chairs = [name(mujoco.mjtObj.mjOBJ_BODY, "furniture{}".format(k))
                       for k, f in enumerate(hid["furniture"]) if kit()[f["kind"]]["kind"] == MOVABLE]
        d.qpos[self.lid_adr] = LID_OPEN
        mujoco.mj_forward(m, d)
        self.held = None                          # the bar the latch holds
        self.t = 0
        self.outcome = ""
        self.tilt_time = [0.0] * count
        self.shocks = [0.0] * count
        self.safe = [False] * count                   # secured in a shut vessel: it can no longer be lost
        self.shock = 0.0
        self.peak_tilt = 0.0
        self.peak_shock = 0.0
        for _ in range(50):                      # let everything settle
            mujoco.mj_step(m, d)
        self._v_prev = [self._package_velocity(k) for k in range(count)]
        self.chairs_at = [(d.xpos[b][:2].copy(), self._heading(b)) for b in self.chairs]
        if self.camera:
            self.cart.picture = take_picture(m, d, self.cart.cam, self.cart.colours, 0.0)
        return self.observation()

    # -- helpers ---------------------------------------------------------------------------
    @property
    def latched(self):
        return self.held is not None

    def _package_velocity(self, k):
        """The velocity of the package's centre of mass: its case and its contents together."""
        v, w = np.zeros(6), np.zeros(6)
        mujoco.mj_objectVelocity(self.model, self.data, mujoco.mjtObj.mjOBJ_BODY, self.packages[k], v, 0)
        mujoco.mj_objectVelocity(self.model, self.data, mujoco.mjtObj.mjOBJ_BODY, self.liquids[k], w, 0)
        mc, ml = self.model.body_mass[self.packages[k]], self.model.body_mass[self.liquids[k]]
        return (mc * v[3:] + ml * w[3:]) / (mc + ml)

    def _set_latch(self, want):
        """Close on the nearest bar whose axis is within LATCH_REACH of the latch point; the
        point of that bar's body that is at the latch point then stays there."""
        m, d = self.model, self.data
        if want and self.held is None:
            best = None
            for b in self.bars:
                R = d.xmat[b["body"]].reshape(3, 3)
                local = R.T @ (d.site_xpos[self.cart.latch] - d.xpos[b["body"]])
                on_axis = b["mid"].copy()
                on_axis[0] += float(np.clip(local[0] - b["mid"][0], -b["half"], b["half"]))
                dist = float(np.linalg.norm(local - on_axis))
                if dist <= LATCH_REACH and (best is None or dist < best[0]):
                    best = (dist, b, local)
            if best is not None:
                _, b, local = best
                m.site_pos[b["site"]] = local
                mujoco.mj_kinematics(m, d)
                d.eq_active[b["eq"]] = 1
                self.held = b
        elif not want and self.held is not None:
            d.eq_active[self.held["eq"]] = 0
            self.held = None

    # -- the step --------------------------------------------------------------------------
    def step(self, action):
        """`action`: {"base": [vx, vy, wz], "q": seven joint targets, "latch": 0 or 1,
        "look": 0 or 1}; see cart_model.Cart.command and .look."""
        if self.outcome:
            return self.observation(), True
        m, d = self.model, self.data
        self.cart.command(action)
        self._set_latch(bool(int(action.get("latch", 0))))
        hid = self.ep["hidden"]
        held = self.held["name"] if self.held is not None else ""
        alpha = PHYSICS_DT / (SHOCK_TAU_S + PHYSICS_DT)
        for i in range(SUBSTEPS):
            self.cart.drive(i)
            for k, dof in enumerate(self.pkg_dof):
                d.qfrc_applied[dof + 3:dof + 6] = (-hid["packages"][k]["swing_damping"] * d.qvel[dof + 3:dof + 6]
                                                   if held == "package{}".format(k) else 0.0)
            mujoco.mj_step(m, d)
            for k in range(len(self.packages)):
                v = self._package_velocity(k)
                acc = float(np.linalg.norm(v - self._v_prev[k])) / PHYSICS_DT
                self._v_prev[k] = v
                self.shocks[k] += alpha * (acc - self.shocks[k])
                tilt = _lean_deg(d.xmat[self.packages[k]], d.xmat[self.liquids[k]])
                self.tilt_time[k] = self.tilt_time[k] + PHYSICS_DT if tilt > self.ep["tilt_limit_deg"] else 0.0
                if self.safe[k]:
                    continue
                self.peak_tilt = max(self.peak_tilt, tilt)
                self.peak_shock = max(self.peak_shock, self.shocks[k])
                if self.tilt_time[k] >= TILT_HOLD_S:
                    self.outcome = "spilled:tilt"
                elif self.shocks[k] > self.ep["shock_limit"]:
                    self.outcome = "spilled:shock"
            self.shock = max(self.shocks)
            if self.outcome:
                break
        self.t += 1
        if not self.outcome:
            now = self.t * CONTROL_DT
            for k in range(len(self.packages)):
                if not self.safe[k] and self._secured(k):
                    self.safe[k] = True
            if all(self.safe) and self.secured_at is None:
                self.secured_at = now
            if self.secured_at is not None and self._restored() and self.held is None \
                    and all(self._secured(k) for k in range(len(self.packages))):
                self.outcome = "contained"
            elif now >= self.ep["countdown_s"]:              # one clock for all of it, the lab put right included
                if self.secured_at is None:
                    self.outcome = "spilled:countdown"
                else:
                    why = self._restored(why=True)
                    if not why:
                        why = "the latch holding " + self.held["name"] if self.held is not None else "a vessel open"
                    self.outcome = "left:" + why
        self.cart.look(action, self.t)
        return self.observation(), bool(self.outcome)

    def _secured(self, k):
        """Package k rests on the floor of the vessel of its class, whose lid is shut, and the latch holds
        neither of the two."""
        d = self.data
        p = d.xpos[self.packages[k]]
        cls = self.ep["hidden"]["packages"][k]["cls"]
        for v, stand in enumerate(STANDS):
            if self.ep["hidden"]["vessel_class"][v] != cls:
                continue                              # only the vessel of its own class
            if max(abs(p[0] - stand[0]), abs(p[1] - stand[1])) < VESSEL_INNER \
                    and p[2] < STAND_H + VESSEL_FLOOR + BOX_HALF[2] + 0.004:
                if self.held is not None and self.held["name"] in ("package{}".format(k), "lid{}".format(v)):
                    return False
                return bool(d.qpos[self.lid_adr[v]] <= LID_SHUT and float(np.linalg.norm(self._v_prev[k])) < 0.01)
        return False

    def _heading(self, body):
        R = self.data.xmat[body].reshape(3, 3)
        return math.atan2(R[1, 0], R[0, 0])

    def _restored(self, why=False):
        """Is the lab as it was: every drawer shut, every stool and chair where it stood?"""
        d = self.data
        out = [k for k, adr in enumerate(self.drawer_adr) if d.qpos[adr] > DRAWER_SHUT]
        moved = []
        for b, (xy, yaw) in zip(self.chairs, self.chairs_at):
            turn = abs((self._heading(b) - yaw + math.pi) % (2 * math.pi) - math.pi)
            if np.linalg.norm(d.xpos[b][:2] - xy) > IN_PLACE[0] or turn > IN_PLACE[1] or d.xmat[b][8] < 0.9:
                moved.append(int(b))
        if why:
            return "; ".join(t for t in ("drawers open {}".format(out) if out else "",
                                         "{} chair(s) moved".format(len(moved)) if moved else "") if t)
        return not out and not moved

    def observation(self, q_offset=None, base_offset=None):
        hid = self.ep["hidden"]
        return self.cart.observation(self.t, self.site, hid["ft_bias"], hid["noise_seed"],
                                     self.held is not None, q_offset=q_offset, base_offset=base_offset)

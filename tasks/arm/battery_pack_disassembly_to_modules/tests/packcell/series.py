"""The HVB-P battery pack series and the disassembly cell: the cell's geometry, the tools, the fastener catalog, the
three pack variants, and the draw of one pack with its hidden differences from a 64-bit seed.

NumPy only: the scene builder (`scene.py`), the rule engine (`rules.py`), the cell (`cell.py`), the documents'
figures and tables (`tests/review/make_documents.py`) and the reference program's privileged view all read the same
numbers from here.

Frames. The world frame is the robot's base frame (z up, origin on the base's mounting face). A pack has its own frame
P: origin on the fixture's surface under the tray's centre, x and y along the tray's short and long sides; on the
fixture P sits at `CELL["pack_nominal"]` plus the pack's placement error. Every part's body frame is parallel to P
when the pack arrives and has its origin at the part's centre; a fastener's frame has its origin on the axis in the
seat plane (the surface its head sits on), z out of the hole.
"""

import hashlib
import math
import struct

import numpy as np

VERSION = "0.1.0"

# --------------------------------------------------------------------------------------------- time
TICK = 0.05           # one control tick (s)
SUBSTEPS = 6          # physics steps per tick
PHYS_DT = TICK / SUBSTEPS

# --------------------------------------------------------------------------------------------- the cell (world frame)
TABLE_Z = -0.40       # the top of the fixture and of every table
FLOOR_Z = -0.80

CELL = {
    "pack_nominal": [0.86, 0.0, TABLE_Z],
    "tables": {                              # name: (x0, x1, y0, y1), top at TABLE_Z
        "fixture": (0.45, 1.27, -0.56, 0.56),
        "right": (0.10, 1.06, -0.88, -0.58),
        "left": (0.48, 1.06, 0.58, 0.86),
        "rack": (-0.62, 0.44, 0.60, 0.98),
        "lid_stand": (-1.30, -0.60, -0.55, 0.55),
        "tool_stand": (-0.56, 0.08, -0.86, -0.60),
    },
    # bins: inner footprint centre and size, floor at TABLE_Z, walls `wall_h` high and 0.008 thick
    "bins": {
        "electronics": {"center": [0.30, -0.73], "size": [0.34, 0.26], "wall_h": 0.10},
        "fasteners_right": {"center": [0.64, -0.72], "size": [0.22, 0.22], "wall_h": 0.10},
        "copper": {"center": [0.92, -0.72], "size": [0.22, 0.22], "wall_h": 0.10},
        "fasteners_left": {"center": [0.64, 0.72], "size": [0.22, 0.22], "wall_h": 0.10},
        "steel": {"center": [0.92, 0.72], "size": [0.22, 0.22], "wall_h": 0.10},
    },
    # the module rack: six bays in a row along x, a module stands in a bay with its long side along y
    "rack": {"bay_centers": [[-0.515 + 0.17 * k, 0.79] for k in range(6)], "bay_size": [0.15, 0.33],
             "divider_h": 0.05, "divider_t": 0.012},
    "lid_stand": {"center": [-0.95, 0.0], "size": [0.70, 1.10]},
    # the tool stand: where the flange is when a tool is exchanged (tool axis straight down, flange x along world x)
    "docks": {"nutrunner": [-0.18, -0.73, -0.06], "gripper": [-0.42, -0.73, -0.06]},
    # the bit station: where the nutrunner's bit tip is when the bit is exchanged (tool axis straight down)
    "bit_station": [0.00, -0.73, -0.30],
    "dock_tol": {"pos": 0.006, "angle_deg": 3.0, "speed": 0.01},
    "home_q": [0.0, -1.45, 1.85, -1.97, -1.5708, 0.0],
}

# cameras: OpenCV convention (x right, y down, z forward); R's columns are the camera axes in the parent frame
CAMERAS = {
    "overhead": {"parent": "world", "pos": [0.86, 0.0, 1.25],
                 "R": [[0.0, -1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, -1.0]],
                 "width": 1440, "height": 1080, "hfov_deg": 45.6, "clip": [0.3, 6.0]},
    "wrist": {"parent": "flange", "pos": [0.070, 0.0, 0.045], "tilt_deg": 11.0,
              "width": 960, "height": 720, "hfov_deg": 38.0, "clip": [0.03, 4.0]},
}


def camera_R(name):
    """Rotation of camera `name` in its parent frame (columns: the camera's x, y, z axes)."""
    c = CAMERAS[name]
    if "R" in c:
        return np.array(c["R"], dtype=float).T
    b = math.radians(c["tilt_deg"])
    z = np.array([-math.sin(b), 0.0, math.cos(b)])
    x = np.array([0.0, 1.0, 0.0])
    return np.stack([x, np.cross(z, x), z], axis=1)


def camera_K(name):
    c = CAMERAS[name]
    f = 0.5 * c["width"] / math.tan(math.radians(c["hfov_deg"]) / 2)
    return [[f, 0.0, c["width"] / 2.0], [0.0, f, c["height"] / 2.0], [0.0, 0.0, 1.0]]


# The cameras' depth channel as their data sheets state it (`sensors.measure` applies it; the colour image is the
# renderer's). The overhead camera is a long-range industrial active-stereo RGB-D camera, the wrist camera a
# short-range one: noise sigma_k z^2 (1 sigma, z in m), quantization one `subpixel` disparity step at `baseline_m`,
# readings lost at discontinuities, in the second view's occlusion, on dark and on bright (specular) surfaces.
# Why these sizes: at 1.65 m (the fixture under the overhead camera) the noise is 2.5 mm, so one overhead frame does
# not tell a 4.5 mm head from a 6 mm one (1.35 mm) but several frames do; at 0.35 m (a close wrist view) it is
# 0.3 mm (tests/README.md, section 2).
SENSORS = {
    "overhead": {"range_m": [0.5, 3.0], "sigma_k": 0.0009, "baseline_m": 0.10, "subpixel": 0.0625, "corr_px": 2.0,
                 "blur_px": 1.0, "edge_px": 2.0, "dark": [12, 40], "dim": 80, "bright": [215, 245]},
    "wrist": {"range_m": [0.07, 1.0], "sigma_k": 0.0025, "baseline_m": 0.018, "subpixel": 0.0625, "corr_px": 2.0,
              "blur_px": 1.0, "edge_px": 2.0, "dark": [12, 40], "dim": 80, "bright": [215, 245]},
}

# A cell's cameras are mounted by hand: each sits off its nominal mounting (CAMERAS: in the base frame for the
# overhead camera, in the flange frame for the wrist camera) by an offset of its own, six numbers in the camera's own
# frame (x right, y down, z forward): a translation (mm) and a rotation vector (degrees), each component within
# +-MOUNT_TOL. The true camera is nominal @ offset. Every pack is taken apart in a cell of its own (the hidden set's
# six and the four development packs alike); the offsets are drawn under tests/review/select_set.py's rule 2 with a
# key of their own and are never published. The calibration plates below and the robot's exact kinematics are what a
# program identifies them with.
MOUNT_TOL = {"overhead": {"mm": 15.0, "deg": 1.5}, "wrist": {"mm": 4.0, "deg": 1.0}}

# Calibration plates fixed to the fixture's surface on the two strips a pack never covers (nearest to and farthest
# from the robot), surveyed in the base frame. A plate is a checkerboard of `squares` [along x, along y] squares of
# `square` metres; its frame is parallel to the base frame with its origin at the board's corner of least x and y,
# on the plate's surface; the square (i, j) (i along x, j along y, from 0) is black when i + j is even. The white
# plate reaches `margin` beyond the board on every side.
CAL_PLATES = {
    "near": {"origin": [0.460, -0.100, TABLE_Z + 0.001], "squares": [5, 20], "square": 0.010, "margin": 0.008},
    "far": {"origin": [1.208, -0.100, TABLE_Z + 0.001], "squares": [5, 20], "square": 0.010, "margin": 0.008},
}


def mount_offset_T(off):
    """The 4x4 of a mounting offset [dx, dy, dz (mm), rx, ry, rz (deg, a rotation vector)] (camera frame)."""
    from . import kin
    off = [0.0] * 6 if off is None else [float(v) for v in off]
    return kin.pose(kin.rotvec_R(np.radians(off[3:6])), np.array(off[:3]) / 1000.0)


def camera_mount(name, offset=None):
    """Camera `name` in its parent frame (4x4, OpenCV axes): the nominal mounting of CAMERAS, times the cell's
    mounting `offset` when one is given (the true camera)."""
    from . import kin
    T = kin.pose(camera_R(name), np.array(CAMERAS[name]["pos"], dtype=float))
    return T @ mount_offset_T(offset) if offset is not None else T


# --------------------------------------------------------------------------------------------- the tools (flange frame)
TOOLS = {
    "nutrunner": {
        "mass": 3.9, "com": [0.0, 0.0, 0.085], "spindle_mass": 0.3,
        "body": {"r": 0.030, "z0": 0.030, "z1": 0.170},
        "spindle_origin_z": 0.170, "spindle_len": 0.100, "bit_len": 0.030,
        "tcp_z": 0.300,                       # the bit's tip with the spindle uncompressed
        "stroke": 0.025, "spring_N_per_m": 4000.0, "spring_damping": 120.0,
        "speed_max_rpm": 300.0, "torque_max": 60.0,
        "collider_r": 0.006,
    },
    "gripper": {
        "mass": 2.6, "com": [0.0, 0.0, 0.07],
        "body": {"size": [0.05, 0.10, 0.09], "z0": 0.030},
        "finger_len": 0.13, "finger_t": 0.010, "finger_w": 0.022,
        "tcp_z": 0.235,                       # between the fingertips, 0.015 above the fingers' ends
        "open_max": 0.085, "speed": 0.08,     # full stroke in about a second
        "close_axis": [0.0, 1.0, 0.0],
    },
    "changer_plate": {"r": 0.045, "h": 0.030},
}
BITS = ("S10", "S13", "T30", "T40", "EX")
BIT_LOOK = {"S10": {"r": 0.0090}, "S13": {"r": 0.0110}, "T30": {"r": 0.0040}, "T40": {"r": 0.0050}, "EX": {"r": 0.0075}}

# --------------------------------------------------------------------------------------------- fasteners
THREADS = {
    # tighten: the assembly torque; breakaway (first 30 degrees of loosening) = 1.15 x tighten, 1.6 to 1.9 x on a
    # corroded fastener, 1.15 to 1.6 x shear on a seized one (SEIZED); head_limit: above it the head or its recess gives
    # way (either way); shear: the shank twists off at this torque (either way); turns: full turns to free it
    "M6": {"tighten": 10.0, "head_limit": 22.0, "shear": 20.0, "turns": 7.0, "shank_r": 0.003, "shank_len": 0.016},
    "M8": {"tighten": 23.0, "head_limit": 45.0, "shear": 46.0, "turns": 9.0, "shank_r": 0.004, "shank_len": 0.022},
}
HEADS = {
    # kind hex: a hexagon `af` across flats on a flange; kind torx: a round pan head with a six-lobe recess
    "S10": {"thread": "M6", "kind": "hex", "af": 0.010, "flange_r": 0.0071, "flange_h": 0.0012, "height": 0.0060},
    "T30": {"thread": "M6", "kind": "torx", "head_r": 0.0060, "recess_r": 0.00276, "height": 0.0045},
    "S13": {"thread": "M8", "kind": "hex", "af": 0.013, "flange_r": 0.0089, "flange_h": 0.0015, "height": 0.0080},
    "T40": {"thread": "M8", "kind": "torx", "head_r": 0.0080, "recess_r": 0.00333, "height": 0.0060},
}
# what a bit does on a head: "fit" drives it, "loose" grips badly and wears the head, anything else turns freely
BIT_ON_HEAD = {
    ("S10", "S10"): "fit", ("S13", "S13"): "fit", ("T30", "T30"): "fit", ("T40", "T40"): "fit",
    ("S13", "S10"): "loose",        # a 13 mm socket on a 10 mm hexagon rounds its corners
    ("T30", "T40"): "loose",        # a T30 bit in a T40 recess reams it out
}
ENGAGE = {
    # lateral offset of the bit's axis from the fastener's (m) and angle between the axes (degrees)
    "hex": {"full": 0.0008, "partial": 0.0035},       # (rule 8, 2026-11-26: full was 0.0020; S13 below)
    "full_by_head": {"S13": 0.0012},                  # (rule 8, amended 2026-11-27: a 13 mm socket's lead-in)
    "torx": {"full": 0.0005, "partial": 0.0025},      # (rule 8: full was 0.0012)
    "tilt_full_deg": 2.0, "tilt_partial_deg": 8.0,    # (rule 8: full was 3.0)
    "seat_force": 8.0,                 # axial force (N) below which the bit is not seated
    "torx_force_per_Nm": 1.5, "torx_force_base": 10.0,   # a Torx bit cams out below base + per_Nm x torque
    "seat_gap": 0.0015,                # the tip counts as on the head within this distance above it
    "wear_per_rev_partial": 1.0 / 1.5, "wear_per_rev_camout": 1.0, "wear_per_s_overtorque": 2.0,
    "breakaway_deg": 30.0, "running_fraction": 0.12, "free_torque": 0.15,
    "extractor": {"lateral": 0.0015, "force": 50.0, "torque_factor": 1.2, "speed_factor": 0.5},
}

# --------------------------------------------------------------------------------------------- the pack (pack frame)
TRAY = {"size": [0.62, 1.00], "height": 0.160, "wall": 0.020, "bottom": 0.012}
LID = {"t": 0.004, "mass": 5.0, "handle_z": 0.038}
MODULE = {"size": [0.30, 0.13, 0.11], "x": 0.09, "pitch": 0.15, "mass": 11.5, "handle_z": 0.077,
          "post": [-0.135, 0.035], "post_r": 0.009, "post_h": 0.010}
Z = {
    "lid_seat": TRAY["height"] + LID["t"],                         # 0.164: where a lid bolt's head sits
    "lid_center": TRAY["height"] + LID["t"] / 2,
    "module_center": TRAY["bottom"] + MODULE["size"][2] / 2,       # 0.067
    "post_top": TRAY["bottom"] + MODULE["size"][2] + MODULE["post_h"],   # 0.132
    "busbar_center": 0.134, "busbar_seat": 0.136,
    "bracket_center": 0.029, "bracket_seat": 0.040,
}
SERVICE_X = -0.20          # the service bay: service disconnect and battery management unit
JUNCTION_X = -0.145
BRACKET_X = (0.258, -0.078)

VARIANTS = {
    "P4": {"modules": 4, "msd_y": 0.33, "bms_y": -0.25,
           "lid_bolts": [[sx * 0.30, y] for sx in (-1, 1) for y in (-0.36, -0.12, 0.12, 0.36)] + [[0.0, -0.49], [0.0, 0.49]],
           "ribs": [("y", -0.14), ("y", 0.14)]},
    "P5": {"modules": 5, "msd_y": -0.33, "bms_y": 0.22,
           "lid_bolts": [[sx * 0.30, y] for sx in (-1, 1) for y in (-0.40, -0.20, 0.0, 0.20, 0.40)] + [[0.0, -0.49], [0.0, 0.49]],
           "ribs": [("x", -0.35), ("x", -0.15), ("x", 0.15), ("x", 0.35)]},
    "P6": {"modules": 6, "msd_y": 0.0, "bms_y": 0.27,
           "lid_bolts": [[sx * 0.30, y] for sx in (-1, 1) for y in (-0.40, -0.20, 0.0, 0.20, 0.40)]
           + [[x, sy * 0.49] for sy in (-1, 1) for x in (-0.15, 0.15)],
           "ribs": [("y", -0.14), ("y", 0.14), ("x", -0.30), ("x", 0.30)]},
}

# Sealant partly over heads (a field pack's resealing in service; service manual, section 7). A dab of grey sealant
# is an ellipsoidal dome standing on the fastener's seat: its centre `d` from the fastener's axis in the direction
# `angle`, its radius `r` on the seat, its top `h` above the seat (the head's height plus `above`). It covers part of
# the head's top and runs over the head's edge onto the seat. It is drawn only (no collider, no mass): soft, a bit
# goes through it, and it comes out with its fastener. Sizes in units of the head's top radius (a hexagon's half
# width across flats, a pan head's radius). Drawn per pack under tests/review/select_set.py's rule 3 (a seed of its
# own per pack, `sealant_seed`).
SEALANT = {"p": {"lid": 0.35, "busbar": 0.25, "bracket": 0.25}, "d": (0.3, 0.9), "r": (0.6, 1.0), "above": (0.0010, 0.0025)}


# A pack's appearance (the fourth lever; tests/README.md, section 2). The cell stands in an open hall whose light
# varies from pack to pack, and a field pack comes dirty and labelled. Per pack (tests/review/select_set.py's rule 4,
# `appearance_seed`): the light's brightness `gain` (every lamp and the dome) and colour temperature `kelvin`, and
# each of the four lamps' own share `lamp`; patches of dark grime and of light dust on the lid (ellipses with semi-axes
# `patch_axes`, anywhere on it); service labels on the lid (white, `label_size`, black bars printed on them), clear of
# its bolts, handle and ribs by `label_clear`; a dusty film on some clean zinc heads (`dusty_p`). All drawn only, all
# on the lid or its bolts except the light. A surface's depth reading does not depend on the hall's light
# (`sensors.measure`'s `light_gain`).
# Seized fasteners (the fifth lever, 2026-11-26; service manual, section 7; rule 5 of tests/review/select_set.py). A
# fastener seized by corrosion or thread-locker breaks away only at `breakaway` times its thread's shear torque, more
# than its shank holds: it has to be worked loose (WORK). It looks as it was drawn (rusty when drawn corroded, else
# clean). A development pack has `n_dev` of them, a hidden pack `n_hidden` (the documents: up to 4 in a pack),
# chosen with `weights` by group among the pack's present fasteners. Every fastener has a working factor `work`.
SEIZED = {"n_dev": 1, "n_hidden": (2, 4), "max": 4, "weights": {"lid": 1.0, "busbar": 1.0, "bracket": 2.0},
          "breakaway": (1.15, 1.6), "work": (0.55, 0.80)}
# Working a thread loose (rules.Rules._work): with the bit fully engaged, a loosening stall at `loosen` times the
# fastener's own shear torque or more (below it) held `loosen_s` in all, then a tightening stall at the assembly torque
# or more (below the head's limit and the shear torque) held `tighten_s`, is one cycle; a cycle multiplies what the
# fastener's breakaway exceeds a clean one's (1.15 x tighten) by its working factor. (Rule 9, 2026-11-26: `loosen` was
# 0.85 of the thread's shear torque; it is now 0.80 of the fastener's own, which SHANK draws.)
WORK = {"loosen": 0.80, "loosen_s": 0.5, "tighten_s": 0.25}

# Swollen modules (the sixth lever, 2026-11-26; service manual, sections 7 and 9; rule 6). Every module's top plate
# carries a smooth bump (drawn only, `rules`/`scene`): a healthy module's `crown`, a swollen module's `bulge` (heights in
# m at the bump's centre over the plate's rim), with semi-axes `a` along the module's length and `b` across, its centre
# `dx` along the length off the plate's centre. A module whose bump is `criterion` or more is swollen; it stays in
# its seat (`vent_m`: moved that far from its seat, it vents and the pack ends). Which packs carry one swollen module
# is fixed by `dev` and `hidden` (rule 6).
SWOLLEN = {"dev": (0, 1, 1, 0), "hidden": (1, 1, 0, 1, 1, 0), "bulge": (0.0006, 0.0030), "crown": (0.0, 0.00025),
           "criterion": 0.0004, "a": (0.09, 0.12), "b": (0.035, 0.050), "dx": (-0.015, 0.015), "vent_m": 0.005,
           "level_deg": 5.0}

# Foam pads over fasteners (the seventh lever, 2026-11-26; service manual, section 7; rule 7). A square pad of soft
# open-cell foam, `side` across and `thick` high above the fastener's seat, turned by up to 90 degrees, its centre
# within `off` of the fastener's nominal place, glued to the part the fastener holds (it leaves with that part). Drawn
# only: a bit goes through it; it hides the head (or the hole of a missing fastener) from both cameras. How many
# fasteners a pack has under pads is fixed by `dev` and `hidden`; they are chosen with `weights` among the lid's and
# the busbars' fasteners that a harness remnant does not cover.
FOAM = {"dev": (1, 0, 1, 0), "hidden": (2, 1, 1, 2, 0, 1), "max": 2, "weights": {"lid": 2.0, "busbar": 1.0},
        "side": (0.040, 0.060), "thick": (0.010, 0.014), "off": 0.008}

# The third hardening round (2026-11-26, evening; rules 8 to 11 of tests/review/select_set.py; tests/README.md,
# section 2), after a frontier model scored 0.97 on the tree of rules 5 to 7: what an expert who knows the procedure
# still has to do well, and mistakes that cannot be undone.
#
# Rule 8, precision engagement (a rule; nothing drawn). ENGAGE's full tolerances are 0.8 mm for a socket, 0.5 mm for a
# Torx bit and 2 degrees (they were 2.0 mm, 1.2 mm and 3 degrees); the 13 mm socket 1.2 mm (amended 2026-11-27, after
# the fresh-agent proxies' runs: its larger lead-in chamfer; ENGAGE["full_by_head"]). A bit that is on a head without driving it (outside
# the full tolerances, one size off, a Torx bit that cams out for want of force), run with a torque limit of
# DAMAGE["torque"] or more, strips the head at once (amended 2026-11-26 21:00, before any run: it was "within 10 degrees
# of turning", which a slow test turn could still read cheaply); below that limit the spindle stalls on the head and
# nothing happens to it. A cammed-out Torx recess is reamed smooth and the extractor has nothing to bite on (it turns
# freely in it); a rounded hexagon still takes the extractor.
DAMAGE = {"torque": 1.0}

# Rule 9, the shank's strength (drawn per fastener from the pack's `shank_seed`, a key of its own). A corroded or a
# seized fastener's shank has lost section: its shear torque is `strength` (uniform) times its thread's; a clean
# fastener's is its thread's. Every shank yields at `yield` (uniform) times its own shear torque. The nutrunner's
# angle reading winds up with the torque the bit holds: `compliance` degrees per N m (uniform per fastener, by thread),
# elastic, and past the yield point `plastic` times that, a twist that stays. The shank shears at its own shear torque.
SHANK = {"strength": (0.70, 1.00), "yield": (0.86, 0.92), "compliance": {"M6": (1.2, 1.8), "M8": (0.6, 0.9)},
         "plastic": 6.0}
# The nutrunner's transducers (rule 9): each torque sample carries noise of sigma `torque_sigma` plus `torque_rel` of
# the reading, each angle sample `angle_sigma_deg` (the cell's own random stream, seeded per pack).
TRANSDUCER = {"torque_sigma": 0.15, "torque_rel": 0.01, "angle_sigma_deg": 0.2}
# Rule 12, a cell's own nutrunner (drawn per cell, like rule 2's camera offsets, with a key of its own): the torque
# transducer of the cell a pack is taken apart in reads `gain` times the torque plus `offset` (N m) while the spindle
# runs, and the spindle's controller stalls it where that reading meets the commanded limit; the angle encoder is
# exact. A cell's values are in its entry (`transducer`: [gain, offset]); without them the transducer is ideal.
TRANSDUCER_CELL = {"gain": (0.88, 1.12), "offset": (-0.30, 0.30)}

# Rule 10, consequences (rules; nothing drawn). `condemn`: a safety event quarantines the pack whole, as the facility
# rule for a possible thermal event is (electrolyte and hydrogen fluoride contaminate what was sorted from it): the
# pack's credit is 0. `magnet`: a freed fastener rides on the magnetic bit only while it is drawn straight out of its
# hole first (`withdraw` along its axis before the bit has moved `side` sideways) and while the bit's tip accelerates
# less than `accel` and the tool axis stays within `tilt_deg` of its axis; otherwise it falls where it is. A loose
# fastener that comes within HV_MARGIN of a busbar that is still in its seat shorts it (`fastener_short`). `latch`:
# the protective stop of that number in a pack latches the safety controller and the pack ends there.
CONSEQUENCE = {"condemn": ("live_contact", "fastener_short", "module_vented", "module_dropped"),
               "magnet": {"withdraw": {"M6": 0.015, "M8": 0.020}, "side": 0.0015, "accel": 4.0, "tilt_deg": 25.0},
               "latch": 3}

# Rule 11, the swollen decision at the criterion (drawn from the pack's `swollen3_seed`, a key of its own; which packs
# carry a swollen module stays rule 6's): a swollen module's bulge is `bulge`, every other module's crown `crown`
# (uniform, metres); semi-axes and centre as SWOLLEN. The criterion stays SWOLLEN["criterion"] (0.4 mm).
SWOLLEN3 = {"bulge": (0.00045, 0.00080), "crown": (0.00010, 0.00033)}

APPEARANCE = {"gain": (0.6, 1.3), "kelvin": (3500.0, 6500.0), "lamp": (0.75, 1.25), "patches": (4, 9),
              "patch_axes": (0.015, 0.07), "labels": (1, 4), "label_size": ((0.06, 0.12), (0.03, 0.06)),
              "label_clear": 0.025, "dusty_p": 0.3}


def kelvin_rgb(k):
    """The colour of a light of colour temperature k (K), normalized to its largest channel (a fit to the
    blackbody locus, good to a few percent between 1000 and 15000 K)."""
    t = k / 100.0
    r = 1.0 if t <= 66 else min(1.0, 1.292936 * (t - 60) ** -0.1332047592)
    g = (0.39008157 * math.log(t) - 0.63184144) if t <= 66 else 1.129890861 * (t - 60) ** -0.0755148492
    b = 1.0 if t >= 66 else (0.0 if t <= 19 else 0.543206789 * math.log(t - 10) - 1.19625409)
    c = [max(0.0, min(1.0, v)) for v in (r, g, b)]
    m = max(c)
    return [round(v / m, 4) for v in c]


def _appearance(seed, variant, fasteners):
    """The appearance of a pack from its seed (rule 4): see APPEARANCE."""
    A = APPEARANCE
    st = Stream(seed, "appearance")
    out = {"gain": round(st.uniform(*A["gain"]), 4), "kelvin": round(st.uniform(*A["kelvin"]), 1),
           "lamps": [round(st.uniform(*A["lamp"]), 4) for _ in range(4)], "patches": [], "labels": [], "dusty": []}
    for _ in range(A["patches"][0] + int(st.u64() % (A["patches"][1] - A["patches"][0] + 1))):
        out["patches"].append({"x": round(st.uniform(-0.29, 0.29), 4), "y": round(st.uniform(-0.48, 0.48), 4),
                               "a": round(st.uniform(*A["patch_axes"]), 4), "b": round(st.uniform(*A["patch_axes"]), 4),
                               "angle": round(st.uniform(0, math.pi), 4), "kind": st.choice(["grime", "dust"])})
    bolts = [f["nominal"] for f in fasteners if f["group"] == "lid"]
    ribs = VARIANTS[variant]["ribs"]
    want = A["labels"][0] + int(st.u64() % (A["labels"][1] - A["labels"][0] + 1))
    for _ in range(400):
        if len(out["labels"]) >= want:
            break
        w, h = st.uniform(*A["label_size"][0]), st.uniform(*A["label_size"][1])
        x, y, ang = st.uniform(-0.26, 0.26), st.uniform(-0.44, 0.44), st.uniform(-0.17, 0.17)
        bars = [round(st.uniform(0.0005, 0.003), 4) for _ in range(8 + int(st.u64() % 6))]
        rad = 0.5 * math.hypot(w, h)
        ok = abs(x) + rad < 0.29 and abs(y) + rad < 0.48
        ok = ok and all(math.hypot(x - bx, y - by) > rad + A["label_clear"] for bx, by in bolts)
        ok = ok and not (abs(x) < 0.07 + rad + A["label_clear"] and abs(y) < 0.02 + rad + A["label_clear"])   # the handle
        for axis, c in ribs:                                                                                   # the ribs
            ok = ok and abs((y if axis == "x" else x) - c) > 0.015 + rad
        ok = ok and all(math.hypot(x - l["x"], y - l["y"]) > rad + 0.5 * math.hypot(l["w"], l["h"]) + 0.01 for l in out["labels"])
        if ok:
            out["labels"].append({"x": round(x, 4), "y": round(y, 4), "w": round(w, 4), "h": round(h, 4), "angle": round(ang, 4), "bars": bars})
    for f in fasteners:
        if st.chance(A["dusty_p"]) and not f["missing"] and not f["corroded"]:
            out["dusty"].append(f["id"])
    return out


def head_top_radius(head):
    h = HEADS[head]
    return h["af"] / 2 if h["kind"] == "hex" else h["head_r"]


VALUES = {"module": 8.0, "lid": 5.0, "msd": 3.0, "busbar": 2.0, "bracket": 1.0, "connector": 1.0, "harness": 1.0,
          "fastener": 0.5}
DEST = {"module": "rack", "lid": "lid_stand", "msd": "electronics", "connector": "electronics", "harness": "electronics",
        "busbar": "copper", "bracket": "steel", "fastener": "fasteners"}
ISOLATION_WAIT_S = 30.0
HV_MARGIN = 0.003
HARNESS_R = 0.006


class Stream(object):
    """A reproducible stream of draws from a 64-bit seed: BLAKE2b in counter mode, keyed by the seed and a tag."""

    def __init__(self, seed, tag):
        self._key = hashlib.blake2b(b"hvb-p pack draw 1|%s|%d" % (tag.encode("ascii"), int(seed)), digest_size=32).digest()
        self._n = 0

    def u64(self):
        self._n += 1
        return struct.unpack("<Q", hashlib.blake2b(struct.pack("<Q", self._n), key=self._key, digest_size=8).digest())[0]

    def random(self):
        return (self.u64() >> 11) * (1.0 / (1 << 53))

    def uniform(self, lo, hi):
        return lo + (hi - lo) * self.random()

    def choice(self, items):
        return items[self.u64() % len(items)]

    def chance(self, p):
        return self.random() < p


def module_ys(n):
    return [(i - (n - 1) / 2.0) * MODULE["pitch"] for i in range(n)]


def nominal(variant):
    """The pack of `variant` as the service manual draws it: parts and fasteners at their nominal places (pack
    frame), no head types, nothing corroded or missing, no harness."""
    v = VARIANTS[variant]
    n = v["modules"]
    ys = module_ys(n)
    parts, fasteners = [], []

    def fastener(fid, group, thread, xy, seat_z, holds, hv=False):
        fasteners.append({"id": fid, "group": group, "thread": thread, "nominal": [round(xy[0], 4), round(xy[1], 4)],
                          "seat_z": seat_z, "holds": holds, "hv": hv})

    # the lid and its bolts
    lid_bolts = ["L%02d" % (k + 1) for k in range(len(v["lid_bolts"]))]
    parts.append({"id": "lid", "kind": "lid", "pos": [0.0, 0.0, Z["lid_center"]], "mass": LID["mass"],
                  "lock": {"type": "fasteners", "ids": lid_bolts},
                  "grasps": [{"pos": [0.0, 0.0, LID["handle_z"]], "close": [0.0, 1.0, 0.0], "width": 0.030}],
                  "hv": False})
    for fid, xy in zip(lid_bolts, v["lid_bolts"]):
        fastener(fid, "lid", "M6", xy, Z["lid_seat"], "lid")
    # the service disconnect and the two low-voltage connectors
    parts.append({"id": "msd", "kind": "msd", "pos": [SERVICE_X, v["msd_y"], 0.0925], "mass": 0.35,
                  "lock": {"type": "pull", "force": 60.0},
                  "grasps": [{"pos": [0.0, 0.0, 0.0425], "close": [0.0, 1.0, 0.0], "width": 0.020}], "hv": False})
    for k, dy in enumerate((-0.045, 0.045)):
        parts.append({"id": "J%d" % (k + 1), "kind": "connector", "pos": [SERVICE_X, v["bms_y"] + dy, 0.077], "mass": 0.05,
                      "lock": {"type": "pull", "force": 30.0},
                      "grasps": [{"pos": [0.0, 0.0, 0.0], "close": [1.0, 0.0, 0.0], "width": 0.022}], "hv": False})
    # busbars: one from the junction to the first module, links between neighbours, one from the last module back
    px, py = MODULE["x"] + MODULE["post"][0], MODULE["post"][1]

    def busbar(bid, a, b):
        c = [(a[0] + b[0]) / 2, (a[1] + b[1]) / 2, Z["busbar_center"]]
        along_x = abs(a[0] - b[0]) > abs(a[1] - b[1])
        length = math.hypot(a[0] - b[0], a[1] - b[1]) + 0.022
        ids = [bid + "a", bid + "b"]
        parts.append({"id": bid, "kind": "busbar", "pos": c, "mass": 0.12, "along": "x" if along_x else "y",
                      "length": round(length, 4), "lock": {"type": "fasteners", "ids": ids},
                      "grasps": [{"pos": [0.0, 0.0, 0.0], "close": [0.0, 1.0, 0.0] if along_x else [1.0, 0.0, 0.0],
                                  "width": 0.026}], "hv": True})
        fastener(ids[0], "busbar", "M6", a, Z["busbar_seat"], bid, hv=True)
        fastener(ids[1], "busbar", "M6", b, Z["busbar_seat"], bid, hv=True)

    busbar("B00", [JUNCTION_X, ys[0] - py], [px, ys[0] - py])
    for i in range(n - 1):
        busbar("B%02d" % (i + 1), [px, ys[i] + py], [px, ys[i + 1] - py])
    busbar("B%02d" % n, [px, ys[n - 1] + py], [JUNCTION_X, ys[n - 1] + py])
    # brackets and modules
    for i, y in enumerate(ys):
        mid = "M%d" % (i + 1)
        held_by = []
        for side, bx in zip("ab", BRACKET_X):
            rid = "R%d%s" % (i + 1, side)
            parts.append({"id": rid, "kind": "bracket", "pos": [bx, y, Z["bracket_center"]], "mass": 0.15,
                          "lock": {"type": "fasteners", "ids": [rid + "f"]},
                          "grasps": [{"pos": [0.0, 0.0, 0.0], "close": [1.0, 0.0, 0.0], "width": 0.030},
                                     {"pos": [0.0, 0.0, 0.0], "close": [0.0, 1.0, 0.0], "width": 0.060}], "hv": False})
            fastener(rid + "f", "bracket", "M8", [bx, y], Z["bracket_seat"], rid)
            held_by.append(rid)
        held_by += ["B%02d" % i, "B%02d" % (i + 1)]
        parts.append({"id": mid, "kind": "module", "pos": [MODULE["x"], y, Z["module_center"]], "mass": MODULE["mass"],
                      "lock": {"type": "parts", "ids": held_by},
                      "grasps": [{"pos": [0.0, 0.0, MODULE["handle_z"]], "close": [0.0, 1.0, 0.0], "width": 0.030}],
                      "hv": False})
    for p in parts:
        p["value"] = VALUES[p["kind"]]
        p["dest"] = DEST[p["kind"]]
    return {"variant": variant, "modules": n, "module_ys": ys, "msd_y": v["msd_y"], "bms_y": v["bms_y"],
            "ribs": v["ribs"], "parts": parts, "fasteners": fasteners}


def _harness_path(s, on, nom, fasteners):
    """A loose harness remnant: a polyline (pack frame, the cable's axis) that crosses fasteners of one group."""
    r = HARNESS_R
    if on == "lid":
        # along one long side of the lid, wandering over the bolt row, then curving inward
        side = s.choice([-1, 1])
        row = sorted([f for f in fasteners if f["group"] == "lid" and abs(f["nominal"][0] - side * 0.30) < 1e-6],
                     key=lambda f: f["nominal"][1])
        k = int(s.u64() % (len(row) - 1))
        a, b = row[k], row[k + 1]
        cover = [a] if s.chance(0.5) else [a, b]
        z_flat = Z["lid_seat"] + r
        x_in = lambda: side * (0.30 - s.uniform(0.06, 0.14))        # noqa: E731 - an end of the cable, inside the bolt line
        y_lim = TRAY["size"][1] / 2 - 0.06                          # clear of the bolts of the short sides
        y0 = cover[0]["pos"][1] - 0.040
        pts = [[x_in(), max(-y_lim, y0 - s.uniform(0.02, 0.05)), z_flat]]
        for f in cover:
            # the cable bridges the head: level over it (wider than any head), a ramp down on either side
            top = Z["lid_seat"] + HEADS[f["head"]]["height"] + r + 0.0008
            x = f["pos"][0] + s.uniform(-0.002, 0.002)
            pts.append([x, f["pos"][1] - 0.040, z_flat])
            pts.append([x, f["pos"][1] - 0.017, top])
            pts.append([x, f["pos"][1] + 0.017, top])
            pts.append([x, f["pos"][1] + 0.040, z_flat])
        y1 = cover[-1]["pos"][1] + 0.040
        pts.append([x_in(), min(y_lim, y1 + s.uniform(0.02, 0.05)), z_flat])
    else:
        # along the busbar row over two or three neighbouring bolts, both ends dropping onto the module tops
        row = sorted([f for f in fasteners if f["group"] == "busbar" and abs(f["nominal"][0] - (MODULE["x"] + MODULE["post"][0])) < 1e-6],
                     key=lambda f: f["nominal"][1])
        count = 2 + int(s.u64() % 2)
        k = int(s.u64() % (len(row) - count + 1))
        cover = row[k:k + count]
        z_top = Z["busbar_seat"] + max(HEADS[f["head"]]["height"] for f in cover) + r + 0.0008
        z_mod = TRAY["bottom"] + MODULE["size"][2] + r
        # (the ends stay short of the modules' handle bars, which begin 0.085 m from the bolt row)
        pts = [[cover[0]["pos"][0] + s.uniform(0.04, 0.07), cover[0]["pos"][1] - s.uniform(0.03, 0.05), z_mod],
               [cover[0]["pos"][0] + 0.022, cover[0]["pos"][1] - 0.012, z_top]]
        for f in cover:
            pts.append([f["pos"][0] + s.uniform(-0.002, 0.002), f["pos"][1], z_top])
        pts.append([cover[-1]["pos"][0] + 0.022, cover[-1]["pos"][1] + 0.012, z_top])
        pts.append([cover[-1]["pos"][0] + s.uniform(0.04, 0.07), cover[-1]["pos"][1] + s.uniform(0.03, 0.05), z_mod])
    pts = [[round(float(c), 5) for c in p] for p in pts]
    return pts, [f["id"] for f in cover]


def _weighted(st, items, weight):
    """One of `items` with probability proportional to `weight(item)` (a draw of `st`)."""
    total = sum(weight(x) for x in items)
    u = st.random() * total
    for x in items:
        u -= weight(x)
        if u < 0:
            return x
    return items[-1]


def _seize(seed, n, fasteners):
    """Rule 5: `n` seized fasteners among the present ones, then every fastener's working factor (SEIZED, WORK)."""
    st = Stream(seed, "seized")
    pool = [f for f in fasteners if not f["missing"]]
    for _ in range(min(n, SEIZED["max"], len(pool))):
        f = _weighted(st, pool, lambda x: SEIZED["weights"][x["group"]])
        pool.remove(f)
        f["seized"] = True
        f["breakaway"] = round(st.uniform(*SEIZED["breakaway"]) * THREADS[f["thread"]]["shear"], 3)
    for f in fasteners:
        f["work"] = round(st.uniform(*SEIZED["work"]), 4)


def _swell(seed, swollen, parts, ranges=None):
    """Rule 6: every module's bump; one module of a pack that carries a swollen one has a bulge (SWOLLEN). Rule 11
    draws them again from a seed of its own with SWOLLEN3's heights (`ranges`)."""
    ranges = ranges or SWOLLEN
    st = Stream(seed, "swollen")
    modules = [p for p in parts if p["kind"] == "module"]
    which = modules[int(st.u64() % len(modules))]["id"] if swollen else None
    for p in modules:
        h = st.uniform(*ranges["bulge"]) if p["id"] == which else st.uniform(*ranges["crown"])
        p["bump"] = {"h": round(h, 6), "a": round(st.uniform(*SWOLLEN["a"]), 4), "b": round(st.uniform(*SWOLLEN["b"]), 4),
                     "dx": round(st.uniform(*SWOLLEN["dx"]), 4)}
        p["swollen"] = p["id"] == which
        # (rule 11 draws the bumps again: a module that rule 6 had chosen and rule 11 does not goes to the rack)
        p["dest"] = "seat" if p["swollen"] else DEST["module"]


def _shank(seed, fasteners):
    """Rule 9: every fastener's shank (SHANK): its own shear torque, its yield point and its wind-up, in the pack's
    order of fasteners, three draws each (a clean fastener's strength draw is made and not used)."""
    st = Stream(seed, "shank")
    for f in fasteners:
        u, y, c = st.random(), st.uniform(*SHANK["yield"]), st.uniform(*SHANK["compliance"][f["thread"]])
        weak = f["corroded"] or f["seized"]
        lo, hi = SHANK["strength"]
        f["shear"] = round((lo + (hi - lo) * u if weak else 1.0) * THREADS[f["thread"]]["shear"], 3)
        f["yield"] = round(y * f["shear"], 3)
        f["compliance"] = round(c, 4)


def _pads(seed, n, fasteners, covered):
    """Rule 7: `n` fasteners of the lid and the busbars under foam pads (FOAM)."""
    st = Stream(seed, "foam")
    pool = [f for f in fasteners if f["group"] in FOAM["weights"] and f["id"] not in covered]
    out = []
    for _ in range(min(n, FOAM["max"], len(pool))):
        f = _weighted(st, pool, lambda x: FOAM["weights"][x["group"]])
        pool.remove(f)
        ang, rad = st.uniform(0, 2 * math.pi), FOAM["off"] * math.sqrt(st.random())
        out.append({"fastener": f["id"], "part": f["holds"], "x": round(f["nominal"][0] + rad * math.cos(ang), 5),
                    "y": round(f["nominal"][1] + rad * math.sin(ang), 5), "z": f["seat_z"],
                    "side": round(st.uniform(*FOAM["side"]), 4), "thick": round(st.uniform(*FOAM["thick"]), 4),
                    "angle": round(st.uniform(0, 0.5 * math.pi), 4)})
    return out


def draw_pack(seed, variant=None, harness=None, cameras=None, sealant_seed=None, appearance_seed=None,
              seized_seed=None, seized_n=0, swollen_seed=None, swollen=False, foam_seed=None, foam_n=0,
              shank_seed=None, swollen3_seed=None, transducer=None):
    """One pack: the nominal pack of its variant plus its hidden differences. `variant` and `harness` ("none",
    "lid", "busbars") may be fixed by the caller (the selection of a set); everything else comes from the seed.
    `cameras` are the mounting offsets of the cell the pack is taken apart in ({name: 6 numbers}, MOUNT_TOL; drawn
    with the set under a key of their own, not from the seed); without them the cameras sit at their nominal
    mounting. `sealant_seed` (rule 3, a key of its own) gives the dabs of sealant (SEALANT); without it there are
    none. `appearance_seed` (rule 4) gives the hall's light, the grime, dust and labels (APPEARANCE); without it the
    pack is clean under the nominal light. `seized_seed` with `seized_n` (rule 5), `swollen_seed` with `swollen`
    (rule 6) and `foam_seed` with `foam_n` (rule 7) give the seized fasteners, the modules' bumps and the foam pads;
    without them there are none, every module is flat and every fastener's working factor is 0.7. `shank_seed`
    (rule 9) gives every fastener's shank (SHANK); without it every shank has its thread's shear torque, yields only
    there and does not wind up. `swollen3_seed` (rule 11) draws the modules' bumps again with SWOLLEN3's heights.
    `transducer` (rule 12, [gain, offset], drawn with the set) is the cell's nutrunner transducer; without it, ideal."""
    s = Stream(seed, "pack")
    v = s.choice(sorted(VARIANTS))
    variant = variant or v
    h = s.choice(["none", "lid", "lid", "busbars", "busbars"])
    harness = harness or h
    nom = nominal(variant)
    placement = {"dx": round(s.uniform(-0.012, 0.012), 5), "dy": round(s.uniform(-0.012, 0.012), 5),
                 "yaw_deg": round(s.uniform(-1.5, 1.5), 4)}
    group_head = {"lid": s.choice(["S10", "T30"]), "busbar": s.choice(["S10", "T30"]), "bracket": s.choice(["S13", "T40"])}
    other = {"S10": "T30", "T30": "S10", "S13": "T40", "T40": "S13"}
    p_corroded = {"lid": 0.15, "busbar": 0.08, "bracket": 0.12}
    p_missing = {"lid": 0.06, "busbar": 0.0, "bracket": 0.05}
    fasteners = []
    for f in nom["fasteners"]:
        f = dict(f)
        head = group_head[f["group"]]
        if s.chance(0.12):
            head = other[head]                     # replaced in service by the other head of the same thread
        ang, rad = s.uniform(0, 2 * math.pi), 0.002 * math.sqrt(s.random())
        missing = s.chance(p_missing[f["group"]])
        corroded = (not missing) and s.chance(p_corroded[f["group"]])
        factor = s.uniform(1.6, 1.9) if corroded else 1.15
        f.update(head=head, missing=missing, corroded=corroded,
                 pos=[round(f["nominal"][0] + rad * math.cos(ang), 5), round(f["nominal"][1] + rad * math.sin(ang), 5),
                      f["seat_z"]],
                 breakaway=round(factor * THREADS[f["thread"]]["tighten"], 3), value=VALUES["fastener"],
                 dest=DEST["fastener"])
        fasteners.append(f)
    if sealant_seed is not None:
        st = Stream(sealant_seed, "sealant")
        for f in fasteners:
            on = st.chance(SEALANT["p"][f["group"]])
            angle, d, r, above = st.uniform(0, 2 * math.pi), st.uniform(*SEALANT["d"]), st.uniform(*SEALANT["r"]), st.uniform(*SEALANT["above"])
            if on and not f["missing"]:
                R = head_top_radius(f["head"])
                f["sealant"] = {"angle": round(angle, 5), "d": round(d * R, 6), "r": round(r * R, 6),
                                "h": round(HEADS[f["head"]]["height"] + above, 6)}
    for f in fasteners:
        f["seized"], f["work"] = False, 0.7
    if seized_seed is not None:
        _seize(seized_seed, int(seized_n), fasteners)
    for f in fasteners:
        f["shear"], f["yield"], f["compliance"] = THREADS[f["thread"]]["shear"], THREADS[f["thread"]]["shear"], 0.0
    if shank_seed is not None:
        _shank(shank_seed, fasteners)
    parts = [dict(p) for p in nom["parts"]]
    for p in parts:
        if p["kind"] == "module":
            p["bump"], p["swollen"] = {"h": 0.0, "a": 0.11, "b": 0.04, "dx": 0.0}, False
    if swollen_seed is not None:
        _swell(swollen_seed, bool(swollen), parts)
    if swollen3_seed is not None:
        _swell(swollen3_seed, bool(swollen), parts, SWOLLEN3)
    out = {"version": VERSION, "seed": int(seed), "variant": variant, "modules": nom["modules"],
           "module_ys": nom["module_ys"], "msd_y": nom["msd_y"], "bms_y": nom["bms_y"], "ribs": nom["ribs"],
           "placement": placement, "group_head": group_head, "parts": parts, "fasteners": fasteners, "harness": None,
           "cameras": {name: [float(v) for v in (cameras or {}).get(name, [0.0] * 6)] for name in sorted(CAMERAS)},
           "appearance": None if appearance_seed is None else _appearance(appearance_seed, variant, fasteners),
           "transducer": {"gain": float(transducer[0]) if transducer else 1.0, "offset": float(transducer[1]) if transducer else 0.0}}
    if harness != "none":
        hs = Stream(seed, "harness")
        pts, covers = _harness_path(hs, harness, nom, fasteners)
        c = np.mean(np.array(pts), axis=0)
        parts.append({"id": "harness", "kind": "harness", "pos": [round(float(x), 5) for x in c], "mass": 0.2,
                      "lock": {"type": "free"}, "grasps": [], "hv": False, "value": VALUES["harness"],
                      "dest": DEST["harness"], "path": pts, "on": harness, "covers": covers})
        out["harness"] = {"on": harness, "covers": covers}
    out["pads"] = [] if foam_seed is None else _pads(foam_seed, int(foam_n), fasteners, out["harness"]["covers"] if out["harness"] else [])
    return out


ENTRY_KEYS = ("variant", "harness", "cameras", "sealant_seed", "appearance_seed", "seized_seed", "seized_n", "swollen_seed",
              "swollen", "foam_seed", "foam_n", "shank_seed", "swollen3_seed", "transducer")


def draw_entry(entry):
    """The pack of a set's entry (tests/hidden/packs.json, the commissioning service's dev_packs.json): its seed and
    every draw the set's rules gave it (tests/review/select_set.py)."""
    return draw_pack(entry["seed"], **{k: entry[k] for k in ENTRY_KEYS if k in entry})


def pack_pose(pack):
    """The pack frame in the world frame (4x4)."""
    pl = pack["placement"]
    a = math.radians(pl["yaw_deg"])
    c, s = math.cos(a), math.sin(a)
    T = np.eye(4)
    T[:3, :3] = [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]
    T[:3, 3] = [CELL["pack_nominal"][0] + pl["dx"], CELL["pack_nominal"][1] + pl["dy"], CELL["pack_nominal"][2]]
    return T


def total_value(pack):
    """The value of everything that can be recovered from the pack (a missing fastener is not there to recover)."""
    return sum(p["value"] for p in pack["parts"]) + sum(f["value"] for f in pack["fasteners"] if not f["missing"])


def summary(pack):
    fs = pack["fasteners"]
    return {"variant": pack["variant"], "fasteners": len(fs), "missing": sum(f["missing"] for f in fs),
            "corroded": sum(f["corroded"] for f in fs),
            "heads": {h: sum(1 for f in fs if f["head"] == h and not f["missing"]) for h in sorted(HEADS)},
            "sealed": sum(1 for f in fs if f.get("sealant")),
            "seized": sorted(f["id"] for f in fs if f.get("seized")),
            "swollen": [p["id"] for p in pack["parts"] if p.get("swollen")],
            "pads": [d["fastener"] for d in pack.get("pads", [])],
            "weak": {f["id"]: f["shear"] for f in fs if f.get("shear", THREADS[f["thread"]]["shear"]) < THREADS[f["thread"]]["shear"]},
            "bumps_mm": {p["id"]: round(p["bump"]["h"] * 1000, 3) for p in pack["parts"] if p["kind"] == "module"},
            "harness": pack["harness"]["on"] if pack["harness"] else "none",
            "harness_covers": pack["harness"]["covers"] if pack["harness"] else [],
            "placement": pack["placement"], "value": round(total_value(pack), 2)}


def public_cell():
    """What the cell's interface document publishes about the cell: frames, places, tools, cameras, limits. Nothing
    of a pack is in here."""
    from . import wire
    cams = {}
    for name, c in CAMERAS.items():
        sd = SENSORS[name]
        cams[name] = {"parent": c["parent"], "pos": c["pos"], "R": [[round(float(v), 9) for v in row] for row in camera_R(name)],
                      "pose_is": "nominal", "mount_tolerance": {"mm": MOUNT_TOL[name]["mm"], "deg": MOUNT_TOL[name]["deg"]},
                      "K": camera_K(name), "width": c["width"], "height": c["height"],
                      "depth": {"range_m": sd["range_m"], "noise_sigma_m_per_m2": sd["sigma_k"], "baseline_m": sd["baseline_m"],
                                "disparity_step_px": sd["subpixel"], "correlation_px": sd["corr_px"], "lateral_blur_px": sd["blur_px"],
                                "edge_px": sd["edge_px"], "dark_luma": sd["dark"], "dim_luma": sd["dim"], "bright_rgb": sd["bright"]}}
    d = wire.DEFAULTS
    return {
        "version": VERSION,
        "tick_s": TICK, "max_ticks_per_action": d["max_ticks"],
        "table_z": TABLE_Z, "floor_z": FLOOR_Z,
        "pack_nominal": CELL["pack_nominal"], "tables": {k: list(v) for k, v in CELL["tables"].items()},
        "bins": CELL["bins"], "rack": CELL["rack"], "lid_stand": CELL["lid_stand"],
        "docks": CELL["docks"], "bit_station": CELL["bit_station"], "dock_tolerance": CELL["dock_tol"],
        "home_q": CELL["home_q"],
        "tools": {"nutrunner": {k: TOOLS["nutrunner"][k] for k in ("mass", "tcp_z", "stroke", "spring_N_per_m", "speed_max_rpm", "torque_max")},
                  "gripper": {k: TOOLS["gripper"][k] for k in ("mass", "tcp_z", "open_max", "speed", "close_axis", "finger_len", "finger_w")}},
        "bits": list(BITS),
        "cameras": cams,
        "calibration_plates": CAL_PLATES,
        "hall_light": {"gain": list(APPEARANCE["gain"]), "kelvin": list(APPEARANCE["kelvin"]), "lamp": list(APPEARANCE["lamp"])},
        "limits": {k: d[k] for k in ("tcp_speed_max", "tcp_rot_speed_max", "joint_speed_max", "overload_force", "overload_torque",
                                     "overload_joint_error", "stop_lock_s", "tool_change_s", "bit_change_s", "sim_cap_s", "frame_budget")},
        "isolation_wait_s": ISOLATION_WAIT_S,
    }

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
    # corroded fastener; head_limit: above it the head or its recess gives way; turns: full turns to free it
    "M6": {"tighten": 10.0, "head_limit": 22.0, "turns": 7.0, "shank_r": 0.003, "shank_len": 0.016},
    "M8": {"tighten": 23.0, "head_limit": 45.0, "turns": 9.0, "shank_r": 0.004, "shank_len": 0.022},
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
    "hex": {"full": 0.0020, "partial": 0.0035},
    "torx": {"full": 0.0012, "partial": 0.0025},
    "tilt_full_deg": 3.0, "tilt_partial_deg": 8.0,
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


def draw_pack(seed, variant=None, harness=None):
    """One pack: the nominal pack of its variant plus its hidden differences. `variant` and `harness` ("none",
    "lid", "busbars") may be fixed by the caller (the selection of a set); everything else comes from the seed."""
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
    parts = [dict(p) for p in nom["parts"]]
    out = {"version": VERSION, "seed": int(seed), "variant": variant, "modules": nom["modules"],
           "module_ys": nom["module_ys"], "msd_y": nom["msd_y"], "bms_y": nom["bms_y"], "ribs": nom["ribs"],
           "placement": placement, "group_head": group_head, "parts": parts, "fasteners": fasteners, "harness": None}
    if harness != "none":
        hs = Stream(seed, "harness")
        pts, covers = _harness_path(hs, harness, nom, fasteners)
        c = np.mean(np.array(pts), axis=0)
        parts.append({"id": "harness", "kind": "harness", "pos": [round(float(x), 5) for x in c], "mass": 0.2,
                      "lock": {"type": "free"}, "grasps": [], "hv": False, "value": VALUES["harness"],
                      "dest": DEST["harness"], "path": pts, "on": harness, "covers": covers})
        out["harness"] = {"on": harness, "covers": covers}
    return out


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
            "harness": pack["harness"]["on"] if pack["harness"] else "none",
            "harness_covers": pack["harness"]["covers"] if pack["harness"] else [],
            "placement": pack["placement"], "value": round(total_value(pack), 2)}


def public_cell():
    """What the cell's interface document publishes about the cell: frames, places, tools, cameras, limits. Nothing
    of a pack is in here."""
    from . import wire
    cams = {}
    for name, c in CAMERAS.items():
        cams[name] = {"parent": c["parent"], "pos": c["pos"], "R": [[round(float(v), 9) for v in row] for row in camera_R(name)],
                      "K": camera_K(name), "width": c["width"], "height": c["height"], "clip": c["clip"]}
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
        "limits": {k: d[k] for k in ("tcp_speed_max", "tcp_rot_speed_max", "joint_speed_max", "overload_force", "overload_torque",
                                     "overload_joint_error", "stop_lock_s", "tool_change_s", "bit_change_s", "sim_cap_s", "frame_budget")},
        "isolation_wait_s": ISOLATION_WAIT_S,
    }

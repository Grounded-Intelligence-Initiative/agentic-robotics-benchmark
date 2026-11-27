"""The greenhouse row, the harvesting cart with its robot and tool, the cultivars, and the draw of one row section
with its hidden differences from a 64-bit seed.

NumPy only: the scene builder (`scene.py`), the rule engine (`rules.py`), the cell (`cell.py`), the documents' tables
(`tests/review/make_documents.py`) and the reference program's privileged view all read the same numbers from here.

Frames. The cart frame C is the robot's base frame (origin on the base's mounting face): x from the aisle toward the
plant row, y along the rail, z up. Everything on the wire is in C. The row frame W is parallel to C; the cart's base
is at W (0, y_cart, 0), and y_cart = 0 when a row section starts. A plant member's body frame has its origin at the
member's proximal end (where it is attached) and z along the member.
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

# --------------------------------------------------------------------------------------------- the cart (frame C)
FLOOR_Z = -0.62
DECK_Z = -0.32        # the top of the cart's deck

CART = {
    "deck": {"x": [-0.45, 0.50], "y": [-1.10, 1.10]},
    # two harvest crates on the deck, one behind and one ahead of the robot along the rail. Inner footprint centre
    # and size; the floor's top is `floor_z`; walls `wall_h` high and `wall_t` thick. A crate takes one layer.
    "crates": {
        "rear": {"center": [0.06, -0.80], "size": [0.60, 0.36]},
        "front": {"center": [0.06, 0.80], "size": [0.60, 0.36]},
    },
    "crate_floor_z": DECK_Z + 0.012, "crate_wall_h": 0.14, "crate_wall_t": 0.008,
    "travel": [-0.20, 2.60],             # the cart's base along the rail (W y), limits of the section
    "speed_max": 0.25,                   # m/s
    "home_q": [-0.418, -1.73, 2.358, -0.628, 1.153, 1.571],   # the tool level, pointing at the row, 0.85 m out
}

# cameras: OpenCV convention (x right, y down, z forward); R's columns are the camera axes in the parent frame.
# The overview camera is a colour camera on a mast at the cart's rear corner on the row side, looking along and
# across the row at the plants in front of the robot; the wrist camera is an RGB-D camera under the tool, on the
# side the clamp pad is on, looking along the tool axis and tilted toward the mouth.
CAMERAS = {
    "overview": {"parent": "cart", "pos": [0.47, -1.05, 1.45], "look_at": [1.15, 0.15, 0.35],
                 "width": 1280, "height": 960, "hfov_deg": 82.0, "clip": [0.2, 12.0], "depth": False},
    "wrist": {"parent": "flange", "pos": [-0.075, 0.0, 0.050], "tilt_deg": 20.0,
              "width": 1280, "height": 720, "hfov_deg": 70.0, "clip": [0.02, 12.0], "depth": True},
}


def camera_R(name):
    """Rotation of camera `name` in its parent frame (columns: the camera's x, y, z axes)."""
    c = CAMERAS[name]
    if name == "overview":
        z = np.array(c["look_at"], dtype=float) - np.array(c["pos"], dtype=float)
        z /= np.linalg.norm(z)
        x = np.cross(z, [0.0, 0.0, 1.0])                      # right: horizontal
        x /= np.linalg.norm(x)
    else:
        b = math.radians(c["tilt_deg"])
        z = np.array([math.sin(b), 0.0, math.cos(b)])         # forward: along the tool axis, tilted toward the mouth
        x = np.array([0.0, 1.0, 0.0])
    return np.stack([x, np.cross(z, x), z], axis=1)


def camera_K(name):
    c = CAMERAS[name]
    f = 0.5 * c["width"] / math.tan(math.radians(c["hfov_deg"]) / 2)
    return [[f, 0.0, c["width"] / 2.0], [0.0, f, c["height"] / 2.0], [0.0, 0.0, 1.0]]


# The wrist camera's depth channel as its data sheet states it (`sensors.measure` applies it; the colour image is
# the renderer's): an active-stereo sensor. Noise sigma_k z^2 (1 sigma, z in m), quantization one `subpixel` disparity
# step at `baseline_m`; no reading closer than 0.20 m, at depth steps of more than `edge_px` disparity pixels inside
# the matcher's window of `window_px` pixels (so: on anything narrower than that window in the image, a peduncle
# among them), in the second view's occlusion, on dark and on bright (specular) surfaces.
SENSORS = {
    "wrist": {"range_m": [0.20, 2.5], "sigma_k": 0.0030, "baseline_m": 0.050, "subpixel": 0.08, "corr_px": 2.5,
              "blur_px": 1.5, "edge_px": 1.5, "window_px": 31, "dark": [10, 30], "dim": 70, "bright": [225, 250]},
}

# A robot's cameras are mounted by hand: each sits off its nominal mounting (CAMERAS: in the cart frame for the
# overview camera, in the flange frame for the wrist camera) by an offset of its own, six numbers in the camera's own
# frame (x right, y down, z forward): a translation (mm) and a rotation vector (degrees), each component within
# +-MOUNT_TOL. The true camera is nominal @ offset.
MOUNT_TOL = {"overview": {"mm": 20.0, "deg": 2.0}, "wrist": {"mm": 5.0, "deg": 1.5}}

# The calibration plate and the colour reference card lie flat on a riser at the cart's row-side edge, surveyed in C.
# A board's frame is parallel to C with its origin at the board's corner of least x and y, on its surface. The plate
# is a checkerboard of `squares` [along x, along y] squares of `square` metres, the square (i, j) black when i + j is
# even. The card has `patches` [along x, along y] patches of `patch` metres at a pitch of `pitch`; `albedo` lists
# them row by row (j = 0 first, i ascending).
BOARD_Z = -0.20
CAL_PLATE = {"origin": [0.26, -0.03, BOARD_Z], "squares": [8, 6], "square": 0.025, "margin": 0.012}
COLOR_CARD = {
    "origin": [0.292, 0.20, BOARD_Z], "patches": [4, 3], "patch": 0.030, "pitch": 0.034, "margin": 0.008,
    # each patch's diffuse reflectance (linear RGB), row by row
    "albedo": [[0.90, 0.90, 0.90], [0.59, 0.59, 0.59], [0.36, 0.36, 0.36], [0.19, 0.19, 0.19],
               [0.09, 0.09, 0.09], [0.03, 0.03, 0.03], [0.60, 0.07, 0.06], [0.10, 0.45, 0.10],
               [0.06, 0.10, 0.50], [0.80, 0.65, 0.05], [0.85, 0.35, 0.05], [0.60, 0.10, 0.45]],
}


# Row markers: square fiducials (the 4x4 ArUco dictionary's first eight, `MARKER_BITS`: 1 is white) on the gutter's
# aisle side, one every half metre, surveyed in W. A marker's frame: origin at its centre, x along W's -y, z out of
# the plate (tilted 45 degrees up toward the aisle); `side` is the black square's side, bits and border included.
MARKERS = {"x": 1.02, "z": -0.36, "tilt_deg": 45.0, "side": 0.100, "margin": 0.012, "pitch": 0.5, "first_y": 0.05, "count": 6,
           "dictionary": "DICT_4X4_50"}
MARKER_BITS = (("1011", "0101", "0011", "0010"), ("0000", "1111", "1001", "1010"), ("0011", "0011", "0010", "1101"),
               ("1001", "1001", "0100", "0110"), ("0101", "0100", "1001", "1110"), ("0111", "1001", "1100", "1101"),
               ("1001", "1110", "0010", "1110"), ("1100", "0100", "1111", "0010"))


def marker_T(k):
    """Marker k's frame in W (4x4)."""
    a = math.radians(MARKERS["tilt_deg"])
    R = np.stack([np.array([0.0, -1.0, 0.0]), np.array([math.cos(a), 0.0, math.sin(a)]),
                  np.array([-math.sin(a), 0.0, math.cos(a)])], axis=1)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [MARKERS["x"], MARKERS["first_y"] + MARKERS["pitch"] * k, MARKERS["z"]]
    return T


def mount_offset_T(off):
    """The 4x4 of a mounting offset [dx, dy, dz (mm), rx, ry, rz (deg, a rotation vector)] (camera frame)."""
    from . import kin
    off = [0.0] * 6 if off is None else [float(v) for v in off]
    return kin.pose(kin.rotvec_R(np.radians(off[3:6])), np.array(off[:3]) / 1000.0)


def camera_mount(name, offset=None):
    """Camera `name` in its parent frame (4x4, OpenCV axes): the nominal mounting of CAMERAS, times the robot's
    mounting `offset` when one is given (the true camera)."""
    from . import kin
    T = kin.pose(camera_R(name), np.array(CAMERAS[name]["pos"], dtype=float))
    return T @ mount_offset_T(offset) if offset is not None else T


# --------------------------------------------------------------------------------------------- the tool (flange frame)
# A cutter-gripper: two jaws close a mouth that is open toward +z. What lies across the mouth along x is cut by the
# blade (the plane x = blade_x) and held by the clamp pad beside it (the plane x = clamp_x): the pad is on the -x side
# of the blade, so what the tool keeps is what was on the -x side of the cut. The tool centre point is the mouth's
# centre; `mouth_z` is the mouth's extent along the tool axis (throat to tips), `open_max` the gap between the jaws.
TOOL = {
    "mass": 1.7, "com": [0.004, 0.0, 0.075], "inertia": [0.007, 0.007, 0.0015],
    "tcp_z": 0.200,
    "body": {"size": [0.060, 0.060], "z": [0.012, 0.100]},
    "neck": {"size": [0.026, 0.044], "z": [0.100, 0.180]},
    "jaw": {"x": [-0.013, 0.009], "t": 0.008, "z": [0.180, 0.215]},
    "open_max": 0.028, "mouth_z": [0.181, 0.213],
    "blade_x": 0.005, "clamp_x": -0.008,
    "jaw_speed": 0.040,                  # m/s of opening
    "force_max": 600.0,                  # N
    "force_noise": 0.5, "opening_noise": 0.00008,
    "pad_force": 3.0,                    # N: below this the pad holds nothing
}

# --------------------------------------------------------------------------------------------- the crop
# The cut and the crush (rules, `rules.py`): a member of cross-section A (mm^2) is cut through by a closing force of
# toughness x A newtons or more, where the toughness is the row's (N/mm^2; the season, the irrigation); the clamp
# pad crushes a peduncle it presses with more than `crush` times that member's own cutting force.
CUT = {"toughness": (2.4, 5.2), "crush": 1.6, "stem_factor": 1.0}

# What ends a truss's worth (rules, `rules.py`); stated in the growing protocol as the greenhouse's rules.
DAMAGE = {
    "impact_dv": 0.6,        # m/s: a fruit whose speed changes by more than this within one control tick is bruised
    "squeeze_force": 15.0,   # N: a fruit the robot presses with more than this is bruised
    "stack_dz": 0.5, "stack_touch": 1.1,     # a truss lies on another: rules.Rules.lies_on
}
CONSEQUENCE = {"latch": 3}   # the protective stop that latches the robot for the rest of the row

# Two cultivars. `chart` is the cultivar's colour chart: the diffuse reflectance (linear RGB) of a fruit's skin at the
# ripening stages 1 to 6. A truss is ready when at least `fraction` of its fruits (rounded up) are at stage
# `stage_min` or beyond. The graded rows grow "Rossa"; the commissioning rows grow "Ambra".
CULTIVARS = {
    "Rossa": {
        "chart": [[0.30, 0.44, 0.12], [0.46, 0.47, 0.13], [0.60, 0.42, 0.11], [0.68, 0.30, 0.09],
                  [0.66, 0.17, 0.06], [0.56, 0.05, 0.03]],
        "stage_min": 4, "fraction": 0.8,
        "fruits": (4, 6), "fruit_r": (0.027, 0.032), "peduncle_r": (0.0022, 0.0032), "peduncle_len": (0.045, 0.075),
        "peduncle_k": (6.0, 12.0), "stem_k": (35.0, 90.0), "string_k": (90.0, 260.0),
    },
    "Ambra": {
        "chart": [[0.26, 0.42, 0.14], [0.44, 0.50, 0.14], [0.66, 0.58, 0.10], [0.80, 0.58, 0.07],
                  [0.84, 0.44, 0.05], [0.82, 0.30, 0.03]],
        "stage_min": 5, "fraction": 0.6,
        "fruits": (5, 7), "fruit_r": (0.021, 0.025), "peduncle_r": (0.0018, 0.0026), "peduncle_len": (0.040, 0.065),
        "peduncle_k": (3.0, 7.0), "stem_k": (25.0, 60.0), "string_k": (60.0, 180.0),
    },
}
FRUIT_DENSITY = 950.0
# A truss's own joints (peduncle to rachis, rachis segment to segment): bending stiffness (N m/rad) and damping
# (N m s/rad). Held level by its peduncle, a truss droops by about a third of a right angle.
TRUSS = {"rachis_k": 0.9, "rachis_d": 0.06}
LOOK = {"calyx": [0.10, 0.26, 0.05], "stem": [0.20, 0.36, 0.10], "peduncle": [0.22, 0.40, 0.11],
        "leaf": [0.07, 0.24, 0.05], "leaf_back": [0.12, 0.30, 0.09]}

ROW = {
    "plants": 4, "trusses": 3, "pitch": 0.50, "first_y": 0.30, "x": 1.20, "gutter_z": -0.48,
    "stem_segments": 5, "stem_len": 0.36, "stem_r": (0.0055, 0.0068),
    "ready": (4, 6),                    # ready trusses in a row section (the draw repeats until it is within)
    "slip": (0.02, 0.09), "slip_sigma": 0.004,
    "light_gain": (0.65, 1.35), "light_g": (0.35, 1.0), "light_b": (0.45, 1.0),
    "leaf_p": (0.55, 0.9),
}


# --------------------------------------------------------------------------------------------- draws
class Stream(object):
    """A reproducible stream of draws from a 64-bit seed: BLAKE2b in counter mode, keyed by the seed and a tag."""

    def __init__(self, seed, tag):
        self._key = hashlib.blake2b(b"greenhouse row draw 1|%s|%d" % (tag.encode("ascii"), int(seed)), digest_size=32).digest()
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


def _unit(v):
    v = np.asarray(v, dtype=float)
    return v / np.linalg.norm(v)


def _r(v, n=5):
    return [round(float(x), n) for x in v]


def chart_color(cultivar, s):
    """The skin's reflectance at the continuous ripeness s (1 to 6): the chart, linear between stages."""
    chart = CULTIVARS[cultivar]["chart"]
    s = min(6.0, max(1.0, float(s)))
    i = min(4, int(math.floor(s - 1.0)))
    f = s - 1.0 - i
    return [chart[i][k] * (1.0 - f) + chart[i + 1][k] * f for k in range(3)]


def is_ready(cultivar, stages):
    c = CULTIVARS[cultivar]
    return sum(1 for s in stages if s >= c["stage_min"]) >= int(math.ceil(c["fraction"] * len(stages) - 1e-9))


def _stem_point(points, z):
    """The point of a stem's polyline at height z."""
    for a, b in zip(points[:-1], points[1:]):
        if a[2] <= z <= b[2]:
            t = (z - a[2]) / (b[2] - a[2])
            return np.asarray(a) + t * (np.asarray(b) - np.asarray(a))
    raise ValueError("height %.3f is not on the stem" % z)


def _segment_of(points, z):
    for i, (a, b) in enumerate(zip(points[:-1], points[1:])):
        if a[2] <= z <= b[2]:
            return i, (z - a[2]) / (b[2] - a[2])
    raise ValueError(z)


def _ripeness(st, cultivar, n, ready, below=None):
    """The ripeness of a truss's n fruits, proximal first: a truss maturity m, each fruit 0.45 behind the one before
    it, +-0.3; every fruit within 0.35 of a whole stage (so a fruit's stage is not a matter of rounding). Drawn until
    the truss is ready or not, as asked: a ready truss from m in 5.0 to 6.9, the first truss above the ready ones from
    3.8 to 5.6 (often one fruit short), every truss above that 1.1 to 2.3 behind the one below."""
    for _ in range(400):
        if ready:
            m = st.uniform(5.0, 6.9)
        elif below is None:
            m = st.uniform(3.8, 5.6)
        else:
            m = max(0.8, below - st.uniform(1.1, 2.3))
        out = []
        for j in range(n):
            s = min(6.3, max(0.7, m - 0.45 * j + st.uniform(-0.3, 0.3)))
            stage = int(min(6, max(1, round(s))))
            out.append((stage, min(stage + 0.35, max(stage - 0.35, s))))
        if is_ready(cultivar, [o[0] for o in out]) == bool(ready):
            return m, out
    raise RuntimeError("no ripeness draw")


def _truss(st, cultivar, pid, k, points, stem_r, z_node, az):
    """One truss: the peduncle (the cut section, from the stem to the first fruit's node), three rachis segments
    hanging from its end, the fruits on short pedicels on alternating sides. `az` is the peduncle's azimuth
    (0: toward the aisle; positive: toward +y)."""
    c = CULTIVARS[cultivar]
    h = np.array([-math.cos(az), math.sin(az), 0.0])
    el = math.radians(st.uniform(30.0, 62.0))             # below the horizontal
    d0 = math.cos(el) * h - math.sin(el) * np.array([0.0, 0.0, 1.0])
    L0 = st.uniform(*c["peduncle_len"])
    rp = st.uniform(*c["peduncle_r"])
    node = _stem_point(points, z_node) + stem_r * h
    n = int(c["fruits"][0] + st.u64() % (c["fruits"][1] - c["fruits"][0] + 1))
    rf = st.uniform(c["fruit_r"][0], c["fruit_r"][1] - 0.0012 * (n - c["fruits"][0]))      # a fuller truss, smaller fruits
    gap = st.uniform(0.003, 0.008)
    first = rf + st.uniform(0.012, 0.020)                 # the first fruit's node along the rachis, from the peduncle's end
    step = rf + gap
    total = first + (n - 0.5) * step
    seg_len = total / 3.0
    tilt = [math.radians(a * st.uniform(0.8, 1.2)) for a in (14.0, 6.0, 2.0)]    # the rachis hangs: nearly plumb
    segs, a = [], node + L0 * d0
    for j in range(3):
        d = math.sin(tilt[j]) * h - math.cos(tilt[j]) * np.array([0.0, 0.0, 1.0])
        b = a + seg_len * d
        segs.append({"a": _r(a), "b": _r(b)})
        a = b
    side0 = st.uniform(0.0, 2 * math.pi)
    fruits = []
    for j in range(n):
        arc = first + j * step
        si = min(2, int(arc / seg_len))
        t = arc - si * seg_len
        A, B = np.array(segs[si]["a"]), np.array(segs[si]["b"])
        d = _unit(B - A)
        u = _unit(np.cross(d, [0.0, 0.0, 1.0]) if abs(d[2]) < 0.95 else np.cross(d, [1.0, 0.0, 0.0]))
        v = np.cross(d, u)
        ang = side0 + j * math.pi + st.uniform(-0.35, 0.35)
        ped = st.uniform(0.005, 0.010)
        r_j = rf * st.uniform(0.94, 1.0)
        center = A + t * d + (r_j + ped) * (math.cos(ang) * u + math.sin(ang) * v)
        fruits.append({"segment": si, "center": _r(center), "attach": _r(A + t * d), "r": round(r_j, 5),
                       "mass": round(4.0 / 3.0 * math.pi * r_j ** 3 * FRUIT_DENSITY, 5)})
    return {"id": "%s%d" % (pid, k + 1), "plant": pid, "node": _r(node), "azimuth_deg": round(math.degrees(az), 3),
            "elevation_deg": round(math.degrees(el), 3),
            "peduncle": {"a": _r(node), "b": _r(node + L0 * d0), "r": round(rp, 5), "len": round(L0, 5)},
            "rachis": segs, "rachis_r": round(rp * 0.9, 5), "fruits": fruits}


def _ripen(st, cultivar, truss, ready, below=None):
    m, out = _ripeness(st, cultivar, len(truss["fruits"]), ready, below)
    for f, (stage, s) in zip(truss["fruits"], out):
        w = st.uniform(0.95, 1.05)
        f.update(stage=stage, ripeness=round(s, 3), albedo=_r([min(1.0, v * w) for v in chart_color(cultivar, s)], 4))
    truss.update(stages=[o[0] for o in out], ready=bool(ready), maturity=round(m, 3))
    return m


def _leaf(st, pid, i, points, stem_r, z_node, az):
    """A leaf: a petiole rising from the stem, a blade drooping beyond it (one rigid plate on a compliant joint)."""
    h = np.array([-math.cos(az), math.sin(az), 0.0])
    rise = math.radians(st.uniform(8.0, 32.0))
    Lp = st.uniform(0.07, 0.11)
    rp = st.uniform(0.0022, 0.0032)
    node = _stem_point(points, z_node) + stem_r * h
    dp = math.cos(rise) * h + math.sin(rise) * np.array([0.0, 0.0, 1.0])
    end = node + Lp * dp
    droop = math.radians(st.uniform(20.0, 60.0))
    db = math.cos(droop) * h - math.sin(droop) * np.array([0.0, 0.0, 1.0])
    length, width = st.uniform(0.22, 0.30), st.uniform(0.13, 0.18)
    return {"id": "%sL%d" % (pid, i + 1), "plant": pid, "node": _r(node), "azimuth_deg": round(math.degrees(az) % 360.0, 3),
            "petiole": {"a": _r(node), "b": _r(end), "r": round(rp, 5)},
            "blade": {"a": _r(end), "dir": _r(db), "length": round(length, 4), "width": round(width, 4)},
            "k": round(st.uniform(0.35, 0.9), 4), "mass": round(0.012 + 0.03 * length * width / 0.04, 5)}


def approach_axes(truss, alpha_deg=0.0):
    """The tool frame for a cut of `truss` at rest (columns x, y, z in W): x along the peduncle toward the stem, z the
    approach direction, perpendicular to the peduncle; alpha 0 is the approach in the peduncle's vertical plane, from
    above and in front; +-90 the approaches along the row."""
    a, b = np.array(truss["peduncle"]["a"]), np.array(truss["peduncle"]["b"])
    x = _unit(a - b)
    m = _unit(np.cross(x, [0.0, 0.0, 1.0]))          # horizontal, perpendicular to the peduncle
    n1 = _unit(np.cross(m, x))                       # in the vertical plane
    if n1[2] > 0:
        n1 = -n1                                     # the one that points down
    al = math.radians(alpha_deg)
    z = math.cos(al) * n1 + math.sin(al) * np.cross(x, n1)
    return np.stack([x, np.cross(z, x), z], axis=1)


def _box(x, y, z, n=(3, 3, 5)):
    return [(a, b, c) for a in np.linspace(x[0], x[1], n[0]) for b in np.linspace(y[0], y[1], n[1]) for c in np.linspace(z[0], z[1], n[2])]


def tool_points():
    """Sample points of the tool, the wrist camera's housing and the robot's wrist behind it, in the tool frame
    (origin at the tool centre point): what a straight approach sweeps."""
    tz, j, nk, bd = TOOL["tcp_z"], TOOL["jaw"], TOOL["neck"], TOOL["body"]
    hw = TOOL["open_max"] / 2 + j["t"]
    pts = _box(j["x"], (-hw, hw), (j["z"][0] - tz, j["z"][1] - tz), (3, 3, 3))
    pts += _box((-nk["size"][0] / 2, nk["size"][0] / 2), (-nk["size"][1] / 2, nk["size"][1] / 2), (nk["z"][0] - tz, nk["z"][1] - tz), (3, 3, 5))
    pts += _box((-bd["size"][0] / 2, bd["size"][0] / 2), (-bd["size"][1] / 2, bd["size"][1] / 2), (bd["z"][0] - tz, bd["z"][1] - tz), (3, 3, 5))
    cx, _cy, cz = CAMERAS["wrist"]["pos"]
    pts += _box((cx - 0.020, cx + 0.020), (-0.022, 0.022), (cz - 0.025 - tz, cz + 0.025 - tz), (2, 2, 3))
    for zl in np.linspace(-tz - 0.17, -tz, 5):                # the robot's wrist: a cylinder about the tool axis
        pts += [(0.06 * math.cos(a), 0.06 * math.sin(a), zl) for a in np.linspace(0.0, 2 * math.pi, 8, endpoint=False)]
    return np.array(pts)


_TOOL_PTS = tool_points()
APPROACH_ALPHAS = (0.0, -55.0, 55.0)
APPROACH_BACK = 0.16


def cut_fraction(truss):
    """Where along the peduncle (from the stem, as a fraction of its length) the mouth's centre goes for a cut that
    leaves the jaws clear of the stem and puts blade and pad on the peduncle."""
    L = truss["peduncle"]["len"]
    lo, hi = 0.024, L - 0.010
    return float(min(hi, max(lo, 0.5 * (lo + hi))) / L)


def _swept(truss, alpha_deg):
    R = approach_axes(truss, alpha_deg)
    a, b = np.array(truss["peduncle"]["a"]), np.array(truss["peduncle"]["b"])
    p = a + cut_fraction(truss) * (b - a)
    return np.concatenate([(p - back * R[:, 2]) + _TOOL_PTS @ R.T for back in np.linspace(0.0, APPROACH_BACK, 5)])


def _dist_points_segment(pts, a, b):
    a, b = np.asarray(a, dtype=float), np.asarray(b, dtype=float)
    ab = b - a
    t = np.clip((pts - a) @ ab / float(ab @ ab), 0.0, 1.0)
    return np.linalg.norm(pts - (a + t[:, None] * ab), axis=1)


def _hits(pts, kind, item):
    """True when the swept points come too near a member at rest."""
    if kind == "leaf":
        if _dist_points_segment(pts, item["petiole"]["a"], item["petiole"]["b"]).min() < item["petiole"]["r"] + 0.012:
            return True
        ba = np.array(item["blade"]["a"])
        be = ba + item["blade"]["length"] * np.array(item["blade"]["dir"])
        return bool(_dist_points_segment(pts, ba, be).min() < item["blade"]["width"] / 2 + 0.012)
    if kind == "truss":
        cs = np.array([f["center"] for f in item["fruits"]])
        rs = np.array([f["r"] for f in item["fruits"]])
        d = np.linalg.norm(pts[:, None, :] - cs[None, :, :], axis=2) - rs[None, :]
        return bool(d.min() < 0.008)
    if kind == "stem":
        return any(_dist_points_segment(pts, s0, s1).min() < item["stem_r"] + 0.008
                   for s0, s1 in zip(item["points"][:-1], item["points"][1:]))
    if kind == "peduncle":
        return bool(_dist_points_segment(pts, item["peduncle"]["a"], item["peduncle"]["b"]).min() < item["peduncle"]["r"] + 0.008)
    raise ValueError(kind)


def clear_alphas(truss, plants, leaves=True):
    """The stated approach directions (APPROACH_ALPHAS) along which the tool reaches the truss's cut point in a
    straight line of APPROACH_BACK without touching a stem, a fruit, another truss's peduncle or (with `leaves`) a
    leaf, everything at rest."""
    out = []
    for al in APPROACH_ALPHAS:
        pts = _swept(truss, al)
        ok = True
        for q in plants:
            if abs(q["points"][0][1] - truss["node"][1]) > 0.9:
                continue
            if _hits(pts, "stem", q) or any(_hits(pts, "truss", t) for t in q["trusses"]) \
                    or any(t["id"] != truss["id"] and _hits(pts, "peduncle", t) for t in q["trusses"]) \
                    or (leaves and any(_hits(pts, "leaf", lf) for lf in q["leaves"])):
                ok = False
                break
        if ok:
            out.append(al)
    return out


def _plants(seed, cultivar):
    """The plants of a row section. By construction: 4 to 6 trusses are ready, on each plant the lowest ones; every
    ready truss can be cut by a straight approach from at least one of the three stated directions without pushing
    anything aside (a truss that cannot is drawn again, up to 40 times; a leaf that would close a ready truss's last
    approach is one the grower has taken off)."""
    for attempt in range(50):
        st = Stream(seed, "plants|%d" % attempt)
        leaf_p = st.uniform(*ROW["leaf_p"])
        n_ready = int(ROW["ready"][0] + st.u64() % (ROW["ready"][1] - ROW["ready"][0] + 1))
        quota = [0] * ROW["plants"]
        for _ in range(n_ready):
            free = [i for i in range(ROW["plants"]) if quota[i] < 2]
            quota[st.choice(free)] += 1
        plants, ok = [], True
        for i in range(ROW["plants"]):
            pid = "ABCDEFGH"[i]
            base = np.array([ROW["x"] + st.uniform(-0.03, 0.03), ROW["first_y"] + ROW["pitch"] * i + st.uniform(-0.05, 0.05), ROW["gutter_z"]])
            lean_y, lean_x = st.uniform(-0.10, 0.10), st.uniform(-0.03, 0.03)
            pts = [base]
            for j in range(ROW["stem_segments"]):
                d = _unit([math.sin(lean_x + st.uniform(-0.03, 0.03)), math.sin(lean_y + st.uniform(-0.04, 0.04)), 1.0])
                pts.append(pts[-1] + ROW["stem_len"] * d)
            plant = {"id": pid, "points": [_r(p) for p in pts], "stem_r": round(st.uniform(*ROW["stem_r"]), 5), "trusses": [], "leaves": []}
            plants.append(plant)
            zs = [st.uniform(0.0, 0.12)]
            for _ in range(ROW["trusses"] - 1):
                zs.append(zs[-1] + st.uniform(0.27, 0.33))
            plant["node_z"] = [round(z, 4) for z in zs]
            sign = st.choice([-1.0, 1.0])
            for k in range(ROW["trusses"]):
                for _try in range(40):
                    az = sign * math.radians(st.uniform(8.0, 50.0)) if _try < 30 else math.radians(st.uniform(-50.0, 50.0))
                    tr = _truss(st, cultivar, pid, k, pts, plant["stem_r"], zs[k], az)
                    plant["trusses"].append(tr)
                    # the new truss must leave an approach to every ready truss below it, and have one itself if ready
                    good = all(clear_alphas(t, plants, leaves=False) for t in plant["trusses"][:quota[i]] if t is not tr or k < quota[i])
                    if good:
                        break
                    plant["trusses"].pop()
                else:
                    ok = False
                    break
                sign = -sign                             # successive trusses hang on alternating sides
            if not ok:
                break
            m = None
            for k, tr in enumerate(plant["trusses"]):
                m = _ripen(st, cultivar, tr, k < quota[i], below=m if k > quota[i] else None)
        if not ok:
            continue
        # leaves: two nodes above every truss, on the plant's own phyllotaxis; the grower has taken most of the
        # leaves below the ripening trusses, and every leaf that would close a ready truss's last approach
        ready = [t for p in plants for t in p["trusses"] if t["ready"]]
        for i, plant in enumerate(plants):
            pts = [np.array(p) for p in plant["points"]]
            az, n = st.uniform(0.0, 2 * math.pi), 0
            for k in range(ROW["trusses"]):
                for dz in (st.uniform(0.085, 0.125), st.uniform(0.165, 0.215)):
                    az += math.radians(137.5 + st.uniform(-20.0, 20.0))
                    leaf = _leaf(st, plant["id"], n, pts, plant["stem_r"], plant["node_z"][k] + dz, az)
                    n += 1
                    if not st.chance(leaf_p * (0.45 if k == 0 else 1.0)):
                        continue
                    plant["leaves"].append(leaf)
                    near = [t for t in ready if abs(t["node"][1] - leaf["node"][1]) < 0.9]
                    if not all(clear_alphas(t, plants) for t in near):
                        plant["leaves"].pop()
        if all(clear_alphas(t, plants) for t in ready):
            return plants, attempt, round(leaf_p, 4)
    raise RuntimeError("no row section within the rule's bounds after 50 draws")


def draw_row(seed, cultivar="Rossa", cameras=None, light_seed=None, mech_seed=None):
    """One row section from its 64-bit seed. `cameras` are the robot's mounting offsets, `light_seed` and `mech_seed`
    the seeds of the row's light and of its mechanics (tests/review/select_set.py draws them from keys of their own);
    without them the cameras are nominal, the light is the nominal white and the mechanics are mid-range."""
    c = CULTIVARS[cultivar]
    plants, attempt, leaf_p = _plants(seed, cultivar)
    light = {"gain": 1.0, "rgb": [1.0, 1.0, 1.0], "lamps": [1.0, 1.0, 1.0]}
    if light_seed is not None:
        st = Stream(light_seed, "light")
        light = {"gain": round(st.uniform(*ROW["light_gain"]), 4),
                 "rgb": [1.0, round(st.uniform(*ROW["light_g"]), 4), round(st.uniform(*ROW["light_b"]), 4)],
                 "lamps": [round(st.uniform(0.75, 1.25), 4) for _ in range(3)]}
    mid = lambda r: 0.5 * (r[0] + r[1])                       # noqa: E731
    mech = {"toughness": mid(CUT["toughness"]), "stem_k": mid(c["stem_k"]), "string_k": mid(c["string_k"]),
            "damping": 0.25, "slip": mid(ROW["slip"])}
    if mech_seed is not None:
        st = Stream(mech_seed, "mechanics")
        mech = {"toughness": round(st.uniform(*CUT["toughness"]), 4), "stem_k": round(st.uniform(*c["stem_k"]), 3),
                "string_k": round(st.uniform(*c["string_k"]), 3), "damping": round(st.uniform(0.15, 0.35), 4),
                "slip": round(st.uniform(*ROW["slip"]), 5)}
    st = Stream(seed, "truss mechanics")
    for p in plants:
        for t in p["trusses"]:
            t["k"] = round(st.uniform(*c["peduncle_k"]), 4)
    return {"seed": int(seed), "version": VERSION, "cultivar": cultivar, "plants": plants, "attempt": attempt,
            "leaf_p": leaf_p, "light": light, "mechanics": mech,
            "cameras": {k: [float(x) for x in v] for k, v in (cameras or {}).items()},
            "ready": [t["id"] for p in plants for t in p["trusses"] if t["ready"]]}


ENTRY_KEYS = ("cultivar", "cameras", "light_seed", "mech_seed")


def draw_entry(entry):
    """The row of a set's entry (tests/hidden/rows.json, the commissioning service's dev_rows.json)."""
    return draw_row(entry["seed"], **{k: entry[k] for k in ENTRY_KEYS if k in entry})


def cut_force(row, r):
    """The closing force (N) that cuts a member of radius r (m) in this row."""
    return float(row["mechanics"]["toughness"] * math.pi * (r * 1000.0) ** 2)


def summary(row):
    ts = [t for p in row["plants"] for t in p["trusses"]]
    return {"cultivar": row["cultivar"], "trusses": len(ts), "ready": list(row["ready"]),
            "fruits": sum(len(t["fruits"]) for t in ts), "leaves": sum(len(p["leaves"]) for p in row["plants"]),
            "stages": {t["id"]: "".join(str(s) for s in t["stages"]) for t in ts},
            "peduncle_mm": {t["id"]: [round(t["peduncle"]["len"] * 1000, 1), round(2000 * t["peduncle"]["r"], 2)] for t in ts},
            "light": row["light"], "mechanics": row["mechanics"], "attempt": row["attempt"]}


def crate_slots():
    """Where the reference lays trusses: three lanes a crate, along the rail."""
    out = []
    for name, cr in CART["crates"].items():
        (cx, cy), (sx, sy) = cr["center"], cr["size"]
        for k in range(3):
            out.append({"crate": name, "x": round(cx - sx / 2 + sx * (k + 0.5) / 3.0, 4), "y0": round(cy - sy / 2, 4), "y1": round(cy + sy / 2, 4)})
    return out


def public_site():
    """What the documents publish about the cart, the tool, the cameras and the limits. Nothing of a row is in here."""
    from . import wire
    cams = {}
    for name, c in CAMERAS.items():
        cams[name] = {"parent": c["parent"], "pos": c["pos"], "R": [[round(float(v), 9) for v in row] for row in camera_R(name)],
                      "pose_is": "nominal", "mount_tolerance": {"mm": MOUNT_TOL[name]["mm"], "deg": MOUNT_TOL[name]["deg"]},
                      "K": camera_K(name), "width": c["width"], "height": c["height"]}
        if c["depth"]:
            sd = SENSORS[name]
            cams[name]["depth"] = {"range_m": sd["range_m"], "noise_sigma_m_per_m2": sd["sigma_k"], "baseline_m": sd["baseline_m"],
                                   "disparity_step_px": sd["subpixel"], "correlation_px": sd["corr_px"], "lateral_blur_px": sd["blur_px"],
                                   "edge_px": sd["edge_px"], "window_px": sd["window_px"], "dark_luma": sd["dark"], "dim_luma": sd["dim"],
                                   "bright_rgb": sd["bright"]}
    d = wire.DEFAULTS
    return {
        "version": VERSION, "tick_s": TICK, "max_ticks_per_action": d["max_ticks"],
        "floor_z": FLOOR_Z, "deck_z": DECK_Z,
        "cart": {"deck": CART["deck"], "crates": CART["crates"], "crate_floor_z": CART["crate_floor_z"],
                 "crate_wall_h": CART["crate_wall_h"], "travel": CART["travel"], "speed_max": CART["speed_max"],
                 "home_q": CART["home_q"]},
        "tool": {k: TOOL[k] for k in ("mass", "tcp_z", "body", "neck", "jaw", "open_max", "mouth_z", "blade_x", "clamp_x",
                                      "jaw_speed", "force_max")},
        "cameras": cams,
        "calibration_plate": dict(CAL_PLATE),
        "color_card": dict(COLOR_CARD),
        "row_markers": dict(MARKERS, bits=[list(b) for b in MARKER_BITS[:MARKERS["count"]]],
                            poses=[[[round(float(v), 6) for v in r] for r in marker_T(k)] for k in range(MARKERS["count"])]),
        "row": {"plants": ROW["plants"], "trusses_per_plant": ROW["trusses"], "pitch": ROW["pitch"], "first_y": ROW["first_y"],
                "x": ROW["x"], "gutter_z": ROW["gutter_z"], "ready": list(ROW["ready"]), "slip": list(ROW["slip"]),
                "light_gain": list(ROW["light_gain"]), "light_g": list(ROW["light_g"]), "light_b": list(ROW["light_b"])},
        "cut": {"toughness": list(CUT["toughness"]), "crush": CUT["crush"]},
        "damage": {"impact_dv": DAMAGE["impact_dv"], "squeeze_force": DAMAGE["squeeze_force"]},
        "cultivars": {k: {"chart": v["chart"], "stage_min": v["stage_min"], "fraction": v["fraction"], "fruits": list(v["fruits"]),
                          "fruit_r": list(v["fruit_r"]), "peduncle_r": list(v["peduncle_r"]), "peduncle_len": list(v["peduncle_len"]),
                          "peduncle_k": list(v["peduncle_k"]), "stem_k": list(v["stem_k"]), "string_k": list(v["string_k"])}
                      for k, v in CULTIVARS.items()},
        "limits": {k: d[k] for k in ("tcp_speed_max", "tcp_rot_speed_max", "joint_speed_max", "overload_force", "overload_torque",
                                     "overload_joint_error", "stop_lock_s", "sim_cap_s", "frame_budget")},
    }

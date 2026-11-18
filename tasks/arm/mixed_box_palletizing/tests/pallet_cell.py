"""pallet_cell.py: the mixed-box palletizing cell.

One file holds the whole cell: the 4-axis palletizing arm with its vacuum gripper, the infeed
conveyor, the buffer table, the pallet, the cartons with their printed handling marks, the three
RGB-D cameras, the sensors, the handling rules (seal, shock, stacking load) and the end-of-order
tilt test with the shipped / not-shipped decision for every carton.

The lab that serves the agent's practice orders and the grader run this exact file. What differs
between them is only the catalogue of cartons, the orders and the seeds. The agent never reads
this file: it has `task_card.md`, which describes the same numbers in prose; where the two
disagree, this file is what runs and the card is wrong.

Frames and units: SI units, radians, world frame with z up and the floor at z = 0. The arm's base
axis is the world z axis. Camera poses are OpenCV style (x right, y down, z forward).

Typical use:

    cell = PalletCell(catalog)                 # catalog: dict sku -> spec (see CATALOG FORMAT)
    obs = cell.reset(order, seed)              # order: list of sku ids in arrival order
    while not cell.done:
        obs = cell.step(action)                # action: 7 numbers, see ACTION
    result = cell.finish()                     # tilt test + per-carton outcome

CATALOG FORMAT (one entry per stock-keeping unit):

    {"size": [L, W, H],            carton dimensions along its own x, y, z (m)
     "mass": kg,
     "com": [cx, cy],              centre of gravity offset from the centre of the top face (m)
     "stack_limit": kg or null,    printed stacking limit: 0 = "do not stack", null = no mark
     "fragile": bool,
     "tone": 0..3,                 cardboard tone
     "brand": "TEXT"}              brand text printed on the carton (a distractor)
"""

import base64
import io
import math

import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

# --------------------------------------------------------------------------------------------
# Numbers of the cell
# --------------------------------------------------------------------------------------------

G = 9.81
TIMESTEP = 0.002                # s, physics step
SUBSTEPS = 50                   # physics steps per control step
CONTROL_DT = TIMESTEP * SUBSTEPS  # 0.1 s: the controller runs at 10 Hz

# Arm geometry, in the vertical plane through the base axis (r = horizontal distance from the
# base axis, z = height). See fk() / ik().
A0 = 0.25                       # base axis -> shoulder axis (m)
H0 = 0.80                       # shoulder axis height (m)
L1 = 1.20                       # upper arm (m)
L2 = 1.30                       # forearm (m)
A3 = 0.20                       # wrist pivot -> tool axis (m)
D_TOOL = 0.55                   # wrist pivot -> pad face, cup spring extended (m)
Q_MIN = np.array([-0.80, -0.90, -1.30, -4.20])
Q_MAX = np.array([3.95, 1.45, 2.30, 4.20])
Q_HOME = np.array([0.785, 0.0, 0.35, -0.785])

# Command limits (the action is a velocity of the pad-face centre, see ACTION).
V_MAX = 1.2                     # m/s per axis
W_MAX = 2.5                     # rad/s, tool yaw rate
TARGET_LEAD_MAX = 0.12          # m: the internal target never runs further ahead of the arm
R_MIN, R_MAX = 0.80, 2.15       # m: allowed horizontal distance of the pad centre from the base axis
Z_MIN, Z_MAX = 0.10, 2.10       # m: allowed height of the pad face

# Vacuum gripper: a foam pad, PAD_L x PAD_W, on a spring-loaded cup (30 mm stroke).
PAD_L, PAD_W = 0.20, 0.14       # m
PAD_AREA = PAD_L * PAD_W
CUP_STROKE = 0.03               # m
CUP_STIFFNESS = 1200.0          # N/m
CUP_MASS = 1.0                  # kg (hangs below the force/torque sensor)
SEAL_GAP_LO, SEAL_GAP_HI = -0.004, 0.002   # m: pad face minus carton top, to seal
SEAL_MIN_AREA = 0.35 * PAD_AREA  # m^2 of pad on the carton's top face, to seal
SEAL_PRESSURE = 30000.0         # Pa: holding force = SEAL_PRESSURE * overlap area
SEAL_MOMENT_ARM = 0.15          # holding moment = SEAL_MOMENT_ARM * holding force * sqrt(overlap area)
SEAL_FILTER_TAU = 0.010         # s: the load on the seal is low-pass filtered with this time constant

# Layout (world frame).
STATION = np.array([1.45, 0.0])     # centre of the pick station at the end of the infeed conveyor
BELT_Z = 0.60                       # conveyor surface
STATION_HALF = np.array([0.45, 0.40])
TABLE = np.array([-1.45, 0.0])      # buffer table centre
TABLE_Z = 0.60
TABLE_HALF = np.array([0.55, 0.70])
PALLET = np.array([0.0, 1.50])      # pallet centre; 1.2 m along world x, 0.8 m along world y
PALLET_HALF = np.array([0.60, 0.40])
DECK_Z = 0.144                      # pallet deck surface
STACK_HEIGHT_MAX = 1.00             # m above the deck
INFEED_DELAY = 2.0                  # s the station must be clear before the next carton arrives

# Handling rules.
SHOCK_WINDOW = 0.020            # s
SHOCK_LIMIT = 60.0              # m/s^2, mean acceleration over SHOCK_WINDOW, any carton
SHOCK_LIMIT_FRAGILE = 20.0      # m/s^2, cartons marked fragile
LOAD_FILTER_TAU = 0.5           # s
LOAD_NO_STACK_KG = 0.5          # a "do not stack" carton tolerates this much
LOAD_UNMARKED_KG = 80.0         # a carton without a stacking mark tolerates this much
LOAD_PERIOD = 5                 # physics steps between load evaluations

# The tilt test at the end of an order, and the shipped decision.
SETTLE_TIME = 2.0               # s before and after the tilt
TILT_ANGLE = math.radians(10.0)
TILT_RAMP, TILT_HOLD = 1.0, 2.0  # s
OVERHANG_TOL = 0.005            # m a carton may reach beyond the pallet's edge
LEVEL_TOL = math.radians(3.0)   # a shipped carton's top face is level within this
MOVE_TOL = 0.030                # m a carton's centre may move during the tilt test
TURN_TOL = math.radians(5.0)    # rad a carton may turn during the tilt test

# Episode length: steps allowed per carton of the order, plus a fixed allowance.
STEPS_PER_CARTON = 400
STEPS_FIXED = 400

# Cameras: name -> (width, height, vertical field of view in degrees, position, look-at point, up).
CAMERAS = {
    "infeed": (800, 600, 27.0, (1.45, 0.0, 2.60), (1.45, 0.0, 0.60), (0.0, 1.0, 0.0)),
    "pallet": (640, 480, 19.0, (0.0, 1.50, 4.00), (0.0, 1.50, 0.144), (0.0, 1.0, 0.0)),
    "buffer": (640, 480, 28.5, (-1.45, 0.0, 3.00), (-1.45, 0.0, 0.60), (1.0, 0.0, 0.0)),
}
CAMERA_BITS = {"infeed": 1, "pallet": 2, "buffer": 4}
CAM_JITTER_POS = 0.03           # m, per-order camera position error against the drawing (not reported)
CAM_JITTER_ROT = math.radians(1.5)
# What differs from the drawing in every order, and is told to nobody: the zero of each joint's
# encoder, where the cameras really hang, and where the forklift put the pallet down.
Q_OFFSET_MAX = np.radians([0.5, 0.3, 0.3, 1.0])   # rad, reported joint angle minus true joint angle
PALLET_SHIFT = 0.025            # m, pallet centre against PALLET, per axis
PALLET_TURN = math.radians(2.0)  # rad, pallet yaw against the world axes
# Survey marks: white discs with a black centre, painted where the drawing says (exact).
MARK_R, MARK_DOT_R = 0.025, 0.008
SURVEY_MARKS = {
    "infeed": [(1.05, -0.33, 0.60), (1.05, 0.33, 0.60), (1.85, -0.33, 0.60), (1.85, 0.33, 0.60)],
    "pallet": [(-0.75, 0.97, 0.0), (0.75, 0.97, 0.0), (-0.75, 2.03, 0.0), (0.75, 2.03, 0.0)],
    "buffer": [(-1.93, -0.63, 0.60), (-1.93, 0.63, 0.60), (-0.97, -0.63, 0.60), (-0.97, 0.63, 0.60)],
}
TOOL_MARK_Z = 0.06              # m above the tool pivot: a magenta disc on the tool axis, with a
TOOL_MARK_R = 0.04              # yellow dot TOOL_DOT_X along the tool's x axis
TOOL_DOT_X, TOOL_DOT_R = 0.07, 0.015
DEPTH_SIGMA = 0.0015            # m, depth noise at 1 m; grows linearly with distance
DEPTH_DROPOUT = 0.005           # fraction of depth pixels that read 0
WRENCH_SIGMA_F = 0.4            # N
WRENCH_SIGMA_T = 0.04           # N m

# Printing.
FACE_PX = 320                   # texture resolution of every carton face
DRAW_PPM = 1000.0               # px per metre while drawing a face, before it is resized
TAPE_W = 0.048                  # m
STACK_LIMITS = (0, 5, 10, 20, 40)   # kg, the stacking marks that exist (0 = do not stack)
TONES = ((196, 158, 112), (181, 140, 96), (205, 172, 130), (170, 134, 98))


# --------------------------------------------------------------------------------------------
# Kinematics
# --------------------------------------------------------------------------------------------

def fk(q, cup=0.0):
    """Pad-face centre [x, y, z] and tool yaw for joint angles q = [q1, q2, q3, q4].

    q1 turns the arm about the world z axis. q2 is the upper arm's angle from the vertical
    (positive leans outward) and q2 + q3 the forearm's angle below the horizontal. A parallelogram
    keeps the wrist level, so the tool axis is always vertical. q4 turns the tool about it; the
    tool yaw in the world frame is q1 + q4. `cup` is the cup's spring compression (m).
    """
    q1, q2, q3, q4 = (float(v) for v in q[:4])
    phi = q2 + q3
    r = A0 + L1 * math.sin(q2) + L2 * math.cos(phi) + A3
    z = H0 + L1 * math.cos(q2) - L2 * math.sin(phi) - D_TOOL + cup
    return np.array([r * math.cos(q1), r * math.sin(q1), z]), q1 + q4


def ik(xyz, yaw):
    """Joint angles for a pad-face centre and a tool yaw (elbow up), or None when out of reach.

    Joint limits are not checked here; see Q_MIN / Q_MAX.
    """
    x, y, z = (float(v) for v in xyz)
    q1 = math.atan2(y, x)
    # The arm's q1 range is [-0.80, 3.95]: map the angle into it.
    if q1 < Q_MIN[0]:
        q1 += 2.0 * math.pi
    u = math.hypot(x, y) - A3 - A0
    v = z + D_TOOL - H0
    d2 = u * u + v * v
    d = math.sqrt(d2)
    if d > L1 + L2 - 1e-6 or d < abs(L1 - L2) + 1e-6:
        return None
    beta = math.acos(max(-1.0, min(1.0, (L1 * L1 + d2 - L2 * L2) / (2.0 * L1 * d))))
    alpha = math.atan2(v, u) + beta
    fu, fv = u - L1 * math.cos(alpha), v - L1 * math.sin(alpha)
    phi = -math.atan2(fv, fu)
    q2 = math.pi / 2.0 - alpha
    return np.array([q1, q2, phi - q2, yaw - q1])


# --------------------------------------------------------------------------------------------
# Printing: the faces of a carton
# --------------------------------------------------------------------------------------------

_FONT_CACHE = {}


def _font(px):
    px = max(6, int(round(px)))
    if px not in _FONT_CACHE:
        _FONT_CACHE[px] = ImageFont.load_default(size=px)
    return _FONT_CACHE[px]


def mark_image(kind, size_px):
    """One handling mark as a square greyscale image (0 = ink, 255 = blank), size_px on a side.

    kind: "fragile", "nostack" or "max5" / "max10" / "max20" / "max40".
    """
    s = int(size_px)
    big = 4 * s
    img = Image.new("L", (big, big), 255)
    d = ImageDraw.Draw(img)
    w = max(2, big // 22)
    if kind == "fragile":
        # A stemmed glass with a crack.
        d.polygon([(0.20 * big, 0.08 * big), (0.80 * big, 0.08 * big), (0.56 * big, 0.52 * big),
                   (0.44 * big, 0.52 * big)], fill=0)
        d.rectangle([0.46 * big, 0.50 * big, 0.54 * big, 0.84 * big], fill=0)
        d.rectangle([0.26 * big, 0.84 * big, 0.74 * big, 0.92 * big], fill=0)
        d.line([(0.58 * big, 0.08 * big), (0.46 * big, 0.22 * big), (0.56 * big, 0.30 * big),
                (0.48 * big, 0.44 * big)], fill=255, width=w)
    elif kind == "nostack":
        # Two stacked cartons, crossed out.
        d.rectangle([0.22 * big, 0.10 * big, 0.78 * big, 0.46 * big], outline=0, width=w)
        d.rectangle([0.22 * big, 0.54 * big, 0.78 * big, 0.90 * big], outline=0, width=w)
        d.line([(0.06 * big, 0.06 * big), (0.94 * big, 0.94 * big)], fill=0, width=int(1.6 * w))
        d.line([(0.94 * big, 0.06 * big), (0.06 * big, 0.94 * big)], fill=0, width=int(1.6 * w))
    elif kind.startswith("max"):
        # Stacking limit by mass: the number of kilograms, a load bar with an arrow, a carton.
        text = kind[3:]
        font = _font(0.50 * big)
        box = d.textbbox((0, 0), text, font=font)
        tw, th = box[2] - box[0], box[3] - box[1]
        d.text(((big - tw) / 2.0 - box[0], 0.03 * big - box[1]), text, fill=0, font=font)
        small = _font(0.15 * big)
        d.text((0.70 * big, 0.50 * big), "kg", fill=0, font=small)
        d.rectangle([0.10 * big, 0.56 * big, 0.62 * big, 0.61 * big], fill=0)
        d.polygon([(0.26 * big, 0.61 * big), (0.46 * big, 0.61 * big), (0.36 * big, 0.71 * big)], fill=0)
        d.rectangle([0.16 * big, 0.74 * big, 0.84 * big, 0.95 * big], outline=0, width=w)
    else:
        raise ValueError("unknown mark {!r}".format(kind))
    return img.resize((s, s), Image.LANCZOS)


def stack_mark_kind(stack_limit):
    """The name of the stacking mark for a catalogue stack_limit, or None when there is no mark."""
    if stack_limit is None:
        return None
    return "nostack" if float(stack_limit) <= 0 else "max{}".format(int(stack_limit))


def mark_size(width):
    """Side of the square handling marks (m) on a carton whose top face is `width` wide."""
    band = (width - TAPE_W) / 2.0 - 0.012
    return float(min(0.085, max(0.050, band)))


def _cardboard(w, h, tone, rng):
    base = np.array(TONES[tone % len(TONES)], dtype=np.float32)
    img = np.empty((h, w, 3), dtype=np.float32)
    img[:] = base
    img += rng.normal(0.0, 3.0, size=(h, w, 1))                     # speckle
    img += (rng.normal(0.0, 2.0, size=(1, w, 1)))                   # faint flutes
    return np.clip(img, 0, 255).astype(np.uint8)


def _paste_ink(canvas, ink, x0, y0, colour=(28, 26, 24), strength=0.92, wear=None):
    """Print a greyscale ink image (0 = ink) onto an RGB canvas at (x0, y0). `wear` is an
    optional array like `ink`, 0..1, by which the ink is thinned."""
    h, w = ink.shape
    region = canvas[y0:y0 + h, x0:x0 + w].astype(np.float32)
    a = (255.0 - ink.astype(np.float32))[:, :, None] / 255.0 * strength
    if wear is not None:
        a = a * wear[:, :, None]
    region = region * (1.0 - a) + np.array(colour, dtype=np.float32) * a
    canvas[y0:y0 + h, x0:x0 + w] = np.clip(region, 0, 255).astype(np.uint8)


def draw_top_face(spec, rng):
    """The printed top face of a carton as an RGB array (rows = carton y, columns = carton x).

    Returned at DRAW_PPM pixels per metre: shape (round(W * DRAW_PPM), round(L * DRAW_PPM), 3),
    as seen from above: row 0 at carton y = +W/2 and column 0 at carton x = -L/2. Also returns the layout: a list of
    (kind, centre x, centre y, quarter turns, side) in carton coordinates, for the marks it printed.

    Print varies from carton to carton as it does between suppliers and print runs: the marks'
    size (0.75 to 1.0 of mark_size(width)), the ink (black to dark grey, brown or blue), how
    strongly it took, and worn patches. Nothing printed says where the centre of gravity is.
    """
    L, W = float(spec["size"][0]), float(spec["size"][1])
    w, h = int(round(L * DRAW_PPM)), int(round(W * DRAW_PPM))
    canvas = _cardboard(w, h, int(spec.get("tone", 0)), rng)
    # Sealing tape along the carton's x axis.
    t0, t1 = int((W / 2 - TAPE_W / 2) * DRAW_PPM), int((W / 2 + TAPE_W / 2) * DRAW_PPM)
    tape = canvas[t0:t1].astype(np.float32) * 0.55 + np.array([214, 196, 160], dtype=np.float32) * 0.45
    canvas[t0:t1] = np.clip(tape + rng.normal(0, 2.0, size=(t1 - t0, 1, 1)), 0, 255).astype(np.uint8)

    s = mark_size(W)
    taken, layout = [], []
    inks = ((28, 26, 24), (52, 50, 48), (64, 44, 30), (30, 38, 78), (22, 22, 22))
    ink_colour = inks[int(rng.integers(0, len(inks)))]
    ink_strength = float(rng.uniform(0.70, 0.95))
    s_px = int(round(s * float(rng.uniform(0.75, 1.0)) * DRAW_PPM))

    def place(pw, ph):
        """A free spot for a pw x ph rectangle in one of the two bands beside the tape."""
        margin = int(0.006 * DRAW_PPM)
        for _ in range(400):
            upper = rng.random() < 0.5
            y_lo, y_hi = (margin, t0 - margin - ph) if upper else (t1 + margin, h - margin - ph)
            if y_hi < y_lo or w - 2 * margin - pw < 0:
                continue
            x = int(rng.integers(margin, w - margin - pw + 1))
            y = int(rng.integers(y_lo, y_hi + 1))
            rect = (x - 4, y - 4, x + pw + 4, y + ph + 4)
            if all(rect[2] <= o[0] or rect[0] >= o[2] or rect[3] <= o[1] or rect[1] >= o[3] for o in taken):
                taken.append(rect)
                return x, y
        return None

    kinds = []
    stack = stack_mark_kind(spec.get("stack_limit"))
    if stack:
        kinds.append(stack)
    if spec.get("fragile"):
        kinds.append("fragile")
    for kind in kinds:
        spot = place(s_px, s_px)
        if spot is None:
            raise ValueError("no room for mark {} on a {} x {} face".format(kind, L, W))
        turns = int(rng.integers(0, 4))
        ink = np.rot90(np.asarray(mark_image(kind, s_px)), turns)
        # Wear: smooth patches where the ink is thinner, never gone.
        coarse = rng.uniform(0.55, 1.0, size=(4, 4)).astype(np.float32)
        wear = np.asarray(Image.fromarray(coarse).resize((s_px, s_px), Image.BILINEAR))
        _paste_ink(canvas, ink, spot[0], spot[1], ink_colour, ink_strength, wear)
        layout.append((kind, (spot[0] + s_px / 2) / DRAW_PPM - L / 2, W / 2 - (spot[1] + s_px / 2) / DRAW_PPM, turns,
                       s_px / DRAW_PPM))

    # Distractors: a shipping label and the brand text.
    lw, lh = int(0.080 * DRAW_PPM), int(min(0.050, s) * DRAW_PPM)
    spot = place(lw, lh)
    if spot is not None:
        label = Image.new("RGB", (lw, lh), (246, 246, 242))
        d = ImageDraw.Draw(label)
        x = 5
        while x < lw - 8:
            bar = int(rng.integers(1, 4))
            d.rectangle([x, 4, x + bar, int(lh * 0.55)], fill=(20, 20, 20))
            x += bar + int(rng.integers(1, 4))
        for row in range(2):
            y = int(lh * (0.66 + 0.15 * row))
            d.rectangle([5, y, int(lw * (0.55 + 0.3 * rng.random())), y + 2], fill=(60, 60, 60))
        canvas[spot[1]:spot[1] + lh, spot[0]:spot[0] + lw] = np.asarray(label)
    brand = str(spec.get("brand", ""))
    if brand:
        font = _font(0.030 * DRAW_PPM)
        box = font.getbbox(brand)
        bw, bh = box[2] - box[0] + 6, box[3] - box[1] + 6
        spot = place(bw, bh)
        if spot is not None:
            patch = Image.fromarray(canvas[spot[1]:spot[1] + bh, spot[0]:spot[0] + bw])
            colour = [(24, 60, 120), (20, 96, 60), (120, 30, 30)][int(rng.integers(0, 3))]
            ImageDraw.Draw(patch).text((3 - box[0], 3 - box[1]), brand, fill=colour, font=font)
            canvas[spot[1]:spot[1] + bh, spot[0]:spot[0] + bw] = np.asarray(patch)

    return canvas, layout


def _side_face(w_m, h_m, spec, rng):
    w, h = max(8, int(w_m * 400)), max(8, int(h_m * 400))
    canvas = _cardboard(w, h, int(spec.get("tone", 0)), rng)
    band = canvas[int(h * 0.42):int(h * 0.58)].astype(np.float32) * 0.8
    canvas[int(h * 0.42):int(h * 0.58)] = band.astype(np.uint8)
    return canvas


def _png(array):
    buf = io.BytesIO()
    Image.fromarray(array).resize((FACE_PX, FACE_PX), Image.LANCZOS).save(buf, format="PNG", compress_level=1)
    return buf.getvalue()


def carton_textures(spec, rng):
    """The six face images of a carton as PNG bytes, keyed by MuJoCo cube-texture face, and the
    top face's mark layout. The top face (+z of the carton) is printed; see draw_top_face."""
    L, W, H = (float(v) for v in spec["size"])
    top, layout = draw_top_face(spec, rng)
    # MuJoCo samples the +z cube face with the image's rows running along -y and its columns
    # along +x, which is the drawing as seen from above.
    faces = {
        "front": _png(top),
        "back": _png(_cardboard(64, 64, int(spec.get("tone", 0)), rng)),
        "right": _png(_side_face(W, H, spec, rng)),
        "left": _png(_side_face(W, H, spec, rng)),
        "up": _png(_side_face(L, H, spec, rng)),
        "down": _png(_side_face(L, H, spec, rng)),
    }
    return faces, layout


# --------------------------------------------------------------------------------------------
# Geometry helpers
# --------------------------------------------------------------------------------------------

def _rect_corners(centre, ax, ay, hx, hy):
    c = np.asarray(centre, dtype=float)
    return [c + sx * hx * ax + sy * hy * ay for sx, sy in ((1, 1), (-1, 1), (-1, -1), (1, -1))]


def _clip_polygon(subject, clip):
    """Sutherland-Hodgman: the part of convex polygon `subject` inside convex polygon `clip`
    (both lists of 2-vectors, counter-clockwise)."""
    out = list(subject)
    for i in range(len(clip)):
        a, b = clip[i], clip[(i + 1) % len(clip)]
        edge = b - a
        inp, out = out, []
        if not inp:
            break
        for j in range(len(inp)):
            p, q = inp[j], inp[(j + 1) % len(inp)]
            sp = edge[0] * (p[1] - a[1]) - edge[1] * (p[0] - a[0])
            sq = edge[0] * (q[1] - a[1]) - edge[1] * (q[0] - a[0])
            if sp >= 0:
                out.append(p)
            if (sp >= 0) != (sq >= 0):
                t = sp / (sp - sq)
                out.append(p + t * (q - p))
    return out


def _polygon_area(poly):
    if len(poly) < 3:
        return 0.0
    x = np.array([p[0] for p in poly])
    y = np.array([p[1] for p in poly])
    return 0.5 * abs(float(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1))))


def overlap_area(c1, yaw1, h1, c2, ax2, ay2, h2):
    """Area shared by two rectangles in the horizontal plane."""
    a1 = np.array([math.cos(yaw1), math.sin(yaw1)])
    b1 = np.array([-a1[1], a1[0]])
    r1 = _rect_corners(c1, a1, b1, h1[0], h1[1])
    if ax2[0] * ay2[1] - ax2[1] * ay2[0] < 0:
        ay2 = -ay2
    r2 = _rect_corners(c2, ax2, ay2, h2[0], h2[1])
    return _polygon_area(_clip_polygon(r1, r2))


def _encode(array):
    array = np.ascontiguousarray(array)
    return {"dtype": str(array.dtype), "shape": list(array.shape),
            "b64": base64.b64encode(array.tobytes()).decode("ascii")}


def decode(image):
    """An observation image ({"dtype", "shape", "b64"}) as a numpy array."""
    return np.frombuffer(base64.b64decode(image["b64"]), dtype=image["dtype"]).reshape(image["shape"])


def decode_depth(image):
    """An observation depth image (uint8, H x W x 2: the low and the high byte of the depth in
    millimetres) as a uint16 H x W array."""
    raw = decode(image)
    return raw[..., 0].astype(np.uint16) | (raw[..., 1].astype(np.uint16) << 8)


# --------------------------------------------------------------------------------------------
# The cell
# --------------------------------------------------------------------------------------------

_XML_HEAD = """
<mujoco model="pallet_cell">
  <compiler angle="radian" autolimits="true"/>
  <option timestep="{dt}" integrator="implicitfast" cone="elliptic" impratio="4" gravity="0 0 -{g}"/>
  <size memory="400M"/>
  <visual>
    <global offwidth="800" offheight="600"/>
    <quality shadowsize="2048"/>
    <headlight ambient="0.32 0.32 0.32" diffuse="0.25 0.25 0.25" specular="0 0 0"/>
    <map znear="0.02" zfar="30"/>
  </visual>
  <default>
    <geom contype="1" conaffinity="3" friction="0.5 0.005 0.0001" solref="0.005 1"/>
    <default class="arm">
      <geom contype="2" conaffinity="0" rgba="0.92 0.52 0.10 1"/>
    </default>
    <default class="look">
      <geom contype="0" conaffinity="0"/>
    </default>
  </default>
  <asset>
    <texture name="floor" type="2d" builtin="flat" rgb1="0.46 0.46 0.45" width="64" height="64"/>
    <material name="floor" texture="floor" reflectance="0"/>
    <material name="steel" rgba="0.55 0.57 0.60 1" specular="0.3"/>
    <material name="belt" rgba="0.16 0.17 0.18 1"/>
    <material name="wood" rgba="0.76 0.62 0.42 1"/>
    <material name="table" rgba="0.80 0.81 0.83 1"/>
{assets}
  </asset>
  <worldbody>
    <light name="key" pos="0.5 0.8 5.0" dir="0 0 -1" directional="true" diffuse="0.55 0.55 0.55" specular="0.1 0.1 0.1" castshadow="true"/>
    <light name="fill" pos="-2 -2 4" dir="0.4 0.4 -1" directional="true" diffuse="0.2 0.2 0.2" specular="0 0 0" castshadow="false"/>
    <geom name="floor" type="plane" size="8 8 0.1" material="floor"/>
    <!-- infeed conveyor: runs along world x, ends at the stop near the arm -->
    <geom name="belt" type="box" pos="2.50 0 {belt_c}" size="1.50 0.40 {belt_h}" material="belt" friction="0.45 0.005 0.0001"/>
    <geom name="stop" type="box" pos="0.985 0 {stop_c}" size="0.015 0.40 0.03" material="steel"/>
    <geom name="guide_l" type="box" pos="2.50 0.415 {stop_c}" size="1.50 0.015 0.03" material="steel"/>
    <geom name="guide_r" type="box" pos="2.50 -0.415 {stop_c}" size="1.50 0.015 0.03" material="steel"/>
    <!-- buffer table -->
    <geom name="table" type="box" pos="{tx} {ty} {table_c}" size="{thx} {thy} {table_h}" material="table"/>
    <!-- pallet -->
    <body name="pallet" pos="{px} {py} 0" euler="0 0 {pyaw}">
      <geom name="deck" type="box" pos="0 0 {deck_c}" size="{phx} {phy} {deck_h}" material="wood"/>
      <geom class="look" type="box" pos="0 0 {deck_top}" size="{phx} 0.004 0.0006" rgba="0.5 0.4 0.26 1"/>
      <geom class="look" type="box" pos="0 -0.2 {deck_top}" size="{phx} 0.004 0.0006" rgba="0.5 0.4 0.26 1"/>
      <geom class="look" type="box" pos="0 0.2 {deck_top}" size="{phx} 0.004 0.0006" rgba="0.5 0.4 0.26 1"/>
    </body>
{marks}
    <!-- arm -->
    <geom class="look" type="cylinder" pos="0 0 0.30" size="0.24 0.30" rgba="0.25 0.26 0.28 1"/>
    <body name="turret" pos="0 0 0.60" gravcomp="1">
      <joint name="j1" type="hinge" axis="0 0 1" range="{q1lo} {q1hi}" armature="30" damping="60"/>
      <geom class="look" type="cylinder" pos="0 0 0.08" size="0.22 0.08" rgba="0.92 0.52 0.10 1" mass="60"/>
      <geom class="look" type="box" pos="{a0h} 0 0.16" size="{a0h} 0.10 0.06" rgba="0.92 0.52 0.10 1" mass="10"/>
      <body name="upper" pos="{a0} 0 {h0r}" gravcomp="1">
        <joint name="j2" type="hinge" axis="0 1 0" range="{q2lo} {q2hi}" armature="30" damping="60"/>
        <geom class="arm" type="capsule" fromto="0 0 0 0 0 {l1}" size="0.075" mass="40"/>
        <body name="fore" pos="0 0 {l1}" gravcomp="1">
          <joint name="j3" type="hinge" axis="0 1 0" range="{q3lo} {q3hi}" armature="20" damping="40"/>
          <geom class="arm" type="capsule" fromto="0 0 0 {l2} 0 0" size="0.06" mass="30"/>
          <body name="wrist" pos="{l2} 0 0" gravcomp="1">
            <joint name="jw" type="hinge" axis="0 1 0" armature="2" damping="5"/>
            <geom class="arm" type="box" pos="{a3h} 0 0" size="{a3h} 0.05 0.04" mass="5"/>
            <body name="tool" pos="{a3} 0 0" gravcomp="1">
              <joint name="j4" type="hinge" axis="0 0 1" range="{q4lo} {q4hi}" armature="1" damping="4"/>
              <geom class="arm" type="capsule" fromto="0 0 0 0 0 -{mast}" size="0.035" mass="5" rgba="0.3 0.31 0.33 1"/>
              <geom class="look" type="cylinder" pos="0 0 {tmz}" size="{tmr} 0.002" rgba="0.90 0.10 0.75 1" mass="0"/>
              <geom class="look" type="cylinder" pos="{tdx} 0 {tmz2}" size="{tdr} 0.002" rgba="0.98 0.90 0.10 1" mass="0"/>
              <body name="cup" pos="0 0 -{cup_z}">
                <joint name="jc" type="slide" axis="0 0 1" range="0 {stroke}" stiffness="{cup_k}" damping="90" solreflimit="0.004 1"/>
                <geom name="pad" class="arm" type="box" pos="0 0 -0.010" size="{padx} {pady} 0.010" mass="{cup_m}" rgba="0.12 0.12 0.13 1" friction="0.9 0.005 0.0001"/>
                <site name="flange" pos="0 0 0" size="0.01"/>
                <site name="tcp" pos="0 0 -0.020" size="0.01"/>
              </body>
            </body>
          </body>
        </body>
      </body>
    </body>
{cameras}
{bodies}
  </worldbody>
  <tendon>
    <fixed name="level"><joint joint="j2" coef="1"/><joint joint="j3" coef="1"/><joint joint="jw" coef="1"/></fixed>
  </tendon>
  <equality>
    <tendon name="level" tendon1="level" polycoef="0 1 0 0 0" solref="0.004 1"/>
{welds}
  </equality>
  <actuator>
    <position name="a1" joint="j1" kp="500000" kv="50000" forcerange="-6000 6000"/>
    <position name="a2" joint="j2" kp="600000" kv="60000" forcerange="-9000 9000"/>
    <position name="a3" joint="j3" kp="500000" kv="45000" forcerange="-7000 7000"/>
    <position name="a4" joint="j4" kp="40000" kv="1200" forcerange="-400 400"/>
  </actuator>
  <sensor>
    <force name="ft_force" site="flange"/>
    <torque name="ft_torque" site="flange"/>
  </sensor>
</mujoco>
"""


class PalletCell(object):
    """The palletizing cell. See the module docstring for use, and ACTION / OBSERVATION below.

    ACTION: 7 numbers [vx, vy, vz, wyaw, vacuum, cams, done].
      vx, vy, vz   velocity of the pad-face centre in the world frame (m/s, each clipped to
                   +-V_MAX). The arm's target moves with a velocity that ramps linearly from the
                   previous command to this one over the control step (so the commanded
                   acceleration is the change of the command divided by CONTROL_DT), and the
                   joints servo to that target.
      wyaw         tool yaw rate (rad/s, clipped to +-W_MAX), ramped the same way.
      vacuum       1 = vacuum on, 0 = off.
      cams         bit mask of the cameras whose images the next observation carries
                   (1 infeed, 2 pallet, 4 buffer): an integer from 0 to 7.
      done         1 = the order is finished: the episode ends after this step.

    OBSERVATION: a JSON-serialisable dict, see _observation().
    """

    def __init__(self, catalog, render=True):
        self.catalog = dict(catalog)
        self.render_enabled = bool(render)
        self.model = None
        self.data = None
        self._renderers = {}
        self.done = True

    # ------------------------------------------------------------------ building the model

    def _build(self, order, rng):
        n = len(order)
        assets, bodies, welds, files = [], [], [], {}
        self.layouts = []
        for i, sku in enumerate(order):
            spec = self.catalog[sku]
            L, W, H = (float(v) for v in spec["size"])
            mass = float(spec["mass"])
            com = [float(v) for v in spec.get("com", (0.0, 0.0))]
            faces, layout = carton_textures(spec, rng)
            self.layouts.append(layout)
            names = {}
            for face, png in faces.items():
                name = "c{}_{}.png".format(i, face)
                files[name] = png
                names[face] = name
            assets.append(
                '    <texture name="tex{i}" type="cube" fileright="{right}" fileleft="{left}" fileup="{up}" '
                'filedown="{down}" filefront="{front}" fileback="{back}"/>\n'
                '    <material name="mat{i}" texture="tex{i}" specular="0.05" shininess="0.05"/>'.format(i=i, **names))
            # A carton is a uniform shell around a load whose centre of gravity may sit off
            # centre: the inertia is that of a uniform block, about the shifted centre.
            ixx = mass * (W * W + H * H) / 12.0
            iyy = mass * (L * L + H * H) / 12.0
            izz = mass * (L * L + W * W) / 12.0
            bodies.append(
                '    <body name="box{i}" pos="{x} 40 5">\n'
                '      <freejoint name="free{i}"/>\n'
                '      <inertial pos="{cx} {cy} 0" mass="{m}" diaginertia="{ixx} {iyy} {izz}"/>\n'
                '      <geom name="gbox{i}" type="box" size="{hx} {hy} {hz}" material="mat{i}"/>\n'
                '    </body>'.format(i=i, x=40.0 + 1.5 * i, cx=com[0], cy=com[1], m=mass, ixx=ixx, iyy=iyy,
                                     izz=izz, hx=L / 2, hy=W / 2, hz=H / 2))
            welds.append('    <weld name="seal{i}" body1="cup" body2="box{i}" active="false" solref="0.005 1"/>'.format(i=i))
        cams = []
        self.cam_pose = {}
        for name, (w, h, fovy, pos, look, up) in CAMERAS.items():
            pos = np.array(pos) + rng.uniform(-CAM_JITTER_POS, CAM_JITTER_POS, size=3)
            fwd = np.array(look) - pos
            fwd = fwd + np.linalg.norm(fwd) * rng.uniform(-CAM_JITTER_ROT, CAM_JITTER_ROT, size=3) * np.array([1, 1, 0])
            fwd /= np.linalg.norm(fwd)
            right = np.cross(fwd, np.array(up, dtype=float))
            right /= np.linalg.norm(right)
            cam_up = np.cross(right, fwd)
            cams.append('    <camera name="{}" pos="{} {} {}" xyaxes="{} {} {} {} {} {}" fovy="{}"/>'.format(
                name, *pos, *right, *cam_up, fovy))
            T = np.eye(4)
            T[:3, 0], T[:3, 1], T[:3, 2], T[:3, 3] = right, -cam_up, fwd, pos
            f = (h / 2.0) / math.tan(math.radians(fovy) / 2.0)
            K = [[f, 0.0, (w - 1) / 2.0], [0.0, f, (h - 1) / 2.0], [0.0, 0.0, 1.0]]
            self.cam_pose[name] = {"T_world_cam": [[round(float(v), 6) for v in row] for row in T],
                                   "K": [[round(float(v), 4) for v in row] for row in K],
                                   "width": w, "height": h}
        marks = []
        for group in SURVEY_MARKS.values():
            for x, y, z in group:
                marks.append('    <geom class="look" type="cylinder" pos="{} {} {}" size="{} 0.0008" rgba="0.96 0.96 0.96 1"/>'
                             .format(x, y, z + 0.0008, MARK_R))
                marks.append('    <geom class="look" type="cylinder" pos="{} {} {}" size="{} 0.0008" rgba="0.03 0.03 0.03 1"/>'
                             .format(x, y, z + 0.0020, MARK_DOT_R))
        belt_h, table_h, deck_h = BELT_Z / 2, TABLE_Z / 2, DECK_Z / 2
        xml = _XML_HEAD.format(
            dt=TIMESTEP, g=G, assets="\n".join(assets), cameras="\n".join(cams), bodies="\n".join(bodies),
            welds="\n".join(welds),
            belt_c=belt_h, belt_h=belt_h, stop_c=BELT_Z + 0.03,
            tx=TABLE[0], ty=TABLE[1], table_c=table_h, table_h=table_h, thx=TABLE_HALF[0], thy=TABLE_HALF[1],
            px=self.pallet_xy[0], py=self.pallet_xy[1], pyaw=self.pallet_yaw, deck_c=deck_h, deck_h=deck_h,
            phx=PALLET_HALF[0], phy=PALLET_HALF[1], deck_top=DECK_Z, marks="\n".join(marks),
            tmz=TOOL_MARK_Z, tmz2=TOOL_MARK_Z + 0.002, tmr=TOOL_MARK_R, tdx=TOOL_DOT_X, tdr=TOOL_DOT_R,
            q1lo=Q_MIN[0], q1hi=Q_MAX[0], q2lo=Q_MIN[1], q2hi=Q_MAX[1], q3lo=Q_MIN[2], q3hi=Q_MAX[2],
            q4lo=Q_MIN[3], q4hi=Q_MAX[3],
            a0=A0, a0h=A0 / 2, h0r=H0 - 0.60, l1=L1, l2=L2, a3=A3, a3h=A3 / 2,
            mast=D_TOOL - 0.06, cup_z=D_TOOL - 0.020, stroke=CUP_STROKE, cup_k=CUP_STIFFNESS,
            padx=PAD_L / 2, pady=PAD_W / 2, cup_m=CUP_MASS)
        self.model = mujoco.MjModel.from_xml_string(xml, files)
        self.data = mujoco.MjData(self.model)
        m = self.model
        self.n = n
        ident = lambda kind, name: mujoco.mj_name2id(m, kind, name)
        self.box_body = np.array([ident(mujoco.mjtObj.mjOBJ_BODY, "box{}".format(i)) for i in range(n)])
        self.box_geom = np.array([ident(mujoco.mjtObj.mjOBJ_GEOM, "gbox{}".format(i)) for i in range(n)])
        self.box_qadr = np.array([m.jnt_qposadr[m.body_jntadr[b]] for b in self.box_body])
        self.box_vadr = np.array([m.jnt_dofadr[m.body_jntadr[b]] for b in self.box_body])
        self.weld = np.array([ident(mujoco.mjtObj.mjOBJ_EQUALITY, "seal{}".format(i)) for i in range(n)])
        self.geom_box = -np.ones(m.ngeom, dtype=int)
        self.geom_box[self.box_geom] = np.arange(n)
        self.cup_body = ident(mujoco.mjtObj.mjOBJ_BODY, "cup")
        self.tcp_site = ident(mujoco.mjtObj.mjOBJ_SITE, "tcp")
        self.flange_site = ident(mujoco.mjtObj.mjOBJ_SITE, "flange")
        self.arm_qadr = np.array([m.jnt_qposadr[ident(mujoco.mjtObj.mjOBJ_JOINT, j)] for j in ("j1", "j2", "j3", "j4")])
        self.arm_vadr = np.array([m.jnt_dofadr[ident(mujoco.mjtObj.mjOBJ_JOINT, j)] for j in ("j1", "j2", "j3", "j4")])
        self.jw_qadr = m.jnt_qposadr[ident(mujoco.mjtObj.mjOBJ_JOINT, "jw")]
        self.jc_qadr = m.jnt_qposadr[ident(mujoco.mjtObj.mjOBJ_JOINT, "jc")]
        self.half = np.array([[float(v) / 2 for v in self.catalog[s]["size"]] for s in order])
        self.mass = np.array([float(self.catalog[s]["mass"]) for s in order])
        self.volume = np.prod(2 * self.half, axis=1)
        self.fragile = np.array([bool(self.catalog[s].get("fragile")) for s in order])
        limits = []
        for s in order:
            lim = self.catalog[s].get("stack_limit")
            limits.append(LOAD_UNMARKED_KG if lim is None else max(float(lim), LOAD_NO_STACK_KG))
        self.load_limit = np.array(limits) * G
        self.shock_limit = np.where(self.fragile, SHOCK_LIMIT_FRAGILE, SHOCK_LIMIT)
        self.light_base = m.light_diffuse.copy()
        self.light_dir_base = m.light_dir.copy()
        self._close_renderers()

    def _close_renderers(self):
        for renderer in self._renderers.values():
            try:
                renderer.close()
            except Exception:       # a GL context that fails to free must not stop a run
                pass
        self._renderers = {}

    # ------------------------------------------------------------------ reset

    def reset(self, order, seed, raw=False):
        """Start an order: `order` is the list of catalogue ids in arrival order, `seed` an
        integer that decides the printing, the arrival poses, the lighting and the sensor noise."""
        seed = int(seed)
        streams = np.random.SeedSequence(seed).spawn(5)
        rng_build, self.rng_arrive, self.rng_sense, self.rng_look, rng_cal = (np.random.default_rng(s) for s in streams)
        self.order = list(order)
        self.q_off = rng_cal.uniform(-1.0, 1.0, size=4) * Q_OFFSET_MAX
        self.pallet_xy = PALLET + rng_cal.uniform(-PALLET_SHIFT, PALLET_SHIFT, size=2)
        self.pallet_yaw = float(rng_cal.uniform(-PALLET_TURN, PALLET_TURN))
        self._build(self.order, rng_build)
        m, d = self.model, self.data
        # Parked cartons float out of sight, far from the cell, without contacts.
        m.geom_contype[self.box_geom] = 0
        m.geom_conaffinity[self.box_geom] = 0
        m.body_gravcomp[self.box_body] = 1.0
        # The controller and the observation live in the encoders' frame (reported angles); the
        # joints themselves stand at reported minus q_off.
        q_true = Q_HOME - self.q_off
        d.qpos[self.arm_qadr] = q_true
        d.qpos[self.jw_qadr] = -(q_true[1] + q_true[2])
        d.ctrl[:4] = q_true
        mujoco.mj_forward(m, d)
        xyz, yaw = fk(Q_HOME)
        self.target = np.array([xyz[0], xyz[1], xyz[2], yaw])
        self.q_cmd = Q_HOME.copy()
        self.vel_cmd = np.zeros(4)
        self.ik_ok = True
        self.vacuum = False
        self.held = -1                 # index of the sealed carton, or -1
        self.seal_lost = False         # the seal broke; vacuum must be switched off to re-arm
        self.seal_force = 0.0
        self.seal_moment = 0.0
        self.seal_load = np.zeros(6)   # filtered load on the seal
        self.state = np.zeros(self.n, dtype=int)     # 0 parked, 1 delivered
        self.delivered = 0
        self.clear_time = INFEED_DELAY - 1.0         # the first carton arrives after 1 s
        self.damage = [None] * self.n                # None or (reason, time)
        self.load = np.zeros(self.n)
        self.peak_load = np.zeros(self.n)
        self.peak_shock = np.zeros(self.n)
        window = int(round(SHOCK_WINDOW / TIMESTEP))
        self.vel_hist = np.zeros((window, self.n, 3))
        self.hist_i = 0
        self.fresh = np.zeros(self.n, dtype=int)     # physics steps to skip the shock rule after a teleport
        self.t = 0
        self.time = 0.0
        self.max_steps = STEPS_PER_CARTON * self.n + STEPS_FIXED
        self.cams_next = 7
        self.done = False
        self.finished = False
        self.unstable = False
        # Lighting of this order: direction and strength of the key light, and a colour cast.
        m.light_dir[0] = self.light_dir_base[0] + np.append(self.rng_look.uniform(-0.45, 0.45, size=2), 0.0)
        m.light_diffuse[0] = self.light_base[0] * self.rng_look.uniform(0.75, 1.25)
        self.tint = self.rng_look.uniform(0.90, 1.10, size=3)
        return self._observation(raw)

    # ------------------------------------------------------------------ helpers on cartons

    def box_pose(self, i):
        """World position (centre) and 3x3 rotation of carton i."""
        b = self.box_body[i]
        return self.data.xpos[b].copy(), self.data.xmat[b].reshape(3, 3).copy()

    def _top_face(self, i):
        """The upward face of carton i: (centre, normal, axis a, axis b, half a, half b), or None
        when no face is within 6 degrees of level."""
        pos, R = self.box_pose(i)
        k = int(np.argmax(np.abs(R[2])))
        nz = R[2, k]
        if abs(nz) < math.cos(math.radians(6.0)):
            return None
        sign = 1.0 if nz > 0 else -1.0
        normal = sign * R[:, k]
        a, b = [j for j in range(3) if j != k]
        return pos + normal * self.half[i, k], normal, R[:, a], R[:, b], self.half[i, a], self.half[i, b]

    def _pad(self):
        """Pad-face centre (world) and the pad's yaw."""
        p = self.data.site_xpos[self.tcp_site]
        R = self.data.site_xmat[self.tcp_site].reshape(3, 3)
        return p, math.atan2(R[1, 0], R[0, 0])

    def _station_occupied(self):
        for i in np.nonzero(self.state == 1)[0]:
            p = self.data.xpos[self.box_body[i]]
            if abs(p[0] - STATION[0]) < STATION_HALF[0] + 0.15 and abs(p[1] - STATION[1]) < STATION_HALF[1] and \
                    p[2] < BELT_Z + 0.55 and i != self.held:
                return True
        if self.held >= 0:
            p = self.data.xpos[self.box_body[self.held]]
            if abs(p[0] - STATION[0]) < STATION_HALF[0] and abs(p[1] - STATION[1]) < STATION_HALF[1] and \
                    p[2] - self.half[self.held].max() < BELT_Z + 0.02:
                return True
        return False

    def _present(self):
        """The station's photo-eye: a carton rests in the pick station."""
        for i in np.nonzero(self.state == 1)[0]:
            if i == self.held:
                continue
            p = self.data.xpos[self.box_body[i]]
            if abs(p[0] - STATION[0]) < STATION_HALF[0] and abs(p[1] - STATION[1]) < STATION_HALF[1] and \
                    p[2] < BELT_Z + 0.45:
                return True
        return False

    def _deliver(self):
        """The next carton arrives in the pick station: skewed, a little off centre, long side
        along the conveyor or across it."""
        i = self.delivered
        L, W, H = 2 * self.half[i]
        quarter = int(self.rng_arrive.integers(0, 2))
        yaw = quarter * math.pi / 2 + self.rng_arrive.uniform(-math.radians(20), math.radians(20))
        xy = STATION + self.rng_arrive.uniform(-0.04, 0.04, size=2)
        adr = self.box_qadr[i]
        self.data.qpos[adr:adr + 3] = [xy[0], xy[1], BELT_Z + H / 2 + 0.001]
        self.data.qpos[adr + 3:adr + 7] = [math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)]
        self.data.qvel[self.box_vadr[i]:self.box_vadr[i] + 6] = 0.0
        self.model.geom_contype[self.box_geom[i]] = 1
        self.model.geom_conaffinity[self.box_geom[i]] = 3
        self.model.body_gravcomp[self.box_body[i]] = 0.0
        self.state[i] = 1
        self.fresh[i] = int(0.3 / TIMESTEP)
        self.delivered += 1

    # ------------------------------------------------------------------ vacuum seal

    def _try_seal(self):
        pad, yaw = self._pad()
        best, best_area, best_face = -1, SEAL_MIN_AREA, None
        for i in np.nonzero(self.state == 1)[0]:
            if np.linalg.norm(self.data.xpos[self.box_body[i]] - pad) > 0.75:
                continue
            face = self._top_face(i)
            if face is None:
                continue
            centre, normal, a, b, ha, hb = face
            gap = pad[2] - (centre[2] + (normal[0] * (pad[0] - centre[0]) + normal[1] * (pad[1] - centre[1])) / -normal[2])
            if not SEAL_GAP_LO <= gap <= SEAL_GAP_HI:
                continue
            area = overlap_area(pad[:2], yaw, (PAD_L / 2, PAD_W / 2), centre[:2], a[:2] / np.linalg.norm(a[:2]),
                                b[:2] / np.linalg.norm(b[:2]), (ha, hb))
            if area >= best_area:
                best, best_area, best_face = i, area, face
        if best < 0:
            return
        m, d = self.model, self.data
        cup_p, cup_R = d.xpos[self.cup_body], d.xmat[self.cup_body].reshape(3, 3)
        box_p, box_R = self.box_pose(best)
        # The seal holds the carton where it is, 0.5 mm below the pad.
        gap = pad[2] - best_face[0][2]
        box_p = box_p + np.array([0.0, 0.0, gap - 0.0005])
        rel_R = cup_R.T @ box_R
        quat = np.zeros(4)
        mujoco.mju_mat2Quat(quat, rel_R.flatten())
        seal_point = np.array([pad[0], pad[1], pad[2]])
        eq = self.weld[best]
        m.eq_data[eq, 0:3] = box_R.T @ (seal_point - box_p)        # the seal point in the carton frame
        m.eq_data[eq, 3:6] = cup_R.T @ (seal_point - cup_p)        # the same point in the cup's frame
        m.eq_data[eq, 6:10] = quat
        m.eq_data[eq, 10] = 1.0
        d.eq_active[eq] = 1
        self.held = best
        self.seal_force = SEAL_PRESSURE * best_area
        self.seal_moment = SEAL_MOMENT_ARM * self.seal_force * math.sqrt(best_area)
        self.seal_load[:] = self.wrench()

    def _release(self):
        if self.held >= 0:
            self.data.eq_active[self.weld[self.held]] = 0
            self.held = -1

    def _check_seal(self):
        """The seal breaks when the pull on it and the moment on it together exceed what the
        vacuum holds: (pull + 0.5 shear) / seal_force + moment / seal_moment > 1. The load is
        what the wrist sensor measures (wrench(), without noise) less the cup's own weight,
        low-pass filtered: pull is the downward force, shear the horizontal force, moment the
        torque about the pad-face centre about horizontal axes."""
        raw = self.wrench()
        alpha = TIMESTEP / (SEAL_FILTER_TAU + TIMESTEP)
        self.seal_load += alpha * (raw - self.seal_load)
        pull = max(0.0, -self.seal_load[2] - CUP_MASS * G)
        shear = math.hypot(self.seal_load[0], self.seal_load[1])
        moment = math.hypot(self.seal_load[3], self.seal_load[4])
        self.seal_stress = (pull + 0.5 * shear) / self.seal_force + moment / self.seal_moment
        if self.seal_stress > 1.0:
            self._release()
            self.seal_lost = True

    # ------------------------------------------------------------------ handling rules

    def _monitor(self, step_index):
        d = self.data
        active = self.state == 1
        v = np.stack([d.qvel[a:a + 3] for a in self.box_vadr])
        old = self.vel_hist[self.hist_i]
        shock = np.linalg.norm(v - old, axis=1) / SHOCK_WINDOW
        self.vel_hist[self.hist_i] = v
        self.hist_i = (self.hist_i + 1) % self.vel_hist.shape[0]
        counted = active & (self.fresh == 0)
        self.fresh[self.fresh > 0] -= 1
        shock = np.where(counted, shock, 0.0)
        self.peak_shock = np.maximum(self.peak_shock, shock)
        for i in np.nonzero(shock > self.shock_limit)[0]:
            if self.damage[i] is None:
                self.damage[i] = ("shock", round(self.time, 3))
        if step_index % LOAD_PERIOD:
            return
        load = np.zeros(self.n)
        ncon = d.ncon
        if ncon:
            con = d.contact
            g1, g2 = con.geom1[:ncon], con.geom2[:ncon]
            b1, b2 = self.geom_box[g1], self.geom_box[g2]
            both = (b1 >= 0) & (b2 >= 0) & (con.efc_address[:ncon] >= 0)
            for c in np.nonzero(both)[0]:
                normal_z = abs(con.frame[c, 2])
                if normal_z < 0.5:
                    continue
                force = d.efc_force[con.efc_address[c]] * normal_z
                i, j = b1[c], b2[c]
                lower = i if d.xpos[self.box_body[i], 2] < d.xpos[self.box_body[j], 2] else j
                load[lower] += force
        alpha = (LOAD_PERIOD * TIMESTEP) / LOAD_FILTER_TAU
        self.load += alpha * (load - self.load)
        self.peak_load = np.maximum(self.peak_load, self.load)
        for i in np.nonzero(self.load > self.load_limit)[0]:
            if self.damage[i] is None:
                self.damage[i] = ("load", round(self.time, 3))

    # ------------------------------------------------------------------ stepping

    def _physics(self, substeps, v0=None, v1=None):
        m, d = self.model, self.data
        for k in range(substeps):
            if v1 is not None:
                vel = v0 + (v1 - v0) * ((k + 0.5) / substeps)
                target = self.target + vel * TIMESTEP
                r = math.hypot(target[0], target[1])
                if r > R_MAX or r < R_MIN:
                    scale = min(max(r, R_MIN), R_MAX) / max(r, 1e-9)
                    target[0] *= scale
                    target[1] *= scale
                target[2] = min(max(target[2], Z_MIN), Z_MAX)
                q = ik(target[:3], target[3])
                if q is not None and np.all(q >= Q_MIN) and np.all(q <= Q_MAX):
                    self.target, self.q_cmd, self.ik_ok = target, q, True
                else:
                    self.ik_ok = False
                d.ctrl[:4] = self.q_cmd - self.q_off
            if self.vacuum and self.held < 0 and not self.seal_lost:
                self._try_seal()
            mujoco.mj_step(m, d)
            self.time += TIMESTEP
            if self.held >= 0:
                self._check_seal()
            self._monitor(k)
            if self.delivered < self.n:
                if self._station_occupied():
                    self.clear_time = 0.0
                else:
                    self.clear_time += TIMESTEP
                    if self.clear_time >= INFEED_DELAY:
                        self._deliver()
                        self.clear_time = 0.0
        if v1 is not None:
            # The target never runs far ahead of an arm that cannot follow.
            actual, _ = fk(d.qpos[self.arm_qadr] + self.q_off)
            lead = self.target[:3] - actual
            dist = float(np.linalg.norm(lead))
            if dist > TARGET_LEAD_MAX:
                self.target[:3] = actual + lead * (TARGET_LEAD_MAX / dist)
                q = ik(self.target[:3], self.target[3])
                if q is not None:
                    self.q_cmd = np.clip(q, Q_MIN, Q_MAX)
        if not np.all(np.isfinite(d.qpos)) or np.any(d.warning.number > 0):
            self.unstable = True

    def step(self, action, raw=False):
        """Advance one control step (CONTROL_DT). Returns the next observation."""
        if self.done:
            raise RuntimeError("the episode is over: call reset()")
        a = [float(x) for x in action]
        if len(a) != 7 or not all(math.isfinite(x) for x in a):
            raise ValueError("an action is 7 finite numbers")
        vel = np.array([max(-V_MAX, min(V_MAX, a[0])), max(-V_MAX, min(V_MAX, a[1])),
                        max(-V_MAX, min(V_MAX, a[2])), max(-W_MAX, min(W_MAX, a[3]))])
        vacuum = a[4] >= 0.5
        cams = int(round(a[5]))
        if cams < 0 or cams > 7 or abs(a[5] - cams) > 1e-6:
            raise ValueError("cams is an integer from 0 to 7")
        if not vacuum:
            self._release()
            self.seal_lost = False
        self.vacuum = vacuum
        self._physics(SUBSTEPS, self.vel_cmd, vel)
        self.vel_cmd = vel
        self.t += 1
        self.cams_next = cams
        if a[6] >= 0.5 or self.t >= self.max_steps or self.unstable:
            self.done = True
        return self._observation(raw)

    # ------------------------------------------------------------------ sensing

    def wrench(self):
        """Force (N) and torque (N m) that the cup and whatever hangs on it exert on the arm's
        flange, in world axes, the torque about the pad-face centre. A mass m hanging at rest
        reads Fz = -m * G (the cup itself reads -CUP_MASS * G); pressing the pad down on a
        surface reads a positive Fz. No noise here."""
        d = self.data
        R = d.site_xmat[self.flange_site].reshape(3, 3)
        f = -(R @ d.sensordata[0:3])
        tq = -(R @ d.sensordata[3:6])
        arm = d.site_xpos[self.tcp_site] - d.site_xpos[self.flange_site]
        tq = tq - np.cross(arm, f)
        return np.concatenate([f, tq])

    def truth(self):
        """What the cell really is, for the reference's token runs only (tests/verify.py): the
        hidden calibration of this order and the state of every carton that has arrived."""
        cartons = []
        for i in np.nonzero(self.state == 1)[0]:
            pos, R = self.box_pose(int(i))
            spec = self.catalog[self.order[int(i)]]
            cartons.append({
                "index": int(i), "pos": [float(v) for v in pos], "yaw": float(math.atan2(R[1, 0], R[0, 0])),
                "upright": bool(R[2, 2] > 0.99), "size": [float(v) for v in spec["size"]],
                "mass": float(spec["mass"]), "com": [float(v) for v in spec.get("com", (0.0, 0.0))],
                "stack_limit": spec.get("stack_limit"), "fragile": bool(spec.get("fragile")),
                "held": bool(int(i) == self.held)})
        return {"q_off": [float(v) for v in self.q_off], "pallet_xy": [float(v) for v in self.pallet_xy],
                "pallet_yaw": float(self.pallet_yaw), "cameras": self.cam_pose, "cartons": cartons}

    def render(self, name):
        """RGB (uint8, HxWx3) and depth (uint16, millimetres along the optical axis, 0 = no
        reading) of one camera, with the order's lighting, colour cast and sensor noise."""
        w, h = CAMERAS[name][0], CAMERAS[name][1]
        key = (w, h)
        if key not in self._renderers:
            self._renderers[key] = mujoco.Renderer(self.model, h, w)
        renderer = self._renderers[key]
        renderer.disable_depth_rendering()
        renderer.update_scene(self.data, camera=name)
        rgb = renderer.render().astype(np.float32)
        renderer.enable_depth_rendering()
        renderer.update_scene(self.data, camera=name)
        depth = renderer.render().astype(np.float64)
        renderer.disable_depth_rendering()
        rgb = rgb * self.tint + self.rng_sense.normal(0.0, 1.5, size=rgb.shape)
        rgb = np.clip(rgb, 0, 255).astype(np.uint8)
        depth = depth + self.rng_sense.normal(0.0, 1.0, size=depth.shape) * DEPTH_SIGMA * depth
        depth[self.rng_sense.random(size=depth.shape) < DEPTH_DROPOUT] = 0.0
        depth[(depth < 0.05) | (depth > 20.0)] = 0.0
        return rgb, np.round(depth * 1000.0).astype(np.uint16)

    def _observation(self, raw=False):
        """The observation. With raw=True the images are numpy arrays instead of base64 fields."""
        d = self.data
        q = d.qpos[self.arm_qadr] + self.q_off
        dq = d.qvel[self.arm_vadr]
        xyz, yaw = fk(q)
        wrench = self.wrench()
        wrench[:3] += self.rng_sense.normal(0.0, WRENCH_SIGMA_F, size=3)
        wrench[3:] += self.rng_sense.normal(0.0, WRENCH_SIGMA_T, size=3)
        obs = {
            "t": self.t,
            "q": [round(float(v), 6) for v in q],
            "dq": [round(float(v), 6) for v in dq],
            "tcp": [round(float(v), 6) for v in (xyz[0], xyz[1], xyz[2], yaw)],
            "wrench": [round(float(v), 4) for v in wrench],
            "vacuum": int(self.vacuum),
            "sealed": int(self.held >= 0),
            "in_reach": int(self.ik_ok),
            "infeed_present": int(self._present()),
            "delivered": int(self.delivered),
            "total": int(self.n),
            "steps_left": int(self.max_steps - self.t),
            "images": {},
        }
        for name, bit in CAMERA_BITS.items():
            if self.render_enabled and self.cams_next & bit:
                rgb, depth = self.render(name)
                depth = np.stack([(depth & 0xFF).astype(np.uint8), (depth >> 8).astype(np.uint8)], axis=-1)
            else:
                rgb, depth = np.zeros((0, 0, 3), dtype=np.uint8), np.zeros((0, 0, 2), dtype=np.uint8)
            obs["images"][name] = {"rgb": rgb, "depth": depth} if raw else {"rgb": _encode(rgb), "depth": _encode(depth)}
        return obs

    # ------------------------------------------------------------------ the end of an order

    def _on_pallet(self, i):
        """Whether carton i stands on the pallet's stack within the shipping limits, and why not."""
        pos, R = self.box_pose(i)
        h = self.half[i]
        if R[2, 2] < math.cos(LEVEL_TOL):
            return "not upright"
        corners = np.array([[sx * h[0], sy * h[1], sz * h[2]] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])
        world = pos + corners @ R.T
        c, s_ = math.cos(self.pallet_yaw), math.sin(self.pallet_yaw)
        rel = world[:, :2] - self.pallet_xy
        local = np.stack([c * rel[:, 0] + s_ * rel[:, 1], -s_ * rel[:, 0] + c * rel[:, 1]], axis=1)
        if np.any(np.abs(local) > PALLET_HALF + OVERHANG_TOL):
            return "outside the pallet"
        if world[:, 2].min() < DECK_Z - 0.005:
            return "below the deck"
        if world[:, 2].max() > DECK_Z + STACK_HEIGHT_MAX + 0.005:
            return "above the height limit"
        return None

    def finish(self):
        """End the order: release the gripper, let the cell settle, run the tilt test and decide
        which cartons ship. Returns {"shipped_volume_fraction", "cartons": [...]}.

        The tilt test tilts gravity by TILT_ANGLE toward +x, -x, +y and -y in turn (ramp up over
        TILT_RAMP, hold TILT_HOLD, ramp down), which is what a pallet tilted that way feels. A
        carton ships when, after the test, it stands on the pallet's stack inside the limits
        (_on_pallet), it has not moved more than MOVE_TOL or turned more than TURN_TOL during
        the test, and it was never damaged (shock or load, the test included)."""
        if self.finished:
            raise RuntimeError("finish() was already called")
        self.finished = True
        self.done = True
        m, d = self.model, self.data
        self._release()
        self.vacuum = False
        if self.unstable:
            return self._result(["the simulation became unstable"] * self.n)
        self._physics(int(SETTLE_TIME / TIMESTEP))
        before = [self.box_pose(i) for i in range(self.n)]
        for direction in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            for duration, a0, a1 in ((TILT_RAMP, 0.0, 1.0), (TILT_HOLD, 1.0, 1.0), (TILT_RAMP, 1.0, 0.0)):
                steps = int(duration / TIMESTEP)
                chunk = 10
                for s in range(0, steps, chunk):
                    angle = TILT_ANGLE * (a0 + (a1 - a0) * (s + chunk) / steps)
                    m.opt.gravity[:] = [G * math.sin(angle) * direction[0], G * math.sin(angle) * direction[1],
                                        -G * math.cos(angle)]
                    self._physics(chunk)
        m.opt.gravity[:] = [0.0, 0.0, -G]
        self._physics(int(SETTLE_TIME / TIMESTEP))
        reasons = []
        for i in range(self.n):
            reason = None
            if self.state[i] != 1:
                reason = "not delivered"
            if reason is None:
                reason = self._on_pallet(i)
            if reason is None:
                pos, R = self.box_pose(i)
                if np.linalg.norm(pos - before[i][0]) > MOVE_TOL:
                    reason = "moved in the tilt test"
                else:
                    turn = math.acos(max(-1.0, min(1.0, (np.trace(before[i][1].T @ R) - 1.0) / 2.0)))
                    if turn > TURN_TOL:
                        reason = "turned in the tilt test"
            if reason is None and self.damage[i] is not None:
                reason = "damaged ({} at {} s)".format(*self.damage[i])
            reasons.append(reason)
        if self.unstable:
            reasons = ["the simulation became unstable"] * self.n
        return self._result(reasons)

    def _result(self, reasons):
        shipped = [r is None for r in reasons]
        total = float(self.volume.sum())
        cartons = []
        for i in range(self.n):
            cartons.append({
                "index": i, "sku": self.order[i], "shipped": bool(shipped[i]), "reason": reasons[i],
                "volume": round(float(self.volume[i]), 6),
                "peak_load_kg": round(float(self.peak_load[i] / G), 3),
                "peak_shock": round(float(self.peak_shock[i]), 2),
            })
        fraction = float(self.volume[np.array(shipped, dtype=bool)].sum()) / total if total > 0 else 0.0
        return {"shipped_volume_fraction": fraction, "cartons": cartons, "steps": int(self.t)}

    def close(self):
        self._close_renderers()

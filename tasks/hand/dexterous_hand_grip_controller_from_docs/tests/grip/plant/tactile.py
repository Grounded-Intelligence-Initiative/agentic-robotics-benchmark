"""The four tactile modules: the taxel maps on the pads, the contact force spread over the taxels, the encoding of a
module's bytes, each taxel's zero and its slow drift, the re-zero, and the frame stream.

Maps. Each module type's map gives every taxel's position (mm) in the module frame; model/taxel_maps.json holds the
maps, where each module frame sits on its pad's link (fitted so that the taxels lie on the pad's surface), each
map's outline (its outermost taxels in order) and each pad's centre in its module frame.

Force. Every pad-specimen contact (position c, normal force f_n, the force it applies to the pad) is spread over the
module's taxels with weights w_i = A N(|x_i - c|; sigma), N the two-dimensional normal density, so that the part of
the kernel beyond the array is lost; A is the module's kernel area (by the rule in the hidden constants: the kernel
sums to one at the pad's centre, or the map's pitch squared). Every value is multiplied by the module's gain and by
an edge factor e = floor + (1 - floor) min(1, d / ramp), d the distance of the module's force-weighted contact
centroid to the map's outline (0 when the centroid lies outside it). A taxel's Fz is its share of the normal force;
its Fx, Fy are its share of the shear the contacts apply to the pad, along the module frame's x and y. The resultant
block is gain x e x the total force on the pad in the module frame, its z counted positive into the pad.

Encoding. The resultant block: 3 x int16 little-endian at 0.01 N per LSB. Then per taxel Fx, Fy as int8 and Fz as
uint8 at 0.1 N per step, each rounded. A taxel's Fz carries its baseline (the delivered zero residual plus a reflected
random walk drawn at every frame); a reading below the taxel's zero wraps modulo 256 (one step below reads 0xFF), one
of 255 steps or more reads 255; Fx and Fy saturate at -128 and 127. A re-zero sets a module's zero to the values it
reports at that instant, contact included.

Frames. The board captures a frame of all four modules every 1 / f s (f and the first frame's phase are the unit's);
a frame arrives at the host the fixed transfer time after its capture. Every random draw comes from the unit's drift
seed through noise.py, at every frame for every taxel, whether or not anyone reads it. A stream may carry a schedule
of dropouts (capture-time windows whose frames never reach the host); the acceptance's schedule is empty.
"""
import collections
import json
import math
import os
import struct

import numpy as np

from contract import constants as C
from contract import names as N
from plant import MODEL_DIR, TRUTH, noise

MAPS_FILE = "taxel_maps.json"
MAPS_FORMAT = "grip-taxel-maps/1"
_TAC = TRUTH["tactile"]
_TC = C.TACTILE
STEP_N = _TC["taxel_n_per_step"]
LSB_N = _TC["resultant_n_per_lsb"]
XY_LO, XY_HI = _TC["fxy_saturation"]
FZ_TOP = _TC["fz_saturation_reads"]
ARRIVAL_MS = _TC["arrival_after_capture_ms"]
I16 = (-(1 << 15), (1 << 15) - 1)
BYTE = 1 << 8                                    # the values one byte holds
KEEP_FRAMES = 4


# ------------------------------------------------------------------------------------------------- maps
class Maps:
    """The taxel maps and where each module frame sits on its pad's link (model/taxel_maps.json)."""

    def __init__(self, model_dir=None):
        md = MODEL_DIR if model_dir is None else model_dir
        with open(os.path.join(md, MAPS_FILE), encoding="utf-8") as fh:
            rec = json.load(fh)
        if rec.get("format") != MAPS_FORMAT:
            raise ValueError("%s: not a taxel-maps file" % MAPS_FILE)
        self.types = {}
        for kind, v in rec["maps"].items():
            self.types[kind] = {"P": np.array(v["taxels_mm"], np.float64), "outline": np.array(v["outline"], int)}
        self.modules = {}
        col = 0
        for mod in N.MODULES:
            v = rec["modules"][mod]
            P = self.types[v["type"]]["P"]
            if len(P) != N.MODULE_TAXELS[mod]:
                raise ValueError("module %s: %d taxels in its map" % (mod, len(P)))
            self.modules[mod] = {"type": v["type"], "link": v["link"],
                                 "origin_link_m": np.array(v["origin_link_m"], np.float64),
                                 "axes_link": np.array(v["axes_link"], np.float64),
                                 "centre_mm": np.array(v["centre_mm"], np.float64),
                                 "centre_normal": np.array(v["centre_normal"], np.float64),
                                 "cols": (col, col + len(P))}
            col += len(P)
        self.width = col

    def P(self, mod):
        return self.types[self.modules[mod]["type"]]["P"]

    def outline(self, mod):
        return self.types[self.modules[mod]["type"]]["outline"]


# ------------------------------------------------------------------------------------------------- the force model
def kernel(d2_mm2, sigma_mm):
    """The two-dimensional normal density (1 / mm^2) at squared distances d2."""
    s2 = float(sigma_mm) ** 2
    return np.exp(-np.asarray(d2_mm2) / (2.0 * s2)) / (2.0 * math.pi * s2)


def kernel_area_mm2(maps, mod, sigma_mm, area_rule=None):
    """The module's kernel area A: 1 / (the kernel summed over the taxels at the pad's centre), or the pitch squared
    of the module's type (area_rule "pitch_squared")."""
    kr = _TAC["kernel"]
    rule = kr["area_rule"] if area_rule is None else area_rule
    if rule == "pitch_squared":
        return float(kr["pitch_squared_mm2"][maps.modules[mod]["type"]])
    if rule != "pad_centre_sum_one":
        raise ValueError("unknown kernel area rule %r" % (rule,))
    P = maps.P(mod)
    c = maps.modules[mod]["centre_mm"]
    return 1.0 / float(kernel(((P - c) ** 2).sum(1), sigma_mm).sum())


def outline_distance_mm(P, outline, c):
    """Distance (mm) of a point c (module frame) to the closed polyline through the outline's taxels, 0 when c's x, y
    lies outside the outline's polygon."""
    poly = P[outline]
    x, y = float(c[0]), float(c[1])
    inside = False
    n = len(poly)
    for k in range(n):
        x1, y1 = poly[k, 0], poly[k, 1]
        x2, y2 = poly[(k + 1) % n, 0], poly[(k + 1) % n, 1]
        if (y1 > y) != (y2 > y) and x < x1 + (y - y1) * (x2 - x1) / (y2 - y1):
            inside = not inside
    if not inside:
        return 0.0
    A = poly
    B = np.roll(poly, -1, axis=0)
    AB = B - A
    t = np.clip(((c - A) * AB).sum(1) / (AB * AB).sum(1), 0.0, 1.0)
    return float(np.min(np.linalg.norm(A + t[:, None] * AB - c, axis=1)))


def edge_factor(P, outline, centroid_mm):
    ed = _TAC["edge"]
    d = outline_distance_mm(P, outline, centroid_mm)
    return float(ed["floor"]) + (1.0 - float(ed["floor"])) * min(1.0, d / float(ed["ramp_mm"]))


def module_forces(P, outline, contacts, gain, sigma_mm, area_mm2):
    """The module's force values before encoding.

    contacts: (pos (k, 3) mm, normal (k, 3) unit from the pad into the object, fn (k,) N, f_pad (k, 3) N the force the
    contact applies to the pad), all in the module frame. Returns (taxels (n, 3) N as Fx, Fy, Fz; resultant (3,) N;
    the edge factor)."""
    n = len(P)
    pos, nrm, fn, fpad = contacts
    if len(fn) == 0 or float(np.sum(fn)) <= 0.0:
        return np.zeros((n, 3)), np.zeros(3), 1.0
    cen = (fn[:, None] * pos).sum(0) / float(np.sum(fn))
    e = edge_factor(P, outline, cen)
    s = float(gain) * e
    d2 = ((P[:, None, :] - pos[None, :, :]) ** 2).sum(2)               # (n, k)
    w = area_mm2 * kernel(d2, sigma_mm)
    shear = fpad + fn[:, None] * nrm                                     # the tangential part of the force on the pad
    tax = np.empty((n, 3))
    tax[:, 0] = s * (w @ shear[:, 0])
    tax[:, 1] = s * (w @ shear[:, 1])
    tax[:, 2] = s * (w @ fn)
    tot = fpad.sum(0)
    res = s * np.array([tot[0], tot[1], -tot[2]])
    return tax, res, e


def round_half_up(x):
    return np.floor(np.asarray(x, np.float64) + 0.5).astype(np.int64)


def raw_counts(tax_n, res_n, baseline_steps):
    """The module's own integer values before the zero: Fz steps with the baseline, Fx / Fy steps, resultant LSB."""
    rz = round_half_up(tax_n[:, 2] / STEP_N + baseline_steps)
    rxy = round_half_up(tax_n[:, :2] / STEP_N)
    rr = round_half_up(np.asarray(res_n) / LSB_N)
    return rz, rxy, rr


def encode(rz, rxy, rr, zero):
    """A module's bytes from its raw values and its zero (zz, zxy, zr)."""
    zz, zxy, zr = zero
    vz = rz - zz
    fz = np.where(vz > FZ_TOP, FZ_TOP, np.mod(vz, BYTE)).astype(np.uint8)
    fxy = np.clip(rxy - zxy, XY_LO, XY_HI).astype(np.int8)
    r = np.clip(rr - zr, I16[0], I16[1])
    body = np.empty((len(rz), 3), np.uint8)
    body[:, :2] = fxy.view(np.uint8)
    body[:, 2] = fz
    return struct.pack("<hhh", int(r[0]), int(r[1]), int(r[2])) + body.tobytes()


def decode(b, n):
    """(resultant (3,) LSB, Fx (n,), Fy (n,), Fz (n,) as read) from a module's bytes (for tests and checks)."""
    r = struct.unpack("<hhh", b[:6])
    a = np.frombuffer(b[6:6 + 3 * n], np.uint8).reshape(n, 3)
    return np.array(r), a[:, 0].view(np.int8).astype(int), a[:, 1].view(np.int8).astype(int), a[:, 2].astype(int)


def active_count(fz_reads, above_n=None):
    """Taxels whose Fz reading is above the active threshold (0 wraps not unwrapped)."""
    thr = _TAC["kernel"]["rule"]["active_above_n"] if above_n is None else above_n
    return int(np.sum(np.asarray(fz_reads) * STEP_N > thr + 1e-9))


# ------------------------------------------------------------------------------------------------- the kernel rule
def centre_press(maps, mod, fn):
    """One contact pressing straight into the pad at its centre with normal force fn (module frame)."""
    v = maps.modules[mod]
    nrm = v["centre_normal"] / np.linalg.norm(v["centre_normal"])
    return (v["centre_mm"][None, :].copy(), nrm[None, :], np.array([float(fn)]), (-float(fn) * nrm)[None, :])


def active_at(maps, mod, contacts, gain, sigma_mm, area_rule=None):
    """Active taxels of one module for given contacts on a zero baseline."""
    P = maps.P(mod)
    A = kernel_area_mm2(maps, mod, sigma_mm, area_rule)
    tax, res, _e = module_forces(P, maps.outline(mod), contacts, gain, sigma_mm, A)
    rz, rxy, rr = raw_counts(tax, res, np.zeros(len(P)))
    _r, _x, _y, fz = decode(encode(rz, rxy, rr, (0, 0, 0)), len(P))
    return active_count(fz)


def sigma_by_rule(maps, rule=None, area_rule=None):
    """The smallest kernel width on the rule's grid that gives every module the rule's count of active taxels for the
    rule's normal force at its pad centre with the rule's gain; None if no width on the grid does."""
    r = _TAC["kernel"]["rule"] if rule is None else rule
    lo, hi, step = r["sigma_grid_mm"]
    k = 0
    while True:
        s = round(float(lo) + k * float(step), 9)
        if s > float(hi) + 1e-9:
            return None
        counts = [active_at(maps, mod, centre_press(maps, mod, r["test_normal_n"]), r["test_gain"], s, area_rule)
                  for mod in N.MODULES]
        if all(r["active_count"][0] <= c <= r["active_count"][1] for c in counts):
            return s
        k += 1


# ------------------------------------------------------------------------------------------------- the stream
class Stream:
    """The tactile state of one unit: zeros, baselines and their drift, and the frames captured so far."""

    def __init__(self, unit, maps, dropouts=()):
        self.maps = maps
        self.dropouts = tuple((float(a), float(b)) for a, b in dropouts)     # [t0, t1) ms of capture time
        self.f_hz = float(unit["frame_hz"])
        self.period_ms = 1000.0 / self.f_hz
        self.phase_ms = float(unit["frame_phase_ms"])
        self.gain = {mod: float(unit["tactile"][mod]["gain"]) for mod in N.MODULES}
        res = unit.get("residuals") or {}
        self.residual = np.zeros(maps.width)
        for mod in N.MODULES:
            a, b = maps.modules[mod]["cols"]
            if mod in res:
                self.residual[a:b] = np.asarray(res[mod], np.float64)
        self.sigma_mm = float(_TAC["kernel"]["sigma_mm"])
        self.area = {mod: kernel_area_mm2(maps, mod, self.sigma_mm) for mod in N.MODULES}
        dr = _TAC["drift"]
        self.drift_sd = float(dr["steps_per_sqrt_s"]) * math.sqrt(1.0 / self.f_hz)
        self.bound = float(dr["bound_steps"])
        self.noise = noise.NormalStream(int(unit["seeds"]["drift"]), maps.width)
        self.walk = np.zeros(maps.width)
        self.zero = {mod: (np.zeros(N.MODULE_TAXELS[mod], np.int64), np.zeros((N.MODULE_TAXELS[mod], 2), np.int64),
                           np.zeros(3, np.int64)) for mod in N.MODULES}
        self.seq = 0
        self.frames = collections.deque(maxlen=KEEP_FRAMES)
        self.pending_zero = []

    def next_capture_ms(self):
        return self.phase_ms + self.seq * self.period_ms

    def _raw(self, mod, contacts):
        P = self.maps.P(mod)
        a, b = self.maps.modules[mod]["cols"]
        tax, res, _e = module_forces(P, self.maps.outline(mod), contacts, self.gain[mod], self.sigma_mm,
                                     self.area[mod])
        return raw_counts(tax, res, self.residual[a:b] + self.walk[a:b])

    def request_zero(self, modules):
        for mod in modules:
            if mod not in N.MODULES:
                raise ValueError("not a tactile module: %r" % (mod,))
            if mod not in self.pending_zero:
                self.pending_zero.append(mod)

    def apply_zero(self, contacts):
        """Each requested module's zero := its raw values now (contacts: {module: contact arrays})."""
        for mod in self.pending_zero:
            rz, rxy, rr = self._raw(mod, contacts[mod])
            self.zero[mod] = (rz, rxy, rr)
        self.pending_zero = []

    def capture(self, contacts):
        """Capture the next frame (its drift step first); returns (seq, t_capture_ms, t_arrival_ms, {module: bytes})."""
        z = self.noise.row(self.seq)
        w = self.walk + self.drift_sd * z
        w = np.where(w > self.bound, 2.0 * self.bound - w, w)
        self.walk = np.where(w < -self.bound, -2.0 * self.bound - w, w)
        out = {}
        for mod in N.MODULES:
            rz, rxy, rr = self._raw(mod, contacts[mod])
            out[mod] = encode(rz, rxy, rr, self.zero[mod])
        tc = self.next_capture_ms()
        fr = (self.seq, tc, tc + ARRIVAL_MS, out)
        self.frames.append(fr)
        self.seq += 1
        return fr

    def newest(self, after_seq, t_ms):
        """The newest frame that has arrived by t_ms with seq > after_seq, or None."""
        best = None
        for fr in self.frames:
            if fr[0] > after_seq and fr[2] <= t_ms + 1e-9 and not self.dropped(fr[1]):
                best = fr
        return best

    def dropped(self, t_capture_ms):
        return any(a <= t_capture_ms < b for a, b in self.dropouts)


NO_CONTACT = (np.zeros((0, 3)), np.zeros((0, 3)), np.zeros(0), np.zeros((0, 3)))

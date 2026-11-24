"""A recorded procedure: what the surgeon's two instruments, the assistant's instrument and the scene do, step by step.

HIDDEN: this file never reaches the agent's account (the grader stages it root-only; the commissioning service's copy
is readable by the service's account alone). The agent's documents describe the kinds of things that vary, never the
generator.

`generate(seed, factors)` builds one procedure from procedural primitives (no surgeon recordings are used or shipped;
the question of a redistributable recorded dataset is an open item in tests/README.md):

* a path of working sites across the cavity floor (the working area travels: a camera that stays where it is loses
  the instruments), by procedure kind;
* per site a travel of both hands and a dwell made of gestures (hold, dissect, suture bite, pull, stretch, clip);
* instrument exchanges: one hand withdraws through its trocar, stays out, and comes back, possibly as another
  instrument; the hand is not `active` from the moment it starts to withdraw until it is back at work;
* the assistant's third instrument, never active: it holds near the work, lags behind when the work moves, and
  sometimes dabs into the middle of the view;
* smoke bursts (energy devices), the tissue's breathing, the scope (0 or 30 degrees), every look.

Everything is drawn from `Rng`, a keyed BLAKE2b counter stream: the same seed gives the same procedure on every
machine and numpy version (the arithmetic is plain float64 numpy). Factors named in `factors` are pinned instead of
drawn; the fixed hidden set pins the kind, the scope and the three events per procedure for a balanced set
(tests/hidden/procedures.json).
"""

import hashlib
import math
import struct

import numpy as np

from . import model
from .tissue import WORK_X, WORK_Y, Tissue, trocar_point

KINDS = ("dissection", "suturing", "exploration", "clip_and_cut")
HANDS = ("left", "right")
ROLES = ("left", "right", "assistant")
JAWS = ("grasper", "scissors", "hook", "needle_driver", "clip_applier", "suction")
GENERATOR_VERSION = "endosim-procedure/0.1"


class Rng(object):
    """A deterministic stream keyed by (seed, label): uniform floats from BLAKE2b in counter mode."""

    def __init__(self, seed, label):
        self._key = hashlib.blake2b("{}:{}".format(int(seed), label).encode("ascii"), digest_size=32).digest()
        self._n = 0
        self._buf = []

    def random(self):
        if not self._buf:
            self._n += 1
            d = hashlib.blake2b(struct.pack("<Q", self._n), key=self._key, digest_size=64).digest()
            self._buf = [(v >> 11) * (1.0 / (1 << 53)) for v in struct.unpack("<8Q", d)]
        return self._buf.pop()

    def uniform(self, lo, hi):
        return lo + (hi - lo) * self.random()

    def integers(self, lo, hi):
        """An integer in [lo, hi)."""
        return lo + min(int(self.random() * (hi - lo)), hi - lo - 1)

    def choice(self, items):
        return items[self.integers(0, len(items))]

    def chance(self, p):
        return self.random() < p

    def normal(self, n=None):
        if n is None:
            return math.sqrt(-2.0 * math.log(1.0 - self.random())) * math.cos(2.0 * math.pi * self.random())
        return np.array([self.normal() for _ in range(int(n))])

    def unit2(self):
        a = self.uniform(0.0, 2.0 * math.pi)
        return np.array([math.cos(a), math.sin(a), 0.0])


# ---------------------------------------------------------------------------------------------------------- looks
_SHAFTS = (  # (name, rgb, metallic, roughness)
    ("black", (0.025, 0.025, 0.03), 0.0, 0.45), ("graphite", (0.12, 0.12, 0.13), 0.3, 0.4),
    ("steel", (0.72, 0.72, 0.74), 1.0, 0.28), ("satin", (0.55, 0.56, 0.58), 1.0, 0.45),
    ("white", (0.82, 0.82, 0.80), 0.0, 0.35), ("blue", (0.05, 0.16, 0.42), 0.2, 0.35),
    ("green", (0.06, 0.30, 0.16), 0.1, 0.4), ("gold", (0.65, 0.48, 0.18), 1.0, 0.3),
)
_BANDS = ((0.85, 0.75, 0.10), (0.10, 0.55, 0.80), (0.85, 0.85, 0.85), (0.80, 0.25, 0.10), (0.20, 0.70, 0.25))


def draw_instrument_look(rng, jaw=None):
    name, rgb, metallic, rough = rng.choice(_SHAFTS)
    jaw = jaw or rng.choice(JAWS[:5])
    steel = rng.uniform(0.55, 0.8)
    look = {
        "jaw": jaw, "shaft": name,
        "shaft_rgb": [float(c) for c in rgb], "shaft_metallic": metallic, "shaft_rough": rough + rng.uniform(-0.08, 0.08),
        "wrist_len": rng.uniform(0.008, 0.018), "wrist_rgb": [steel, steel, steel + 0.02], "wrist_rough": rng.uniform(0.2, 0.45),
        "jaw_len": {"grasper": rng.uniform(0.014, 0.019), "scissors": rng.uniform(0.012, 0.016), "hook": rng.uniform(0.010, 0.014),
                    "needle_driver": rng.uniform(0.009, 0.012), "clip_applier": rng.uniform(0.011, 0.014),
                    "suction": rng.uniform(0.012, 0.02)}[jaw],
        "jaw_rgb": [steel * rng.uniform(0.8, 1.0)] * 3, "jaw_rough": rng.uniform(0.18, 0.4),
        "band_rgb": [float(c) for c in rng.choice(_BANDS)] if rng.chance(0.45) else None,
        "band_at": rng.uniform(0.024, 0.045), "band_len": rng.uniform(0.003, 0.008),
    }
    return look


_PALETTES = (  # (name, base, dark, pale, fat, vessel)
    ("pink", (0.74, 0.36, 0.34), (0.50, 0.16, 0.15), (0.86, 0.60, 0.55), (0.86, 0.72, 0.36), (0.42, 0.06, 0.08)),
    ("red", (0.66, 0.22, 0.20), (0.40, 0.09, 0.09), (0.80, 0.45, 0.40), (0.84, 0.68, 0.30), (0.33, 0.03, 0.06)),
    ("liver", (0.50, 0.20, 0.16), (0.30, 0.09, 0.08), (0.68, 0.38, 0.30), (0.80, 0.66, 0.34), (0.26, 0.04, 0.05)),
    ("pale", (0.80, 0.52, 0.46), (0.58, 0.28, 0.25), (0.90, 0.72, 0.66), (0.90, 0.80, 0.48), (0.50, 0.12, 0.14)),
    ("fatty", (0.80, 0.60, 0.36), (0.62, 0.36, 0.22), (0.88, 0.74, 0.52), (0.92, 0.82, 0.42), (0.52, 0.10, 0.10)),
)


def draw_scene_look(rng):
    name, base, dark, pale, fat, vessel = rng.choice(_PALETTES)
    return {
        "palette": name, "base": list(base), "dark": list(dark), "pale": list(pale), "fat": list(fat), "vessel": list(vessel),
        "texture_seed": rng.integers(0, 1 << 30), "fat_cover": rng.uniform(0.05, 0.45), "vessels": rng.integers(18, 60),
        "blotch_scale": rng.uniform(0.6, 1.6), "roughness": rng.uniform(0.16, 0.42), "clearcoat": rng.uniform(0.2, 1.0),
        "bump": rng.uniform(0.3, 1.0), "wall_shift": rng.uniform(-0.06, 0.06),
        "light": rng.uniform(0.75, 1.35), "white_balance": [rng.uniform(0.94, 1.06), 1.0, rng.uniform(0.92, 1.06)],
        "noise_sigma": rng.uniform(1.0, 3.0), "smoke_tint": [rng.uniform(0.78, 0.9)] * 3,
    }


# ------------------------------------------------------------------------------------------------- small helpers
def _minjerk(n):
    s = np.linspace(0.0, 1.0, int(n))
    return 10 * s ** 3 - 15 * s ** 4 + 6 * s ** 5


def _tremor(rng, n, sigma, pole=0.85):
    out = np.zeros((int(n), 3))
    w = rng.normal(3 * int(n)).reshape(-1, 3) * sigma * math.sqrt(1.0 - pole * pole)
    for k in range(1, int(n)):
        out[k] = pole * out[k - 1] + w[k]
    return out


def _steps(seconds):
    return max(1, int(round(seconds / model.DT)))


class _Site(object):
    """A working site: where on the tissue, how the two hands stand at it."""

    def __init__(self, rng, tissue, xy):
        self.c = np.array([xy[0], xy[1], float(tissue.height(xy[0], xy[1]))])
        ang = rng.uniform(-0.45, 0.45)
        self.lat = np.array([math.cos(ang), math.sin(ang), 0.0])       # from the left hand to the right hand
        self.fwd = np.array([-math.sin(ang), math.cos(ang), 0.0])
        self.up = np.array([0.0, 0.0, 1.0])
        self.sep = rng.uniform(0.020, 0.042)
        self.station = {}
        for hand, sign in (("left", -1.0), ("right", 1.0)):
            p = self.c + sign * 0.5 * self.sep * self.lat + rng.uniform(-0.004, 0.004) * self.fwd
            p[2] = float(tissue.height(p[0], p[1])) + rng.uniform(0.007, 0.014)
            self.station[hand] = p


def _site_path(rng, kind, count):
    """Site centres (x, y) on the working floor; consecutive sites far enough apart that the view has to follow."""
    step = {"dissection": (0.045, 0.085), "suturing": (0.030, 0.050), "exploration": (0.060, 0.110),
            "clip_and_cut": (0.050, 0.100)}[kind]
    xy = np.array([rng.uniform(WORK_X[0] + 0.01, WORK_X[1] - 0.01), rng.uniform(WORK_Y[0] + 0.01, WORK_Y[1] - 0.01)])
    heading = rng.uniform(0.0, 2.0 * math.pi)
    out = [xy]
    jump_at = rng.integers(2, max(3, count - 1)) if kind == "suturing" else -1
    for k in range(1, count):
        lo, hi = (0.085, 0.130) if k == jump_at else step
        for _ in range(60):
            turn = rng.uniform(0.0, 2.0 * math.pi) if k == jump_at else rng.normal() * (0.25 if kind == "suturing" else 0.9)
            d = rng.uniform(lo, hi)
            cand = out[-1] + d * np.array([math.cos(heading + turn), math.sin(heading + turn)])
            if WORK_X[0] <= cand[0] <= WORK_X[1] and WORK_Y[0] <= cand[1] <= WORK_Y[1]:
                heading += turn
                break
            heading = rng.uniform(0.0, 2.0 * math.pi)       # blocked by the edge of the working floor: turn
        else:
            cand = np.array([rng.uniform(*WORK_X), rng.uniform(*WORK_Y)])
        out.append(cand)
    return out


# ------------------------------------------------------------------------------------------------------ the builder
class _Builder(object):
    def __init__(self, rng, tissue, trocars, total):
        self.rng, self.tissue, self.trocars, self.total = rng, tissue, trocars, int(total)
        self.tip = {h: [] for h in HANDS}
        self.jaw = {h: [] for h in HANDS}
        self.active = {h: [] for h in HANDS}
        self.present = {h: [] for h in HANDS}
        self.look = {h: [] for h in HANDS}
        self.cur_look = {"left": 0, "right": 1}
        self.smoke_events = []        # (step, peak, decay_s)
        self.site_of_step = []
        self.gestures = []            # (start step, name) for the maintainers' record
        self.n = 0

    def done(self):
        return self.n >= self.total

    def _put(self, name, pos, jaw=None, active=None, present=None, site=-1):
        """Append one segment: pos {hand: (n, 3)}; jaw {hand: (n,)}; active / present {hand: (n,) bool}."""
        n = len(pos["left"])
        self.gestures.append((self.n, name))
        for h in HANDS:
            self.tip[h].append(np.asarray(pos[h], dtype=float))
            self.jaw[h].append(np.full(n, 0.08) if jaw is None or h not in jaw else np.asarray(jaw[h], dtype=float))
            self.active[h].append(np.ones(n, bool) if active is None or h not in active else np.asarray(active[h], bool))
            self.present[h].append(np.ones(n, bool) if present is None or h not in present else np.asarray(present[h], bool))
            self.look[h].append(np.full(n, self.cur_look[h], dtype=int))
        self.site_of_step += [site] * n
        self.n += n

    def last(self, hand):
        return self.tip[hand][-1][-1]

    # -- gestures -------------------------------------------------------------------------------------------------
    def start(self, site, k):
        n = _steps(self.rng.uniform(1.5, 2.5))
        self._put("hold", {h: site.station[h] + _tremor(self.rng, n, 0.0003) for h in HANDS}, site=k)

    def travel(self, site, k):
        rng = self.rng
        dist = float(np.linalg.norm(site.c - 0.5 * (self.last("left") + self.last("right"))))
        n = _steps(max(1.4, dist / rng.uniform(0.030, 0.042)))
        lead = _steps(rng.uniform(0.0, 0.3))                # the right hand leads
        lift = rng.uniform(0.010, 0.022)
        pos = {}
        for h in HANDS:
            a, b = self.last(h), site.station[h]
            s = _minjerk(n)
            if h == "left" and lead:
                s = np.concatenate([np.zeros(lead), _minjerk(n - lead)])
            p = a[None, :] + s[:, None] * (b - a)[None, :]
            p[:, 2] += lift * np.sin(math.pi * s) ** 2
            floor = self.tissue.height(p[:, 0], p[:, 1]) + 0.006
            p[:, 2] = np.maximum(p[:, 2], floor)
            pos[h] = p
        jaw = {h: np.full(n, rng.uniform(0.05, 0.3)) for h in HANDS}
        self._put("travel", pos, jaw=jaw, site=k)

    def hold(self, site, k, seconds):
        n = _steps(seconds)
        self._put("hold", {h: self.last(h) + _tremor(self.rng, n, 0.0003) for h in HANDS}, site=k)

    def dissect(self, site, k, seconds, smoke):
        rng = self.rng
        n = _steps(seconds)
        t = np.arange(n) * model.DT
        amp, freq = rng.uniform(0.003, 0.0055), rng.uniform(0.7, 1.3)
        a0, da = rng.uniform(0.0, 2.0 * math.pi), rng.uniform(-0.25, 0.25)
        ang = a0 + da * t
        d = np.cos(ang)[:, None] * site.lat[None, :] + np.sin(ang)[:, None] * site.fwd[None, :]
        phase = np.sin(2.0 * math.pi * freq * t + rng.uniform(0.0, 6.28))
        ramp = np.minimum(1.0, np.minimum(t, t[-1] - t) / 0.4)
        right = self.last("right")[None, :] + (amp * phase * ramp)[:, None] * d
        right[:, 2] += 0.5 * amp * np.abs(phase) * ramp
        left = self.last("left")[None, :] + _tremor(rng, n, 0.0003)
        if n > 30 and rng.chance(0.6):                       # a regrasp: a short shift and back
            at, m = rng.integers(8, n - 16), _steps(0.6)
            shift = rng.uniform(0.004, 0.007) * (rng.uniform(-1, 1) * site.lat + rng.uniform(-1, 1) * site.fwd)
            prof = np.zeros(n)
            prof[at:at + m] = _minjerk(m)
            prof[at + m:] = 1.0
            hold_until = min(n, at + m + _steps(rng.uniform(0.6, 1.5)))
            back = min(n - hold_until, m)
            if back > 1:
                prof[hold_until:hold_until + back] = 1.0 - _minjerk(back)
                prof[hold_until + back:] = 0.0
            left = left + prof[:, None] * shift[None, :]
        jaw = {"right": 0.18 + 0.16 * phase * ramp, "left": np.full(n, 0.03)}
        if smoke and rng.chance(0.75):
            for _ in range(rng.integers(1, 3)):
                self.smoke_events.append((self.n + rng.integers(2, max(3, n - 5)), rng.uniform(0.35, 0.95), rng.uniform(3.0, 6.0)))
        self._put("dissect", {"left": left, "right": right}, jaw=jaw, site=k)

    def suture(self, site, k, bites):
        rng = self.rng
        for _ in range(bites):
            n, rest = _steps(rng.uniform(1.6, 2.4)), _steps(rng.uniform(0.4, 0.8))
            r = rng.uniform(0.0045, 0.0065)
            th = 2.0 * math.pi * _minjerk(n)
            e = math.cos(rng.uniform(-0.6, 0.6)) * site.lat + math.sin(rng.uniform(-0.6, 0.6)) * site.fwd
            loop = r * (np.sin(th)[:, None] * e[None, :] + (1.0 - np.cos(th))[:, None] * site.up[None, :])
            loop = np.concatenate([loop, np.zeros((rest, 3))])
            m = n + rest
            right = self.last("right")[None, :] + loop + _tremor(rng, m, 0.0002)
            left = self.last("left")[None, :] + 0.003 * np.sin(np.linspace(0.0, math.pi, m))[:, None] * site.lat[None, :] \
                + _tremor(rng, m, 0.0003)
            jaw = {"right": np.full(m, 0.04), "left": np.full(m, 0.10)}
            self._put("suture", {"left": left, "right": right}, jaw=jaw, site=k)

    def _excursion(self, name, hand, site, k, vec, out_s, hold_s, jaw_hold=0.03):
        """One hand goes out by `vec`, holds, comes back; the other hand holds."""
        rng = self.rng
        a, h, b = _steps(out_s), _steps(hold_s), _steps(out_s)
        prof = np.concatenate([_minjerk(a), np.ones(h), 1.0 - _minjerk(b)])
        n = len(prof)
        other = "left" if hand == "right" else "right"
        pos = {hand: self.last(hand)[None, :] + prof[:, None] * vec[None, :] + _tremor(rng, n, 0.0002),
               other: self.last(other)[None, :] + _tremor(rng, n, 0.0003)}
        jaw = {hand: np.full(n, jaw_hold), other: np.full(n, 0.05)}
        self._put(name, pos, jaw=jaw, site=k)

    def pull(self, site, k):
        rng = self.rng
        lat = rng.uniform(0.008, max(0.010, 0.062 - site.sep))
        vec = lat * site.lat + rng.uniform(0.015, 0.030) * site.up + rng.uniform(-0.010, 0.010) * site.fwd
        self._excursion("pull", "right", site, k, vec, rng.uniform(1.0, 1.4), rng.uniform(0.4, 0.8))

    def stretch(self, site, k):
        rng = self.rng
        lat = rng.uniform(0.010, max(0.012, 0.062 - site.sep))
        vec = -lat * site.lat + rng.uniform(0.008, 0.020) * site.up + rng.uniform(-0.008, 0.008) * site.fwd
        self._excursion("stretch", "left", site, k, vec, rng.uniform(0.9, 1.3), rng.uniform(2.0, 4.0))

    def clip(self, site, k):
        rng = self.rng
        vec = rng.uniform(0.007, 0.011) * site.fwd - rng.uniform(0.0, 0.003) * site.up
        self._excursion("clip", "right", site, k, vec, rng.uniform(0.7, 0.9), rng.uniform(1.2, 2.0), jaw_hold=0.0)

    def exchange(self, hand, site, k, new_look):
        """`hand` withdraws through its trocar, stays out, returns (as look `new_look`) to its station."""
        rng = self.rng
        other = "left" if hand == "right" else "right"
        a = self.last(hand)
        out_pt = self.trocars[hand] + 0.012 * (self.trocars[hand] - a) / np.linalg.norm(self.trocars[hand] - a)
        dist = float(np.linalg.norm(out_pt - a))
        n_out, n_away, n_in = _steps(dist / rng.uniform(0.085, 0.115)), _steps(rng.uniform(6.0, 10.0)), _steps(dist / rng.uniform(0.07, 0.10))
        back = site.station[hand]
        # out
        s = _minjerk(n_out)
        pos = {hand: a[None, :] + s[:, None] * (out_pt - a)[None, :], other: self.last(other)[None, :] + _tremor(rng, n_out, 0.0003)}
        self._put("exchange_out", pos, active={hand: np.zeros(n_out, bool)}, site=k)
        # away
        pos = {hand: np.repeat(out_pt[None, :], n_away, 0), other: self.last(other)[None, :] + _tremor(rng, n_away, 0.0003)}
        self._put("exchange_away", pos, active={hand: np.zeros(n_away, bool)}, present={hand: np.zeros(n_away, bool)}, site=k)
        self.cur_look[hand] = new_look
        # back in: active again once it is within 10 mm of its station
        s = _minjerk(n_in)
        p = out_pt[None, :] + s[:, None] * (back - out_pt)[None, :]
        act = np.linalg.norm(p - back[None, :], axis=1) <= 0.010
        pos = {hand: p, other: self.last(other)[None, :] + _tremor(rng, n_in, 0.0003)}
        self._put("exchange_in", pos, active={hand: act}, jaw={hand: np.full(n_in, 0.25)}, site=k)

    def arrays(self):
        cut = self.total
        cat = lambda d: {h: np.concatenate(d[h])[:cut] for h in HANDS}   # noqa: E731
        return cat(self.tip), cat(self.jaw), cat(self.active), cat(self.present), cat(self.look), np.array(self.site_of_step[:cut])


def _assistant(rng, tissue, trocar, sites, site_of_step, hands_tip, total, wanted):
    """The assistant's instrument: tip positions, presence. It holds beside the work, follows a move late or not at
    all, and now and then dabs towards the middle of the work and back."""
    tip = np.repeat((trocar + np.array([0.0, 0.0, 0.012]))[None, :], total, 0)
    present = np.zeros(total, bool)
    if not wanted:
        return tip, present, np.full(total, 0.0)
    side = 1.0 if trocar[0] > 0 else -1.0
    enter = _steps(rng.uniform(0.0, 12.0)) if rng.chance(0.5) else 0
    leave = total - _steps(rng.uniform(4.0, 15.0)) if rng.chance(0.35) else total + 10

    def hold_point(k):
        s = sites[k]
        p = s.c + side * rng.uniform(0.030, 0.055) * s.lat + rng.uniform(-0.01, 0.035) * s.fwd
        p[2] = float(tissue.height(p[0], p[1])) + rng.uniform(0.012, 0.028)
        return p

    out_pt = trocar + np.array([0.0, 0.0, 0.012])
    pos, target = out_pt.copy(), None
    cur_site, follow, dab = None, None, None          # follow = (step, site) of a pending move to the new site
    speed = 0.06
    for t in range(total):
        k = int(site_of_step[t])
        if t < enter or t >= leave:
            goal = out_pt
        else:
            if cur_site is None:
                target, cur_site = hold_point(k), k
            elif k != cur_site:
                # the work moved: follow after a delay, or stay behind for this site
                cur_site = k
                follow = (t + _steps(rng.uniform(1.5, 5.0)), k) if rng.chance(0.7) else None
            if follow is not None and t >= follow[0]:
                target, follow = hold_point(follow[1]), None
            if dab is None and rng.chance(0.004):
                mid = 0.5 * (hands_tip["left"][t] + hands_tip["right"][t])
                dab = (t + _steps(rng.uniform(1.5, 3.0)), mid + np.array([0.0, 0.0, rng.uniform(0.004, 0.010)]))
            if dab is not None and t >= dab[0]:
                dab = None
            goal = dab[1] if dab is not None else target
        d = goal - pos
        dist = float(np.linalg.norm(d))
        if dist > 1e-9:
            pos = pos + d * min(1.0, speed * model.DT / dist)
        tip[t] = pos
        present[t] = float(np.linalg.norm(pos - out_pt)) > 0.02
    tip += _tremor(rng, total, 0.00035) * present[:, None]
    return tip, present, np.full(total, 0.05)


def _fine_flags(tips, active):
    """Fine work at step t: over the last FINE_WINDOW_S every active tip stayed within FINE_RADIUS of its own mean
    and the set of active tips did not change. Observable from the instruments' motion alone."""
    total = len(active["left"])
    w = _steps(model.FINE_WINDOW_S)
    fine = np.zeros(total, bool)
    for t in range(w, total):
        ok = True
        for h in HANDS:
            a = active[h][t - w:t + 1]
            if a.any() != a.all():
                ok = False
                break
            if not a.all():
                continue
            seg = tips[h][t - w:t + 1]
            if np.linalg.norm(seg - seg.mean(0), axis=1).max() > model.FINE_RADIUS:
                ok = False
                break
        fine[t] = ok
    return fine


class Procedure(object):
    """One generated procedure: arrays over steps 0 .. steps (inclusive), the scene's look, the facts."""

    def __init__(self, **kw):
        self.__dict__.update(kw)

    def roles_state(self, t):
        """The instruments at step t for the renderer: tip, the trocar it pivots in, jaw, look, presence."""
        out = {}
        for r in ROLES:
            out[r] = {"tip": [float(v) for v in self.tip[r][t]], "jaw": float(self.jaw[r][t]), "look": int(self.look[r][t]),
                      "present": bool(self.present[r][t])}
        return out

    def scene_spec(self):
        """What the renderer builds once per procedure (JSON-serialisable; no trajectory in it)."""
        return {"tissue": self.tissue.params, "look": self.scene_look, "instrument_looks": self.looks,
                "trocars": {r: [float(v) for v in self.trocars[r]] for r in ROLES}, "scope": self.scope}

    def facts(self):
        return {"seed": int(self.seed), "kind": self.kind, "scope": self.scope, "steps": int(self.steps),
                "exchanges": int(self.exchanges), "assistant": bool(self.assistant), "smoke": bool(self.smoke_on),
                "sites": len(self.sites_xy), "path_m": round(float(self.path_m), 3),
                "fine_fraction": round(float(self.fine[1:].mean()), 3),
                "both_active_fraction": round(float((self.active["left"][1:] & self.active["right"][1:]).mean()), 3),
                "generator": GENERATOR_VERSION}


def generate(seed, factors=None, steps=800):
    """The procedure of `seed`. `factors` may pin: kind, scope, exchanges (0-2), assistant (bool), smoke (bool)."""
    f = dict(factors or {})
    rng = Rng(seed, "procedure")
    kind = f.get("kind") or rng.choice(KINDS)
    scope = str(f.get("scope") or rng.choice(("0", "30")))
    default_x = {"dissection": rng.integers(0, 2), "suturing": rng.integers(0, 2), "exploration": 0, "clip_and_cut": 2}[kind]
    exchanges = int(f["exchanges"]) if "exchanges" in f else int(default_x)
    assistant = bool(f["assistant"]) if "assistant" in f else rng.chance(0.6)
    smoke_on = bool(f["smoke"]) if "smoke" in f else (kind in ("dissection", "clip_and_cut") and rng.chance(0.7))
    total = int(steps) + 1

    tissue = Tissue.draw(Rng(seed, "tissue"))
    trng = Rng(seed, "ports")
    jit = lambda: trng.uniform(-0.012, 0.012)   # noqa: E731
    a_side = trng.choice((-1.0, 1.0))
    trocars = {"left": trocar_point(-0.075 + jit(), -0.045 + jit()), "right": trocar_point(0.075 + jit(), -0.045 + jit()),
               "assistant": trocar_point(a_side * (0.125 + jit()), 0.070 + jit())}

    lrng = Rng(seed, "looks")
    work_jaws = {"dissection": ("grasper", "hook"), "suturing": ("grasper", "needle_driver"),
                 "exploration": ("grasper", "grasper"), "clip_and_cut": ("grasper", "scissors")}[kind]
    looks = [draw_instrument_look(lrng, work_jaws[0]), draw_instrument_look(lrng, work_jaws[1]),
             draw_instrument_look(lrng, lrng.choice(("suction", "suction", "grasper")))]
    scene_look = draw_scene_look(Rng(seed, "scene"))

    count = {"dissection": 7, "suturing": 8, "exploration": 12, "clip_and_cut": 8}[kind]
    prng = Rng(seed, "path")
    sites_xy = _site_path(prng, kind, count)
    srng = Rng(seed, "sites")
    sites = [_Site(srng, tissue, xy) for xy in sites_xy]

    g = Rng(seed, "gestures")
    b = _Builder(g, tissue, trocars, total)
    # which sites carry an exchange (never the first site; at most `exchanges`)
    x_sites = set()
    while len(x_sites) < min(exchanges, count - 2):
        x_sites.add(g.integers(1, count - 1))
    x_jaw = {"dissection": ("scissors", "clip_applier", "grasper"), "suturing": ("scissors", "grasper"),
             "exploration": ("grasper",), "clip_and_cut": ("clip_applier", "scissors", "hook")}[kind]
    k = 0
    b.start(sites[0], 0)
    while not b.done():
        site = sites[k % count] if k < count else _Site(srng, tissue, _site_path(prng, kind, 2)[1])
        if k >= count:
            sites.append(site)
            sites_xy.append(site.c[:2])
        if k > 0:
            b.travel(site, k)
        if kind == "dissection":
            for _ in range(g.integers(2, 4)):
                if g.chance(0.25):
                    b.stretch(site, k)
                b.dissect(site, k, g.uniform(2.5, 5.0), smoke_on)
        elif kind == "suturing":
            b.suture(site, k, g.integers(1, 3))
            b.pull(site, k)
            if g.chance(0.4):
                b.hold(site, k, g.uniform(0.8, 1.6))
        elif kind == "exploration":
            b.hold(site, k, g.uniform(1.2, 3.0))
            if g.chance(0.5):
                b.stretch(site, k)
        else:
            b.dissect(site, k, g.uniform(2.0, 3.5), smoke_on)
            b.clip(site, k)
            if g.chance(0.5):
                b.hold(site, k, g.uniform(0.8, 1.5))
        if k in x_sites:
            hand = "right" if (kind != "exploration" and g.chance(0.75)) else g.choice(HANDS)
            looks.append(draw_instrument_look(lrng, g.choice(x_jaw)))
            b.exchange(hand, site, k, len(looks) - 1)
            b.hold(site, k, g.uniform(1.0, 2.0))
        k += 1

    tip, jaw, active, present, look, site_of_step = b.arrays()
    a_tip, a_present, a_jaw = _assistant(Rng(seed, "assistant"), tissue, trocars["assistant"], sites, site_of_step, tip, total,
                                         assistant)
    tip["assistant"], jaw["assistant"] = a_tip, a_jaw
    present["assistant"], active["assistant"] = a_present, np.zeros(total, bool)
    look["assistant"] = np.full(total, 2, dtype=int)

    smoke = np.zeros(total)
    for at, peak, decay in b.smoke_events:
        t = np.arange(total) - at
        rise = np.clip(t / _steps(0.6), 0.0, 1.0)
        smoke += np.where(t >= 0, peak * rise * np.exp(-np.clip(t, 0, None) * model.DT / decay), 0.0)
    smoke = np.clip(smoke, 0.0, 1.0)

    used = sorted(set(int(s) for s in site_of_step))
    path = sum(float(np.linalg.norm(sites[b_].c - sites[a_].c)) for a_, b_ in zip(used[:-1], used[1:]))
    return Procedure(seed=int(seed), kind=kind, scope=scope, steps=int(steps), exchanges=len(x_sites), assistant=assistant,
                     smoke_on=smoke_on, tissue=tissue, trocars=trocars, looks=looks, scene_look=scene_look,
                     tip=tip, jaw=jaw, active=active, present=present, look=look, smoke=smoke,
                     fine=_fine_flags(tip, active), site_of_step=site_of_step, sites_xy=[list(map(float, s.c[:2])) for s in sites],
                     path_m=path, gestures=b.gestures)

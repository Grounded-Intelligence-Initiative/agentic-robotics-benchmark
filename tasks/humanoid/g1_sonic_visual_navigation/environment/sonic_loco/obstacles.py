"""Physical fork and cabinet layouts in the starting heading frame (x forward, y left)."""
from dataclasses import dataclass, replace
import hashlib
import math
from numbers import Real

import numpy as np

LANE_HALF_WIDTH = 1.58
PANEL_LENGTH = 2.4
PANEL_DEPTH = .18
PANEL_HEIGHT = .72
ISLAND_WIDTH = .44
PLATE_WIDTH = 1.08
PLATE_DEPTH = .09
PLATE_OFFSET_X = -.14
PERIOD_S = 12.
FULL_HOLD_S = 4.2
OPEN_S = CLOSE_S = 1.
LATERAL = (('right', -1), ('left', 1))


def _seed(seed):
    if type(seed) is not int or not 0 <= seed < 2**64:
        raise ValueError("layout and phase seeds must be unsigned 64-bit integers")
    return seed


def _draw(seed, label, lo, hi):
    bits = hashlib.blake2b(label.encode(), key=seed.to_bytes(8, 'little'), digest_size=8,
                           person=b'g1nav.fork.v1').digest()
    return lo + (hi-lo) * ((int.from_bytes(bits, 'little') >> 11) * 2.**-53)


def cycle(t):
    """Unit travel command and its derivative for a smooth periodic position servo."""
    t %= PERIOD_S
    if t < OPEN_S:
        u = math.pi*t/OPEN_S
        return .5*(1-math.cos(u)), .5*math.pi*math.sin(u)/OPEN_S
    if t < OPEN_S+FULL_HOLD_S:
        return 1., 0.
    if t < OPEN_S+FULL_HOLD_S+CLOSE_S:
        u = math.pi*(t-OPEN_S-FULL_HOLD_S)/CLOSE_S
        return .5*(1+math.cos(u)), -.5*math.pi*math.sin(u)/CLOSE_S
    return 0., 0.


@dataclass(frozen=True)
class Gate:
    forward_m: float
    left_m: float
    gap_width_m: float
    rgb: tuple

    @property
    def side(self):
        return 1 if self.left_m > 0 else -1

    @property
    def edges(self):
        return self.left_m - self.gap_width_m/2, self.left_m + self.gap_width_m/2


@dataclass(frozen=True)
class Fork:
    forward_m: float
    island_center_m: float
    right_width_m: float
    left_width_m: float
    rgb: tuple

    def corridor(self, side):
        inner = self.island_center_m + side*ISLAND_WIDTH/2
        if side == 1:
            return inner, inner+self.left_width_m
        if side == -1:
            return inner-self.right_width_m, inner
        raise ValueError("side must be +1 (left) or -1 (right)")

    def clear_bounds(self, side, slide):
        lo, hi = self.corridor(side)
        width = max(0., min(hi-lo, slide-(PLATE_WIDTH-(hi-lo))/2))
        return (lo, lo+width) if side == 1 else (hi-width, hi)


@dataclass(frozen=True)
class Layout:
    fork: Fork
    tail: Gate | None
    service: str
    phases: tuple
    short_stroke_m: float

    @property
    def gates(self):
        return (self.fork, self.tail) if self.tail else (self.fork,)

    def stroke(self, side):
        name = 'left' if side == 1 else 'right'
        if self.service not in ('both', name):
            return self.short_stroke_m
        lo, hi = self.fork.corridor(side)
        return (PLATE_WIDTH+hi-lo)/2 + .03

    def mirrored(self):
        f = self.fork
        fork = replace(f, island_center_m=-f.island_center_m, right_width_m=f.left_width_m,
                       left_width_m=f.right_width_m)
        tail = replace(self.tail, left_m=-self.tail.left_m) if self.tail else None
        service = {'left':'right','right':'left','both':'both'}[self.service]
        return replace(self, fork=fork, tail=tail, service=service, phases=self.phases[::-1])


def make_layout(seed, first_m, *, tail=None, service=None, phase_seed=None, phases=None):
    """Overrides fix only their named factor; independent geometry and schedule draws persist."""
    _seed(seed)
    phase_seed = seed if phase_seed is None else _seed(phase_seed)
    if not math.isfinite(first_m) or not 5. <= first_m <= 8.:
        raise ValueError("the obstacle approach must be between 5 and 8 metres")
    if tail not in (None, 'none', 'left', 'right') or service not in (None,'both','left','right'):
        raise ValueError("invalid tail or service setting")
    if phases is None:
        phases = tuple(_draw(phase_seed, 'phase-'+name, 0., PERIOD_S) for name,_ in LATERAL)
    if (not isinstance(phases,(tuple,list)) or len(phases)!=2 or any(isinstance(v,bool)
            or not isinstance(v,Real) or not 0<=v<PERIOD_S for v in phases)):
        raise ValueError("phases must give right and left offsets in [0,12) seconds")
    def draw(field, lo, hi):
        return _draw(seed, field, lo, hi)
    def material(prefix):
        return tuple(draw(prefix+channel, lo, hi) for channel,lo,hi in
                     [('r',.075,.115),('g',.49,.61),('b',.56,.68)])
    family = draw('service',0.,1.)
    service = service or ('both' if family<.5 else ('left' if family<.75 else 'right'))
    fork = Fork(draw('fork-x',2.20,2.55),draw('island-y',-.02,.02),
                draw('right-width',.96,1.06),draw('left-width',.96,1.06),material('fork-'))
    if tail is None:
        tail = 'none' if draw('tail-present',0.,1.)<.5 else (
            'left' if draw('tail-sign',0.,1.)<.5 else 'right')
    downstream = None
    if tail!='none':
        x = max(fork.forward_m+1.65, first_m*draw('tail-x',.66,.72))
        y = (1 if tail=='left' else -1)*draw('tail-offset',.57,.76)
        downstream = Gate(x,y,draw('tail-width',.95,1.08),material('tail-'))
    return Layout(fork,downstream,service,tuple(map(float,phases)),draw('limited-travel',.06,.12))


def heading_axes(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c,-s],[s,c]])

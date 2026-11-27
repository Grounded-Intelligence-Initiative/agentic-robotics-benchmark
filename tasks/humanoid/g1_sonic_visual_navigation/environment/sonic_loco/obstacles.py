"""Floor-anchored gate layouts in the starting pelvis heading frame (x forward, y left)."""
from dataclasses import dataclass, replace
import hashlib
import math

import numpy as np

LANE_HALF_WIDTH = 1.58
PANEL_LENGTH = 2.4
PANEL_DEPTH = .18
PANEL_HEIGHT = .72
MAX_GATES = 2
GAP_PATTERNS = ('L', 'R', 'LL', 'LR', 'RL', 'RR')


def _draw(seed, label, lo, hi):
    bits = hashlib.blake2b(label.encode(), key=seed.to_bytes(8, 'little'), digest_size=8,
                           person=b'g1nav.gates.v1').digest()
    return lo + (hi-lo) * ((int.from_bytes(bits, 'little') >> 11) * 2.**-53)


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
        return self.left_m - self.gap_width_m / 2, self.left_m + self.gap_width_m / 2


@dataclass(frozen=True)
class Layout:
    gates: tuple

    def mirrored(self):
        return Layout(tuple(replace(g, left_m=-g.left_m) for g in self.gates))


def make_layout(seed, first_m, gaps=None):
    """A public fixture override sets only gate count and signs. All other draws stay keyed."""
    if type(seed) is not int or not 0 <= seed < 2**64:
        raise ValueError("layout seed must be an unsigned 64-bit integer")
    if not math.isfinite(first_m) or not 5. <= first_m <= 8.:
        raise ValueError("the obstacle approach must be between 5 and 8 metres")
    if gaps is not None and gaps not in GAP_PATTERNS:
        raise ValueError("gaps must be one of L, R, LL, LR, RL, RR")
    count = len(gaps) if gaps else (1 if _draw(seed, "count", 0., 1.) < .25 else 2)
    gates = []
    for i in range(count):
        def draw(field, lo, hi):
            return _draw(seed, f"gate-{i}-{field}", lo, hi)
        sign = (1 if gaps[i] == 'L' else -1) if gaps else (1 if draw("sign", 0., 1.) < .5 else -1)
        if count == 1:
            x = first_m * draw("one-x", .38, .49)
        elif i == 0:
            x = draw("first-x", 1.65, 1.95)
        else:
            x = max(gates[0].forward_m + 1.60, first_m * draw("second-x", .62, .69))
        # The same material range covers every opening direction.
        color = (draw('red', .075, .115), draw('green', .49, .61), draw('blue', .56, .68))
        gates.append(Gate(x, sign*draw('offset', .57, .76), draw('width', .95, 1.08), color))
    return Layout(tuple(gates))


def heading_axes(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s], [s, c]])

"""Image-only commissioning of the enclosed crossed-belt drive."""
from dataclasses import dataclass, field

import numpy as np
from scipy.ndimage import label

from policy import decode

HOME = np.array([-1.9, -1.7, 1.])
U, Z = np.meshgrid((np.arange(256)+.5)/32-4, 6-(np.arange(256)+.5)/32)
PULSE = (1.,)*7 + (0.,)*5 + (-1.,)*7 + (0.,)*5


def side_marker(image, command=HOME, x_radius=.26):
    r, g, b = image.astype(float).transpose(2, 0, 1)
    other = np.maximum(g, b)
    roi = (abs(U-command[0]) < x_radius) & (abs(Z-command[2]) < .26)
    components, n = label((r-2.25*other > 32) & roi)
    if not n:
        return (np.nan, np.nan)
    ys, xs = np.where(components == np.argmax(np.bincount(components.ravel())[1:])+1)
    if len(xs) < 3:
        return (np.nan, np.nan)
    nearby = (abs(U-U[ys, xs].mean()) < .14) & (abs(Z-Z[ys, xs].mean()) < .14)
    w = np.maximum(0., r-2*other)*nearby
    # The upper red band is fixed to the carriage. The cup below it can
    # pitch or be masked by the shell; its centroid is not a load gauge.
    profile = w.sum(axis=1)
    threshold = .35*profile.max()
    row = np.flatnonzero(profile > threshold)[0]
    fraction = ((profile[row]-threshold)/(profile[row]-profile[row-1])) if row else 0.
    return ((U*w).sum()/w.sum(), Z[row, 0]+fraction/32)


def motor_matrix(cassette):
    sign, ra, rb = cassette
    return np.array([[sign/ra, sign/ra], [sign/rb, -sign/rb]])


@dataclass
class CassetteProbe:
    tick: int = 0
    home_x: list = field(default_factory=list)
    peaks: list = field(default_factory=lambda: [[], []])
    cassette: tuple | None = None
    shifts: tuple | None = None

    def action(self, obs):
        axis, t = divmod(self.tick, len(PULSE))
        if t in (0, 10, 11, 12, 22, 23):
            x, _ = side_marker(decode(obs, 1), x_radius=.80)
            if np.isfinite(x):
                (self.peaks[axis] if t in (10, 11, 12) else self.home_x).append(x)
        if axis == 2:
            if self.home_x and all(self.peaks):
                shifts = np.array([np.median(p)-np.median(self.home_x) for p in self.peaks])
                signed = shifts*2/(7*.14)
                sign = 1 if signed.sum() > 0 else -1
                radii = np.where(abs(signed) > 1., 1.2, .8)
                if np.max(abs(signed-sign*radii)) < .18:
                    self.shifts = tuple(shifts)
                    self.cassette = (sign, *radii)
                    return None
            # A view was lost; the physical command is already back at home.
            self.tick, self.home_x, self.peaks = 0, [], [[], []]
            return self.action(obs)
        action = [0.]*4
        action[axis] = PULSE[t]
        self.tick += 1
        return action

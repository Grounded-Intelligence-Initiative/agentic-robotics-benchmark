"""The scout's map, in the scout's own frame (its body frame at its first observation, origin at the dock).

Three things are kept, all on one grid at the scout camera's floor scale (10.9 mm per cell):

  the floor mosaic   the brightness of the floor wherever a frame showed floor: what the carrier's floor
                     camera will be matched against, and what every new scout frame is registered on;
  free votes         how often a cell was seen as floor in the trusted centre of a frame;
  solid votes        how often a cell was seen occupied at floor level (the foot of a wall, a crate's footprint).

Every new frame is registered on the mosaic built so far (turn and shift, to a fraction of a pixel), so a slip
(the odometer over-reads, the body turns) is simply measured, and the map stays consistent with itself.
"""
import math

import numpy as np
from scipy import ndimage

import perception as P
import vision as V

SIZE = 1800                      # canvas cells: +-9.8 m around the dock
ORIGIN = (SIZE-1)/2
MPP = V.SCOUT_MPP

WINDOW = (V.hann()*(~P.OWN)).astype(np.float32)
_INNER = ndimage.binary_erosion(np.ones((V.N, V.N), bool), iterations=2, border_value=0)


def to_cell(x, y):
    """Map point (metres) -> canvas (row, col), fractional."""
    return ORIGIN-y/MPP, ORIGIN+x/MPP


class Mosaic:
    def __init__(self):
        self.gsum = np.zeros((SIZE, SIZE), np.float32)       # floor brightness, summed
        self.gcnt = np.zeros((SIZE, SIZE), np.float32)       # floor observations
        self.free = np.zeros((SIZE, SIZE), np.float32)       # votes: floor, seen in the trusted centre
        self.solid = np.zeros((SIZE, SIZE), np.float32)      # votes: occupied at floor level
        self.padv = np.zeros((SIZE, SIZE), np.float32)       # votes: the pad
        self.box = [SIZE, 0, SIZE, 0]                        # rows and columns written so far

    def expected(self, pose):
        """The floor the scout's camera would show at `pose` according to the mosaic, and where it is known."""
        x, y, th = pose
        A, t = V.rot(th), (x, y)
        s = V.resample(self.gsum, A, t, MPP, (ORIGIN, ORIGIN), MPP, (V.H, V.H), (V.N, V.N))
        c = V.resample(self.gcnt, A, t, MPP, (ORIGIN, ORIGIN), MPP, (V.H, V.H), (V.N, V.N))
        known = c > .5
        return np.where(known, s/np.maximum(c, 1e-3), 0.), known

    def add(self, image, pose, masks):
        """Write a frame (its grey image), taken at `pose`, into the map."""
        ground, solid, pad, floor = masks
        x, y, th = pose
        half = V.N*MPP/2*(abs(math.cos(th))+abs(math.sin(th)))+2*MPP
        c0 = max(0, int(math.floor(ORIGIN+(x-half)/MPP)))
        c1 = min(SIZE, int(math.ceil(ORIGIN+(x+half)/MPP))+1)
        r0 = max(0, int(math.floor(ORIGIN-(y+half)/MPP)))
        r1 = min(SIZE, int(math.ceil(ORIGIN-(y-half)/MPP))+1)
        shape = (r1-r0, c1-c0)
        A = V.rot(-th)                                        # canvas point q -> body point R(-th)(q - p)
        t = -A@np.array([x, y])
        origin_out = (ORIGIN-r0, ORIGIN-c0)

        def warp(layer):
            return V.resample(layer.astype(np.float32), A, t, MPP, (V.H, V.H), MPP, origin_out, shape)
        region = (slice(r0, r1), slice(c0, c1))
        seen = warp(floor & _INNER) > .99
        self.gsum[region] += np.where(seen, warp(image), 0.)
        self.gcnt[region] += seen
        self.free[region] += warp(ground & _INNER) > .99
        self.solid[region] += warp(solid) > .5
        self.padv[region] += warp(pad) > .5
        self.box = [min(self.box[0], r0), max(self.box[1], r1), min(self.box[2], c0), max(self.box[3], c1)]

    # --- what the planners read ------------------------------------------------------------- #
    def pad(self):
        """The pad's centre in the map frame, or None while it has not been seen."""
        r0, r1, c0, c1 = self.box
        mask = self.padv[r0:r1, c0:c1] >= 1
        if mask.sum() < 150:
            return None
        labels, _ = ndimage.label(mask)
        sizes = np.bincount(labels.ravel())
        sizes[0] = 0
        rows, cols = np.nonzero(labels == int(np.argmax(sizes)))
        return np.array([(cols.mean()+c0-ORIGIN)*MPP, (ORIGIN-rows.mean()-r0)*MPP])

    def grids(self, scale, align=None):
        """(free, solid, seen, origin, metres per cell) on a grid `scale` times coarser, cropped to what has
        been written (to multiples of `align` cells): a coarse cell is free when all of it was seen as floor at
        least twice and none of it as occupied; solid when some of it was seen occupied at least twice; seen
        when a frame showed its floor at all (anywhere in the frame).
        `origin`: the fractional (row, col) of the map's origin in that grid."""
        align = align or scale
        r0, r1, c0, c1 = self.box
        r0, c0 = (r0//align)*align, (c0//align)*align
        r1, c1 = min(SIZE, -(-r1//align)*align), min(SIZE, -(-c1//align)*align)

        def blocks(a):
            return a.reshape((r1-r0)//scale, scale, (c1-c0)//scale, scale)
        region = (slice(r0, r1), slice(c0, c1))
        # (a foot is seen as floor by no frame; a cell seen as floor many times and as occupied twice is floor)
        solid = blocks((self.solid[region] >= 2) & (self.solid[region] > .3*self.free[region])).any(axis=(1, 3))
        free = blocks(self.free[region] >= 2).all(axis=(1, 3)) & ~solid
        seen = blocks(self.gcnt[region] > .5).mean(axis=(1, 3)) >= .5
        origin = ((ORIGIN-r0+.5)/scale-.5, (ORIGIN-c0+.5)/scale-.5)
        return free, solid, seen, origin, MPP*scale


class Tracker:
    """The scout's pose in the map, from each new frame registered on the mosaic."""

    SWEEP = np.radians(np.arange(-9., 9.01, 1.))

    def __init__(self, mosaic, image):
        self.mosaic = mosaic
        self.pose = np.zeros(3)
        self.scores = []
        self.masks = P.scout_masks(image)
        mosaic.add(V.gray(image), self.pose, self.masks)

    def update(self, image, stride):
        """`stride`: the odometer's reading of the last press (body frame). Returns the pose (x, y, heading) and
        the registration score."""
        x, y, th = self.pose
        guess = np.array([*(np.array([x, y])+V.rot(th)@np.asarray(stride, float)), th])
        expected, known = self.mosaic.expected(guess)
        fill = float(expected[known].mean()) if known.any() else 0.
        ref = V.highpass(np.where(known, expected, fill))*WINDOW*ndimage.binary_erosion(known, iterations=4)
        self.masks = P.scout_masks(image)
        floor = self.masks[3]
        g = V.gray(image)
        fill = float(g[floor].mean()) if floor.any() else 0.
        cur = V.highpass(np.where(floor, g, fill))*WINDOW*ndimage.binary_erosion(floor, iterations=3)
        step = math.radians(1.)
        results = V.register(cur, ref, [-step, 0., step])
        usual = np.median(self.scores[-30:]) if len(self.scores) >= 5 else .5
        if max(r[0] for r in results) < .75*usual or int(np.argmax([r[0] for r in results])) != 1:
            # A slip turned the body: look over the whole range of turns.
            centre, _ = V.best_angle(V.register(cur, ref, self.SWEEP))
            results = V.register(cur, ref, [centre-step, centre, centre+step])
        angle, _ = V.best_angle(results)
        score, _, dr, dc = V.register(cur, ref, [angle])[0]
        if score < .35*usual:
            # Nothing to hold on to in this frame (hardly any known floor in it): go by the odometer for this
            # press and keep the frame out of the map.
            self.pose = guess
            return self.pose.copy(), score
        self.scores.append(score)
        shift = np.array([dc*MPP, -dr*MPP])                 # body-frame offset from the guess
        self.pose = np.array([*(guess[:2]+V.rot(guess[2])@shift), guess[2]+angle])
        self.mosaic.add(g, self.pose, self.masks)
        return self.pose.copy(), score

"""The carrier finds itself in the scout's map by the floor.

The carrier's floor camera shows about 0.6 m of floor under its base (the exact span is not known: between
0.56 and 0.64 m), with its own exposure, colour balance, dark corners and noise. The scout's mosaic holds the
same floor at 10.9 mm per cell, in the scout's frame. So:

  relocalize  the first floor image, reduced to the mosaic's scale, is compared with the mosaic around the
              dock at every heading (3 degree steps) and five spans, by normalized correlation of the local
              contrast; the best match is refined in heading, span and position;
  track       after every press the new floor image is registered on the previous one (turn and shift, to a
              fraction of a pixel: visual odometry that no slip can fool), and the predicted pose is then
              corrected against the mosaic; the span is refined from the two measures of the distance driven.
"""
import math

import numpy as np
from numpy.fft import irfft2, rfft2
from scipy import ndimage

import mapping as M
import vision as V

SPANS = (.56, .58, .60, .62, .64)
RADIUS = 23                      # template radius in mosaic cells (0.25 m: inside the narrowest span)
SIDE = 2*RADIUS+1
_yy, _xx = np.mgrid[:SIDE, :SIDE]
DISC = (_xx-RADIUS)**2+(_yy-RADIUS)**2 <= RADIUS**2
COUNT = float(DISC.sum())
WINDOW = V.hann()


def contrast(g, sigma, valid=None):
    """Local contrast, (g - local mean) / local mean: free of exposure, vignetting and slow shading."""
    if valid is None:
        mean = ndimage.gaussian_filter(g, sigma)
        return (g-mean)/np.maximum(mean, 1.)
    weight = ndimage.gaussian_filter(valid.astype(np.float32), sigma)
    mean = ndimage.gaussian_filter(np.where(valid, g, 0.), sigma)/np.maximum(weight, 1e-3)
    return np.where(valid, (g-mean)/np.maximum(mean, 1.), 0.)


class Search:
    """Normalized correlation of the disc template at every placement in a square piece of the mosaic."""

    def __init__(self, image, valid, size):
        self.size = size
        self.image = np.zeros((size, size), np.float32)
        self.image[:image.shape[0], :image.shape[1]] = image
        known = np.zeros((size, size), np.float32)
        known[:valid.shape[0], :valid.shape[1]] = valid
        self.F = rfft2(self.image)
        kernel = np.zeros((size, size), np.float32)
        kernel[:SIDE, :SIDE] = DISC
        K = np.conj(rfft2(kernel))
        s1 = irfft2(self.F*K, s=(size, size))
        s2 = irfft2(rfft2(self.image*self.image)*K, s=(size, size))
        cover = irfft2(rfft2(known)*K, s=(size, size))/COUNT
        self.deviation = np.sqrt(np.maximum(s2-s1*s1/COUNT, 1e-9))
        self.usable = cover > .85
        self.usable[size-SIDE+1:, :] = False
        self.usable[:, size-SIDE+1:] = False

    def scores(self, template):
        t = np.where(DISC, template, 0.)
        t = np.where(DISC, t-t.sum()/COUNT, 0.)
        norm = math.sqrt(float((t*t).sum()))+1e-9
        kernel = np.zeros((self.size, self.size), np.float32)
        kernel[:SIDE, :SIDE] = t
        out = irfft2(self.F*np.conj(rfft2(kernel)), s=(self.size, self.size))/(norm*self.deviation)
        return np.where(self.usable, out, -1.)

    def best(self, template):
        """(score, row, col) of the best placement of the template's centre, to a fraction of a cell."""
        s = self.scores(template)
        r, c = np.unravel_index(int(np.argmax(s)), s.shape)
        v = float(s[r, c])

        def refine(m, z, p):
            d = m-2*z+p
            return 0. if d >= 0 or min(m, p) < -.5 else float(np.clip(.5*(m-p)/d, -.5, .5))
        dr = refine(s[r-1, c], v, s[min(r+1, self.size-1), c]) if r > 0 else 0.
        dc = refine(s[r, c-1], v, s[r, min(c+1, self.size-1)]) if c > 0 else 0.
        return v, r+dr+RADIUS, c+dc+RADIUS


class FloorLocalizer:
    def __init__(self, mosaic, log=lambda *a: None):
        self.log = log
        r0, r1, c0, c1 = mosaic.box
        self.corner = (r0, c0)
        valid = mosaic.gcnt[r0:r1, c0:c1] > .5
        mean = np.where(valid, mosaic.gsum[r0:r1, c0:c1]/np.maximum(mosaic.gcnt[r0:r1, c0:c1], 1e-3), 0.)
        self.valid = valid
        self.floor = contrast(mean, 4.3, valid).astype(np.float32)
        self.span = .60
        self.pose = None
        self.previous = None
        self.scores = []
        self.driven_vo = np.zeros(2)       # distance driven since the last span update, by visual odometry
        self.driven_map = np.zeros(2)      # and by the map
        self.anchor = None

    # --- the floor image at the mosaic's scale --------------------------------------------------- #
    def prepare(self, image):
        g = V.gray(image)
        return contrast(ndimage.gaussian_filter(g, 1.6), 20.)

    def template(self, prepared, heading, span):
        """The floor image as the mosaic would hold it for a carrier at `heading`: out(q) = image(R(-heading) q)."""
        return V.resample(prepared, V.rot(-heading), (0., 0.), span/V.N, (V.H, V.H), M.MPP, (RADIUS, RADIUS), (SIDE, SIDE))

    def _piece(self, centre, half):
        """A square piece of the mosaic around the map point `centre`: (search, row0, col0)."""
        r, c = M.to_cell(centre[0], centre[1])
        size = 2*half+SIDE
        r0 = int(round(r-self.corner[0]))-half-RADIUS
        c0 = int(round(c-self.corner[1]))-half-RADIUS
        rows = slice(max(0, r0), max(0, min(self.floor.shape[0], r0+size)))
        cols = slice(max(0, c0), max(0, min(self.floor.shape[1], c0+size)))
        image = np.zeros((size, size), np.float32)
        valid = np.zeros((size, size), bool)
        image[rows.start-r0:rows.stop-r0, cols.start-c0:cols.stop-c0] = self.floor[rows, cols]
        valid[rows.start-r0:rows.stop-r0, cols.start-c0:cols.stop-c0] = self.valid[rows, cols]
        fft = 1 << int(math.ceil(math.log2(size)))
        return Search(image, valid, fft), r0, c0

    def _point(self, row, col, r0, c0):
        return np.array([(col+c0+self.corner[1]-M.ORIGIN)*M.MPP, (M.ORIGIN-(row+r0+self.corner[0]))*M.MPP])

    # --- finding itself --------------------------------------------------------------------- #
    def relocalize(self, image, around=(0., 0.), reach=.95):
        """The carrier's pose in the map from one floor image: somewhere within `reach` of `around`, at any
        heading."""
        prepared = self.prepare(image)
        search, r0, c0 = self._piece(around, int(reach/M.MPP)+4)
        best = (-2.,)
        for span in SPANS:
            for heading in np.radians(np.arange(0., 360., 3.)):
                score, row, col = search.best(self.template(prepared, heading, span))
                if score > best[0]:
                    best = (score, heading, span, row, col)
        score, heading, span, row, col = best
        for step_h, step_s in ((math.radians(1.), .01), (math.radians(.3), .004)):
            for h in heading+step_h*np.arange(-2, 3):
                for s in span+step_s*np.arange(-2, 3):
                    if not .55 <= s <= .65:
                        continue
                    sc, r, c = search.best(self.template(prepared, h, s))
                    if sc > best[0]:
                        best = (sc, h, s, r, c)
            score, heading, span, row, col = best
        self.span = float(span)
        self.pose = np.array([*self._point(row, col, r0, c0), heading])
        self.previous = prepared
        self.anchor = self.pose[:2].copy()
        self.log("relocalized: score %.3f pose %s span %.3f" % (score, self.pose.round(3), span))
        return self.pose.copy(), score

    # --- keeping track ---------------------------------------------------------------------- #
    def track(self, image):
        """The pose after a press: visual odometry from the previous floor image, corrected on the mosaic."""
        prepared = self.prepare(image)
        cur, ref = prepared*WINDOW, self.previous*WINDOW
        step = math.radians(1.)
        results = V.register(cur, ref, [-step, 0., step], limit=100)
        usual = np.median(self.scores[-20:]) if len(self.scores) >= 5 else .4
        if max(r[0] for r in results) < .7*usual or int(np.argmax([r[0] for r in results])) != 1:
            centre, _ = V.best_angle(V.register(cur, ref, np.radians(np.arange(-9., 9.01, 1.)), limit=100))
            results = V.register(cur, ref, [centre-step, centre, centre+step], limit=100)
        turn, _ = V.best_angle(results)
        score, _, dr, dc = V.register(cur, ref, [turn], limit=100)[0]
        self.scores.append(score)
        mpp = self.span/V.N
        moved = V.rot(self.pose[2])@np.array([dc*mpp, -dr*mpp])
        guess = np.array([*(self.pose[:2]+moved), self.pose[2]+turn])
        self.driven_vo += moved
        # The mosaic's say: the template at the predicted heading (and a degree either side), near the prediction.
        search, r0, c0 = self._piece(guess[:2], 10)
        fits = []
        for h in (guess[2]-step, guess[2], guess[2]+step):
            fits.append((*search.best(self.template(prepared, h, self.span)), h))
        k = int(np.argmax([f[0] for f in fits]))
        fit_score, row, col, _ = fits[k]
        if fit_score > .35:
            # the heading the mosaic likes best, between the three tried
            heading = guess[2]
            m, z, p = fits[0][0], fits[1][0], fits[2][0]
            d = m-2*z+p
            if d < 0:
                heading = guess[2]+float(np.clip(.5*(m-p)/d, -1., 1.))*step
            if k != 1:
                fit_score, row, col = search.best(self.template(prepared, heading, self.span))
            fixed = self._point(row, col, r0, c0)
            if np.hypot(*(fixed-guess[:2])) < .08:
                self.driven_map += fixed-self.pose[:2]
                # (the mosaic is trusted for position; for heading it is averaged with the odometry)
                self.pose = np.array([*fixed, .5*(heading+guess[2])])
            else:
                self.driven_map += moved
                self.pose = guess
        else:
            self.driven_map += moved
            self.pose = guess
        self.previous = prepared
        if np.hypot(*self.driven_vo) > .6:
            ratio = float(np.hypot(*self.driven_map)/np.hypot(*self.driven_vo))
            if .93 < ratio < 1.07:
                self.span = float(np.clip(self.span*ratio, .55, .65))
            self.driven_vo[:] = 0.
            self.driven_map[:] = 0.
        return self.pose.copy(), score, fit_score

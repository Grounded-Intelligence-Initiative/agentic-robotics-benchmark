"""What one frame of the scout's mast camera says about the floor plane.

The camera is a pinhole 2.0 m above the floor, looking straight down: a floor point appears where it is,
but a point at height h appears at 2 / (2 - h) times its distance from the image centre. So a wall or a
crate leans away from the centre, shows its near side faces and hides the floor behind it, and the colour
of a pixel says nothing about the floor point it covers unless that pixel is floor. A frame therefore yields

  ground  pixels that show the floor itself (concrete, paint, the pad): free at that very place. Only the
          central 1.6 m square of the image is used: farther out a crate's pale, sunlit side passes for floor
          while its lid, which tells them apart, may be out of the picture;
  solid   places certainly occupied at floor level: the foot of a near face (the first wall or crate-side
          pixel on a ray from the centre) and a crate's footprint (its lid, a known 0.62 m up, brought down);
  pad     the delivery pad;

and nothing about the rest (hidden, or leaning obstacle faces).
"""
import numpy as np
from scipy import ndimage

import vision as V

LENS = 2.0                       # the mast camera's height above the floor
LID = .62                        # height of a crate's lid
RIM = 3                          # pixels a crate's top extends beyond its lid (3 cm)
BODY = .42                       # the scout and its shadow hide the floor within this radius of the centre
TRUST = .80                      # half-width of the central square in which a floor-coloured pixel is floor

_yy, _xx = np.mgrid[:V.N, :V.N]
_dx, _dy = _xx-V.H, _yy-V.H
_rho = np.hypot(_dx, _dy)
OWN = _rho*V.SCOUT_MPP < BODY
# A crate's sunlit side can have the colour of the floor. Its lid cannot, and the lid lies on the same ray at
# most (LENS / (LENS - LID)) = 1.45 times as far out: inside this square the lid would be in the image too.
CENTRAL = (np.abs(_dx) < TRUST/V.SCOUT_MPP) & (np.abs(_dy) < TRUST/V.SCOUT_MPP)
# For every pixel: the pixel two steps nearer the image centre, and the two pixels one step to either side of
# its ray.
_k = np.maximum(_rho, 1e-6)
_IN_R = np.clip(np.rint(_yy-2*_dy/_k).astype(int), 0, V.N-1)
_IN_C = np.clip(np.rint(_xx-2*_dx/_k).astype(int), 0, V.N-1)
_SIDE = [(np.clip(np.rint(_yy+s*1.5*_dx/_k).astype(int), 0, V.N-1), np.clip(np.rint(_xx-s*1.5*_dy/_k).astype(int), 0, V.N-1))
         for s in (-1, 1)]
_INSIDE = (np.abs(_dx) < V.H-3) & (np.abs(_dy) < V.H-3)       # not at the very edge of the image
_SQUARE5 = np.ones((5, 5), bool)


def _components(mask, min_pixels):
    labels, count = ndimage.label(mask)
    if not count:
        return mask
    sizes = np.bincount(labels.ravel())
    keep = sizes >= min_pixels
    keep[0] = False
    return keep[labels]


def _scaled(mask, factor):
    """The mask shrunk toward the image centre: out(q) = mask(q / factor)."""
    return ndimage.affine_transform(mask.astype(np.float32), np.eye(2)/factor, offset=V.H*(1-1/factor),
                                    order=1, mode="constant", cval=0.) > .5


def feet(standing, floor):
    """Where something that stands on the floor meets it, as seen from above: the pixels of `standing` whose
    neighbour toward the image centre is floor. A vertical edge of a wall or a crate lies along a ray too and
    has floor beside it, but it rises from the floor: there the pixel beside the ray is floor as well, and such
    pixels are left out."""
    beside = floor[_SIDE[0]] | floor[_SIDE[1]]
    return standing & floor[_IN_R, _IN_C] & ~beside & _INSIDE


def colours(image):
    px = image.astype(np.float32)
    r, g, b = px[..., 0], px[..., 1], px[..., 2]
    lum = px.mean(axis=2)
    rb, gb, rg = r/np.maximum(b, 1.), g/np.maximum(b, 1.), r/np.maximum(g, 1.)
    concrete = (lum > 55) & (lum < 230) & (rb > 1.08) & (rb < 1.75) & (gb > 1.04) & (gb < 1.45) & (rg > 1.02) & (rg < 1.22)
    yellow = (rb > 2.0) & (rg > .95) & (rg < 1.5) & (lum > 110)          # floor paint, and the crates' straps
    pad = (b > 215) & (r < 115) & (g < 125)
    blue = (rb < .59) & (b > 30) & ~pad                                   # walls: tops and sides, lit or not
    dark = (lum < 62) & ~concrete & ~blue & ~pad                          # a crate's lid, or a wall's unlit side
    rust = (rb >= 1.75) & (lum > 100) & ~yellow                           # the crates' sides (with pale patches)
    strap = (rb > 2.6) & (rg > 1.) & (rg < 1.4) & (lum > 120)             # saturated yellow: straps (and paint)
    return concrete, yellow, pad, blue, dark, rust, strap


def scout_masks(image):
    """(ground, solid, pad, floor) of a scout frame, each a boolean image; `floor` is the floor-coloured part of
    the whole image (what registration and the floor mosaic use), `ground` its trusted, central part."""
    concrete, yellow, pad, blue, dark, rust, strap = colours(image)
    # A crate's top: a dark lid crossed by yellow straps (a dark patch without straps is a wall's unlit side).
    # Its footprint is the top brought down to the floor.
    lid = _components(ndimage.binary_opening(dark, structure=np.ones((3, 3), bool)), 300)
    crate_image = np.zeros_like(lid)
    footprint = np.zeros_like(lid)
    top = np.zeros_like(lid)
    if lid.any():
        labels, count = ndimage.label(ndimage.binary_dilation(lid, structure=_SQUARE5, iterations=2))
        straps = strap & (labels > 0)
        sizes = np.bincount(labels[straps], minlength=count+1)
        walls = np.bincount(labels[blue], minlength=count+1)
        keep = (sizes >= 20) & (walls < 30)                 # straps across it, and no wall beside it
        keep[0] = False
        top = ndimage.binary_closing((lid | straps) & keep[labels], structure=_SQUARE5, iterations=2)
        if top.any():
            top = ndimage.binary_dilation(top, iterations=RIM)
            down = (LENS-LID)/LENS
            footprint = _scaled(top, down)
            crate_image = top.copy()
            for factor in np.linspace(down, 1., 14)[:-1]:
                crate_image |= _scaled(top, factor)
            crate_image = ndimage.binary_dilation(crate_image, iterations=2)
    # (the pad is 4 cm thick: its own dark blue rim is neither a wall nor an obstacle)
    pad_area = ndimage.binary_dilation(_components(pad, 60), structure=_SQUARE5, iterations=2)
    wall = _components((blue | dark) & ~top & ~pad_area, 60)
    # Paint is a thin line; a broad yellow or rusty patch is the side of a crate.
    side = _components(ndimage.binary_opening(rust | yellow, structure=_SQUARE5), 120)
    side = ndimage.binary_dilation(side, structure=_SQUARE5)
    # The floor: concrete, paint and the pad, less stains' holes, in one large piece.
    ground = (concrete | yellow | pad_area) & ~wall & ~crate_image & ~side
    ground |= ~_components(~ground, 40) & ~wall & ~crate_image & ~side
    core = _components(ndimage.binary_opening(ground | OWN, structure=_SQUARE5), 1500)
    floor = ground & ndimage.binary_dilation(core, structure=_SQUARE5) & ~OWN
    ground = floor & CENTRAL
    # The foot of a wall or of a crate's side: its pixels whose neighbour toward the centre is floor.
    solid = (feet(wall | side, floor) | footprint) & ~OWN
    return ground, solid & ~pad_area, pad & floor, floor

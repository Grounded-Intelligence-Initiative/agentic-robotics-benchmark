"""The field-replaceable parts of the data-hall cell and the tools (cell internals): their geometry, how they look, where
a hand takes hold of them, and the slot rules (axis, travel, alignment tolerances, release and latch forces).

Every removable part has a frame: origin at its slot's mouth (the face it is pushed through), +y pointing OUT of the
slot (the direction it is pulled), z up for a horizontal slot. A GPU module's slot is vertical: its frame has +z out
(up, off its guide pins). A part is seated when its frame coincides with its port's frame. The same look is drawn on a
seated part (a static body of the equipment) and on the free body that stands in for it while it is out (scene.py's
proxies). Grasp features are given in the part frame (grips.feature_score's forms).
"""

from __future__ import annotations

import math

import numpy as np

from . import hall as H
from . import meshes as M
from . import textures as T

# ------------------------------------------------------------------------------------------------ the parts
# kind: dims (x, y_in, y_out, z) of the box a part occupies (y_in behind the mouth, y_out in front of it), mass (kg),
# axis (unit vector, part frame) along which it leaves its slot, travel (m) along the axis while the slot guides it,
# lateral / angular tolerance for entering the slot, the pull that releases its latch, the push that latches it,
# the push at which a misaligned part damages the slot, and the grasp features.
KINDS = {
    "fan": dict(dims=(0.0845, 0.060, 0.010, 0.2547), mass=0.55, axis=(0, 1, 0), travel=0.062, tol=(0.0015, math.radians(3)),
                release_N=8.0, latch_N=12.0, damage_N=40.0,
                features=[("cyl", (-0.025, 0.017, 0.2347 - 0.1274), (0.025, 0.017, 0.2347 - 0.1274), 0.0032, 0.010)]),
    "psu": dict(dims=(0.0710, 0.185, 0.006, 0.0405), mass=1.10, axis=(0, 1, 0), travel=0.190, tol=(0.0015, math.radians(3)),
                release_N=8.0, latch_N=15.0, damage_N=45.0,
                features=[("cyl", (-0.032, 0.022, -0.012), (0.008, 0.022, -0.012), 0.0028, 0.010)]),
    "plug": dict(dims=(0.022, 0.010, 0.048, 0.017), mass=0.12, axis=(0, 1, 0), travel=0.012, tol=(0.0012, math.radians(4)),
                 release_N=10.0, latch_N=20.0, damage_N=60.0,
                 features=[("cyl", (0.0, 0.012, 0.0), (0.0, 0.046, 0.0), 0.009, 0.008)]),
    "osfp_n": dict(dims=(0.0226, 0.070, 0.030, 0.0125), mass=0.06, axis=(0, 1, 0), travel=0.072, tol=(0.0008, math.radians(3)),
                   release_N=6.0, latch_N=18.0, damage_N=28.0,
                   features=[("cyl", (0.0, 0.030, -0.0056), (0.0, 0.052, -0.0056), 0.0012, 0.005)]),
    "osfp_s": dict(dims=(0.0226, 0.070, 0.032, 0.0185), mass=0.08, axis=(0, 1, 0), travel=0.072, tol=(0.0008, math.radians(3)),
                   release_N=6.0, latch_N=18.0, damage_N=28.0,
                   features=[("cyl", (0.0, 0.032, -0.0085), (0.0, 0.054, -0.0085), 0.0012, 0.005)]),
    "mpo": dict(dims=(0.0124, 0.008, 0.034, 0.0080), mass=0.02, axis=(0, 1, 0), travel=0.008, tol=(0.0005, math.radians(3)),
                release_N=5.0, latch_N=10.0, damage_N=25.0,
                features=[("cyl", (0.0, 0.010, 0.0), (0.0, 0.032, 0.0), 0.0050, 0.006)]),
    "gpu": dict(dims=(0.094, 0.0, 0.118, 0.118), mass=1.80, axis=(0, 0, 1), travel=0.015, tol=(0.0015, math.radians(2)),
                release_N=3.0, latch_N=0.0, damage_N=60.0,   # a module sits on its pins: a deliberate lift frees it
                features=[("cyl", (0.0, -0.040, 0.128), (0.0, 0.040, 0.128), 0.0055, 0.012)]),
}
# a GPU module's frame: origin at the middle of its base's underside, +z up (out of its socket), +y towards the back of
# the tray. Its screws (two passes, T20, 0.9 N m) sit at the base's corners, the lift bar on the fins.
GPU_SCREWS = [(-0.041, -0.053), (0.041, -0.053), (-0.041, 0.053), (0.041, 0.053)]    # 1 front-left, 2 front-right,
SCREW_ORDER_NOTE = "1-4-2-3 (diagonal), snug first, then 0.9 N m"                        # 3 back-left, 4 back-right
SCREW = {"turns": 6.0, "rev_per_s": 5.0, "snug_Nm": 0.3, "final_Nm": 0.9, "head_r": 0.0042, "head_h": 0.0055,
         "rise_per_turn": 0.0005, "engage_lat": 0.0010, "engage_ax": (-0.0065, 0.0045), "engage_ang": math.radians(6)}
# the bit stays in the head while its tip is within engage_ax of the head's top (a spring-loaded bit: the head may rise
# or sink by its whole travel, 6 turns x 0.5 mm, without the tool being moved)

# OSFP receptacle (the MPO's port) in the module frame, and the MPO connector's face in its own frame
OSFP_RECEPTACLE = {"osfp_n": (0.0, 0.030, 0.0005), "osfp_s": (0.0, 0.032, 0.0)}
MPO_FACE = (0.0, 0.0, 0.0)

# ------------------------------------------------------------------------------------------------ the tools
TOOLS = {
    "driver": dict(mass=0.45, features=[("cyl", (0.0, 0.0, 0.050), (0.0, 0.0, 0.140), 0.016, 0.012)],
                   tip=(0.0, 0.0, -0.190)),        # the bit's tip below the grip (the tool's axis is its z): a long-reach
                                                    # T20 extension, for the heatsink screws at the bottom of their wells
    "scope": dict(mass=0.30, features=[("cyl", (0.0, 0.0, 0.040), (0.0, 0.0, 0.140), 0.014, 0.012)],
                  tip=(0.0, 0.0, -0.050)),
    "cleaner": dict(mass=0.10, features=[("cyl", (0.0, 0.0, 0.030), (0.0, 0.0, 0.120), 0.011, 0.012)],
                    tip=(0.0, 0.0, -0.045)),
}
SCOPE = {"lat_sharp": 0.0005, "lat_view": 0.0015, "ang": math.radians(3), "ax": (-0.002, 0.003)}
CLEANER = {"lat": 0.0006, "ang": math.radians(4), "click_depth": 0.002}


# ------------------------------------------------------------------------------------------------ looks
# A part is drawn as ONE mesh (one renderable) sampling an atlas: the left half holds its face picture (a power
# supply's, a fan module's), the right half flat swatches; the ORM atlas carries each swatch's roughness / metal.
ATLAS = 512
SWATCHES = {"dark": ((0.012, 0.012, 0.014), 0.50, 0.0), "alu": ((0.62, 0.63, 0.65), 0.38, 0.75),
            "beige": ((0.78, 0.70, 0.55), 0.60, 0.0), "blue": ((0.06, 0.28, 0.72), 0.45, 0.0),
            "aqua": ((0.10, 0.62, 0.70), 0.45, 0.0), "orange": ((0.88, 0.36, 0.08), 0.50, 0.0),
            "black": ((0.02, 0.02, 0.022), 0.50, 0.0), "screw": ((0.03, 0.03, 0.035), 0.35, 0.6),
            "base": ((0.05, 0.05, 0.055), 0.40, 0.0), "alu_dark": ((0.40, 0.41, 0.43), 0.40, 0.75)}
_SW_INDEX = {k: i for i, k in enumerate(SWATCHES)}


def _sw_uv(name):
    i = _SW_INDEX[name]
    c, r = i % 4, i // 4
    x = ATLAS // 2 + c * 64 + 32
    y = r * 64 + 32
    return x / ATLAS, 1.0 - y / ATLAS


def _atlas_images(face=None):
    from .textures import _srgb_from_lin, _lin_from_srgb  # noqa: F401
    alb = np.zeros((ATLAS, ATLAS, 3), np.uint8)
    orm = np.zeros((ATLAS, ATLAS, 3), np.uint8)
    orm[..., 0] = 255
    orm[..., 1] = 128
    if face is not None:
        from PIL import Image
        img = np.asarray(Image.fromarray(face).resize((ATLAS // 2, ATLAS)))
        alb[:, :ATLAS // 2] = img
        orm[:, :ATLAS // 2, 1] = 128
    for k, (rgb, rough, metal) in SWATCHES.items():
        i = _SW_INDEX[k]
        c, r = i % 4, i // 4
        x0, y0 = ATLAS // 2 + c * 64, r * 64
        alb[y0:y0 + 64, x0:x0 + 64] = np.clip(np.round(255 * np.asarray(rgb) ** (1 / 2.2)), 0, 255).astype(np.uint8)
        orm[y0:y0 + 64, x0:x0 + 64, 1] = int(255 * rough)
        orm[y0:y0 + 64, x0:x0 + 64, 2] = int(255 * metal)
    return alb, orm


def _paint(mesh, region):
    """Map a mesh's texture coordinates into the atlas: "face" = its own 0..1 coordinates into the left half, a swatch
    name = every vertex at that swatch's centre."""
    V, F, N, UV = mesh
    UV = np.array(UV, float)
    if region == "face":
        UV = np.stack([np.clip(UV[:, 0], 0.0, 1.0) * 0.5, np.clip(UV[:, 1], 0.0, 1.0)], 1)
    else:
        u, v = _sw_uv(region)
        UV = np.tile([u, v], (len(V), 1))
    return V, F, N, UV


def _look_mesh(kind):
    """(pieces [(mesh, region)], face picture or None) of a part kind, in the part frame."""
    if kind == "fan":
        Ht = 0.2547
        panel = M.transform(M.panel(0.0845, Ht, 0.060, "+y"), t=(0.0, -0.030, 0.0))
        V, F, N, UV = panel
        return [((V, F, N, UV), "face"),
                (M.transform(M.bar_handle(0.050, 0.016, 0.0032, axis="x"), t=(0.0, 0.001, Ht / 2 - 0.020)), "orange")]
    if kind == "psu":
        return [(M.panel(H.PSU_W - 0.0015, H.PSU_H, 0.006, "+y"), "face"),
                (M.transform(M.box((0.0352, 0.090, 0.0198)), t=(0.0, -0.095, 0.0)), "alu_dark"),
                (M.transform(M.bar_handle(0.040, 0.022, 0.0028, axis="x"), t=(-0.012, 0.001, -0.012)), "black")]
    if kind == "plug":
        return [(M.transform(M.rounded_box((0.011, 0.024, 0.0085), 0.002, n=2), t=(0.0, 0.024, 0.0)), "black")]
    if kind == "osfp_n":
        return [(M.transform(M.box((0.0110, 0.050, 0.0062)), t=(0.0, -0.020, 0.0)), "alu"),
                (M.transform(M.box((0.0035, 0.012, 0.0012)), t=(0.0, 0.041, -0.0056)), "beige")]
    if kind == "osfp_s":
        return [(M.transform(M.merge(M.box((0.0110, 0.051, 0.0062)),
                                     M.transform(M.fin_stack(0.020, 0.060, 0.006, 8, 0.0006, base=0.001), t=(0, 0.0, 0.0062))),
                             t=(0.0, -0.019, 0.0)), "alu"),
                (M.transform(M.box((0.0035, 0.012, 0.0012)), t=(0.0, 0.043, -0.0085)), "beige")]
    if kind == "mpo":
        return [(M.transform(M.box((0.0062, 0.017, 0.0040)), t=(0.0, 0.017, 0.0)), "dark"),
                (M.tube([(0, 0.034, 0), (0, 0.050, 0)], 0.0035, 10), "aqua")]
    if kind == "gpu":
        gm = H.GPU_MODULE
        pieces = [(M.transform(M.box((gm["w"] / 2, gm["d"] / 2, gm["base"] / 2)), t=(0, 0, gm["base"] / 2)), "base"),
                  (M.transform(M.fin_stack(gm["w"] - 0.006, gm["d"] - 0.008, gm["fins_h"], 34, 0.0007, base=0.005),
                               t=(0, 0, gm["base"])), "alu"),
                  (M.merge(M.tube([(0, -0.040, 0.128), (0, 0.040, 0.128)], 0.0055, 12),
                           M.transform(M.box((0.004, 0.004, 0.008)), t=(0, -0.040, 0.118)),
                           M.transform(M.box((0.004, 0.004, 0.008)), t=(0, 0.040, 0.118))), "blue")]
        screw = M.lathe([(0.0, 0.0), (0.0042, 0.0), (0.0042, 0.0045), (0.0030, 0.0055), (0.0, 0.0055)], 18, cap_top=True)
        for sx, sy in GPU_SCREWS:
            pieces.append((M.transform(screw, t=(sx, sy, gm["base"])), "screw"))
        return pieces
    raise KeyError(kind)


_LOOK_CACHE = {}


def ensure_look(B, kind):
    """Register a part kind's plain look mesh ("look_<kind>") and return its name."""
    name = "look_" + kind
    if name not in B.meshes:
        base = _LOOK_CACHE.get(kind)
        if base is None:
            base = _LOOK_CACHE[kind] = M.merge(*[_paint(m, r) for m, r in _look_mesh(kind)])
        B.mesh(name, base)
    return name


def add_look(B, body, kind, *, key="", idx=1, feed="A", extra=None):
    """Draw part `kind` on `body` (the body's frame = the part frame) as one mesh geom with its atlas material;
    `extra` = [(mesh, swatch)] drawn with it (a seated connector's cable: then the mesh is the seat's own)."""
    face = None
    variant = kind
    if kind == "fan":
        face = T.fan_module_front(idx=idx)
        variant = "fan%d" % idx
    elif kind == "psu":
        face = T.psu_face(idx=idx, feed=feed)
        variant = "psu%d%s" % (idx, feed)
    mname = "look_" + variant
    if extra:
        mname = "look_" + body.name
    if mname not in B.meshes:
        base = _LOOK_CACHE.get(kind)
        if base is None:
            base = _LOOK_CACHE[kind] = M.merge(*[_paint(m, r) for m, r in _look_mesh(kind)])
        data = base if not extra else M.merge(base, *[_paint(m, r) for m, r in extra])
        B.mesh(mname, data)
    alb, orm = _atlas_images(face)
    mat = B.bank.atlas(variant, alb, orm)
    g = B.vis(body, mname, None, mat, name="g_" + body.name)
    return [g]


def _mk(B, name, data):
    if name not in B.meshes:
        B.mesh(name, data)
    return name


def add_tool_look(B, body, tool):
    bank = B.bank
    out = []
    if tool == "driver":
        grip = _mk(B, "tl_driver", M.merge(M.lathe([(0.0, 0.03), (0.020, 0.03), (0.020, 0.15), (0.0, 0.155)], 24, cap_top=True),
                                           M.lathe([(0.0, -0.19), (0.0025, -0.19), (0.0025, -0.17), (0.0032, -0.165),
                                                    (0.0032, 0.03), (0.0, 0.03)], 12)))
        out.append(B.vis(body, grip, None, bank.plain((0.85, 0.32, 0.06), 0.5)))
        motor = _mk(B, "tl_driver_m", M.transform(M.box((0.022, 0.035, 0.030)), t=(0.0, 0.020, 0.150)))
        out.append(B.vis(body, motor, None, bank.plain((0.04, 0.04, 0.045), 0.5)))
    elif tool == "scope":
        grip = _mk(B, "tl_scope", M.merge(M.lathe([(0.0, 0.02), (0.017, 0.02), (0.017, 0.15), (0.0, 0.155)], 24, cap_top=True),
                                          M.lathe([(0.0, -0.05), (0.0065, -0.05), (0.0065, 0.02), (0.0, 0.02)], 16)))
        out.append(B.vis(body, grip, None, bank.plain((0.10, 0.35, 0.60), 0.5)))
    elif tool == "cleaner":
        grip = _mk(B, "tl_cleaner", M.merge(M.lathe([(0.0, 0.0), (0.013, 0.0), (0.013, 0.125), (0.0, 0.13)], 20, cap_top=True),
                                            M.lathe([(0.0, -0.045), (0.006, -0.045), (0.006, 0.0), (0.0, 0.0)], 14)))
        out.append(B.vis(body, grip, None, bank.plain((0.95, 0.80, 0.10), 0.5)))
    return out


def collider_half(kind):
    """Half sizes and centre (part frame) of a part's one box collider."""
    x, yin, yout, z = KINDS[kind]["dims"]
    if kind == "gpu":
        gm = H.GPU_MODULE
        return (gm["w"] / 2, gm["d"] / 2, (gm["base"] + gm["fins_h"]) / 2), (0.0, 0.0, (gm["base"] + gm["fins_h"]) / 2)
    return (x / 2, (yin + yout) / 2, z / 2), (0.0, (yout - yin) / 2, 0.0)

"""The calibration site `cal` of task 07, version 1.1 (calgen 0.2.0): one hand-laid site package built from the
families' own parts (w2gen: boxes, opening classes, the camera rule, lighting, MJCF, the free-block spacing rule;
w3gen: the site depth, the camera rectangle, the plan radius and clear heights of the slab, point 3's clearance, the
blind-route window), every dimension public (the calibration page states them; it never says what passes).

    python -I -B calgen.py --out <dir> [--no-render] [--envelope <capability table json>]
                           [--passage-length L]   (not the calibration site unless L = PASSAGE_LEN)

writes <dir>/cal/{site.json, site.xml, qc.json}, <dir>/cal.tar.xz (the package the practice ground serves; members
cal/site.json, cal/site.xml; deterministic: sorted, mtime 0, uid / gid 0) and <dir>/plan.json (the public plan: every
dimension, for the calibration page; no friction, no 'passes' / 'does not pass'). The envelope is the family's
(w2gen.envelope.load(): the simulation base's capability sheet, the table every scored site of W2 and W3 is sized
from); --envelope replaces it with a local table (then the generator string says so: not the calibration site).

Layout (4.4 x 5.8 m; the camera rule of the families on the camera rectangle 4.4 x 4.0 m, CAMERA_SEED 9001 as in
version 1.0, so the camera is version 1.0's; floor friction 0.80; x east, y north, origin south-west corner):
  south    (unchanged) a 3.0 x 1.5 m flat floor with visual 0.5 m grid lines and four crosses; the pad; search point
           1; a dust strip
  ridge    (unchanged) y 1.60-2.15, west to east: gap_XN (A), gap_T (B), gap_P (C, open), slot_XL (D, covered,
           clear 0.15), a covered mouth closed at depth 0.30 (E, class B), tunnel_P (F, covered, flat roof, 0.55)
  north    (unchanged up to y 3.0) the roofed room x 2.80-4.40, y 2.15-3.00 behind tunnel_P; a free-standing wall;
           a gravel strip; search point 2; the red crate; six colour chips
  roofed   (new) from the room's north-west corner a roofed passage x 2.80-3.65 northward (0.85 wide, 1.40 long:
           see PASSAGE_LEN), in it one rubble block against its west wall (rubble__passage_block:
           x 2.80 to 2.80 + (0.85 - P), along it 0.25 m centred on the passage's length, floor to roof; the clear
           width beside it = the P class's width = the width of openings C-F); then a roofed chamber x 2.80-4.40 up
           to the site's back wall (its east wall = the site's east wall); one side wall x 2.62-2.80 from the ridge
           to the back wall; east of the passage solid (x 3.65-4.40); ONE roof over the room, passage and chamber at
           the slab clear height of the families (H_ROLL + 0.14). Search point 3 = (3.75, 5.15): 0.65 m from the
           east and the back wall.
  blue crate: version 1.0's place is solid now; it stands flush in the south-east corner (FREE_BLOCK_RULE, CRATES).
Search points 1 and 2 are version 1.0's; weights as every site (read from the acceptance file).
"""
import argparse
import hashlib
import io
import json
import math
import os
import sys
import tarfile

HERE = os.path.dirname(os.path.abspath(__file__))
DEV = os.path.normpath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, os.path.join(DEV, "tools", "sitegen"))

import numpy as np  # noqa: E402

from w2gen import GENERATOR_VERSION as W2_VERSION  # noqa: E402
from w2gen import envelope as envmod  # noqa: E402
from w2gen import family as fam  # noqa: E402
from w2gen import grid as gridmod  # noqa: E402
from w2gen import mjcf, predicates  # noqa: E402
from w3gen import GENERATOR_VERSION as W3_VERSION  # noqa: E402
from w3gen import family as w3fam  # noqa: E402
from w3gen import predicates as w3p  # noqa: E402

CAL_VERSION = "calgen 0.2.0"
LX, LY = 4.4, w3fam.LY                 # 4.4 x 5.8: the W3 site rectangle
LY_CAM = w3fam.LY_CAM                  # 4.0: the camera rule's rectangle (as every W3 site)
CAMERA_SEED = 9001                     # the camera rule's draw for this one site (version 1.0's: the camera is unchanged)
FLOOR_MU = 0.80
PAD = (2.20, 0.45)
SP = {1: (0.95, 1.05), 2: (1.75, 2.70), 3: (3.75, 5.15)}
WHERE = {1: "flat floor", 2: "north of the ridge, open floor", 3: "roofed chamber"}
RIDGE_Y = (1.60, 2.15)
END_W = 0.15                           # west end solid of the ridge
SOLID_W = 0.14                         # solid between two open gaps / an open gap and the covered block
BLIND_DEPTH = 0.30
ROOM = dict(x0=2.80, x1=4.40, y0=2.15, y1=3.00)          # interior; east side = the site's east wall
SIDE_T = 0.18                          # the side wall west of the room, the passage and the chamber (x 2.62-2.80)
PASSAGE_X = (2.80, 3.65)               # 0.85 wide, from the room's north-west corner northward
PASSAGE_LEN = 1.40                     # y 3.00-4.40 (a first layout with 1.20 m was laid before it; the blind route
                                       # measured 3.755 m with either length)
BLOCK_ALONG = 0.25                     # the passage block's length along the passage, centred on its length
FREE_WALL = dict(x0=0.55, x1=2.05, y0=3.20, y1=3.35, h=0.60)
GRAVEL = dict(x0=0.15, x1=1.25, y0=2.40, y1=3.00)        # 1.10 x 0.60
DUST = dict(x0=3.75, x1=4.35, y0=0.30, y1=1.20)          # 0.60 x 0.90 (0.90 x 0.60 turned)
FLAT = dict(x0=0.70, x1=3.70, y0=0.05, y1=1.55)          # 3.0 x 1.5
CROSSES = [(1.20, 0.55), (3.20, 0.55), (1.20, 1.30), (3.20, 1.30)]
# the crates (free blocks, hand-laid). RED is version 1.0's (unchanged with everything at y <= 3.0; it does not meet
# FREE_BLOCK_RULE: 0.15 / 0.20 m from the west / south walls, and is not judged by it). The blue crate of version 1.0
# (x 3.90-4.18, y 3.45-3.75) is inside the solid east of the passage now; BLUE is its version 1.1 place, same size and
# turn: flush in the south-east corner. An exhaustive search (0.01 m grid, both turns, the routes pad -> 1 -> 2 -> 3
# -> pad kept) found exactly two places that pass the rule: this one and its 0.30 x 0.28 turn.
RED = dict(name="free_red", x0=0.15, x1=0.45, y0=0.20, y1=0.48, h=0.30, rgb=fam.CRATE_RGB[0])
BLUE = dict(name="free_blue", x0=4.12, x1=4.40, y0=0.00, y1=0.30, h=0.36, rgb=fam.CRATE_RGB[1])
CRATES = [c for c in (RED, BLUE) if c]
RULED = ("free_blue",)                  # the crates FREE_BLOCK_RULE gates (the ones placed in version 1.1)
FREE_BLOCK_RULE = ("the families' free-block rule (w2gen.family.place_free_blocks / spacing_ok): to every other solid "
                   "(perimeter, ridge, walls, other crates) a gap >= free_block_clearance (W_SWEEP + 0.18) or flush "
                   "(gap 0, no overlap); the centre > r + half the diagonal from the pad (0.45), points 1 and 2 (0.40) "
                   "and 0.25 m before and behind every opening (0.40); on no ground strip; plus, here: not on the flat "
                   "floor (the page says nothing stands on it), inside the camera rectangle, its top seen by the camera")
CHIPS = [("red", "disc", 0.040, (1.55, 0.95)), ("green", "square", 0.050, (2.85, 1.15)),
         ("blue", "disc", 0.055, (0.95, 0.40)), ("red", "square", 0.035, (1.25, 3.70)),
         ("green", "disc", 0.045, (0.40, 2.25)), ("blue", "square", 0.060, (2.60, 0.85))]
LINE_W, LINE_T = 0.012, 0.0008
MARK_GROUP = 2                          # visual only, rendered by the search camera (as the family's chips)
TOL = 1e-6 + 1e-9                       # package coordinates are rounded to 1e-6 m


def r6(x):
    return round(float(x), 6)


def layout(passage_len=PASSAGE_LEN):
    """Rectangles (x0, x1, y0, y1) of the roofed part (room, passage, chamber, side wall, fill, roof, block)."""
    py0 = ROOM["y1"]
    py1 = py0 + passage_len
    passage = dict(x0=PASSAGE_X[0], x1=PASSAGE_X[1], y0=r6(py0), y1=r6(py1))
    chamber = dict(x0=ROOM["x0"], x1=LX, y0=r6(py1), y1=LY)
    side = dict(x0=r6(ROOM["x0"] - SIDE_T), x1=ROOM["x0"], y0=ROOM["y0"], y1=LY)
    fill = dict(x0=PASSAGE_X[1], x1=LX, y0=r6(py0), y1=r6(py1))
    roof = dict(x0=side["x0"], x1=LX, y0=ROOM["y0"], y1=LY)
    ym = 0.5 * (py0 + py1)
    block_y = (r6(ym - BLOCK_ALONG / 2), r6(ym + BLOCK_ALONG / 2))
    return dict(room=dict(ROOM), passage=passage, chamber=chamber, side=side, fill=fill, roof=roof, block_y=block_y)


def ridge(cls):
    """Openings and boxes of the ridge (west to east)."""
    a, y1b = RIDGE_Y
    depth = y1b - a
    P_w, P_h, T_w, XN_w = cls["P"]["w"], cls["P"]["clear_h"], cls["T"]["w"], cls["XN"]["w"]
    SUP = fam.SUP
    boxes, ops = [], []
    x = 0.0

    def solid(name, x0, x1, h=0.70):
        boxes.append(fam._box(name, "rubble", x0, x1, a, y1b, 0.0, h))

    def open_gap(gid, typ, c, w):
        nonlocal x
        ops.append(dict(id=gid, row="R1", type=typ, cls=c, x0=r6(x), x1=r6(x + w), y0=r6(a), y1=r6(y1b),
                        foot=[r6(x), r6(x + w)], covered=False, depth=r6(depth), role="opening"))
        x += w

    solid("ridge_w", 0.0, END_W)
    x = END_W
    open_gap("R1_0", "gap_XN", "XN", XN_w)
    solid("ridge_1", x, x + SOLID_W)
    x += SOLID_W
    open_gap("R1_1", "gap_T", "T", T_w)
    solid("ridge_2", x, x + SOLID_W)
    x += SOLID_W
    open_gap("R1_2", "gap_P", "P", P_w)
    solid("ridge_3", x, x + SOLID_W)
    x += SOLID_W
    # covered block: sup | slot_XL | sup | blind | sup | tunnel_P | sup
    sup_h = P_h
    covered = [("R1_3", "slot_XL", "XL", fam.XL_CLEAR), ("R1_4", "mouth_blind", "B", P_h), ("R1_5", "tunnel_P", "P", P_h)]
    xs = x
    boxes.append(fam._box("sup_0", "support", xs, xs + SUP, a, y1b, 0.0, sup_h))
    x = xs + SUP
    for k, (gid, typ, c, h) in enumerate(covered):
        x0, x1 = x, x + P_w
        f0, f1 = x0 - SUP, x1 + SUP
        op = dict(id=gid, row="R1", type=typ, cls=c, x0=r6(x0), x1=r6(x1), y0=r6(a), y1=r6(y1b), foot=[r6(f0), r6(f1)],
                  covered=True, clear_h=r6(h), clear_h_min=r6(h), roof="flat", roof_t=fam.R1_ROOF_T, depth=r6(depth),
                  role="opening")
        if typ == "slot_XL":
            # the slot's own low supports inside the shared ones would leave nothing to see: its roof spans the
            # opening between the two (taller) shared supports at the low clear height
            boxes.append(fam._box("roof_" + gid, "roof", x0, x1, a - fam.ROOF_OVERHANG, y1b, h, h + fam.R1_ROOF_T))
        else:
            # the covered mouth and the passage share the support between them: each roof covers half of it (no
            # two coplanar roof boxes overlap: they would flicker in the camera's frames)
            r0 = f0 if typ == "mouth_blind" else x0 - SUP / 2
            r1 = x1 + SUP / 2 if typ == "mouth_blind" else f1
            boxes.append(fam._box("roof_" + gid, "roof", r0, r1, a - fam.ROOF_OVERHANG, y1b, h, h + fam.R1_ROOF_T))
        if typ == "mouth_blind":
            boxes.append(fam._box("plug_" + gid, "plug", x0, x1, a + BLIND_DEPTH, y1b, 0.0, h))
            op.update(depth=r6(BLIND_DEPTH), plug_depth_m=r6(BLIND_DEPTH), role="blind mouth")
        ops.append(op)
        x = x1
        boxes.append(fam._box("sup_%d" % (k + 1), "support", x, x + SUP, a, y1b, 0.0, sup_h))
        x += SUP
    solid("ridge_e", x, LX)
    return ops, boxes, dict(end_w=r6(END_W), solid_w=r6(SOLID_W), east_end_w=r6(LX - x))


def roofed(cls, L):
    """The roofed part behind tunnel_P: side wall, fill east of the passage, the passage block, ONE roof (the
    families' slab: underside at the clear height H_ROLL + 0.14, 0.12 thick)."""
    hz = cls["gallery"]["clear_h"]
    P_w = cls["P"]["w"]
    s, f, r, p = L["side"], L["fill"], L["roof"], L["passage"]
    bx1 = r6(p["x0"] + (p["x1"] - p["x0"]) - P_w)          # x 2.80 to 2.80 + (0.85 - P)
    by0, by1 = L["block_y"]
    boxes = [fam._box("side_w", "wall", s["x0"], s["x1"], s["y0"], s["y1"], 0.0, hz),
             fam._box("passage_fill_e", "wall", f["x0"], f["x1"], f["y0"], f["y1"], 0.0, hz),
             fam._box("passage_block", "rubble", p["x0"], bx1, by0, by1, 0.0, hz),
             fam._box("void_slab", "roof", r["x0"], r["x1"], r["y0"], r["y1"], hz, hz + fam.SLAB_T)]
    slab = dict(x0=r6(r["x0"]), x1=r6(r["x1"]), y0=r6(r["y0"]), y1=r6(r["y1"]), z0=r6(hz), z1=r6(hz + fam.SLAB_T))
    block = dict(name="rubble__passage_block", kind="rubble", x0=r6(p["x0"]), x1=bx1, y0=by0, y1=by1, z0=0.0, z1=r6(hz),
                 against="west wall of the passage", across_m=r6(bx1 - p["x0"]), along_m=r6(by1 - by0),
                 gap_east_m=r6(p["x1"] - bx1), gap_rule="clear width beside it = the P class's width (openings C-F)")
    return boxes, slab, block, hz


def visual_marks():
    """0.5 m grid lines on the flat floor and four crosses (visual only, MJCF text, and their public description)."""
    F = FLAT
    L = []
    lines = []
    gx = np.arange(F["x0"], F["x1"] + 1e-9, 0.5)
    gy = np.arange(F["y0"] + 0.45, F["y1"] + 1e-9, 0.5)          # y lines at 0.5, 1.0, 1.5
    z = LINE_T / 2
    for k, xv in enumerate(gx):
        L.append('<geom name="mark__gx%d" type="box" pos="%s %s %s" size="%s %s %s" rgba="0.92 0.92 0.88 1" '
                 'contype="0" conaffinity="0" group="%d"/>' % (k, mjcf._f(xv), mjcf._f((F["y0"] + F["y1"]) / 2), mjcf._f(z),
                                                            mjcf._f(LINE_W / 2), mjcf._f((F["y1"] - F["y0"]) / 2),
                                                            mjcf._f(LINE_T / 2), MARK_GROUP))
        lines.append(dict(axis="x", at=r6(xv), from_=r6(F["y0"]), to=r6(F["y1"])))
    for k, yv in enumerate(gy):
        L.append('<geom name="mark__gy%d" type="box" pos="%s %s %s" size="%s %s %s" rgba="0.92 0.92 0.88 1" '
                 'contype="0" conaffinity="0" group="%d"/>' % (k, mjcf._f((F["x0"] + F["x1"]) / 2), mjcf._f(yv), mjcf._f(z),
                                                            mjcf._f((F["x1"] - F["x0"]) / 2), mjcf._f(LINE_W / 2),
                                                            mjcf._f(LINE_T / 2), MARK_GROUP))
        lines.append(dict(axis="y", at=r6(yv), from_=r6(F["x0"]), to=r6(F["x1"])))
    arm, w = 0.10, 0.02
    for k, (cx, cy) in enumerate(CROSSES):
        for j, (sx, sy) in enumerate(((arm, w / 2), (w / 2, arm))):
            L.append('<geom name="mark__cross%d_%d" type="box" pos="%s %s %s" size="%s %s %s" rgba="0.95 0.85 0.10 1" '
                     'contype="0" conaffinity="0" group="%d"/>' % (k, j, mjcf._f(cx), mjcf._f(cy), mjcf._f(1.5 * LINE_T),
                                                                mjcf._f(sx), mjcf._f(sy), mjcf._f(LINE_T), MARK_GROUP))
    return "\n    ".join(L), dict(grid_lines=lines, line_width_m=LINE_W, crosses=[dict(xy=[r6(a), r6(b)], arm_m=arm,
                                                                                         bar_width_m=w) for a, b in CROSSES])


def chips():
    out = []
    for k, (colour, shape, s, (x, y)) in enumerate(CHIPS):
        out.append(dict(name="chip__%d" % k, shape=shape, size_m=r6(s), thickness_m=0.002, xy=[r6(x), r6(y)], z_m=0.0,
                        yaw_deg=r6(17.0 * k), colour=colour, lightness=1.0, rgb=[r6(c) for c in fam.CAP_RGB[colour]],
                        dist_to_route_m=None))
    return out


def build(env, passage_len=PASSAGE_LEN, crates=None):
    crates = CRATES if crates is None else crates
    cls = w3fam.classes(env)
    L = layout(passage_len)
    ops, rboxes, ridge_info = ridge(cls)
    vboxes, slab, block, hz = roofed(cls, L)
    boxes = []
    walls = []
    for nm, (x0, x1, y0, y1) in dict(s=(-fam.WALL_T, LX + fam.WALL_T, -fam.WALL_T, 0.0),
                                     n=(-fam.WALL_T, LX + fam.WALL_T, LY, LY + fam.WALL_T),
                                     w=(-fam.WALL_T, 0.0, 0.0, LY), e=(LX, LX + fam.WALL_T, 0.0, LY)).items():
        boxes.append(fam._box(nm, "wall", x0, x1, y0, y1, 0.0, fam.WALL_H))
        walls.append("wall__" + nm)
    boxes += rboxes + vboxes
    W = FREE_WALL
    boxes.append(fam._box("free_wall", "wall", W["x0"], W["x1"], W["y0"], W["y1"], 0.0, W["h"]))
    for c in crates:
        boxes.append(fam._box(c["name"], "crate", c["x0"], c["x1"], c["y0"], c["y1"], 0.0, c["h"], rgb=c["rgb"],
                              free_block=True))
    patches = [dict(name="dust", x0=DUST["x0"], x1=DUST["x1"], y0=DUST["y0"], y1=DUST["y1"], mu=fam.PATCH_MU["dust"],
                    thickness=0.002, rgb=list(fam.PATCH_RGB["dust"])),
               dict(name="gravel", x0=GRAVEL["x0"], x1=GRAVEL["x1"], y0=GRAVEL["y0"], y1=GRAVEL["y1"],
                    mu=fam.PATCH_MU["gravel"], thickness=0.002, rgb=list(fam.PATCH_RGB["gravel"]))]
    rng = np.random.default_rng(CAMERA_SEED)
    camera = fam.draw_camera(rng, LX, LY_CAM)
    lighting = dict(key_dir=[r6(q) for q in np.array([0.15, 0.25, -1.0]) / np.linalg.norm([0.15, 0.25, -1.0])],
                    key_off_nominal_deg=0.0, key_diffuse=0.75, ambient=0.35,
                    nominal=dict(key_dir=None, key_diffuse=0.75, ambient=0.35),
                    rule="the family's nominal key light and ambient (the calibration site's lighting does not vary)")
    lighting["nominal"]["key_dir"] = lighting["key_dir"]
    R, P, C = L["room"], L["passage"], L["chamber"]
    runs = dict(room=dict(x0=r6(R["x0"]), x1=r6(R["x1"]), y0=r6(R["y0"]), y1=r6(R["y1"]), width=r6(R["y1"] - R["y0"]),
                          axis="x"),
                passage=dict(x0=r6(P["x0"]), x1=r6(P["x1"]), y0=r6(P["y0"]), y1=r6(P["y1"]), width=r6(P["x1"] - P["x0"]),
                             axis="y"))
    plug_op = [o for o in ops if o["cls"] == "B"][0]
    weights = w3fam.WEIGHTS
    gen = "%s (parts of %s, %s)" % (CAL_VERSION, W2_VERSION, W3_VERSION)
    if abs(passage_len - PASSAGE_LEN) > 1e-9:
        gen += " [passage length %.2f m: not the calibration site]" % passage_len
    site = dict(
        family="CAL", generator=gen, seed=0, kind="cal",
        frame="x east, y north, z up, origin = south-west floor corner; metres",
        size_m=[LX, LY], camera_depth_m=LY_CAM, body_length_m=fam.BL, envelope=env, classes=cls,
        params=dict(ridge_y=[RIDGE_Y[0], RIDGE_Y[1]], slack_R1_m=0.0, slack_R2_m=0.0, y2=RIDGE_Y[0], **ridge_info),
        bands=dict(south=[0.0, RIDGE_Y[0]], ridge=list(RIDGE_Y), north=[RIDGE_Y[1], LY]),
        r1=dict(order=[o["type"] for o in ops], third="all", tunnel_roof="flat", openings=ops),
        r2=dict(order=[], decoy_type=None, openings=[]),
        void=dict(sp3_mode="chamber", true_mouth="R1_5", blind_mouth="R1_4", runs=runs,
                  chamber=dict(x0=r6(C["x0"]), x1=r6(C["x1"]), y0=r6(C["y0"]), y1=r6(C["y1"])), pockets=[],
                  gallery=dict(x0=r6(R["x0"]), x1=r6(R["x1"]), y0=r6(R["y0"]), y1=r6(R["y1"]), clear_h=r6(hz),
                               note="= the roofed room (legacy key)"),
                  slab=slab, passage_block=block,
                  plug=dict(x0=plug_op["x0"], x1=plug_op["x1"], y0=r6(RIDGE_Y[0] + BLIND_DEPTH), y1=r6(RIDGE_Y[1]),
                            z1=r6(cls["P"]["clear_h"])),
                  note="the calibration site's roofed part behind tunnel_P (room -> passage -> chamber, one roof) "
                       "stands in the void's place (truth fields named as in W3: runs room and passage, chamber C, "
                       "slab = the one roof; gallery = the room, legacy key; params.y2 = the ridge's south face)"),
        pad=dict(xy=[PAD[0], PAD[1]], retrieval_radius_m=0.60, setdown_error_m=0.10),
        search_points=[dict(id=i, xy=[SP[i][0], SP[i][1]], weight=weights[i], where=WHERE[i]) for i in (1, 2, 3)],
        patches=patches, boxes=boxes, walls=walls, camera=camera, lighting=lighting,
        physics=dict(floor_mu=FLOOR_MU, box_mu=fam.BOX_MU, patch_mu=dict(fam.PATCH_MU),
                     floor_mu_rule="the calibration site's floor: 0.80"),
        gravel_relief=dict(enabled=False, grid_m=0.01, wavelength_m=[0.03, 0.08], rms_m=0.008, peak_max_m=0.025,
                           edge_taper_m=0.05, heights="off", rng_stream=None),
        free_blocks=dict(placed=len(crates), dropped_for_G9=0, kept=["crate__" + c["name"] for c in crates],
                         rule="hand-laid; " + FREE_BLOCK_RULE + " (judged for: %s)" % ", ".join(RULED),
                         moved=(["crate__free_blue: from x 3.90-4.18, y 3.45-3.75 (version 1.0; solid now) to the "
                                 "south-east corner"] if BLUE else []),
                         removed=([] if BLUE else ["the blue crate of version 1.0 (its place is solid now; no place "
                                                   "passes the rule)"])),
        chips=chips(), chip_rule="six chips, hand-laid, visual only",
        room=dict(interior=dict(R), wall_t=SIDE_T, clear_h=r6(hz), slab_t=fam.SLAB_T),
        passage=dict(interior=dict(P), width_m=r6(P["x1"] - P["x0"]), length_m=r6(P["y1"] - P["y0"]), clear_h=r6(hz),
                     block=block),
        chamber=dict(interior=dict(C), size_m=[r6(C["x1"] - C["x0"]), r6(C["y1"] - C["y0"])], clear_h=r6(hz)),
        side_wall=dict(L["side"], t=SIDE_T, h=r6(hz)),
        passage_fill=dict(L["fill"], h=r6(hz)),
        free_wall=dict(FREE_WALL),
    )
    marks_xml, marks = visual_marks()
    site["marks"] = marks
    return site, marks_xml


def site_xml(site, marks_xml):
    """The family's MJCF (mjcf.world_fragment) plus the calibration marks (visual only)."""
    lt = site["lighting"]
    a = lt["ambient"]
    head = ['<mujoco model="cal">', '  <visual>', '    <global offwidth="640" offheight="480"/>',
            '    <quality shadowsize="4096"/>',
            '    <headlight ambient="%s" diffuse="0 0 0" specular="0 0 0"/>' % mjcf._v([a, a, a]),
            '    <map znear="0.01" zfar="50"/>', '  </visual>']
    body = ['  <worldbody>', mjcf.world_fragment(site, extra=marks_xml), '  </worldbody>', '</mujoco>']
    return "\n".join(head + body) + "\n"


# ------------------------------------------------------------------------------------------------ predicates
def _rect_pt(r, p):
    return math.hypot(max(r[0] - p[0], 0.0, p[0] - r[1]), max(r[2] - p[1], 0.0, p[1] - r[3]))


def _solids(site, skip=()):
    return [(b["name"], tuple(b["aabb"][:4])) for b in site["boxes"]
            if b["kind"] in gridmod.SOLID_KINDS and b["name"] not in skip]


def crate_rule(site, c, m=None, d=None, cpos=None):
    """FREE_BLOCK_RULE for one crate (box dict): the families' spacing rule and keep-out discs, ground strips, the flat
    floor, the camera rectangle and the camera's view of its top."""
    cls = site["classes"]
    clr = cls["free_block_clearance"]
    a = c["aabb"]
    rect = (a[0], a[1], a[2], a[3])
    w, dd = a[1] - a[0], a[3] - a[2]
    cx, cy = 0.5 * (a[0] + a[1]), 0.5 * (a[2] + a[3])
    pad = site["pad"]["xy"]
    sp = {s["id"]: s["xy"] for s in site["search_points"]}
    keep = [("pad", pad, 0.45), ("point 1", sp[1], 0.40), ("point 2", sp[2], 0.40)]
    for row in ("r1", "r2"):
        for g in site[row]["openings"]:
            gx = 0.5 * (g["x0"] + g["x1"])
            keep += [(g["id"] + " south", (gx, g["y0"] - 0.25), 0.40), (g["id"] + " north", (gx, g["y1"] + 0.25), 0.40)]
    lx, ly = site["size_m"]
    T = fam.WALL_T
    perim = [("perimeter w", (-T, 0.0, -T, ly + T)), ("perimeter e", (lx, lx + T, -T, ly + T)),
             ("perimeter s", (-T, lx + T, -T, 0.0)), ("perimeter n", (-T, lx + T, ly, ly + T))]
    others = perim + [(nm, r) for nm, r in _solids(site, skip=(c["name"], "wall__w", "wall__e", "wall__s", "wall__n"))]
    bad_gap = []
    for nm, o in others:
        if fam._rect_overlap(rect, o):
            bad_gap.append((nm, "overlap"))
            continue
        gp = fam._rect_gap(rect, o)
        if 0.0 < gp < clr - 1e-9:
            bad_gap.append((nm, round(gp, 4)))
    bad_keep = [nm for nm, p, r in keep if not math.hypot(cx - p[0], cy - p[1]) > r + 0.5 * math.hypot(w, dd)]
    on_patch = [p["name"] for p in site["patches"] if fam._rect_overlap(rect, (p["x0"], p["x1"], p["y0"], p["y1"]))]
    on_flat = bool(fam._rect_overlap(rect, (FLAT["x0"], FLAT["x1"], FLAT["y0"], FLAT["y1"])))
    in_cam_rect = bool(a[0] >= 0 and a[1] <= lx and a[2] >= 0 and a[3] <= LY_CAM)
    seen = None
    if m is not None:
        xs = np.linspace(a[0] + 0.02, a[1] - 0.02, 5)
        ys = np.linspace(a[2] + 0.02, a[3] - 0.02, 5)
        P = np.array([(x, y, a[5] + 0.001) for x in xs for y in ys])
        seen = round(float(predicates.visible(m, d, cpos, P).mean()), 4)
    ok = (not bad_gap and not bad_keep and not on_patch and not on_flat and in_cam_rect
          and (seen is None or seen >= 0.90))
    return dict(pass_=bool(ok), name=c["name"], rect=[r6(q) for q in rect], gap_violations=bad_gap,
                keep_out_violations=bad_keep, on_strip=on_patch, on_flat_floor=on_flat, in_camera_rectangle=in_cam_rect,
                top_seen_fraction=seen, clearance_m=clr)


def blind_route(site, with_block=True):
    """The families' blind route (w3gen: grid shortest path, disc of the plan radius, exact clearance) from the true
    mouth (tunnel_P = opening F) at (its centre, y2 + 0.10) to search point 3, over y >= y2 - 0.30."""
    F = [o for o in site["r1"]["openings"] if o["id"] == site["void"]["true_mouth"]][0]
    y2 = site["params"]["y2"]
    skip = () if with_block else ("rubble__passage_block",)
    rects = [r for _nm, r in _solids(site, skip=skip)]
    # covered openings too low for the robot are solid (gridmod.solid_footprints' rule)
    need = site["envelope"]["values"]["H_ROLL"] + 0.02
    rects += [(g["x0"], g["x1"], g["y0"], g["y1"]) for g in site["r1"]["openings"]
              if g["covered"] and g["clear_h_min"] < need]
    sp3 = [s["xy"] for s in site["search_points"] if s["id"] == 3][0]
    start = (0.5 * (F["x0"] + F["x1"]), y2 + 0.10)
    L, path, _d = w3fam._route(rects, site["size_m"][0], site["size_m"][1], start, tuple(sp3),
                              site["classes"]["plan_radius"], y2 - 0.30)
    return (None if L is None else round(L, 3)), path, [r6(start[0]), r6(start[1])]


def qc(site, xml, render=True):
    """G1 (compiles), G2 (corner margin on the camera rectangle), G3 (projection; rendered), routes pad -> 1 -> 2 ->
    3 -> pad at the families' plan radius (as G9a), the roofed part unseen (room, passage, chamber, point 3's disc and
    the passage block: no sampled point visible, no camera ray hits the block or ends inside them), pad and points
    1, 2 seen, the blind route from opening F to point 3 (window of W3's G8b), the passage block's dimensions (gap =
    the P class = openings C-F, floor to roof, centred, away from the passage's corners), point 3's clearance (W3's
    SP3_CLEAR) and the crates (FREE_BLOCK_RULE)."""
    q = {}
    m, d = predicates.compile_world(xml)
    q["G1"] = dict(pass_=True, compiles=True, ngeom=int(m.ngeom))
    cpos, R = predicates.cam_pose(site)
    margin, _u, _v = fam.corner_margin(cpos, R, LX, LY_CAM, site["camera"]["model"])
    q["G2"] = dict(pass_=bool(margin >= 12.0), corner_margin_px=round(margin, 2), threshold_px=12.0,
                   rectangle="[0, LX] x [0, LY_CAM]")
    if render:
        q["G3"] = w3p.g3_projection(site)
    else:
        q["G3"] = dict(pass_=None, note="not rendered")
    # routes
    G = gridmod.Grid(site, 0.02)
    ok = G.clearance(gridmod.solid_footprints(site)) >= site["classes"]["plan_radius"]
    pts = predicates._points(site)
    legs = G.legs(ok, pts, ("pad", 1, 2, 3, "pad"))
    q["routes"] = dict(pass_=all(l["length_m"] is not None for l in legs), radius_m=site["classes"]["plan_radius"],
                       legs=[dict(leg="%s->%s" % (l["a"], l["b"]), length_m=l["length_m"]) for l in legs])
    # the roofed part unseen
    v = site["void"]
    hz = v["slab"]["z0"]
    rects = [("room", v["runs"]["room"]), ("passage", v["runs"]["passage"]), ("chamber", v["chamber"])]
    blk = v["passage_block"]
    vis_rows = {}
    n_vis = 0
    for name, r in rects:
        xs = np.arange(r["x0"] + 0.025, r["x1"] - 0.02, 0.05)
        ys = np.arange(r["y0"] + 0.025, r["y1"] - 0.02, 0.05)
        P = np.array([(x, y, z) for x in xs for y in ys for z in (0.05, 0.20, 0.30)
                      if not (blk["x0"] - 1e-9 <= x <= blk["x1"] + 1e-9 and blk["y0"] - 1e-9 <= y <= blk["y1"] + 1e-9)])
        vv = predicates.visible(m, d, cpos, P)
        vis_rows[name] = dict(points=int(len(P)), visible=int(vv.sum()))
        n_vis += int(vv.sum())
    sp3 = pts[3]
    P3 = []
    for rr in (0.0, 0.10, 0.20, 0.30):
        for aa in np.linspace(0, 2 * math.pi, 12, endpoint=False):
            for z in (0.05, 0.20, 0.30):
                P3.append((sp3[0] + rr * math.cos(aa), sp3[1] + rr * math.sin(aa), z))
    v3 = predicates.visible(m, d, cpos, np.array(P3))
    vis_rows["point3_disc_0p30"] = dict(points=len(P3), visible=int(v3.sum()))
    n_vis += int(v3.sum())
    # the block's free faces (east, south, north), 1 mm outside
    zs = np.linspace(0.02, hz - 0.02, 6)
    FB = [(blk["x1"] + 0.001, y, z) for y in np.linspace(blk["y0"] + 0.01, blk["y1"] - 0.01, 6) for z in zs]
    FB += [(x, yy, z) for x in np.linspace(blk["x0"] + 0.01, blk["x1"] - 0.01, 6) for yy in (blk["y0"] - 0.001, blk["y1"] + 0.001)
           for z in zs]
    vb = predicates.visible(m, d, cpos, np.array(FB))
    vis_rows["block_faces"] = dict(points=len(FB), visible=int(vb.sum()))
    n_vis += int(vb.sum())
    gid, _dist, hit, _shape = predicates.cast_image(m, d, site, ss=2)
    block_ids = predicates._geom_ids(m, "rubble__passage_block")
    n_block_rays = int(np.isin(gid, block_ids).sum())
    good = gid >= 0
    inside = np.zeros(len(gid), bool)
    for _name, r in rects:
        inside |= good & (hit[:, 0] > r["x0"] + 1e-6) & (hit[:, 0] < r["x1"] - 1e-6) & (hit[:, 1] > r["y0"] + 1e-6) \
            & (hit[:, 1] < r["y1"] - 1e-6) & (hit[:, 2] < hz - 1e-6)
    n_inside = int(inside.sum())
    q["roofed_unseen"] = dict(pass_=bool(n_vis == 0 and n_block_rays == 0 and n_inside == 0), sampled=vis_rows,
                              camera_rays=int(len(gid)), rays_on_block=n_block_rays, rays_ending_inside=n_inside,
                              what="2 x 2 supersampled camera rays; points every 0.05 m at 0.05 / 0.20 / 0.30 m")
    # pad, points 1 and 2 seen
    P1 = []
    for key in ("pad", 1, 2):
        cx, cy = pts[key]
        for rr in (0.0, 0.15, 0.30):
            for aa in np.linspace(0, 2 * math.pi, 8, endpoint=False):
                for z in (0.05, 0.20):
                    P1.append((cx + rr * math.cos(aa), cy + rr * math.sin(aa), z))
    v1 = predicates.visible(m, d, cpos, np.array(P1))
    q["open_points_seen"] = dict(pass_=bool(v1.mean() >= 0.90), visible_fraction=round(float(v1.mean()), 4),
                                 threshold=0.90, what="pad and search points 1, 2: discs of 0.30 m at 0.05 and 0.20 m")
    # the blind route from F's south mouth to point 3
    Lb, path_b, start = blind_route(site, True)
    Ln, _p, _s = blind_route(site, False)
    win = w3fam.ROUTE_WIN
    q["blind_route"] = dict(pass_=bool(Lb is not None and win[0] <= Lb <= win[1]), window_m=list(win), start=start,
                            with_block_m=Lb, without_block_m=Ln,
                            rule="w3gen._route: 0.02 m grid, disc of the plan radius, from (opening F's centre, y2 + "
                                 "0.10), y2 = the ridge's south face, to point 3")
    if path_b is not None:
        pa = np.asarray(path_b)
        sel = (pa[:, 1] >= blk["y0"]) & (pa[:, 1] <= blk["y1"])
        q["blind_route"]["x_beside_block"] = [round(float(pa[sel, 0].min()), 3), round(float(pa[sel, 0].max()), 3)] \
            if sel.any() else None
    # the passage block
    P_w = site["classes"]["P"]["w"]
    pas = v["runs"]["passage"]
    cd = [o for o in site["r1"]["openings"] if o["id"] in ("R1_2", "R1_3", "R1_4", "R1_5")]
    widths = sorted({r6(o["x1"] - o["x0"]) for o in cd})
    gap = r6(pas["x1"] - blk["x1"])
    corners = [(pas["x1"], pas["y0"]), (pas["x1"], pas["y1"])]
    dcorn = [round(_rect_pt((blk["x0"], blk["x1"], blk["y0"], blk["y1"]), c), 4) for c in corners]
    others = [(nm, r) for nm, r in _solids(site, skip=("rubble__passage_block",))
              if fam._rect_overlap(r, (blk["x1"], pas["x1"], blk["y0"], blk["y1"]))]
    centred = abs(0.5 * (blk["y0"] + blk["y1"]) - 0.5 * (pas["y0"] + pas["y1"])) <= TOL
    against_w = abs(blk["x0"] - pas["x0"]) <= TOL
    floor_roof = abs(blk["z0"]) <= TOL and abs(blk["z1"] - hz) <= TOL
    q["passage_block"] = dict(
        pass_=bool(abs(gap - P_w) <= TOL and widths == [r6(P_w)] and centred and against_w and floor_roof
                   and min(dcorn) >= 0.5 - TOL and not others and abs(blk["y1"] - blk["y0"] - BLOCK_ALONG) <= TOL
                   and n_block_rays == 0),
        gap_east_m=gap, P_w=r6(P_w), widths_C_to_F=widths, centred_on_passage=bool(centred), against_west_wall=bool(against_w),
        floor_to_roof=bool(floor_roof), d_corners_east_wall_m=dcorn, corners=[list(c) for c in corners],
        solids_in_gap=[nm for nm, _r in others], camera_rays=n_block_rays)
    # point 3's clearance (W3's rule)
    sol = [r for _nm, r in _solids(site)]
    clr = w3fam._clearance_at(sol, sp3, LX, LY)
    q["sp3_clearance"] = dict(pass_=bool(clr >= w3fam.SP3_CLEAR - 1e-9), clearance_m=round(clr, 4),
                              threshold_m=w3fam.SP3_CLEAR,
                              d_walls_m=dict(east=r6(LX - sp3[0]), back=r6(LY - sp3[1]),
                                             chamber_west=r6(sp3[0] - v["chamber"]["x0"]),
                                             chamber_south=r6(sp3[1] - v["chamber"]["y0"])))
    # crates
    rows = []
    for b in site["boxes"]:
        if b["kind"] == "crate":
            r = crate_rule(site, b, m, d, cpos)
            r["gated"] = any(b["name"].endswith(k) for k in RULED)
            rows.append(r)
    q["crates"] = dict(pass_=all(r["pass_"] for r in rows if r["gated"]), rows=rows, rule=FREE_BLOCK_RULE,
                       gated=list(RULED), note="the red crate is version 1.0's, unchanged; it is not judged by the rule")
    q["pass"] = bool(all(v_.get("pass_") is not False for k, v_ in q.items() if isinstance(v_, dict)))
    q["failed"] = [k for k, v_ in q.items() if isinstance(v_, dict) and v_.get("pass_") is False]
    return q


# ------------------------------------------------------------------------------------------------ public plan
def public_plan(site):
    """Every dimension of the calibration site (the calibration page and its figure), nothing else."""
    def rect(b):
        return dict(x0=b["x0"], x1=b["x1"], y0=b["y0"], y1=b["y1"])
    ops = []
    label = "ABCDEF"
    desc = {"gap_XN": "open gap", "gap_T": "open gap", "gap_P": "open gap", "slot_XL": "covered slot",
            "mouth_blind": "covered mouth with a back wall", "tunnel_P": "covered passage"}
    for k, o in enumerate(site["r1"]["openings"]):
        e = dict(label="opening %s" % label[k], kind=desc[o["type"]], x0=o["x0"], x1=o["x1"], width_m=r6(o["x1"] - o["x0"]),
                 y0=o["y0"], y1=o["y1"])
        if o["covered"]:
            e.update(clear_height_m=o["clear_h"], roof_thickness_m=o["roof_t"])
        if o["type"] == "mouth_blind":
            e.update(depth_to_back_wall_m=o["plug_depth_m"])
        else:
            e.update(length_m=o["depth"])
        if o["type"] == "tunnel_P":
            e.update(leads_to="the roofed room")
        ops.append(e)
    solids = [dict(name=b["name"], kind=b["kind"], x0=b["aabb"][0], x1=b["aabb"][1], y0=b["aabb"][2], y1=b["aabb"][3],
                   height_m=r6(b["aabb"][5])) for b in site["boxes"]
              if (b["kind"] in ("rubble", "support") and b["aabb"][2] == RIDGE_Y[0] and b["aabb"][3] == RIDGE_Y[1])]
    R, P, C = site["room"], site["passage"], site["chamber"]
    blk = P["block"]
    sw, pf, sl = site["side_wall"], site["passage_fill"], site["void"]["slab"]
    plan = dict(
        what="the calibration site of the practice ground: every dimension (metres, site frame: x east, y north, "
             "origin = south-west floor corner)",
        size_m=site["size_m"], perimeter_wall=dict(thickness_m=fam.WALL_T, height_m=fam.WALL_H),
        flat_floor=dict(rect(FLAT), grid_lines=site["marks"]["grid_lines"], line_width_m=site["marks"]["line_width_m"],
                        crosses=site["marks"]["crosses"]),
        pad=dict(xy=site["pad"]["xy"], retrieval_radius_m=site["pad"]["retrieval_radius_m"]),
        search_points=[dict(id=p["id"], xy=p["xy"], where=p["where"]) for p in site["search_points"]],
        ridge=dict(y0=RIDGE_Y[0], y1=RIDGE_Y[1], openings=ops, solids=solids),
        roofed_room=dict(interior=rect(R["interior"]),
                         size_m=[r6(R["interior"]["x1"] - R["interior"]["x0"]), r6(R["interior"]["y1"] - R["interior"]["y0"])],
                         clear_height_m=R["clear_h"], roof_thickness_m=R["slab_t"], wall_thickness_m=R["wall_t"],
                         entrance="opening F (covered passage) on its south side",
                         exit="the roofed passage, from its north-west corner",
                         sides=dict(west="the side wall", east="the site's east wall",
                                    north="the roofed passage (x %s to %s) and the solid east of it (x %s to %s)"
                                          % (P["interior"]["x0"], P["interior"]["x1"], pf["x0"], pf["x1"]),
                                    south="the ridge")),
        roofed_passage=dict(interior=rect(P["interior"]), width_m=P["width_m"], length_m=P["length_m"],
                            clear_height_m=P["clear_h"], leads_from="the roofed room's north-west corner",
                            leads_to="the roofed chamber",
                            sides=dict(west="the side wall", east="the solid east of the passage"),
                            block=dict(kind="rubble block", x0=blk["x0"], x1=blk["x1"], y0=blk["y0"], y1=blk["y1"],
                                       width_m=blk["across_m"], length_m=blk["along_m"], height_m=blk["z1"],
                                       stands="on the floor against the passage's west wall, up to the roof's underside",
                                       clear_width_beside_m=blk["gap_east_m"], beside="east of it")),
        roofed_chamber=dict(interior=rect(C["interior"]), size_m=C["size_m"], clear_height_m=C["clear_h"],
                            sides=dict(west="the side wall", east="the site's east wall", north="the site's back wall",
                                       south="the roofed passage (x %s to %s) and the solid east of it (x %s to %s)"
                                             % (P["interior"]["x0"], P["interior"]["x1"], pf["x0"], pf["x1"]))),
        side_wall=dict(x0=sw["x0"], x1=sw["x1"], y0=sw["y0"], y1=sw["y1"], thickness_m=sw["t"], height_m=sw["h"],
                       note="west of the room, the passage and the chamber; up to the roof's underside"),
        solid_east_of_passage=dict(x0=pf["x0"], x1=pf["x1"], y0=pf["y0"], y1=pf["y1"], height_m=pf["h"],
                                   note="solid up to the roof's underside, from the passage to the site's east wall"),
        roof=dict(x0=sl["x0"], x1=sl["x1"], y0=sl["y0"], y1=sl["y1"], underside_height_m=sl["z0"],
                  thickness_m=r6(sl["z1"] - sl["z0"]), covers="the roofed room, the roofed passage and the roofed chamber"),
        free_standing_wall=dict(x0=FREE_WALL["x0"], x1=FREE_WALL["x1"], y0=FREE_WALL["y0"], y1=FREE_WALL["y1"],
                                height_m=FREE_WALL["h"], length_m=r6(FREE_WALL["x1"] - FREE_WALL["x0"])),
        strips=[dict(name="gravel strip", **rect(GRAVEL)), dict(name="dust strip", **rect(DUST))],
        crates=[dict(colour=b["name"].split("_")[-1], x0=b["aabb"][0], x1=b["aabb"][1], y0=b["aabb"][2], y1=b["aabb"][3],
                     height_m=b["aabb"][5]) for b in site["boxes"] if b["kind"] == "crate"],
        chips=[dict(colour=c["colour"], shape=c["shape"], size_m=c["size_m"], xy=c["xy"]) for c in site["chips"]],
        time_limit_s=None,
        note="dimensions only; the page built from this file states no 'passes' or 'does not pass'",
    )
    return plan


def plan_vs_site(plan, site):
    """Every number of the plan against the site package (boxes by name, openings, points, patches, crates);
    (number of comparisons, list of mismatches)."""
    B = {b["name"]: b["aabb"] for b in site["boxes"]}
    out, n = [], [0]

    def eq(what, a, b):
        n[0] += 1
        if abs(float(a) - float(b)) > TOL:
            out.append("%s: plan %r, site %r" % (what, a, b))

    def box(what, pr, name, h_key=None):
        a = B[name]
        for k, i in (("x0", 0), ("x1", 1), ("y0", 2), ("y1", 3)):
            eq("%s.%s" % (what, k), pr[k], a[i])
        if h_key:
            eq("%s.%s" % (what, h_key), pr[h_key], a[5])
    eq("size_m.x", plan["size_m"][0], site["size_m"][0])
    eq("size_m.y", plan["size_m"][1], site["size_m"][1])
    eq("perimeter.t", plan["perimeter_wall"]["thickness_m"], B["wall__w"][1] - B["wall__w"][0])
    eq("perimeter.h", plan["perimeter_wall"]["height_m"], B["wall__w"][5])
    eq("pad.x", plan["pad"]["xy"][0], site["pad"]["xy"][0])
    eq("pad.y", plan["pad"]["xy"][1], site["pad"]["xy"][1])
    eq("pad.r", plan["pad"]["retrieval_radius_m"], site["pad"]["retrieval_radius_m"])
    for p, s in zip(plan["search_points"], site["search_points"]):
        eq("sp%d.x" % s["id"], p["xy"][0], s["xy"][0])
        eq("sp%d.y" % s["id"], p["xy"][1], s["xy"][1])
    for o, g in zip(plan["ridge"]["openings"], site["r1"]["openings"]):
        for k in ("x0", "x1", "y0", "y1"):
            eq("%s.%s" % (o["label"], k), o[k], g[k])
        eq("%s.width" % o["label"], o["width_m"], g["x1"] - g["x0"])
        if "clear_height_m" in o:
            eq("%s.clear_h" % o["label"], o["clear_height_m"], g["clear_h"])
            roof = B["roof__roof_" + g["id"]]
            eq("%s.roof_t" % o["label"], o["roof_thickness_m"], roof[5] - roof[4])
            eq("%s.roof_z0" % o["label"], o["clear_height_m"], roof[4])
        if "depth_to_back_wall_m" in o:
            eq("%s.depth" % o["label"], o["y0"] + o["depth_to_back_wall_m"], B["plug__plug_" + g["id"]][2])
    for s in plan["ridge"]["solids"]:
        box(s["name"], s, s["name"], "height_m")
    hz = B["roof__void_slab"][4]
    rm, pa, ch = plan["roofed_room"], plan["roofed_passage"], plan["roofed_chamber"]
    for k in ("x0", "x1", "y0", "y1"):
        eq("room." + k, rm["interior"][k], site["void"]["runs"]["room"][k])
        eq("passage." + k, pa["interior"][k], site["void"]["runs"]["passage"][k])
        eq("chamber." + k, ch["interior"][k], site["void"]["chamber"][k])
    eq("room.size_x", rm["size_m"][0], rm["interior"]["x1"] - rm["interior"]["x0"])
    eq("room.size_y", rm["size_m"][1], rm["interior"]["y1"] - rm["interior"]["y0"])
    eq("chamber.size_x", ch["size_m"][0], ch["interior"]["x1"] - ch["interior"]["x0"])
    eq("chamber.size_y", ch["size_m"][1], ch["interior"]["y1"] - ch["interior"]["y0"])
    eq("passage.width", pa["width_m"], pa["interior"]["x1"] - pa["interior"]["x0"])
    eq("passage.length", pa["length_m"], pa["interior"]["y1"] - pa["interior"]["y0"])
    for nm, x in (("room", rm), ("passage", pa), ("chamber", ch)):
        eq(nm + ".clear_h", x["clear_height_m"], hz)
    eq("room.roof_t", rm["roof_thickness_m"], B["roof__void_slab"][5] - B["roof__void_slab"][4])
    eq("room.wall_t", rm["wall_thickness_m"], B["wall__side_w"][1] - B["wall__side_w"][0])
    # the side walls of the roofed part are really where the plan says: west faces, east faces
    eq("room.west_face", rm["interior"]["x0"], B["wall__side_w"][1])
    eq("passage.west_face", pa["interior"]["x0"], B["wall__side_w"][1])
    eq("chamber.west_face", ch["interior"]["x0"], B["wall__side_w"][1])
    eq("passage.east_face", pa["interior"]["x1"], B["wall__passage_fill_e"][0])
    eq("chamber.south_face_east", ch["interior"]["y0"], B["wall__passage_fill_e"][3])
    eq("room.north_face_east", rm["interior"]["y1"], B["wall__passage_fill_e"][2])
    eq("chamber.east_face", ch["interior"]["x1"], B["wall__e"][0])
    eq("chamber.north_face", ch["interior"]["y1"], B["wall__n"][2])
    eq("room.east_face", rm["interior"]["x1"], B["wall__e"][0])
    blk = pa["block"]
    box("passage block", blk, "rubble__passage_block", "height_m")
    eq("block.width", blk["width_m"], blk["x1"] - blk["x0"])
    eq("block.length", blk["length_m"], blk["y1"] - blk["y0"])
    eq("block.gap", blk["clear_width_beside_m"], B["wall__passage_fill_e"][0] - B["rubble__passage_block"][1])
    eq("block.gap = P", blk["clear_width_beside_m"], site["classes"]["P"]["w"])
    eq("block.height = roof underside", blk["height_m"], hz)
    box("side wall", plan["side_wall"], "wall__side_w", "height_m")
    eq("side wall.t", plan["side_wall"]["thickness_m"], B["wall__side_w"][1] - B["wall__side_w"][0])
    box("solid east of passage", plan["solid_east_of_passage"], "wall__passage_fill_e", "height_m")
    box("roof", plan["roof"], "roof__void_slab")
    eq("roof.underside", plan["roof"]["underside_height_m"], hz)
    eq("roof.t", plan["roof"]["thickness_m"], B["roof__void_slab"][5] - B["roof__void_slab"][4])
    fw = plan["free_standing_wall"]
    box("free wall", fw, "wall__free_wall", "height_m")
    eq("free wall.length", fw["length_m"], fw["x1"] - fw["x0"])
    pt = {p["name"]: p for p in site["patches"]}
    for s in plan["strips"]:
        p = pt[s["name"].split()[0]]
        for k in ("x0", "x1", "y0", "y1"):
            eq("%s.%s" % (s["name"], k), s[k], p[k])
    for c in plan["crates"]:
        box("crate " + c["colour"], c, "crate__free_" + c["colour"], "height_m")
    if len(plan["crates"]) != sum(1 for b in site["boxes"] if b["kind"] == "crate"):
        out.append("crates: plan %d, site %d" % (len(plan["crates"]), sum(1 for b in site["boxes"] if b["kind"] == "crate")))
    for c, s in zip(plan["chips"], site["chips"]):
        eq("chip.x", c["xy"][0], s["xy"][0])
        eq("chip.y", c["xy"][1], s["xy"][1])
        eq("chip.size", c["size_m"], s["size_m"])
    fl = plan["flat_floor"]
    for k in ("x0", "x1", "y0", "y1"):
        eq("flat." + k, fl[k], FLAT[k])
    return n[0], out


# ------------------------------------------------------------------------------------------------ files
def _clean(o):
    if isinstance(o, dict):
        return {(k[:-1] if k.endswith("_") and k != "pass_" else k): _clean(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [_clean(v) for v in o]
    if isinstance(o, (np.floating,)):
        return float(o)
    if isinstance(o, (np.integer,)):
        return int(o)
    if isinstance(o, np.bool_):
        return bool(o)
    if isinstance(o, np.ndarray):
        return _clean(o.tolist())
    return o


def dumps(o):
    return json.dumps(_clean(o), indent=1, ensure_ascii=True) + "\n"


def write_archive(path, members):
    """A deterministic .tar.xz of {name: bytes} (sorted, mtime 0, uid / gid 0, mode 0644)."""
    buf = io.BytesIO()
    with tarfile.open(fileobj=buf, mode="w", format=tarfile.PAX_FORMAT) as tf:
        for name in sorted(members):
            data = members[name]
            ti = tarfile.TarInfo("cal/" + name)
            ti.size, ti.mtime, ti.mode, ti.uid, ti.gid, ti.uname, ti.gname = len(data), 0, 0o644, 0, 0, "", ""
            tf.addfile(ti, io.BytesIO(data))
    import lzma
    raw = lzma.compress(buf.getvalue(), format=lzma.FORMAT_XZ, check=lzma.CHECK_CRC64, preset=6)
    with open(path, "wb") as fh:
        fh.write(raw)
    return hashlib.sha256(raw).hexdigest()


def load_envelope(path=None):
    """The families' envelope (capability sheet), or a local table (--envelope)."""
    if path:
        env = envmod.load(cap_path=None, local_path=path)
        bad = [k for k, s in env["source"].items() if s != "capability_local"]
    else:
        env = envmod.load()
        bad = [k for k, s in env["source"].items() if s != "capability_sheet"]
    if bad:
        raise SystemExit("envelope keys not from the expected table: %s" % bad)
    # no key uses the provisional values: their provenance note (development paths) does not go into the package
    env = dict(env)
    env.pop("provisional_source", None)
    return env


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--envelope", default=None, help="a local capability table instead of the families' sheet")
    ap.add_argument("--passage-length", type=float, default=PASSAGE_LEN)
    ap.add_argument("--no-render", action="store_true")
    a = ap.parse_args(argv)
    env = load_envelope(a.envelope)
    site, marks_xml = build(env, passage_len=a.passage_length)
    if a.envelope:
        site["generator"] += " [local envelope table: not the calibration site]"
    xml = site_xml(site, marks_xml)
    q = qc(site, xml, render=not a.no_render)
    plan = public_plan(site)
    n_cmp, bad = plan_vs_site(plan, site)
    q["plan_vs_site"] = dict(pass_=not bad, compared=n_cmp, mismatches=bad)
    q["pass"] = bool(q["pass"] and not bad)
    q["failed"] = [k for k, v in q.items() if isinstance(v, dict) and v.get("pass_") is False]
    d = os.path.join(a.out, "cal")
    os.makedirs(d, exist_ok=True)
    sj, sx = dumps(site).encode(), xml.encode()
    with open(os.path.join(d, "site.json"), "wb") as fh:
        fh.write(sj)
    with open(os.path.join(d, "site.xml"), "wb") as fh:
        fh.write(sx)
    with open(os.path.join(d, "qc.json"), "w") as fh:
        fh.write(dumps(dict(generator=site["generator"], envelope_source=site["envelope"]["source"],
                            envelope_files=site["envelope"]["files_read"], **q)))
    sha = write_archive(os.path.join(a.out, "cal.tar.xz"), {"site.json": sj, "site.xml": sx})
    with open(os.path.join(a.out, "plan.json"), "w") as fh:
        fh.write(dumps(plan))
    print(json.dumps(_clean(dict(pass_=q["pass"], failed=q["failed"], G2=q["G2"]["corner_margin_px"],
                                 G3=q["G3"].get("as_built", {}).get("max_px") if isinstance(q["G3"], dict) else None,
                                 routes=q["routes"], roofed_unseen={k: q["roofed_unseen"][k] for k in
                                                                    ("rays_on_block", "rays_ending_inside")},
                                 blind=q["blind_route"], block=q["passage_block"]["gap_east_m"],
                                 sp3=q["sp3_clearance"]["clearance_m"], crates=[r["pass_"] for r in q["crates"]["rows"]],
                                 plan_vs_site=[n_cmp, len(bad)], archive_sha256=sha))))
    return 0


if __name__ == "__main__":
    sys.exit(main())

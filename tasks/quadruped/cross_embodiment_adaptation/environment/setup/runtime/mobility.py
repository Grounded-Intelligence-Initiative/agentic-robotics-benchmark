"""Physical warehouse fittings and a fixed inspection load on the native Go1."""
from dataclasses import dataclass
import xml.etree.ElementTree as ET

from scene_assets import geom, vec


CLEARANCE, DUCT_DEPTH = .426, .068
DECK_Z, BEVEL, TOE = .065, .33, .012
GUIDE_RISE, COVER_OFFSET = .082, .155
TERRAIN_BIT = 1 << 39
SUPPORT = ("deck_table", "deck_west_ramp", "deck_east_ramp", "deck_center_guide",
           "deck_south_shoulder", "deck_north_shoulder")
DUCT = ("duct_box", "duct_left_shoe", "duct_right_shoe")
PROTECTED = ("crown_guard", "scanner_head", "scanner_shank", "guard_pedestal")


@dataclass(frozen=True)
class Fitting:
    kind: str
    x: float
    lo: float
    hi: float
    length: float


def fittings(seed, racks):
    duct_first = bool(seed & TERRAIN_BIT)
    result = []
    for index, rack in enumerate(racks[::2]):
        duct = duct_first == (index == 0)
        result.append(Fitting("duct" if duct else "deck", (rack.near+rack.far)/2,
                              *rack.wide, .44 if duct else .62))
    return result


def add_load(trunk):
    # A fixed child contributes mass and inertia but no joint, actuator or stabilizer.
    body = ET.SubElement(trunk, "body", name="inspection_saddle")
    for name, xyz, half, mass in (
        ("guard_pedestal", (-.085, 0, .111), (.032, .045, .062), .15),
        ("crown_guard", (-.055, 0, .183), (.102, .085, .016), .20),
        ("scanner_shank", (.248, 0, -.087), (.025, .038, .065), .12),
        ("scanner_head", (.309, 0, -.143), (.084, .088, .020), .23),
    ):
        geom(body, name, pos=xyz, size=half, rgba=(.42, .43, .46, 1), mass=mass,
             contype=1, conaffinity=1)


def add_fittings(root, world, items):
    metal = (.43, .46, .48, 1)

    def prism(name, x, y, half, section, color):
        verts = [(xx, yy, zz) for yy in (-half, half) for xx, zz in section]
        ET.SubElement(root.find("asset"), "mesh", name=name, vertex=vec(v for pt in verts for v in pt))
        geom(world, name, "mesh", (x, y, 0.), (), rgba=color, mesh=name,
             contype=1, conaffinity=1, friction="1 .02 .001")

    for f in items:
        w, y, x = (f.hi-f.lo)/2, (f.hi+f.lo)/2, f.x
        cover_x = x
        if f.kind == "duct":
            top = CLEARANCE+DUCT_DEPTH
            geom(world, "duct_box", pos=(x, y, (top+CLEARANCE)/2), size=(f.length/2, w+.045, DUCT_DEPTH/2),
                 rgba=metal, contype=1, conaffinity=1)
            for name, rim in zip(DUCT[1:], (f.lo-.025, f.hi+.025)):
                geom(world, name, pos=(x, rim, (.32+CLEARANCE)/2), size=(.075, .024, (CLEARANCE-.32)/2),
                     rgba=metal, contype=1, conaffinity=1)
        else:
            top = DECK_Z
            geom(world, SUPPORT[0], pos=(x, y, top/2), size=(f.length/2, w, top/2),
                 rgba=metal, contype=1, conaffinity=1, friction="1 .02 .001")
            for sign, name in zip((-1, 1), SUPPORT[1:3]):
                prism(name, x+sign*f.length/2, y, w,
                      ((0., 0.), (sign*BEVEL, 0.), (sign*BEVEL, TOE), (0., top)), metal)
            # Two wheel tracks at +/-0.125 m. All guide surfaces are legal footholds.
            section = ((-.305, top), (.305, top), (.249, top+GUIDE_RISE), (.061, top+GUIDE_RISE))
            for name, (lo, hi) in zip(SUPPORT[3:], ((-.050, .050), (-w, -.200), (.200, w))):
                prism(name, x, y+(lo+hi)/2, (hi-lo)/2, section, (.55, .56, .54, 1))
            top += GUIDE_RISE
            cover_x += COVER_OFFSET
        # Identical flat access covers on both fittings. Hue locates a cover, not its height.
        geom(world, f.kind+"_cover", pos=(cover_x, y, top+.0007), size=(.080, .039, .0007),
             rgba=(.06, .65, .68, 1))

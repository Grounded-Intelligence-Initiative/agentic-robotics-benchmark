"""Lay out a marked inspection bay around the pinned Menagerie Go1."""
import math
from itertools import permutations
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

GO1 = Path("/opt/visual_robotics/assets/unitree_go1/go1.xml")
JOINTS = tuple(f"{leg}_{part}" for leg in ("FR", "FL", "RR", "RL") for part in ("hip", "thigh", "calf"))
DELIVERIES = 6
LEGS = ("FR", "FL", "RR", "RL")
PICKUPS = np.array([(.188, -.128), (.188, .128), (-.188, -.128), (-.188, .128)])
# Screen columns are physical left/right; the front axle is the top row.
LENSES = [(.638, y, z) for z in (.425, .303) for y in (-.11, .11)]
PIT_START, PIT_END, PIT_DEPTH = .245, .560, .045


def pit_bounds(seed):
    # Solid shoulder immediately outside each steel cap; open trench between.
    return (PIT_START,PIT_END,-.188,.290,PIT_DEPTH) if seed%2==0 else (
        PIT_START,PIT_END,-.290,.188,PIT_DEPTH)


def loom(seed, scene_index):
    # A shuffled, balanced factory lot, independent of geometry, gains and performance.
    if scene_index is None or scene_index < 0:
        return tuple(map(int, np.random.default_rng([seed, 0x4C4F4F4D]).permutation(4)))
    order = np.random.default_rng(0x4C4F4F4D).permutation(24)
    return tuple(permutations(range(4)))[order[scene_index % 24]]


def paint(parent, pos, size, **style):
    return ET.SubElement(parent, "geom", type="box", pos=pos, size=size,
                         contype="0", conaffinity="0", group="1", **style)


def plan(seed, scene_index=None):
    rng = np.random.default_rng(seed)
    rng.integers(2)  # preserve the V6 route, settled pose and twelve position gains
    yaw = rng.uniform(-.035, .035)
    hand = int(rng.choice([-1, 1]))
    layout = np.random.default_rng([seed, 0xBA7])
    returning = bool(layout.integers(2))
    bearings = [0, 90, 0, 90, 180, 90] if returning else [0, 90, 0, -90, 0, 90]
    inspection = int(layout.choice([3, 4, 5]))
    offset = layout.choice([-1, 1])*layout.uniform(150, 172) if returning else hand*layout.uniform(52, 76)
    xy, stations = np.zeros(2), []
    for i, bearing in enumerate(bearings):
        travel = math.radians(hand*bearing + rng.uniform(-3, 3))
        length = rng.uniform(.49, .55) if not i else rng.uniform(.74, .87)
        xy = xy + length*np.array([math.cos(travel), math.sin(travel)])
        facing = math.remainder(travel + (math.radians(offset) if i == inspection else 0), 2*math.pi)
        stations.append((*xy, facing))
    # Same draws up to and including the gains as v5. Route family/inspection has its own stream.
    return stations, loom(seed, scene_index), yaw, rng.uniform(.88, 1.12, size=12)


def stripe(parent, a, b, width, height, color):
    delta = b-a
    mid = (a+b)/2
    paint(parent, f"{mid[0]} {mid[1]} {height}", f"{np.linalg.norm(delta)/2} {width/2} .0006",
          rgba=color, euler=f"0 0 {math.atan2(delta[1], delta[0])}")


def split_floor(world, assets, seed):
    x0, x1, y0, y1, depth = pit_bounds(seed)
    side = 1 if seed % 2 == 0 else -1

    def slab(name, xa, xb, ya, yb, bottom, top, **style):
        return ET.SubElement(world, "geom", name=name, type="box",
                             pos=f"{(xa+xb)/2} {(ya+yb)/2} {(bottom+top)/2}",
                             size=f"{(xb-xa)/2} {(yb-ya)/2} {(top-bottom)/2}", **style)

    ET.SubElement(world, "geom", name="pit_bed", type="plane", size="0 0 .05",
                  pos=f"0 0 {-depth}", rgba=".29 .32 .32 1")
    for name, xa, xb, ya, yb in (("floor", -5, x0, -5, 5), ("landing", x1, 5, -5, 5),
                                 ("slab_right", x0, x1, -5, y0), ("slab_left", x0, x1, y1, 5)):
        slab(name, xa, xb, ya, yb, -depth, 0, material="concrete")
    def hull(name, vertices):
        ET.SubElement(assets,"mesh",name=name,vertex=" ".join(str(v) for p in vertices for v in p))
        ET.SubElement(world,"geom",name=name,type="mesh",mesh=name,rgba=".36 .40 .40 1")

    # Fixed beveled webs keep an exiting toe from catching under a steel plate.
    # They bear load but are not the inspected top faces.
    ET.SubElement(assets, "material", name="outboard_ink", rgba=".43 .28 .98 1", emission=".56")
    for name, xa, xb, yc, half in (("near_runner", x0-.025, x1-.045, -side*.121, .067),
                                   ("wide_runner", x0+.045, x1+.025, side*.240, .050)):
        slab(name, xa, xb, yc-half, yc+half, -.012, .0008, rgba=".46 .54 .53 1")
        hull(name+"_web",[(x,yc+s*(half+w),z)
             for run,w,z in ((.065,.047 if name=="near_runner" else .080,-depth),(0.,0.,-.012))
             for x in (xa-run,xb+run) for s in (-1,1)])
        for sign in (-1, 1):
            style = ({"material": "outboard_ink"} if name == "wide_runner" else {"rgba": ".91 .85 .62 1"})
            paint(world, f"{(xa+xb)/2} {yc+sign*(half-.007)} .0016",
                  f"{(xb-xa)/2} .005 .0006", **style)
        for k in range(5):
            paint(world, f"{xa+(k+.5)*(xb-xa)/5} {yc} .0016", f".003 {half-.015} .0006",
                  rgba=".27 .31 .30 1")


def pad_graphics(world, assets, first):
    pad = ET.SubElement(world,"body",name="pad_paint",mocap="true")
    overlay = ET.SubElement(world,"body",name="landing_paint",mocap="true",pos="0 0 -3")
    tx,ty,yaw = first
    def rotate(p, a):
        x,y = p
        return np.array([x*math.cos(a)-y*math.sin(a),x*math.sin(a)+y*math.cos(a)])
    for i,(x,y,z,hx,hy,hz,a,color) in enumerate((
            (0,0,.002,.15,.15,.002,0,".035 .31 .72 1"),
            (0,0,.005,.10,.010,.001,0,".98 .98 .95 1"),
            (.075,.014,.005,.031,.009,.001,-.52,".98 .98 .95 1"),
            (.075,-.014,.005,.031,.009,.001,.52,".98 .98 .95 1"))):
        paint(pad,f"{x} {y} {z}",f"{hx} {hy} {hz}",euler=f"0 0 {a}",rgba=color)
        # First pad straddles the far lip. Continue the actual arrow on the
        # concrete, clipping its footprint rather than painting across mid-air.
        poly = [np.array([tx,ty])+rotate((x,y)+rotate((sx*hx,sy*hy),a),yaw)
                for sx,sy in ((-1,-1),(1,-1),(1,1),(-1,1))]
        clipped = []
        for p,q in zip(poly,poly[1:]+poly[:1]):
            if p[0] >= PIT_END:
                clipped.append(p)
            if (p[0] > PIT_END) != (q[0] > PIT_END):
                clipped.append(p+(q-p)*((PIT_END-p[0])/(q[0]-p[0])))
        if len(clipped) >= 3:
            mesh = "landing_paint_"+str(i)
            verts = [(u,v,h) for h in (z-hz,z+hz) for u,v in clipped]
            ET.SubElement(assets,"mesh",name=mesh,vertex=" ".join(str(n) for p in verts for n in p))
            ET.SubElement(overlay,"geom",type="mesh",mesh=mesh,rgba=color,contype="0",conaffinity="0",group="1")


def aisle_dimensions(seed, stations):
    rng = np.random.default_rng([seed, 0xA151E])
    a, b, deck = stations[:3]
    return {"hand": 1 if b[1] > a[1] else -1, "gate_x": (a[0]+b[0])/2,
            "gate_y": (a[1]+b[1])/2, "width": rng.uniform(.75,.81),
            "clearance": rng.uniform(.380,.390), "deck": deck,
            "rise": rng.uniform(.046,.056)}


def service_aisle(world, assets, spec):
    hand, x, s = spec["hand"], spec["gate_x"], spec["hand"]*spec["gate_y"]
    half, gap = spec["width"]/2, spec["clearance"]
    def wall(name, x0, x1, s0, s1, bottom, top, color=".41 .45 .45 1"):
        return ET.SubElement(world,"geom",name=name,type="box",rgba=color,
                             pos=f"{(x0+x1)/2} {hand*(s0+s1)/2} {(bottom+top)/2}",
                             size=f"{(x1-x0)/2} {(s1-s0)/2} {(top-bottom)/2}")
    lo, hi, back = -.65, x+half+.215, -.50  # solid turning apron beyond the hatch
    wall("bay_back",lo,hi,back-.034,back,0,.60)
    wall("bay_left",lo-.032,lo,back-.034,s+.09,0,.60)
    # Sightlines through a real steel guardrail: the front survey posts stand just beyond it.
    for k,(bottom,top) in enumerate(((0,.085),(.212,.228),(.347,.366),(.529,.552),(.61,.633))):
        wall("bay_rail_"+str(k),hi,hi+.023,back-.034,s+.09,bottom,top)
    for k,u in enumerate((back,(back+s)/2,s+.05)):
        wall("bay_stud_"+str(k),hi,hi+.029,u,u+.018,0,.633)
    wall("duct_jamb_left",lo,x-half,s-.09,s+.09,0,.60)
    wall("duct_jamb_right",x+half,hi,s-.09,s+.09,0,.60)
    wall("overhead_duct",x-half,x+half,s-.078,s+.078,gap,gap+.105,".73 .69 .45 1")
    ET.SubElement(assets,"material",name="warning_band",rgba=".98 .79 .095 1",emission=".38")
    for side in (-1,1):
        paint(world,f"{x} {hand*(s+side*.079)} {gap+.052}",f"{half} .001 .048",material="warning_band")
        for k in range(9):
            paint(world,f"{x+(k-4)*.080} {hand*(s+side*.082)} {gap+.054}",
                  ".020 .001 .049",rgba=".14 .14 .13 1",euler="0 -.36 0")
    # One raised cable inspection cover, with real beveled edges and a flat load-bearing top.
    dx,dy,heading = spec["deck"]
    z = spec["rise"]
    vertices = [(sx*hx,sy*hy,h) for hx,hy,h in ((.48,.45,0),(.48,.45,.018),(.35,.32,z))
                for sx,sy in ((-1,-1),(1,-1),(1,1),(-1,1))]
    ET.SubElement(assets,"mesh",name="deck_hull",vertex=" ".join(str(v) for p in vertices for v in p))
    deck=ET.SubElement(world,"body",name="cable_cover",pos=f"{dx} {dy} 0",euler=f"0 0 {heading}")
    ET.SubElement(deck,"geom",name="cable_deck",type="mesh",mesh="deck_hull",rgba=".55 .53 .48 1",
                  friction=".95 .005 .0001")
    for side in (-1,1):
        for k in range(7):
            paint(deck,f"{side*.335} {k*.09-.27} {z+.001}",".014 .022 .001",material="warning_band")
    paint(deck,f"0 0 {z+.001}",".35 .012 .001",rgba=".93 .72 .2 1")


def launch_rack(world, assets):
    rack = ET.SubElement(world, "body", name="launch_rack", mocap="true", pos="0 0 -3")
    paint(rack, ".663 0 .39", ".011 .208 .147", rgba=".105 .12 .13 1")
    for y in (-.187, .187):
        paint(rack, f".67 {y} .13", ".009 .009 .13", rgba=".27 .29 .30 1")
    for x, y in PICKUPS:
        paint(rack, f"{x} {y} .001", ".075 .075 .001", rgba=".33 .35 .35 1")
    for name, rgb in (("covered", ".015 .75 1"), ("clear", "1 .29 .012")):
        ET.SubElement(assets, "material", name=name, rgba=rgb+" 1", emission=".8", specular="0")
    # Raised, readable physical letters, not text composited into the camera image.
    strokes = {"F": [(0,0,0,1),(0,1,.65,1),(0,.52,.53,.52)],
               "L": [(0,1,0,0),(0,0,.65,0)],
               "R": [(0,0,0,1),(0,1,.6,1),(.6,1,.6,.52),(.6,.52,0,.52),(.22,.52,.68,0)]}
    for i, (x,y,z) in enumerate(LENSES):
        paint(rack, f"{x+.012} {y} {z}", ".004 .050 .025", material="covered")
        for j,ch in enumerate(LEGS[i]):
            for a,b,c,d in strokes[ch]:
                left = y+.033-j*.043
                line = f".647 {left-a*.044} {z+.036+b*.036} .647 {left-c*.044} {z+.036+d*.036}"
                ET.SubElement(rack,"geom",type="capsule",fromto=line,size=".0024",
                              rgba=".98 .98 .98 1",contype="0",conaffinity="0")
        flag = ET.SubElement(world,"body",name="clear_"+LEGS[i],mocap="true",pos="0 0 -3")
        paint(flag, "0 0 0", ".003 .050 .025", material="clear")


def build(seed, scene_index=None):
    stations, legs_by_port, yaw, gains = plan(seed, scene_index)

    root = ET.parse(GO1).getroot()
    root.find("compiler").set("meshdir", str(GO1.parent / "assets"))
    world = root.find("worldbody")
    trunk = world.find("body[@name='trunk']")
    trunk.remove(trunk.find("camera[@name='tracking']"))
    # The original inspection camera now has a visible collision housing around its optical centre.
    # The factory trunk has an explicit inertial; these fixed geoms do not replace it.
    for name,pos,size in (("camera_hood",".096 0 .176",".026 .032 .006"),
                          ("camera_stanchion",".069 0 .117",".009 .012 .057")):
        ET.SubElement(trunk,"geom",name=name,type="box",pos=pos,size=size,rgba=".24 .25 .24 1")
    ET.SubElement(trunk, "camera", name="onboard", pos=".11 0 .16", fovy="82",
                  xyaxes="0 -1 0 .422618262 0 .906307787")
    texture = GO1.parent.parent / "textures"
    assets = root.find("asset")
    for name, file, repeat in (("concrete", "concrete_floor_02.png", "1 1"),
                               ("brick", "brick_wall_001.png", "4 1")):
        ET.SubElement(assets, "texture", type="2d", name=name, file=str(texture / file))
        ET.SubElement(assets, "material", name=name, texture=name, texrepeat=repeat, texuniform="true")
    colors = {"pink": ".95 .02 .67 1", "cyan": ".015 .83 .94 1",
              "red": ".96 .045 .025 1", "green": ".035 .92 .045 1"}
    for name, rgba in colors.items():
        ET.SubElement(assets, "material", name=name, rgba=rgba, emission=".8", specular="0")
    split_floor(world, assets, seed)
    ET.SubElement(world, "light", pos="2 0 3", dir="0 0 -1", directional="true", castshadow="false")
    for pos, size in (("1 -4 .55", "3.5 .05 .55"), ("1 4 .55", "3.5 .05 .55"),
                      ("4.5 0 .55", ".05 4 .55"), ("-2.5 0 .55", ".05 4 .55")):
        ET.SubElement(world, "geom", type="box", pos=pos, size=size, material="brick")
    start = np.zeros(2)
    pit = pit_bounds(seed)
    for x, y, _ in stations:
        end = np.array([x, y])
        unit = (end-start)/np.linalg.norm(end-start)
        side = np.array([-unit[1], unit[0]])
        # Paint follows the bed, not a fictitious surface across the opening.
        cuts = [0., 1.]
        for j, bounds in enumerate((pit[:2], pit[2:4])):
            if abs(end[j]-start[j]) > 1e-9:
                cuts += [t for v in bounds if 0 < (t := (v-start[j])/(end[j]-start[j])) < 1]
        def level(point):
            return -pit[4] if pit[0] < point[0] < pit[1] and pit[2] < point[1] < pit[3] else 0.
        cuts.sort()
        for t, u in zip(cuts, cuts[1:]):
            a, b = start+t*(end-start), start+u*(end-start)
            stripe(world, a, b, .025, level((a+b)/2)+.001, ".91 .68 .15 1")
        for f in (.30, .68):
            tip = start+f*(end-start)
            for sign in (-1, 1):
                stripe(world, tip-.082*unit+sign*.050*side, tip, .019, level(tip)+.002, ".99 .84 .33 1")
        start = end
    # A physical movable *visual* fixture. It cannot propel, restrain or support the robot.
    dock = ET.SubElement(world, "body", name="station", mocap="true")
    pad_graphics(world, assets, stations[0])
    for x, y, color in ((.52, .29, "pink"), (.52, -.29, "cyan"),
                        (-.52, .29, "red"), (-.52, -.29, "green")):
        paint(dock, f"{x} {y} .23", ".009 .009 .23", rgba=".25 .28 .30 1")
        for z in (.20, .46):
            ET.SubElement(dock, "geom", type="sphere", pos=f"{x} {y} {z}", size=".024",
                          material=color, contype="0", conaffinity="0", group="1")
    # A physical departure placard fixed in the prior station frame, not a camera overlay.
    sign = ET.SubElement(world, "body", name="departure", mocap="true")
    paint(sign, ".012 0 -.22", ".010 .010 .22", rgba=".29 .31 .30 1")
    paint(sign, "0 0 0", ".009 .119 .115", rgba=".14 .15 .16 1")
    pointer = ET.SubElement(world, "body", name="pointer", mocap="true")
    ET.SubElement(assets, "material", name="pin", rgba=".035 .05 1 1", emission=".8")
    ET.SubElement(pointer, "geom", type="sphere", pos="-.023 0 -.064", size=".013",
                  material="pin", contype="0", conaffinity="0", group="1")
    for pos, size, angle in (("-.013 0 0", ".002 .009 .072", 0),
                             ("-.014 .018 .051", ".002 .010 .032", .64),
                             ("-.014 -.018 .051", ".002 .010 .032", -.64)):
        paint(pointer, pos, size, rgba="1 .90 .20 1", euler=f"{angle} 0 0")
    launch_rack(world, assets)
    service_aisle(world, assets, aisle_dimensions(seed,stations))
    model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
    if (tuple(model.actuator(i).name for i in range(model.nu)) != JOINTS
            or (model.nq, model.nv, model.neq, model.ncam) != (19, 18, 0, 1)):
        raise RuntimeError("expected the free-base Go1 and twelve native position servos")
    model.actuator_gainprm[:, 0] *= gains
    model.actuator_biasprm[:, 1] *= gains
    return model, stations, legs_by_port, yaw

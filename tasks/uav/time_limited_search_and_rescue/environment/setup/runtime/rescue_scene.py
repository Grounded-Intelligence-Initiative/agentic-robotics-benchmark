"""Procedural local-view training wing and its rendered collision geometry."""
from collections import deque
from dataclasses import dataclass
from io import BytesIO
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image, ImageDraw, ImageFont

from pbr_scene import prop
from floor_finish import install
from scene_assets import asset_root, geom, materials, robot_meshes

SIDE = 5
START_Z = 1.30
SLOT_SILL, SLOT_LINTEL = .43, .89
RAISED_SILL = .73
DECK, LANDING, WALL_TOP = .60, .99, 2.42
DARK = (.075, .112, .126, 1)
WALL = (.65, .70, .70, 1)
WHITE = (.94, .95, .91, 1)
AMBER = (.95, .67, .08, 1)
TEAL = (.022, .74, .39, 1)
BLUE = (.05, .32, .84, 1)
ORANGE = (1., .26, .018, 1)
PURPLE = (.72, .19, .86, 1)
RED = (.90, .045, .045, 1)
STEEL = (.06, .79, .91, 1)
FLANGE_RADII = (.041, .093)
# Inner throat and outer seat of the closed collet, in metres.
COLLETS = ((.027, .054), (.074, .106))
DOCK_Z = .224
LITTER = (.22, .90, .072)
LIME = (.48, .98, .035, 1)
GRIP_TAPE = (.37, .08, .98, 1)
THROAT = .55
POCKET_HALF_DEPTH = 2.30


def edge(a, b):
    return tuple(sorted((a, b)))


def axis_at(axis, i):
    if i < 0:
        return 2*axis[0]-axis[1]
    if i >= len(axis):
        return 2*axis[-1]-axis[-2]
    return axis[i]


def boundaries(axis, i):
    center = axis[i]
    return (axis_at(axis, i-1)+center)/2, (axis_at(axis, i+1)+center)/2


def floor_z(cell):
    return DECK if cell[1] >= 3 else 0.


def interior_edges():
    for y in range(1, SIDE):
        for x in range(SIDE):
            for dx, dy in ((1, 0), (0, 1)):
                if x+dx < SIDE and y+dy < SIDE:
                    yield edge((x, y), (x+dx, y+dy))


def distances(start, edges):
    dist = {start: 0}
    visit = deque([start])
    while visit:
        a = visit.popleft()
        for u, v in edges:
            b = v if u == a else u if v == a else None
            if b is not None and b not in dist:
                dist[b] = dist[a]+1
                visit.append(b)
    return dist


@dataclass(frozen=True)
class Layout:
    halls: frozenset
    raised: frozenset
    inlet: int
    outlet: int
    rooms: tuple
    pod: tuple
    safety: tuple
    tone: float
    columns: tuple
    rows: tuple
    launch: tuple
    gains: tuple
    pad: tuple
    flange: float
    depot: tuple
    loading: tuple

    def xy(self, cell):
        return np.array([axis_at(self.columns, cell[0]), axis_at(self.rows, cell[1])])

    @property
    def deck_y(self):
        return (self.rows[2]+self.rows[3])/2

    @property
    def inlet_y(self):
        return (self.rows[0]+self.rows[1])/2

    def dock(self, slot):
        return (self.columns[-1]+.32*(2*slot-1), .08, DOCK_Z)

    @property
    def pocket(self):
        return np.array([self.columns[self.outlet], boundaries(self.rows, 0)[0]-.90-POCKET_HALF_DEPTH])

    @property
    def receiving_pad(self):
        offset, side, stagger = self.loading
        return self.pocket + [side*3.80+self.pad[0], stagger+self.pad[1]]

    @property
    def bounds(self):
        return ((min(boundaries(self.columns, 0)[0], self.pocket[0]-5.5),
                 max(boundaries(self.columns, SIDE-1)[1], self.pocket[0]+5.5)),
                (self.pocket[1]-POCKET_HALF_DEPTH-.15, boundaries(self.rows, SIDE-1)[1]-.05),
                (.39, 2.09))


def layout_for(seed):
    rng = np.random.default_rng(seed >> 1)
    inlet, outlet = int(rng.integers(1, 4)), int(rng.choice((0, 4)))
    service_row = int(rng.integers(3, SIDE))
    all_edges = [e for e in interior_edges() if all(p[0] != outlet for p in e)]
    groups = {p: p for e in all_edges for p in e}

    def find(p):
        while p != groups[p]:
            p = groups[p]
        return p

    halls, loops = [], []
    for i in rng.permutation(len(all_edges)):
        u, v = e = all_edges[i]
        a, b = find(u), find(v)
        if a != b:
            groups[a] = b
            halls.append(e)
        else:
            loops.append(e)
    # The cargo column is a dedicated egress spine, linked to the rooms only
    # near the far end. It cannot shorten any outbound search path.
    halls += loops[:3]
    halls += [edge((outlet, y), (outlet, y+1)) for y in range(1, SIDE-1)]
    halls.append(edge((outlet, service_row), (1 if outlet == 0 else SIDE-2, service_row)))
    raised = {halls[i] for i in rng.choice(len(halls), size=3, replace=False)}
    # The inspection pocket is below the hall sills: level flight from the
    # insertion slot into the rest of the wing is physically blocked.
    raised.update(e for e in halls if (inlet, 1) in e)
    from_inlet = distances((inlet, 1), halls)
    from_outlet = distances((outlet, 1), halls)
    deep = sorted((p for p in from_inlet if p[1] >= 3 and p[0] != outlet),
                  key=lambda p: min(from_inlet[p], from_outlet[p]), reverse=True)[:8]
    rooms = tuple(deep[i] for i in rng.choice(len(deep), size=3, replace=False))
    target = rooms[int(rng.integers(3))]
    # On a cycle, never the sole connection. Prefer a shutter that changes
    # the shortest loaded route, rather than closing an irrelevant dead end.
    choices = []
    for e in halls:
        if any(p[0] == outlet for p in e):
            continue
        after = distances((outlet, 1), [other for other in halls if other != e])
        if len(after) == len(from_outlet):
            choices.append((after[target]-from_outlet[target], int(e[0][1] >= 3), e))
    safety = max(choices)[2]
    dx, dy = rng.uniform(-.16, .16, 2)
    yaw = float(rng.uniform(-.50, .50))
    tone = float(rng.uniform(.92, 1.05))
    columns = np.r_[0., np.cumsum(rng.uniform(2.60, 2.80, SIDE-1))]
    columns -= columns[2]
    rows = np.r_[0., np.cumsum(rng.uniform(2.60, 2.80, SIDE-1))]
    start_x, start_y = rng.uniform(-.13, .13, 2)
    gains = tuple(rng.uniform(.96, 1.045, 2))
    pad_x, pad_y = rng.uniform(-.16, .16, 2)
    flange = FLANGE_RADII[int(rng.integers(2))]
    depot = tuple(COLLETS[i] for i in rng.permutation(2))
    yaw += int(rng.integers(2))*np.pi/2
    loading = (rng.choice((-1, 1))*rng.uniform(.56, .66), int(rng.choice((-1, 1))),
               rng.choice((-1, 1))*rng.uniform(.55, .70))

    def mirror(p):
        return (SIDE-1-p[0], p[1]) if seed & 1 else p

    def transformed(edges):
        return frozenset(edge(mirror(u), mirror(v)) for u, v in edges)

    if seed & 1:
        dx, yaw = -dx, -yaw
        start_x = -start_x
        pad_x = -pad_x
        columns = -columns[::-1]
    bx, by = mirror(target)
    px, py = columns[bx]+dx, rows[by]+dy
    return Layout(transformed(halls), transformed(raised), mirror((inlet, 1))[0],
                  mirror((outlet, 1))[0], tuple(map(mirror, rooms)), (px, py, yaw),
                  edge(*map(mirror, safety)), tone, tuple(columns), tuple(rows),
                  (start_x, start_y), gains, (pad_x, pad_y), flange, depot, loading)


def build(layout):
    xy = layout.xy
    root = ET.fromstring("""
    <mujoco model="indoor rescue training wing">
        <compiler angle="degree" autolimits="true"/>
        <option timestep=".005" integrator="implicitfast"/>
        <visual>
          <global offwidth="256" offheight="256"/>
          <headlight ambient=".54 .54 .54" diffuse=".26 .26 .26" specular="0 0 0"/>
          <map znear=".001" zfar="6"/>
        </visual>
        <worldbody>
          <light directional="true" dir=".3 .42 -1" diffuse=".62 .60 .58" castshadow="false"/>
          <geom name="floor" type="plane" size="16 16 1" friction=".9 .02 .005"/>
          <body name="uav" gravcomp="1">
            <inertial pos="0 0 0" mass=".8" diaginertia=".012 .012 .015"/>
            <joint name="x" type="slide" axis="1 0 0" damping="9"/>
            <joint name="y" type="slide" axis="0 1 0" damping="9"/>
            <joint name="z" type="slide" axis="0 0 1" damping="9"/>
            <joint name="yaw" type="hinge" axis="0 0 1" damping=".65"/>
            <camera name="down" pos="0 .46 -.145" fovy="94"/>
            <camera name="forward" pos="0 .17 .045" xyaxes="1 0 0 0 .12 .99277" fovy="94"/>
          </body>
          <body name="pod" pos="0 2 .074">
            <freejoint name="pod_free"/>
            <geom name="litter_board" type="box" size=".22 .9 .072" mass=".20"
                  rgba=".12 .33 .31 1" friction=".95 .04 .01"/>
          </body>
        </worldbody>
        <equality><weld name="clamp" body1="uav" body2="pod" active="false"
                        solref=".008 1" relpose="0 0 -.31 1 0 0 0"/></equality>
        <actuator>
          <position name="mx" joint="x" kp="190" kv="16"/>
          <position name="my" joint="y" kp="190" kv="16"/>
          <position name="mz" joint="z" kp="190" kv="16"/>
          <position name="heading" joint="yaw" kp="25" kv="1.1"/>
        </actuator>
    </mujoco>""")
    asset = materials(root)
    decals = {}
    install(asset, decals)
    w = root.find("worldbody")
    w.find("geom[@name='floor']").set("material", "quarry")
    uav = w.find("body[@name='uav']")
    robot_meshes(root, uav, "drone")
    geom(uav, "gripper", "cylinder", (0, 0, -.133), (.035, .098), DARK)
    for side in (-1, 1):
        geom(uav, f"jaw{side}", pos=(side*.053, 0, -.207), size=(.025, .050, .025), rgba=WALL)
    pod = w.find("body[@name='pod']")
    pod.find("geom").set("size", " ".join(map(str, LITTER)))
    for end in (-1, 1):
        geom(pod, f"end_grip{end}", pos=(0, end*.81, .078),
             size=(.155, .035, .006), rgba=GRIP_TAPE)
    for side in (-1, 1):
        geom(pod, f"frame_rail{side}", pos=(side*.199, 0, .079),
             size=(.020, .89, .009), rgba=WHITE)
    geom(pod, "shell", pos=(0, 0, .053), size=(.155, .26, .051), rgba=ORANGE)
    geom(pod, "torso", "ellipsoid", (0, .24, .113), (.13, .24, .055), AMBER)
    geom(pod, "head", "ellipsoid", (0, .57, .12), (.092, .108, .075), AMBER)
    for side in (-1, 1):
        geom(pod, f"leg{side}", pos=(side*.064, -.38, .094), size=(.044, .30, .030), rgba=AMBER)
        geom(pod, f"arm{side}", pos=(side*.15, .17, .10), size=(.026, .19, .024), rgba=AMBER)
    geom(pod, "flange", "cylinder", (0, 0, .164), (layout.flange, .023), STEEL)
    geom(pod, "stem", "cylinder", (0, 0, .211), (.020, .025), DARK)
    # Same top cover on both fittings; read the load-bearing flange from the side.
    geom(pod, "fitting_cover", "cylinder", (0, 0, .247), (.132, .012), DARK)
    geom(pod, "centering_button", "cylinder", (0, 0, .261), (.080, .003), (.04, .90, .06, 1))
    px, py = layout.receiving_pad
    geom(w, "receiving_mat", pos=(px, py, .001), size=(1.25, .74, .001), rgba=TEAL)
    geom(w, "pad", pos=(px, py, .004), size=(.56, .53, .004), rgba=BLUE)
    for side in (-1, 1):
        geom(w, f"pad_side{side}", pos=(px+side*.39, py, .010), size=(.030, .42, .002), rgba=WHITE)
    geom(w, "pad_cross", pos=(px, py, .010), size=(.39, .030, .002), rgba=WHITE)

    xmin, xmax = boundaries(layout.columns, 0)[0], boundaries(layout.columns, SIDE-1)[1]
    ymax = boundaries(layout.rows, SIDE-1)[1]
    deck_center = ((xmin+xmax)/2, (layout.deck_y+ymax)/2, DECK/2)
    deck_half = ((xmax-xmin)/2, (ymax-layout.deck_y)/2, DECK/2)
    geom(w, "mezzanine", pos=deck_center, size=deck_half, material="terrazzo",
         rgba=(1, 1, 1, 1), contype="1", conaffinity="1")
    obstacles = [("deck", "mezzanine")]
    shutters = {}

    def sign(text, x, y, z, width=.34):
        key = text.lower().replace(" ", "_")
        filename = key+".png"
        if filename not in decals:
            im = Image.new("RGB", (256, 64), (27, 42, 48))
            ImageDraw.Draw(im).text((128, 31), text, anchor="mm", fill=(241, 246, 240),
                                    font=ImageFont.load_default(size=35))
            out = BytesIO()
            im.save(out, format="PNG")
            decals[filename] = out.getvalue()
            ET.SubElement(asset, "texture", name=key, type="2d", file=filename)
            ET.SubElement(asset, "material", name=key, texture=key, texrepeat="1 1")
        geom(w, f"sign{len(w)}", "plane", (x, y, z+.012), (width, .085, .002), material=key)

    sign("STAGING", 0., -.64, 0.)
    sign("TOOL DEPOT", layout.columns[-1], -.47, 0.)
    for slot, (throat, seat) in enumerate(layout.depot):
        name = f"collet{slot}"
        x, y, z = layout.dock(slot)
        stand = ET.SubElement(w, "body", name=name+"_stand", pos=f"{x} {y} {z}")
        geom(stand, name+"_post", "cylinder", (0, 0, .089-z), (.035, .089), DARK)
        head = ET.SubElement(w, "body", name=name, pos=f"{x} {y} {z}")
        ET.SubElement(head, "freejoint", name=name+"_free")
        ET.SubElement(head, "inertial", pos="0 0 0", mass=".035",
                      diaginertia=".00015 .00015 .00015")
        # Two semicircular jaws, with a retaining lip beneath the recessed seat.
        for layer, radius, height in (("seat", seat, 0.), ("lip", throat, -.027)):
            outer = seat+.027
            mid = (radius+outer)/2
            for i in range(16):
                if i in (0, 8):
                    continue
                a = i*np.pi/8
                geom(head, f"{name}_{layer}{i}", pos=(mid*np.cos(a), mid*np.sin(a), height),
                     size=((outer-radius)/2, radius*np.tan(np.pi/16), .013), rgba=STEEL,
                     euler=f"0 0 {np.degrees(a)}")
        eq = root.find("equality")
        ET.SubElement(eq, "weld", name=name+"_dock", body1=name+"_stand", body2=name,
                      relpose="0 0 0 1 0 0 0", solref=".008 1")
        ET.SubElement(eq, "weld", name=name+"_mount", body1="uav", body2=name,
                      active="false", solref=".008 1")

    def wall(center, half, kind, color=WALL):
        name = f"solid{len(obstacles)}"
        obstacles.append((kind, name))
        geom(w, name, pos=center, size=half, rgba=color)

    def boundary(a, b, route=None):
        center = (xy(a)+xy(b))/2
        normal = 0 if a[0] != b[0] else 1
        tangent = 1-normal
        gate_half = .51 if route == "inspection" else .74
        axis = layout.rows if tangent == 1 else layout.columns
        lo, hi = boundaries(axis, a[tangent])
        lo, hi = lo-center[tangent], hi-center[tangent]
        vertical = floor_z(a) != floor_z(b) and min(a[1], b[1]) == 2
        safety = edge(a, b) == layout.safety

        def section(mid, span, lo, hi, kind, color):
            pos, half = [*center, (lo+hi)/2], [.075, .075, (hi-lo)/2]
            pos[tangent] += mid
            half[tangent] = span
            wall(pos, half, kind, color)
            if kind == "wall":
                # Continuous timber skirting ends with the actual wall at a jamb.
                for side, room in ((-1, a), (1, b)):
                    base = pos.copy()
                    base[normal] += side*(half[normal]+.003)
                    base[2] = floor_z(room)+.064
                    trim = half.copy()
                    trim[normal], trim[2] = .003, .064
                    wall(base, trim, kind, (.39, .26, .17, 1))

        if route is None:
            section((lo+hi)/2, (hi-lo)/2+.026, 0., WALL_TOP, "wall", WALL)
            return
        color = PURPLE if route == "inspection" else BLUE if route == "cargo" else AMBER if vertical else WHITE
        for i, (here, there) in enumerate(((a, b), (b, a))):
            # A wear plate extends from the physical opening onto its approach.
            run = .84 if vertical else .79
            pos = center+(xy(here)-center)*(.075+run/2)/np.linalg.norm(xy(here)-center)
            size = [run/2, run/2, .004]
            size[tangent] = gate_half-.07
            name = f"apron{len(w)}"
            geom(w, name, pos=(*pos, floor_z(here)+.007), size=size, rgba=color)
            if safety:
                shutters.setdefault("warning", []).append(name)
        for start, end in ((lo, -gate_half), (gate_half, hi)):
            section((start+end)/2, (end-start)/2+.014, 0., WALL_TOP, "wall", WALL)
        if route == "inspection":
            section(0., gate_half, 0., SLOT_SILL, "inspection", AMBER)
            section(0., gate_half, SLOT_LINTEL, WALL_TOP, "inspection", AMBER)
            sign("INSPECTION", *xy((a[0], 0)), 0.)
        elif vertical:
            section(0., gate_half, 0., LANDING, "landing", AMBER)
            section(0., gate_half, 2.28, WALL_TOP, "landing", AMBER)
        elif route == "raised" and not safety:
            floor = floor_z(a)
            section(0., gate_half, floor, floor+RAISED_SILL, "raised", (.57, .47, .18, 1))
        elif route == "cargo":
            x, y = center
            body = ET.SubElement(w, "body", name="cargo_shutter", mocap="true", pos=f"{x} {y} 1.025")
            geom(body, "cargo_leaf", size=(gate_half, .075, 1.025), rgba=BLUE)
            obstacles.append(("cargo", "cargo_leaf"))
            sign("CARGO", *xy((a[0], 0)), 0.)
            for z in np.linspace(-.90, .90, 9):
                geom(body, f"shutter_rib{z:.3f}", pos=(0, 0, z), size=(gate_half, .084, .018), rgba=DARK)
                obstacles.append(("cargo", f"shutter_rib{z:.3f}"))
            for side in (-1, 1):
                geom(w, f"cargo_jamb{side}", pos=(x+side*(gate_half+.012), y, 1.025),
                     size=(.030, .10, 1.025), rgba=BLUE)
                obstacles.append(("cargo", f"cargo_jamb{side}"))
        if safety:
            x, y = center
            half = [.085, .085, 1.21]
            half[tangent] = gate_half
            body = ET.SubElement(w, "body", name="safety_shutter", mocap="true", pos=f"{x} {y} 3.8")
            geom(body, "safety_leaf", size=half, rgba=RED)
            for i, z in enumerate(np.linspace(-1., 1., 7)):
                rib = half.copy()
                rib[normal], rib[2] = .092, .021
                geom(body, f"safety_rib{i}", pos=(0, 0, z), size=rib, rgba=DARK)
                obstacles.append(("safety", f"safety_rib{i}"))
            obstacles.append(("safety", "safety_leaf"))

    for y in range(SIDE):
        boundary((-1, y), (0, y))
        boundary((SIDE-1, y), (SIDE, y))
    for x in range(SIDE):
        if x != layout.outlet:
            boundary((x, -1), (x, 0))
        boundary((x, SIDE-1), (x, SIDE))
        route = "inspection" if x == layout.inlet else "cargo" if x == layout.outlet else None
        boundary((x, 0), (x, 1), route)
    for a, b in interior_edges():
        route = "raised" if (a, b) in layout.raised else "open" if (a, b) in layout.halls else None
        boundary(a, b, route)

    # The only entrance to receiving is through two full-height loading throats.
    cx, cy = layout.pocket
    entry, side, stagger = layout.loading
    north, south = cy+POCKET_HALF_DEPTH, cy-POCKET_HALF_DEPTH
    far, back, exit_x = cx+side*5.45, cx-side*2.10, cx+side*2.10

    def panel(axis, fixed, a, b, kind="loading_wall", depth=.075):
        mid, span = (a+b)/2, abs(b-a)/2
        pos, size = ([fixed, mid], [depth, span]) if axis == 0 else ([mid, fixed], [span, depth])
        wall((*pos, WALL_TOP/2), (*size, WALL_TOP/2), kind,
             (.32, .57, .59, 1) if kind.startswith("throat") else WALL)

    def throat(axis, fixed, center, lo, hi, inward, name):
        panel(axis, fixed, lo, center-THROAT, name, .28)
        panel(axis, fixed, center+THROAT, hi, name, .28)
        # Reflective toe lines run along the clear inner faces onto the approach.
        for j in (-1, 1):
            p = [fixed-inward*.40, center+j*(THROAT-.025)]
            h = [.84, .025]
            if axis == 1:
                p.reverse()
                h.reverse()
            geom(w, f"{name}_line{j}", pos=(*p, .010), size=(*h, .006), rgba=LIME)

    throat(1, north, cx+entry, cx-2.10, cx+2.10, -1, "throat_entry")
    for x in boundaries(layout.columns, layout.outlet):
        panel(0, x, north, north+.90)
    throat(0, exit_x, cy+stagger, south, north, side, "throat_exit")
    panel(0, back, south, north)
    panel(0, far, south, north)
    panel(1, south, back, far)
    panel(1, north, exit_x, far)
    sign("LITTER TRANSFER", cx, north+.56, 0., .49)
    sign("TURNING POCKET", cx, cy-.49, 0., .48)
    sign("RECEIVING", px, py-.61, 0.)
    # Bay paint runs around (not through) the central pickup area.
    for i, room in enumerate(layout.rooms):
        x, y = xy(room)
        sign("RESCUE BAY", x, y-.43, floor_z(room))
        for side in (-1, 1):
            geom(w, f"bay{i}_rail{side}", pos=(x+side*.47, y, floor_z(room)+.005),
                 size=(.023, .57, .004), rgba=TEAL)
    for y in (1, 2, 3, 4):
        label = "CARGO RUN" if y != 3 else "MEZZANINE"
        sign(label, *xy((layout.outlet, y)), floor_z((layout.outlet, y)))
    for side, x in enumerate((xmin-.52, xmax+.52)):
        prop(root, asset_root(), "utility_box_01", f"cabinet{side}", (x, 0, 0), (.52, .61, .87))

    return ET.tostring(root, encoding="unicode"), decals, obstacles, shutters

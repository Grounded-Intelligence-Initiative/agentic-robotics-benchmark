"""Procedural training wing, occupancy model, and its physical scale plan."""
from collections import deque
from dataclasses import dataclass
import xml.etree.ElementTree as ET

import numpy as np

from pbr_scene import prop
from scene_assets import asset_root, geom, materials, robot_meshes

SIDE = 5
PITCH = 1.72
MODULE = .29
PLAN = (-PITCH, 0.)
DOWN_OFFSET = .20
START = (0., 0., 1.24)
BOUNDS = ((-4.15, 4.15), (-.72, 7.61), (.39, 1.65))
SLOT_SILL, SLOT_LINTEL = .43, .89
RAISED_SILL = .73
WALL_TOP = 2.05
HOME = (2, 0)
DARK = (.075, .112, .126, 1)
WALL = (.65, .70, .70, 1)
WHITE = (.94, .95, .91, 1)
AMBER = (.95, .67, .08, 1)
TEAL = (.022, .74, .39, 1)
BLUE = (.05, .32, .84, 1)
ORANGE = (1., .26, .018, 1)


def edge(a, b):
    return tuple(sorted((a, b)))


def xy(cell, pitch=PITCH):
    return np.array([(cell[0]-2)*pitch, cell[1]*pitch])


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
    tone: float


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
    deep = sorted((p for p in from_inlet if p[1] >= 2 and p[0] != outlet),
                  key=lambda p: min(from_inlet[p], from_outlet[p]), reverse=True)[:8]
    rooms = tuple(deep[i] for i in rng.choice(len(deep), size=3, replace=False))
    target = rooms[int(rng.integers(3))]
    dx, dy = rng.uniform(-.28, .28, 2)
    yaw = float(rng.uniform(-.50, .50))
    tone = float(rng.uniform(.92, 1.05))

    def mirror(p):
        return (SIDE-1-p[0], p[1]) if seed & 1 else p

    def transformed(edges):
        return frozenset(edge(mirror(u), mirror(v)) for u, v in edges)

    if seed & 1:
        dx, yaw = -dx, -yaw
    px, py = xy(mirror(target)) + (dx, dy)
    return Layout(transformed(halls), transformed(raised), mirror((inlet, 1))[0],
                  mirror((outlet, 1))[0], tuple(map(mirror, rooms)), (px, py, yaw), tone)


def build(layout):
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
          <geom name="floor" type="plane" size="6 10 1" friction=".9 .02 .005"/>
          <body name="uav" gravcomp="1">
            <inertial pos="0 0 0" mass=".8" diaginertia=".012 .012 .015"/>
            <joint name="x" type="slide" axis="1 0 0" damping="9"/>
            <joint name="y" type="slide" axis="0 1 0" damping="9"/>
            <joint name="z" type="slide" axis="0 0 1" damping="9"/>
            <camera name="down" pos="0 .20 -.145" fovy="94"/>
            <camera name="forward" pos="0 .17 .045" xyaxes="1 0 0 0 .12 .99277" fovy="94"/>
          </body>
          <body name="pod" pos="0 2 .074">
            <freejoint name="pod_free"/>
            <geom name="sled_collision" type="box" size=".145 .245 .072" mass=".15"
                  rgba=".105 .115 .12 1" friction=".95 .04 .01"/>
          </body>
        </worldbody>
        <equality><weld name="clamp" body1="uav" body2="pod" active="false"
                        solref=".008 1" relpose="0 0 -.31 1 0 0 0"/></equality>
        <actuator>
          <position name="mx" joint="x" kp="190" kv="16"/>
          <position name="my" joint="y" kp="190" kv="16"/>
          <position name="mz" joint="z" kp="190" kv="16"/>
        </actuator>
    </mujoco>""")
    materials(root)
    w = root.find("worldbody")
    w.find("geom[@name='floor']").set("material", "floor_mat")
    uav = w.find("body[@name='uav']")
    robot_meshes(root, uav, "drone")
    geom(uav, "gripper", "cylinder", (0, 0, -.133), (.035, .098), DARK)
    for side in (-1, 1):
        geom(uav, f"jaw{side}", pos=(side*.053, 0, -.207), size=(.025, .050, .025), rgba=WALL)
    pod = w.find("body[@name='pod']")
    geom(pod, "shell", pos=(0, 0, .053), size=(.155, .26, .051), rgba=ORANGE)
    geom(pod, "torso", "ellipsoid", (0, .027, .105), (.086, .109, .038), AMBER)
    geom(pod, "head", "sphere", (0, .155, .107), (.057,), AMBER)
    for side in (-1, 1):
        geom(pod, f"leg{side}", pos=(side*.044, -.123, .11), size=(.032, .083, .027), rgba=AMBER)
        geom(pod, f"arm{side}", pos=(side*.096, .030, .105), size=(.025, .071, .023), rgba=AMBER)
    geom(pod, "bar", pos=(0, 0, .136), size=(.142, .025, .015), rgba=WHITE)
    geom(w, "pad", pos=(0, 0, .004), size=(.56, .53, .004), rgba=BLUE)
    for side in (-1, 1):
        geom(w, f"pad_side{side}", pos=(side*.39, 0, .010), size=(.030, .42, .002), rgba=WHITE)
    geom(w, "pad_cross", pos=(0, 0, .010), size=(.39, .030, .002), rgba=WHITE)

    obstacles = []

    def wall(center, half, kind, color=WALL):
        obstacles.append((kind, center, half))
        geom(w, f"solid{len(obstacles)}", pos=center, size=half, rgba=color)

    def boundary(a, b, route=None):
        center = (xy(a)+xy(b))/2
        normal = 0 if a[0] != b[0] else 1
        tangent = 1-normal
        gate_half = .51 if route == "inspection" else .56

        def section(mid, span, lo, hi, kind, color):
            pos, half = [*center, (lo+hi)/2], [.075, .075, (hi-lo)/2]
            pos[tangent] += mid
            half[tangent] = span
            wall(pos, half, kind, color)

        if route is None:
            section(0., PITCH/2+.026, 0., WALL_TOP, "wall", WALL)
            return
        for side in (-1, 1):
            section(side*(PITCH/2+gate_half)/2, (PITCH/2-gate_half)/2+.014,
                    0., WALL_TOP, "wall", WALL)
        if route == "inspection":
            section(0., gate_half, 0., SLOT_SILL, "inspection", AMBER)
            section(0., gate_half, SLOT_LINTEL, WALL_TOP, "inspection", AMBER)
        elif route == "raised":
            section(0., gate_half, 0., RAISED_SILL, "raised", (.57, .47, .18, 1))
        elif route == "cargo":
            x, y = center
            body = ET.SubElement(w, "body", name="cargo_shutter", mocap="true", pos=f"{x} {y} 1.025")
            geom(body, "cargo_leaf", size=(gate_half, .075, 1.025), rgba=BLUE)
            for z in np.linspace(-.90, .90, 9):
                geom(body, f"shutter_rib{z:.3f}", pos=(0, 0, z), size=(gate_half, .084, .018), rgba=DARK)
            for side in (-1, 1):
                geom(w, f"cargo_jamb{side}", pos=(x+side*.572, y, 1.025),
                     size=(.030, .10, 1.025), rgba=BLUE)

    for y in range(SIDE):
        boundary((-1, y), (0, y))
        boundary((SIDE-1, y), (SIDE, y))
    for x in range(SIDE):
        boundary((x, -1), (x, 0))
        boundary((x, SIDE-1), (x, SIDE))
        route = "inspection" if x == layout.inlet else "cargo" if x == layout.outlet else None
        boundary((x, 0), (x, 1), route)
    for a, b in interior_edges():
        route = "raised" if (a, b) in layout.raised else "open" if (a, b) in layout.halls else None
        boundary(a, b, route)

    # Bay paint runs around (not through) the central pickup area.
    for i, room in enumerate(layout.rooms):
        x, y = xy(room)
        for side in (-1, 1):
            geom(w, f"bay{i}_rail{side}", pos=(x+side*.53, y, .005),
                 size=(.023, .57, .004), rgba=TEAL)
    for side in (-1, 1):
        x = side*4.48
        prop(root, asset_root(), "utility_box_01", f"cabinet{side}", (x, 0, 0), (.52, .61, .87))

    # This scale model is world geometry visible to both onboard cameras.
    # Wall segments and door gaps are generated from the same layout as the wing.
    geom(w, "plan_plinth", pos=(*PLAN, .016), size=(.81, .81, .016), rgba=WHITE)
    geom(w, "plan_surface", pos=(*PLAN, .034), size=(.767, .767, .004), rgba=DARK)

    def plan_xy(cell):
        return np.asarray(PLAN)+xy(cell, MODULE)-(0, 2*MODULE)

    def ink(name, pos, size, color, z=.046):
        return geom(w, "plan_"+name, pos=(*pos, z), size=(*size, .005), rgba=color)

    for i, pos in enumerate(layout.rooms):
        ink(f"room{i}", plan_xy(pos), (.063, .063), TEAL)
    ink("home", plan_xy(HOME), (.062, .062), BLUE)
    for a, b in (list(interior_edges()) + [((x, 0), (x, 1)) for x in range(SIDE)]):
        normal = 0 if a[0] != b[0] else 1
        tangent = 1-normal
        p = (plan_xy(a)+plan_xy(b))/2
        name = f"{a[0]}_{a[1]}_{b[0]}_{b[1]}"
        is_entry = a[1] == 0 and a[0] == layout.inlet
        is_exit = a[1] == 0 and a[0] == layout.outlet
        if is_entry or is_exit or (a, b) in layout.halls:
            for side in (-1, 1):
                mid, sz = p.copy(), [.010, .010]
                mid[tangent] += side*.11
                sz[tangent] = .037
                ink(f"jamb{name}_{side}", mid, sz, WHITE)
            if is_entry or is_exit or (a, b) in layout.raised:
                sz = [.015, .015]
                sz[tangent] = .068
                ink(name, p, sz, BLUE if is_exit else AMBER)
        else:
            sz = [.010, .010]
            sz[tangent] = .152
            ink(name, p, sz, WHITE)
    return ET.tostring(root, encoding="unicode"), obstacles

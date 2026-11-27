"""MuJoCo YAM v1 pouch sorting cell. All control enters through joint actuators."""

from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco as mj
import numpy as np

DT = 0.002
HOME = np.array([0.0, 1.5, 1.0, 0.0, 0.0, 0.0])
TABLE_Z = 0.165
COLORS = {
    "orange": ".95 .27 .045",
    "blue": ".06 .43 .96",
    "yellow": ".99 .73 .07",
    "violet": ".64 .19 .89",
}
SHAPES = {
    "oval": ("ellipsoid", (0.031, 0.019, 0.024)),
    "brick": ("box", (0.028, 0.019, 0.022)),
    "round": ("sphere", (0.024,)),
    "drum": ("cylinder", (0.023, 0.023)),
}
# x, y, base height, half-width, wall height, lane, net distance, net height
FIXTURES = {
    "orange": (0.06, 0.94, 0.025, 0.060, 0.092, "sweep_left", 0.0, 0.0),
    "yellow": (-0.13, -0.72, 0.265, 0.061, 0.092, "sweep_right", 0.0, 0.0),
    "blue": (0.92, -0.69, 0.115, 0.060, 0.092, "lob", 0.77, 0.626),
    "violet": (0.91, 0.40, 0.325, 0.061, 0.092, "lob", 0.68, 0.708),
}
MATERIALS = {
    "foam": (440.0, 1.25, 0.018, 1.0),
    "cork": (610.0, 0.90, 0.013, 0.93),
    "polymer": (880.0, 0.65, 0.009, 0.82),
}
NET_HALF_WIDTH = 0.21
def dimensions(obj):
    kind, axes = SHAPES[obj["shape"]]
    return kind, np.array(axes) * obj["scale"]


def envelope(kind, size, R):
    if kind == "box":
        return abs(R) @ size
    if kind == "cylinder":
        return size[0] * np.linalg.norm(R[:, :2], axis=1) + size[1] * abs(R[:, 2])
    if kind == "sphere":
        return np.full(3, size[0])
    return np.linalg.norm(R * size, axis=1)


PICKS = [
    (-0.92, 0.40),
    (-0.56, 0.52),
    (-0.20, 0.40),
    (0.12, 0.50),
    (0.46, 0.39),
    (0.80, 0.49),
]
SLEW = np.array([3.5, 4.2, 5.5, 6.0, 4.0, 4.0])


def scene(seed):
    import random

    rng = random.Random(seed)
    colors = list(COLORS)
    rng.shuffle(colors)
    shapes = list(SHAPES) + ["brick", "oval"]
    rng.shuffle(shapes)
    parcel_colors = colors + colors[:2]
    rng.shuffle(parcel_colors)
    bins = []
    for color, (x, y, z, w, h, lane, net_r, net_z) in FIXTURES.items():
        bins.append(
            dict(
                color=color,
                x=x + rng.uniform(-0.018, 0.018),
                y=y + rng.uniform(-0.018, 0.018),
                z=z + rng.uniform(-0.014, 0.014),
                w=w + rng.uniform(-0.005, 0.005),
                h=h + rng.uniform(-0.01, 0.01),
                lane=lane,
                net_r=net_r,
                net_z=net_z + (rng.uniform(-0.012, 0.012) if net_r else 0.0),
            )
        )
    objects = []
    materials = list(MATERIALS) * 2
    rng.shuffle(materials)
    for i, ((a, r), shape, color, material) in enumerate(
        zip(PICKS, shapes, parcel_colors, materials)
    ):
        a += rng.uniform(-0.055, 0.055)
        r += rng.uniform(-0.022, 0.022)
        obj = dict(
            id=i,
            color=color,
            shape=shape,
            material=material,
            scale=rng.uniform(0.83, 1.17),
            yaw=rng.uniform(-0.35, 0.35),
            x=max(0.24, r * np.cos(a)),
            y=r * np.sin(a),
        )
        kind, size = dimensions(obj)
        if kind == "box":
            volume = 8 * np.prod(size)
        elif kind == "cylinder":
            volume = 2 * np.pi * size[0] ** 2 * size[1]
        elif kind == "sphere":
            volume = 4 * np.pi * size[0] ** 3 / 3
        else:
            volume = 4 * np.pi * np.prod(size) / 3
        obj["mass"] = float(volume * MATERIALS[material][0])
        objects.append(obj)
    rng.shuffle(bins)
    return dict(objects=objects, bins=bins)


ROOT = Path(__file__).resolve().parent / "yam"


def add(parent, tag, **attrs):
    return ET.SubElement(parent, tag, {k: str(v) for k, v in attrs.items()})


def model_for(layout):
    tree = ET.fromstring((ROOT / "yam.xml").read_text().split("-->")[-1])
    tree.find("compiler").set("meshdir", str(ROOT.parent / "assets"))
    option = add(tree, "option", timestep=DT, integrator="implicit", cone="elliptic")
    add(option, "flag", autoreset="disable")
    world = tree.find("worldbody")
    for geom in world.iter("geom"):
        if geom.get("contype") != "0":
            geom.attrib.update(contype="4", conaffinity="3")
    for site in world.iter("site"):
        site.set("rgba", "0 0 0 0")
    for name, sign in (("tip_left", -1), ("tip_right", 1)):
        body = world.find(f".//body[@name='{name}']")
        origin = np.fromstring(body.get("pos"), sep=" ")
        # Inner faces coincide at the manufacturer's zero-gap grasp site.
        center = np.array([0, sign * 0.003, -0.1347]) - origin
        add(
            body,
            "geom",
            name=name + "_pad",
            type="box",
            pos=" ".join(map(str, center)),
            size=".014 .003 .013",
            rgba=".12 .14 .16 1",
            contype=4,
            conaffinity=3,
            friction="1.3 .012 .001",
            solref=".008 1",
        )
        body.find("geom").attrib.update(contype="0", conaffinity="0")
    for joint in world.iter("joint"):
        joint.set("damping", ".15" if joint.get("type") == "hinge" else "2")
        joint.set("armature", ".018" if joint.get("type") == "hinge" else ".004")
    add(world, "light", pos=".2 -1 1.6", dir="0 .5 -1")
    add(world, "light", pos="-.6 .5 1.4", dir=".2 -.3 -1")
    add(
        world,
        "geom",
        type="plane",
        name="floor",
        size="2 2 .05",
        rgba=".27 .32 .35 1",
        contype=1,
        conaffinity=6,
    )
    add(
        world,
        "geom",
        name="table",
        type="box",
        pos=f".41 0 {TABLE_Z - 0.018}",
        size=".207 .47 .018",
        rgba=".49 .54 .55 1",
        contype=1,
        conaffinity=6,
    )
    for x in (0.25, 0.57):
        for y in (-0.40, 0.40):
            add(
                world,
                "geom",
                type="box",
                pos=f"{x} {y} .074",
                size=".015 .015 .074",
                rgba=".25 .29 .32 1",
            )
    for dest in layout["bins"]:
        col = dest["color"]
        x, y, z, w, h = (dest[k] for k in ("x", "y", "z", "w", "h"))
        if dest["lane"] == "lob":
            normal = np.array([x, y]) / np.hypot(x, y)
            px, py = normal * dest["net_r"]
            height = dest["net_z"]
            net = add(
                world,
                "body",
                name=col + "_barrier",
                pos=f"{px} {py} 0",
                euler=f"0 0 {np.arctan2(y, x)}",
            )
            add(
                net,
                "geom",
                name="net_" + col,
                type="box",
                pos=f"0 0 {height / 2}",
                size=f".004 {NET_HALF_WIDTH} {height / 2}",
                rgba=".8 .88 .87 .06",
                contype=1,
                conaffinity=6,
            )
            for side in (-1, 1):
                add(
                    net,
                    "geom",
                    name=f"post_{col}_{side}",
                    type="cylinder",
                    pos=f"0 {side * NET_HALF_WIDTH} {height / 2}",
                    size=f".009 {height / 2}",
                    rgba=".88 .84 .67 1",
                    contype=1,
                    conaffinity=6,
                )
            for yline in np.linspace(-NET_HALF_WIDTH, NET_HALF_WIDTH, 13):
                add(
                    net,
                    "geom",
                    type="capsule",
                    fromto=f"0 {yline} 0 0 {yline} {height}",
                    size=".0017",
                    rgba=".82 .88 .84 1",
                    contype=0,
                    conaffinity=0,
                )
            for zline in np.linspace(0.035, height, 13):
                add(
                    net,
                    "geom",
                    type="capsule",
                    fromto=f"0 {-NET_HALF_WIDTH} {zline} 0 {NET_HALF_WIDTH} {zline}",
                    size=".004" if zline == height else ".0017",
                    rgba=".94 .91 .73 1",
                    contype=0,
                    conaffinity=0,
                )
        b = add(world, "body", pos=f"{x} {y} {z}")
        if z > 0.01:
            add(
                b,
                "geom",
                name=col + "_base",
                type="box",
                pos=f"0 0 {-z / 2}",
                size=f"{w * 0.75} {w * 0.75} {z / 2}",
                rgba=".28 .33 .37 1",
                contype=1,
                conaffinity=6,
            )
        for name, px, py, pz, sx, sy, sz in [
            ("bottom", 0, 0, 0.008, w, w, 0.008),
            ("left", -w, 0, h / 2, 0.004, w, h / 2),
            ("right", w, 0, h / 2, 0.004, w, h / 2),
            ("front", 0, -w, h / 2, w, 0.004, h / 2),
            ("back", 0, w, h / 2, w, 0.004, h / 2),
        ]:
            add(
                b,
                "geom",
                name=col + "_" + name,
                type="box",
                pos=f"{px} {py} {pz}",
                size=f"{sx} {sy} {sz}",
                rgba=COLORS[col] + (" 1" if name == "bottom" else " .48"),
                contype=1,
                conaffinity=6,
            )
    for obj in layout["objects"]:
        kind, size = dimensions(obj)
        b = add(
            world,
            "body",
            name=f"item{obj['id']}",
            euler=f"0 0 {obj['yaw']}",
            pos=f"{obj['x']} {obj['y']} {TABLE_Z + size[-1]}",
        )
        add(b, "freejoint")
        add(
            b,
            "geom",
            name=f"parcel{obj['id']}",
            type=kind,
            size=" ".join(map(str, size)),
            mass=obj["mass"],
            rgba=COLORS[obj["color"]] + " 1",
            condim="6",
            contype=2,
            conaffinity=7,
            priority=1,
            friction=f"{MATERIALS[obj['material']][1]} .018 .001",
            solref=f"{MATERIALS[obj['material']][2]} {MATERIALS[obj['material']][3]}",
        )
    actuators = add(tree, "actuator")
    for i, limit in enumerate([18, 24, 19, 8, 5, 5], 1):
        add(actuators, "motor", joint=f"joint{i}", ctrlrange=f"{-limit} {limit}")
    add(
        actuators,
        "position",
        joint="joint7",
        kp=1800,
        kv=20,
        forcerange="-18 18",
        ctrlrange="0 .04695",
    )
    return mj.MjModel.from_xml_string(ET.tostring(tree, encoding="unicode"))

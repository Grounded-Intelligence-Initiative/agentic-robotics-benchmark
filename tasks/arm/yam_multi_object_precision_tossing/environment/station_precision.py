"""Tabletop events using YAM rigid-body dynamics and bilateral grasp contact."""

import math
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import mujoco as mj
import numpy as np
from station import model_for, add, TABLE_Z

SIZES = ("small", "medium", "large")
WOOD_BIN_HALF_OPENING = 0.038


def size_geometry(mode, size):
    """Dimensions in meters for the piece and its receiving fixture."""
    index = SIZES.index(size)
    if mode == "ring":
        radius = (0.040, 0.045, 0.051)[index]
        return dict(
            rim_radius=radius,
            rim_half_thickness=0.007,
            hole_radius=radius - 0.007,
            post_radius=(0.027, 0.030, 0.035)[index],
        )
    if mode == "slot":
        return dict(
            disk_radius=(0.034, 0.039, 0.044)[index],
            disk_half_thickness=0.013,
            slot_half_width=(0.0245, 0.028, 0.035)[index],
            slot_half_length=0.079,
            slot_depth=0.146,
        )
    if mode == "wood" and size == "medium":
        return {}
    raise ValueError(f"unsupported fixture and size: {mode}, {size}")


def nominal_mass(mode, size):
    dims = size_geometry(mode, size)
    if mode == "ring":
        return 0.038 * dims["rim_radius"] / 0.045
    if mode == "slot":
        return 0.022 * (dims["disk_radius"] / 0.039) ** 2
    return 0.045


def layout(mode, seed, size):
    r = np.random.default_rng(seed)
    pickup = np.array([0.40, -0.15]) + r.uniform([-0.018, -0.025], [0.018, 0.025])
    if mode == "wood":
        angle = 0.414 + r.uniform(-0.16, 0.16)
        radius = r.uniform(1.04, 1.14)
    else:
        angle = r.uniform(-0.33, 0.36)
        radius = r.uniform(1.035, 1.085) if mode == "ring" else r.uniform(1.04, 1.15)
    goal = radius * np.array([np.cos(angle), np.sin(angle)])
    level_range = {
        "wood": (0.25, 0.31),
        "ring": (0.045, 0.068),
        "slot": (0.07, 0.10),
    }[mode]
    return dict(
        mode=mode,
        size=size,
        pickup=pickup,
        goal=goal,
        post_h=r.uniform(0.09, 0.135) if mode == "ring" else 0.0,
        level=r.uniform(*level_range),
        mass=nominal_mass(mode, size) * r.uniform(0.90, 1.10),
    )


def geometry(parent, name, kind, pos, size, color, **kw):
    return add(
        parent,
        "geom",
        name=name,
        type=kind,
        pos=" ".join(map(str, pos)),
        size=" ".join(map(str, size)),
        rgba=color,
        **kw,
    )


def build(spec):
    # Reuse the manufacturer's arm, pad and servo definitions from the original task.
    m = model_for({"objects": [], "bins": []})
    import tempfile

    with tempfile.TemporaryDirectory() as folder:
        path = Path(folder) / "scene.xml"
        mj.mj_saveLastXML(str(path), m)
        root = ET.parse(path).getroot()
    world = root.find("worldbody")
    mode = spec["mode"]
    dims = size_geometry(mode, spec["size"])
    x, y = spec["pickup"]
    gx, gy = spec["goal"]
    h = spec["level"]
    env = dict(contype=1, conaffinity=6)
    obj = dict(
        contype=2, conaffinity=5, condim=6, friction="1.1 .02 .001", solref=".009 1"
    )
    base = TABLE_Z
    if mode in ("ring", "slot"):
        # Contact all external hand meshes, including CAD fingers behind the pad boxes.
        for body_name in (
            "tip_left",
            "tip_right",
            "gripper",
            "camera_bracket",
            "camera_body",
            "camera_cover",
            "camera_cable_holder",
        ):
            body_node = world.find(f".//body[@name='{body_name}']")
            if body_node is None:
                continue
            for g in body_node.findall("geom"):
                if "mesh" in g.attrib:
                    g.attrib.update(
                        contype="4", conaffinity="3", name=body_name + "_shell"
                    )
    if mode in ("ring", "slot"):
        hulls = json.loads(Path(__file__).with_name("finger-hulls.json").read_text())
        for name, pieces in hulls.items():
            body = world.find(f".//body[@name='{name}']")
            shell = body.find(f"geom[@name='{name}_shell']")
            shell.attrib.update(contype="0", conaffinity="0")
            for index, piece in enumerate(pieces):
                label = f"{name}_hull{index}"
                add(root.find("asset"), "mesh", name=label, **piece)
                attrs = dict(shell.attrib)
                attrs.update(
                    mesh=label,
                    name=label,
                    contype="4",
                    conaffinity="3",
                    group="3",
                    rgba="0 0 0 0",
                    mass="0",
                )
                add(body, "geom", **attrs)
    if mode == "ring":
        radial = np.array([x, y]) / np.hypot(x, y)
        lateral = np.array([-radial[1], radial[0]])
        support_offset = dims["rim_radius"] * (0.037 / 0.045)
        for side in (-1, 1):
            support = np.array([x, y]) + side * support_offset * lateral
            geometry(
                world,
                f"cradle{side}",
                "cylinder",
                (*support, base + 0.044),
                (0.012, 0.044),
                ".28 .32 .36 1",
                **env,
            )
        base += 0.088
    if mode == "wood":
        wall = 0.003
        hc = WOOD_BIN_HALF_OPENING + wall
        target = add(world, "body", pos=f"{gx} {gy} {h}")
        for name, px, py, pz, sx, sy, sz in [
            ("floor", 0, 0, 0.005, hc, hc, 0.005),
            ("west", -hc, 0, 0.049, wall, hc, 0.049),
            ("east", hc, 0, 0.049, wall, hc, 0.049),
            ("south", 0, -hc, 0.049, hc, wall, 0.049),
            ("north", 0, hc, 0.049, hc, wall, 0.049),
        ]:
            panel = geometry(
                target,
                "bin_" + name,
                "box",
                (px, py, pz),
                (sx, sy, sz),
                ".32 .54 .84 .65",
                **env,
            )
            if name == "floor":
                # A rigid floor separates the receiving volume from its support below.
                panel.attrib.update(
                    priority="2", solref=".004 1", solimp=".98 .999 .001"
                )
        geometry(
            target,
            "bin_stand",
            "box",
            (0, 0, -h / 2),
            (hc - 0.005, hc - 0.005, h / 2),
            ".24 .28 .31 1",
            **env,
        )
    elif mode == "ring":
        geometry(
            world,
            "pedestal",
            "cylinder",
            (gx, gy, h / 2),
            (0.045, h / 2),
            ".25 .30 .35 1",
            **env,
        )
        geometry(
            world,
            "post",
            "cylinder",
            (gx, gy, h + spec["post_h"] / 2),
            (dims["post_radius"], spec["post_h"] / 2),
            ".92 .74 .25 1",
            **env,
        )
        geometry(
            world,
            "post_base",
            "cylinder",
            (gx, gy, h + 0.003),
            (0.080, 0.003),
            ".26 .46 .56 1",
            **env,
        )
    else:
        bearing = math.atan2(gy, gx)
        fixture = add(world, "body", pos=f"{gx} {gy} {h}", euler=f"0 0 {bearing}")
        # The disk fits edgewise. Flat-on, its diameter exceeds the slot width.
        half_width = dims["slot_half_width"]
        half_length = dims["slot_half_length"]
        half_depth = dims["slot_depth"] / 2
        wall = 0.003
        radial_wall = (wall, half_length + wall, half_depth)
        lateral_wall = (half_width + wall, wall, half_depth)
        for name, pos, size in [
            ("floor", (0, 0, -h / 2), (half_width, half_length + wall, h / 2)),
            ("left", (-half_width - wall, 0, half_depth), radial_wall),
            ("right", (half_width + wall, 0, half_depth), radial_wall),
            ("front", (0, -half_length - wall, half_depth), lateral_wall),
            ("back", (0, half_length + wall, half_depth), lateral_wall),
        ]:
            geometry(fixture, "slot_" + name, "box", pos, size, ".27 .61 .67 .60", **env)
    half_height = {
        "ring": dims.get("rim_half_thickness"),
        "slot": dims.get("disk_half_thickness"),
        "wood": 0.018,
    }[mode]
    z = base + half_height
    body = add(world, "body", name="play_piece", pos=f"{x} {y} {z}")
    add(body, "freejoint")
    if mode == "ring":
        radius = dims["rim_radius"]
        half_thickness = dims["rim_half_thickness"]
        tangent = (radius + half_thickness) * np.tan(np.pi / 24)
        for j in range(24):
            theta = 2 * np.pi * j / 24
            geo = geometry(
                body,
                f"rim{j}",
                "box",
                (radius * np.cos(theta), radius * np.sin(theta), 0),
                (half_thickness, tangent, half_thickness),
                ".92 .35 .20 1",
                mass=spec["mass"] / 24,
                **obj,
            )
            geo.set("euler", f"0 0 {theta}")
    elif mode == "slot":
        geometry(
            body,
            "foam_puck",
            "cylinder",
            (0, 0, 0),
            (dims["disk_radius"], dims["disk_half_thickness"]),
            ".86 .28 .43 1",
            mass=spec["mass"],
            **obj,
        )
        for face in (-1, 1):
            geometry(
                body,
                f"face{face}",
                "cylinder",
                (0, 0, face * dims["disk_half_thickness"]),
                (dims["disk_radius"] * (0.028 / 0.039), 0.0001),
                ".97 .78 .58 1",
                mass=0,
                contype=0,
                conaffinity=0,
            )
    else:
        geometry(
            body,
            "wood_block",
            "box",
            (0, 0, 0),
            (0.026, 0.018, 0.018),
            ".67 .40 .19 1",
            mass=spec["mass"],
            **obj,
        )
    model = mj.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
    if mode == "ring":
        model.geom_priority[:] = 1
        model.geom_solref[:] = [0.004, 1]
        model.geom_solimp[:, :3] = [0.98, 0.999, 0.001]
    return model

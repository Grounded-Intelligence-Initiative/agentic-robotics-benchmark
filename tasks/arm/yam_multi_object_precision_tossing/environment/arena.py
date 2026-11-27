"""Assemble an inspectable two-arm arena from the task's MuJoCo scene pieces."""

import argparse
from copy import deepcopy
import math
from pathlib import Path
import shutil
from tempfile import TemporaryDirectory
import xml.etree.ElementTree as ET

import mujoco as mj
import numpy as np


ENVIRONMENT = Path(__file__).resolve().parent

import station
import station_precision as precision


TABLE_HEIGHT = 0.72
LEFT_Y = 1.21
RIGHT_Y = -1.24

# Placement coordinates are local to the right arm. All seven pieces and their receivers coexist.
PRECISION_LAYOUT = (
    ("wood", "medium", (1.05, 0.66), (0.36, 0.63), 0.12, -20),
    ("ring", "small", (0.66, 0.59), (0.36, 0.43), 0.075, 0),
    ("ring", "medium", (0.83, 0.43), (0.34, 0.24), 0.135, 0),
    ("ring", "large", (1.04, 0.34), (0.38, 0.05), 0.19, 0),
    ("slot", "small", (0.66, -0.52), (0.35, -0.31), 0.075, 0),
    ("slot", "medium", (0.92, -0.64), (0.39, -0.49), 0.135, 34),
    ("slot", "large", (1.12, -0.42), (0.34, -0.67), 0.195, 89),
)

REFERENCES = ("name", "body", "body1", "body2", "joint", "joint1", "joint2",
              "geom", "geom1", "geom2", "site", "site1", "site2")


def vector(values):
    return " ".join(f"{float(value):.12g}" for value in values)


def snapshot(model, path):
    mj.mj_saveLastXML(str(path), model)
    return ET.parse(path).getroot()


def prefix_names(node, prefix):
    for part in node.iter():
        for name in REFERENCES:
            if name in part.attrib:
                part.set(name, prefix + part.get(name))


def offset(node, y, z):
    pos = np.fromstring(node.get("pos", "0 0 0"), sep=" ")
    node.set("pos", vector(pos + (0, y, z)))


def yaw(node, degrees):
    for field in ("quat", "axisangle", "xyaxes", "zaxis"):
        node.attrib.pop(field, None)
    node.set("euler", vector((0, 0, math.radians(degrees))))


def box(parent, name, pos, size, color):
    return station.add(parent, "geom", name=name, type="box", pos=vector(pos),
                       size=vector(size), rgba=color, contype=1, conaffinity=6)


def add_table(world, name, y, half_width, surface_color):
    half_length = 0.78
    x = 0.47
    box(world, f"{name}_table_top", (x, y, TABLE_HEIGHT - 0.025),
        (half_length, half_width, 0.025), surface_color)
    for i, leg_x in enumerate((x - half_length + 0.11, x + half_length - 0.11)):
        for j, leg_y in enumerate((y - half_width + 0.11, y + half_width - 0.11)):
            box(world, f"{name}_table_leg_{i}_{j}", (leg_x, leg_y, 0.335),
                (0.035, 0.035, 0.335), ".19 .24 .25 1")


def add_arms(root, world, source_world):
    source_arm = source_world.find("body[@name='base']")
    templates = {section: list(root.find(section)) for section in ("contact", "equality", "actuator")}
    for section in templates:
        root.find(section).clear()
    for side, location in (("left", LEFT_Y), ("right", RIGHT_Y)):
        arm = deepcopy(source_arm)
        offset(arm, location, TABLE_HEIGHT)
        prefix_names(arm, f"{side}_")
        world.append(arm)
        for section, children in templates.items():
            for prototype in children:
                node = deepcopy(prototype)
                prefix_names(node, f"{side}_")
                if section == "actuator":
                    node.set("name", node.get("joint") + "_drive")
                root.find(section).append(node)


def add_sorting(world, source_world):
    for body in source_world.findall("body"):
        name = body.get("name", "")
        if name == "base":
            continue
        node = deepcopy(body)
        height = TABLE_HEIGHT - station.TABLE_Z if name.startswith("item") else TABLE_HEIGHT
        offset(node, LEFT_Y, height)
        for color, angle in (("orange", -24), ("yellow", 22)):
            if node.find(f"geom[@name='{color}_bottom']") is not None:
                yaw(node, angle)
        prefix_names(node, "left_")
        world.append(node)
    orange_guide = box(world, "left_orange_guide", (0.14, LEFT_Y + 0.75, TABLE_HEIGHT + 0.035),
                       (0.065, 0.008, 0.035), ".32 .55 .48 1")
    yellow_guide = box(world, "left_yellow_guide", (0.09, LEFT_Y - 0.58, TABLE_HEIGHT + 0.060),
                       (0.075, 0.008, 0.060), ".37 .54 .43 1")
    yaw(orange_guide, 31)
    yaw(yellow_guide, -30)


def add_precision(world, folder, seed):
    rng = np.random.default_rng(seed)
    for index, (mode, size, goal, pickup, level, angle) in enumerate(PRECISION_LAYOUT):
        # The published nominal course is also a reproducible regression fixture.
        if seed != 17:
            pickup = np.asarray(pickup) + rng.uniform(-.009, .009, size=2)
            goal = np.asarray(goal) + rng.uniform(-.015, .015, size=2)
        name = "wood" if mode == "wood" else f"{mode}_{size}"
        spec = precision.layout(mode, seed + index, size)
        spec.update(goal=np.asarray(goal), pickup=np.asarray(pickup), level=level)
        source = snapshot(precision.build(spec), folder / "precision.xml").find("worldbody")
        receiving_geoms = {"pedestal", "post", "post_base"} if mode == "ring" else set()
        fixtures = {"wood": "bin_floor", "slot": "slot_floor"}
        markers = fixtures.get(mode)
        for node in source:
            node_name = node.get("name", "")
            if node_name == "play_piece" or node_name.startswith("cradle"):
                offset(node, RIGHT_Y, TABLE_HEIGHT - station.TABLE_Z)
            elif node_name in receiving_geoms:
                offset(node, RIGHT_Y, TABLE_HEIGHT)
            elif markers and node.find(f"geom[@name='{markers}']") is not None:
                node.set("name", "target")
                offset(node, RIGHT_Y, TABLE_HEIGHT)
                if mode in ("wood", "slot"):
                    yaw(node, angle)
            else:
                continue
            prefix_names(node, f"right_{name}_")
            world.append(node)


def add_lighting(root, world):
    visual = station.add(root, "visual")
    station.add(visual, "headlight", ambient=".39 .40 .42", diffuse=".7 .72 .74",
                specular=".16 .17 .18")
    station.add(visual, "global", offwidth=1440, offheight=1000, azimuth=5, elevation=-52)
    station.add(visual, "map", znear=.01, zfar=35, fogend=10)
    station.add(world, "light", name="left_fill", pos="-1 1.8 3.6", dir=".45 -.1 -1",
                diffuse=".78 .78 .71", castshadow="true")
    station.add(world, "light", name="right_fill", pos="1.7 -2.1 3.1", dir="-.2 .1 -1",
                diffuse=".64 .71 .79", castshadow="false")


def compile_xml(root):
    return mj.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))


def build_arena(seed=17):
    """Return the combined model and its exportable MJCF document."""
    with TemporaryDirectory() as temp:
        folder = Path(temp)
        root = snapshot(station.model_for(station.scene(seed)), folder / "sorting.xml")
        root.set("model", "YAM dual-arm arena preview")
        source_world = deepcopy(root.find("worldbody"))
        world = root.find("worldbody")
        world.clear()
        floor = source_world.find("geom[@name='floor']")
        floor.attrib.update(size="12 12 .05", rgba=".36 .40 .39 1")
        world.append(floor)
        add_lighting(root, world)
        add_table(world, "left", LEFT_Y, 1.12, ".52 .65 .59 1")
        add_table(world, "right", RIGHT_Y, 0.85, ".54 .61 .72 1")
        add_arms(root, world, source_world)
        add_sorting(world, source_world)
        add_precision(world, folder, seed)

    model = compile_xml(root)
    state = mj.MjData(model)
    for side in ("left", "right"):
        for index, joint_value in enumerate((*station.HOME, .025, .025), start=1):
            joint = model.joint(f"{side}_joint{index}")
            state.qpos[joint.qposadr[0]] = joint_value
        state.ctrl[model.actuator(f"{side}_joint7_drive").id] = .025
    keyframes = station.add(root, "keyframe")
    station.add(keyframes, "key", name="preview", qpos=vector(state.qpos), ctrl=vector(state.ctrl))
    return compile_xml(root), root


def preview_data(model):
    data = mj.MjData(model)
    mj.mj_resetDataKeyframe(model, data, model.key("preview").id)
    mj.mj_forward(model, data)
    return data


def export_mjcf(root, folder):
    compiler = root.find("compiler")
    source_assets = Path(compiler.get("meshdir"))
    assets = folder / "assets"
    assets.mkdir(parents=True, exist_ok=True)
    for mesh in root.findall("asset/mesh"):
        filename = Path(mesh.get("file"))
        target = assets / filename
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source_assets / filename, target)
    compiler.set("meshdir", "assets")
    ET.indent(root)
    ET.ElementTree(root).write(folder / "arena.xml", encoding="utf-8", xml_declaration=True)
    for name in ("LICENSE", "SOURCE.md"):
        shutil.copy2(ENVIRONMENT / "yam" / name, folder / name)


def render_views(model, folder):
    from PIL import Image

    data = preview_data(model)
    views = [
        ("overview", (0.47, -0.05, 0.81), 4.45, 5, -52),
        ("overhead", (0.47, -0.05, 0.76), 4.38, 0, -89.9),
        ("left", (0.48, LEFT_Y, 0.87), 3.08, 14, -47),
        ("right", (0.47, RIGHT_Y, 0.87), 2.77, -16, -52),
    ]
    with mj.Renderer(model, height=830, width=1440) as renderer:
        for name, lookat, distance, azimuth, elevation in views:
            camera = mj.MjvCamera()
            camera.type = mj.mjtCamera.mjCAMERA_FREE
            camera.lookat[:] = lookat
            camera.distance, camera.azimuth, camera.elevation = distance, azimuth, elevation
            renderer.update_scene(data, camera=camera)
            Image.fromarray(renderer.render()).save(folder / f"{name}.png")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    parser.add_argument("--seed", type=int, default=17)
    args = parser.parse_args()
    model, root = build_arena(args.seed)
    export_mjcf(root, args.output)
    render_views(model, args.output)


if __name__ == "__main__":
    main()

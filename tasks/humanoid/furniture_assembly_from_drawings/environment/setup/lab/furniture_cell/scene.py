"""Build the cell's MuJoCo model: the room, the sideboard, the kitting table, the panel cart, the customer's couch,
plant and rug, the bookcase parts as free bodies, two G1 humanoids, the inactive weld pool (hand grips, dowel joints,
the back panel's seat) and the cameras.

Collision bits: 1 = static world and parts, 2 = robot a, 4 = robot b, 8 = floor. Parts touch everything; robots
touch the world, the parts and each other, never the floor (the locomotion abstraction carries them) or themselves.
"""

from __future__ import annotations

import math

import mujoco
import numpy as np

from . import furniture as F
from . import room as RM
from .robot import FLOOR, OBJ, ROBOT_BITS, g1_spec

MM = 1e-3
PART_FRICTION = [0.55, 0.005, 0.0001]
RAW = [0.66, 0.55, 0.42, 1.0]          # raw particleboard edge
WHITE = [0.93, 0.93, 0.91, 1.0]        # melamine face / edge banding
HARDBOARD = [0.47, 0.33, 0.22, 1.0]    # raw hardboard
HOLE = [0.12, 0.10, 0.08, 1.0]
DOWEL = [0.80, 0.68, 0.48, 1.0]
ROBOTS = ("a", "b")


def _quat_z(yaw: float) -> list[float]:
    return [math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)]


def _look_at_quat(pos, target) -> list[float]:
    """MuJoCo camera orientation (looks along its -z, +y up) looking from pos at target, horizon level."""
    pos, target = np.asarray(pos, float), np.asarray(target, float)
    zc = pos - target
    zc /= np.linalg.norm(zc)
    xc = np.cross([0.0, 0.0, 1.0], zc)
    xc /= np.linalg.norm(xc)
    yc = np.cross(zc, xc)
    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, np.stack([xc, yc, zc], axis=1).reshape(-1))
    return list(q)


def _box(body, name, center_mm, half_mm, rgba, *, collide: bool, mass=None, material=None):
    kw = dict(name=name, type=mujoco.mjtGeom.mjGEOM_BOX, size=[h * MM for h in half_mm],
              pos=[c * MM for c in center_mm], rgba=rgba)
    if material:
        kw["material"] = material
    if collide:
        kw.update(contype=OBJ, conaffinity=OBJ | ROBOT_BITS["a"] | ROBOT_BITS["b"] | FLOOR,
                  friction=PART_FRICTION, condim=3, group=0)
        if mass is not None:
            kw["mass"] = mass
    else:
        kw.update(contype=0, conaffinity=0, group=1, mass=0.0)
    return body.add_geom(**kw)


def add_part(spec: mujoco.MjSpec, part: F.Part, pos, quat) -> None:
    """One bookcase part as a free body: collision boxes (raw colour), thin visual skins for the white faces and the
    banded edge, dark discs for the holes and wooden pegs for the dowels."""
    b = spec.worldbody.add_body(name=f"part_{part.key}", pos=list(pos), quat=list(quat))
    b.add_freejoint(name=f"part_{part.key}_free")
    total = sum(8 * bx.half[0] * bx.half[1] * bx.half[2] for bx in part.boxes)
    raw = HARDBOARD if part.material == "hardboard" else RAW
    for i, bx in enumerate(part.boxes):
        vol = 8 * bx.half[0] * bx.half[1] * bx.half[2]
        _box(b, f"part_{part.key}_c{i}", bx.center, bx.half, raw, collide=True, mass=part.mass_kg * vol / total)
    hx, hy, hz = part.half
    skin = 0.25                                        # mm, visual skins sit just outside the collision box
    if part.material == "particleboard":
        thin = int(np.argmin(part.size))               # the board's thickness axis
        for sgn in (-1, 1):
            c = [0.0, 0.0, 0.0]
            h = [hx, hy, hz]
            c[thin] = sgn * (part.half[thin] + skin)
            h[thin] = skin
            if part.key in ("L", "R") and ((sgn > 0) == (part.key == "L")):
                # the inner face stops at the rabbet
                c[1], h[1] = -F.RABBET_W / 2, hy - F.RABBET_W / 2
            _box(b, f"part_{part.key}_face{sgn:+d}", c, h, WHITE, collide=False)
    if part.banded_face:
        ax = "xyz".index(part.banded_face[1])
        sgn = -1 if part.banded_face[0] == "-" else 1
        c = [0.0, 0.0, 0.0]
        h = [hx + skin, hy + skin, hz + skin]
        c[ax] = sgn * (part.half[ax] + skin)
        h[ax] = skin
        _box(b, f"part_{part.key}_band", c, h, WHITE, collide=False)
    if part.finished_face:
        ax = "xyz".index(part.finished_face[1])
        sgn = -1 if part.finished_face[0] == "-" else 1
        c = [0.0, 0.0, 0.0]
        h = [hx, hy, hz]
        c[ax] = sgn * (part.half[ax] + skin)
        h[ax] = skin
        _box(b, f"part_{part.key}_white", c, h, WHITE, collide=False)
    for i, hole in enumerate(part.holes):
        a = np.array(hole.axis)
        p = np.array(hole.pos) - a * (skin * 2.5)        # just proud of the face skin
        q = np.zeros(4)
        mujoco.mju_quatZ2Vec(q, a)
        b.add_geom(name=f"part_{part.key}_hole{i}", type=mujoco.mjtGeom.mjGEOM_CYLINDER,
                   size=[F.DOWEL_D / 2 * MM, 0.4 * MM, 0], pos=list(p * MM), quat=list(q), rgba=HOLE,
                   contype=0, conaffinity=0, group=1, mass=0.0)
    for i, dw in enumerate(part.dowels):
        a = np.array(dw.axis)
        p = np.array(dw.pos) + a * (F.DOWEL_OUT / 2)
        q = np.zeros(4)
        mujoco.mju_quatZ2Vec(q, a)
        b.add_geom(name=f"part_{part.key}_dowel{i}", type=mujoco.mjtGeom.mjGEOM_CYLINDER,
                   size=[F.DOWEL_D / 2 * MM, F.DOWEL_OUT / 2 * MM, 0], pos=list(p * MM), quat=list(q), rgba=DOWEL,
                   contype=0, conaffinity=0, group=1, mass=0.0)


def build(layout: dict, *, offscreen=(960, 720)) -> mujoco.MjSpec:
    """The cell spec for one episode layout: {"parts": {key: (pos, quat)}, "robots": {name: (x, y, yaw)}}."""
    s = mujoco.MjSpec()
    s.modelname = "furniture_cell"
    o = s.option
    o.timestep = 0.002
    o.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    o.cone = mujoco.mjtCone.mjCONE_PYRAMIDAL
    o.iterations = 60
    o.ls_iterations = 30
    o.disableflags |= mujoco.mjtDisableBit.mjDSBL_MULTICCD
    s.memory = 256 * 1024 * 1024
    s.visual.global_.offwidth, s.visual.global_.offheight = offscreen
    s.visual.quality.shadowsize = 2048
    s.visual.headlight.ambient = [0.30, 0.30, 0.30]
    s.visual.headlight.diffuse = [0.35, 0.35, 0.35]
    s.visual.headlight.specular = [0.0, 0.0, 0.0]
    s.visual.map.znear = 0.01
    wb = s.worldbody
    # materials
    s.add_texture(name="floor_tex", type=mujoco.mjtTexture.mjTEXTURE_2D, builtin=mujoco.mjtBuiltin.mjBUILTIN_CHECKER,
                  rgb1=[0.62, 0.58, 0.52], rgb2=[0.58, 0.54, 0.48], width=512, height=512)
    s.add_material(name="floor_mat", textures=["", "floor_tex"], texrepeat=[12, 10], reflectance=0.0)
    s.add_material(name="wall_mat", rgba=[0.86, 0.85, 0.82, 1.0])
    s.add_material(name="sideboard_mat", rgba=[0.30, 0.31, 0.33, 1.0])
    s.add_material(name="table_mat", rgba=[0.55, 0.40, 0.26, 1.0])
    s.add_material(name="couch_mat", rgba=[0.15, 0.14, 0.13, 1.0])            # dark brown leather
    s.add_material(name="couch_leg_mat", rgba=[0.68, 0.69, 0.70, 1.0])        # brushed metal
    s.add_material(name="plant_pot_mat", rgba=[0.17, 0.15, 0.15, 1.0])        # charcoal pot
    s.add_material(name="plant_foliage_mat", rgba=[0.34, 0.42, 0.15, 1.0])    # leaves
    s.add_material(name="rug_mat", rgba=[0.33, 0.34, 0.35, 1.0])              # grey wool
    # lights: two ceiling lamps
    for i, (lx, ly) in enumerate(((1.8, 2.0), (4.0, 2.2))):
        wb.add_light(name=f"lamp{i}", pos=[lx, ly, RM.WALL_H - 0.05], dir=[0.0, 0.0, -1.0],
                     type=mujoco.mjtLightType.mjLIGHT_SPOT, cutoff=75.0, exponent=1.0, castshadow=(i == 0),
                     diffuse=[0.55, 0.55, 0.52], specular=[0.05, 0.05, 0.05], attenuation=[1.0, 0.0, 0.0])
    # floor and walls
    wb.add_geom(name="floor", type=mujoco.mjtGeom.mjGEOM_PLANE, size=[RM.ROOM_X, RM.ROOM_Y, 0.05],
                pos=[RM.ROOM_X / 2, RM.ROOM_Y / 2, 0.0], material="floor_mat", contype=FLOOR, conaffinity=OBJ,
                friction=[0.6, 0.005, 0.0001])
    t = RM.WALL_T
    walls = {
        "wall_w": ([-t / 2, RM.ROOM_Y / 2, RM.WALL_H / 2], [t / 2, RM.ROOM_Y / 2 + t, RM.WALL_H / 2]),
        "wall_e": ([RM.ROOM_X + t / 2, RM.ROOM_Y / 2, RM.WALL_H / 2], [t / 2, RM.ROOM_Y / 2 + t, RM.WALL_H / 2]),
        "wall_n": ([RM.ROOM_X / 2, RM.ROOM_Y + t / 2, RM.WALL_H / 2], [RM.ROOM_X / 2, t / 2, RM.WALL_H / 2]),
    }
    d0, d1 = RM.DOOR_X
    walls["wall_s1"] = ([d0 / 2, -t / 2, RM.WALL_H / 2], [d0 / 2, t / 2, RM.WALL_H / 2])
    walls["wall_s2"] = ([(d1 + RM.ROOM_X) / 2, -t / 2, RM.WALL_H / 2], [(RM.ROOM_X - d1) / 2, t / 2, RM.WALL_H / 2])
    walls["wall_s3"] = ([(d0 + d1) / 2, -t / 2, (RM.WALL_H + 2.05) / 2], [(d1 - d0) / 2, t / 2, (RM.WALL_H - 2.05) / 2])
    for name, (c, h) in walls.items():
        wb.add_geom(name=name, type=mujoco.mjtGeom.mjGEOM_BOX, size=h, pos=c, material="wall_mat", contype=OBJ,
                    conaffinity=OBJ | ROBOT_BITS["a"] | ROBOT_BITS["b"], friction=[0.4, 0.005, 0.0001])
    w0, w1 = RM.WINDOW_Y
    wb.add_geom(name="window", type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.005, (w1 - w0) / 2, 0.6],
                pos=[RM.ROOM_X - 0.004, (w0 + w1) / 2, 1.5], rgba=[0.70, 0.82, 0.92, 1.0], contype=0, conaffinity=0)
    # sideboard
    sb = RM.SIDEBOARD
    sx, sy, sz = sb["size"]
    wb.add_geom(name="sideboard", type=mujoco.mjtGeom.mjGEOM_BOX, size=[sx / 2, sy / 2, sz / 2],
                pos=[sb["center"][0], sb["center"][1], sz / 2], material="sideboard_mat", contype=OBJ,
                conaffinity=OBJ | ROBOT_BITS["a"] | ROBOT_BITS["b"], friction=[0.6, 0.005, 0.0001])
    # kitting table: top, legs, two battens along x
    tb = RM.TABLE
    tx, ty = tb["center"]
    lx, ly = tb["size"]
    th = tb["height"]
    wb.add_geom(name="table_top", type=mujoco.mjtGeom.mjGEOM_BOX, size=[lx / 2, ly / 2, 0.015],
                pos=[tx, ty, th - 0.015], material="table_mat", contype=OBJ,
                conaffinity=OBJ | ROBOT_BITS["a"] | ROBOT_BITS["b"], friction=[0.6, 0.005, 0.0001])
    for i, (sxl, syl) in enumerate(((-1, -1), (-1, 1), (1, -1), (1, 1))):
        wb.add_geom(name=f"table_leg{i}", type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.025, 0.025, (th - 0.03) / 2],
                    pos=[tx + sxl * (lx / 2 - 0.05), ty + syl * (ly / 2 - 0.05), (th - 0.03) / 2],
                    rgba=[0.25, 0.25, 0.25, 1.0], contype=OBJ, conaffinity=OBJ | ROBOT_BITS["a"] | ROBOT_BITS["b"])
    for i, sgn in enumerate((-1, 1)):
        wb.add_geom(name=f"table_batten{i}", type=mujoco.mjtGeom.mjGEOM_BOX,
                    size=[lx / 2 - 0.02, tb["batten_w"] / 2, tb["batten_h"] / 2],
                    pos=[tx, ty + sgn * tb["batten_dy"], th + tb["batten_h"] / 2], rgba=[0.42, 0.42, 0.40, 1.0],
                    contype=OBJ, conaffinity=OBJ | ROBOT_BITS["a"] | ROBOT_BITS["b"], friction=[0.6, 0.005, 0.0001])
    # panel cart: a bed block per groove, two rails on it
    pc = RM.PANEL_CART
    for i, ys in enumerate(pc["slots_y"]):
        wb.add_geom(name=f"cart_bed{i}", type=mujoco.mjtGeom.mjGEOM_BOX,
                    size=[pc["rail_len"] / 2, pc["bed_w"] / 2, pc["bed_h"] / 2], pos=[pc["x"], ys, pc["bed_h"] / 2],
                    rgba=[0.24, 0.30, 0.38, 1.0], contype=OBJ, conaffinity=OBJ | ROBOT_BITS["a"] | ROBOT_BITS["b"],
                    friction=[0.5, 0.005, 0.0001])
        gap = layout.get("groove_gap", {}).get(i, 0.020)
        for j, sgn in enumerate((-1, 1)):
            wb.add_geom(name=f"cart_rail{i}{j}", type=mujoco.mjtGeom.mjGEOM_BOX,
                        size=[pc["rail_len"] / 2, pc["rail_w"] / 2, pc["rail_h"] / 2],
                        pos=[pc["x"], ys + sgn * (gap / 2 + pc["rail_w"] / 2), pc["bed_h"] + pc["rail_h"] / 2],
                        rgba=[0.35, 0.33, 0.30, 1.0], contype=OBJ, conaffinity=OBJ | ROBOT_BITS["a"] | ROBOT_BITS["b"],
                        friction=[0.05, 0.005, 0.0001], priority=1)   # felt-lined rails: their friction wins
    # the customer's couch and plant (static obstacles, like the sideboard) and rug (drawn only: no contact)
    still = dict(contype=OBJ, conaffinity=OBJ | ROBOT_BITS["a"] | ROBOT_BITS["b"], friction=[0.6, 0.005, 0.0001])
    for name, (x0, y0, z0, x1, y1, z1) in RM.COUCH["boxes"].items():
        wb.add_geom(name=f"couch_{name}", type=mujoco.mjtGeom.mjGEOM_BOX,
                    size=[(x1 - x0) / 2, (y1 - y0) / 2, (z1 - z0) / 2],
                    pos=[(x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2], material="couch_mat", **still)
    legs = RM.COUCH["legs"]
    for i, (cx, cy) in enumerate(legs["centers"]):
        wb.add_geom(name=f"couch_leg{i}", type=mujoco.mjtGeom.mjGEOM_BOX,
                    size=[legs["side"] / 2, legs["side"] / 2, legs["height"] / 2], pos=[cx, cy, legs["height"] / 2],
                    material="couch_leg_mat", **still)
    for name, cyl in RM.PLANT.items():
        z0, z1 = cyl["z"]
        wb.add_geom(name=f"plant_{name}", type=mujoco.mjtGeom.mjGEOM_CYLINDER, size=[cyl["radius"], (z1 - z0) / 2, 0.0],
                    pos=[cyl["center"][0], cyl["center"][1], (z0 + z1) / 2], material=f"plant_{name}_mat", **still)
    rug = RM.RUG
    wb.add_geom(name="rug", type=mujoco.mjtGeom.mjGEOM_BOX,
                size=[rug["size"][0] / 2, rug["size"][1] / 2, rug["thickness"] / 2],
                pos=[rug["center"][0], rug["center"][1], rug["thickness"] / 2], material="rug_mat",
                contype=0, conaffinity=0, group=1)
    # parts
    for key, part in F.PARTS.items():
        pos, quat = layout["parts"][key]
        add_part(s, part, pos, quat)
    # robots
    # the base joints are world-aligned (base_x, base_y, base_yaw = the robot's pose in the room); the episode's start
    # pose is written into qpos at reset
    for name in ROBOTS:
        frame = wb.add_frame(name=f"robot_{name}_frame", pos=[0.0, 0.0, 0.0])
        s.attach(g1_spec(name), prefix=f"{name}/", frame=frame)
    # ceiling cameras
    for cname, c in RM.CEILING_CAMERAS.items():
        wb.add_camera(name=cname, pos=list(c["pos"]), quat=_look_at_quat(c["pos"], c["target"]), fovy=c["fovy"])
    # inactive weld pool: every hand x every part (grips), host x guest (dowel joints), back panel x side (seat)
    weld = dict(type=mujoco.mjtEq.mjEQ_WELD, objtype=mujoco.mjtObj.mjOBJ_BODY, active=False,
                solref=[0.01, 1.0], solimp=[0.95, 0.99, 0.001, 0.5, 2.0],
                data=[0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1.0])
    for name in ROBOTS:
        for side in ("left", "right"):
            for key in F.PARTS:
                s.add_equality(name=f"grip_{name}_{side}_{key}", name1=f"{name}/{side}_wrist_yaw_link",
                               name2=f"part_{key}", **weld)
    for host in ("L", "R"):
        for guest in ("T", "C", "D"):
            s.add_equality(name=f"joint_{host}_{guest}", name1=f"part_{host}", name2=f"part_{guest}", **weld)
        s.add_equality(name=f"joint_{host}_K", name1=f"part_{host}", name2="part_K", **weld)
    return s

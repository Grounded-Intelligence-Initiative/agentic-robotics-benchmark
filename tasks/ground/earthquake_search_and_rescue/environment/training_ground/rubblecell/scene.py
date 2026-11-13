"""The MuJoCo model of a rubble site with the crawler on it (cell internals): materials and textures, the painted
heightfield, the static geometry and the movable debris of site.py, the survivor and the decoys, the robot (robot.py)
with its survival kit, the grip weld and the props' holding welds, the sun and the haze.
"""

import math

import mujoco
import numpy as np

from .robot import CRAWLER, build_crawler, _q_axis
from .site import HF_BASE, HF_RES, MAT, build_site
from .textures import TILE_M, fbm, make_textures

__all__ = ["build_scene", "Scene", "GROUP_WORLD", "GROUP_VISUAL"]

GROUP_WORLD = 0
GROUP_VISUAL = 2
CT_WORLD, CA_WORLD = 1, 3          # the world touches the robot (contype 2) and the debris
_TEX_CACHE = {}


def _textures():
    if "T" not in _TEX_CACHE:
        _TEX_CACHE["T"] = make_textures(256)
    return _TEX_CACHE["T"]


def paint_ground(S, px_per_m=48):
    """The heightfield's colour image: road, sidewalk, floors, rubble mounds, pits; dust over the building."""
    T = _textures()
    W = int(round((S.hx1 - S.hx0) * px_per_m))
    H = int(round((S.hy1 - S.hy0) * px_per_m))
    xs = S.hx0 + (np.arange(W) + 0.5) / px_per_m
    ys = S.hy0 + (np.arange(H) + 0.5) / px_per_m
    X, Y = np.meshgrid(xs, ys)
    rng = np.random.default_rng(int(S.plan["seed"]) & 0xFFFFFFFF)
    def lowres(cells, octaves):
        # smooth noise at a quarter of the image's resolution, upsampled (the noise is far smoother than a pixel)
        h4, w4 = max(8, H // 4), max(8, W // 4)
        f4 = fbm((h4, w4), cells, octaves, rng, periodic=False)
        from PIL import Image as _I
        return np.asarray(_I.fromarray((f4 * 65535).astype(np.uint16)).resize((W, H), _I.BILINEAR), dtype=np.float32) / 65535.0

    jit = (lowres(24, 3) - 0.5) * 0.12
    gi = np.clip(np.round((Y + jit - S.hy0) / HF_RES).astype(int), 0, S.hny - 1)
    gj = np.clip(np.round((X - jit - S.hx0) / HF_RES).astype(int), 0, S.hnx - 1)
    cls = S.ground[gi, gj]
    hgt = S.H[gi, gj]
    out = np.zeros((H, W, 3), np.float32)

    def tile(name):
        t = T[name].astype(np.float32) / 255.0
        n = t.shape[0]
        sc = TILE_M.get(name, 1.0)
        u = ((X / sc) * n).astype(int) % n
        v = ((Y / sc) * n).astype(int) % n
        return t[v, u]

    layers = {0: tile("asphalt"), 1: tile("paving"), 2: tile("concrete"), 3: tile("tiles"), 4: tile("rubble")}
    for c, img in layers.items():
        m = cls == c
        out[m] = img[m]
    m5 = cls == 5
    out[m5] = layers[2][m5] * 0.25
    # rubble mounds: blend by height so the mound's foot fades into the floor
    a = np.clip(hgt / 0.15, 0, 1)[..., None] * (cls == 4)[..., None]
    out = out * (1 - a) + layers[4] * a
    # dust over the building and the sidewalk next to it
    dust = T["dust"].astype(np.float32) / 255.0
    n = dust.shape[0]
    du = dust[((Y / 1.5) * n).astype(int) % n, ((X / 1.5) * n).astype(int) % n]
    f = lowres(10, 4)
    inside = ((X > -0.5) & (X < S.W + 0.5) & (Y > -2.5) & (Y < S.D + 0.5)).astype(np.float32)
    k = (0.25 + 0.45 * f) * inside
    out = out * (1 - k[..., None]) + du * k[..., None]
    img = (np.clip(out, 0, 1) * 255).astype(np.uint8)
    return img[::-1].copy()          # texture row 0 = the north edge


class Scene(object):
    """The compiled model + data and the site; names of the elements the cell uses."""

    def __init__(self, site, model, data, info):
        self.site = site
        self.model = model
        self.data = data
        self.info = info


def build_scene(plan, offscreen=(640, 480)):
    S = build_site(plan)
    T = _textures()
    spec = mujoco.MjSpec()
    spec.compiler.degree = False
    spec.option.timestep = 0.002
    spec.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    spec.option.cone = mujoco.mjtCone.mjCONE_ELLIPTIC
    spec.memory = 256 * 1024 * 1024
    v = spec.visual
    v.global_.offwidth = max(1280, offscreen[0])
    v.global_.offheight = max(960, offscreen[1])
    v.quality.shadowsize = 4096
    v.map.znear = 0.01
    v.map.zfar = 80.0
    env = plan["env"]
    v.map.fogstart = 4.0 + 8.0 * (1.0 - env["haze"])
    v.map.fogend = 26.0 + 30.0 * (1.0 - env["haze"])
    v.rgba.fog = [0.74, 0.70, 0.64, 1.0]
    v.headlight.ambient = [0.20, 0.20, 0.20]
    v.headlight.diffuse = [0.0, 0.0, 0.0]
    v.headlight.specular = [0.0, 0.0, 0.0]
    # sky
    sky = spec.add_texture()
    sky.name = "sky"
    sky.type = mujoco.mjtTexture.mjTEXTURE_SKYBOX
    sky.builtin = mujoco.mjtBuiltin.mjBUILTIN_GRADIENT
    sky.rgb1 = [0.78, 0.80, 0.82]
    sky.rgb2 = [0.55, 0.60, 0.66]
    sky.width = 512
    sky.height = 512
    # textures and materials
    for name, img in T.items():
        t = spec.add_texture()
        t.name = "tx_" + name
        t.type = mujoco.mjtTexture.mjTEXTURE_2D
        t.width, t.height = img.shape[1], img.shape[0]
        t.nchannel = 3
        t.data = img.tobytes()
    for name, (tex, rgba, spec_, shin, fr) in MAT.items():
        m = spec.add_material()
        m.name = name
        if tex is not None:
            m.textures[mujoco.mjtTextureRole.mjTEXROLE_RGB] = "tx_" + tex
            m.texuniform = True
            rep = 1.0 / TILE_M.get(tex, 1.0)
            m.texrepeat = [rep, rep]
        m.rgba = list(rgba)
        m.specular = spec_
        m.shininess = shin
    # the ground: heightfield + painted image
    img = paint_ground(S)
    gt = spec.add_texture()
    gt.name = "tx_ground"
    gt.type = mujoco.mjtTexture.mjTEXTURE_2D
    gt.width, gt.height = img.shape[1], img.shape[0]
    gt.nchannel = 3
    gt.data = img.tobytes()
    gm = spec.add_material()
    gm.name = "ground"
    gm.textures[mujoco.mjtTextureRole.mjTEXROLE_RGB] = "tx_ground"
    gm.texuniform = False
    gm.texrepeat = [1, 1]
    gm.specular = 0.02
    gm.shininess = 0.05
    # MuJoCo normalises the elevation data to [0, 1] by its own min / max: the geom sits at the data's minimum and
    # size[2] is the data's range, so every elevation keeps its height
    zmin = float(S.H.min())
    zmax = float(S.H.max())
    if zmax - zmin < 1e-3:
        zmax = zmin + 1e-3
    hf = spec.add_hfield()
    hf.name = "ground"
    hf.nrow = S.hny
    hf.ncol = S.hnx
    hf.size = [(S.hx1 - S.hx0) / 2.0, (S.hy1 - S.hy0) / 2.0, zmax - zmin, 0.1]
    hf.userdata = list(((S.H - zmin) / (zmax - zmin)).astype(np.float32).flatten())
    g = spec.worldbody.add_geom()
    g.name = "ground"
    g.type = mujoco.mjtGeom.mjGEOM_HFIELD
    g.hfieldname = "ground"
    g.material = "ground"
    g.pos = [(S.hx0 + S.hx1) / 2.0, (S.hy0 + S.hy1) / 2.0, zmin]
    g.contype, g.conaffinity = CT_WORLD, CA_WORLD
    g.priority = 1
    g.friction = [0.8, 0.02, 0.001]
    g.group = GROUP_WORLD
    # meshes (rubble chunks)
    for name, verts in S.meshes.items():
        me = spec.add_mesh()
        me.name = name
        me.uservert = list(np.asarray(verts, float).flatten())
    # static geometry
    for st in S.statics:
        g = spec.worldbody.add_geom()
        g.name = st["name"]
        t = st["type"]
        g.type = {"box": mujoco.mjtGeom.mjGEOM_BOX, "cylinder": mujoco.mjtGeom.mjGEOM_CYLINDER,
                  "capsule": mujoco.mjtGeom.mjGEOM_CAPSULE, "ellipsoid": mujoco.mjtGeom.mjGEOM_ELLIPSOID,
                  "mesh": mujoco.mjtGeom.mjGEOM_MESH, "sphere": mujoco.mjtGeom.mjGEOM_SPHERE}[t]
        if t == "mesh":
            g.meshname = st["mesh"]
        if "fromto" in st:
            g.fromto = st["fromto"]
            g.size = st["size"]
        else:
            g.size = st["size"]
            g.pos = st["pos"]
            g.quat = st["quat"]
        if st.get("mat"):
            g.material = st["mat"]
        if st["collide"]:
            g.contype, g.conaffinity = CT_WORLD, CA_WORLD
            g.group = GROUP_WORLD
            g.priority = 1
            g.friction = [MAT[st["mat"]][4] if st.get("mat") in MAT else 0.8, 0.02, 0.001]
        else:
            g.contype, g.conaffinity = 0, 0
            g.group = GROUP_VISUAL
    # people (static: the survivor cannot move, the decoys are things)
    for who, parts in S.parts.items():
        for p in parts:
            g = spec.worldbody.add_geom()
            g.name = p["name"]
            g.type = mujoco.mjtGeom.mjGEOM_CAPSULE if p["type"] == "capsule" else mujoco.mjtGeom.mjGEOM_ELLIPSOID
            if p["type"] == "capsule":
                g.fromto = p["fromto"]
                g.size = p["size"]
            else:
                g.size = p["size"]
                g.pos = p["pos"]
                g.quat = p["quat"]
            g.rgba = p["rgba"]
            g.contype, g.conaffinity = CT_WORLD, CA_WORLD
            g.priority = 1
            g.friction = [0.7, 0.02, 0.001]
            g.group = GROUP_WORLD
    # movable debris
    for bd in S.bodies:
        b = spec.worldbody.add_body()
        b.name = bd["name"]
        b.pos = bd["pos"]
        b.quat = bd["quat"]
        fj = b.add_freejoint()
        fj.name = bd["name"] + "_free"
        for k, gd in enumerate(bd["geoms"]):
            g = b.add_geom()
            g.name = "%s_g%d" % (bd["name"], k)
            g.type = mujoco.mjtGeom.mjGEOM_BOX
            g.size = gd["size"]
            g.material = gd["mat"]
            g.mass = bd["mass"] / len(bd["geoms"])
            if bd.get("robot_only"):
                g.contype, g.conaffinity = 4, 2          # held by its weld: touches the robot only, until it falls
            else:
                g.contype, g.conaffinity = CT_WORLD, CA_WORLD
            g.priority = 1
            g.friction = [MAT.get(gd["mat"], (None, None, None, None, 0.7))[4], 0.02, 0.001]
            g.group = GROUP_WORLD
        if bd["kind"] == "propslab":
            # held in place by a weld to the world; the weld acts at the slab's own centre (data[0:3] is the anchor in
            # body2's frame, here the world's: the slab's position), relpose from qpos0 (zero quaternion)
            eq = spec.add_equality()
            eq.name = "hold_" + bd["name"]
            eq.type = mujoco.mjtEq.mjEQ_WELD
            eq.name1 = bd["name"]
            eq.objtype = mujoco.mjtObj.mjOBJ_BODY
            eq.data = [float(bd["pos"][0]), float(bd["pos"][1]), float(bd["pos"][2]), 0, 0, 0, 0, 0, 0, 0, 1.0]
            eq.active = True
    # the robot and its kit
    st = plan["start"]
    z0 = S.height_at(st["x"], st["y"]) + CRAWLER["track_wheel_r"] + 0.012 + 0.035 + 0.01
    build_crawler(spec, spec.worldbody, pos=(st["x"], st["y"], z0), yaw=st["yaw"])
    ch = spec.body("chassis")
    kx, ky, kz = CRAWLER["kit_cradle"]
    kb = spec.worldbody.add_body()
    kb.name = "kit"
    R = np.array([[math.cos(st["yaw"]), -math.sin(st["yaw"]), 0], [math.sin(st["yaw"]), math.cos(st["yaw"]), 0], [0, 0, 1]])
    kp = np.array([st["x"], st["y"], z0]) + R @ np.array([kx, ky, kz + 0.036])
    kb.pos = kp.tolist()
    kb.quat = _q_axis((0, 0, 1), st["yaw"])
    fj = kb.add_freejoint()
    fj.name = "kit_free"
    g = kb.add_geom()
    g.name = "kit_bag"
    g.type = mujoco.mjtGeom.mjGEOM_BOX
    g.size = [0.085, 0.045, 0.035]
    g.material = "kit"
    g.mass = 1.2
    g.contype, g.conaffinity = CT_WORLD, CA_WORLD
    g.priority = 1
    g.friction = [0.8, 0.02, 0.001]
    g2 = kb.add_geom()
    g2.name = "kit_strap"
    g2.type = mujoco.mjtGeom.mjGEOM_BOX
    g2.size = [0.087, 0.012, 0.037]
    g2.rgba = [0.95, 0.95, 0.95, 1]
    g2.contype, g2.conaffinity = 0, 0
    g2.group = GROUP_VISUAL
    g2.mass = 0.0
    latch = spec.add_equality()
    latch.name = "kit_latch"
    latch.type = mujoco.mjtEq.mjEQ_WELD
    latch.name1 = "chassis"
    latch.name2 = "kit"
    latch.objtype = mujoco.mjtObj.mjOBJ_BODY
    latch.data = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1.0]          # acts at the kit's centre; relpose from qpos0
    latch.active = True
    grip = spec.add_equality()
    grip.name = "grip"
    grip.type = mujoco.mjtEq.mjEQ_WELD
    grip.name1 = "gripper"
    grip.name2 = "kit"
    grip.objtype = mujoco.mjtObj.mjOBJ_BODY
    grip.active = False
    # lidar depth cameras on the mast top (4 x 90 deg)
    mt = spec.body("mast_top")
    q_cam = (0.5, 0.5, -0.5, -0.5)
    from .robot import _qmul
    for k in range(4):
        cam = mt.add_camera()
        cam.name = "lidar_%d" % k
        cam.pos = [0.0, 0.0, 0.105]
        cam.quat = _qmul(_q_axis((0, 0, 1), k * math.pi / 2), list(q_cam))
        cam.fovy = 44.0
    # the sun
    sun = spec.worldbody.add_light()
    sun.name = "sun"
    sun.type = mujoco.mjtLightType.mjLIGHT_DIRECTIONAL
    el = math.radians(env["sun_elev"])
    az = math.radians(env["sun_az"])
    sun.dir = [-math.sin(az) * math.cos(el), math.cos(az) * math.cos(el), -math.sin(el)]
    sun.pos = [S.W / 2, S.D / 2, 30.0]
    sun.castshadow = True
    k = env["sun"]
    sun.diffuse = [0.95 * k, 0.92 * k, 0.85 * k]
    sun.specular = [0.2, 0.2, 0.2]
    sun.ambient = [0.10, 0.10, 0.11]
    model = spec.compile()
    # fix the lidar cameras' fovy (degrees given above while compiler.degree is False only affects joint angles)
    data = mujoco.MjData(model)
    info = {"spec": spec, "chassis_body": mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "chassis")}
    return Scene(S, model, data, info)

"""Build the data-hall cell's MuJoCo model (cell internals): the hall shell, the pod's two rows of racks with their GPU
nodes and leaf switches, the cold-aisle containment, the overhead services, the background rows, the service area,
the lights, one G1 humanoid.

Geometry comes from hall.py. What the cameras see is drawn from the procedural meshes and textures of meshes.py and
textures.py (geom group 1, no contact); what the robot collides with are primitive colliders (group 3, hidden).
Collision bits (robot.py): 1 = the static hall and its fixtures, 2 = the robot, 4 = loose parts and tools, 8 = floor.
"""

from __future__ import annotations

import math

import mujoco
import numpy as np

from . import hall as H
from . import meshes as M
from . import parts as P
from . import textures as T
from .look import add_view_options
from .robot import FLOOR, OBJ, ROBOT, WORLD, g1_spec

G = mujoco.mjtGeom
VIS = dict(contype=0, conaffinity=0, group=1, mass=0.0)            # drawn only: no contact, no mass
EQUIP = 32                                                          # equipment: the robot collides with it, parts do not
COL = dict(contype=WORLD, conaffinity=ROBOT | OBJ, group=3, condim=3, friction=[0.6, 0.005, 0.0001])
COL_EQUIP = dict(contype=EQUIP, conaffinity=ROBOT, group=3, condim=3, friction=[0.6, 0.005, 0.0001])
ROLE = mujoco.mjtTextureRole

LED_RGB = {"green": (0.0, 1.0, 0.12), "amber": (1.0, 0.32, 0.0), "blue": (0.0, 0.22, 1.0), "white": (1.0, 1.0, 1.0),
           "cyan": (0.0, 0.85, 1.0), "red": (1.0, 0.02, 0.0)}
LED_EMISSION = 0.45


def quat_z(yaw):
    return [math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)]


def quat_axis(axis, ang):
    a = np.asarray(axis, float)
    a = a / np.linalg.norm(a)
    s = math.sin(ang / 2)
    return [math.cos(ang / 2), a[0] * s, a[1] * s, a[2] * s]


def rot_z(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


# ------------------------------------------------------------------------------------------------ materials
class Bank:
    """Textures and materials of the model, made on demand (deduplicated by key)."""

    def __init__(self, spec):
        self.s = spec
        self.tex = {}
        self.mats = {}
        self.uv_scale = {}      # translucent textured materials: the repeat lives in the mesh's coordinates

    def _texture(self, key, img, srgb):
        if key not in self.tex:
            t = self.s.add_texture()
            t.name = "tx_%d" % len(self.tex)
            t.type = mujoco.mjtTexture.mjTEXTURE_2D
            img = np.ascontiguousarray(img[..., :3])
            t.width, t.height = int(img.shape[1]), int(img.shape[0])
            t.nchannel = 3
            t.colorspace = mujoco.mjtColorSpace.mjCOLORSPACE_SRGB if srgb else mujoco.mjtColorSpace.mjCOLORSPACE_LINEAR
            t.data = np.ascontiguousarray(img[::-1]).tobytes()       # MuJoCo's row 0 is the image's bottom row
            self.tex[key] = t.name
        return self.tex[key]

    def _orm(self, rough, metal):
        key = ("orm", round(rough, 3), round(metal, 3))
        return self._texture(key, T.pack(1.0, rough, metal), False)

    def _new(self, key):
        mt = self.s.add_material()
        mt.name = "mt_%d" % len(self.mats)
        for role in range(int(ROLE.mjNTEXROLE)):
            mt.textures[role] = ""
        self.mats[key] = mt.name
        return mt

    def textured(self, setname, tint=(1.0, 1.0, 1.0), tile=None, alpha=1.0, emission=0.0):
        """A tileable set (textures.SETS) at `tile` metres per repeat (the mesh's texture coordinates are metres)."""
        tile = float(tile or T.SETS[setname][1])
        key = ("t", setname, tuple(round(c, 3) for c in tint), round(tile, 4), round(alpha, 3), round(emission, 3))
        if key in self.mats:
            return self.mats[key]
        st = T.texture_set(setname)
        mt = self._new(key)
        mt.textures[ROLE.mjTEXROLE_RGB] = self._texture(("set", setname, "albedo"), st["albedo"], True)
        mt.textures[ROLE.mjTEXROLE_NORMAL] = self._texture(("set", setname, "normal"), st["normal"], False)
        mt.textures[ROLE.mjTEXROLE_ORM] = self._texture(("set", setname, "orm"), st["orm"], False)
        mt.texuniform = False
        if alpha < 1.0:         # MuJoCo 3.16's Filament ignores texrepeat on a translucent material
            mt.texrepeat = [1.0, 1.0]
            self.uv_scale[mt.name] = 1.0 / tile
        else:
            mt.texrepeat = [1.0 / tile, 1.0 / tile]
        mt.rgba = [float(tint[0]), float(tint[1]), float(tint[2]), float(alpha)]
        mt.roughness, mt.metallic, mt.specular, mt.reflectance = 1.0, 1.0, 0.5, 0.0
        mt.emission = float(emission)
        return mt.name

    def plain(self, rgb, rough=0.6, metal=0.0, alpha=1.0, emission=0.0):
        key = ("p", tuple(round(c, 4) for c in rgb), round(rough, 3), round(metal, 3), round(alpha, 3), round(emission, 3))
        if key in self.mats:
            return self.mats[key]
        mt = self._new(key)
        mt.textures[ROLE.mjTEXROLE_ORM] = self._orm(rough, metal)
        mt.rgba = [float(rgb[0]), float(rgb[1]), float(rgb[2]), float(alpha)]
        mt.roughness, mt.metallic, mt.specular, mt.reflectance = 1.0, 1.0, 0.5, 0.0
        mt.emission = float(emission)
        return mt.name

    def atlas(self, key, albedo, orm):
        """A part's atlas material: one albedo and one ORM image whose regions the part's mesh samples."""
        mkey = ("a", key)
        if mkey in self.mats:
            return self.mats[mkey]
        mt = self._new(mkey)
        mt.textures[ROLE.mjTEXROLE_RGB] = self._texture(("atlas", key, "albedo"), albedo, True)
        mt.textures[ROLE.mjTEXROLE_ORM] = self._texture(("atlas", key, "orm"), orm, False)
        mt.texuniform = False
        mt.texrepeat = [1.0, 1.0]
        mt.rgba = [1.0, 1.0, 1.0, 1.0]
        mt.roughness, mt.metallic, mt.specular, mt.reflectance = 1.0, 1.0, 0.5, 0.0
        return mt.name

    def image(self, key, img, rough=0.5, metal=0.0, emission=0.0, repeat=(1.0, 1.0)):
        """A single-use panel texture (a label, a bezel, a port face) mapped by the mesh's own 0..1 coordinates."""
        mkey = ("i", key, round(rough, 3), round(metal, 3), round(emission, 3), tuple(repeat))
        if mkey in self.mats:
            return self.mats[mkey]
        mt = self._new(mkey)
        mt.textures[ROLE.mjTEXROLE_RGB] = self._texture(("img", key), img, True)
        mt.textures[ROLE.mjTEXROLE_ORM] = self._orm(rough, metal)
        mt.texuniform = False
        mt.texrepeat = [float(repeat[0]), float(repeat[1])]
        mt.rgba = [1.0, 1.0, 1.0, 1.0]
        mt.roughness, mt.metallic, mt.specular, mt.reflectance = 1.0, 1.0, 0.5, 0.0
        mt.emission = float(emission)
        return mt.name


# ------------------------------------------------------------------------------------------------ geometry helper
class Builder:
    def __init__(self, spec, bank):
        self.s, self.bank = spec, bank
        self.meshes = set()
        self.spec_meshes = set()
        self.mesh_data = {}
        self.n = 0
        self.leds = {}          # name -> (geom name, initial state): the cell switches their materials
        self.static_frames = {}  # body name -> (R, t) world pose of a static body whose drawn geoms are merged
        self.static_parts = {}   # material -> [mesh data in the world frame]
        self.merge_bodies = {}   # body name -> body: drawn geoms merged per material within the body (moving bodies)
        self.local_parts = {}    # (body name, material) -> [mesh data in the body frame]

    def static_frame(self, body, pos, quat):
        """Register a body that never moves: its drawn geoms are merged, per material, into world meshes."""
        R = np.zeros(9)
        mujoco.mju_quat2Mat(R, np.asarray(quat, float))
        self.static_frames[body.name] = (R.reshape(3, 3), np.asarray(pos, float))

    def flush_static(self):
        wb = self.s.worldbody
        for k, (mat, parts) in enumerate(sorted(self.static_parts.items())):
            for j in range(0, len(parts), 400):          # at most 400 pieces per mesh (keeps each mesh modest)
                data = M.merge(*parts[j:j + 400])
                name = "static_%d_%d" % (k, j // 400)
                self.mesh(name, data)
                self.spec_mesh(name)
                wb.add_geom(name=name, type=G.mjGEOM_MESH, meshname=name, material=mat, **VIS)
        self.static_parts.clear()
        for k, ((bname, mat), parts) in enumerate(sorted(self.local_parts.items())):
            name = "local_%d" % k
            self.mesh(name, M.merge(*parts))
            self.spec_mesh(name)
            self.merge_bodies[bname].add_geom(name=name, type=G.mjGEOM_MESH, meshname=name, material=mat, **VIS)
        self.local_parts.clear()

    def mesh(self, name, data):
        """Remember a mesh; it becomes a model asset only when a geom draws it (merged pieces never do: every asset
        costs the renderer GPU handles)."""
        if name not in self.meshes:
            self.mesh_data[name] = data
            self.meshes.add(name)
        return name

    def spec_mesh(self, name):
        if name not in self.spec_meshes:
            V, F, N, UV = self.mesh_data[name]
            m = self.s.add_mesh(name=name, inertia=mujoco.mjtMeshInertia.mjMESH_INERTIA_SHELL)
            m.uservert = np.asarray(V, np.float64).ravel()
            m.usernormal = np.asarray(N, np.float64).ravel()
            m.usertexcoord = np.asarray(UV, np.float64).ravel()
            m.userface = np.asarray(F, np.int32).ravel()
            self.spec_meshes.add(name)
        return name

    def vis(self, body, mesh_name, data, material, pos=(0, 0, 0), quat=(1, 0, 0, 0), name=None, rgba=None):
        """A drawn-only mesh geom (group 1). `data` may be None when the mesh is already registered."""
        if data is not None:
            sc = self.bank.uv_scale.get(material)
            if sc is not None:
                V, F, N, UV = data
                data = (V, F, N, UV * sc)
            self.mesh(mesh_name, data)
        if body.name in self.merge_bodies and rgba is None and name is None:
            Rg = np.zeros(9)
            mujoco.mju_quat2Mat(Rg, np.asarray(quat, float))
            V, F, N, UV = self.mesh_data[mesh_name]
            self.local_parts.setdefault((body.name, material), []).append(M.transform((V, F, N, UV), R=Rg.reshape(3, 3), t=pos))
            return None
        if body.name in self.static_frames and rgba is None and name is None:
            Rb, tb = self.static_frames[body.name]
            Rg = np.zeros(9)
            mujoco.mju_quat2Mat(Rg, np.asarray(quat, float))
            Rg = Rg.reshape(3, 3)
            V, F, N, UV = self.mesh_data[mesh_name]
            self.static_parts.setdefault(material, []).append(M.transform((V, F, N, UV), R=Rb @ Rg, t=Rb @ np.asarray(pos, float) + tb))
            return None
        self.n += 1
        kw = dict(VIS)
        if rgba is not None:
            kw["rgba"] = list(rgba)
        self.spec_mesh(mesh_name)
        return body.add_geom(name=name or "v%d" % self.n, type=G.mjGEOM_MESH, meshname=mesh_name, material=material,
                             pos=[float(v) for v in pos], quat=[float(v) for v in quat], **kw)

    def prim(self, body, gtype, size, pos, material, quat=(1, 0, 0, 0), name=None, rgba=None):
        self.n += 1
        kw = dict(VIS)
        if rgba is not None:
            kw["rgba"] = list(rgba)
        return body.add_geom(name=name or "p%d" % self.n, type=gtype, size=[float(v) for v in size],
                             pos=[float(v) for v in pos], quat=[float(v) for v in quat], material=material, **kw)

    def col(self, body, half, pos, quat=(1, 0, 0, 0), name=None, kind=None, mass=None):
        """A box collider (group 3, hidden). kind: None = the hall (robot and parts collide), "equip" = equipment
        (only the robot collides), or a dict of contact attributes."""
        self.n += 1
        kw = dict(COL if kind is None else COL_EQUIP if kind == "equip" else kind)
        if mass is not None:
            kw["mass"] = float(mass)
        else:
            kw.setdefault("mass", 0.0)
        return body.add_geom(name=name or "c%d" % self.n, type=G.mjGEOM_BOX, size=[float(v) for v in half],
                             pos=[float(v) for v in pos], quat=[float(v) for v in quat], **kw)

    def led_material(self, state):
        if state == "off":
            return self.bank.plain((0.03, 0.03, 0.03), 0.3)
        return self.bank.plain(LED_RGB[state], 0.3, emission=LED_EMISSION)

    def led_static(self, body, pos, size, state="green"):
        """A decorative LED that never changes (merged into the static meshes of its body)."""
        if body.name not in self.static_frames:
            return self.led(body, "deco%d" % self.n, pos, size, state)
        mname = "led_box_%g_%g_%g" % tuple(size)
        if mname not in self.meshes:
            self.mesh(mname, M.box(size))
        return self.vis(body, mname, None, self.led_material(state), pos=pos)

    def led(self, body, name, pos, size, state="green", quat=(1, 0, 0, 0), shape="box"):
        """A status LED: one geom whose material the cell switches (off / green / amber / blue ...): materials switch
        at run time in the renderer (geom_matid), rgba does not reach a geom with a material."""
        gt = G.mjGEOM_BOX if shape == "box" else G.mjGEOM_CYLINDER
        g = self.prim(body, gt, size, pos, self.led_material(state), quat, name="led_" + name)
        self.leds[name] = (g.name, state)
        return g


# ------------------------------------------------------------------------------------------------ racks
def _rack_shell(B):
    """The rack's frame, side panels, top and plinth (one shared mesh per material), in the rack frame."""
    if "rack_frame" in B.meshes:
        return
    W, D, Hh = H.RACK_W, H.RACK_D, H.RACK_H
    parts = []
    for sx in (-1, 1):
        for y in (0.02, D - 0.02):
            parts.append(M.transform(M.box((0.02, 0.02, Hh / 2)), t=(sx * (W / 2 - 0.02), y, Hh / 2)))
        for z in (0.07, Hh - 0.03):
            parts.append(M.transform(M.box((0.02, D / 2, 0.02)), t=(sx * (W / 2 - 0.02), D / 2, z)))
    for y in (0.02, D - 0.02):
        for z in (0.07, Hh - 0.03):
            parts.append(M.transform(M.box((W / 2, 0.02, 0.02)), t=(0.0, y, z)))
    B.mesh("rack_frame", M.merge(*parts))
    sides = [M.transform(M.box((0.003, D / 2 - 0.04, Hh / 2 - 0.08)), t=(sx * (W / 2 - 0.004), D / 2, Hh / 2))
             for sx in (-1, 1)]
    B.mesh("rack_sides", M.merge(*sides))
    B.mesh("rack_top", M.transform(M.box((W / 2, D / 2, 0.012)), t=(0.0, D / 2, Hh - 0.012)))
    B.mesh("rack_plinth", M.transform(M.box((W / 2 - 0.01, D / 2 - 0.02, 0.04)), t=(0.0, D / 2, 0.04)))
    # mounting rails: front pair carries the U numbers, rear pair plain
    rail = M.panel(0.016, H.RACK_UNITS * H.U, 0.004, "-y")
    B.mesh("rail_front", rail)
    B.mesh("rail_rear", M.box((0.008, 0.012, H.RACK_UNITS * H.U / 2)))


def blank_mesh(B, n_units):
    name = "blank_%d" % n_units
    if name not in B.meshes:
        V, F, N, UV = M.panel(H.EIA_W - 0.002, n_units * H.U - 0.001, 0.004, "-y")
        UV = UV.copy()
        UV[-4:, 1] *= n_units                 # the 1U texture repeats once per unit on the front face
        B.mesh(name, (V, F, N, UV))
    return name


def pod_offsets(seed):
    """This pod's installation offsets (the drawing is the same for every pod; each pod is built within tolerance):
    {rack name: (dx, dy, dyaw)} in the hall frame. A row's racks are bolted together: one offset per row (x, y, yaw
    about the row's middle) plus a little per rack. Deterministic from the seed."""
    rng = np.random.default_rng([int(seed) & 0xFFFFFFFF, (int(seed) >> 32) & 0xFFFFFFFF, 0xD0D])
    out = {}
    for row in H.ROWS:
        rx, ry = float(np.clip(rng.normal(0, 0.007), -0.015, 0.015)), float(np.clip(rng.normal(0, 0.005), -0.010, 0.010))
        ryaw = float(np.clip(rng.normal(0, math.radians(0.12)), -math.radians(0.3), math.radians(0.3)))
        mid = H.X0 + H.N_RACKS * H.RACK_W / 2.0
        for idx in range(1, H.N_RACKS + 1):
            (x, y, z), yaw = H.rack_frame(row, idx)
            dx = rx + float(np.clip(rng.normal(0, 0.001), -0.002, 0.002))
            dy = ry + (x - mid) * math.sin(ryaw) + float(np.clip(rng.normal(0, 0.0015), -0.003, 0.003))
            dyaw = ryaw + float(np.clip(rng.normal(0, math.radians(0.04)), -math.radians(0.1), math.radians(0.1)))
            out[H.rack_name(row, idx)] = (dx, dy, dyaw)
    return out


RDOOR_MAX = 1.95     # a rear door swings to 112 deg (its stop)


def add_rack(B, row, idx, rear_open=0.0, offset=(0.0, 0.0, 0.0)):
    """One rack of the pod (frame, rails, blanking, plates, PDUs, the rear door); returns its body. `offset` (dx, dy,
    dyaw): this pod's installation offset of the rack."""
    s, bank = B.s, B.bank
    (x, y, z), yaw = H.rack_frame(row, idx)
    x, y, yaw = x + offset[0], y + offset[1], yaw + offset[2]
    name = H.rack_name(row, idx)
    body = s.worldbody.add_body(name="rack_" + name, pos=[x, y, z], quat=quat_z(yaw))
    B.static_frame(body, (x, y, z), quat_z(yaw))
    _rack_shell(B)
    black = bank.textured("powder_black")
    B.vis(body, "rack_frame", None, black)
    B.vis(body, "rack_sides", None, black)
    B.vis(body, "rack_top", None, black)
    B.vis(body, "rack_plinth", None, bank.plain((0.01, 0.01, 0.012), 0.7))
    rail_img = T.rail_u(H.RACK_UNITS)
    rail_mat = bank.image("rail_u", rail_img, 0.55)
    for sx in (-1, 1):
        B.vis(body, "rail_front", None, rail_mat, pos=(sx * (H.EIA_W / 2 - 0.010), H.FRONT_RAIL_Y + 0.002,
                                                       H.U1_Z + H.RACK_UNITS * H.U / 2))
        B.vis(body, "rail_rear", None, black, pos=(sx * (H.EIA_W / 2 - 0.010), H.REAR_RAIL_Y,
                                                  H.U1_Z + H.RACK_UNITS * H.U / 2))
    # the front's blanking panels (what is not equipment)
    blank_mat = bank.image("blank_1u", T.blank_1u(), 0.55, repeat=(1.0, 1.0))
    used = set()
    if H.is_network(idx):
        for u in H.LEAF_U:
            used.add(u)
        used.update(H.PATCH_U)
    else:
        used.update(range(H.NODE_U[0], H.NODE_U[1] + 1))
        for u in H.AUX_2U:
            used.update((u, u + 1))
        used.add(H.MGMT_U)
    runs, cur = [], []
    for u in range(1, H.RACK_UNITS + 1):
        if u in used:
            if cur:
                runs.append(cur)
            cur = []
        else:
            cur.append(u)
    if cur:
        runs.append(cur)
    for run in runs:
        n = len(run)
        zc = H.u_z(run[0]) + n * H.U / 2
        B.vis(body, blank_mesh(B, n), None, blank_mat, pos=(0.0, H.FRONT_RAIL_Y - 0.003, zc))
    # equipment nobody tickets: 2U storage / CPU nodes and the management switch (GPU racks), fibre patch panels
    if "aux2u" not in B.meshes:
        B.mesh("aux2u", M.panel(H.EIA_W - 0.002, 2 * H.U - 0.002, 0.010, "-y"))
        B.mesh("aux1u", M.panel(H.EIA_W - 0.002, H.U - 0.002, 0.010, "-y"))
        B.mesh("aux_body", M.box((0.218, 0.35, H.U - 0.002)))
    if H.is_network(idx):
        for k, u in enumerate(H.PATCH_U):
            B.vis(body, "aux1u", None, bank.image("patch", T.patch_panel(), 0.5), pos=(0.0, H.FRONT_RAIL_Y - 0.006, H.u_z(u) + H.U / 2))
    else:
        for k, u in enumerate(H.AUX_2U):
            nm = "stor-%s%02d-%d" % (row.lower(), idx, k + 1)
            B.vis(body, "aux2u", None, bank.image("aux_" + nm, T.aux_front(name=nm), 0.5), pos=(0.0, H.FRONT_RAIL_Y - 0.006, H.u_z(u) + H.U))
            B.vis(body, "aux_body", None, bank.textured("powder_grey"), pos=(0.0, H.FRONT_RAIL_Y + 0.35, H.u_z(u) + H.U))
            for j in range(6):
                B.led_static(body, (-0.178 + j * 0.0672, H.FRONT_RAIL_Y - 0.0125, H.u_z(u) + 1.55 * H.U),
                             (0.0012, 0.0008, 0.0012), "green" if (j + k + idx) % 5 else "blue")
        B.vis(body, "aux1u", None, bank.image("mgmt", T.mgmt_switch(), 0.5), pos=(0.0, H.FRONT_RAIL_Y - 0.006, H.u_z(H.MGMT_U) + H.U / 2))
        blue = bank.plain((0.12, 0.35, 0.80), 0.55)
        zc = H.u_z(H.MGMT_U) + H.U / 2
        for j in range(6):
            x0 = -0.17 + j * 0.0128 * 2
            pts = [(x0, H.FRONT_RAIL_Y - 0.02, zc), (x0, H.FRONT_RAIL_Y - 0.06, zc - 0.03), (0.21, H.FRONT_RAIL_Y - 0.05, zc - 0.10 - 0.01 * j),
                   (0.225, H.FRONT_RAIL_Y - 0.02, zc - 0.5)]
            B.vis(body, "rj45_%s_%d" % (name, j), M.cable(pts, 0.0028, samples=5, segments=6), blue)
    # the rack's name plates (front top, rear top)
    plate = bank.image("plate_" + name, T.rack_plate(name), 0.4)
    B.vis(body, "plate_" + name + "_f", M.panel(0.12, 0.045, 0.002, "-y"), plate, pos=(0.0, -0.002, H.RACK_H - 0.075))
    B.vis(body, "plate_" + name + "_r", M.panel(0.12, 0.045, 0.002, "+y"), plate,
          pos=(0.0, H.RACK_D + 0.002, H.RACK_H - 0.075))
    # vertical PDUs at the rear corners: feed A on the rack's +x side, feed B on its -x side
    for feed, sx in (("A", 1), ("B", -1)):
        pdu = bank.image("pdu_" + feed, T.pdu_face(feed=feed, amps=9.6 if feed == "A" else 10.1), 0.5)
        V, F, N, UV = M.panel(0.052, 1.70, 0.045, "+y")
        B.vis(body, "pdu_" + feed, (V, F, N, UV) if ("pdu_" + feed) not in B.meshes else None, pdu,
              pos=(sx * 0.262, H.RACK_D - 0.045, 1.15))
    # the rear door: a framed perforated panel hinged on the rack's -x rear post (seen from the hot aisle: the right)
    door = s.worldbody.add_body(name="rdoor_" + name, pos=[x, y, z], quat=quat_z(yaw))
    hinge_x, hinge_y = -(H.RACK_W / 2 - 0.01), H.RACK_D + 0.012
    leaf = door.add_body(name="rdoor_leaf_" + name, pos=[hinge_x, hinge_y, 0.0])      # opens outwards, into the hot aisle
    leaf.add_joint(name="rdoor_" + name, type=mujoco.mjtJoint.mjJNT_HINGE, axis=[0, 0, 1], range=[0.0, RDOOR_MAX],
                   limited=mujoco.mjtLimited.mjLIMITED_TRUE, damping=2.0, frictionloss=0.3, armature=0.05)
    META_REF["init_q"]["rdoor_" + name] = rear_open
    META_REF["features"][leaf.name] = [("cyl", (H.RACK_W - 0.045, 0.024, 1.03), (H.RACK_W - 0.045, 0.024, 1.17), 0.009, 0.012)]
    META_REF["graspable"].append(leaf.name)
    META_REF["fixtures"][leaf.name] = {"type": "rdoor", "rack": name, "joint": "rdoor_" + name, "latch_pull_N": 10.0}
    B.col(leaf, (H.RACK_W / 2 - 0.01, 0.008, H.RACK_H / 2 - 0.06), (H.RACK_W / 2 - 0.005, 0.0, H.RACK_H / 2), kind="equip", mass=9.0)
    perf_data = None
    if "door_perf" not in B.meshes:
        perf_data = M.transform(M.box((H.RACK_W / 2 - 0.03, 0.002, H.RACK_H / 2 - 0.09)),
                                t=(H.RACK_W / 2 - 0.005, 0.0, H.RACK_H / 2))
        fr = []
        for xx in (0.006, H.RACK_W - 0.02):
            fr.append(M.transform(M.box((0.012, 0.008, H.RACK_H / 2 - 0.06)), t=(xx, 0.0, H.RACK_H / 2)))
        for zz in (0.08, H.RACK_H - 0.07):
            fr.append(M.transform(M.box((H.RACK_W / 2 - 0.01, 0.008, 0.015)), t=(H.RACK_W / 2 - 0.006, 0.0, zz)))
        B.mesh("door_frame", M.merge(*fr))
        B.mesh("door_handle", M.transform(M.box((0.008, 0.012, 0.09)), t=(H.RACK_W - 0.045, 0.012, 1.10)))
    B.vis(leaf, "door_perf", perf_data, bank.textured("hex_perf", alpha=0.62))
    B.vis(leaf, "door_frame", None, black)
    B.vis(leaf, "door_handle", None, bank.textured("brushed_alu"))
    # colliders (equipment: the robot collides, parts do not): side panels, top, plinth, the block the mounted
    # equipment fills between the front rails and the nodes' rears, the PDUs; the cable space behind stays open
    for sx in (-1, 1):
        B.col(body, (0.006, H.RACK_D / 2, H.RACK_H / 2), (sx * (H.RACK_W / 2 - 0.006), H.RACK_D / 2, H.RACK_H / 2), kind="equip")
        B.col(body, (0.028, 0.025, 0.86), (sx * 0.262, H.RACK_D - 0.045, 1.15), kind="equip")
    B.col(body, (H.RACK_W / 2, H.RACK_D / 2, 0.02), (0.0, H.RACK_D / 2, H.RACK_H - 0.02), kind="equip")
    B.col(body, (H.RACK_W / 2, H.RACK_D / 2, H.U1_Z / 2), (0.0, H.RACK_D / 2, H.U1_Z / 2), kind="equip")
    ye = H.NODE_Y1 if not H.is_network(idx) else 1.00
    B.col(body, (0.240, (ye - 0.092) / 2, (H.u_z(49) - H.U1_Z) / 2), (0.0, (ye + 0.092) / 2, (H.u_z(49) + H.U1_Z) / 2), kind="equip")
    return body


# ------------------------------------------------------------------------------------------------ containment
def add_containment(B, west_open=0.0, east_open=0.0):
    """The cold aisle's roof (translucent panels on aluminium beams) and its two sliding double doors."""
    s, bank = B.s, B.bank
    wb = s.worldbody
    alu = bank.textured("brushed_alu", tint=(0.85, 0.86, 0.88))
    roof = bank.plain((0.82, 0.86, 0.88), 0.25, alpha=0.30)
    L = H.N_RACKS * H.RACK_W
    zt = H.CONTAIN_TOP
    B.vis(wb, "roof_panels", M.box((L / 2, H.COLD_AISLE_HALF, 0.003)), roof, pos=(H.X0 + L / 2, 0.0, zt + 0.003))
    beams = [M.transform(M.box((0.025, H.COLD_AISLE_HALF + 0.03, 0.02)), t=(H.X0 + i * H.RACK_W, 0.0, zt + 0.02))
             for i in range(H.N_RACKS + 1)]
    beams += [M.transform(M.box((L / 2 + 0.03, 0.025, 0.02)), t=(H.X0 + L / 2, sy * (H.COLD_AISLE_HALF - 0.01), zt + 0.02))
              for sy in (-1, 1)]
    B.vis(wb, "roof_beams", M.merge(*beams), alu)
    housing = bank.plain((0.86, 0.87, 0.88), 0.4, 0.2)
    diff = bank.plain((1.0, 0.985, 0.96), 0.3, emission=1.6)
    for i in range(H.N_RACKS):                              # the contained aisle's own LED fixtures
        xl = H.X0 + (i + 0.5) * H.RACK_W
        B.vis(wb, "cfix_%d" % i, M.box((0.27, 0.035, 0.018)), housing, pos=(xl, 0.0, zt - 0.03))
        B.vis(wb, "cdif_%d" % i, M.box((0.26, 0.028, 0.003)), diff, pos=(xl, 0.0, zt - 0.049))
        lt = wb.add_light(name="clt_%d" % i, type=mujoco.mjtLightType.mjLIGHT_SPOT, pos=[xl, 0.0, zt - 0.06],
                          dir=[0.0, 0.0, -1.0])
        lt.cutoff, lt.intensity, lt.range, lt.castshadow, lt.bulbradius = 80.0, CONTAIN_CD, 8.0, SHADOWS, 0.25
    glass = bank.plain((0.70, 0.78, 0.80), 0.06, alpha=0.22)
    sign = bank.image("sign_cold", T.sign("COLD AISLE  POD 3", 768, 96), 0.5, emission=0.15)
    for end, xd, opening in (("w", H.DOOR_X[0], west_open), ("e", H.DOOR_X[1], east_open)):
        fr = [M.transform(M.box((0.03, 0.03, H.RACK_H / 2)), t=(xd, sy * (H.COLD_AISLE_HALF - 0.03), H.RACK_H / 2))
              for sy in (-1, 1)]
        fr.append(M.transform(M.box((0.03, H.COLD_AISLE_HALF, 0.09)), t=(xd, 0.0, H.RACK_H - 0.09)))
        B.vis(wb, "door_frame_" + end, M.merge(*fr), alu)
        B.vis(wb, "door_sign_" + end, M.panel(0.92, 0.115, 0.004, "-x" if end == "w" else "+x"), sign,
              pos=(xd + (-0.032 if end == "w" else 0.032), 0.0, H.RACK_H - 0.09))
        for sy in (-1, 1):                     # two leaves slide apart, behind the rows' end panels
            leaf = wb.add_body(name="cdoor_%s%d" % (end, sy), pos=[xd, sy * 0.29, 0.0])
            leaf.add_joint(name="cdoor_%s%d" % (end, sy), type=mujoco.mjtJoint.mjJNT_SLIDE, axis=[0, sy, 0], range=[0.0, 0.56],
                           limited=mujoco.mjtLimited.mjLIMITED_TRUE, damping=8.0, frictionloss=2.0, armature=0.5)
            META_REF["init_q"]["cdoor_%s%d" % (end, sy)] = opening      # automatic doors (mech.AisleDoors)
            META_REF["fixtures"][leaf.name] = {"type": "cdoor", "joint": leaf.name}
            lh = H.RACK_H - 0.19
            B.vis(leaf, "cdoor_glass", M.box((0.006, 0.28, lh / 2 - 0.04)) if "cdoor_glass" not in B.meshes else None,
                  glass, pos=(0.0, 0.0, lh / 2 + 0.01))
            if "cdoor_frame" not in B.meshes:
                f2 = [M.transform(M.box((0.012, 0.018, lh / 2)), t=(0, yy, lh / 2 + 0.01)) for yy in (-0.28, 0.28)]
                f2 += [M.transform(M.box((0.012, 0.29, 0.02)), t=(0, 0, zz)) for zz in (0.03, lh - 0.01)]
                B.mesh("cdoor_frame", M.merge(*f2))
                B.mesh("cdoor_handle", M.box((0.02, 0.012, 0.08)))
            B.vis(leaf, "cdoor_frame", None, alu)
            B.vis(leaf, "cdoor_handle", None, alu, pos=(0.0, -sy * 0.24, 1.05))
            B.col(leaf, (0.012, 0.29, lh / 2), (0.0, 0.0, lh / 2 + 0.01), kind="equip", mass=12.0)


# ------------------------------------------------------------------------------------------------ overhead
def add_overhead(B):
    """Fibre raceways (yellow), two busways per row with their tap-off boxes, wire-basket trays over the hot aisles,
    the linear LED fixtures, sprinkler pipes, the ceiling."""
    s, bank = B.s, B.bank
    wb = s.worldbody
    x0, x1 = H.X0 - 1.6, H.X0 + H.N_RACKS * H.RACK_W + 1.2
    L, xc = x1 - x0, (x0 + x1) / 2
    yel = bank.textured("yellow_duct")
    ft = H.FIBER_TRAY
    for sy in (-1, 1):
        yc = sy * (H.COLD_AISLE_HALF + 0.82)
        parts = [M.transform(M.box((L / 2, ft["w"] / 2, 0.003)), t=(xc, yc, ft["z"]))]
        for dy in (-1, 1):
            parts.append(M.transform(M.box((L / 2, 0.003, ft["h"] / 2)), t=(xc, yc + dy * ft["w"] / 2, ft["z"] + ft["h"] / 2)))
        B.vis(wb, "raceway_%d" % sy, M.merge(*parts), yel)
        for i in range(H.N_RACKS):                         # drop-outs into each rack's top
            xr = H.X0 + (i + 0.5) * H.RACK_W
            B.vis(wb, "spout_%d_%d" % (sy, i), M.box((0.07, 0.05, 0.11)), yel, pos=(xr, yc, ft["z"] - 0.11))
        aqua = bank.plain((0.10, 0.62, 0.70), 0.45)
        for k in range(6):                                  # the fibre bundle in the raceway
            pts = [(x0 + 0.05, yc - 0.10 + 0.04 * k, ft["z"] + 0.01 + 0.006 * (k % 2)), (xc, yc - 0.09 + 0.04 * k, ft["z"] + 0.012),
                   (x1 - 0.05, yc - 0.10 + 0.04 * k, ft["z"] + 0.01)]
            B.vis(wb, "bundle_%d_%d" % (sy, k), M.cable(pts, 0.012, samples=4, segments=8), aqua)
        # busways A and B over the row's front half, tap-off boxes over every rack
        bw = H.BUSWAY
        for feed, dy, col in (("A", 0.20, (0.70, 0.10, 0.08)), ("B", 0.42, (0.10, 0.25, 0.62))):
            yb = sy * (H.COLD_AISLE_HALF + dy)
            B.vis(wb, "busway_%s%d" % (feed, sy), M.box((L / 2, bw["w"] / 2, bw["h"] / 2)),
                  bank.textured("powder_grey", tint=(3.0, 3.05, 3.1)), pos=(xc, yb, bw["z"]))
            for i in range(H.N_RACKS):
                xr = H.X0 + (i + 0.5) * H.RACK_W + (-0.12 if feed == "A" else 0.12)
                B.vis(wb, "tapoff_%s%d_%d" % (feed, sy, i), M.box((0.09, 0.07, 0.11)), bank.plain(col, 0.5),
                      pos=(xr, yb, bw["z"] - bw["h"] / 2 - 0.11))
                pts = [(xr, yb, bw["z"] - 0.30), (xr, yb + sy * 0.25, 2.45), (xr, sy * (H.COLD_AISLE_HALF + 1.0), H.RACK_H + 0.01)]
                B.vis(wb, "feed_%s%d_%d" % (feed, sy, i), M.cable(pts, 0.011, samples=5, segments=8),
                      bank.plain((0.02, 0.02, 0.022), 0.6))
    for yb in (2.4, -2.4):                                  # wire baskets over the hot aisles
        B.vis(wb, "basket_%d" % int(yb), M.box((L / 2, H.BASKET["w"] / 2, H.BASKET["h"] / 2)),
              bank.textured("wire_mesh", alpha=0.85), pos=(xc, yb, H.BASKET["z"]))
    # linear LED fixtures over the aisles (and the hall's other aisles) with their spot lights
    housing = bank.plain((0.86, 0.87, 0.88), 0.4, 0.2)
    diff = bank.plain((1.0, 0.985, 0.96), 0.3, emission=1.6)
    for yl in (0.0, 2.4, -2.4, 4.8, -4.8):
        for xl in (-2.4, -0.6, 1.2, 3.0, 4.8):
            B.vis(wb, "fix_%g_%g" % (xl, yl), M.box((0.60, 0.045, 0.025)), housing, pos=(xl, yl, H.LIGHT_Z + 0.03))
            B.vis(wb, "dif_%g_%g" % (xl, yl), M.box((0.58, 0.035, 0.004)), diff, pos=(xl, yl, H.LIGHT_Z))
            lt = wb.add_light(name="lt_%g_%g" % (xl, yl), type=mujoco.mjtLightType.mjLIGHT_SPOT,
                              pos=[xl, yl, H.LIGHT_Z - 0.02], dir=[0.0, 0.0, -1.0])
            lt.cutoff, lt.intensity, lt.range = 75.0, LIGHT_CD, 14.0
            lt.castshadow = SHADOWS
            lt.bulbradius = 0.35
    for yl in (0.0, 2.4, -2.4):                             # the light the pale floor throws back up onto the racks
        for xl in (-0.6, 0.9, 2.4, 3.9):
            lt = wb.add_light(name="bounce_%g_%g" % (xl, yl), type=mujoco.mjtLightType.mjLIGHT_SPOT,
                              pos=[xl, yl, 0.05], dir=[0.0, 0.0, 1.0])
            lt.cutoff, lt.intensity, lt.range, lt.castshadow, lt.bulbradius = 85.0, BOUNCE_CD, 6.0, False, 0.6
    red = bank.plain((0.65, 0.06, 0.05), 0.45)
    for yp in (1.2, -1.2, 3.6, -3.6):
        B.vis(wb, "sprink_%g" % yp, M.transform(M.tube([(x0 - 1, 0, 0), (x1 + 1, 0, 0)], 0.025, 10), t=(0, yp, 3.46)), red)
    B.vis(wb, "ceiling", M.box(((H.HALL["x"][1] - H.HALL["x"][0]) / 2, (H.HALL["y"][1] - H.HALL["y"][0]) / 2, 0.02)),
          bank.textured("ceiling", tint=(0.55, 0.56, 0.58)),
          pos=((H.HALL["x"][0] + H.HALL["x"][1]) / 2, (H.HALL["y"][0] + H.HALL["y"][1]) / 2, H.CEILING_Z + 0.02))


META_REF = {}
SHADOWS = False          # shadow maps cost ~0.16 s of CPU per light per frame; ambient occlusion grounds things
LIGHT_CD = 140000.0
BOUNCE_CD = 9000.0
CONTAIN_CD = 55000.0


# ------------------------------------------------------------------------------------------------ the hall shell
def add_shell(B):
    s, bank = B.s, B.bank
    wb = s.worldbody
    hx, hy = H.HALL["x"], H.HALL["y"]
    # raised floor: tiles aligned to the rows (texture origin at a multiple of 0.6 m), perforated tiles in the cold aisle
    fx0, fx1, fy0, fy1 = -4.8, 6.6, -6.6, 6.6
    B.vis(wb, "floor", M.box(((fx1 - fx0) / 2, (fy1 - fy0) / 2, 0.01)), bank.textured("floor_tile"),
          pos=((fx0 + fx1) / 2, (fy0 + fy1) / 2, -0.01))
    B.vis(wb, "perf", M.box((H.N_RACKS * H.RACK_W / 2, H.COLD_AISLE_HALF, 0.0006)), bank.textured("floor_perf"),
          pos=(H.X0 + H.N_RACKS * H.RACK_W / 2, 0.0, 0.0007))
    wb.add_geom(name="floor_col", type=G.mjGEOM_PLANE, size=[20, 20, 0.1], pos=[0, 0, 0], contype=FLOOR,
                conaffinity=OBJ, group=3)
    wall = bank.textured("wall_paint")
    Hh = H.CEILING_Z
    for name, half, pos in (("wall_n", ((hx[1] - hx[0]) / 2, 0.1, Hh / 2), ((hx[0] + hx[1]) / 2, hy[1] + 0.1, Hh / 2)),
                            ("wall_s", ((hx[1] - hx[0]) / 2, 0.1, Hh / 2), ((hx[0] + hx[1]) / 2, hy[0] - 0.1, Hh / 2)),
                            ("wall_w", (0.1, (hy[1] - hy[0]) / 2, Hh / 2), (hx[0] - 0.1, 0.0, Hh / 2)),
                            ("wall_e", (0.1, (hy[1] - hy[0]) / 2, Hh / 2), (hx[1] + 0.1, 0.0, Hh / 2))):
        B.vis(wb, name, M.box(half), wall, pos=pos)
        B.col(wb, half, pos)
    exitsign = bank.image("exit", T.sign("EXIT", 256, 96, bg=(0.05, 0.55, 0.25)), 0.4, emission=0.6)
    B.vis(wb, "exit", M.panel(0.36, 0.13, 0.03, "+x"), exitsign, pos=(hx[0] + 0.02, 1.0, 2.35))


# ------------------------------------------------------------------------------------------------ background rows
def add_background(B):
    """Two more rows beyond the hot aisles (their rears onto our hot aisles), with their own contained cold aisles:
    drawn and solid, never serviced."""
    s, bank = B.s, B.bank
    _rack_shell(B)
    black = bank.textured("powder_black")
    if "bg_rear" not in B.meshes:
        B.mesh("bg_rear", M.quad(0.44, 1.40, "+y", 0.0, 0.44, 0.0, 1.40))
    for rowname, yrear, sgn in H.BACKGROUND_ROWS:
        yfront = yrear + sgn * H.RACK_D
        yaw = math.pi if sgn > 0 else 0.0
        for i in range(7):
            xc = H.X0 - 0.3 + i * H.RACK_W
            body = s.worldbody.add_body(name="bg_%s%d" % (rowname, i), pos=[xc, yfront, 0.0], quat=quat_z(yaw))
            B.static_frame(body, (xc, yfront, 0.0), quat_z(yaw))
            for mesh in ("rack_frame", "rack_sides", "rack_top"):
                B.vis(body, mesh, None, black)
            B.vis(body, "bg_rear", None, bank.textured("honeycomb", tint=(1.6, 1.6, 1.7)), pos=(0.0, H.REAR_RAIL_Y, 0.95))
            rng = np.random.default_rng(100 + i + (0 if sgn > 0 else 50))
            for k in range(10):
                B.led_static(body, (rng.uniform(-0.2, 0.2), H.REAR_RAIL_Y + 0.004, rng.uniform(0.3, 1.6)),
                             (0.0015, 0.0006, 0.0015), "green" if rng.random() < 0.85 else "blue")
            B.vis(body, "door_perf", None, bank.textured("hex_perf", alpha=0.62), pos=(-(H.RACK_W / 2 - 0.01), H.RACK_D + 0.012, 0.0))
            B.vis(body, "door_frame", None, black, pos=(-(H.RACK_W / 2 - 0.01), H.RACK_D + 0.012, 0.0))
            B.col(body, (H.RACK_W / 2, H.RACK_D / 2, H.RACK_H / 2), (0.0, H.RACK_D / 2, H.RACK_H / 2))
        yr0, yr1 = sorted((yfront, yfront + sgn * 1.2))
        B.vis(s.worldbody, "bg_roof_" + rowname, M.box((7 * H.RACK_W / 2, (yr1 - yr0) / 2, 0.003)),
              bank.plain((0.82, 0.86, 0.88), 0.25, alpha=0.30), pos=(H.X0 + 1.8, (yr0 + yr1) / 2, H.CONTAIN_TOP + 0.003))


# ------------------------------------------------------------------------------------------------ the service area
def add_service(B):
    """The spares cage (wire-mesh walls, shelving with boxed parts), the bench, the RMA tote, the robot's dock."""
    s, bank = B.s, B.bank
    wb = s.worldbody
    c = H.CAGE
    mesh_mat = bank.textured("wire_mesh", alpha=0.75)
    post = bank.plain((0.05, 0.05, 0.055), 0.5)
    cx, cy = (c["x"][0] + c["x"][1]) / 2, (c["y"][0] + c["y"][1]) / 2
    for name, half, pos in (("cage_e", (0.01, (c["y"][1] - c["y"][0]) / 2, c["h"] / 2), (c["x"][1], cy, c["h"] / 2)),
                            ("cage_n", ((c["x"][1] - c["x"][0]) / 2, 0.01, c["h"] / 2), (cx, c["y"][1], c["h"] / 2)),
                            ("cage_s", ((c["x"][1] - c["x"][0]) / 2, 0.01, c["h"] / 2), (cx, c["y"][0], c["h"] / 2))):
        B.vis(wb, name, M.box(half), mesh_mat, pos=pos)
        B.col(wb, half, pos)
    for y in np.linspace(c["y"][0], c["y"][1], 5):
        B.vis(wb, "cpost_%g" % y, M.box((0.02, 0.02, c["h"] / 2)), post, pos=(c["x"][1], y, c["h"] / 2))
    shelf = bank.plain((0.10, 0.11, 0.12), 0.5, 0.3)
    box_mats = [bank.plain((0.55, 0.42, 0.26), 0.85), bank.plain((0.12, 0.30, 0.62), 0.6), bank.plain((0.85, 0.52, 0.65), 0.7)]
    rng = np.random.default_rng(7)
    for k, ys in enumerate((-1.4, 0.0, 1.4)):
        for lvl in range(5):
            z = 0.12 + lvl * 0.42
            B.vis(wb, "shelf_%d_%d" % (k, lvl), M.box((0.30, 0.60, 0.012)), shelf, pos=(c["x"][0] + 0.45, ys, z))
            for j in range(4):
                if rng.random() < 0.8:
                    w, d, h = rng.uniform(0.10, 0.16), rng.uniform(0.18, 0.28), rng.uniform(0.08, 0.20)
                    B.vis(wb, "box_%d_%d_%d" % (k, lvl, j), M.rounded_box((w, d / 2, h / 2), 0.004, n=2),
                          box_mats[int(rng.integers(0, 3))], pos=(c["x"][0] + 0.45, ys - 0.45 + j * 0.30, z + 0.012 + h / 2))
        for sxy in ((-0.30, -0.60), (0.30, -0.60), (-0.30, 0.60), (0.30, 0.60)):
            B.vis(wb, "spost_%d_%g_%g" % (k, sxy[0], sxy[1]), M.box((0.012, 0.012, 1.0)), post,
                  pos=(c["x"][0] + 0.45 + sxy[0], ys + sxy[1], 1.0))
    b = H.BENCH
    B.vis(wb, "bench_top", M.box(((b["x"][1] - b["x"][0]) / 2, (b["y"][1] - b["y"][0]) / 2, 0.02)),
          bank.plain((0.10, 0.30, 0.55), 0.7), pos=((b["x"][0] + b["x"][1]) / 2, (b["y"][0] + b["y"][1]) / 2, b["h"]))
    B.col(wb, ((b["x"][1] - b["x"][0]) / 2, (b["y"][1] - b["y"][0]) / 2, b["h"] / 2),
          ((b["x"][0] + b["x"][1]) / 2, (b["y"][0] + b["y"][1]) / 2, b["h"] / 2))
    B.vis(wb, "dock", M.rounded_box((0.25, 0.20, 0.60), 0.02, n=2), bank.plain((0.14, 0.14, 0.15), 0.5),
          pos=(H.DOCK[0] - 0.5, H.DOCK[1], 0.60))
    B.led(wb, "dock", (H.DOCK[0] - 0.249, H.DOCK[1], 1.0), (0.001, 0.12, 0.008), "blue")


# ------------------------------------------------------------------------------------------------ parts, tools, cart
OBJ_CONTACT = dict(contype=OBJ, conaffinity=WORLD | ROBOT | OBJ | FLOOR, group=3, condim=3, friction=[0.8, 0.005, 0.0001])
POOL = {"fan": 4, "psu": 3, "plug": 2, "osfp_n": 4, "osfp_s": 4, "mpo": 4, "gpu": 3}
CART = {"x": -0.95, "y": -1.55, "top": 0.80, "half": (0.30, 0.45)}
TOTE_XY = (CART["x"], CART["y"] - 0.71)       # the RMA tote just south of the cart (its label's side), on its stand
TOTE_Z = 0.45                                 # the stand's top (the tote's bottom): the rim at 0.75 m
SPARES = [("gpu", (0.12, 0.36), "south", 0.0), ("osfp_n", (-0.22, -0.14), "west", 0.18),
          ("osfp_n", (-0.22, -0.21), "west", 0.18), ("osfp_s", (-0.22, -0.28), "west", 0.18),
          ("osfp_s", (-0.22, -0.35), "west", 0.18), ("psu", (-0.26, -0.02), "west", 0.18),
          ("fan", (-0.26, 0.14), "west", 0.0), ("fan", (-0.26, 0.30), "west", 0.0)]
# kind, (x, y) of the part's origin on the cart's top (cart frame), the way its front (the face with its handle, the
# end that leaves the slot first) faces, and what it stands on: each spare sits the way it goes into its slot, its
# front to the cart's west side; the supply and the transceivers on the foam riser along the west edge (their handles
# at hand height), the fan modules upright on the top, the GPU module by the north edge (fins and lift bar up, taken
# from the north side). Two of each kind a shift can need twice, one GPU and one supply.
RISER = {"x": (-0.29, -0.04), "y": (-0.40, 0.03), "h": 0.18}      # the foam riser (cart frame, m)
FACE_YAW = {"north": 0.0, "west": math.pi / 2, "south": math.pi, "east": -math.pi / 2}
# the tool belt: one holster per tool on the robot's pelvis (pelvis frame: x forward, y left, z up), the tool upright
# with its origin at the holster's mouth; the driver on the right hip, the scope and the cleaner on the left
BELT = {"driver": (0.045, -0.170, 0.020), "scope": (0.070, 0.170, 0.020), "cleaner": (-0.035, 0.170, 0.020)}
BELT_BODY = "g1/pelvis"
PARK_Z = -30.0
SPARE_JITTER = {}                     # proxy name -> (dx, dy) of where this pod's cart holds the spare (build's cfg)
CHAIN_SEG = 0.05
CHAIN_N = {"mpo": 12, "plug": 22}


def add_cart(B, meta):
    """The parts cart by the cold aisle's west door: the spares on its top, the RMA tote (a red bin) on the floor south
    of it. The tools are not here: they ride in the robot's belt (add_belt)."""
    s, bank = B.s, B.bank
    cart = s.worldbody.add_body(name="cart", pos=[CART["x"], CART["y"], 0.0])
    B.static_frame(cart, (CART["x"], CART["y"], 0.0), (1.0, 0.0, 0.0, 0.0))
    hx, hy = CART["half"]
    top = CART["top"]
    steel = bank.plain((0.18, 0.19, 0.20), 0.5, 0.4)
    B.vis(cart, "cart_top", M.box((hx, hy, 0.012)), bank.plain((0.10, 0.30, 0.55), 0.7), pos=(0.0, 0.0, top - 0.012))
    B.vis(cart, "cart_shelf", M.box((hx, hy, 0.010)), steel, pos=(0.0, 0.0, 0.22))
    for sx in (-1, 1):
        for sy in (-1, 1):
            B.vis(cart, "cart_post", M.box((0.012, 0.012, top / 2)) if "cart_post" not in B.meshes else None, steel,
                  pos=(sx * (hx - 0.012), sy * (hy - 0.012), top / 2))
    B.col(cart, (hx, hy, 0.012), (0.0, 0.0, top - 0.012), kind=dict(contype=WORLD, conaffinity=ROBOT | OBJ, group=3, condim=3,
                                                                      friction=[0.8, 0.005, 0.0001]))
    B.col(cart, (hx, hy, top / 2 - 0.02), (0.0, 0.0, top / 2 - 0.02), kind="equip")
    rx, ry, rh = RISER["x"], RISER["y"], RISER["h"]
    rhalf = ((rx[1] - rx[0]) / 2, (ry[1] - ry[0]) / 2, rh / 2)
    rpos = ((rx[0] + rx[1]) / 2, (ry[0] + ry[1]) / 2, top + rh / 2)
    B.vis(cart, "cart_riser", M.box(rhalf), bank.plain((0.22, 0.23, 0.25), 0.9), pos=rpos)
    B.col(cart, rhalf, rpos, kind=dict(contype=WORLD, conaffinity=ROBOT | OBJ, group=3, condim=3,
                                        friction=[0.8, 0.005, 0.0001]))
    lab = bank.image("cart_label", T.label(["SPARES - POD 3", "RMA TOTE: RED BIN, THIS SIDE"], 512, 128), 0.6)
    B.vis(cart, "cart_label", M.panel(0.28, 0.07, 0.002, "-y"), lab, pos=(0.0, -hy - 0.002, top - 0.06))
    # the tote's stand: a steel frame with a top, the tote standing on it
    tw, td, th = 0.28, 0.20, 0.30
    stand = s.worldbody.add_body(name="rma_tote_stand", pos=[TOTE_XY[0], TOTE_XY[1], 0.0])
    steel = bank.plain((0.18, 0.19, 0.21), 0.5)
    # (no wider than the tote: the passage between it and the next row's rears stays as it was)
    B.vis(stand, "tote_stand_top", M.box((tw, td, 0.012)), steel, pos=(0, 0, TOTE_Z - 0.012))
    for sx in (-1, 1):
        for sy in (-1, 1):
            B.vis(stand, "tote_stand_leg_%d_%d" % (sx, sy), M.box((0.015, 0.015, (TOTE_Z - 0.024) / 2)), steel,
                  pos=(sx * (tw - 0.015), sy * (td - 0.015), (TOTE_Z - 0.024) / 2))
    B.vis(stand, "tote_stand_shelf", M.box((tw - 0.03, td - 0.03, 0.008)), steel, pos=(0, 0, 0.12))
    B.col(stand, (tw, td, TOTE_Z / 2), (0, 0, TOTE_Z / 2 - 0.012),
          kind=dict(contype=WORLD, conaffinity=ROBOT | OBJ, group=3, condim=3, friction=[0.8, 0.005, 0.0001]))
    tote = s.worldbody.add_body(name="rma_tote", pos=[TOTE_XY[0], TOTE_XY[1], TOTE_Z])
    red = bank.plain((0.70, 0.08, 0.06), 0.6)
    # a moulded tote: a 3 cm bottom and 2 cm walls (a small part dropped into it does not pass through a thin plate)
    walls = [((tw, td, 0.015), (0, 0, 0.015)), ((0.010, td, th / 2), (-tw, 0, th / 2)), ((0.010, td, th / 2), (tw, 0, th / 2)),
             ((tw, 0.010, th / 2), (0, -td, th / 2)), ((tw, 0.010, th / 2), (0, td, th / 2))]
    for k, (hh, pp) in enumerate(walls):
        B.vis(tote, "tote_%d" % k, M.box(hh), red, pos=pp)
        if k == 0:
            # the bottom's collider reaches 0.2 m into the stand (unseen): a small part dropped in fast is stopped by
            # it however far the soft contact lets it sink (a thin plate lets it through)
            hh, pp = (tw, td, 0.12), (0, 0, 0.03 - 0.12)
        B.col(tote, hh, pp, kind=dict(contype=WORLD, conaffinity=ROBOT | OBJ, group=3, condim=3, friction=[0.8, 0.005, 0.0001],
                                      solref=[0.005, 1.0]))
    meta["rma_tote"] = {"body": "rma_tote", "inside": [[-tw, -td, 0.0], [tw, td, th]]}
    meta["cart"] = {"body": "cart", "top": top, "spares": SPARES}
    return cart


def add_pool(B, meta):
    """The free bodies that stand in for a part while it is out of its slot (parked far below the floor, floating
    without contact, until mech.py hands one a part), the spares (on the cart) and the tools (in their holders)."""
    s = B.s
    wb = s.worldbody
    k_all = 0
    spares = {}
    for kind, n in POOL.items():
        for k in range(n):
            name = "px_%s_%d" % (kind, k)
            park = [20.0 + 0.5 * (k_all % 10), 20.0 + 0.5 * (k_all // 10), PARK_Z]
            k_all += 1
            body = wb.add_body(name=name, pos=park, gravcomp=1.0)
            body.add_freejoint(name=name + "_free")
            P.add_look(B, body, kind)
            half, ctr = P.collider_half(kind)
            B.col(body, half, ctr, kind=dict(OBJ_CONTACT), mass=P.KINDS[kind]["mass"])
            meta["features"][name] = P.KINDS[kind]["features"]
            meta["graspable"].append(name)
            meta["loose"].append(name)
            meta["kind"][name] = kind
            rec = {"kind": kind, "seat_eq": "seat_" + name, "park": park}
            if kind in ("mpo", "plug"):
                end = (0.0, 0.050, 0.0) if kind == "mpo" else (0.0, 0.048, 0.0)
                body.add_site(name="end_" + name, pos=list(end), size=[0.002, 0, 0])
                anc = wb.add_body(name="anc_" + name, pos=[park[0], park[1], PARK_Z + 1.0], mocap=True)
                anc.add_site(name="anc_" + name, size=[0.002, 0, 0])
                radius = 0.0026 if kind == "mpo" else 0.0042
                mat = B.bank.plain((0.10, 0.62, 0.70), 0.45) if kind == "mpo" else B.bank.plain((0.02, 0.02, 0.022), 0.6)
                chain = []
                for i in range(CHAIN_N[kind]):
                    cb = wb.add_body(name="chain_%s_%d" % (name, i), pos=[park[0], park[1], PARK_Z - 1.0 - 0.1 * i], mocap=True)
                    B.prim(cb, G.mjGEOM_CAPSULE, (radius, CHAIN_SEG / 2 + radius * 0.5), (0, 0, 0), mat)
                    chain.append(cb.name)
                rec.update(end_site="end_" + name, anchor_body=anc.name, anchor_site="anc_" + name, tendon="cable_" + name,
                           chain=chain, seg=CHAIN_SEG)
            meta["proxies"][name] = rec
            spares.setdefault(kind, []).append(name)
    # the spares on the cart (not parked): the first proxies of their kinds
    used = {}
    cart_xy = (CART["x"], CART["y"])
    for kind, (cx, cy), face, riser in SPARES:
        i = used.get(kind, 0)
        used[kind] = i + 1
        name = spares[kind][i]
        rec = meta["proxies"][name]
        half, ctr = P.collider_half(kind)
        yaw = FACE_YAW[face]
        quat = [math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)]
        if kind == "gpu":
            z = CART["top"] + 0.002
        else:                                   # as installed, on the top or on the riser
            z = CART["top"] + riser + half[2] - ctr[2] + 0.002
        rec["spare"] = True
        jx, jy = SPARE_JITTER.get(name, (0.0, 0.0))
        rec["start"] = {"pos": [cart_xy[0] + cx + jx, cart_xy[1] + cy + jy, z], "quat": list(quat)}
    for tool, td in P.TOOLS.items():
        name = "tool_" + tool
        bx, by, bz = BELT[tool]
        body = wb.add_body(name=name, pos=[H.ROBOT_START[0] + bx, H.ROBOT_START[1] + by, 0.80 + bz])   # holstered at reset
        body.add_freejoint(name=name + "_free")
        P.add_tool_look(B, body, tool)
        # the collider is the grip body only: the probe's and the cleaner's noses enter a receptacle, the driver's bit a
        # screw well (their tips are judged geometrically: mech.Tools)
        B.col(body, (0.016, 0.016, 0.065), (0.0, 0.0, 0.085), kind=dict(OBJ_CONTACT), mass=td["mass"])
        body.add_site(name="tip_" + tool, pos=list(td["tip"]), size=[0.0015, 0, 0])
        meta["features"][name] = td["features"]
        meta["graspable"].append(name)
        meta["loose"].append(name)
        meta["kind"][name] = "tool"
        meta["tools"][name] = {"tool": tool, "tip_site": "tip_" + tool, "holster_eq": "holster_" + tool,
                               "belt": list(BELT[tool]), "belt_body": BELT_BODY}


PI_2 = math.pi / 2


def add_constraints(s, meta, robot_prefix="g1/"):
    """The model's equality constraints (all but the holsters inactive at first: the cell switches them): every
    hand's grip on every graspable body (a point connect on a fixture, a weld on a part or a tool), each proxy's slot
    weld, the tools' holster welds, the locks of the trays, covers, levers and rear doors; the cables' tendons."""
    for name in meta["graspable"]:
        fixture = name in meta["fixtures"]
        for side in ("left", "right"):
            eq = s.add_equality(name="grip_%s_%s" % (side, name), objtype=mujoco.mjtObj.mjOBJ_BODY,
                                name1=robot_prefix + "%s_wrist_yaw_link" % side, name2=name, active=False)
            eq.type = mujoco.mjtEq.mjEQ_CONNECT if fixture else mujoco.mjtEq.mjEQ_WELD
            eq.solref = [0.02, 1.0]
    for name, rec in meta["proxies"].items():
        s.add_equality(name=rec["seat_eq"], type=mujoco.mjtEq.mjEQ_WELD, objtype=mujoco.mjtObj.mjOBJ_BODY, name1="world",
                       name2=name, active=False, solref=[0.01, 1.0])
        if "tendon" in rec:
            t = s.add_tendon(name=rec["tendon"], limited=mujoco.mjtLimited.mjLIMITED_TRUE, range=[0.0, 2.0])
            t.wrap_site(rec["anchor_site"])
            t.wrap_site(rec["end_site"])
    for name, rec in meta["tools"].items():
        s.add_equality(name=rec["holster_eq"], type=mujoco.mjtEq.mjEQ_WELD, objtype=mujoco.mjtObj.mjOBJ_BODY,
                       name1=rec["belt_body"], name2=name, active=False, solref=[0.01, 1.0])
    for j in [t["joint"] for t in meta["trays"].values()] + [f["joint"] for f in meta["fixtures"].values()
                                                             if f["type"] in ("bezel", "cover", "rdoor", "lever")]:
        s.add_equality(name="lock_" + j, type=mujoco.mjtEq.mjEQ_JOINT, objtype=mujoco.mjtObj.mjOBJ_JOINT, name1=j,
                       active=False, solref=[0.01, 1.0])


def add_belt(B, s):
    """The tool belt's drawn parts on the pelvis: a strap and one open sleeve per tool (no colliders)."""
    pel = s.body(BELT_BODY)
    bank = B.bank
    nylon = bank.plain((0.035, 0.035, 0.038), 0.85)
    if "belt_strap" not in B.meshes:
        ring = [(0.115 * math.cos(a), 0.180 * math.sin(a), 0.022) for a in np.linspace(0, 2 * math.pi, 41)]
        B.mesh("belt_strap", M.tube(ring, 0.009, 8))
        for tool, depth in (("driver", 0.19), ("scope", 0.06), ("cleaner", 0.05)):
            r_in = {"driver": 0.0045, "scope": 0.0085, "cleaner": 0.0075}[tool]
            B.mesh("belt_sleeve_" + tool, M.merge(M.lathe([(0.0, -depth - 0.006), (r_in + 0.006, -depth - 0.006), (r_in + 0.006, 0.0),
                                                            (r_in, 0.0), (r_in, -depth), (0.0, -depth)], 16),
                                                   M.transform(M.box((0.004, 0.012, 0.018)), t=(0.0, 0.0, -0.02))))
    B.vis(pel, "belt_strap", None, nylon)
    for tool, (bx, by, bz) in BELT.items():
        B.vis(pel, "belt_sleeve_" + tool, None, nylon, pos=(bx, by, bz))


# the wrist cameras (a D405-class RGB-D camera on each wrist, above the back of the hand, looking along the grasp
# approach a little down and toed in towards the grasp point, so the fingers fill only the lower part of the picture
# and a held tool's tip and its target are in view): in the wrist_yaw_link frame (right hand; the left one mirrored)
WRIST_CAM = {"pos": (0.020, -0.020, 0.085), "pitch": 0.18, "yaw": 0.20, "fovy": 65.0, "size": (640, 480),
             "depth_min": 0.07, "depth_max": 0.6}


def wrist_cam_quat(pitch, yaw):
    """MuJoCo camera frame (looks along -z, +y up) for an optical axis along the body's +x, pitched down and yawed."""
    d = np.array([math.cos(pitch) * math.cos(yaw), math.cos(pitch) * math.sin(yaw), -math.sin(pitch)])
    z_c = -d
    up = np.array([0.0, 0.0, 1.0]) - d[2] * d
    y_c = up / np.linalg.norm(up)
    x_c = np.cross(y_c, z_c)
    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, np.stack([x_c, y_c, z_c], 1).reshape(-1))
    return q


def wrist_cam_mount(side):
    """(position, quaternion) of a wrist camera in its wrist_yaw_link frame."""
    x, y, z = WRIST_CAM["pos"]
    sgn = 1.0 if side == "right" else -1.0
    return (x, sgn * y, z), wrist_cam_quat(WRIST_CAM["pitch"], sgn * WRIST_CAM["yaw"])


def add_wrist_cameras(s, prefix="g1/"):
    for side in ("left", "right"):
        b = s.body(prefix + "%s_wrist_yaw_link" % side)
        pos, quat = wrist_cam_mount(side)
        b.add_camera(name="wrist_cam_" + side, pos=list(pos), quat=list(quat), fovy=WRIST_CAM["fovy"])


# ------------------------------------------------------------------------------------------------ the model
def build(cfg=None):
    """The cell spec. cfg: {"tray_out": {host: m}, "cover_open": {host: rad},
    "levers_open": {host: rad}, "rear_open": {rack: rad}, "west_door": m, "robot": bool, "states": {...}}.
    Returns (spec, meta)."""
    from . import equipment as E
    cfg = cfg or {}
    s = mujoco.MjSpec()
    s.modelname = "dc_cell"
    s.compiler.degree = False
    o = s.option
    o.timestep = 0.002
    o.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    o.cone = mujoco.mjtCone.mjCONE_PYRAMIDAL
    o.iterations = 60
    o.ls_iterations = 30
    s.memory = 768 * 1024 * 1024
    s.visual.global_.offwidth, s.visual.global_.offheight = 1920, 1080
    s.visual.map.znear = 0.01
    add_view_options(s)
    bank = Bank(s)
    B = Builder(s, bank)
    B.static_frames["world"] = (np.eye(3), np.zeros(3))
    meta = {"racks": {}, "nodes": {}, "trays": {}, "leaves": {}, "ports": {}, "fixtures": {}, "features": {}, "graspable": [],
            "loose": [], "kind": {}, "proxies": {}, "tools": {}, "init_q": {}}
    META_REF.clear()
    META_REF.update(meta)
    meta = META_REF
    add_shell(B)
    for row in H.ROWS:
        for idx in range(1, H.N_RACKS + 1):
            name = H.rack_name(row, idx)
            body = add_rack(B, row, idx, rear_open=cfg.get("rear_open", {}).get(name, 0.0),
                            offset=tuple(cfg.get("pod", {}).get(name, (0.0, 0.0, 0.0))))
            meta["racks"][name] = body.name
            if H.is_network(idx):
                E.add_leaves(B, body, row, meta, cfg.get("states", {}))
            else:
                host = "gpn-%s%02d" % (row.lower(), idx)
                E.add_node(B, body, row, idx, meta, tray_out=cfg.get("tray_out", {}).get(host, 0.0),
                           cover_open=cfg.get("cover_open", {}).get(host, 0.0),
                           levers_open=cfg.get("levers_open", {}).get(host, 0.0), states=cfg.get("states", {}).get(host, {}))
                meta["init_q"]["cover_" + host] = cfg.get("cover_open", {}).get(host, 0.0)
    add_containment(B, west_open=cfg.get("west_door", 0.0))
    add_overhead(B)
    add_background(B)
    add_service(B)
    add_cart(B, meta)
    SPARE_JITTER.clear()
    SPARE_JITTER.update({k: tuple(v) for k, v in cfg.get("spare_jitter", {}).items()})
    add_pool(B, meta)
    if cfg.get("robot", True):
        frame = s.worldbody.add_frame()
        s.attach(g1_spec("g1"), prefix="g1/", frame=frame)
        add_belt(B, s)
    meta["led_materials"] = {st: B.led_material(st) for st in ("off", "green", "amber", "blue", "cyan", "red", "white")}
    B.flush_static()
    if cfg.get("robot", True):
        add_wrist_cameras(s)
        add_constraints(s, meta)
    meta["leds"] = dict(B.leds)
    meta["n_geoms"] = B.n
    return s, meta


def apply_initial_state(m, d, meta):
    """Joint positions of the fixtures (meta["init_q"]), the spares on the cart, the tools in their holders (holster
    welds on), every other proxy parked; then forward kinematics."""
    J = mujoco.mjtObj.mjOBJ_JOINT
    for jn, q in meta["init_q"].items():
        j = mujoco.mj_name2id(m, J, jn)
        if j >= 0:
            d.qpos[m.jnt_qposadr[j]] = float(q)
    for name, rec in meta["proxies"].items():
        j = mujoco.mj_name2id(m, J, name + "_free")
        a = m.jnt_qposadr[j]
        b = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, name)
        if rec.get("spare"):
            d.qpos[a:a + 3] = rec["start"]["pos"]
            d.qpos[a + 3:a + 7] = rec["start"]["quat"]
            m.body_gravcomp[b] = 0.0
        else:
            d.qpos[a:a + 3] = rec["park"]
            d.qpos[a + 3:a + 7] = [1.0, 0.0, 0.0, 0.0]
            m.body_gravcomp[b] = 1.0
    mujoco.mj_forward(m, d)

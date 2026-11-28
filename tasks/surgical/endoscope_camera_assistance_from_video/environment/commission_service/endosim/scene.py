"""The rendered scene: the cavity, the instruments, the scope with its coaxial light, as a USD stage.

Runs inside the Isaac Sim process (render_server.py).

The tissue moves (version 0.3): the floor mesh's vertices are its resting surface moved by tissue.Tissue.displacement,
the breathing and the traction of a hand that holds the tissue, recomputed every frame from the frame's time and the
instruments' tips in the state (so a probe copy's moved instrument drags the tissue as the true one would), with the
normals recomputed from the moved heights; what lies on the tissue moves with it. The stage is authored in centimetres (metersPerUnit 0.01): with
a stage in metres the RTX renderer ignores light sources a few millimetres across, which is what a scope's light is
(measured, tests/README.md section 2). Nothing in the stage has a physics schema: every prim is posed kinematically
from the episode's state, once per frame.

A corner of every frame carries the frame clock: a small emissive quad in the camera's own frame whose slot along the
top-left edge encodes the frame's serial number. The renderer's annotator sometimes returns the previous frame; the
server reads the clock back and renders again until the frame shows the state it was asked for. The corner lies
outside the endoscope's image disk and is masked before a frame leaves the server (post.py).
"""

import math
import os

import numpy as np
from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdShade, Vt

from . import looks, model
from .tissue import MESH_STEP, WALL_AXES, WALL_CENTRE, Tissue

K = 100.0                     # stage units per metre
SHAFT_LEN = 0.40              # m, from the wrist up through the trocar
CLOCK_SLOTS = 8
CLOCK_U0, CLOCK_PITCH, CLOCK_V, CLOCK_PX = 10.0, 10.0, 12.0, 8.0     # slot centres (px), quad size (px)
CLOCK_DEPTH = 0.002           # m in front of the lens: nearer than anything but a shaft run through the lens itself
NEAR_CLIP = 0.001


def _material(stage, path, rgb=None, metallic=0.0, roughness=0.5, clearcoat=0.0, emissive=None, colour_map=None,
              normal_map=None, tile=(1.0, 1.0)):
    m = UsdShade.Material.Define(stage, path)
    sh = UsdShade.Shader.Define(stage, path + "/Surface")
    sh.CreateIdAttr("UsdPreviewSurface")
    if colour_map:
        rd = UsdShade.Shader.Define(stage, path + "/st")
        rd.CreateIdAttr("UsdPrimvarReader_float2")
        rd.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
        rd.CreateOutput("result", Sdf.ValueTypeNames.Float2)
        tx = UsdShade.Shader.Define(stage, path + "/colour")
        tx.CreateIdAttr("UsdUVTexture")
        tx.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(colour_map)
        tx.CreateInput("sourceColorSpace", Sdf.ValueTypeNames.Token).Set("sRGB")
        tx.CreateInput("wrapS", Sdf.ValueTypeNames.Token).Set("mirror")
        tx.CreateInput("wrapT", Sdf.ValueTypeNames.Token).Set("mirror")
        tx.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(rd.ConnectableAPI(), "result")
        tx.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
        sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(tx.ConnectableAPI(), "rgb")
        if normal_map:
            rd2 = UsdShade.Shader.Define(stage, path + "/st_fine")
            rd2.CreateIdAttr("UsdPrimvarReader_float2")
            rd2.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st_fine")
            rd2.CreateOutput("result", Sdf.ValueTypeNames.Float2)
            nm = UsdShade.Shader.Define(stage, path + "/normal")
            nm.CreateIdAttr("UsdUVTexture")
            nm.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(normal_map)
            nm.CreateInput("sourceColorSpace", Sdf.ValueTypeNames.Token).Set("raw")
            nm.CreateInput("wrapS", Sdf.ValueTypeNames.Token).Set("mirror")
            nm.CreateInput("wrapT", Sdf.ValueTypeNames.Token).Set("mirror")
            nm.CreateInput("scale", Sdf.ValueTypeNames.Float4).Set(Gf.Vec4f(2.0, 2.0, 2.0, 1.0))
            nm.CreateInput("bias", Sdf.ValueTypeNames.Float4).Set(Gf.Vec4f(-1.0, -1.0, -1.0, 0.0))
            nm.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(rd2.ConnectableAPI(), "result")
            nm.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
            sh.CreateInput("normal", Sdf.ValueTypeNames.Normal3f).ConnectToSource(nm.ConnectableAPI(), "rgb")
    else:
        sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*[float(c) for c in rgb]))
    sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(float(metallic))
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(float(min(max(roughness, 0.05), 1.0)))
    if clearcoat:
        sh.CreateInput("clearcoat", Sdf.ValueTypeNames.Float).Set(float(clearcoat))
        sh.CreateInput("clearcoatRoughness", Sdf.ValueTypeNames.Float).Set(0.06)
    if emissive is not None:
        sh.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*[float(c) for c in emissive]))
    m.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    return m


def _bind(prim, material):
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(material)


def _grid_mesh(stage, path, pts, nx, ny, st, st_fine, normals, flip=False):
    """A quad grid mesh of ny x nx points (row-major), with two UV sets and smooth normals."""
    idx = np.arange(nx * ny, dtype=np.int32).reshape(ny, nx)
    a, b, c, d = idx[:-1, :-1], idx[:-1, 1:], idx[1:, 1:], idx[1:, :-1]
    quads = np.stack([a, d, c, b] if flip else [a, b, c, d], axis=-1).reshape(-1)
    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(np.ascontiguousarray(pts, dtype=np.float32)))
    mesh.CreateFaceVertexCountsAttr(Vt.IntArray.FromNumpy(np.full((nx - 1) * (ny - 1), 4, dtype=np.int32)))
    mesh.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(np.ascontiguousarray(quads, dtype=np.int32)))
    mesh.CreateSubdivisionSchemeAttr("none")
    mesh.CreateDoubleSidedAttr(True)
    mesh.CreateNormalsAttr(Vt.Vec3fArray.FromNumpy(np.ascontiguousarray(normals, dtype=np.float32)))
    mesh.SetNormalsInterpolation(UsdGeom.Tokens.vertex)
    api = UsdGeom.PrimvarsAPI(mesh.GetPrim())
    api.CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex).Set(
        Vt.Vec2fArray.FromNumpy(np.ascontiguousarray(st, dtype=np.float32)))
    api.CreatePrimvar("st_fine", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex).Set(
        Vt.Vec2fArray.FromNumpy(np.ascontiguousarray(st_fine, dtype=np.float32)))
    return mesh


def _cyl(stage, path, radius, z0, z1, material, axis="Z"):
    c = UsdGeom.Cylinder.Define(stage, path)
    c.CreateRadiusAttr(radius * K)
    c.CreateHeightAttr(abs(z1 - z0) * K)
    c.CreateAxisAttr(axis)
    UsdGeom.XformCommonAPI(c).SetTranslate(Gf.Vec3d(0.0, 0.0, 0.5 * (z0 + z1) * K))
    _bind(c.GetPrim(), material)
    return c


def _box(stage, path, size, centre, material):
    c = UsdGeom.Cube.Define(stage, path)
    c.CreateSizeAttr(1.0)
    api = UsdGeom.XformCommonAPI(c)
    api.SetTranslate(Gf.Vec3d(*[v * K for v in centre]))
    api.SetScale(Gf.Vec3f(*[v * K for v in size]))
    _bind(c.GetPrim(), material)
    return c


def _size(look):
    """An instrument look's scale against the 5 mm instrument the geometry below is written for."""
    return float(look.get("shaft_mm", 5)) / 5.0


class Scene(object):
    """One procedure's stage. `apply(state)` poses it for a frame."""

    _loads = 0

    def __init__(self, stage, spec, texture_dir):
        Scene._loads += 1
        for old in os.listdir(texture_dir):                 # the previous scene's textures
            try:
                os.remove(os.path.join(texture_dir, old))
            except OSError:
                pass
        self.stage, self.spec = stage, spec
        self.scope = spec["scope"]
        self.k = model.SCOPES[self.scope]
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 1.0 / K)
        UsdGeom.Xform.Define(stage, "/World")
        UsdGeom.Scope.Define(stage, "/World/Looks")
        self._cavity(texture_dir)
        self._scope_rig()
        self.inst = {}
        self.trocars = {r: np.array(v, dtype=float) for r, v in spec["trocars"].items()}
        used = spec.get("looks_used") or {r: list(range(len(spec["instrument_looks"]))) for r in self.trocars}
        for role in self.trocars:
            self.inst[role] = self._instrument(role, used[role])
        self._shown = {}

    # -- the cavity ---------------------------------------------------------------------------------------------
    def _cavity(self, texture_dir):
        stage, look = self.stage, self.spec["look"]
        tissue = Tissue(self.spec["tissue"])
        paths = {}
        for name, wall in (("tissue", False), ("wall", True)):
            colour, normal = looks.tissue_maps(look, size=3072 if not wall else 1536, wall=wall)
            # a fresh file name per scene: the renderer's texture cache goes by path
            tag = "%s_%d_%d" % (name, int(look["texture_seed"]), Scene._loads)
            paths[name] = (os.path.join(texture_dir, tag + "_colour.png"), os.path.join(texture_dir, tag + "_normal.png"))
            looks.write_png(paths[name][0], colour)
            looks.write_png(paths[name][1], normal)
        xs, ys, z = tissue.grid()
        gx, gy = np.meshgrid(xs, ys, indexing="xy")
        pts = np.stack([gx, gy, z], axis=-1).reshape(-1, 3) * K
        dzdy, dzdx = np.gradient(z, MESH_STEP, MESH_STEP)
        nrm = np.stack([-dzdx, -dzdy, np.ones_like(z)], axis=-1)
        nrm /= np.linalg.norm(nrm, axis=-1, keepdims=True)
        # one texture over the whole floor (about 0.14 mm a texel), the relief map aligned with the colour map
        st = np.stack([(gx - xs[0]) / (xs[-1] - xs[0]), (gy - ys[0]) / (ys[-1] - ys[0])], axis=-1).reshape(-1, 2)
        st_fine = st
        UsdGeom.Xform.Define(stage, "/World/Tissue")
        mesh = _grid_mesh(stage, "/World/Tissue/Floor", pts, len(xs), len(ys), st, st_fine, nrm.reshape(-1, 3))
        self.tissue, self.floor = tissue, mesh
        self.rest = (gx, gy, z)
        self.floor_points, self.floor_normals = mesh.GetPointsAttr(), mesh.GetNormalsAttr()
        self.clutter_ops = []
        _bind(mesh.GetPrim(), _material(stage, "/World/Looks/tissue", roughness=look["roughness"], clearcoat=look["clearcoat"],
                                        colour_map=paths["tissue"][0], normal_map=paths["tissue"][1]))
        # what lies in the field: gauze swabs and clips, on the tissue (they breathe with it)
        for i, item in enumerate(self.spec.get("clutter") or []):
            path = "/World/Tissue/Clutter%d" % i
            x = UsdGeom.Xform.Define(stage, path)
            op = x.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble)
            op.Set(Gf.Vec3d(*[float(v) * K for v in item["pos"]]))
            self.clutter_ops.append((op, np.array(item["pos"], dtype=float)))
            x.AddRotateZOp(UsdGeom.XformOp.PrecisionDouble).Set(math.degrees(float(item["yaw"])))
            if item["kind"] == "gauze":
                mat = _material(stage, "/World/Looks/clutter%d" % i, rgb=item["rgb"], roughness=1.0)
                _box(stage, path + "/Swab", (item["size"][0], item["size"][1], 0.003), (0.0, 0.0, 0.0), mat)
            else:
                mat = _material(stage, "/World/Looks/clutter%d" % i, rgb=item["rgb"], metallic=1.0, roughness=0.3)
                c = UsdGeom.Cylinder.Define(stage, path + "/Clip")
                c.CreateRadiusAttr(float(item["size"][1]) * K)
                c.CreateHeightAttr(float(item["size"][0]) * K)
                c.CreateAxisAttr("X")
                _bind(c.GetPrim(), mat)
        # the abdominal wall: the upper half of the ellipsoid, seen from inside
        cx, cy, cz = WALL_CENTRE
        ax, ay, az = WALL_AXES
        th = np.linspace(0.0, 0.5 * math.pi, 40)          # from the equator up
        ph = np.linspace(0.0, 2.0 * math.pi, 97)
        gt, gp = np.meshgrid(th, ph, indexing="xy")
        wx = cx + ax * np.cos(gt) * np.cos(gp)
        wy = cy + ay * np.cos(gt) * np.sin(gp)
        wz = cz + az * np.sin(gt)
        wn = -np.stack([np.cos(gt) * np.cos(gp) / ax, np.cos(gt) * np.sin(gp) / ay, np.sin(gt) / az], axis=-1)
        wn /= np.linalg.norm(wn, axis=-1, keepdims=True)
        wpts = np.stack([wx, wy, wz], axis=-1).reshape(-1, 3) * K
        wst = np.stack([gp / (2 * math.pi) * 4.0, gt / (0.5 * math.pi) * 1.0], axis=-1).reshape(-1, 2)
        wall = _grid_mesh(stage, "/World/Wall", wpts, len(th), len(ph), wst, wst, wn.reshape(-1, 3))
        _bind(wall.GetPrim(), _material(stage, "/World/Looks/wall", roughness=min(1.0, look["roughness"] + 0.1),
                                        clearcoat=0.5 * look["clearcoat"], colour_map=paths["wall"][0], normal_map=paths["wall"][1]))

    # -- the scope: camera, light, frame clock ---------------------------------------------------------------------
    def _scope_rig(self):
        stage, k = self.stage, self.k
        rig = UsdGeom.Xform.Define(stage, "/World/Scope")
        self.scope_op = rig.AddTransformOp(UsdGeom.XformOp.PrecisionDouble)
        cam = UsdGeom.Camera.Define(stage, "/World/Scope/Cam")
        aperture = 20.955
        cam.CreateHorizontalApertureAttr(aperture)
        cam.CreateVerticalApertureAttr(aperture * model.IMAGE_H / model.IMAGE_W * k["fx"] / k["fy"])
        cam.CreateFocalLengthAttr(k["fx"] * aperture / model.IMAGE_W)
        cam.CreateClippingRangeAttr(Gf.Vec2f(NEAR_CLIP * K, 2.0 * K))
        cam.CreateFStopAttr(0.0)
        cam.CreateProjectionAttr("perspective")
        intensity = 6.0e5 * float(self.spec["look"]["light"])
        for i, ang in enumerate((90.0, 210.0, 330.0)):       # three fibre bundles round the lens
            lt = UsdLux.SphereLight.Define(stage, "/World/Scope/Light%d" % i)
            lt.CreateRadiusAttr(0.0016 * K)
            lt.CreateIntensityAttr(intensity * 1.9)
            lt.CreateColorAttr(Gf.Vec3f(1.0, 0.98, 0.94))
            a = math.radians(ang)
            UsdGeom.XformCommonAPI(lt).SetTranslate(Gf.Vec3d(0.0038 * K * math.cos(a), 0.0038 * K * math.sin(a), 0.0005 * K))
            shaping = UsdLux.ShapingAPI.Apply(lt.GetPrim())
            shaping.CreateShapingConeAngleAttr(62.0)
            shaping.CreateShapingConeSoftnessAttr(0.45)
        # the frame clock
        s = CLOCK_PX / k["fx"] * CLOCK_DEPTH * K
        quad = UsdGeom.Mesh.Define(stage, "/World/Scope/Clock")
        quad.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(-s / 2, -s / 2, 0), Gf.Vec3f(s / 2, -s / 2, 0), Gf.Vec3f(s / 2, s / 2, 0),
                                             Gf.Vec3f(-s / 2, s / 2, 0)]))
        quad.CreateFaceVertexCountsAttr(Vt.IntArray([4]))
        quad.CreateFaceVertexIndicesAttr(Vt.IntArray([0, 1, 2, 3]))
        quad.CreateDoubleSidedAttr(True)
        UsdGeom.PrimvarsAPI(quad.GetPrim()).CreatePrimvar("doNotCastShadows", Sdf.ValueTypeNames.Bool).Set(True)
        self.clock_op = quad.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble)
        _bind(quad.GetPrim(), _material(stage, "/World/Looks/clock", rgb=(1, 1, 1), roughness=1.0, emissive=(6.0, 6.0, 6.0)))
        # a black plate behind the clock's slots, so that whatever the scene shows in that corner (a white shaft, a
        # highlight) cannot be read as the clock; it reflects nothing and casts no shadow
        depth = CLOCK_DEPTH * 1.15
        u0, u1 = CLOCK_U0 - CLOCK_PITCH, CLOCK_U0 + CLOCK_PITCH * CLOCK_SLOTS
        v0, v1 = CLOCK_V - 1.5 * CLOCK_PX, CLOCK_V + 1.5 * CLOCK_PX
        corners = [((u - k["cx"]) / k["fx"] * depth * K, -(v - k["cy"]) / k["fy"] * depth * K, -depth * K)
                   for u, v in ((u0, v1), (u1, v1), (u1, v0), (u0, v0))]
        plate = UsdGeom.Mesh.Define(stage, "/World/Scope/ClockPlate")
        plate.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*c) for c in corners]))
        plate.CreateFaceVertexCountsAttr(Vt.IntArray([4]))
        plate.CreateFaceVertexIndicesAttr(Vt.IntArray([0, 1, 2, 3]))
        plate.CreateDoubleSidedAttr(True)
        UsdGeom.PrimvarsAPI(plate.GetPrim()).CreatePrimvar("doNotCastShadows", Sdf.ValueTypeNames.Bool).Set(True)
        black = UsdShade.Material.Define(stage, "/World/Looks/clock_plate")
        bsh = UsdShade.Shader.Define(stage, "/World/Looks/clock_plate/Surface")
        bsh.CreateIdAttr("UsdPreviewSurface")
        bsh.CreateInput("useSpecularWorkflow", Sdf.ValueTypeNames.Int).Set(1)
        bsh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.0, 0.0, 0.0))
        bsh.CreateInput("specularColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.0, 0.0, 0.0))
        bsh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(1.0)
        black.CreateSurfaceOutput().ConnectToSource(bsh.ConnectableAPI(), "surface")
        _bind(plate.GetPrim(), black)

    def clock_position(self, serial):
        k = self.k
        u = CLOCK_U0 + CLOCK_PITCH * (int(serial) % CLOCK_SLOTS)
        x = (u - k["cx"]) / k["fx"] * CLOCK_DEPTH
        y = -(CLOCK_V - k["cy"]) / k["fy"] * CLOCK_DEPTH
        return Gf.Vec3d(x * K, y * K, -CLOCK_DEPTH * K)

    # -- instruments -------------------------------------------------------------------------------------------------
    def _instrument(self, role, look_ids):
        stage = self.stage
        root = UsdGeom.Xform.Define(stage, "/World/Inst_" + role)
        rec = {"op": root.AddTransformOp(UsdGeom.XformOp.PrecisionDouble), "looks": {}, "img": UsdGeom.Imageable(root.GetPrim())}
        # the cannula the shaft runs in: slides along the shaft as the instrument moves through the trocar
        can = UsdGeom.Xform.Define(stage, "/World/Inst_%s/Cannula" % role)
        rec["cannula_op"] = can.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble)
        plastic = _material(stage, "/World/Looks/cannula_" + role, rgb=(0.78, 0.80, 0.82), roughness=0.35)
        widest = max([_size(self.spec["instrument_looks"][lid]) for lid in look_ids] or [1.0])
        _cyl(stage, "/World/Inst_%s/Cannula/Sleeve" % role, 0.0042 * widest, -0.030, 0.010, plastic)
        for lid in look_ids:
            look = self.spec["instrument_looks"][lid]
            base = "/World/Inst_%s/Look%d" % (role, lid)
            grp = UsdGeom.Xform.Define(stage, base)
            if _size(look) != 1.0:
                # a 10 mm instrument is a 5 mm one built twice as large about its tip (shaft, wrist, jaws, bands)
                grp.AddScaleOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(_size(look), _size(look), _size(look)))
            mp = "/World/Looks/%s_%d_" % (role, lid)
            shaft = _material(stage, mp + "shaft", rgb=look["shaft_rgb"], metallic=look["shaft_metallic"], roughness=look["shaft_rough"])
            # blood and tissue on the working end: the wrist and the jaws go from bare metal towards a wet dark red
            st = float(look.get("stain") or 0.0)
            blood = np.array(self.spec["look"]["vessel"]) * 0.9
            stained = lambda rgb, k: [float(v) for v in (1.0 - k) * np.array(rgb) + k * blood]   # noqa: E731
            wrist = _material(stage, mp + "wrist", rgb=stained(look["wrist_rgb"], 0.7 * st), metallic=1.0 - 0.7 * st, roughness=look["wrist_rough"])
            jawm = _material(stage, mp + "jaw", rgb=stained(look["jaw_rgb"], st), metallic=1.0 - st, roughness=look["jaw_rough"],
                             clearcoat=0.6 * st)
            r = 0.5 * model.SHAFT_DIAMETER
            jl, wl, jaw = look["jaw_len"], look["wrist_len"], look["jaw"]
            ops = []
            if jaw == "suction":
                _cyl(stage, base + "/Shaft", r, 0.004, SHAFT_LEN, shaft)
                _cyl(stage, base + "/End", r * 0.96, 0.0, 0.004, wrist)
            else:
                _cyl(stage, base + "/Shaft", r, jl + wl, SHAFT_LEN, shaft)
                _cyl(stage, base + "/Wrist", r * 0.86, jl, jl + wl, wrist)
                if jaw == "hook":
                    _cyl(stage, base + "/Rod", 0.0007, 0.0, jl, jawm)
                    bend = UsdGeom.Cylinder.Define(stage, base + "/Bend")
                    bend.CreateRadiusAttr(0.0007 * K)
                    bend.CreateHeightAttr(0.0045 * K)
                    bend.CreateAxisAttr("X")
                    UsdGeom.XformCommonAPI(bend).SetTranslate(Gf.Vec3d(0.0022 * K, 0.0, 0.0004 * K))
                    _bind(bend.GetPrim(), jawm)
                else:
                    width, thick = {"grasper": (0.0040, 0.0016), "scissors": (0.0030, 0.0009), "needle_driver": (0.0034, 0.0022),
                                    "clip_applier": (0.0048, 0.0020)}[jaw]
                    for name, side in (("JawA", 1.0), ("JawB", -1.0)):
                        hinge = UsdGeom.Xform.Define(stage, base + "/" + name)
                        hinge.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(0.0, 0.0, jl * K))
                        ops.append((hinge.AddRotateXOp(UsdGeom.XformOp.PrecisionDouble), side))
                        _box(stage, base + "/" + name + "/Blade", (width, thick, jl), (0.0, side * 0.5 * thick, -0.5 * jl), jawm)
            if look.get("band_rgb"):
                band = _material(stage, mp + "band", rgb=look["band_rgb"], roughness=0.4)
                z0 = jl + wl + look["band_at"]
                _cyl(stage, base + "/Band", r * 1.03, z0, z0 + look["band_len"], band)
            rec["looks"][lid] = {"img": UsdGeom.Imageable(grp.GetPrim()), "jaws": ops}
            UsdGeom.Imageable(grp.GetPrim()).MakeInvisible()
        return rec

    @staticmethod
    def _shaft_pose(tip, trocar):
        """The instrument's frame: origin at the tip, +z up the shaft towards its trocar."""
        z = trocar - tip
        z = z / np.linalg.norm(z)
        x = np.cross(np.array([0.0, 1.0, 0.0]), z)
        x = x / np.linalg.norm(x)
        y = np.cross(z, x)
        m = np.eye(4)
        m[0, :3], m[1, :3], m[2, :3], m[3, :3] = x, y, z, tip * K
        return Gf.Matrix4d(m.tolist())

    # -- one frame ---------------------------------------------------------------------------------------------------
    def _move_tissue(self, state):
        """The floor's vertices and normals, and the clutter, where the moving tissue has them at this frame."""
        time_s = float(state["time_s"])
        grips = [(s["grip"], s["tip"]) for r, s in state["inst"].items() if s.get("grip") is not None and s["present"]]
        gx, gy, z = self.rest
        d = self.tissue.displacement(gx, gy, time_s, grips)
        pts = np.stack([gx + d[..., 0], gy + d[..., 1], z + d[..., 2]], axis=-1)
        zz = z + d[..., 2]
        dzdy, dzdx = np.gradient(zz, MESH_STEP, MESH_STEP)
        nrm = np.stack([-dzdx, -dzdy, np.ones_like(zz)], axis=-1)
        nrm /= np.linalg.norm(nrm, axis=-1, keepdims=True)
        self.floor_points.Set(Vt.Vec3fArray.FromNumpy(np.ascontiguousarray(pts.reshape(-1, 3) * K, dtype=np.float32)))
        self.floor_normals.Set(Vt.Vec3fArray.FromNumpy(np.ascontiguousarray(nrm.reshape(-1, 3), dtype=np.float32)))
        for op, pos in self.clutter_ops:
            dp = self.tissue.displacement(np.array([pos[0]]), np.array([pos[1]]), time_s, grips)[0]
            op.Set(Gf.Vec3d(*[float(v) * K for v in pos + dp]))

    def apply(self, state):
        with Sdf.ChangeBlock():
            r = np.array(state["cam"]["r"], dtype=float)
            p = np.array(state["cam"]["p"], dtype=float)
            m = np.eye(4)
            m[0, :3], m[1, :3], m[2, :3], m[3, :3] = r[:, 0], r[:, 1], r[:, 2], p * K
            self.scope_op.Set(Gf.Matrix4d(m.tolist()))
            self.clock_op.Set(self.clock_position(state["serial"]))
            self._move_tissue(state)
            for role, s in state["inst"].items():
                rec = self.inst[role]
                shown = (bool(s["present"]), int(s["look"]))
                if self._shown.get(role) != shown:
                    for lid, lk in rec["looks"].items():
                        if shown[0] and lid == shown[1]:
                            lk["img"].MakeVisible()
                        else:
                            lk["img"].MakeInvisible()
                    self._shown[role] = shown
                tip = np.array(s["tip"], dtype=float)
                trocar = self.trocars[role]
                rec["op"].Set(self._shaft_pose(tip, trocar))
                rec["cannula_op"].Set(Gf.Vec3d(0.0, 0.0, float(np.linalg.norm(trocar - tip)) * K))
                if shown[0] and shown[1] in rec["looks"]:
                    half = math.degrees(0.5 * float(s["jaw"]))
                    for op, side in rec["looks"][shown[1]]["jaws"]:
                        op.Set(side * half)


class MarkerScene(object):
    """The render canary's stage: a matte floor, the scope rig of one scope, and one emissive marker ball that the
    canary places at known points; where the ball lands in the frame is compared with the public camera model."""

    def __init__(self, stage, scope):
        self.stage, self.scope, self.k = stage, scope, model.SCOPES[scope]
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 1.0 / K)
        UsdGeom.Xform.Define(stage, "/World")
        UsdGeom.Scope.Define(stage, "/World/Looks")
        self.spec = {"look": {"light": 1.0}}
        Scene._scope_rig(self)
        _box(stage, "/World/Floor", (0.6, 0.6, 0.01), (0.0, 0.06, -0.20), _material(stage, "/World/Looks/floor", rgb=(0.35, 0.2, 0.2), roughness=0.8))
        ball = UsdGeom.Sphere.Define(stage, "/World/Marker")
        ball.CreateRadiusAttr(0.003 * K)
        self.marker_op = ball.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble)
        _bind(ball.GetPrim(), _material(stage, "/World/Looks/marker", rgb=(0, 1, 0), roughness=1.0, emissive=(0.0, 8.0, 0.0)))

    clock_position = Scene.clock_position

    def apply(self, serial, pose, marker):
        r, p = pose
        m = np.eye(4)
        m[0, :3], m[1, :3], m[2, :3], m[3, :3] = r[:, 0], r[:, 1], r[:, 2], np.asarray(p) * K
        with Sdf.ChangeBlock():
            self.scope_op.Set(Gf.Matrix4d(m.tolist()))
            self.clock_op.Set(self.clock_position(serial))
            self.marker_op.Set(Gf.Vec3d(*[float(v) * K for v in marker]))

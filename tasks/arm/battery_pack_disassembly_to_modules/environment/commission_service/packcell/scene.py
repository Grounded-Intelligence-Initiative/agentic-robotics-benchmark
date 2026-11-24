"""Build the disassembly cell and one pack as a USD stage (pxr only; nothing here needs a running simulator).

`build(stage, pack, robot_usd)` authors everything under `/World` and returns an index of the prims the cell drives:
the robot's articulation with its two added links (the tool carrier `ee_link`, fixed to the flange, and the
nutrunner's sprung spindle `spindle_link`), the tools' geometry (both tools are children of `ee_link`; the cell shows
and collides only the mounted one), the parts and fasteners of the pack as rigid bodies (kinematic until the rules
free them), the bins, the rack, the stands and the two cameras. All geometry comes from `series.py`.
"""

import math

import numpy as np
from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdLux, UsdPhysics, UsdShade

from . import kin
from . import series as S

LOOKS = {
    # name: (diffuse rgb, metallic, roughness)
    "floor": ((0.42, 0.42, 0.40), 0.0, 0.85),
    "table": ((0.23, 0.27, 0.31), 0.1, 0.60),
    "fixture": ((0.50, 0.52, 0.54), 0.6, 0.50),
    "stand": ((0.80, 0.62, 0.10), 0.1, 0.55),
    "bin_blue": ((0.05, 0.20, 0.55), 0.0, 0.50),
    "bin_red": ((0.60, 0.08, 0.06), 0.0, 0.50),
    "bin_green": ((0.07, 0.40, 0.16), 0.0, 0.50),
    "bin_grey": ((0.45, 0.47, 0.50), 0.0, 0.50),
    "rack": ((0.85, 0.80, 0.70), 0.0, 0.60),
    "pedestal": ((0.15, 0.16, 0.18), 0.3, 0.50),
    "tray": ((0.50, 0.51, 0.53), 0.7, 0.50),
    "lid": ((0.055, 0.060, 0.070), 0.2, 0.42),
    "lid_rib": ((0.075, 0.080, 0.090), 0.2, 0.42),
    "handle": ((0.85, 0.83, 0.10), 0.0, 0.50),
    "hole": ((0.01, 0.01, 0.01), 0.0, 0.90),
    "module": ((0.20, 0.21, 0.23), 0.5, 0.50),
    "module_top": ((0.55, 0.56, 0.58), 0.7, 0.45),
    "strap": ((0.02, 0.02, 0.02), 0.0, 0.70),
    "copper": ((0.78, 0.45, 0.22), 1.0, 0.30),
    "hv_orange": ((1.00, 0.36, 0.02), 0.0, 0.45),
    "bms": ((0.03, 0.03, 0.035), 0.0, 0.55),
    "conn_grey": ((0.60, 0.60, 0.58), 0.0, 0.50),
    "conn_white": ((0.92, 0.92, 0.88), 0.0, 0.50),
    "bracket": ((0.74, 0.66, 0.30), 0.9, 0.40),
    "insulator": ((0.75, 0.10, 0.08), 0.0, 0.55),
    "harness": ((0.10, 0.22, 0.80), 0.0, 0.55),
    "zinc": ((0.72, 0.72, 0.75), 1.0, 0.35),
    "rust": ((0.42, 0.17, 0.06), 0.2, 0.92),
    "rust_stain": ((0.33, 0.14, 0.05), 0.0, 0.95),
    "recess": ((0.005, 0.005, 0.005), 0.0, 0.90),
    "tool_blue": ((0.05, 0.12, 0.40), 0.3, 0.45),
    "tool_black": ((0.03, 0.03, 0.03), 0.2, 0.50),
    "steel": ((0.72, 0.72, 0.74), 1.0, 0.25),
    "changer": ((0.70, 0.05, 0.05), 0.4, 0.40),
    "alu": ((0.80, 0.81, 0.83), 1.0, 0.35),
}


class Builder(object):
    def __init__(self, stage):
        self.st = stage
        self.mats = {}

    # ------------------------------------------------------------------ materials and primitives
    def mat(self, name):
        if name not in self.mats:
            color, metallic, rough = LOOKS[name]
            path = "/World/Looks/" + name
            m = UsdShade.Material.Define(self.st, path)
            sh = UsdShade.Shader.Define(self.st, path + "/Shader")
            sh.CreateIdAttr("UsdPreviewSurface")
            sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
            sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(float(metallic))
            sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(float(rough))
            m.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
            self.mats[name] = m
        return self.mats[name]

    def bind(self, prim, look):
        if look:
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(self.mat(look))

    @staticmethod
    def place(prim, pos=(0, 0, 0), quat=None, scale=None):
        """translate, orient (w x y z), scale ops on a fresh prim."""
        xf = UsdGeom.Xformable(prim)
        xf.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*[float(v) for v in pos]))
        q = (1.0, 0.0, 0.0, 0.0) if quat is None else [float(v) for v in quat]
        xf.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(q[0], q[1], q[2], q[3]))
        if scale is not None:
            xf.AddScaleOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*[float(v) for v in scale]))

    def xform(self, path, pos=(0, 0, 0), quat=None):
        x = UsdGeom.Xform.Define(self.st, path)
        self.place(x.GetPrim(), pos, quat)
        return x.GetPrim()

    def box(self, path, size, pos=(0, 0, 0), look=None, collide=False, quat=None, visible=True):
        c = UsdGeom.Cube.Define(self.st, path)
        c.CreateSizeAttr(1.0)
        self.place(c.GetPrim(), pos, quat, size)
        self.bind(c.GetPrim(), look)
        if collide:
            UsdPhysics.CollisionAPI.Apply(c.GetPrim())
        if not visible:
            UsdGeom.Imageable(c.GetPrim()).MakeInvisible()
        return c.GetPrim()

    def cyl(self, path, r, h, pos=(0, 0, 0), look=None, collide=False, quat=None, visible=True, axis="Z"):
        c = UsdGeom.Cylinder.Define(self.st, path)
        c.CreateRadiusAttr(float(r))
        c.CreateHeightAttr(float(h))
        c.CreateAxisAttr(axis)
        self.place(c.GetPrim(), pos, quat)
        self.bind(c.GetPrim(), look)
        self.smooth(c.GetPrim())
        if collide:
            UsdPhysics.CollisionAPI.Apply(c.GetPrim())
        if not visible:
            UsdGeom.Imageable(c.GetPrim()).MakeInvisible()
        return c.GetPrim()

    @staticmethod
    def smooth(prim):
        """Ask the renderer for a finer tessellation of a round primitive (a cylinder is otherwise drawn with about
        ten sides, which reads as a polygon on a fastener's head)."""
        prim.CreateAttribute("refinementEnableOverride", Sdf.ValueTypeNames.Bool).Set(True)
        prim.CreateAttribute("refinementLevel", Sdf.ValueTypeNames.Int).Set(2)

    def capsule(self, path, r, a, b, look=None, collide=False):
        """A capsule from point a to point b (parent frame)."""
        a, b = np.array(a, dtype=float), np.array(b, dtype=float)
        d = b - a
        L = float(np.linalg.norm(d))
        z = d / L
        x = np.cross([0.0, 0.0, 1.0], z)
        if np.linalg.norm(x) < 1e-9:
            x = np.array([1.0, 0.0, 0.0])
        x /= np.linalg.norm(x)
        R = np.stack([x, np.cross(z, x), z], axis=1)
        c = UsdGeom.Capsule.Define(self.st, path)
        c.CreateRadiusAttr(float(r))
        c.CreateHeightAttr(L)
        c.CreateAxisAttr("Z")
        self.place(c.GetPrim(), (a + b) / 2, kin.R_quat(R))
        self.bind(c.GetPrim(), look)
        self.smooth(c.GetPrim())
        if collide:
            UsdPhysics.CollisionAPI.Apply(c.GetPrim())
        return c.GetPrim()

    def prism(self, path, outline, z0, z1, pos=(0, 0, 0), look=None):
        """A closed prism over a 2D outline (counter-clockwise), from z0 to z1."""
        n = len(outline)
        pts = [Gf.Vec3f(float(x), float(y), float(z0)) for x, y in outline] + [Gf.Vec3f(float(x), float(y), float(z1)) for x, y in outline]
        cx, cy = float(np.mean([p[0] for p in outline])), float(np.mean([p[1] for p in outline]))
        pts += [Gf.Vec3f(cx, cy, float(z0)), Gf.Vec3f(cx, cy, float(z1))]
        counts, idx = [], []
        for i in range(n):
            j = (i + 1) % n
            counts.append(4)
            idx += [i, j, n + j, n + i]
            counts.append(3)
            idx += [2 * n + 1, n + i, n + j]         # top fan
            counts.append(3)
            idx += [2 * n, j, i]                     # bottom fan
        m = UsdGeom.Mesh.Define(self.st, path)
        m.CreatePointsAttr(pts)
        m.CreateFaceVertexCountsAttr(counts)
        m.CreateFaceVertexIndicesAttr(idx)
        m.CreateSubdivisionSchemeAttr("none")
        m.CreateDoubleSidedAttr(True)
        self.place(m.GetPrim(), pos)
        self.bind(m.GetPrim(), look)
        return m.GetPrim()

    def grip(self, prim):
        """Bind the high-friction physics material to a collider. A bit on a fastener's head is such a pair: the
        friction stands in for the socket closing over the head (or the Torx bit sitting in its recess), which
        the colliders do not model, so a seated bit does not skate off the head under a side load."""
        path = "/World/Looks/physics_grip"
        if not self.st.GetPrimAtPath(path).IsValid():
            m = UsdShade.Material.Define(self.st, path)
            api = UsdPhysics.MaterialAPI.Apply(m.GetPrim())
            api.CreateStaticFrictionAttr(3.0)
            api.CreateDynamicFrictionAttr(3.0)
            api.CreateRestitutionAttr(0.0)
            px = PhysxSchema.PhysxMaterialAPI.Apply(m.GetPrim())
            px.CreateFrictionCombineModeAttr("max")
            px.CreateRestitutionCombineModeAttr("min")
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(UsdShade.Material(self.st.GetPrimAtPath(path)), UsdShade.Tokens.weakerThanDescendants, "physics")

    def body(self, prim, mass, kinematic=True, com=None, inertia=None):
        rb = UsdPhysics.RigidBodyAPI.Apply(prim)
        rb.CreateKinematicEnabledAttr(bool(kinematic))
        m = UsdPhysics.MassAPI.Apply(prim)
        m.CreateMassAttr(float(mass))
        if com is not None:
            m.CreateCenterOfMassAttr(Gf.Vec3f(*[float(v) for v in com]))
        if inertia is not None:
            m.CreateDiagonalInertiaAttr(Gf.Vec3f(*[float(v) for v in inertia]))
        return rb


def hex_outline(af):
    r = af / math.sqrt(3.0)          # across corners / 2
    return [(r * math.cos(math.radians(60 * k)), r * math.sin(math.radians(60 * k))) for k in range(6)]


def star_outline(r_out, lobes=6, r_in_factor=0.72, steps=8):
    """A six-lobe (Torx) outline: rounded lobes between the outer radius and r_in_factor of it."""
    pts = []
    n = lobes * steps
    for k in range(n):
        a = 2 * math.pi * k / n
        r = r_out * (r_in_factor + (1 - r_in_factor) * (0.5 + 0.5 * math.cos(lobes * a)) ** 0.8)
        pts.append((r * math.cos(a), r * math.sin(a)))
    return pts


def fastener_shapes(b, path, head_name, look):
    """A fastener's looks under `path` (origin on the axis in the seat plane, head toward +z). Returns the radius
    of its head."""
    head = S.HEADS[head_name]
    thread = S.THREADS[head["thread"]]
    h = head["height"]
    if head["kind"] == "hex":
        b.cyl(path + "/flange", head["flange_r"], head["flange_h"], (0, 0, head["flange_h"] / 2), look)
        b.prism(path + "/hex", hex_outline(head["af"]), head["flange_h"], h, look=look)
        r = head["flange_r"]
    else:
        b.cyl(path + "/pan", head["head_r"], h, (0, 0, h / 2), look)
        b.prism(path + "/recess", star_outline(head["recess_r"]), h - 0.0002, h + 0.00015, look="recess")
        r = head["head_r"]
    b.cyl(path + "/shank", thread["shank_r"], thread["shank_len"], (0, 0, -thread["shank_len"] / 2), look)
    return r


def tfm(T, p):
    return (T[:3, :3] @ np.asarray(p, dtype=float)) + T[:3, 3]


def build(stage, pack, robot_usd, render=True):
    """Author the cell with `pack` on its fixture. Returns the index: prim paths and constant poses."""
    b = Builder(stage)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(stage.GetPrimAtPath("/World"))
    scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    scene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
    scene.CreateGravityMagnitudeAttr(9.81)
    px = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
    px.CreateEnableGPUDynamicsAttr(False)          # physics on the CPU: the GPU only renders
    px.CreateBroadphaseTypeAttr("MBP")
    px.CreateSolverTypeAttr("TGS")
    px.CreateEnableCCDAttr(False)
    px.CreateEnableStabilizationAttr(False)
    index = {"parts": {}, "fasteners": {}, "bits": {}, "parked": {}}

    # ------------------------------------------------------------------ the cell
    tz = S.TABLE_Z
    b.box("/World/Cell/Floor", (8.0, 8.0, 0.2), (0, 0, S.FLOOR_Z - 0.1), "floor", collide=True)
    b.cyl("/World/Cell/Pedestal", 0.16, -S.FLOOR_Z - 0.004, (0, 0, (S.FLOOR_Z - 0.004) / 2), "pedestal", collide=True)
    for name, (x0, x1, y0, y1) in S.CELL["tables"].items():
        look = "fixture" if name == "fixture" else ("stand" if name in ("tool_stand",) else "table")
        b.box("/World/Cell/Table_" + name, (x1 - x0, y1 - y0, 0.04), ((x0 + x1) / 2, (y0 + y1) / 2, tz - 0.02), look, collide=True)
        for i, (lx, ly) in enumerate(((x0 + 0.04, y0 + 0.04), (x1 - 0.04, y0 + 0.04), (x0 + 0.04, y1 - 0.04), (x1 - 0.04, y1 - 0.04))):
            b.box("/World/Cell/Table_%s_leg%d" % (name, i), (0.05, 0.05, tz - 0.04 - S.FLOOR_Z), (lx, ly, (tz - 0.04 + S.FLOOR_Z) / 2), "pedestal")
    bin_look = {"electronics": "bin_green", "fasteners_right": "bin_grey", "fasteners_left": "bin_grey", "copper": "bin_red",
                "steel": "bin_blue"}
    for name, bn in S.CELL["bins"].items():
        (cx, cy), (sx, sy), h, t = bn["center"], bn["size"], bn["wall_h"], 0.008
        root = "/World/Cell/Bin_" + name
        b.box(root + "/floor", (sx + 2 * t, sy + 2 * t, 0.004), (cx, cy, tz + 0.002), bin_look[name], collide=True)
        b.box(root + "/wx0", (t, sy + 2 * t, h), (cx - sx / 2 - t / 2, cy, tz + h / 2), bin_look[name], collide=True)
        b.box(root + "/wx1", (t, sy + 2 * t, h), (cx + sx / 2 + t / 2, cy, tz + h / 2), bin_look[name], collide=True)
        b.box(root + "/wy0", (sx, t, h), (cx, cy - sy / 2 - t / 2, tz + h / 2), bin_look[name], collide=True)
        b.box(root + "/wy1", (sx, t, h), (cx, cy + sy / 2 + t / 2, tz + h / 2), bin_look[name], collide=True)
    rk = S.CELL["rack"]
    (bw, bl), dh, dt = rk["bay_size"], rk["divider_h"], rk["divider_t"]
    xs = [c[0] for c in rk["bay_centers"]]
    y_c = rk["bay_centers"][0][1]
    pitch = xs[1] - xs[0]
    for k in range(len(xs) + 1):
        b.box("/World/Cell/Rack/div%d" % k, (dt, bl + 2 * dt, dh), (xs[0] - pitch / 2 + k * pitch, y_c, tz + dh / 2), "rack", collide=True)
    for k, sy in enumerate((-1, 1)):
        b.box("/World/Cell/Rack/end%d" % k, (pitch * len(xs) + dt, dt, dh), ((xs[0] + xs[-1]) / 2, y_c + sy * (bl / 2 + dt / 2), tz + dh / 2), "rack", collide=True)
    # the tool stand: two holders and the bit station (looks only; the docking is a rule)
    for name, d in S.CELL["docks"].items():
        b.box("/World/Cell/ToolStand/post_" + name, (0.03, 0.03, d[2] - tz - 0.02), (d[0], d[1] - 0.085, (d[2] - 0.02 + tz) / 2), "pedestal")
        b.box("/World/Cell/ToolStand/fork_" + name, (0.12, 0.10, 0.008), (d[0], d[1] - 0.04, d[2] - 0.024), "pedestal")
    bs = S.CELL["bit_station"]
    b.box("/World/Cell/ToolStand/bit_block", (0.20, 0.07, bs[2] - 0.02 - tz), (bs[0], bs[1] - 0.07, (bs[2] - 0.02 + tz) / 2), "tool_black")

    # ------------------------------------------------------------------ lights
    if render:
        dome = UsdLux.DomeLight.Define(stage, "/World/Lights/Dome")
        dome.CreateIntensityAttr(150.0)
        dome.CreateColorAttr(Gf.Vec3f(0.95, 0.97, 1.0))
        for i, (lx, ly) in enumerate(((0.3, -0.9), (0.3, 0.9), (1.5, 0.0), (-0.9, 0.0))):
            r = UsdLux.RectLight.Define(stage, "/World/Lights/Panel%d" % i)
            r.CreateIntensityAttr(5500.0)
            r.CreateWidthAttr(1.2)
            r.CreateHeightAttr(0.5)
            b.place(r.GetPrim(), (lx, ly, 2.3))      # a rect light shines along its -z: straight down

    # ------------------------------------------------------------------ the robot
    rob = stage.DefinePrim("/World/Robot", "Xform")
    rob.GetReferences().AddReference(robot_usd)
    R = "/World/Robot"
    art = PhysxSchema.PhysxArticulationAPI.Apply(stage.OverridePrim(R + "/root_joint"))
    art.CreateEnabledSelfCollisionsAttr(False)
    art.CreateSleepThresholdAttr(0.0)              # never asleep: a sleeping arm reports stale forces and speeds
    art.CreateStabilizationThresholdAttr(0.0)
    # position drives (stiffness in N m per degree), torque limits as on the UR20's data sheet
    for jn, (k, d, mf) in {"shoulder_pan_joint": (12000.0, 60.0, 738.0), "shoulder_lift_joint": (12000.0, 60.0, 738.0),
                           "elbow_joint": (12000.0, 60.0, 433.0), "wrist_1_joint": (4000.0, 30.0, 107.0),
                           "wrist_2_joint": (4000.0, 30.0, 107.0), "wrist_3_joint": (4000.0, 30.0, 107.0)}.items():
        dr = UsdPhysics.DriveAPI.Apply(stage.OverridePrim("%s/joints/%s" % (R, jn)), "angular")
        dr.CreateStiffnessAttr(k)
        dr.CreateDampingAttr(d)
        dr.CreateMaxForceAttr(mf)
    nr, gr, cp = S.TOOLS["nutrunner"], S.TOOLS["gripper"], S.TOOLS["changer_plate"]
    ee = b.xform(R + "/ee_link")
    # mass and inertia are the mounted tool's; the cell sets them again at every tool change
    b.body(ee, nr["mass"], kinematic=False, com=nr["com"], inertia=(0.02, 0.02, 0.006))
    fj = UsdPhysics.FixedJoint.Define(stage, R + "/joints/ee_joint")
    fj.CreateBody0Rel().SetTargets([R + "/wrist_3_link"])
    fj.CreateBody1Rel().SetTargets([R + "/ee_link"])
    for a in (fj.CreateLocalPos0Attr, fj.CreateLocalPos1Attr):
        a(Gf.Vec3f(0, 0, 0))
    for a in (fj.CreateLocalRot0Attr, fj.CreateLocalRot1Attr):
        a(Gf.Quatf(1, 0, 0, 0))
    b.cyl(R + "/ee_link/changer", cp["r"], cp["h"], (0, 0, cp["h"] / 2), "changer", collide=True)
    # the wrist camera's housing (on the tool carrier, present with either tool)
    wc = S.CAMERAS["wrist"]
    b.box(R + "/ee_link/cam_housing", (0.030, 0.040, 0.035), (wc["pos"][0], wc["pos"][1], wc["pos"][2] - 0.022), "tool_black")
    # nutrunner: body on ee_link, the sprung spindle is a link of its own
    b.xform(R + "/ee_link/nutrunner")
    nb = nr["body"]
    b.cyl(R + "/ee_link/nutrunner/body", nb["r"], nb["z1"] - nb["z0"], (0, 0, (nb["z0"] + nb["z1"]) / 2), "tool_blue", collide=True)
    b.cyl(R + "/ee_link/nutrunner/collar", 0.018, 0.012, (0, 0, nb["z1"] - 0.004), "steel")
    sp = b.xform(R + "/spindle_link")
    b.body(sp, nr["spindle_mass"], kinematic=False, com=(0, 0, 0.06), inertia=(4e-4, 4e-4, 2e-5))
    total = nr["spindle_len"] + nr["bit_len"]
    b.cyl(R + "/spindle_link/rod", 0.0055, nr["spindle_len"], (0, 0, nr["spindle_len"] / 2), "steel")
    b.grip(b.cyl(R + "/spindle_link/collider", nr["collider_r"], total, (0, 0, total / 2), None, collide=True, visible=False))
    for name in S.BITS:
        r = S.BIT_LOOK[name]["r"]
        root = R + "/spindle_link/bit_" + name
        b.xform(root)
        if name.startswith("S"):
            b.cyl(root + "/socket", r, nr["bit_len"], (0, 0, nr["spindle_len"] + nr["bit_len"] / 2), "steel")
            b.prism(root + "/mouth", hex_outline(float(name[1:]) / 1000.0), total - 0.0002, total + 0.0002, look="recess")
        elif name.startswith("T"):
            b.cyl(root + "/shank", 0.006, nr["bit_len"] - 0.010, (0, 0, nr["spindle_len"] + (nr["bit_len"] - 0.010) / 2), "tool_black")
            b.prism(root + "/tip", star_outline(S.HEADS[name]["recess_r"] * 0.98), total - 0.010, total, look="steel")
        else:
            b.cyl(root + "/socket", r, nr["bit_len"], (0, 0, nr["spindle_len"] + nr["bit_len"] / 2), "changer")
        index["bits"][name] = root
    # a freed fastener stays on the bit until it is ejected: one hidden copy per head type rides on the spindle
    index["held"] = {}
    flip = kin.R_quat(np.diag([1.0, -1.0, -1.0]))
    for name, head in S.HEADS.items():
        inset = head["height"] if head["kind"] == "hex" else 0.002
        root = R + "/spindle_link/held_" + name
        b.xform(root, (0, 0, total - inset + head["height"]), flip)
        fastener_shapes(b, root, name, "zinc")
        index["held"][name] = root
    pj = UsdPhysics.PrismaticJoint.Define(stage, R + "/joints/spindle_joint")
    pj.CreateBody0Rel().SetTargets([R + "/ee_link"])
    pj.CreateBody1Rel().SetTargets([R + "/spindle_link"])
    pj.CreateAxisAttr("Z")
    pj.CreateLowerLimitAttr(-float(nr["stroke"]))
    pj.CreateUpperLimitAttr(0.0)
    pj.CreateLocalPos0Attr(Gf.Vec3f(0, 0, float(nr["spindle_origin_z"])))
    pj.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
    pj.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
    pj.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
    dr = UsdPhysics.DriveAPI.Apply(pj.GetPrim(), "linear")
    dr.CreateTypeAttr("force")
    dr.CreateStiffnessAttr(float(nr["spring_N_per_m"]))
    dr.CreateDampingAttr(float(nr["spring_damping"]))
    dr.CreateTargetPositionAttr(0.0)
    dr.CreateMaxForceAttr(1000.0)
    # gripper: body and two fingers (the fingers are looks only: the grasp is a rule, see rules.py)
    b.xform(R + "/ee_link/gripper")
    gs, gz0 = gr["body"]["size"], gr["body"]["z0"]
    b.box(R + "/ee_link/gripper/body", gs, (0, 0, gz0 + gs[2] / 2), "tool_black", collide=True)
    fz0 = gz0 + gs[2]
    for side, sgn in (("l", -1), ("r", 1)):
        f = b.xform(R + "/ee_link/gripper/finger_" + side, (0, sgn * gr["open_max"] / 2, 0))
        b.box(R + "/ee_link/gripper/finger_%s/bar" % side, (gr["finger_w"], gr["finger_t"], gr["finger_len"]),
              (0, sgn * gr["finger_t"] / 2, fz0 + gr["finger_len"] / 2), "alu")
        index["finger_" + side] = str(f.GetPath())
    index.update(robot=R, base=R + "/base_link", ee=R + "/ee_link", spindle=R + "/spindle_link",
                 ee_colliders={"nutrunner": [R + "/ee_link/nutrunner/body", R + "/spindle_link/collider"],
                               "gripper": [R + "/ee_link/gripper/body"]},
                 tool_roots={"nutrunner": [R + "/ee_link/nutrunner", R + "/spindle_link"], "gripper": [R + "/ee_link/gripper"]})
    # parked copies on the stand (looks only)
    for name, d in S.CELL["docks"].items():
        root = "/World/Cell/ToolStand/parked_" + name
        b.xform(root, d, kin.R_quat(np.diag([1.0, -1.0, -1.0])))       # tool axis down
        b.cyl(root + "/changer", cp["r"] * 0.9, 0.012, (0, 0, cp["h"] - 0.006), "steel")
        if name == "nutrunner":
            b.cyl(root + "/body", nb["r"], nb["z1"] - nb["z0"], (0, 0, (nb["z0"] + nb["z1"]) / 2), "tool_blue")
            b.cyl(root + "/rod", 0.0055, nr["spindle_len"], (0, 0, nb["z1"] + nr["spindle_len"] / 2), "steel")
        else:
            b.box(root + "/body", gs, (0, 0, gz0 + gs[2] / 2), "tool_black")
            for side, sgn in (("l", -1), ("r", 1)):
                b.box(root + "/finger_" + side, (gr["finger_w"], gr["finger_t"], gr["finger_len"]),
                      (0, sgn * (gr["open_max"] / 2 + gr["finger_t"] / 2), fz0 + gr["finger_len"] / 2), "alu")
        index["parked"][name] = root
    for k, name in enumerate(S.BITS):
        root = "/World/Cell/ToolStand/rack_bit_" + name
        b.xform(root, (bs[0] - 0.08 + 0.04 * k, bs[1] - 0.07, bs[2] - 0.02))
        b.cyl(root + "/bit", max(S.BIT_LOOK[name]["r"], 0.005), 0.03, (0, 0, 0.015), "changer" if name == "EX" else "steel")
        index["parked"]["bit_" + name] = root

    # ------------------------------------------------------------------ cameras
    if render:
        for name, c in S.CAMERAS.items():
            path = "/World/Cams/overhead" if c["parent"] == "world" else R + "/ee_link/wrist_cam"
            cam = UsdGeom.Camera.Define(stage, path)
            hap = 20.955
            cam.CreateHorizontalApertureAttr(hap)
            cam.CreateVerticalApertureAttr(hap * c["height"] / c["width"])
            cam.CreateFocalLengthAttr(0.5 * hap / math.tan(math.radians(c["hfov_deg"]) / 2))
            cam.CreateClippingRangeAttr(Gf.Vec2f(*c["clip"]))
            cam.CreateFStopAttr(0.0)
            Rc = S.camera_R(name)                               # OpenCV axes; a USD camera looks along -z with +y up
            Ru = np.stack([Rc[:, 0], -Rc[:, 1], -Rc[:, 2]], axis=1)
            b.place(cam.GetPrim(), c["pos"], kin.R_quat(Ru))
            index["cam_" + name] = path

    # ------------------------------------------------------------------ the pack
    T = S.pack_pose(pack)
    qP = kin.R_quat(T[:3, :3])
    tr = S.TRAY
    (sx, sy), th, tw, tb = tr["size"], tr["height"], tr["wall"], tr["bottom"]
    b.xform("/World/Pack", T[:3, 3], qP)
    P = "/World/Pack/Tray"
    b.box(P + "/bottom", (sx, sy, tb), (0, 0, tb / 2), "tray", collide=True)
    for k, sgn in enumerate((-1, 1)):
        b.box(P + "/wall_x%d" % k, (tw, sy, th - tb), (sgn * (sx / 2 - tw / 2), 0, (th + tb) / 2), "tray", collide=True)
        b.box(P + "/wall_y%d" % k, (sx - 2 * tw, tw, th - tb), (0, sgn * (sy / 2 - tw / 2), (th + tb) / 2), "tray", collide=True)
    # the service bay's fixed equipment
    b.box(P + "/msd_base", (0.07, 0.05, 0.058), (S.SERVICE_X, pack["msd_y"], tb + 0.029), "hv_orange", collide=True)
    b.box(P + "/bms", (0.12, 0.18, 0.05), (S.SERVICE_X, pack["bms_y"], tb + 0.025), "bms", collide=True)
    py = S.MODULE["post"][1]
    ys = pack["module_ys"]
    for k, y in enumerate((ys[0] - py, ys[-1] + py)):
        b.box(P + "/junction%d" % k, (0.03, 0.03, S.Z["post_top"] - tb), (S.JUNCTION_X, y, (S.Z["post_top"] + tb) / 2), "insulator", collide=True)
    # the ledge a bracket sits on (the module's foot lies on it; the foot itself is only drawn)
    for i, y in enumerate(ys):
        for k, bx in enumerate(S.BRACKET_X):
            b.box(P + "/ledge%d_%d" % (i, k), (0.030, 0.060, S.Z["bracket_center"] - 0.011 - tb), (bx, y, (S.Z["bracket_center"] - 0.011 + tb) / 2),
                  None, collide=True, visible=False)
    # dark holes show where a fastener is gone
    fast_by_id = {f["id"]: f for f in pack["fasteners"]}

    def hole(parent, name, xy_local, z_local, thread):
        b.cyl("%s/hole_%s" % (parent, name), S.THREADS[thread]["shank_r"] * 1.15, 0.0004, (xy_local[0], xy_local[1], z_local + 0.0001), "hole")

    def rigid(pid, part, shapes):
        """One part of the pack as a kinematic rigid body at its place; `shapes` adds its geometry (body frame)."""
        path = "/World/Parts/" + pid
        prim = b.xform(path, tfm(T, part["pos"]), qP)
        b.body(prim, part["mass"], kinematic=True)
        shapes(path)
        index["parts"][pid] = path
        return path

    for part in pack["parts"]:
        pid, kind = part["id"], part["kind"]
        if kind == "lid":
            def shapes(path, part=part):
                b.box(path + "/plate", (sx, sy, S.LID["t"]), (0, 0, 0), "lid", collide=True)
                for i, (axis, c) in enumerate(pack["ribs"]):
                    size = (0.50, 0.03, 0.004) if axis == "x" else (0.03, 0.86, 0.004)
                    pos = (0, c, S.LID["t"] / 2 + 0.002) if axis == "x" else (c, 0, S.LID["t"] / 2 + 0.002)
                    b.box(path + "/rib%d" % i, size, pos, "lid_rib")
                hz = S.LID["handle_z"]
                for i, hx in enumerate((-0.05, 0.05)):
                    b.box(path + "/handle_post%d" % i, (0.012, 0.03, hz - 0.006 - S.LID["t"] / 2), (hx, 0, (hz - 0.006 + S.LID["t"] / 2) / 2), "strap")
                b.box(path + "/handle", (0.12, 0.03, 0.012), (0, 0, hz), "handle", collide=True)
                for fid in part["lock"]["ids"]:
                    f = fast_by_id[fid]
                    hole(path, fid, f["pos"], S.LID["t"] / 2, f["thread"])
        elif kind == "msd":
            def shapes(path, part=part):
                b.box(path + "/plug", (0.06, 0.04, 0.045), (0, 0, 0), "hv_orange", collide=True)
                for i, hx in enumerate((-0.021, 0.021)):
                    b.box(path + "/post%d" % i, (0.008, 0.02, 0.014), (hx, 0, 0.0225 + 0.007), "strap")
                b.box(path + "/handle", (0.05, 0.02, 0.012), (0, 0, 0.0425), "strap", collide=True)
        elif kind == "connector":
            def shapes(path, part=part):
                b.box(path + "/plug", (0.022, 0.034, 0.030), (0, 0, 0), "conn_grey" if part["id"] == "J1" else "conn_white", collide=True)
                b.box(path + "/latch", (0.024, 0.010, 0.006), (0, 0, 0.012), "strap")
        elif kind == "busbar":
            def shapes(path, part=part):
                L = part["length"]
                size = (L, 0.022, 0.004) if part["along"] == "x" else (0.022, L, 0.004)
                sleeve = (L - 0.060, 0.026, 0.008) if part["along"] == "x" else (0.026, L - 0.060, 0.008)
                b.box(path + "/bar", size, (0, 0, 0), "copper", collide=True)
                b.box(path + "/sleeve", sleeve, (0, 0, 0), "hv_orange")
                for fid in part["lock"]["ids"]:
                    f = fast_by_id[fid]
                    hole(path, fid, np.array(f["pos"][:2]) - np.array(part["pos"][:2]), 0.002, f["thread"])
        elif kind == "bracket":
            def shapes(path, part=part):
                b.box(path + "/block", (0.030, 0.060, 0.022), (0, 0, 0), "bracket", collide=True)
                f = fast_by_id[part["lock"]["ids"][0]]
                hole(path, f["id"], np.array(f["pos"][:2]) - np.array(part["pos"][:2]), 0.011, f["thread"])
        elif kind == "module":
            def shapes(path, part=part):
                ml, mw, mh = S.MODULE["size"]
                b.box(path + "/cells", (ml, mw, mh), (0, 0, 0), "module", collide=True)
                b.box(path + "/top", (ml - 0.02, mw - 0.02, 0.002), (0, 0, mh / 2 + 0.001), "module_top")
                hz = S.MODULE["handle_z"]
                for i, hx in enumerate((-0.044, 0.044)):
                    b.box(path + "/strap_post%d" % i, (0.012, 0.03, hz - 0.006 - mh / 2), (hx, 0, (hz - 0.006 + mh / 2) / 2), "strap")
                b.box(path + "/strap", (0.10, 0.03, 0.012), (0, 0, hz), "handle", collide=True)
                for i, sgn in enumerate((-1, 1)):
                    b.cyl(path + "/post%d" % i, S.MODULE["post_r"], S.MODULE["post_h"],
                          (S.MODULE["post"][0], sgn * S.MODULE["post"][1], mh / 2 + S.MODULE["post_h"] / 2), "copper", collide=True)
                    b.box(path + "/foot%d" % i, (0.032, 0.10, 0.006), (sgn * (ml / 2 + 0.016), 0, -mh / 2 + 0.003), "module")
        elif kind == "harness":
            def shapes(path, part=part):
                c = np.array(part["pos"])
                pts = [np.array(p) - c for p in part["path"]]
                for i in range(len(pts) - 1):
                    b.capsule(path + "/seg%d" % i, S.HARNESS_R, pts[i], pts[i + 1], "harness", collide=True)
                for i, p in enumerate((pts[0], pts[-1])):
                    b.box(path + "/end%d" % i, (0.016, 0.016, 0.012), p, "conn_grey")
        else:
            raise ValueError(kind)
        rigid(pid, part, shapes)

    # fasteners: a head on its seat (a missing one leaves the hole); bodies are kinematic until ejected
    for f in pack["fasteners"]:
        if f["missing"]:
            continue
        fid, head, thread = f["id"], S.HEADS[f["head"]], S.THREADS[f["thread"]]
        path = "/World/Fasteners/" + fid
        prim = b.xform(path, tfm(T, f["pos"]), qP)
        b.body(prim, 0.012 if f["thread"] == "M6" else 0.028, kinematic=True)
        h = head["height"]
        r_col = fastener_shapes(b, path, f["head"], "rust" if f["corroded"] else "zinc")
        if f["corroded"]:
            b.cyl(path + "/stain", r_col + 0.0035, 0.0003, (0, 0, 0.00015), "rust_stain")
        b.grip(b.cyl(path + "/col_head", r_col, h, (0, 0, h / 2), None, collide=True, visible=False))
        b.cyl(path + "/col_shank", thread["shank_r"], thread["shank_len"], (0, 0, -thread["shank_len"] / 2), None, collide=True, visible=False)
        index["fasteners"][fid] = path
    index["pack_T"] = T.tolist()
    return index

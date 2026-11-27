"""Build the greenhouse row section, the cart and the robot as a USD stage (pxr only; nothing here needs a running
simulator).

`build(stage, row, robot_usd)` authors everything under `/World` and returns an index of the prims the cell drives:
the robot's articulation with two links added (a `rail_link` that the base rides along on a prismatic joint: the
cart's travel; the tool carrier `ee_link`, fixed to the flange, with the cutter-gripper's colliders), the cart's own
body with the deck and the two crates, the plants (stems, trusses, leaves: rigid bodies on compliant joints), and the
two cameras. All geometry comes from `farm.py`.

Plants. A stem is a chain of capsule bodies on D6 joints with angular spring drives, anchored at the gutter and
held at its top by the support string (a D6 joint to the world with linear spring drives). A truss is a peduncle
body (the cut section) on the stem and three rachis bodies below it, each carrying its fruits as sphere colliders,
all on D6 joints with spring drives. A leaf is one body (petiole and blade) on a D6 joint with a spring drive. Every
member's body frame has its origin at the member's proximal end and z along it (`rules.frame_z`), and its joint to
what it grows on is named after the member: cutting a member removes that joint.
"""

import math

import numpy as np
from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdLux, UsdPhysics, UsdShade

from . import farm as F
from . import kin
from .rules import frame_z

LOOKS = {
    # name: (diffuse rgb, metallic, roughness)
    "floor": ((0.78, 0.78, 0.76), 0.0, 0.80),
    "screen": ((0.62, 0.64, 0.66), 0.0, 0.90),
    "rail": ((0.80, 0.80, 0.82), 0.6, 0.45),
    "gutter": ((0.86, 0.86, 0.84), 0.2, 0.55),
    "slab": ((0.30, 0.22, 0.16), 0.0, 0.95),
    "deck": ((0.20, 0.24, 0.30), 0.3, 0.60),
    "crate": ((0.10, 0.22, 0.52), 0.0, 0.55),
    "pedestal": ((0.15, 0.16, 0.18), 0.3, 0.50),
    "mast": ((0.55, 0.56, 0.58), 0.7, 0.45),
    "tool_body": ((0.82, 0.42, 0.06), 0.2, 0.50),
    "tool_dark": ((0.04, 0.04, 0.04), 0.2, 0.50),
    "steel": ((0.72, 0.72, 0.74), 1.0, 0.30),
    "pad": ((0.05, 0.25, 0.60), 0.0, 0.80),
    "plate_white": ((0.85, 0.85, 0.83), 0.0, 0.85),
    "plate_black": ((0.02, 0.02, 0.02), 0.0, 0.88),
    "stem": (tuple(F.LOOK["stem"]), 0.0, 0.70),
    "peduncle": (tuple(F.LOOK["peduncle"]), 0.0, 0.65),
    "calyx": (tuple(F.LOOK["calyx"]), 0.0, 0.70),
    "leaf": (tuple(F.LOOK["leaf"]), 0.0, 0.60),
    "string": ((0.80, 0.80, 0.70), 0.0, 0.80),
}


class Builder(object):
    def __init__(self, stage):
        self.st = stage
        self.mats = {}

    # ------------------------------------------------------------------ materials and primitives
    def mat(self, name, color=None, metallic=0.0, rough=0.5):
        if name not in self.mats:
            if color is None:
                color, metallic, rough = LOOKS[name]
            path = "/World/Looks/" + name
            m = UsdShade.Material.Define(self.st, path)
            sh = UsdShade.Shader.Define(self.st, path + "/Shader")
            sh.CreateIdAttr("UsdPreviewSurface")
            sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*[float(v) for v in color]))
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

    def cyl(self, path, r, h, pos=(0, 0, 0), look=None, collide=False, quat=None, axis="Z"):
        c = UsdGeom.Cylinder.Define(self.st, path)
        c.CreateRadiusAttr(float(r))
        c.CreateHeightAttr(float(h))
        c.CreateAxisAttr(axis)
        self.place(c.GetPrim(), pos, quat)
        self.bind(c.GetPrim(), look)
        self.smooth(c.GetPrim())
        if collide:
            UsdPhysics.CollisionAPI.Apply(c.GetPrim())
        return c.GetPrim()

    def sphere(self, path, r, pos=(0, 0, 0), look=None, collide=False):
        s = UsdGeom.Sphere.Define(self.st, path)
        s.CreateRadiusAttr(float(r))
        self.place(s.GetPrim(), pos)
        self.bind(s.GetPrim(), look)
        self.smooth(s.GetPrim())
        if collide:
            UsdPhysics.CollisionAPI.Apply(s.GetPrim())
        return s.GetPrim()

    @staticmethod
    def smooth(prim):
        """Ask the renderer for a finer tessellation of a round primitive."""
        prim.CreateAttribute("refinementEnableOverride", Sdf.ValueTypeNames.Bool).Set(True)
        prim.CreateAttribute("refinementLevel", Sdf.ValueTypeNames.Int).Set(2)

    def capsule(self, path, r, a, b, look=None, collide=False):
        """A capsule from point a to point b (parent frame)."""
        a, b = np.array(a, dtype=float), np.array(b, dtype=float)
        L = float(np.linalg.norm(b - a))
        c = UsdGeom.Capsule.Define(self.st, path)
        c.CreateRadiusAttr(float(r))
        c.CreateHeightAttr(L)
        c.CreateAxisAttr("Z")
        self.place(c.GetPrim(), (a + b) / 2, kin.R_quat(frame_z(b - a)))
        self.bind(c.GetPrim(), look)
        self.smooth(c.GetPrim())
        if collide:
            UsdPhysics.CollisionAPI.Apply(c.GetPrim())
        return c.GetPrim()

    def mesh(self, path, pts, counts, idx, pos=(0, 0, 0), quat=None, look=None):
        m = UsdGeom.Mesh.Define(self.st, path)
        m.CreatePointsAttr([Gf.Vec3f(float(p[0]), float(p[1]), float(p[2])) for p in pts])
        m.CreateFaceVertexCountsAttr(counts)
        m.CreateFaceVertexIndicesAttr(idx)
        m.CreateSubdivisionSchemeAttr("none")
        m.CreateDoubleSidedAttr(True)
        self.place(m.GetPrim(), pos, quat)
        self.bind(m.GetPrim(), look)
        return m.GetPrim()

    def physics_material(self, name, friction, restitution=0.0):
        path = "/World/Looks/physics_" + name
        if not self.st.GetPrimAtPath(path).IsValid():
            m = UsdShade.Material.Define(self.st, path)
            api = UsdPhysics.MaterialAPI.Apply(m.GetPrim())
            api.CreateStaticFrictionAttr(float(friction))
            api.CreateDynamicFrictionAttr(float(friction))
            api.CreateRestitutionAttr(float(restitution))
            PhysxSchema.PhysxMaterialAPI.Apply(m.GetPrim()).CreateRestitutionCombineModeAttr("min")
        return UsdShade.Material(self.st.GetPrimAtPath(path))

    def friction(self, prim, name, friction):
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(self.physics_material(name, friction), UsdShade.Tokens.weakerThanDescendants, "physics")

    def body(self, prim, mass, kinematic=False, com=None, inertia=None, damping=(0.05, 0.6), iters=(16, 2), max_depenetration=0.5,
             report=False):
        rb = UsdPhysics.RigidBodyAPI.Apply(prim)
        rb.CreateKinematicEnabledAttr(bool(kinematic))
        m = UsdPhysics.MassAPI.Apply(prim)
        m.CreateMassAttr(float(mass))
        if com is not None:
            m.CreateCenterOfMassAttr(Gf.Vec3f(*[float(v) for v in com]))
        if inertia is not None:
            m.CreateDiagonalInertiaAttr(Gf.Vec3f(*[float(v) for v in inertia]))
        px = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        px.CreateLinearDampingAttr(float(damping[0]))
        px.CreateAngularDampingAttr(float(damping[1]))
        px.CreateSolverPositionIterationCountAttr(int(iters[0]))
        px.CreateSolverVelocityIterationCountAttr(int(iters[1]))
        px.CreateMaxDepenetrationVelocityAttr(float(max_depenetration))
        px.CreateSleepThresholdAttr(0.0)
        px.CreateStabilizationThresholdAttr(0.0)
        if report:
            PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.0)
        return rb

    def d6(self, path, body0, T0, body1, T1_world, T_joint, k_rot=None, d_rot=0.0, k_lin=None, d_lin=0.0, local1=None):
        """A D6 joint at the world pose `T_joint` between body0 (world pose T0; None: the world) and body1 (world pose
        `T1_world`). Rotations: spring drives of `k_rot` (N m/rad) and `d_rot` (N m s/rad) about every axis (one
        number, or one per axis of the joint's frame: x, y, z), or free when k_rot is None. Translations: locked, or spring drives of `k_lin` (N/m per axis: a 3-list) and `d_lin`."""
        j = UsdPhysics.Joint.Define(self.st, path)
        if body0 is not None:
            j.CreateBody0Rel().SetTargets([body0])
            L0 = kin.inv(T0) @ T_joint
        else:
            L0 = T_joint
        L1 = kin.inv(T1_world) @ T_joint
        j.CreateBody1Rel().SetTargets([body1])
        for L, pos, rot in ((L0, j.CreateLocalPos0Attr, j.CreateLocalRot0Attr), (L1, j.CreateLocalPos1Attr, j.CreateLocalRot1Attr)):
            pos(Gf.Vec3f(*[float(v) for v in L[:3, 3]]))
            q = kin.R_quat(L[:3, :3])
            rot(Gf.Quatf(float(q[0]), float(q[1]), float(q[2]), float(q[3])))
        for i, ax in enumerate(("transX", "transY", "transZ")):
            if k_lin is None:
                lim = UsdPhysics.LimitAPI.Apply(j.GetPrim(), ax)
                lim.CreateLowAttr(1.0)
                lim.CreateHighAttr(-1.0)                 # low > high: locked
            else:
                d = UsdPhysics.DriveAPI.Apply(j.GetPrim(), ax)
                d.CreateTypeAttr("force")
                d.CreateStiffnessAttr(float(k_lin[i]))
                d.CreateDampingAttr(float(d_lin))
                d.CreateTargetPositionAttr(0.0)
        if k_rot is not None:
            ks = k_rot if isinstance(k_rot, (list, tuple)) else [k_rot] * 3
            ds = d_rot if isinstance(d_rot, (list, tuple)) else [d_rot] * 3
            for i, ax in enumerate(("rotX", "rotY", "rotZ")):
                d = UsdPhysics.DriveAPI.Apply(j.GetPrim(), ax)
                d.CreateTypeAttr("force")
                d.CreateStiffnessAttr(float(ks[i]) * math.pi / 180.0)          # USD: per degree
                d.CreateDampingAttr(float(ds[i]) * math.pi / 180.0)
                d.CreateTargetPositionAttr(0.0)
        return j


def member_T(a, b):
    """A member's body frame in W: origin at its proximal end a, z toward b."""
    a, b = np.array(a, dtype=float), np.array(b, dtype=float)
    return kin.pose(frame_z(b - a), a)


def leaf_mesh(length, width, leaflets=7):
    """A pinnate blade in its own plane (x along the blade from its base, y across, z the normal): a midrib strip and
    oval leaflets on both sides and at the tip. Returns (points, face counts, indices)."""
    pts, counts, idx = [], [], []

    def poly(ring):
        base = len(pts)
        pts.extend(ring)
        counts.append(len(ring))
        idx.extend(range(base, base + len(ring)))

    poly([(0.0, -0.004, 0.0), (length, -0.003, 0.0), (length, 0.003, 0.0), (0.0, 0.004, 0.0)])
    n = (leaflets - 1) // 2
    for k in range(n):
        x = length * (0.18 + 0.64 * k / max(1, n - 1))
        a, b = length * 0.13, width * (0.5 - 0.08 * abs(k - (n - 1) / 2.0))
        for sgn in (-1.0, 1.0):
            poly([(x + a * math.cos(t) * 0.9, sgn * (0.5 * b + 0.5 * b * math.sin(t) + 0.004), 0.0005 * sgn)
                  for t in [2 * math.pi * i / 14 for i in range(14)]][::int(sgn)])
    poly([(length * 0.88 + length * 0.14 * math.cos(t), width * 0.22 * math.sin(t), 0.0) for t in [2 * math.pi * i / 14 for i in range(14)]])
    return pts, counts, idx


def calyx_mesh(r):
    """A five-pointed star lying on a sphere of radius r around its +z pole (a fruit's calyx)."""
    pts, counts, idx = [(0.0, 0.0, r * 1.01)], [], []
    for i in range(10):
        a = 2 * math.pi * i / 10
        rho = (0.62 if i % 2 == 0 else 0.22) * r
        pts.append((rho * math.cos(a), rho * math.sin(a), math.sqrt(max(1e-9, r * r - rho * rho)) * 1.01))
    for i in range(10):
        counts.append(3)
        idx += [0, 1 + i, 1 + (i + 1) % 10]
    return pts, counts, idx


def build(stage, row, robot_usd, render=True):
    """Author the row section, the cart and the robot. Returns the index: prim paths and constants."""
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
    index = {"bodies": {}, "joints": {}, "rachis": [], "stems": [], "leaves": [], "peduncles": []}
    y0, y1 = F.CART["travel"][0] - 1.6, F.CART["travel"][1] + 1.6
    ym, yl = (y0 + y1) / 2, y1 - y0

    # ------------------------------------------------------------------ the greenhouse
    b.box("/World/House/Floor", (6.0, yl + 4.0, 0.2), (0.6, ym, F.FLOOR_Z - 0.1), "floor", collide=True)
    b.box("/World/House/Screen", (0.02, yl + 4.0, 3.2), (2.05, ym, F.FLOOR_Z + 1.6), "screen")
    for i, x in enumerate((-0.28, 0.28)):
        b.cyl("/World/House/Rail%d" % i, 0.0255, yl + 3.0, (x, ym, F.FLOOR_Z + 0.06), "rail", axis="Y")
    gz = F.ROW["gutter_z"]
    b.box("/World/House/Gutter", (0.24, yl + 2.0, 0.06), (F.ROW["x"], ym, gz - 0.10), "gutter", collide=True)
    b.box("/World/House/Slab", (0.18, yl + 1.0, 0.07), (F.ROW["x"], ym, gz - 0.035), "slab")       # the stems stand in it
    for k in range(int(yl / 1.5) + 2):
        b.box("/World/House/GutterPost%d" % k, (0.04, 0.04, gz - 0.13 - F.FLOOR_Z), (F.ROW["x"], y0 + 1.5 * k, (gz - 0.13 + F.FLOOR_Z) / 2), "mast")
    # row markers on the gutter's aisle side (looks only)
    mk = F.MARKERS
    cell = mk["side"] / 6.0
    for k in range(mk["count"]):
        T = F.marker_T(k)
        root = "/World/House/Marker%d" % k
        b.xform(root, T[:3, 3], kin.R_quat(T[:3, :3]))
        full = mk["side"] + 2 * mk["margin"]
        b.box(root + "/plate", (full, full, 0.002), (0, 0, -0.001), "plate_white")
        b.box(root + "/black", (mk["side"], mk["side"], 0.0004), (0, 0, 0.0002), "plate_black")
        for r_, bits in enumerate(F.MARKER_BITS[k]):
            for c_, bit in enumerate(bits):
                if bit == "1":
                    # the image's row r_ from the top is the plate's +y from the top; its column from the left the plate's +x
                    b.box("%s/w_%d_%d" % (root, r_, c_), (cell, cell, 0.0003),
                          ((c_ - 1.5) * cell, (1.5 - r_) * cell, 0.00055), "plate_white")

    # ------------------------------------------------------------------ lights (this row's: farm.ROW light_*)
    light = row.get("light") or {}
    gain, rgb, lamps = light.get("gain", 1.0), light.get("rgb", [1.0, 1.0, 1.0]), light.get("lamps", [1.0] * 3)
    if render:
        dome = UsdLux.DomeLight.Define(stage, "/World/Lights/Dome")
        dome.CreateIntensityAttr(260.0 * gain)
        dome.CreateColorAttr(Gf.Vec3f(float(rgb[0]), float(rgb[1]), float(rgb[2])))
        for i, ly in enumerate((ym - 1.6, ym, ym + 1.6)):
            r = UsdLux.RectLight.Define(stage, "/World/Lights/Panel%d" % i)
            r.CreateIntensityAttr(4200.0 * gain * lamps[i])
            r.CreateColorAttr(Gf.Vec3f(float(rgb[0]), float(rgb[1]), float(rgb[2])))
            r.CreateWidthAttr(1.4)
            r.CreateHeightAttr(1.2)
            b.place(r.GetPrim(), (0.45, ly, 2.7))        # a rect light shines along its -z: straight down
        for i, ly in enumerate((ym - 1.2, ym + 1.2)):
            # the aisle's side light: what lets a camera see under the leaves
            r = UsdLux.RectLight.Define(stage, "/World/Lights/Side%d" % i)
            r.CreateIntensityAttr(1500.0 * gain)
            r.CreateColorAttr(Gf.Vec3f(float(rgb[0]), float(rgb[1]), float(rgb[2])))
            r.CreateWidthAttr(1.6)
            r.CreateHeightAttr(1.0)
            b.place(r.GetPrim(), (-1.3, ly, 0.9), kin.R_quat(np.array([[0.0, 0.0, -1.0], [0.0, 1.0, 0.0], [1.0, 0.0, 0.0]])))

    # ------------------------------------------------------------------ the robot on its rail
    rob = stage.DefinePrim("/World/Robot", "Xform")
    rob.GetReferences().AddReference(robot_usd)
    R = "/World/Robot"
    art = PhysxSchema.PhysxArticulationAPI.Apply(stage.OverridePrim(R + "/root_joint"))
    art.CreateEnabledSelfCollisionsAttr(False)
    art.CreateSleepThresholdAttr(0.0)              # never asleep: a sleeping arm reports stale forces and speeds
    art.CreateStabilizationThresholdAttr(0.0)
    art.CreateSolverPositionIterationCountAttr(32)
    # position drives (stiffness in N m per degree), torque limits as on the UR20's data sheet
    for jn, (k, d, mf) in {"shoulder_pan_joint": (12000.0, 60.0, 738.0), "shoulder_lift_joint": (12000.0, 60.0, 738.0),
                           "elbow_joint": (12000.0, 60.0, 433.0), "wrist_1_joint": (4000.0, 30.0, 107.0),
                           "wrist_2_joint": (4000.0, 30.0, 107.0), "wrist_3_joint": (4000.0, 30.0, 107.0)}.items():
        dr = UsdPhysics.DriveAPI.Apply(stage.OverridePrim("%s/joints/%s" % (R, jn)), "angular")
        dr.CreateStiffnessAttr(k)
        dr.CreateDampingAttr(d)
        dr.CreateMaxForceAttr(mf)
    # the rail: the articulation's fixed root; the robot's base rides along it (the cart's travel along W y)
    rail = b.xform(R + "/rail_link")
    b.body(rail, 50.0, iters=(32, 4))
    UsdPhysics.Joint(stage.OverridePrim(R + "/root_joint")).GetBody1Rel().SetTargets([R + "/rail_link"])
    pj = UsdPhysics.PrismaticJoint.Define(stage, R + "/joints/cart_joint")
    pj.CreateBody0Rel().SetTargets([R + "/rail_link"])
    pj.CreateBody1Rel().SetTargets([R + "/base_link"])
    pj.CreateAxisAttr("Y")
    pj.CreateLowerLimitAttr(float(F.CART["travel"][0] - 0.05))
    pj.CreateUpperLimitAttr(float(F.CART["travel"][1] + 0.05))
    for a in (pj.CreateLocalPos0Attr, pj.CreateLocalPos1Attr):
        a(Gf.Vec3f(0, 0, 0))
    for a in (pj.CreateLocalRot0Attr, pj.CreateLocalRot1Attr):
        a(Gf.Quatf(1, 0, 0, 0))
    dr = UsdPhysics.DriveAPI.Apply(pj.GetPrim(), "linear")
    dr.CreateTypeAttr("force")
    dr.CreateStiffnessAttr(6.0e6)
    dr.CreateDampingAttr(8.0e4)
    dr.CreateMaxForceAttr(20000.0)
    dr.CreateTargetPositionAttr(0.0)
    UsdPhysics.MassAPI(stage.OverridePrim(R + "/base_link")).CreateMassAttr(260.0)       # the cart's chassis
    # the cart's furniture that only shows (children of the base link: they ride with it exactly)
    C = R + "/base_link/cart"
    b.xform(C)
    b.cyl(C + "/mast", 0.02, F.CAMERAS["overview"]["pos"][2] - F.DECK_Z, (F.CAMERAS["overview"]["pos"][0] - 0.03, F.CAMERAS["overview"]["pos"][1] - 0.03,
          (F.CAMERAS["overview"]["pos"][2] + F.DECK_Z) / 2), "mast")
    for name, bd in (("plate", F.CAL_PLATE), ("card", F.COLOR_CARD)):
        ox, oy, oz = bd["origin"]
        if name == "plate":
            sx, sy = bd["squares"][0] * bd["square"], bd["squares"][1] * bd["square"]
        else:
            sx = (bd["patches"][0] - 1) * bd["pitch"] + bd["patch"]
            sy = (bd["patches"][1] - 1) * bd["pitch"] + bd["patch"]
        mg = bd["margin"]
        b.box("%s/%s/riser" % (C, name), (sx + 2 * mg, sy + 2 * mg, oz - 0.0012 - F.DECK_Z), (ox + sx / 2, oy + sy / 2, (oz - 0.0012 + F.DECK_Z) / 2), "pedestal")
        b.box("%s/%s/board" % (C, name), (sx + 2 * mg, sy + 2 * mg, 0.001), (ox + sx / 2, oy + sy / 2, oz - 0.0005), "plate_white" if name == "plate" else "plate_black")
        if name == "plate":
            sq = bd["square"]
            for i in range(bd["squares"][0]):
                for j in range(bd["squares"][1]):
                    if (i + j) % 2 == 0:
                        b.box("%s/plate/sq_%d_%d" % (C, i, j), (sq, sq, 0.0002), (ox + (i + 0.5) * sq, oy + (j + 0.5) * sq, oz + 0.0001), "plate_black")
        else:
            n = 0
            for j in range(bd["patches"][1]):
                for i in range(bd["patches"][0]):
                    lk = "card_%d" % n
                    b.mat(lk, bd["albedo"][n], 0.0, 0.90)
                    b.box("%s/card/p_%d" % (C, n), (bd["patch"], bd["patch"], 0.0003),
                          (ox + i * bd["pitch"] + bd["patch"] / 2, oy + j * bd["pitch"] + bd["patch"] / 2, oz + 0.00015), lk)
                    n += 1
    # the cart's own body: deck, pedestal and crates collide with the tool, the arm and the trusses (the arm's own
    # links do not collide with each other, so these are not part of the articulation); held to the base by a joint
    cart = b.xform("/World/Cart")
    b.body(cart, 90.0, iters=(32, 4), damping=(0.0, 0.0))
    dk = F.CART["deck"]
    b.box("/World/Cart/deck", (dk["x"][1] - dk["x"][0], dk["y"][1] - dk["y"][0], 0.05), ((dk["x"][0] + dk["x"][1]) / 2, (dk["y"][0] + dk["y"][1]) / 2, F.DECK_Z - 0.025),
          "deck", collide=True)
    for i, (wx, wy) in enumerate(((-0.28, -0.8), (0.28, -0.8), (-0.28, 0.8), (0.28, 0.8))):
        b.cyl("/World/Cart/wheel%d" % i, 0.07, 0.05, (wx, wy, F.FLOOR_Z + 0.155), "pedestal", axis="X")
    b.cyl("/World/Cart/pedestal", 0.16, -F.DECK_Z - 0.004, (0, 0, (F.DECK_Z - 0.004) / 2), "pedestal", collide=True)
    fz, wh, wt = F.CART["crate_floor_z"], F.CART["crate_wall_h"], F.CART["crate_wall_t"]
    for name, cr in F.CART["crates"].items():
        (cx, cy), (sx, sy) = cr["center"], cr["size"]
        root = "/World/Cart/crate_" + name
        prims = [b.box(root + "/floor", (sx + 2 * wt, sy + 2 * wt, fz - F.DECK_Z), (cx, cy, (fz + F.DECK_Z) / 2), "crate", collide=True),
                 b.box(root + "/wx0", (wt, sy + 2 * wt, wh), (cx - sx / 2 - wt / 2, cy, fz + wh / 2), "crate", collide=True),
                 b.box(root + "/wx1", (wt, sy + 2 * wt, wh), (cx + sx / 2 + wt / 2, cy, fz + wh / 2), "crate", collide=True),
                 b.box(root + "/wy0", (sx, wt, wh), (cx, cy - sy / 2 - wt / 2, fz + wh / 2), "crate", collide=True),
                 b.box(root + "/wy1", (sx, wt, wh), (cx, cy + sy / 2 + wt / 2, fz + wh / 2), "crate", collide=True)]
        nb, dh, dt = F.CART["crate_bays"], F.CART["crate_divider_h"], F.CART["crate_divider_t"]
        for k in range(1, nb):
            prims.append(b.box("%s/div%d" % (root, k), (dt, sy, dh), (cx - sx / 2 + sx * k / nb, cy, fz + dh / 2), "crate", collide=True))
        for p in prims:
            b.friction(p, "crate", 0.9)
    cj = UsdPhysics.FixedJoint.Define(stage, "/World/CartJoint")
    cj.CreateBody0Rel().SetTargets([R + "/base_link"])
    cj.CreateBody1Rel().SetTargets(["/World/Cart"])
    for a in (cj.CreateLocalPos0Attr, cj.CreateLocalPos1Attr):
        a(Gf.Vec3f(0, 0, 0))
    for a in (cj.CreateLocalRot0Attr, cj.CreateLocalRot1Attr):
        a(Gf.Quatf(1, 0, 0, 0))
    cj.CreateExcludeFromArticulationAttr(True)

    # ------------------------------------------------------------------ the tool
    tl = F.TOOL
    ee = b.xform(R + "/ee_link")
    b.body(ee, tl["mass"], com=tl["com"], inertia=tl["inertia"], damping=(0.0, 0.0), iters=(32, 4))
    fj = UsdPhysics.FixedJoint.Define(stage, R + "/joints/ee_joint")
    fj.CreateBody0Rel().SetTargets([R + "/wrist_3_link"])
    fj.CreateBody1Rel().SetTargets([R + "/ee_link"])
    for a in (fj.CreateLocalPos0Attr, fj.CreateLocalPos1Attr):
        a(Gf.Vec3f(0, 0, 0))
    for a in (fj.CreateLocalRot0Attr, fj.CreateLocalRot1Attr):
        a(Gf.Quatf(1, 0, 0, 0))
    E = R + "/ee_link"
    b.cyl(E + "/adapter", 0.045, 0.012, (0, 0, 0.006), "steel", collide=True)
    bd, nk, jw = tl["body"], tl["neck"], tl["jaw"]
    b.box(E + "/body", (bd["size"][0], bd["size"][1], bd["z"][1] - bd["z"][0]), (0, 0, (bd["z"][0] + bd["z"][1]) / 2), "tool_body", collide=True)
    b.box(E + "/neck", (nk["size"][0], nk["size"][1], nk["z"][1] - nk["z"][0]), (0, 0, (nk["z"][0] + nk["z"][1]) / 2), "tool_dark", collide=True)
    jx, jl = (jw["x"][0] + jw["x"][1]) / 2, jw["z"][1] - jw["z"][0]
    jwid = jw["x"][1] - jw["x"][0]
    for side, sgn in (("l", -1.0), ("r", 1.0)):
        # the jaws' colliders stand at the open mouth; what shows moves with the commanded opening (looks only)
        b.box("%s/jaw_col_%s" % (E, side), (jwid, jw["t"], jl), (jx, sgn * (tl["open_max"] / 2 + jw["t"] / 2), (jw["z"][0] + jw["z"][1]) / 2),
              None, collide=True, visible=False)
        f = b.xform("%s/jaw_%s" % (E, side), (0, sgn * tl["open_max"] / 2, 0))
        b.box("%s/jaw_%s/bar" % (E, side), (jwid, jw["t"], jl), (jx, sgn * jw["t"] / 2, (jw["z"][0] + jw["z"][1]) / 2), "steel")
        b.box("%s/jaw_%s/pad" % (E, side), (0.010, 0.0015, jl - 0.004), (tl["clamp_x"], -sgn * 0.0006, (jw["z"][0] + jw["z"][1]) / 2), "pad")
        index["jaw_" + side] = str(f.GetPath())
    wc = F.CAMERAS["wrist"]
    b.box(E + "/cam_housing", (0.036, 0.044, 0.040), (wc["pos"][0] - 0.004, wc["pos"][1], wc["pos"][2] - 0.024), "tool_dark", collide=True)
    b.box(E + "/cam_arm", (abs(wc["pos"][0]) - 0.02, 0.02, 0.02), (wc["pos"][0] / 2 - 0.004, 0, wc["pos"][2] - 0.02), "tool_dark")
    index.update(robot=R, rail=R + "/rail_link", base=R + "/base_link", ee=E, cart="/World/Cart")

    # ------------------------------------------------------------------ cameras
    if render:
        for name, c in F.CAMERAS.items():
            path = R + "/base_link/overview_cam" if c["parent"] == "cart" else E + "/wrist_cam"
            cam = UsdGeom.Camera.Define(stage, path)
            hap = 20.955
            cam.CreateHorizontalApertureAttr(hap)
            cam.CreateVerticalApertureAttr(hap * c["height"] / c["width"])
            cam.CreateFocalLengthAttr(0.5 * hap / math.tan(math.radians(c["hfov_deg"]) / 2))
            cam.CreateClippingRangeAttr(Gf.Vec2f(*c["clip"]))
            cam.CreateFStopAttr(0.0)
            # this robot's own camera: the nominal mounting times its mounting offset (farm.MOUNT_TOL)
            Tc = F.camera_mount(name, (row.get("cameras") or {}).get(name))
            Rc = Tc[:3, :3]                                     # OpenCV axes; a USD camera looks along -z with +y up
            Ru = np.stack([Rc[:, 0], -Rc[:, 1], -Rc[:, 2]], axis=1)
            b.place(cam.GetPrim(), Tc[:3, 3], kin.R_quat(Ru))
            index["cam_" + name] = path

    # ------------------------------------------------------------------ the plants
    mech = row["mechanics"]
    zeta = mech["damping"]
    b.xform("/World/Row")
    b.xform("/World/Row/Joints")
    fruit_pm = b.physics_material("fruit", 0.7)
    for p in row["plants"]:
        pid = p["id"]
        P = "/World/Row/" + pid
        b.xform(P)
        pts = [np.array(q, dtype=float) for q in p["points"]]
        n_seg = len(pts) - 1
        seg_T, seg_path = [], []
        for i in range(n_seg):
            T = member_T(pts[i], pts[i + 1])
            L = float(np.linalg.norm(pts[i + 1] - pts[i]))
            path = "%s/s%d" % (P, i)
            prim = b.xform(path, T[:3, 3], kin.R_quat(T[:3, :3]))
            m_seg = 0.085
            b.body(prim, m_seg, inertia=(m_seg * L * L / 12.0, m_seg * L * L / 12.0, 2e-5), com=(0, 0, L / 2), damping=(0.08, 0.5))
            b.capsule(path + "/cap", p["stem_r"], (0, 0, 0), (0, 0, L), "stem", collide=True)
            mid = "%s.s%d" % (pid, i)
            jp = "/World/Row/Joints/%s_s%d" % (pid, i)
            k = mech["stem_k"] * (2.5 if i == 0 else 1.0)
            dmp = 2.0 * zeta * math.sqrt(k * m_seg * L * L / 3.0) * 3.0
            if i == 0:
                b.d6(jp, None, None, path, T, T, k_rot=k, d_rot=dmp)
            else:
                b.d6(jp, seg_path[i - 1], seg_T[i - 1], path, T, T, k_rot=k, d_rot=dmp)
            seg_T.append(T)
            seg_path.append(path)
            index["bodies"][mid], index["joints"][mid] = path, jp
            index["stems"].append(mid)
        # the support string: the stem's top hangs on the high wire
        top = pts[-1]
        Ttop = kin.pose(np.eye(3), top)
        ks = mech["string_k"]
        b.d6("/World/Row/Joints/%s_string" % pid, None, None, seg_path[-1], seg_T[-1], Ttop, k_rot=None,
             k_lin=[ks, ks, 4000.0], d_lin=2.0 * zeta * math.sqrt(ks * 0.5) * 2.0)
        if render:
            b.capsule("%s/string" % P, 0.0012, top, top + np.array([0.0, 0.0, 2.2]), "string")

        def stem_of(z):
            for i in range(n_seg):
                if pts[i][2] <= z <= pts[i + 1][2]:
                    return i
            return n_seg - 1

        for t in p["trusses"]:
            tid = t["id"]
            Tr = "%s/T%s" % (P, tid[1:])
            b.xform(Tr)
            a0, b0 = np.array(t["peduncle"]["a"]), np.array(t["peduncle"]["b"])
            Tp = member_T(a0, b0)
            L0 = float(np.linalg.norm(b0 - a0))
            path = Tr + "/p"
            prim = b.xform(path, Tp[:3, 3], kin.R_quat(Tp[:3, :3]))
            b.body(prim, 0.012, inertia=(6e-6, 6e-6, 2e-6), com=(0, 0, L0 / 2), damping=(0.05, 0.8))
            b.capsule(path + "/cap", t["peduncle"]["r"], (0, 0, 0), (0, 0, L0), "peduncle", collide=True)
            si = stem_of(a0[2])
            m_truss = sum(f["mass"] for f in t["fruits"]) + 0.03
            k0 = t["k"]
            d0 = 2.0 * 0.30 * math.sqrt(k0 * m_truss * 0.12 ** 2)
            mid = tid + ".p"
            jp = "/World/Row/Joints/%s_p" % tid
            b.d6(jp, seg_path[si], seg_T[si], path, Tp, Tp, k_rot=k0, d_rot=d0)
            index["bodies"][mid], index["joints"][mid] = path, jp
            index["peduncles"].append(mid)
            prev_path, prev_T = path, Tp
            for k, s in enumerate(t["rachis"]):
                Ts = member_T(s["a"], s["b"])
                Ls = float(np.linalg.norm(np.array(s["b"]) - np.array(s["a"])))
                path = "%s/r%d" % (Tr, k)
                prim = b.xform(path, Ts[:3, 3], kin.R_quat(Ts[:3, :3]))
                fr = [f for f in t["fruits"] if f["segment"] == k]
                b.body(prim, 0.008 + sum(f["mass"] for f in fr), damping=(0.08, 1.2), report=True,
                       inertia=None if fr else (4e-6, 4e-6, 1e-6), com=None if fr else (0, 0, Ls / 2))
                b.capsule(path + "/cap", t["rachis_r"], (0, 0, 0), (0, 0, Ls), "peduncle", collide=True)
                for j, f in enumerate(fr):
                    c = Ts[:3, :3].T @ (np.array(f["center"]) - Ts[:3, 3])
                    at = Ts[:3, :3].T @ (np.array(f["attach"]) - Ts[:3, 3])
                    look = "fruit_%s_%d_%d" % (tid, k, j)
                    b.mat(look, f["albedo"], 0.0, 0.28)
                    fp = b.sphere("%s/fruit%d" % (path, j), f["r"], c, look, collide=True)
                    UsdShade.MaterialBindingAPI.Apply(fp).Bind(fruit_pm, UsdShade.Tokens.weakerThanDescendants, "physics")
                    if render:
                        b.capsule("%s/pedicel%d" % (path, j), t["rachis_r"] * 0.8, at, c + (at - c) * (f["r"] / np.linalg.norm(at - c)) * 0.97, "peduncle")
                        cp, cc, ci = calyx_mesh(f["r"])
                        b.mesh("%s/calyx%d" % (path, j), cp, cc, ci, c, kin.R_quat(frame_z(at - c)), "calyx")
                mid = "%s.r%d" % (tid, k)
                jp = "/World/Row/Joints/%s_r%d" % (tid, k)
                # (the joint's frame is the segment's: z along it. It bends against `rachis_k` and turns about its own
                # axis against almost nothing, as fruits on their pedicels let a truss settle flat)
                kb = F.TRUSS["knuckle_k"] if k == 0 else F.TRUSS["rachis_k"]
                b.d6(jp, prev_path, prev_T, path, Ts, Ts, k_rot=[kb, kb, F.TRUSS["twist_k"]],
                     d_rot=[F.TRUSS["rachis_d"]] * 2 + [F.TRUSS["twist_d"]])
                index["bodies"][mid], index["joints"][mid] = path, jp
                index["rachis"].append(mid)
                prev_path, prev_T = path, Ts
        for lf in p["leaves"]:
            a0, b0 = np.array(lf["petiole"]["a"]), np.array(lf["petiole"]["b"])
            Tl = member_T(a0, b0)
            Lp = float(np.linalg.norm(b0 - a0))
            path = "%s/%s" % (P, lf["id"][1:])
            prim = b.xform(path, Tl[:3, 3], kin.R_quat(Tl[:3, :3]))
            b.body(prim, lf["mass"], damping=(0.3, 1.5))
            b.capsule(path + "/petiole", lf["petiole"]["r"], (0, 0, 0), (0, 0, Lp), "peduncle", collide=True)
            db = np.array(lf["blade"]["dir"])
            wdir = np.cross([0.0, 0.0, 1.0], db)
            wdir = wdir / np.linalg.norm(wdir)
            Rb = np.stack([db, wdir, np.cross(db, wdir)], axis=1)          # blade frame in W: x along, y across, z normal
            Rl = Tl[:3, :3].T @ Rb
            pos = np.array([0.0, 0.0, Lp])
            ln, wd = lf["blade"]["length"], lf["blade"]["width"]
            b.box(path + "/blade_col", (ln * 0.92, wd * 0.8, 0.003), pos + Rl @ np.array([ln * 0.52, 0.0, 0.0]), None, collide=True,
                  quat=kin.R_quat(Rl), visible=False)
            if render:
                mp, mc, mi = leaf_mesh(ln, wd)
                b.mesh(path + "/blade", mp, mc, mi, pos, kin.R_quat(Rl), "leaf")
            si = stem_of(a0[2])
            jp = "/World/Row/Joints/%s" % lf["id"]
            b.d6(jp, seg_path[si], seg_T[si], path, Tl, Tl, k_rot=lf["k"], d_rot=2.0 * 0.5 * math.sqrt(lf["k"] * lf["mass"] * 0.2 ** 2))
            index["bodies"][lf["id"]], index["joints"][lf["id"]] = path, jp
            index["leaves"].append(lf["id"])

    # leaves pass through each other and through stems (they lie against each other in a real canopy; here they are
    # plates on single joints, and crossing plates would only jitter)
    g_leaf = UsdPhysics.CollisionGroup.Define(stage, "/World/Row/group_leaves")
    g_stem = UsdPhysics.CollisionGroup.Define(stage, "/World/Row/group_stems")
    g_leaf.GetCollidersCollectionAPI().CreateIncludesRel().SetTargets([index["bodies"][m] for m in index["leaves"]] or ["/World/Row/Joints"])
    g_stem.GetCollidersCollectionAPI().CreateIncludesRel().SetTargets([index["bodies"][m] for m in index["stems"]])
    g_leaf.CreateFilteredGroupsRel().SetTargets(["/World/Row/group_leaves", "/World/Row/group_stems"])
    return index

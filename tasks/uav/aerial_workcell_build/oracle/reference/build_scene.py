"""The reference build of the aerial workcell: the oracle's artifact.

    python3 build_scene.py --out /home/user/submission/scene.usda [--truth truth.json]

Writes one USD stage that follows the interface in the agent's `~/given/interface.md`: the cell (floor, two tables,
a calibration mat), the bin (a fixed articulation: body with the button post, a hinged lid with a handle, a spring
button) whose mechanism is an OmniGraph state machine (no script nodes), six YCB objects, and two drones (each one
floating articulation: airframe + an inverted SO-101 arm + a camera) resting on their stands, inside a warehouse
backdrop. With --truth it also writes the layout the grader compares against (author use only).
"""

import argparse
import json
import math
import os

from isaacsim import SimulationApp

ap = argparse.ArgumentParser()
ap.add_argument("--out", required=True)
ap.add_argument("--truth")
args = ap.parse_args()

app = SimulationApp({"headless": True})
import omni.kit.app  # noqa: E402

omni.kit.app.get_app().get_extension_manager().set_extension_enabled_immediate("isaacsim.core.nodes", True)
import omni.graph.core as og  # noqa: E402
import omni.usd  # noqa: E402
from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade  # noqa: E402

ASSETS = "/opt/assets/isaac/Isaac"

# ---------------------------------------------------------------------------------------------------------------------
# layout (world frame: z up, metres; the cell's origin is the centre of the calibration mat's far edge row)
# ---------------------------------------------------------------------------------------------------------------------
MAT = dict(center=(0.0, -0.60), squares=(7, 5), square=0.10)
TABLES = {  # top centre xy, top size (x, y), top height, yaw [deg], top thickness
    "Table_A": dict(center=(-1.30, 0.70), size=(1.20, 0.60), height=0.75, yaw=0.0, thick=0.04),
    "Table_B": dict(center=(1.45, 0.55), size=(0.80, 0.60), height=0.45, yaw=20.0, thick=0.03),
}
BIN = dict(origin=(0.10, 1.50), yaw=0.0)
ITEMS = {  # name: (asset below Props/YCB, table, xy, yaw [deg], mass [kg]); stood upright (their meshes' up is -y)
    "cracker_box": ("Axis_Aligned_Physics/003_cracker_box.usd", "Table_A", (-1.62, 0.62), 10.0, 0.411),
    "mustard_bottle": ("Axis_Aligned_Physics/006_mustard_bottle.usd", "Table_A", (-1.18, 0.80), -30.0, 0.603),
    "sugar_box": ("Axis_Aligned_Physics/004_sugar_box.usd", "Table_A", (-0.90, 0.58), 75.0, 0.514),
    "soup_can": ("Axis_Aligned_Physics/005_tomato_soup_can.usd", "Table_B", (1.30, 0.50), 0.0, 0.349),
    "power_drill": ("Axis_Aligned/035_power_drill.usd", "Table_B", (1.62, 0.66), 40.0, 0.895),
    "mug": ("Axis_Aligned/025_mug.usd", "Table_B", (1.42, 0.34), -60.0, 0.118),
}
DRONES = {  # stand centre xy, yaw [deg], airframe bottom height on the stand
    "Drone_L": dict(xy=(-0.75, -1.45), yaw=0.0, rest=0.62),
    "Drone_S": dict(xy=(0.85, -1.45), yaw=0.0, rest=0.58),
}

# bin geometry (bin frame: origin on the floor below the bin's centre, +y = back, hinge along x at the back top edge)
BIN_W, BIN_D, BIN_H, WALL = 0.44, 0.40, 0.30, 0.015
LID_T, LID_MASS, LID_OPEN_DEG, TOGGLE_DEG = 0.02, 0.8, 105.0, 28.0
HINGE = (0.0, BIN_D / 2, BIN_H + LID_T / 2)
BAR_LEN, BAR_R, BAR_OUT, BAR_UP = 0.10, 0.006, 0.06, 0.025
HANDLE = (0.0, -BIN_D / 2 - BAR_OUT, BIN_H + LID_T + BAR_UP)
POST_XY, POST_SIZE, POST_H = (-0.55, -0.78), 0.08, 0.50
BUTTON_R, BUTTON_H, BUTTON_TRAVEL = 0.03, 0.02, 0.010
BUTTON_OPEN_S = 20.0

ctx = omni.usd.get_context()
ctx.new_stage()
st = ctx.get_stage()
UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(st, 1.0)
UsdPhysics.SetStageKilogramsPerUnit(st, 1.0)
world = UsdGeom.Xform.Define(st, "/World")
st.SetDefaultPrim(world.GetPrim())
scene = UsdPhysics.Scene.Define(st, "/World/PhysicsScene")
scene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
scene.CreateGravityMagnitudeAttr(9.81)
PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim()).CreateTimeStepsPerSecondAttr(120)
truth = {"tables": {}, "bin": {}, "items": {}, "drones": {}}


def rotz(deg):
    return Gf.Rotation(Gf.Vec3d(0, 0, 1), deg)


def xform(path, pos=(0, 0, 0), yaw=0.0, quat=None):
    x = UsdGeom.Xform.Define(st, path)
    x.AddTranslateOp().Set(Gf.Vec3d(*pos))
    q = quat if quat is not None else Gf.Quatf(rotz(yaw).GetQuat())
    x.AddOrientOp().Set(Gf.Quatf(q))
    return x


def material(name, rgb, rough=0.6, metal=0.0):
    path = f"/World/Looks/{name}"
    m = UsdShade.Material.Define(st, path)
    s = UsdShade.Shader.Define(st, path + "/S")
    s.CreateIdAttr("UsdPreviewSurface")
    s.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
    s.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(rough)
    s.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metal)
    m.CreateSurfaceOutput().ConnectToSource(s.ConnectableAPI(), "surface")
    return m


M = {k: material(k, *v) for k, v in {
    "floor": ((0.42, 0.42, 0.44), 0.85), "wood": ((0.55, 0.40, 0.26), 0.55), "steel": ((0.60, 0.62, 0.65), 0.35, 0.7),
    "bin": ((0.20, 0.42, 0.30),), "lid": ((0.16, 0.34, 0.24),), "dark": ((0.12, 0.12, 0.13),),
    "metal": ((0.75, 0.75, 0.78), 0.3, 0.8), "red": ((0.85, 0.12, 0.10), 0.4), "white": ((0.92, 0.92, 0.92), 0.7),
    "black": ((0.03, 0.03, 0.03), 0.7), "frame": ((0.18, 0.19, 0.21), 0.5, 0.3), "prop": ((0.05, 0.05, 0.05), 0.4),
    "orange": ((0.95, 0.45, 0.08), 0.5)}.items()}


def box(path, center, size, mat, col=True):
    g = UsdGeom.Cube.Define(st, path)
    g.CreateSizeAttr(1.0)
    g.AddTranslateOp().Set(Gf.Vec3d(*center))
    g.AddScaleOp().Set(Gf.Vec3f(*size))
    if col:
        UsdPhysics.CollisionAPI.Apply(g.GetPrim())
    UsdShade.MaterialBindingAPI.Apply(g.GetPrim()).Bind(mat)
    return g


def box_lohi(path, lo, hi, mat, col=True):
    return box(path, [(a + b) / 2 for a, b in zip(lo, hi)], [b - a for a, b in zip(lo, hi)], mat, col)


def cyl(path, center, r, h, axis, mat, col=True):
    g = UsdGeom.Cylinder.Define(st, path)
    g.CreateRadiusAttr(r)
    g.CreateHeightAttr(h)
    g.CreateAxisAttr(axis)
    g.AddTranslateOp().Set(Gf.Vec3d(*center))
    if col:
        UsdPhysics.CollisionAPI.Apply(g.GetPrim())
    UsdShade.MaterialBindingAPI.Apply(g.GetPrim()).Bind(mat)
    return g


# ---------------------------------------------------------------------------------------------------------------------
# backdrop, light, cell
# ---------------------------------------------------------------------------------------------------------------------
bd = UsdGeom.Xform.Define(st, "/World/Backdrop")
bd.GetPrim().GetReferences().AddReference(f"{ASSETS}/Environments/Simple_Warehouse/warehouse.usd")
dome = UsdLux.DomeLight.Define(st, "/World/Lights/Dome")
dome.CreateIntensityAttr(250.0)
key = UsdLux.RectLight.Define(st, "/World/Lights/Key")
key.CreateIntensityAttr(2500.0)
key.CreateWidthAttr(3.0)
key.CreateHeightAttr(3.0)
UsdGeom.Xformable(key).AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 4.5))

floor = box("/World/Cell/Floor", (0, 0, -0.01), (12.0, 12.0, 0.02), M["floor"])
UsdGeom.Imageable(floor).MakeInvisible()  # a collider under the warehouse's own floor
cx, cy = MAT["center"]
nx, ny = MAT["squares"]
s = MAT["square"]
for i in range(nx):
    for j in range(ny):
        box(f"/World/Cell/CalibMat/sq_{i}_{j}", (cx + (i - (nx - 1) / 2) * s, cy + (j - (ny - 1) / 2) * s, 0.0006),
            (s, s, 0.001), M["white" if (i + j) % 2 == 0 else "black"], col=False)
truth["calib_mat"] = dict(center=list(MAT["center"]), squares=list(MAT["squares"]), square=s)

for name, t in TABLES.items():
    x0, y0 = t["center"]
    xf = xform(f"/World/Cell/{name}", (x0, y0, 0.0), t["yaw"])
    w, d, h, th = t["size"][0], t["size"][1], t["height"], t["thick"]
    box(f"/World/Cell/{name}/top", (0, 0, h - th / 2), (w, d, th), M["wood"])
    for k, (sx, sy) in enumerate(((1, 1), (1, -1), (-1, 1), (-1, -1))):
        box(f"/World/Cell/{name}/leg_{k}", (sx * (w / 2 - 0.05), sy * (d / 2 - 0.05), (h - th) / 2), (0.05, 0.05, h - th),
            M["steel"])
    truth["tables"][name] = dict(center=[x0, y0], size=[w, d], height=h, yaw=t["yaw"])

# ---------------------------------------------------------------------------------------------------------------------
# the bin: one fixed articulation (root: Body, anchored to the world where it stands)
# ---------------------------------------------------------------------------------------------------------------------
bx, by = BIN["origin"]
xform("/World/Bin", (bx, by, 0.0), BIN["yaw"])
w, d, h, t = BIN_W / 2, BIN_D / 2, BIN_H, WALL
body = UsdGeom.Xform.Define(st, "/World/Bin/Body")
UsdPhysics.RigidBodyAPI.Apply(body.GetPrim())
UsdPhysics.MassAPI.Apply(body.GetPrim()).CreateMassAttr(15.0)
UsdPhysics.ArticulationRootAPI.Apply(body.GetPrim())
pa = PhysxSchema.PhysxArticulationAPI.Apply(body.GetPrim())
pa.CreateSolverPositionIterationCountAttr(32)
pa.CreateEnabledSelfCollisionsAttr(False)
for nm, lo, hi, m in (("floor", (-w, -d, 0), (w, d, t), "bin"), ("wall_front", (-w, -d, 0), (w, -d + t, h), "bin"),
                      ("wall_back", (-w, d - t, 0), (w, d, h), "bin"), ("wall_left", (-w, -d, 0), (-w + t, d, h), "bin"),
                      ("wall_right", (w - t, -d, 0), (w, d, h), "bin"), ("stop", (-w, d, h - 0.06), (w, d + 0.03, h - 0.04), "dark"),
                      ("post", (POST_XY[0] - POST_SIZE / 2, POST_XY[1] - POST_SIZE / 2, 0),
                       (POST_XY[0] + POST_SIZE / 2, POST_XY[1] + POST_SIZE / 2, POST_H), "dark")):
    box_lohi(f"/World/Bin/Body/{nm}", lo, hi, M[m])
anchor = UsdPhysics.FixedJoint.Define(st, "/World/Bin/RootJoint")
anchor.CreateBody1Rel().SetTargets(["/World/Bin/Body"])
anchor.CreateLocalPos0Attr(Gf.Vec3f(bx, by, 0.0))
anchor.CreateLocalRot0Attr(Gf.Quatf(rotz(BIN["yaw"]).GetQuat()))
lid = UsdGeom.Xform.Define(st, "/World/Bin/Lid")
UsdPhysics.RigidBodyAPI.Apply(lid.GetPrim())
UsdPhysics.MassAPI.Apply(lid.GetPrim()).CreateMassAttr(LID_MASS)
box_lohi("/World/Bin/Lid/panel", (-w, -d, h), (w, d, h + LID_T), M["lid"])
hx, hy, hz = HANDLE
UsdGeom.Xform.Define(st, "/World/Bin/Lid/Handle")
for i, bxx in enumerate((-BAR_LEN / 2 - 0.006, BAR_LEN / 2 + 0.006)):
    box_lohi(f"/World/Bin/Lid/Handle/bracket_{i}", (bxx - 0.006, hy - 0.008, h + LID_T - 0.004),
             (bxx + 0.006, -d + 0.01, hz + 0.008), M["metal"])
grip = cyl("/World/Bin/Lid/Handle/bar", (hx, hy, hz), BAR_R, BAR_LEN + 0.024, "X", M["red"])
gm = UsdShade.Material.Define(st, "/World/Looks/grip_rubber")
gmat = UsdPhysics.MaterialAPI.Apply(gm.GetPrim())
gmat.CreateStaticFrictionAttr(1.5)
gmat.CreateDynamicFrictionAttr(1.3)
UsdShade.MaterialBindingAPI.Apply(grip.GetPrim()).Bind(gm, UsdShade.Tokens.weakerThanDescendants, "physics")
btn = UsdGeom.Xform.Define(st, "/World/Bin/Button")
UsdPhysics.RigidBodyAPI.Apply(btn.GetPrim())
UsdPhysics.MassAPI.Apply(btn.GetPrim()).CreateMassAttr(0.02)
cyl("/World/Bin/Button/cap", (POST_XY[0], POST_XY[1], POST_H + BUTTON_H / 2), BUTTON_R, BUTTON_H, "Z", M["red"])
hj = UsdPhysics.RevoluteJoint.Define(st, "/World/Bin/LidHinge")
hj.CreateBody0Rel().SetTargets(["/World/Bin/Body"])
hj.CreateBody1Rel().SetTargets(["/World/Bin/Lid"])
hj.CreateAxisAttr("X")
hj.CreateLowerLimitAttr(-LID_OPEN_DEG)
hj.CreateUpperLimitAttr(0.0)
hj.CreateLocalPos0Attr(Gf.Vec3f(*HINGE))
hj.CreateLocalPos1Attr(Gf.Vec3f(*HINGE))
dr = UsdPhysics.DriveAPI.Apply(hj.GetPrim(), "angular")
dr.CreateTypeAttr("force")
dr.CreateStiffnessAttr(0.0)
dr.CreateDampingAttr(0.06)
dr.CreateTargetPositionAttr(0.0)
dr.CreateMaxForceAttr(5.0)
bj = UsdPhysics.PrismaticJoint.Define(st, "/World/Bin/ButtonJoint")
bj.CreateBody0Rel().SetTargets(["/World/Bin/Body"])
bj.CreateBody1Rel().SetTargets(["/World/Bin/Button"])
bj.CreateAxisAttr("Z")
bj.CreateLowerLimitAttr(-BUTTON_TRAVEL)
bj.CreateUpperLimitAttr(0.0)
bd_ = UsdPhysics.DriveAPI.Apply(bj.GetPrim(), "linear")
bd_.CreateTypeAttr("force")
bd_.CreateStiffnessAttr(1000.0)
bd_.CreateDampingAttr(20.0)
bd_.CreateTargetPositionAttr(0.0)
bd_.CreateMaxForceAttr(20.0)
truth["bin"] = dict(origin=list(BIN["origin"]), yaw=BIN["yaw"], size=[BIN_W, BIN_D, BIN_H], hinge_local=list(HINGE),
                    handle_local=list(HANDLE), post_local=list(POST_XY), post_height=POST_H, open_deg=LID_OPEN_DEG,
                    toggle_deg=TOGGLE_DEG, button_open_s=BUTTON_OPEN_S, button_travel=BUTTON_TRAVEL)

# --- the mechanism: an OmniGraph state machine on every physics step. Modes: 0 closed (gravity; the strut eases the
# lift), 1 the strut carries the lid open, 2 held on the stop, 3 the motor opens it (button), 4 soft close (pulled off
# the stop), 5 the motor closes it. Each step it writes the hinge drive's target / stiffness / damping for the mode.
K = og.Controller.Keys
nodes, vals, conns = [], [], []


def node(nid, typ, **inputs):
    nodes.append((nid, typ))
    for k, v in inputs.items():
        vals.append((f"{nid}.inputs:{k}", v))
    return nid


def const(nid, v, typ="Double"):
    return node(nid, f"omni.graph.nodes.Constant{typ}", value=v) + ".inputs:value"


def link(src, dst):
    conns.append((src, dst))


def op(nid, typ, a, b, **kw):
    node(nid, typ, **kw)
    link(a, f"{nid}.inputs:a")
    link(b, f"{nid}.inputs:b")
    return nid


def cmp(nid, a, b, operation):
    return op(nid, "omni.graph.nodes.Compare", a, b, operation=operation) + ".outputs:result"


def sel(nid, cond, yes, no):
    node(nid, "omni.graph.nodes.SelectIf")
    link(cond, f"{nid}.inputs:condition")
    link(yes, f"{nid}.inputs:ifTrue")
    link(no, f"{nid}.inputs:ifFalse")
    return f"{nid}.outputs:result"


def band(nid, a, b):
    return op(nid, "omni.graph.nodes.BooleanAnd", a, b) + ".outputs:result"


node("tick", "isaacsim.core.nodes.OnPhysicsStep")
node("state", "isaacsim.core.nodes.IsaacArticulationState", robotPath="/World/Bin/Body", jointNames=["LidHinge", "ButtonJoint"])
link("tick.outputs:step", "state.inputs:execIn")
node("q_lid", "omni.graph.nodes.ArrayIndex", index=0)
node("q_btn", "omni.graph.nodes.ArrayIndex", index=1)
link("state.outputs:jointPositions", "q_lid.inputs:array")
link("state.outputs:jointPositions", "q_btn.inputs:array")
open_deg = op("open_deg", "omni.graph.nodes.Multiply", "q_lid.outputs:value", const("k_open", -180.0 / math.pi)) + ".outputs:product"
btn_mm = op("btn_mm", "omni.graph.nodes.Multiply", "q_btn.outputs:value", const("k_btn", -1000.0)) + ".outputs:product"
# the graph keeps its own clock: the sum of the physics steps it has seen (IsaacReadSimulationTime, a pure data node,
# was read stale at the moment of a press: the 20 s window came out at 18 s)
node("clock_r", "omni.graph.core.ReadVariable", variableName="clock")
now = op("clock_next", "omni.graph.nodes.Add", "clock_r.outputs:value", "tick.outputs:deltaSimulationTime") + ".outputs:sum"
node("mode_r", "omni.graph.core.ReadVariable", variableName="mode")
node("tpress_r", "omni.graph.core.ReadVariable", variableName="t_press")
mode, tpress = "mode_r.outputs:value", "tpress_r.outputs:value"
I = {k: const(f"i{k}", k, "Int") for k in range(6)}
is_m = {k: cmp(f"is_m{k}", mode, I[k], "==") for k in range(6)}
above_toggle = cmp("above_toggle", open_deg, const("c_toggle_a", TOGGLE_DEG), ">")
below_toggle = cmp("below_toggle", open_deg, const("c_toggle_b", TOGGLE_DEG), "<")
pressed = cmp("pressed", btn_mm, const("c_press", 4.0), ">")
settled = cmp("settled", now, const("c_settle", 0.5), ">")  # (a spring button sags at the very first step)
on_stop = cmp("on_stop", open_deg, const("c_on_stop", LID_OPEN_DEG - 2.0), ">=")
off_stop = cmp("off_stop", open_deg, const("c_off_stop", LID_OPEN_DEG - 8.0), "<")
shut = cmp("shut", open_deg, const("c_shut", 1.0), "<")
elapsed = op("elapsed", "omni.graph.nodes.Subtract", now, tpress) + ".outputs:difference"
timed_out = cmp("timed_out", elapsed, const("c_open_s", BUTTON_OPEN_S), ">")
press_ok = band("press_ok", band("pressed_settled", pressed, settled), below_toggle)
n0 = sel("n0", press_ok, I[3], sel("n0b", above_toggle, I[1], I[0]))
n1 = sel("n1", on_stop, I[2], sel("n1b", below_toggle, I[0], I[1]))
n2 = sel("n2", off_stop, I[4], I[2])
n3 = sel("n3", timed_out, I[5], I[3])
n4 = sel("n4", shut, I[0], I[4])
n5 = sel("n5", shut, I[0], I[5])
nxt = sel("m0", is_m[0], n0, sel("m1", is_m[1], n1, sel("m2", is_m[2], n2, sel("m3", is_m[3], n3, sel("m4", is_m[4], n4, n5)))))
tpress_next = sel("tpress_sel", band("start_press", is_m[0], press_ok), now, tpress)
is_n = {k: cmp(f"is_n{k}", nxt, I[k], "==") for k in range(6)}
drive = {}
for key, per_mode in (("targetPosition", {0: -TOGGLE_DEG, 1: -LID_OPEN_DEG, 2: -LID_OPEN_DEG, 3: -LID_OPEN_DEG, 4: 0.0, 5: 0.0}),
                      ("stiffness", {0: 0.023, 1: 0.05, 2: 0.05, 3: 0.1, 4: 0.01, 5: 0.1}),
                      ("damping", {0: 0.06, 1: 0.03, 2: 0.03, 3: 0.15, 4: 0.09, 5: 0.15})):
    cs = {k: const(f"{key}_{k}", v) for k, v in per_mode.items()}
    expr = cs[5]
    for k in (4, 3, 2, 1, 0):
        expr = sel(f"{key}_s{k}", is_n[k], cs[k], expr)
    drive[key] = expr
# the writes, in an exec chain. A data input is pulled when its writer runs, and a variable read after it has been
# written this step reads the new value - so everything that depends on a variable is written before that variable:
# drive (needs mode, t_press, clock) -> t_press (needs mode, clock) -> mode (needs t_press, clock) -> clock
prev = "tick.outputs:step"
for key, expr in drive.items():
    node(f"w_{key}", "omni.graph.nodes.WritePrimAttribute", primPath="/World/Bin/LidHinge", usePath=True,
         name=f"drive:angular:physics:{key}")
    link(prev, f"w_{key}.inputs:execIn")
    link(expr, f"w_{key}.inputs:value")
    prev = f"w_{key}.outputs:execOut"
for var, expr in (("t_press", tpress_next), ("mode", nxt), ("clock", now)):
    node(f"{var}_w", "omni.graph.core.WriteVariable", variableName=var)
    link(prev, f"{var}_w.inputs:execIn")
    link(expr, f"{var}_w.inputs:value")
    prev = f"{var}_w.outputs:execOut"
og.Controller.edit({"graph_path": "/World/Bin/Logic", "evaluator_name": "execution",
                    "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND},
                   {K.CREATE_NODES: nodes, K.CREATE_VARIABLES: [("mode", og.Type(og.BaseDataType.INT)),
                                                                ("t_press", og.Type(og.BaseDataType.DOUBLE)),
                                                                ("clock", og.Type(og.BaseDataType.DOUBLE))],
                    K.SET_VALUES: vals, K.CONNECT: conns})

# ---------------------------------------------------------------------------------------------------------------------
# items: YCB meshes (y-up) stood upright on the tables
# ---------------------------------------------------------------------------------------------------------------------
cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"])
for name, (asset, table, (ix, iy), yaw, mass) in ITEMS.items():
    path = f"/World/Items/{name}"
    up = Gf.Rotation(Gf.Vec3d(1, 0, 0), -90.0)  # the YCB meshes' up is their -y: -> world +z
    q = (up * rotz(yaw)).GetQuat()
    it = xform(path, (ix, iy, 0.0), quat=Gf.Quatf(q))
    it.GetPrim().GetReferences().AddReference(f"{ASSETS}/Props/YCB/{asset}")
    prim = it.GetPrim()
    if not prim.HasAPI(UsdPhysics.RigidBodyAPI):  # the plain YCB meshes: give them a body and a convex collider
        UsdPhysics.RigidBodyAPI.Apply(prim)
        for p in Usd.PrimRange(prim):
            if p.IsA(UsdGeom.Mesh):
                UsdPhysics.CollisionAPI.Apply(p)
                UsdPhysics.MeshCollisionAPI.Apply(p).CreateApproximationAttr("convexDecomposition")
    UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(mass)
    lo = cache.ComputeWorldBound(prim).ComputeAlignedRange().GetMin()[2]
    top = TABLES[table]["height"]
    it.GetOrderedXformOps()[0].Set(Gf.Vec3d(ix, iy, top - lo + 0.002))
    truth["items"][name] = dict(asset=asset.split("/")[-1], table=table, xy=[ix, iy], yaw=yaw, mass=mass)
    cache.Clear()

# ---------------------------------------------------------------------------------------------------------------------
# drones: one floating articulation each = airframe + the SO-101 hung upside down below it (fixed mount) + a camera
# ---------------------------------------------------------------------------------------------------------------------
ARM_DROP = 0.02  # [m] the arm's base plate below the airframe's bottom


def mount_arm(root, frame_body, frame_bottom_z):
    """Reference the SO-101 under `root`/Arm, drop its world anchor, fix its base upside down under the frame."""
    arm = UsdGeom.Xform.Define(st, f"{root}/Arm")
    arm.GetPrim().GetReferences().AddReference(f"{ASSETS}/Robots/RobotStudio/so101_new_calib/so101_new_calib.usd")
    rj = st.GetPrimAtPath(f"{root}/Arm/root_joint")
    rj.SetActive(False)  # the asset bolts its base to the world; here the drone carries it
    base = st.GetPrimAtPath(f"{root}/Arm/base")
    # place the whole arm upside down (180 deg about x) with its base plate ARM_DROP below the frame
    flip = Gf.Quatf(Gf.Rotation(Gf.Vec3d(1, 0, 0), 180.0).GetQuat())
    xa = UsdGeom.Xformable(arm)
    xa.ClearXformOpOrder()  # (the asset's own root ops would compose underneath ours)
    xa.AddTranslateOp(opSuffix="mount").Set(Gf.Vec3d(0, 0, frame_bottom_z - ARM_DROP))
    xa.AddOrientOp(opSuffix="mount").Set(flip)
    j = UsdPhysics.FixedJoint.Define(st, f"{root}/ArmMount")
    j.CreateBody0Rel().SetTargets([frame_body])
    j.CreateBody1Rel().SetTargets([base.GetPath()])
    j.CreateLocalPos0Attr(Gf.Vec3f(0, 0, frame_bottom_z - ARM_DROP))
    j.CreateLocalRot0Attr(flip)
    j.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
    j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
    return arm


def camera(path, pos, pitch_down, focal, hap=20.955, vap=15.716):
    """A camera looking along the drone's +y (its forward), pitched down."""
    c = UsdGeom.Camera.Define(st, path)
    c.CreateFocalLengthAttr(focal)
    c.CreateHorizontalApertureAttr(hap)
    c.CreateVerticalApertureAttr(vap)
    c.CreateClippingRangeAttr(Gf.Vec2f(0.02, 100.0))
    # USD cameras look down their -z with +y up: turn -z to +y (forward), then pitch it down about x
    q = Gf.Rotation(Gf.Vec3d(1, 0, 0), 90.0 - pitch_down)
    x = UsdGeom.Xformable(c)
    x.AddTranslateOp().Set(Gf.Vec3d(*pos))
    x.AddOrientOp().Set(Gf.Quatf(q.GetQuat()))
    return dict(pos=list(pos), pitch_down=pitch_down, focal=focal, hfov_deg=math.degrees(2 * math.atan(hap / 2 / focal)))


def stand(root_xy, yaw, name, top_z, half_span, length=0.36):
    """A launch stand: two rails under the airframe's sides along its forward axis, on four legs (the hanging arm
    passes between the rails)."""
    sx, sy = root_xy
    x = xform(f"/World/Cell/Stand_{name}", (sx, sy, 0.0), yaw)
    for k, side in enumerate((-1, 1)):
        box(f"/World/Cell/Stand_{name}/rail_{k}", (side * half_span, 0.0, top_z - 0.02), (0.04, length, 0.04), M["orange"])
        for e, end in enumerate((-1, 1)):
            box(f"/World/Cell/Stand_{name}/leg_{k}{e}", (side * half_span, end * (length / 2 - 0.02), (top_z - 0.04) / 2),
                (0.03, 0.03, top_z - 0.04), M["steel"])
    return x


# --- the small drone: NTNU ARL-Robot-1 airframe (its own articulation root, 1.2 kg + props) ---
cfg = DRONES["Drone_S"]
frame_bottom = -0.10  # the airframe's bottom below its frame origin
root = "/World/Robots/Drone_S"
xform(root, (cfg["xy"][0], cfg["xy"][1], cfg["rest"] - frame_bottom), cfg["yaw"])
air = UsdGeom.Xform.Define(st, f"{root}/Airframe")
air.GetPrim().GetReferences().AddReference(f"{ASSETS}/Robots/NTNU/ARL-Robot-1/arl_robot_1.usd")
# the asset collides as one 0.5 m cube about its frame origin (it would rest 15 cm above its visible frame): replace it
# by a box around the frame itself
st.GetPrimAtPath(f"{root}/Airframe/base_link/collisions").SetActive(False)
fc = box(f"{root}/Airframe/base_link/frame_collider", (0, 0, -0.03), (0.46, 0.46, 0.14), M["dark"])
fc.CreatePurposeAttr(UsdGeom.Tokens.guide)
mount_arm(root, f"{root}/Airframe/base_link", frame_bottom)
cam_s = camera(f"{root}/Airframe/base_link/Camera", (0.0, 0.11, -0.05), 40.0, 12.0)
stand(cfg["xy"], cfg["yaw"], "S", cfg["rest"], 0.11)
truth["drones"]["Drone_S"] = dict(airframe="arl_robot_1", arm="so101_new_calib", xy=list(cfg["xy"]), yaw=cfg["yaw"],
                                  rest=cfg["rest"], camera=cam_s)

# --- the large drone: a 0.9 m (motor to motor, diagonal) quadrotor built here; one rigid frame, rotors marked ---
cfg = DRONES["Drone_L"]
root = "/World/Robots/Drone_L"
xform(root, (cfg["xy"][0], cfg["xy"][1], cfg["rest"] + 0.071), cfg["yaw"])  # skids (bottom 0.07 below) on the stand
fr = UsdGeom.Xform.Define(st, f"{root}/Frame")
UsdPhysics.RigidBodyAPI.Apply(fr.GetPrim())
UsdPhysics.MassAPI.Apply(fr.GetPrim()).CreateMassAttr(3.20)
UsdPhysics.ArticulationRootAPI.Apply(fr.GetPrim())
PhysxSchema.PhysxArticulationAPI.Apply(fr.GetPrim()).CreateEnabledSelfCollisionsAttr(False)
box(f"{root}/Frame/body", (0, 0, 0), (0.30, 0.30, 0.12), M["frame"])
box(f"{root}/Frame/battery", (0, 0, 0.085), (0.18, 0.10, 0.05), M["dark"])
R = 0.32  # [m] motor offset along each axis (diagonal arms)
for k, (ax, ay) in enumerate(((1, 1), (-1, 1), (-1, -1), (1, -1))):
    ang = math.degrees(math.atan2(ay, ax))
    arm_c = (ax * R / 2, ay * R / 2, 0.02)
    a = box(f"{root}/Frame/arm_{k}", arm_c, (R * math.sqrt(2), 0.04, 0.03), M["frame"])
    a.AddOrientOp().Set(Gf.Quatf(rotz(ang).GetQuat()))
    a.SetXformOpOrder([a.GetOrderedXformOps()[0], a.GetOrderedXformOps()[2], a.GetOrderedXformOps()[1]])
    rot_ = UsdGeom.Xform.Define(st, f"{root}/Frame/Rotor_{k}")
    rot_.AddTranslateOp().Set(Gf.Vec3d(ax * R, ay * R, 0.07))
    rot_.GetPrim().CreateAttribute("arb:spin", Sdf.ValueTypeNames.Int).Set(1 if k % 2 == 0 else -1)
    cyl(f"{root}/Frame/Rotor_{k}/motor", (0, 0, -0.025), 0.035, 0.05, "Z", M["steel"])
    cyl(f"{root}/Frame/Rotor_{k}/prop", (0, 0, 0.005), 0.19, 0.006, "Z", M["prop"], col=False)
for k, side in enumerate((-1, 1)):  # skids the stand carries
    box(f"{root}/Frame/skid_{k}", (side * 0.17, 0, -0.06), (0.03, 0.34, 0.02), M["dark"])
mount_arm(root, f"{root}/Frame", -0.07)
cam_l = camera(f"{root}/Frame/Camera", (0.0, 0.17, -0.03), 30.0, 18.0)
stand(cfg["xy"], cfg["yaw"], "L", cfg["rest"], 0.17)
truth["drones"]["Drone_L"] = dict(airframe="custom_quad_0p9m", arm="so101_new_calib", xy=list(cfg["xy"]), yaw=cfg["yaw"],
                                  rest=cfg["rest"], rotor_offset=R, frame_mass=3.20, camera=cam_l)

# the robot assets ship their meshes as instanceable prims; rendered, those stayed at the asset origin while the bodies
# moved (2026-10-30). Plain prims follow their bodies.
for p in list(st.Traverse()):
    if p.IsInstanceable() and p.GetPath().pathString.startswith(("/World/Robots", "/World/Items")):
        p.SetInstanceable(False)

os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
st.GetRootLayer().Export(args.out)
if args.truth:
    with open(args.truth, "w") as f:
        json.dump(truth, f, indent=1)
print("[reference] scene ->", args.out, flush=True)
os._exit(0)

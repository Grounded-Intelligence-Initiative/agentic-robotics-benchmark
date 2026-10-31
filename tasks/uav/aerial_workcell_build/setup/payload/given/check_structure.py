"""Static check of a workcell scene against the interface (`interface.md`): files, stage settings, required prims and
joints, OmniGraph rules. Reads the USD as data (pxr only, no Kit, nothing executed) and scores nothing.

    python3 scan.py /home/user/submission/scene.usda [--json]

The grader runs this same file before it measures anything; a scene with errors scores zero.
"""

import json
import os
import sys

from pxr import Sdf, Usd, UsdGeom, UsdPhysics, UsdUtils

ASSET_ROOT = "/opt/assets/isaac"
CORE_MDL = {"OmniPBR.mdl", "OmniGlass.mdl", "OmniSurface.mdl", "OmniSurfaceLite.mdl", "OmniEmissive.mdl",
            "OmniHair.mdl", "OmniUe4Base.mdl", "OmniUe4Function.mdl", "OmniUe4Translucent.mdl", "OmniUe4FunctionExtension17.mdl",
            "OmniImage.mdl", "OmniPBRBase.mdl", "OmniPBR_ClearCoat.mdl", "OmniPBR_Opacity.mdl", "OmniSurfaceBase.mdl"}
ALLOWED_NODE_PREFIXES = ("omni.graph.action.", "omni.graph.nodes.", "omni.graph.core.")
ALLOWED_NODES = {"isaacsim.core.nodes.OnPhysicsStep", "isaacsim.core.nodes.IsaacArticulationState",
                 "isaacsim.core.nodes.IsaacArticulationController", "isaacsim.core.nodes.IsaacReadSimulationTime",
                 "isaacsim.sensors.physics.IsaacReadJointState"}
FORBIDDEN_NODES = {"omni.graph.nodes.WritePrimLocalTransform", "omni.graph.nodes.MoveToTransform",
                   "omni.graph.nodes.WritePrims", "omni.graph.nodes.WritePrimsV2", "omni.graph.nodes.WritePrim",
                   "omni.graph.nodes.WritePrimRelationship", "omni.graph.nodes.WritePrimMaterial",
                   "omni.graph.nodes.StartTimeline", "omni.graph.nodes.StopTimeline", "omni.graph.nodes.SetTimeline",
                   "omni.graph.nodes.LoopTimeline", "omni.graph.nodes.SetVariantSelection",
                   "omni.graph.nodes.ClearVariantSelection", "omni.graph.nodes.WriteSetting"}
REQUIRED = ("/World/Cell/Floor", "/World/Cell/Table_A", "/World/Cell/Table_B", "/World/Bin/Body", "/World/Bin/Lid",
            "/World/Bin/Lid/Handle", "/World/Bin/Button", "/World/Bin/LidHinge", "/World/Bin/ButtonJoint", "/World/Items",
            "/World/Robots/Drone_L", "/World/Robots/Drone_S")


def _inside(path, roots):
    p = os.path.realpath(path)
    return any(p == r or p.startswith(r + os.sep) for r in roots)


def _referenced_assets(prim):
    """Asset paths of the references / payloads that compose `prim` (its own arcs, not its descendants')."""
    out = []
    for spec in prim.GetPrimStack():
        for lst in (spec.referenceList, spec.payloadList):
            for item in list(lst.prependedItems) + list(lst.appendedItems) + list(lst.explicitItems) + list(lst.addedItems):
                if item.assetPath:
                    out.append(item.assetPath)
    return out


def scan(scene):
    errors, notes, facts = [], [], {}
    sub = os.path.realpath(os.path.dirname(scene))
    roots = (sub, os.path.realpath(ASSET_ROOT))
    if not os.path.isfile(scene):
        return dict(ok=False, errors=[f"no scene file at {scene}"], notes=notes, facts=facts)
    try:
        layers, assets, unresolved = UsdUtils.ComputeAllDependencies(scene)
    except Exception as exc:  # noqa: BLE001
        return dict(ok=False, errors=[f"the scene cannot be read: {exc}"], notes=notes, facts=facts)
    for layer in layers:
        if layer.realPath and not _inside(layer.realPath, roots):
            errors.append(f"layer outside the submission and the asset library: {layer.realPath}")
    for a in assets:
        if not _inside(str(a), roots):
            errors.append(f"file outside the submission and the asset library: {a}")
    for u in unresolved:
        if os.path.basename(str(u)) not in CORE_MDL:
            errors.append(f"unresolved file: {u}")
    stage = Usd.Stage.Open(scene)
    if stage is None:
        return dict(ok=False, errors=errors + ["the scene does not open"], notes=notes, facts=facts)
    if UsdGeom.GetStageUpAxis(stage) != UsdGeom.Tokens.z:
        errors.append("upAxis is not Z")
    if abs(UsdGeom.GetStageMetersPerUnit(stage) - 1.0) > 1e-9:
        errors.append("metersPerUnit is not 1")
    if abs(UsdPhysics.GetStageKilogramsPerUnit(stage) - 1.0) > 1e-9:
        errors.append("kilogramsPerUnit is not 1")
    if not stage.GetDefaultPrim() or stage.GetDefaultPrim().GetPath() != Sdf.Path("/World"):
        errors.append("the default prim is not /World")
    scenes = [p for p in stage.Traverse() if p.IsA(UsdPhysics.Scene)]
    if [str(p.GetPath()) for p in scenes] != ["/World/PhysicsScene"]:
        errors.append(f"expected exactly one physics scene at /World/PhysicsScene, found {[str(p.GetPath()) for p in scenes]}")
    for path in REQUIRED:
        if not stage.GetPrimAtPath(path).IsValid():
            errors.append(f"missing {path}")
    if errors:
        return dict(ok=False, errors=errors, notes=notes, facts=facts)

    def has(path, api):
        return stage.GetPrimAtPath(path).HasAPI(api)

    for path in ("/World/Bin/Body", "/World/Bin/Lid", "/World/Bin/Button"):
        if not has(path, UsdPhysics.RigidBodyAPI):
            errors.append(f"{path} is not a rigid body")
    if not has("/World/Bin/Body", UsdPhysics.ArticulationRootAPI):
        errors.append("/World/Bin/Body carries no ArticulationRootAPI")
    for path, kind, b0, b1 in (("/World/Bin/LidHinge", UsdPhysics.RevoluteJoint, "/World/Bin/Body", "/World/Bin/Lid"),
                               ("/World/Bin/ButtonJoint", UsdPhysics.PrismaticJoint, "/World/Bin/Body", "/World/Bin/Button")):
        prim = stage.GetPrimAtPath(path)
        if not prim.IsA(kind):
            errors.append(f"{path} is not a {kind.__name__}")
            continue
        j = UsdPhysics.Joint(prim)
        got = ([str(t) for t in j.GetBody0Rel().GetTargets()], [str(t) for t in j.GetBody1Rel().GetTargets()])
        if got != ([b0], [b1]):
            errors.append(f"{path} connects {got}, expected {b0} -> {b1}")
    floor = stage.GetPrimAtPath("/World/Cell/Floor")
    if not any(p.HasAPI(UsdPhysics.CollisionAPI) for p in Usd.PrimRange(floor, Usd.TraverseInstanceProxies())):
        errors.append("/World/Cell/Floor has no collider")
    items = [c for c in stage.GetPrimAtPath("/World/Items").GetChildren() if c.HasAPI(UsdPhysics.RigidBodyAPI)]
    if not items:
        errors.append("no rigid body under /World/Items")
    # the library models each item uses (on the item's prim or anywhere below it)
    facts["items"] = {c.GetName(): sorted({a for p in Usd.PrimRange(c) for a in _referenced_assets(p)}) for c in items}
    # robots: each one floating articulation; nothing fixes one of its bodies to the world
    joints = [p for p in stage.Traverse() if p.IsA(UsdPhysics.Joint) and p.IsActive()]
    facts["robots"] = {}
    for name in ("Drone_L", "Drone_S"):
        root = f"/World/Robots/{name}"
        rp = stage.GetPrimAtPath(root)
        art = [p for p in Usd.PrimRange(rp) if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
        if len(art) != 1:
            errors.append(f"{root} holds {len(art)} articulation roots, expected 1")
        for j in joints:
            jj = UsdPhysics.Joint(j)
            b0, b1 = jj.GetBody0Rel().GetTargets(), jj.GetBody1Rel().GetTargets()
            ends = [str(t) for t in list(b0) + list(b1)]
            if (not b0 or not b1) and any(e.startswith(root + "/") or e == root for e in ends):
                errors.append(f"{j.GetPath()} fixes {root} to the world")
        refs = sorted({a for p in Usd.PrimRange(rp) for a in _referenced_assets(p)})
        cams = [str(p.GetPath()) for p in Usd.PrimRange(rp) if p.IsA(UsdGeom.Camera)]
        facts["robots"][name] = dict(articulation_root=str(art[0].GetPath()) if art else None, assets=refs, cameras=cams)
        if not cams:
            notes.append(f"{root} carries no camera")
    # OmniGraph: node types and what they write
    n_nodes = 0
    for p in stage.Traverse():
        if p.GetTypeName() != "OmniGraphNode":
            continue
        n_nodes += 1
        t = p.GetAttribute("node:type").Get() if p.GetAttribute("node:type") else None
        where = str(p.GetPath())
        if t in FORBIDDEN_NODES or (t and "script" in t.lower()):
            errors.append(f"{where}: node type {t} is not allowed")
            continue
        if not t or not (t.startswith(ALLOWED_NODE_PREFIXES) or t in ALLOWED_NODES):
            errors.append(f"{where}: node type {t} is not allowed")
            continue
        if t == "omni.graph.nodes.WritePrimAttribute":
            na = p.GetAttribute("inputs:name")
            if na.HasAuthoredConnections() if na else True:
                errors.append(f"{where}: the written attribute's name must be a constant")
                continue
            name = na.Get() or ""
            if not (name.startswith("drive:") or name.startswith("arb:")):
                errors.append(f"{where}: writes {name!r}; only drive:* and arb:* attributes may be written")
    facts["graph_nodes"] = n_nodes
    return dict(ok=not errors, errors=errors, notes=notes, facts=facts)


def main():
    scene = sys.argv[1] if len(sys.argv) > 1 and not sys.argv[1].startswith("--") else "/home/user/submission/scene.usda"
    if not os.path.isfile(scene) and scene.endswith(".usda") and os.path.isfile(scene[:-1]):
        scene = scene[:-1]
    r = scan(scene)
    if "--json" in sys.argv:
        print(json.dumps(r))
        return
    print("OK" if r["ok"] else "FAILED")
    for e in r["errors"]:
        print("  error:", e)
    for n in r["notes"]:
        print("  note:", n)


if __name__ == "__main__":
    main()

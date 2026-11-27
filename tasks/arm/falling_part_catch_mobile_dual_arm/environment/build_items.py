"""Build OmniGibson-loadable USD items from NVIDIA's public YCB assets, on the CPU (no Kit, no URDF importer).

Per item: /object (default prim) / base_link / visuals (textured mesh) + collisions/part_k (convex parts from CoACD,
each with CollisionAPI, MeshCollisionAPI approximation=convexHull, PhysxCollisionAPI). The mesh is recentred on its
bounding-box centre and kept in metres. Masses are the YCB data-set values (kg)."""
import json, shutil, sys
from pathlib import Path
import numpy as np, trimesh
from pxr import Usd, UsdGeom, UsdPhysics, UsdShade, Sdf, Gf, Vt
SRC = Path(sys.argv[2] if len(sys.argv) > 2 else "/opt/isaac_assets/Isaac/Props/YCB/Axis_Aligned"); OUT = Path(sys.argv[1]); OUT.mkdir(parents=True, exist_ok=True)
MASS = {"002_master_chef_can": 0.414, "003_cracker_box": 0.411, "004_sugar_box": 0.514, "005_tomato_soup_can": 0.349,
        "006_mustard_bottle": 0.603, "007_tuna_fish_can": 0.171, "008_pudding_box": 0.187, "009_gelatin_box": 0.097,
        "010_potted_meat_can": 0.370, "011_banana": 0.066, "019_pitcher_base": 0.178, "021_bleach_cleanser": 1.131,
        "024_bowl": 0.147, "025_mug": 0.118, "035_power_drill": 0.895, "036_wood_block": 0.729, "037_scissors": 0.082,
        "040_large_marker": 0.016, "051_large_clamp": 0.125, "052_extra_large_clamp": 0.202, "061_foam_brick": 0.028}
try:
    import coacd; coacd.set_log_level("error")
except Exception: coacd = None
info = {}
for usd in sorted(SRC.glob("*.usd")):
    name = usd.stem; st = Usd.Stage.Open(str(usd))
    mesh = next(p for p in st.Traverse() if p.GetTypeName() == "Mesh"); m = UsdGeom.Mesh(mesh)
    M = np.array(UsdGeom.Xformable(mesh).ComputeLocalToWorldTransform(Usd.TimeCode.Default()))
    pts = np.array(m.GetPointsAttr().Get(), dtype=float); pts = (np.c_[pts, np.ones(len(pts))] @ M)[:, :3] * UsdGeom.GetStageMetersPerUnit(st)
    pts = pts - (pts.max(0) + pts.min(0)) / 2
    idx = np.array(m.GetFaceVertexIndicesAttr().Get()).reshape(-1, 3)
    pv = UsdGeom.PrimvarsAPI(mesh).GetPrimvar("st"); uv = np.array(pv.Get(), dtype=float) if pv and pv.Get() is not None else None
    tex = None
    for p in st.Traverse():
        a = p.GetAttribute("inputs:diffuse_texture")
        if a and a.Get() is not None: tex = SRC / str(a.Get().path); break
    d = OUT / name; d.mkdir(exist_ok=True)
    if tex is not None and tex.exists(): shutil.copy(tex, d / tex.name)
    tm = trimesh.Trimesh(pts, idx, process=False)
    parts = []
    if coacd is not None:
        try:
            for v, f in coacd.run_coacd(coacd.Mesh(pts, idx), threshold=0.05, max_convex_hull=12):
                parts.append(trimesh.Trimesh(np.asarray(v), np.asarray(f), process=False).convex_hull)
        except Exception as e: print("coacd failed", name, e)
    if not parts: parts = [tm.convex_hull]
    out = Usd.Stage.CreateNew(str(d / f"{name}.usd")); UsdGeom.SetStageUpAxis(out, UsdGeom.Tokens.z); UsdGeom.SetStageMetersPerUnit(out, 1.0)
    root = out.DefinePrim("/object", "Xform"); out.SetDefaultPrim(root); out.DefinePrim("/object/base_link", "Xform")
    vis = UsdGeom.Mesh.Define(out, "/object/base_link/visuals")
    vis.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(pts.astype(np.float32))); vis.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(idx.astype(np.int32).ravel()))
    vis.CreateFaceVertexCountsAttr(Vt.IntArray([3] * len(idx))); vis.CreateSubdivisionSchemeAttr("none")
    vis.CreateExtentAttr([Gf.Vec3f(*pts.min(0)), Gf.Vec3f(*pts.max(0))])
    if uv is not None:
        UsdGeom.PrimvarsAPI(vis).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex).Set(Vt.Vec2fArray.FromNumpy(uv.astype(np.float32)))
    out.DefinePrim("/object/Looks", "Scope")
    mat = UsdShade.Material.Define(out, "/object/Looks/item"); sh = UsdShade.Shader.Define(out, "/object/Looks/item/surface"); sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.6); sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
    if tex is not None and tex.exists():
        rd = UsdShade.Shader.Define(out, "/object/Looks/item/st"); rd.CreateIdAttr("UsdPrimvarReader_float2"); rd.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
        tx = UsdShade.Shader.Define(out, "/object/Looks/item/tex"); tx.CreateIdAttr("UsdUVTexture")
        tx.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath("./" + tex.name)); tx.CreateInput("sourceColorSpace", Sdf.ValueTypeNames.Token).Set("sRGB")
        tx.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(rd.ConnectableAPI(), "result")
        sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(tx.ConnectableAPI(), "rgb")
    else:
        sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.6, 0.6, 0.6))
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface"); UsdShade.MaterialBindingAPI.Apply(vis.GetPrim()).Bind(mat)
    out.DefinePrim("/object/base_link/collisions", "Xform")
    for k, part in enumerate(parts):
        cm = UsdGeom.Mesh.Define(out, f"/object/base_link/collisions/part_{k}")
        cm.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(np.asarray(part.vertices, dtype=np.float32)))
        cm.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(np.asarray(part.faces, dtype=np.int32).ravel())); cm.CreateFaceVertexCountsAttr(Vt.IntArray([3] * len(part.faces)))
        cm.CreateSubdivisionSchemeAttr("none"); UsdGeom.Imageable(cm.GetPrim()).CreatePurposeAttr(UsdGeom.Tokens.guide)
        UsdPhysics.CollisionAPI.Apply(cm.GetPrim()); UsdPhysics.MeshCollisionAPI.Apply(cm.GetPrim()).CreateApproximationAttr("convexHull")
        cm.GetPrim().AddAppliedSchema("PhysxCollisionAPI")   # PhysxSchema is not importable outside Kit; the schema name is enough
    out.Save()
    size = pts.max(0) - pts.min(0)
    info[name] = {"usd": str(d / f"{name}.usd"), "size_m": size.round(4).tolist(), "min_dim_m": float(np.sort(size)[0]), "mass_kg": MASS[name],
                  "collision_parts": len(parts), "texture": bool(tex is not None and tex.exists()), "fits_gripper": bool(np.sort(size)[0] < 0.115)}
    print(f"{name:24s} parts {len(parts):2d} size {size.round(3).tolist()} mass {MASS[name]} fits {info[name]['fits_gripper']}", flush=True)
(OUT / "items.json").write_text(json.dumps(info, indent=1)); print("BUILT", len(info), "coacd", coacd is not None)

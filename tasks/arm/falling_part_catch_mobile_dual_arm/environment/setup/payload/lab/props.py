"""Station props as OmniGibson-loadable USD files with authored friction (written with pxr; no Kit needed).

Every prop is /object/base_link/{visuals,collisions/part_k} built from boxes, in the prop's own frame, with one physics
material bound to its colliders. Friction combine mode "min" makes the prop's friction decide how items slide on it."""
from __future__ import annotations
import numpy as np

# pxr is imported inside the functions: importing it before the Kit app starts loads the wrong USD libraries and the
# simulator then exits at start-up (exit code 127, seen 2026-11-24).

_BOX_F = np.array([[0, 2, 1], [0, 3, 2], [4, 5, 6], [4, 6, 7], [0, 1, 5], [0, 5, 4], [2, 3, 7], [2, 7, 6], [1, 2, 6], [1, 6, 5], [0, 4, 7], [0, 7, 3]])


def _box(size, centre):
    s, c = np.asarray(size, float) / 2, np.asarray(centre, float)
    v = np.array([[-1, -1, -1], [1, -1, -1], [1, 1, -1], [-1, 1, -1], [-1, -1, 1], [1, -1, 1], [1, 1, 1], [-1, 1, 1]]) * s + c
    return v, _BOX_F


def _mesh(stage, path, v, f, collider=False):
    from pxr import UsdGeom, UsdPhysics, Vt
    m = UsdGeom.Mesh.Define(stage, path)
    m.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(np.asarray(v, np.float32))); m.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(np.asarray(f, np.int32).ravel()))
    m.CreateFaceVertexCountsAttr(Vt.IntArray([3] * len(f))); m.CreateSubdivisionSchemeAttr("none")
    if collider:
        UsdGeom.Imageable(m.GetPrim()).CreatePurposeAttr(UsdGeom.Tokens.guide)
        UsdPhysics.CollisionAPI.Apply(m.GetPrim()); UsdPhysics.MeshCollisionAPI.Apply(m.GetPrim()).CreateApproximationAttr("convexHull")
        m.GetPrim().AddAppliedSchema("PhysxCollisionAPI")
    return m


def write_prop(path, boxes, rgb=(0.5, 0.5, 0.55), friction=None, roughness=0.5):
    """boxes: list of (size xyz, centre xyz). friction None leaves the default material."""
    from pxr import Usd, UsdGeom, UsdPhysics, UsdShade, Sdf, Gf
    st = Usd.Stage.CreateNew(str(path)); UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z); UsdGeom.SetStageMetersPerUnit(st, 1.0)
    root = st.DefinePrim("/object", "Xform"); st.SetDefaultPrim(root); st.DefinePrim("/object/base_link", "Xform")
    vs, fs, n = [], [], 0
    for size, centre in boxes:
        v, f = _box(size, centre); vs.append(v); fs.append(f + n); n += len(v)
    vis = _mesh(st, "/object/base_link/visuals", np.concatenate(vs), np.concatenate(fs))
    st.DefinePrim("/object/Looks", "Scope")
    mat = UsdShade.Material.Define(st, "/object/Looks/paint"); sh = UsdShade.Shader.Define(st, "/object/Looks/paint/surface"); sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb)); sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
    sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface"); UsdShade.MaterialBindingAPI.Apply(vis.GetPrim()).Bind(mat)
    st.DefinePrim("/object/base_link/collisions", "Xform")
    pm = None
    if friction is not None:
        pm = UsdShade.Material.Define(st, "/object/Looks/contact"); api = UsdPhysics.MaterialAPI.Apply(pm.GetPrim())
        api.CreateStaticFrictionAttr(float(friction)); api.CreateDynamicFrictionAttr(float(friction)); api.CreateRestitutionAttr(0.0)
        pm.GetPrim().AddAppliedSchema("PhysxMaterialAPI")
        pm.GetPrim().CreateAttribute("physxMaterial:frictionCombineMode", Sdf.ValueTypeNames.Token).Set("min")
        pm.GetPrim().CreateAttribute("physxMaterial:restitutionCombineMode", Sdf.ValueTypeNames.Token).Set("min")
    for k, (size, centre) in enumerate(boxes):
        v, f = _box(size, centre); c = _mesh(st, f"/object/base_link/collisions/part_{k}", v, f, collider=True)
        if pm is not None:
            UsdShade.MaterialBindingAPI.Apply(c.GetPrim()).Bind(pm, UsdShade.Tokens.weakerThanDescendants, "physics")
    st.Save()
    return str(path)


# Geometry shared by the simulator, the judge and the manuals (metres).
CHUTE = {"length": 1.15, "width": 0.5, "rail": 0.05, "stop": 0.06, "thick": 0.03}
TOTE = {"inner": (0.42, 0.32, 0.16), "wall": 0.012}
BELT = {"length": 1.56, "width": 0.5, "pitch": 0.12, "slat_thick": 0.03}
ROD_FRICTION = 1.6        # sponge sleeve against anything (combine mode "max")
FEEDER = {"window": 0.56, "depth": 0.26, "thick": 0.08, "arm": 0.30}     # overhead drop feeder: plate (depth along the reach, window across), rods hang under it in a rack


def chute_boxes():
    """Chute frame: origin at the low end, on the sliding surface, centred across; +x runs up the chute."""
    L, W, r, s, t = (CHUTE[k] for k in ("length", "width", "rail", "stop", "thick"))
    return [((L, W, t), (L / 2, 0, -t / 2)), ((L, 0.02, r), (L / 2, W / 2 - 0.01, r / 2)), ((L, 0.02, r), (L / 2, -W / 2 + 0.01, r / 2)),
            ((0.02, W, s), (0.01, 0, s / 2))]


def tote_boxes():
    """Tote frame: origin at the centre of the inner floor."""
    (a, b, h), w = TOTE["inner"], TOTE["wall"]
    return [((a + 2 * w, b + 2 * w, w), (0, 0, -w / 2)), ((w, b + 2 * w, h), (a / 2 + w / 2, 0, h / 2)), ((w, b + 2 * w, h), (-a / 2 - w / 2, 0, h / 2)),
            ((a, w, h), (0, b / 2 + w / 2, h / 2)), ((a, w, h), (0, -b / 2 - w / 2, h / 2))]


def _cyl(radius, z0, z1, n=24):
    """Closed cylinder about z from z0 to z1: vertices and triangles."""
    a = np.linspace(0, 2 * np.pi, n, endpoint=False); ring = np.stack([radius * np.cos(a), radius * np.sin(a)], 1)
    v = np.concatenate([np.c_[ring, np.full(n, z0)], np.c_[ring, np.full(n, z1)], [[0, 0, z0], [0, 0, z1]]]); f = []
    for k in range(n):
        j = (k + 1) % n; f += [[k, j, n + j], [k, n + j, n + k], [2 * n, j, k], [2 * n + 1, n + k, n + j]]
    return v, np.array(f)


def write_rod(path, radius, length, rgb, friction=1.6):
    """A rod in a sponge sleeve, axis along z, centred on the origin: a bare metal core that shows 15 mm at each end and
    a foam sleeve of the given radius over the rest. One high-friction contact material on both colliders, combine mode
    "max", so the sleeve grips whatever closes on it."""
    from pxr import Usd, UsdGeom, UsdPhysics, UsdShade, Sdf, Gf
    st = Usd.Stage.CreateNew(str(path)); UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z); UsdGeom.SetStageMetersPerUnit(st, 1.0)
    root = st.DefinePrim("/object", "Xform"); st.SetDefaultPrim(root); st.DefinePrim("/object/base_link", "Xform"); st.DefinePrim("/object/Looks", "Scope")
    st.DefinePrim("/object/base_link/collisions", "Xform")
    pm = UsdShade.Material.Define(st, "/object/Looks/contact"); api = UsdPhysics.MaterialAPI.Apply(pm.GetPrim())
    api.CreateStaticFrictionAttr(float(friction)); api.CreateDynamicFrictionAttr(float(friction)); api.CreateRestitutionAttr(0.0)
    pm.GetPrim().AddAppliedSchema("PhysxMaterialAPI")
    pm.GetPrim().CreateAttribute("physxMaterial:frictionCombineMode", Sdf.ValueTypeNames.Token).Set("max")
    pm.GetPrim().CreateAttribute("physxMaterial:restitutionCombineMode", Sdf.ValueTypeNames.Token).Set("min")
    cap = 0.015; parts = [_cyl(0.45 * radius, -length / 2, length / 2), _cyl(radius, -length / 2 + cap, length / 2 - cap)]
    vs = np.concatenate([v for v, _ in parts]); fs = np.concatenate([f + sum(len(q[0]) for q in parts[:k]) for k, (_, f) in enumerate(parts)])
    vis = _mesh(st, "/object/base_link/visuals", vs, fs)             # one visual mesh, as the other props and the items have
    mat = UsdShade.Material.Define(st, "/object/Looks/foam"); sh = UsdShade.Shader.Define(st, "/object/Looks/foam/surface"); sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb)); sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.95)
    sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface"); UsdShade.MaterialBindingAPI.Apply(vis.GetPrim()).Bind(mat)
    for k, (v, f) in enumerate(parts):
        c = _mesh(st, f"/object/base_link/collisions/part_{k}", v, f, collider=True)
        UsdShade.MaterialBindingAPI.Apply(c.GetPrim()).Bind(pm, UsdShade.Tokens.weakerThanDescendants, "physics")
    st.Save()
    return str(path)

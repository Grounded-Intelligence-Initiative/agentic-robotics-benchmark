"""Converts the two distractors' source models (Objaverse 1.0) into the room's render bundle:
OBJ (long axis x, thin axis z, metres, centred) and PNG textures, one pair per material.

    python3 tests/review/build_clutter.py <smartphone.glb> <spectacles_case.glb>

Sources: Objaverse 21e953e3d4484aac88ca108da3291b20 ("SmartPhone" by shedmon, CC BY 4.0) and
142a10077f1c48d385888c84213b85bc ("Spectacles case" by lib.test, CC BY 4.0). The scanned case is
reduced to about 6000 triangles per part (fast-simplification).
"""
import os
import sys

import numpy as np
import trimesh

OUT = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), "tests", "room")
# Full sizes (x, y, z): 2 x PHONE_HALF and 2 x CASE_HALF of remote_env.py.
SIZES = {"phone": (0.150, 0.074, 0.009), "glasses_case": (0.160, 0.070, 0.034)}
FACES = {"phone": None, "glasses_case": 6000}


def convert(name, path):
    scene = trimesh.load(path)
    parts = []
    for node in scene.graph.nodes_geometry:
        transform, key = scene.graph[node]
        mesh = scene.geometry[key]
        parts.append((trimesh.transform_points(mesh.vertices, transform), mesh))
    points = np.vstack([p[0] for p in parts])
    lo, hi = points.min(axis=0), points.max(axis=0)
    order = np.argsort(-(hi - lo))                      # long, wide, thin -> x, y, z
    scale = np.array(SIZES[name]) / (hi - lo)[order]
    mirrored = np.linalg.det(np.eye(3)[:, order]) < 0
    for i, (verts, mesh) in enumerate(parts):
        v = (verts - 0.5 * (lo + hi))[:, order] * scale
        f = mesh.faces[:, ::-1] if mirrored else mesh.faces
        uv = np.asarray(mesh.visual.uv)
        if FACES[name] and len(f) > FACES[name]:
            # Reduce the scan; a vertex keeps the texture coordinate of its nearest original.
            import fast_simplification
            from scipy.spatial import cKDTree
            v2, f2 = fast_simplification.simplify(v.astype(np.float32), f.astype(np.int32), target_count=FACES[name])
            uv = uv[cKDTree(v).query(v2)[1]]
            v, f = v2, f2
        if np.ptp(v, axis=0).min() < 1e-5:              # a flat sheet: MuJoCo wants some volume
            v = v + np.random.default_rng(0).normal(0.0, 2e-5, v.shape)
        with open(os.path.join(OUT, "{}_{}.obj".format(name, i)), "w") as handle:
            for p in v:
                handle.write("v %.6f %.6f %.6f\n" % tuple(p))
            for t in uv:
                handle.write("vt %.6f %.6f\n" % tuple(t))
            for a, b, c in f + 1:
                handle.write("f %d/%d %d/%d %d/%d\n" % (a, a, b, b, c, c))
        mesh.visual.material.baseColorTexture.convert("RGB").resize((512, 512)).save(
            os.path.join(OUT, "{}_{}.png".format(name, i)))
        print(name, i, len(f), "triangles", v.min(axis=0).round(4), v.max(axis=0).round(4))


if __name__ == "__main__":
    convert("phone", sys.argv[1])
    convert("glasses_case", sys.argv[2])

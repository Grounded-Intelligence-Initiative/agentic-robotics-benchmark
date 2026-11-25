"""The hat on the chest (Poly Haven "Fishermans Hat", CC0): drawn mesh, texture, and its collision
shapes: a convex decomposition (CoACD) of the same mesh, so that the hollow crown is hollow."""
import os, numpy as np, trimesh, coacd
from PIL import Image
RAW = "/home/lingjun/ale/dev/assets_raw"; OUT = "/home/lingjun/ale/agentic-robotics-benchmark/tasks/arm/tv_remote_search_and_retrieval/tests/room"
SCALE = 1.25
s = trimesh.load(f"{RAW}/ph/fishermans_hat/fishermans_hat.gltf"); g = list(s.geometry.values())[0]
T = s.graph.get(s.graph.geometry_nodes[list(s.geometry)[0]][0])[0]
v = trimesh.transform_points(g.vertices, T) * SCALE
w = np.stack([v[:, 0], -v[:, 2], v[:, 1]], axis=1)          # Z up
# origin: the crown's axis on the plane the hat rests on
top = w[w[:, 2] > w[:, 2].max() - 0.02]; cx, cy = top[:, 0].mean(), top[:, 1].mean()
w = w - np.array([cx, cy, w[:, 2].min()])
uv = np.asarray(g.visual.uv)
with open(f"{OUT}/hat.obj", "w") as h:
    for p in w: h.write("v %.6f %.6f %.6f\n" % tuple(p))
    for t in uv: h.write("vt %.6f %.6f\n" % tuple(t))
    for a, b, c in g.faces + 1: h.write("f %d/%d %d/%d %d/%d\n" % (a, a, b, b, c, c))
d = f"{RAW}/ph/fishermans_hat/textures"
Image.open([os.path.join(d, x) for x in os.listdir(d) if "diff" in x.lower()][0]).convert("RGB").resize((1024, 1024), Image.LANCZOS).save(f"{OUT}/hat.png")
mesh = coacd.Mesh(w, g.faces)
parts = coacd.run_coacd(mesh, threshold=0.02, max_convex_hull=48, preprocess_resolution=80)
for f in os.listdir(OUT):
    if f.startswith("hat_col_"): os.remove(os.path.join(OUT, f))
hulls = [trimesh.Trimesh(pv, pf).convex_hull for pv, pf in parts]
low = min(h.vertices[:, 2].min() for h in hulls)            # the pieces rest where the drawn hat does
for i, hull in enumerate(hulls):
    hull.vertices = hull.vertices - np.array([0.0, 0.0, low])
    hull.export(f"{OUT}/hat_col_{i:02d}.obj")
print("pieces lowered by", round(float(low), 4))
print("parts", len(parts), "bounds", w.min(0).round(3), w.max(0).round(3))
# is the crown hollow? points on a grid inside the cavity must be in no hull
free = []
for r in (0.0, 0.05, 0.085):
    for a in np.linspace(0, 2 * np.pi, 8, endpoint=False):
        for z in (0.01, 0.03):
            p = np.array([[r * np.cos(a), r * np.sin(a), z]])
            free.append(not any(h.contains(p)[0] for h in hulls))
print("cavity points free:", sum(free), "of", len(free))
rr = np.linalg.norm(w[:, :2], axis=1); low = w[w[:, 2] < 0.004]
print("brim radius x/y", np.abs(w[:, 0]).max().round(3), np.abs(w[:, 1]).max().round(3), "rest ring radius", np.linalg.norm(low[:, :2], axis=1).round(2).min(), np.linalg.norm(low[:, :2], axis=1).round(2).max())
for ang, nm in ((0, "+x"), (np.pi, "-x"), (np.pi / 2, "+y")):
    edge = w[(np.abs(np.arctan2(w[:, 1], w[:, 0]) - ang) < 0.15) | (np.abs(np.arctan2(w[:, 1], w[:, 0]) - ang + 2 * np.pi) < 0.15)]
    far = edge[np.linalg.norm(edge[:, :2], axis=1) > np.linalg.norm(edge[:, :2], axis=1).max() - 0.02]
    print("brim edge", nm, "r %.3f z %.3f..%.3f" % (np.linalg.norm(far[:, :2], axis=1).max(), far[:, 2].min(), far[:, 2].max()))

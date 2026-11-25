"""Convert the downloaded source models into the room's render bundle (OBJ, Z up, metres, PNG textures)."""
import os, sys, numpy as np, trimesh
from PIL import Image
RAW = "/home/lingjun/ale/dev/assets_raw"; OUT = "/home/lingjun/ale/dev/room"

def write_obj(path, v, f, uv, n=None):
    with open(path, "w") as h:
        for p in v: h.write("v %.6f %.6f %.6f\n" % tuple(p))
        for t in uv: h.write("vt %.6f %.6f\n" % tuple(t))
        for a, b, c in f + 1: h.write("f %d/%d %d/%d %d/%d\n" % (a, a, b, b, c, c))

def tex(src, dst, size=1024):
    Image.open(src).convert("RGB").resize((size, size), Image.LANCZOS).save(dst)

def sub(g, faces):
    idx = np.unique(g.faces[faces]); remap = -np.ones(len(g.vertices), int); remap[idx] = np.arange(len(idx))
    return g.vertices[idx].copy(), remap[g.faces[faces]], np.asarray(g.visual.uv)[idx].copy()

def piecewise(z, src, dst):
    return np.interp(z, src, dst)

# ---- sofa ------------------------------------------------------------------------------------
s = trimesh.load(f"{RAW}/ph/Sofa_01/Sofa_01.gltf"); g = list(s.geometry.values())[0]
comp = trimesh.graph.connected_components(g.face_adjacency, nodes=np.arange(len(g.faces)), min_len=1)
cush, frame = [], []
for c in comp:
    v = g.vertices[np.unique(g.faces[c])]; lo, hi = v.min(0), v.max(0)
    is_c = 100 <= len(c) <= 104 and abs(lo[1] - 0.22) < 0.02 and hi[1] < 0.40
    centre_leg = hi[1] < 0.18 and abs(0.5 * (lo[0] + hi[0])) < 0.1 and hi[0] - lo[0] < 0.1
    if centre_leg:
        continue                      # four legs: the two middle ones would stand in the way under the sofa
    seat_roll = len(c) == 210 and abs(lo[1] - 0.22) < 0.02 and hi[1] < 0.36
    if seat_roll:
        continue                      # the padded roll along the seat's front edge: the seat is a flat deck here
    (cush if is_c else frame).append(c)
print("cushions", len(cush), "frame comps", len(frame))
P = dict(W=1.14)
fv, ff, fuv = sub(g, np.concatenate(frame))
# asset (x lateral, y up, z front+) -> world (x depth from the front face, y lateral, z up)
def to_world(v, xs, ys, zmap):
    out = np.empty_like(v)
    out[:, 0] = (0.29 - v[:, 2]) * xs          # front face (asset z = 0.29) at x = 0
    out[:, 1] = -v[:, 0] * ys
    out[:, 2] = zmap(v[:, 1])
    return out
src_h = [0.0, 0.11, 0.23, 0.80]; dst_h = [0.0, 0.06, 0.25, 0.82]
fw = to_world(fv, 0.50 / 0.458 * 0.60 / 0.65, P["W"] / 1.56, lambda y: piecewise(y, src_h, dst_h))
write_obj(f"{OUT}/sofa_frame.obj", fw, ff, fuv)
print("frame bounds", fw.min(0).round(3), fw.max(0).round(3))
for name, c in zip(("a", "b"), cush):
    v, f, uv = sub(g, c); lo, hi = v.min(0), v.max(0); ctr = 0.5 * (lo + hi); ext = hi - lo
    w = np.empty_like(v)
    w[:, 0] = -(v[:, 2] - ctr[2]) / ext[2]; w[:, 1] = -(v[:, 0] - ctr[0]) / ext[0]; w[:, 2] = (v[:, 1] - ctr[1]) / ext[1]
    write_obj(f"{OUT}/cushion_{name}.obj", w, f, uv)      # unit box, scaled in the model
    print("cushion", name, "asset x", lo[0].round(2), hi[0].round(2))
d = f"{RAW}/ph/Sofa_01/textures"; print(os.listdir(d))
tex([os.path.join(d, x) for x in os.listdir(d) if "diff" in x.lower()][0], f"{OUT}/sofa.png")
# ---- commode ---------------------------------------------------------------------------------
s = trimesh.load(f"{RAW}/ph/GothicCommode_01/GothicCommode_01.gltf")
for n, gm in s.geometry.items():
    T = s.graph.get(s.graph.geometry_nodes[n][0])[0]
    v = trimesh.transform_points(gm.vertices, T); lo = v.min(0)
    w = np.stack([(0.29 - v[:, 2]) * 0.79, -v[:, 0] * 0.5, v[:, 1] * 0.6], axis=1)   # front at x = 0
    name = {"Circle.003": "commode_body", "Circle.005": "commode_drawer_lower", "Circle.006": "commode_drawer_mid", "Circle.007": "commode_drawer_upper"}[n]
    write_obj(f"{OUT}/{name}.obj", w, gm.faces, np.asarray(gm.visual.uv)); print(name, w.min(0).round(3), w.max(0).round(3))
d = f"{RAW}/ph/GothicCommode_01/textures"; print(os.listdir(d))
tex([os.path.join(d, x) for x in os.listdir(d) if "diff" in x.lower()][0], f"{OUT}/commode.png")
# ---- remote ----------------------------------------------------------------------------------
r = trimesh.load(os.path.expanduser("~/.objaverse/hf-objaverse-v1/glbs/000-064/212d33f6b5904bed965a87b018325b26.glb"))
gm = list(r.geometry.values())[0]; T = r.graph.get(r.graph.geometry_nodes[list(r.geometry)[0]][0])[0]
v = trimesh.transform_points(gm.vertices, T); ctr = 0.5 * (v.min(0) + v.max(0)); v = v - ctr
print("remote ext", (v.max(0) - v.min(0)).round(4))
order = np.argsort(-(v.max(0) - v.min(0)))            # long, wide, thin -> x, y, z
w = v[:, order]
w = w * np.array([0.172 / 0.2038, 0.044 / 0.0519, 0.018 / 0.0192])
write_obj(f"{OUT}/remote.obj", w, gm.faces, np.asarray(gm.visual.uv))
gm.visual.material.baseColorTexture.convert("RGB").resize((1024, 1024)).save(f"{OUT}/remote.png")
# ---- hanger: a flat wooden bar with rounded edges (the wire hook and clips are primitives) -----
L, T_, H = 0.22, 0.007, 0.016
n, k = 48, 16
vs, uvs, fs = [], [], []
for i in range(n + 1):
    u = i / n; x = -L + 2 * L * u
    taper = 1.0 - 0.35 * abs(2 * u - 1) ** 6            # the ends are rounded off
    for j in range(k):
        a = 2 * np.pi * j / k
        cy, cz = np.cos(a), np.sin(a)
        e = 0.45                                         # superellipse: a board with soft edges
        vs.append((x, T_ * np.sign(cy) * abs(cy) ** e, H * taper * np.sign(cz) * abs(cz) ** e))
        uvs.append((u * 3.0, j / k))
for i in range(n):
    for j in range(k):
        a, b = i * k + j, i * k + (j + 1) % k; c, d = a + k, b + k
        fs += [(a, c, b), (b, c, d)]
c0 = len(vs); vs.append((-L, 0, 0)); uvs.append((0, 0.5)); c1 = len(vs); vs.append((L, 0, 0)); uvs.append((3, 0.5))
for j in range(k):
    fs += [(c0, j, (j + 1) % k), (c1, n * k + (j + 1) % k, n * k + j)]
write_obj(f"{OUT}/hanger.obj", np.array(vs), np.array(fs), np.array(uvs))
TX = f"{RAW}/tex"
for src, dst, size in (("laminate_floor_02", "floor_a", 1024), ("wood_floor", "floor_b", 1024), ("painted_plaster_wall", "wall", 1024), ("oak_veneer_01", "oak", 512)):
    tex(f"{TX}/{src}.jpg", f"{OUT}/{dst}.png", size)
for f in os.listdir(OUT):
    if f not in ("sofa_frame.obj", "commode_body.obj", "commode_drawer_lower.obj", "commode_drawer_mid.obj", "commode_drawer_upper.obj", "remote.obj", "hanger.obj", "sofa.png", "commode.png", "remote.png", "floor_a.png", "floor_b.png", "wall.png", "oak.png"):
        os.remove(os.path.join(OUT, f))
print("done", sorted(os.listdir(OUT)))

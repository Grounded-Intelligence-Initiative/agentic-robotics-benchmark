"""The seat pad's drawn model: a closed box with rounded edges (unit size, scaled in the model), and
a fabric texture cut from the sofa's own cushion area."""
import numpy as np, trimesh
from PIL import Image
RAW = "/home/lingjun/ale/dev/assets_raw"; OUT = "/home/lingjun/ale/agentic-robotics-benchmark/tasks/arm/tv_remote_search_and_retrieval/tests/room"
size = np.array([0.46, 0.49, 0.065]); r = 0.016; n = 14
half = 0.5 * size; inner = half - r
verts, uvs, faces = [], [], []
def face(axis, sign):
    a, b = [i for i in range(3) if i != axis]
    base = len(verts)
    for i in range(n + 1):
        for j in range(n + 1):
            p = np.zeros(3); p[axis] = sign * half[axis]
            p[a] = -half[a] + size[a] * i / n; p[b] = -half[b] + size[b] * j / n
            c = np.clip(p, -inner, inner); d = p - c
            q = c + (r * d / np.linalg.norm(d) if np.linalg.norm(d) > 1e-9 else 0)
            verts.append(q / size)                      # unit box
            uvs.append((p[a] * 3.0 + 0.5, p[b] * 3.0 + 0.5))
    for i in range(n):
        for j in range(n):
            v0 = base + i * (n + 1) + j; v1 = v0 + 1; v2 = v0 + n + 1; v3 = v2 + 1
            tri = [(v0, v2, v1), (v1, v2, v3)]
            flip = (sign > 0) != (axis == 1)
            faces.extend([(t[0], t[2], t[1]) for t in tri] if flip else tri)
for axis in range(3):
    for sign in (-1, 1):
        face(axis, sign)
v = np.array(verts); f = np.array(faces)
m = trimesh.Trimesh(v * size, f, process=False)
if m.volume < 0: f = f[:, ::-1]
with open(f"{OUT}/cushion.obj", "w") as h:
    for p in v: h.write("v %.6f %.6f %.6f\n" % tuple(p))
    for t in uvs: h.write("vt %.5f %.5f\n" % tuple(t))
    for a, b, c in f + 1: h.write("f %d/%d %d/%d %d/%d\n" % (a, a, b, b, c, c))
# fabric: the middle of the cushions' area of the sofa texture
s = trimesh.load(f"{RAW}/ph/Sofa_01/Sofa_01.gltf"); g = list(s.geometry.values())[0]
comp = trimesh.graph.connected_components(g.face_adjacency, nodes=np.arange(len(g.faces)), min_len=1)
uv = np.asarray(g.visual.uv)
for c in comp:
    vv = g.vertices[np.unique(g.faces[c])]
    if 100 <= len(c) <= 104 and abs(vv[:, 1].min() - 0.22) < 0.02 and vv[:, 1].max() < 0.40:
        top = [i for i in np.unique(g.faces[c]) if g.vertices[i][1] > vv[:, 1].max() - 0.03]
        box = uv[top]; break
u0, v0 = np.percentile(box, 25, axis=0); u1, v1 = np.percentile(box, 75, axis=0)
img = Image.open(f"{OUT}/sofa.png"); W, H = img.size
crop = img.crop((int(u0 * W), int((1 - v1) * H), int(u1 * W), int((1 - v0) * H))).resize((256, 256), Image.LANCZOS)
# make it tile: mirror into a 512 square
tile = Image.new("RGB", (512, 512)); tile.paste(crop, (0, 0)); tile.paste(crop.transpose(Image.FLIP_LEFT_RIGHT), (256, 0))
tile.paste(tile.crop((0, 0, 512, 256)).transpose(Image.FLIP_TOP_BOTTOM), (0, 256)); tile.save(f"{OUT}/fabric.png")
print("cushion verts", len(v), "watertight", trimesh.Trimesh(v * size, f).is_watertight, "fabric mean", np.asarray(crop).mean(axis=(0, 1)).round(0), "uv box", u0, v0, u1, v1)

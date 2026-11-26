"""The storage basket on the chest and the book that lies in it: drawn meshes, textures, and the
basket's collision shapes (a convex decomposition, CoACD, of the drawn mesh, so that it is hollow).

    python3 tests/review/build_box.py <wicker_basket_01.gltf> <book_encyclopedia_set_01.gltf>

Sources: Poly Haven "Wicker Basket 01" (CC0), scaled 1.1 in plan and 0.8 in height, its long side
along y; Poly Haven "Book Encyclopedia Set 01" (CC0), one book of the set (book08) at its own
size, lying flat.
"""
import os
import sys

import coacd
import numpy as np
import trimesh
from PIL import Image

OUT = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), "environment", "room")
BASKET_SCALE = (1.1, 1.1, 0.8)
BOOK = "book08"        # 2 x POUCH_HALF
BASKET_PARTS = 40


def write_obj(path, v, uv, f):
    with open(path, "w") as handle:
        for p in v:
            handle.write("v %.6f %.6f %.6f\n" % tuple(p))
        for t in uv:
            handle.write("vt %.6f %.6f\n" % tuple(t))
        for a, b, c in f + 1:
            handle.write("f %d/%d %d/%d %d/%d\n" % (a, a, b, b, c, c))


def one(path):
    scene = trimesh.load(path)
    node = scene.graph.nodes_geometry[0]
    transform, key = scene.graph[node]
    mesh = scene.geometry[key]
    return trimesh.transform_points(mesh.vertices, transform), mesh


def basket(path):
    v, mesh = one(path)
    # glTF is y up, the basket's long side along x: long side to y, up to z (a proper rotation).
    w = np.stack([-v[:, 2], v[:, 0], v[:, 1]], axis=1) * np.array(BASKET_SCALE)
    w = w - np.array([0.5 * (w[:, 0].min() + w[:, 0].max()), 0.5 * (w[:, 1].min() + w[:, 1].max()), w[:, 2].min()])
    write_obj(os.path.join(OUT, "basket.obj"), w, np.asarray(mesh.visual.uv), mesh.faces)
    tex = os.path.join(os.path.dirname(path), "textures")
    diff = [os.path.join(tex, x) for x in os.listdir(tex) if "diff" in x.lower()][0]
    Image.open(diff).convert("RGB").resize((1024, 1024), Image.LANCZOS).save(os.path.join(OUT, "basket.png"))
    parts = coacd.run_coacd(coacd.Mesh(w, mesh.faces), threshold=0.03, max_convex_hull=BASKET_PARTS, preprocess_resolution=80)
    for name in os.listdir(OUT):
        if name.startswith("basket_col_"):
            os.remove(os.path.join(OUT, name))
    hulls = [trimesh.Trimesh(pv, pf).convex_hull for pv, pf in parts]
    for i, hull in enumerate(hulls):
        hull.export(os.path.join(OUT, "basket_col_{:02d}.obj".format(i)))
    print("basket: parts", len(hulls), "bounds", w.min(0).round(3), w.max(0).round(3))
    # The hollow, as the collision shapes have it: the free span along x and y at heights.
    for z in (0.016, 0.02, 0.04, 0.06, 0.08):
        span = []
        for axis in (0, 1):
            free = []
            for t in np.arange(-0.25, 0.25, 0.002):
                p = np.zeros((1, 3))
                p[0, axis], p[0, 2] = t, z
                if not any(h.contains(p)[0] for h in hulls):
                    free.append(t)
            free = np.array(free)
            inner = free[np.abs(free) < 0.5 * np.ptp(w[:, axis]) - 0.004]
            span.append((round(float(inner.min()), 3), round(float(inner.max()), 3)) if len(inner) else None)
        print("  z %.3f free x %s y %s" % (z, span[0], span[1]))
    for z in np.arange(0.0, 0.03, 0.002):
        p = np.array([[0.0, 0.0, z]])
        if not any(h.contains(p)[0] for h in hulls):
            print("  inner floor (collision) at z about %.3f" % z)
            break


def book(path):
    """One book of the set (its cover and its pages), lying flat: spine along y, centred."""
    scene = trimesh.load(path)
    parts = []
    for node in scene.graph.nodes_geometry:
        if "_{}_".format(BOOK) in node:
            transform, key = scene.graph[node]
            parts.append((trimesh.transform_points(scene.geometry[key].vertices, transform), scene.geometry[key]))
    parts.sort(key=lambda p: -len(p[1].faces))                 # the cover first
    points = np.vstack([p[0] for p in parts])
    lo, hi = points.min(axis=0), points.max(axis=0)
    for i, (v, mesh) in enumerate(parts):
        v = v - 0.5 * (lo + hi)
        w = np.stack([v[:, 2], v[:, 1], -v[:, 0]], axis=1)      # thickness to z, height to y (a proper rotation)
        write_obj(os.path.join(OUT, "book_{}.obj".format(i)), w, np.asarray(mesh.visual.uv), mesh.faces)
        tex = os.path.join(os.path.dirname(path), "textures")
        kind = "cover" if i == 0 else "paper"
        diff = [os.path.join(tex, x) for x in os.listdir(tex) if kind + "_diff" in x.lower()][0]
        Image.open(diff).convert("RGB").resize((1024, 1024), Image.LANCZOS).save(os.path.join(OUT, "book_{}.png".format(i)))
        print("book", i, len(mesh.faces), "triangles", w.min(0).round(4), w.max(0).round(4))


if __name__ == "__main__":
    basket(sys.argv[1])
    book(sys.argv[2])

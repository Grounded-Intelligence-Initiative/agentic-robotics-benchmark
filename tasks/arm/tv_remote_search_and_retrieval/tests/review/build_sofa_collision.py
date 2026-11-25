"""The settee's collision shapes: a convex decomposition (CoACD) of its drawn mesh."""
import os, numpy as np, trimesh, coacd
OUT = "/home/lingjun/ale/agentic-robotics-benchmark/tasks/arm/tv_remote_search_and_retrieval/tests/room"
m = trimesh.load(f"{OUT}/sofa_frame.obj", process=False)
parts = coacd.run_coacd(coacd.Mesh(m.vertices, m.faces), threshold=0.03, max_convex_hull=72, preprocess_resolution=100)
for f in os.listdir(OUT):
    if f.startswith("sofa_col_"): os.remove(os.path.join(OUT, f))
vol = 0
for i, (pv, pf) in enumerate(parts):
    hull = trimesh.Trimesh(pv, pf).convex_hull; vol += hull.volume
    hull.export(f"{OUT}/sofa_col_{i:02d}.obj")
print("parts", len(parts), "hull volume", round(vol, 4))

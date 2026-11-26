# The room's meshes and textures

| Files | Source | Author | Licence |
|---|---|---|---|
| `sofa_frame.obj`, `sofa.png`, `fabric.png` (cut from `sofa.png`) | Poly Haven, "Sofa 01" (https://polyhaven.com/a/Sofa_01) | Poly Haven | CC0 1.0 |
| `commode_body.obj`, `commode_drawer_*.obj`, `commode.png` | Poly Haven, "Gothic Commode 01" (https://polyhaven.com/a/GothicCommode_01) | Poly Haven | CC0 1.0 |
| `hat.obj`, `hat.png`, `hat_col_*.obj` | Poly Haven, "Fishermans Hat" (https://polyhaven.com/a/fishermans_hat) | Poly Haven | CC0 1.0 |
| `sofa_col_*.obj` | convex decomposition (CoACD) of `sofa_frame.obj` | as `sofa_frame.obj` | CC0 1.0 |
| `remote.obj`, `remote.png` | Objaverse 1.0 object `212d33f6b5904bed965a87b018325b26`, "Remote control" (Sketchfab) | craazyy | CC BY 4.0 |
| `phone_0.obj`, `phone_1.obj`, `phone_*.png` | Objaverse 1.0 object `21e953e3d4484aac88ca108da3291b20`, "SmartPhone" (Sketchfab) | shedmon | CC BY 4.0 |
| `glasses_case_0.obj`, `glasses_case_1.obj`, `glasses_case_*.png` | Objaverse 1.0 object `142a10077f1c48d385888c84213b85bc`, "Spectacles case" (Sketchfab) | lib.test | CC BY 4.0 |
| `floor_a.png` | Poly Haven, "Laminate Floor 02" | Poly Haven | CC0 1.0 |
| `floor_b.png` | Poly Haven, "Wood Floor" | Poly Haven | CC0 1.0 |
| `wall.png` | Poly Haven, "Painted Plaster Wall" | Poly Haven | CC0 1.0 |
| `oak.png` | Poly Haven, "Oak Veneer 01" | Poly Haven | CC0 1.0 |
| `hanger.obj`, `cushion.obj` | made for this task | task authors | CC0 1.0 |

The storage box and the pouch are plain boxes drawn by the simulator (no files).

Changes made to the sources: the sofa's two seat cushions were taken out of its model (the seat pads are a
mesh of this task's own, covered with the sofa's fabric), its two middle legs and the padded roll along its seat's front edge were removed, and its heights and width were
rescaled; the commode was rescaled; the hat was scaled by 1.25 (the simulator scales the files by 0.68 again: a hat 28 by 30 cm); the remote, the phone and the glasses case were rescaled, and the scanned glasses case was reduced to about 6000 triangles per part (`tests/review/build_clutter.py`); meshes were converted to OBJ
(Z up, metres) and textures resampled to PNG. The conversion script is
`tests/review/build_assets.py` (with `build_cushion.py`, `build_hat.py` and
`build_sofa_collision.py`) in the task's folder of the benchmark repository.

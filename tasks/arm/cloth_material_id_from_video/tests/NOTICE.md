# NOTICE - sources of this task's assets (verify / maintainers only; never shown to the agent)

- **The robot** in the videos and in `carm.py` is the Franka Panda model of the robosuite 1.5.2 package (MIT licence,
  https://github.com/ARISE-Initiative/robosuite), installed unmodified in the task image from PyPI; only its MJCF and
  mesh files are used. The fabric clamp on its flange is this task's own geometry.
- **Everything else is this task's own and synthetic**: the scene (table, rod, floor, wall), the four fabrics, their
  materials, the clip programs, the hidden tests and every texture. The textures (the fabric prints, the table's wood
  grain, the floor tiles) are generated procedurally by `environment/gen/render_clips.py` from fixed seeds; no image,
  mesh, recording or measurement from any other source is used.
- **The videos** are rendered while the task image builds (`environment/Dockerfile`, `environment/gen/`), with MuJoCo
  3.16's native renderer on OSMesa, from the true materials in `environment/gen/fabrics.json` (bind-mounted for that
  step only, so it is in no image layer). The degradations (motion blur from two sub-frame renders, optical blur,
  vignetting, per-camera white balance and gain, sensor noise, H.264 at CRF 23) are applied in the same step.
- **The reference** (`tests/ref/fabrics.json`, `solution/ref/params.json`, `environment/gen/fabrics.json`,
  P's `gt_params.json`) is written by `tests/review/make_ref.py` from the materials listed there and a fixed seed.
  Change a material only there, re-run it, rebuild the image and re-validate.
- **Copies that must stay byte-identical:** `environment/setup/payload/{cphys,ctasks}.py` = `tests/{cphys,ctasks}.py`
  = `environment/gen/{cphys,ctasks}.py`, and `environment/setup/payload/carm.py` = `environment/gen/carm.py`
  (`tests/review/check_copies.py`).

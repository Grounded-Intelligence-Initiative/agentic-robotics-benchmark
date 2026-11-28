# Sources and licences of the cell's assets

Nothing in this task is downloaded from an asset library, and the repository holds no texture or mesh file.

| Asset | Source | Licence |
|---|---|---|
| Franka Panda arm and hand (meshes, kinematics, inertias) | robosuite 1.5.2 (`robosuite/models/assets/robots/panda`, `grippers/panda_gripper.xml`). The image build installs it with `pip install --no-deps robosuite==1.5.2`. Only the model files are read; robosuite's code is never imported. | MIT (robosuite) |
| Simulator and renderer | MuJoCo 3.16.0 (pip wheel) | Apache-2.0 |
| Table wood grain, floor planks, wall plaster, birch grain | Procedural: generated at model build by `toycar_env.py` (`wood_texture`, `planks_texture`, `plaster_texture`) from fixed seeds with numpy value noise. No input image. | Part of this task's code (repository licence) |
| Toy car parts, assembly stand, room walls, table legs, desk lamp | MuJoCo primitives (boxes, cylinders, capsules) defined in `toycar_env.py` | Part of this task's code (repository licence) |
| Picture manuals (each episode's, rendered in the grader and in the practice cell; the examples in `setup/payload/examples/`) | Rendered with MuJoCo from the car's numbers by `tests/toycar_manual.py` (`python3 toycar_manual.py <kit.json> <out_dir>` writes them as PNG files), with the millimetre ruler drawn in numpy | Part of this task's code (repository licence) |

The graded cameras render with MuJoCo's classic OpenGL renderer on Mesa llvmpipe, with shadows and multisampling
off (`cheap_render_settings`). The demo video used a slower render of a separate camera, with a shadow-casting
light that only the demo enables. It is not part of the task.

# Cloth material identification from video

A Franka Panda robot with a flat fabric clamp handles pieces of fabric on a table: it lifts them by a corner and lets
them hang, shakes them, drags them across the table and lets go, lays them over the table's front edge and hangs them
over a rod. The robot plans every garment-handling motion in a cloth simulator, and a plan transfers to the real
fabric only if the simulator's material makes the simulator predict what the real fabric does. You only have **RGB
video** of these interactions. Choose, for each of the three test fabrics **A**, **B** and **C**, the simulator material
the robot should plan with. Your materials are scored by how well the robot does on new manipulation tasks with the
real fabrics when it plans with them.

You have 8 hours, 4 CPU cores, 8 GB of memory (processes that exceed it are killed), no GPU and no network. Python 3.12
is in `/opt/venv` (NumPy, SciPy, OpenCV (headless, with FFmpeg), Pillow, scikit-image, MuJoCo 3.16.0, and the
robosuite 1.5.2 asset files for the robot model); the `ffmpeg` command is installed. MuJoCo offscreen rendering works
out of the box (`MUJOCO_GL=osmesa`, CPU).

## What you get (`/home/user`)

```
data/cameras.json                      three fixed, calibrated cameras (cam0, cam1, cam2)
data/fabrics/<ID>/fabric.json          ID in P (practice), A, B, C: size, simulation grid, mass (kg), number of clips
data/fabrics/<ID>/clipNN/cam0.mp4      RGB video, 640x480, 25 fps, H.264 (also cam1.mp4, cam2.mp4)
data/fabrics/<ID>/clipNN/clip.json     clip kind, fps, number of frames
data/fabrics/<ID>/clipNN/clamp_500hz.npy   commanded clamp pose and jaw state (M, 8), one row per 2 ms physics step
data/fabrics/<ID>/clipNN/arm_q.npy     the arm's 7 joint angles per video frame (N, 7)
data/fabrics/P/gt_params.json          the reference simulator material of the practice fabric P, in the submission format
cphys.py, ctasks.py                    the robot's cloth simulator and the scoring task code the grader plans with (byte-identical)
carm.py                                the robot's visual model (for example to render robot masks)
cio.py                                 I/O helpers (video loading, cameras, projection, writing the submission)
submission/                            write params.json here
```

- **Cameras.** `cameras.json` gives, per camera, the OpenCV pinhole model: pixel ~ K (R_w2c X + t_w2c), no lens
  distortion. World frame: the table top is the plane z = 0 for x <= 0.30 (its front edge is the line x = 0.30), z up,
  metres. The calibration is exact.
- **Videos.** Rendered images of the table, the rod, the robot and the fabric; the robot occludes parts of the scene.
  They are degraded like a real camera: motion blur, optical blur, vignetting, white balance and exposure differences
  between cameras, sensor noise and video compression. Frame k of a clip shows the fabric when the clamp is at row
  20 k of `clamp_500hz.npy` (time k/25 s). Each fabric has its own printed pattern on its upper face (the face up when
  it lies flat).
- **Clips.** Each fabric has 7 clips. At the start of every clip the fabric lies flat and at rest on the table, at a
  pose that is not given (it is in the first frames). Kinds: `lift_hang` (grasp a
  corner, lift, hang, lower partway), `shake` (lift by a short edge, three quick vertical strokes, hold),
  `drag_release` (lift a short edge by 15 mm, drag the fabric slowly along the table and let go while moving),
  `edge_drape` (hang the fabric beyond the table's front edge and lay it back over the edge; the clamp keeps holding),
  `rod_drape` (carry the fabric over the rod, lower it on the far side and let go). The clamp's commanded motion is
  exact; the jaws hold whatever part of the fabric lies between them when they close.
- **The only information about the materials is in the images.** The size and the mass of each fabric are given
  (a ruler and a scale reading); nothing else about the material is.
- **Practice fabric P** has the same kinds of clips. Its `gt_params.json` is the simulator material the maintainers
  calibrated for P by the same criterion your materials are scored by (the best plan transfer on hang, slide and drape
  tasks with the real fabric P); it is not a material that reproduces P's videos exactly. P is not scored.

## The robot's simulator and the real fabrics

The videos show the **real** fabrics. `cphys.py` is the robot's cloth simulator, the one the grader plans every test
in; like any cloth simulator it models a real fabric only approximately, so expect behaviour it does not reproduce
with any material. The tests are executed on the real fabrics, not in `cphys`.

`cphys.py`: MuJoCo 3.16.0, timestep 2 ms, the `discrete`
integrator, the CG solver, elliptic friction cones. A fabric is a 0.30 x 0.15 m rectangle simulated as a MuJoCo
`flexcomp` grid of 21 x 11 vertices (2D shell, every vertex a point mass, the total mass given), with MuJoCo's shell
elasticity, Rayleigh damping and one friction coefficient for all of its contacts; it does not collide with itself.
The table and the rod have zero friction of their own, so the fabric's coefficient decides every contact. The clamp
holds the vertices inside its jaws with stiff equality constraints and never touches anything. `cphys.Sim` builds this
world from **your** material and replays a clamp program, so you can simulate any clip with your own material. The
table, rod, clamp, robot, cameras and clamp programs are the same as in the videos.

## Deliverable

`/home/user/submission/params.json` (`cio.write_params` writes it):

```json
{"fabrics": {
  "A": {"young": 2.0e5, "poisson": 0.3, "thickness": 0.002, "damping": 0.003, "mu": 0.5},
  "B": {...}, "C": {...}}}
```

- The five values are MuJoCo's flex material parameters exactly as `cphys.scene_xml` uses them: `young` (Young's
  modulus, Pa), `poisson` (Poisson's ratio), `thickness` (the shell thickness that, with `young` and `poisson`, sets the
  stretch, shear and bending stiffness; m), `damping` (Rayleigh damping, s) and `mu` (sliding friction). Different
  values that behave the same in `cphys` are equally good: only the robot's results on the real fabric are scored.
- Rules: numbers only; each value is clamped to its admissible range (`cphys.RANGES`). A fabric with a missing or
  non-numeric value scores 0; a missing, unreadable or non-conforming file, a symlink or a special file scores 0.

## Scoring

For each test fabric the grader runs a fixed set of hidden tests, two of each family below (poses, corners, directions,
speeds and targets are hidden). Every test starts with the fabric flat at rest at a pose the robot knows; the robot
chooses one control value by planning with `ctasks.plan` in `cphys` made of **your** material, so that the predicted
outcome hits the test's target, and then executes that value once on the **real** fabric (`ctasks.run_test`).

| Family | What the robot does | Outcome that must hit the target | Tolerance |
|---|---|---|---|
| hang | grasps a corner, lifts and holds the fabric, then lowers the clamp to the planned height | height of the fabric's lowest point above the table | 3 mm |
| slide | lifts a short edge 15 mm, drags the fabric along the table reaching the planned speed (0.2 to 1.4 m/s), lets go while moving | how far the fabric's leading point ends up beyond the release point | 6 mm |
| drape | hangs the fabric beyond the table's front edge and lays it back over the edge until the clamp (still holding) reaches the planned position on the table | height of the lowest point of the overhanging part | 5 mm |

A test scores q = clip(2 - error / tolerance, 0, 1): full marks within the tolerance, nothing at twice the tolerance.
A fabric's score is the mean over the three families of the family's mean test score; the metric
`material_transfer_score` is the mean over A, B and C (higher is better; because no simulator material reproduces a
real fabric exactly, not every test can be passed). The mass, gravity,
the clamp programs and the physics settings always come from the grader; nothing you write is executed.

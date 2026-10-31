# Build the simulation of an aerial workcell

A workcell where two flying robots with arms put objects into a lidded bin has to be rebuilt in simulation, in
NVIDIA Isaac Sim, so that it can be used to develop the robots' programs. There is no model of it: you have
photographs of the cell, a short site brief and an offline asset library. Build the cell as a USD stage that
behaves like the real one: the right things in the right places, the right masses, the bin's lid and button working
as the brief describes, the robots as they are, sensors included.

You have **8 hours** of wall time for the whole job. Have a usable scene in `/home/user/submission/` long before the
time is up, and improve it from there.

## What you have

| path | what |
|---|---|
| `~/given/site_brief.md` | the site brief: what the cell holds, the world frame, masses, how the bin works |
| `~/given/photos/` | 20 photographs of the cell (1600 × 1200), `photos.json` with their captions; two of them were taken by the robots' own cameras |
| `~/given/interface.md` | how the grader reads your scene: stage settings, the prim paths it looks for, the OmniGraph rules |
| `~/given/check_structure.py` | checks a scene against `interface.md` (paths, files, graph nodes); it scores nothing |
| `/opt/assets/isaac/` | a part of NVIDIA's Isaac Sim asset library, offline (robots, objects, a warehouse); not everything in it is in the cell |

Software: Isaac Sim 6.2 and Isaac Lab 3.0 with Python 3.12 in `/opt/venv` (run Python as `/opt/venv/bin/python3`;
it is not on your `PATH`), PyTorch, OpenCV, one NVIDIA GPU with RTX rendering, so you can render your own scene and
compare it with the photographs. Isaac Sim's settings already point its asset root at `/opt/assets/isaac`. There is
no network.

## What you deliver

`/home/user/submission/scene.usda` (or `scene.usd`): the cell as one USD stage, following `interface.md`, together
with every file it needs, referenced by relative path, plus whatever it references from `/opt/assets/isaac/`.
Everything the cell does must be in the stage itself: geometry, physics, and OmniGraph action graphs for logic. The
grader runs none of your code.

## How it is graded

After your session ends, every process of yours is stopped and the submission is copied. The grader checks the
scene against `interface.md` (as `check_structure.py` does; a scene that fails it scores 0), then opens it in its own
Isaac Sim process, steps it at 120 Hz and measures it. Each check scores between 0 and 1 by how close the measured
value is to the real cell's; `acceptance_score` is their weighted mean:

- **Layout (0.30)**: where the tables, the bin, the button and the robots are and how big they are; which objects
  stand on the tables and where (an object counts when your scene uses the library's model of it).
- **The bin (0.40)**: the grader pushes the lid's handle and the button from outside, in fresh runs of your scene,
  and measures the lid: at rest and under a small pull, the force that starts to lift it, falling back from 20°,
  the strut's carry from 35° to the stop, staying there, the slow close after a pull back from the stop, the button's
  travel, the motor's opening, how long it holds the lid open, closing again, a second press, and a block dropped
  into the open bin.
- **Physics and robots (0.30)**: nothing moves when the scene starts; each object's mass; each robot's total mass,
  its joints, which airframe and which arm it is; each robot's camera: where it sits on the robot, where it looks,
  its field of view.

Measurements come from the simulation itself (body poses, masses, joint motion), never from anything the scene
reports about itself.

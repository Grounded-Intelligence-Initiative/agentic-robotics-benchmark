# Design a quadruped that can cross a stepped, rubble-strewn basement walkway

You are the hardware engineer of a small robotics integrator. A client wants a small four-legged inspection robot
for walkway W-3 of a pump hall basement: steps up, a ramp down, loose rubble and low overhead pipes
(`~/qm24/site_brief.md`). The robot is built only from your company's QM-24 parts catalog: body modules, joint
actuators, link segments and feet, each with a price, a mass and ratings (`~/qm24/catalog.json`). The client sets
the budget, the mass limit and the footprint.

The controller is fixed. Every QM-24 robot runs the company's standard gait (`~/qm24/qm_gait.py`, given to you in
full). It is a perceptive static crawl that sizes its steps, swing height and period from the robot's geometry.
You cannot change it. Your job is the morphology: choose the parts and how they are put together so that this gait
gets the robot across the walkway, and write the robot's description file.

## What you have (in your home)

- `~/qm24/site_brief.md`: the walkway as surveyed, what counts as progress and as a fall, and how the robot is
  simulated. Read it first.
- `~/qm24/catalog.json`: the parts and the client's limits (budget, mass with the pod, footprint).
- `~/qm24/qm_design.py`: the robot description dialect, a strict subset of MJCF (its docstring defines it), and the
  build check. `~/qm24/check_design.py <file>` runs that check on a file and prints the design's price, mass,
  footprint, reach, clearance and the gait sizes. It does kinematics and bookkeeping only.
- `~/qm24/qm_gait.py`: the standard gait, exactly as the grader runs it.
- `~/qm24/example_robot.xml`: a buildable design that shows the dialect. It is not a good design.
- `~/practice/run_practice.py`: runs one design once over **a practice section of the real walkway** with the
  standard gait. It prints the progress, how and when the run ended, and the path of a side-view video of the run.
  **The whole session has 4 practice runs** (`/run/qmcell/practice_status.json`). A design that fails the build
  check costs no run. There is no other simulator on this machine: MuJoCo is not available to you.
- Python: `/opt/venv/bin/python3` (Python 3.12) with numpy, scipy, opencv (headless) and pillow. Nothing else can be
  installed: there is no network.
- 4 CPUs, 8 GB of memory, no GPU, and **3 hours** in total.

## What you submit

`/home/user/submission/robot.xml`: your robot in the QM-24 dialect, at most 64 KB. Only that file is graded.
Everything about the robot comes from it and from the catalog: its parts and how they are put together, plus the
three declared parameters the gait reads (`stance_height`, `knee_front`, `knee_rear`). Masses, inertias, friction,
servo gains and every other physical parameter come from the catalog and the simulator, never from your file.
Leave a buildable design in place early and improve it.

## How it is graded

The grader first runs the same build check as `check_design.py`. A design that is not buildable from the catalog
(an unknown part, a wrong position or axis, a missing leg, anything outside the dialect) or that breaks a limit
(price, mass with the pod, footprint, stance height, body clearance) scores **0**.

A buildable design is rebuilt from the catalog and walked with the standard gait over **100 hidden sections** of the
walkway, drawn from the family in the site brief. Each section scores the progress made (1.0 at the goal line,
partial credit otherwise; a fall keeps the progress made before it, a simulation fault scores 0). The graded quantity
is the **mean progress over the 100 sections**. It is compared with what a reference design reaches.

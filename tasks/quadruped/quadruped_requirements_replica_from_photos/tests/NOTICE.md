# NOTICE (grader-side; never staged into the agent's home)

## The reference unit

The 12-actuator, belt-reduced quadruped of this task is a hidden VARIANT whose architecture follows the ODRI Solo12
open quadruped of the Open Dynamic Robot Initiative (New York University and Max Planck Gesellschaft):

- `open-dynamic-robot-initiative/robot_properties_solo`, commit `2201f454f4caffa0cad307fa8cbd1d64dd844008`
  (the robot description: kinematic layout, joint arrangement and link conventions), BSD-3-Clause,
  "Copyright (c) 2019, New York University and Max Planck Gesellschaft." Full licence text:
  `LICENSES/robot_properties_solo.BSD-3-Clause.txt`.
- `open-dynamic-robot-initiative/open_robot_actuator_hardware`, commit `66af1522b4fba0ec4a1d7790e66f5e4652208d30`
  (the 12-DOF quadruped's mechanics and the belt-reduced 3-DOF leg), BSD-3-Clause, "Copyright (c) 2019, New York
  University and Max Planck Gesellschaft." Full licence text: `LICENSES/open_robot_actuator_hardware.BSD-3-Clause.txt`.

**Modification statement.** No file of either repository (mesh, CAD, URDF, code) is copied into this task. The unit is
generated from numbers by the task's own ground-truth generator (maintainer side, not published with the task):
parametric printed shells, housings, rings, limiters and modules built from drawn values. The architecture (a trunk
with four identical 3-DOF legs, hip_roll axes along the trunk, the hip_pitch module in the shoulder shell, the knee
module in the thigh shell with a belt to the knee) is the source's; every dimension, stop, mass, housing class, the
spacer rings on two legs, the payload and the power system are drawn per instance and differ from the source's
published values by construction (the generator's laws keep every graded length and stop away from them). The joint and
leg names are the task's own (`hip_roll`, `hip_pitch`, `knee`; `fl`, `fr`, `rl`, `rr`). The source's published values
are used only on the maintainer side, for the leak scans and the recall dummy of the local checks.

Per the owner's rule the givens, the instruction and the public card never name the robot, its project, its
institutions or its suppliers; this file and the maintainer's own records carry the attribution.

## Vendors, documents and photos

The vendors (Wrenfield Drives, Quillon Motion, Ardent Cells, Torvik Controls), their part ids and every datasheet,
erratum, quote and lot-report number are fictional. `gt/catalogue.json` is the generator-side truth of the pages; no
agent-visible code carries these numbers. The pages and reports are synthesised from numbers by the task's
`docs_synth_q` page renderers (matplotlib, DejaVu fonts).

The photos were rendered with Blender Cycles in a lab room built from the Archviz Laboratory Pack [2.0] (Unity Asset
Store, licensed by the maintainer; the pack is not redistributed, only photos rendered with it). Sky: 'Schadowplatz' by
Greg Zaal, Poly Haven, CC0 (polyhaven.com/a/schadowplatz). Bench top: 'Oak Veneer 01' by Jenelle van Heerden, Poly
Haven, CC0 (polyhaven.com/a/oak_veneer_01). The scale references, the kitchen scale, the pod and the service stand are
generated from numbers by the task's bench scene.

The verify stage carries byte copies of the benchmark's `shared/robotics_grader` kit and of two stdlib helpers of the
benchmark's arm tasks (the verify-start lockdown and the stage lock), in `stage_lock/`.

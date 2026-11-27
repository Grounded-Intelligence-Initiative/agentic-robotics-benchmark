# Sources of humanoid_robot_description_from_calibration_records

This note says where the task's robot and its other material come from. The agent never sees this folder; everything
the agent sees uses generic names (the robot, the maker, the actuator vendor are not named there).

## The robot

The unit of this task is one build of Asimov 1, an open-source 25-actuator humanoid robot by Menlo Research, as
published in the public repository `menloresearch/asimov-1` on GitHub (commit
`55fca52b9f82906c00cc70738ca8a38e4be3caaa`). The repository states its licences in `HARDWARE-LICENSE.txt` (the
mechanical design) and `SOFTWARE-LICENSE.txt` (the software); both files are given to the agent with the export. The
design is used as the authors published it: the hidden unit is the authors' geometry with states of a real kind (the
zeros stored on the days of its calibration, the bars' lengths as crated, the masses with its shoes and hands, the
inertial unit as mounted), no redesign and no stand-in for any part.

## What the agent is given, and how it was made

| Given | Where (agent's home) | From |
|---|---|---|
| The maker's repository | `given/repo/` | a source export of the commit above without its history, names through one rename map, lettering on two parts and on the link meshes removed, drawings rasterised |
| The CAD export | `given/cad/` | the authors' parts as meshes in their part frames, with occurrence and bolted-pair tables |
| The maker's documentation | `given/docs/` | the robot's and the SDK's documentation pages, URLs removed, names through the rename map |
| The calibration procedure | `given/procedure/` | the maker's calibration procedure in words, and figures rendered from the given meshes |
| The actuator vendor's sheet | `given/vendor/` | the vendor's tables of the five actuator models, renamed type A to E |
| The lab's notes and the unit's dossier | `given/lab/`, `given/dossier/` | written for this task (the lab's stack and the unit's three days are the task's hidden world) |
| The contract | `given/contract/` | written for this task; `gate.py` is the grader's own gate (tests/rd04/gate.py), byte for byte |

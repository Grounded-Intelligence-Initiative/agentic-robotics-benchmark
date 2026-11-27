# The machine files of one humanoid unit, from its calibration records

You are the modelling engineer of a robotics lab. A partner's technician built one unit of a 25-actuator humanoid
robot from the maker's kit, calibrated it over two days, locked it up on a third day and crated it. The controls team
needs this unit's machine files before it is unpacked: the maker's model file corrected to the unit as crated, and
the unit's calibration. Nothing can be run on the unit: there is no other unit, no simulator of it and no checker
that knows it. What you have is what the lab has: the technician's records of the unit, the maker's published design
and documents, and the lab's own notes.

## What you are given

Everything is in `/home/user/given/` (read-only):

- `dossier/`: the unit's records, day by day: what was done on each day, in which order, with which tools, and the
  readings, weighings and photographs.
- `lab/`: the lab's notes on its control stack: how a drive's reading becomes a joint angle, the limits and latches
  the stack applies, and how a home table is filled for a partner's unit.
- `procedure/`: the maker's calibration procedure, joint by joint, in words, with figures of the parts involved.
- `repo/`: the maker's design repository (mechanical CAD as STEP and STL files, drawings, electrical files, the
  simulation model `sim-model/robot_25dof/xml/robot_25dof.xml` with its meshes).
- `cad/`: the maker's CAD as one mesh per product in its own frame, with tables of every occurrence (placement, mass,
  centroid) and of the bolted pairs, and a README with the frame and the units.
- `docs/`: the maker's documentation pages for this robot and its SDK.
- `vendor/`: the actuator vendor's datasheet.
- `contract/`: `acceptance.txt` (every rule and every number of the grading), `check_submission.py` and `gate.py`
  (the format checker; it is the grader's own gate and knows nothing about the unit), and the calibration file's
  schema.

The robot, its maker and the actuator vendor are not named in these files; the names they use are consistent across
all of them.

## What you deliver

Two files in `/home/user/submission/`:

- `unit.xml`: the given model file (`given/repo/sim-model/robot_25dof/xml/robot_25dof.xml`) corrected to this unit as
  it was crated. Its bodies, body frames, joints, joint axes and joint references stay those of the given file; what
  may and must change, and which elements are allowed, is in `contract/acceptance.txt`. It names a site `imu` (the
  inertial unit's axes) and two box sites `left_sole` and `right_sole` (the contact planes of the feet).
- `calibration.yaml`: the unit's home table (for each of the 25 drives, in firmware slot order, the sense and the
  model angle at the drive's stored zero) and, for each foot, the map from the two ankle motors' coordinates to the
  foot's pitch and roll, as the schema in `contract/` and `acceptance.txt` define them.

Run `python3 /home/user/given/contract/check_submission.py` on your folder before you finish: a submission that does
not pass the gate scores 0. The files must be final when your session ends; nothing in the submission folder may be
written after that. Your session may use 8 processors and 16 GB of memory; tools that count processors or memory
report the host's.

Your session is stopped after 6 hours of wall-clock time from its start. A session stopped by that limit counts as a
failed attempt and scores nothing, whatever the submission folder holds by then. Note the start time with `date` when
you begin, keep a complete submission in the folder from early on, and end the session yourself (stop working and say
that you are done) at least 15 minutes before the limit.

## How it is judged

The grader runs nothing you submit; it reads the two files as data. It compares your files with the unit as crated:

- at samples of the bus that you are not given (the 25 drive readings and the inertial unit's gravity vector) it
  computes, through your calibration and your model, where the feet are, which way the soles face, the trunk's
  attitude, the centre of mass over the feet, the soles' contact planes and footprints, and the elbows;
- it plans standing postures on your files (the soles flat on a level plate), sends the readings your files give
  for them to the real unit, and scores how the unit actually stands: how flat the soles land, where its weight
  falls and how upright it is.

Each error earns a score on a logarithmic band between a full end and a zero end; the score is the gate times the
weighted sample scores times the posture scores, from 0 to 1. `contract/acceptance.txt` prints every family, weight,
band and rule.

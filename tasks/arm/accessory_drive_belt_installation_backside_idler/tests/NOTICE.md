# NOTICE (grader-side; never staged into the agent's home)

The robots are the MuJoCo Menagerie `franka_emika_panda` model (Apache-2.0, its LICENSE ships inside the folder),
fetched by `environment/Dockerfile` at commit f3b6013629c7b94f8ee93fd977b161fa8350f9bc and pinned file by file in
`environment/panda_assets.sha256`. At build time `environment/obj2stl.py` turns the model's visual OBJ meshes into
binary STL (same geometry) and rewrites the mesh references; the result is checked file by file against
`environment/panda_converted.sha256` (these three files and the Dockerfile's fetch step are byte-identical to
`tasks/arm/pump_skid_startup_and_operation/environment/` in the benchmark repository at commit
b7b55acb98cfbda73e4b19fe7d6bbcc09b34763c). The cell builds both arms from that folder with MjSpec at cell start
(`tests/beltcell/physics.py`: torque motors on the seven joints, the force/torque sensor and the wrist camera on each
hand, the grip weld and the lever pins). The agent gets the converted folder in `~/given/panda/`.

**Shared station.** The station is shared with `tasks/arm/accessory_drive_belt_installation` (benchmark pull request
#77, the same contributor; its cell lineage 0.2.x, this package's cell belt-station/0.3.0): the arm controller, the
cameras and sensor model, the grip, safety and damage rules, the wire and the client, the belt chain model, the
tensioner mechanism type, the run-in, the verify kit (`cellkit/`, `robotics_grader/`, `test.sh`, `verify.py`'s
structure) and most of the agent documents' station text. Byte-identical to that package: `beltcell/{control,cameras,
rules,client}.py`, all of `cellkit/` and `robotics_grader/`, the image build files (`environment/Dockerfile`, `requirements.txt`, the
Panda pins) but for comments, and the honest program's `hk.py`.

**New in this task (drive B).**
- `beltcell/drive.py`: the drive B drawing (C, ALT, the backside idler I, WP and the tensioner on the left; five
  members with two back wraps, a 1.21 m belt), the smooth flanged idler pulley type, the tensioner opening clockwise,
  the idler's acceptance sector and its check against the element pitch.
- `beltcell/episode.py`: the clockwise run-in (crank_sense) and the idler I as a hard term of g.
- `beltcell/layout.py`: the idler's bearing friction as a hidden quantity; `beltcell/physics.py`: the peg right of the
  drive; `beltcell/server.py`, `verify.py`: the drive B texts and describe() keys.
- The agent documents' drive, routing, tensioner and belt sections; the survey photographs and describe_example.json
  regenerated on drive B.
- The honest program's sequence (`solution/honest/hseq.py`; `hrig.py` and `heyes.py` generalised to the route, the
  idler and the arm on the tensioner's side), the hidden set and the full_at evaluation, the probes' records.

**Origins of the shared station.** The engine's accessory drive (pulley diameters and spacings of a generic compact
engine, no maker's design or part number), the poly-V belt chain, the automatic tensioner with its service lever,
pointer and window marks, the peg, the grip rule, the run-in and the acceptance were written for the shared station.
The arm controller (`control.py`: the DLS IK and the joint path) and the camera rig (`cameras.py`) are adapted from
`tasks/arm/shoe_lacing_and_boxing/tests/shoecell/{control,cameras}.py` and
`tasks/arm/pump_skid_startup_and_operation/tests/skidcell/{control,cameras}.py` (same commit); the torque-level
Cartesian impedance in `control.py` was written for an unpublished intermediate cell by the same contributor. The
verify kit (`cellkit/`), `test.sh`, `environment/setup/setup.sh` and `verify.py`'s structure are adapted from those two
tasks' `tests/cellkit/`, `tests/test.sh` and `tests/verify.py` (`cellkit/lockdown.py` is identical to theirs);
`cellkit/wire.py` (a bounded request queue, blank lines, monotonic deadlines, the constant-reply guard) and
`cellkit/runner.py` (the program's oom_score_adj, the file-size limit, `run.sh`) were changed for the shared station.

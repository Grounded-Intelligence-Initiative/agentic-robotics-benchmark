# Rebuild a block tower from its own loose blocks

You program a robot cell. A Franka Panda faces a tower of wooden blocks standing on a turntable: 18 levels of three
blocks, each level turned 90 degrees against the one below. Write the cell's program: it takes blocks out of the
lower part of the tower and stacks them on the tower's top, building new levels in the same alternating pattern,
without the tower ever collapsing.

**Deliverable.** `/home/user/submission/play.py` defining `solve(cell)` (helper modules beside it are fine). The
grader runs it once per hidden tower; everything the program does goes through the `cell` object it receives.

**What is given** (in your home):

* `~/given/cell/cell_manual.md`: the cell, the tower and what is known about it, the robot's tools (gripper, a
  deployable pusher rod, a wrist force/torque sensor, the turntable, a staging post, three RGB-D cameras), the rules
  and exactly how a tower is scored. Read it first.
* `~/given/cell/cell_api.md`: the program interface, the budgets and exactly how the grader runs your program.
* `~/given/cell/cell_config.json`: every coordinate and constant; `~/given/lib/towercell/client.py`: the client
  library (low-level calls only: tool-pose and joint goals, the gripper, the pusher, the turntable, inverse kinematics,
  the sensors and cameras); `~/given/robot/panda/`: the robot model.
* `~/example/play.py`: a minimal program (it scores 0).
* `~/commission/run_tower.py`: runs your submission on the commissioning service, the same cell on six practice towers
  (different from the graded ones), the way the grader runs it (10 runs in this session, 2 at a time). It reports no
  score and no hidden state. A full practice episode takes about 20-40 minutes of wall time; run them in the
  background.
* `~/bin/session_time.sh`: how much of your 4-hour session is left.

**The tower is not exact, and it does not tell you which blocks are loose.** Within a level some blocks carry the
levels above and are clamped by their weight; the others are a fraction of a millimetre shorter and loose. Which ones
is hidden and differs per tower; so do every block's friction, mass and small placement offsets and the tower's
placement on the turntable (within the ranges of the manual). There is no block-pose call: find the blocks with the
cameras and by touch. Pushing a clamped block drags the levels above it; pulling a block that still presses on the
level above drags that level.

**Scoring.** Per tower, 2 400 s of simulated time: a removed block counts when it came from a legal level (below the
second-highest complete level when it was removed) and ends at rest in a slot on top (manual section 3). If any block
of the tower ever falls more than 5 mm or tilts more than 5 degrees, the tower has collapsed and scores 0; otherwise
its score is the counted blocks / 30. Your result is the mean over four hidden towers, compared with what a reference
program scored on the same towers (the reference ran with an oracle token that showed it the true block poses and
which blocks carry load). Deciding when to stop is part of the job.

**Budgets per tower** (cell_api.md): 2 400 s simulated; 1 500 camera frames; program time 900 s (your program's own
time, plus 0.005 s per call and 0.2 s per frame; the simulation is not charged; your processes are stopped while the
cell works); at most 20 s between a reply and your next call; 2 700 s of the grader's wall clock at the latest.
Whatever ends an episode early ends only that tower, scored as it stands.

**How grading runs.** In a fresh copy of this image in which only `/home/user/submission/` is restored: everything your
program needs must be inside the submission (at most 60 MB; symbolic links are dropped; no `.pt .onnx .safetensors
.ckpt .h5 .pb .pkl .npz` files). The four hidden towers run at the same time, each program as an unprivileged account
other than yours, with a minimal environment (an empty HOME and TMPDIR, `PYTHONPATH` = the client library,
`ARB_GIVEN_DIR` = a read-only copy of `~/given`, one BLAS / OpenMP thread), under `/opt/venv/bin/python3` (Python 3.12
with numpy, scipy, opencv, pillow, mujoco) and without network.

**What scores 0 for the whole submission**: no `play.py` defining `solve`; a `.py` file the parser cannot read; a
weight / pickle file or more than 60 MB; a Python file of the submission that imports a networking or process module
(`socket`, `ssl`, `http`, `urllib`, `requests`, `asyncio`, `selectors`, `subprocess`, `multiprocessing.connection`,
`ctypes`, `cffi`, `pty` and similar), imports a module under a computed name, runs computed code or starts another
program; `play.py` not importing within 120 s and 120 CPU seconds; no `step` call in any tower; a process of your
program that cannot be killed at the end of an episode; a remote network socket seen; a file of the submission written
after your session ended.

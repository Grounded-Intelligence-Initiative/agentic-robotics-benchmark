# greenhouse_truss_tomato_harvesting: design notes for maintainers and validators

The agent-facing contract is `instruction.md`; the crop is described in
`environment/setup/payload/given/crop/growing_protocol.md`, the tool in `tool/cutter_gripper.md`, the cart, its
cameras and its wire in `site/site_interface.md`. This page records the design decisions, what the grader checks and
why, what is known to be rough, what was and was not verified, and a dated status log.

**Design rule: the agent gets what the automation engineer of a real harvesting robot would have, and nothing
else.** It gets the growing protocol (how a plant is built, the cultivars' colour charts and readiness rules, the
greenhouse's rules), the tool's data sheet, the cart's interface document (the cameras' exact intrinsics, their
nominal mounting and how far a robot's own may be off it, the depth sensor's data sheet, a surveyed calibration
plate, a colour reference card, surveyed row markers), the robot's model, and a commissioning service that runs the
cart on practice row sections. It does not get the cell's code, where anything of a plant is, which truss is ready, a
yield or a debrief of any session, or a rehearsal on a graded row or on the graded cultivar.

## 1. The job and why it is worth measuring

Truss tomatoes are harvested by hand: a picker finds the truss whose fruits have coloured, cuts its peduncle between
the main stem and the knuckle, and lays the truss in a crate in one layer, several hundred times an hour. A robot for
it needs what manipulation benchmarks usually leave out: a target that is a few millimetres thick, hangs on a plant
that gives way and swings when touched, is hidden behind leaves and gives a depth camera no reading; a decision made
from colour under a light that is the row's own; a tool that destroys what it closes on by mistake (a cut main stem
ends a plant's season); and a soft, heavy thing to carry and to put down without dropping, pressing or stacking it.
It is a Physical Execution task for the `arm` platform; nothing among the arm tasks covers harvesting or any work on
a compliant, living target.

## 2. The cell (`tests/rowcell/`)

Where it runs: in the verifier (the grader starts `rowcell/server.py`, one process per row section, as the image's
account `rowcell`) and in the agent's sandbox as the commissioning service (below), from a byte copy that only that
account can read. One process is one row section: Isaac Sim 6.2 headless, PhysX on the CPU (`enableGPUDynamics`
false, TGS solver, 120 Hz, six physics steps per 0.05 s control tick), the RTX renderer on the GPU for the two cameras
only. The physics engine's poses are read through tensor views and written to the stage only before a render (the
write-back at every step cost four times the simulation).

- **`farm.py`**: every number: the cart, the crates, the tool, the cameras and the depth sensor's data sheet, the two
  cultivars, and `draw_row(seed, cultivar, ...)`: a row section's hidden differences from a 64-bit seed through a
  BLAKE2b counter stream. Per row section four plants with three trusses each; per plant a leaning stem, the trusses'
  heights and sides, each peduncle's direction, length and thickness, four to six fruits a truss on alternating
  sides, every fruit's ripeness (a truss maturity, each fruit 0.45 behind the one before it), two leaf nodes above
  every truss on the plant's own phyllotaxis. By construction four to six trusses are ready (on each plant the lowest
  ones; the first truss above them is drawn from 3.8 to 5.6 and is often one fruit short), and every ready truss has
  the stated straight approach free (a truss that has not is drawn again; a leaf that would close it is one the
  grower has taken off). Three more draws come with a set's entry, each from a key of its own
  (`review/select_set.py`): the robot's camera offsets, the row's light, the row's mechanics.
- **`scene.py`**: the USD stage (pxr only): the UR20 of NVIDIA's asset library, referenced unmodified, with two
  links added to its articulation: a `rail_link` that becomes the articulation's fixed root, on which the robot's
  base rides along a prismatic joint (the cart's travel), and `ee_link`, the tool carrier with the cutter-gripper's
  colliders. The cart's deck and crates are a body of their own, held to the base by a joint (the arm's links do not
  collide with each other, so the cart must not be part of the articulation). A plant is rigid bodies on D6 joints
  with spring drives: five stem segments anchored at the gutter and held at the top by the support string (a joint
  to the world with linear springs); a truss = a peduncle body (the cut section, from the stem to the knuckle) and
  three rachis bodies, each carrying its fruits as sphere colliders; a leaf = one body (petiole and blade).
- **`rules.py`**: what is not left to PhysX (next list). NumPy only; no simulator needed to read or test it.
- **`cell.py`**: the runtime: Cartesian and joint moves through damped-least-squares inverse kinematics on the
  commanded joint state (`kin.py`), smooth start and stop, speed limits; the cart's travel by its odometer with the
  row's slip; the external wrench at the flange from the articulation's incoming joint force on the tool carrier, the
  tool's own weight taken out (a held truss's weight shows in it); protective stops; the cameras (Replicator render
  products with pinned RTX settings); the observation; `truth()` and `score()` for the grader only.
- **`sensors.py`**: what the wrist camera's depth channel makes of the renderer's exact depth: an active-stereo
  sensor's noise, quantization and lost readings, among them anything narrower than the matcher's window of 31 pixels
  (a morphological opening of the disparity): a peduncle, a petiole and a far stem give no reading.
- **`wire.py`**, **`server.py`**, **`cellproc.py`**: the action's schema, the cell as a process, and the client the
  grader and the service start it with.

**What is rule-based (the stated simplifications).**

- *The cut is a rule on what lies across the mouth, not fracture mechanics.* Everything the blade can meet is a
  straight member with a radius (a stem segment, a peduncle, a rachis segment, a petiole). A member is in the mouth
  at a plane when its axis crosses that plane between the jaws and between the throat and the tips (16 mm wide, 14 mm
  deep); closing jaws push what is between them to the middle, so a member that was in the mouth at any tick of a
  closing stays caught while it is there. The jaws meet the thickest caught member at the blade's plane when the
  opening equals its diameter: with a closing force of the row's toughness times the member's cross-section or more
  the blade cuts it through (its joint to what it grew on is removed) and the jaws go on; with less they stall and the
  opening reads the diameter. The jaws' colliders stand at the open mouth: what shows closes, what collides does not.
- *The hold is a rule.* The pad, 13 mm to one side of the blade, takes hold of every caught member at its own plane
  that the jaws have closed down to (a fixed joint to the tool carrier, created with the pose the body has: no
  snapping) and keeps it while the jaws stay closed on it with 3 N or more. A cut peduncle stays in the tool only when
  the pad is on the truss's side of the cut. A peduncle the pad presses with more than 1.4 times its own cutting
  force is crushed.
- *What bruises is two thresholds on the physics engine's state.* A rachis body (with its fruits) whose velocity
  changes by more than 1.5 m/s within one control tick (a fall of 11 cm, a blow, a plant that is yanked and snaps
  back); a rachis body that the tool, one of the arm's four forward links or the cart's body (the crates) presses
  with more than 25 N, read once per control tick.
- *A truss is a short, heavy chain.* Its knuckle and rachis joints bend against 0.15 N m/rad and turn about the
  segment's own axis against almost nothing (the fruits' pedicels give: a truss laid down settles flat). Fruits are
  rigid spheres fixed to their rachis segment; pedicels and calyces are drawn only.
- *Leaves are rigid plates on compliant joints*, not deformable; they pass through each other and through stems (a
  collision group), and collide with the robot, the tool and the fruits.
- *The stacking rule is geometry at the end*: a truss lies on another when a fruit of the one touches a fruit of the
  other (centres within 1.1 times the sum of their radii) from above (higher by more than half that sum).

Everything else is rigid-body physics: the arm with its position drives and torque limits, the plants swaying on
their joints and strings, a truss swinging on the tool, lying down in a crate and resting there, the tool pushing a
leaf or a truss aside (which is what makes a target move), the wrench, the protective stops.

**The levers** (each a fact of the job with a line in the documents; none a deduction):

- *A target thinner than the sensor resolves, in a mouth just wider than careful perception is accurate.* A peduncle
  is 4.4 to 6.4 mm thick and gives the depth channel no reading at any distance; the mouth takes it within about
  +-5 mm across and +-7 mm along the tool's axis, and the jaws' bodies push away what they do not take in (the
  truss then swings). Through the nominal camera mounting the tool centre point is 6 mm off in the typical robot and
  up to 14 mm (`MOUNT_TOL`), so the mounting has to be identified (the plate), and the peduncle located from colour
  views (section 5 gives the figure a careful triangulation reaches).
- *Readiness from colour under the row's own light*, against the cultivar's chart, with the truss above the ready
  ones often one fruit short of the rule.
- *Irreversible facts*: the blade cuts whatever is caught once the force is enough (a stem takes four to eight times
  a peduncle's force: a closing at full force cuts it, a closing with a peduncle's force stalls on it and reads its
  diameter); a force above 1.4 times a peduncle's own cutting force crushes it, and the row's toughness is not on
  the wire; the pad on the plant's side drops the truss; a truss dropped, struck or pressed is waste; a crate takes
  one layer.
- *Conditions of the job* (the merged arm tasks' own: `kitchen_appliance_tending`, `lab_sample_handling`): a two-hour
  session; three commissioning sessions in all, never refunded, on four practice rows of another cultivar (another
  chart, another readiness rule, smaller fruits and thinner peduncles, softer plants) worked by other robots (other
  camera mountings); documents that state facts and no procedures; a simulated-time allowance per row section of
  about 1.3 times the reference's slowest hidden row.

**The commissioning service (`environment/commission_service/`).** `commissiond.py` runs as `rowcell`, started by
`environment/setup/setup.sh`, on `/run/rowcell/commission.sock`: one session at a time, a session is one of four
practice row sections in a fresh cell process started without the grader's switch (the process then refuses `truth`
and `score`), three sessions in all, counted in the account's own state folder. The practice rows are drawn by
`review/select_set.py` under another tag and another cultivar; none is a graded row. The agent gets
`given/lib/rowcell_client.py` (the connection) and `~/commission/run_session.py` (runs `run.sh` in a session with the
grader's deadlines and repeated steps, and prints what the program saw).
Before the first session the service warms up, once per sandbox: a throwaway cell renders both cameras, so that the
renderer compiles its shaders in a process that then exits (as the grader's canary does for the graded rows). A cell
that compiles them keeps the compile's memory for as long as it lives: 14.4 GB resident against 6.0 GB on a warm
cache (measured in the image), and an interactive session can live for most of the agent's time. The warm-up is no
session; an `open` that arrives meanwhile waits for it.

## 3. What the grader checks (`verify.py`, `row_grade.py`)

- No `/home/user/submission/run.sh`: an honest zero, no simulator started.
- A canary cell on a row outside the set: the simulator starts, both cameras render (shape, not flat, sane
  brightness, depth present), the robot follows a small move. A failure is a task error, never a zero.
- The fixed hidden set `hidden/rows.json`: six row sections of the cultivar Rossa, the same for every submission,
  drawn by `review/select_set.py` under rules committed before any key was drawn; no row was chosen or dropped for
  how the reference does on it.
- Per row a fresh cell and the kit's `Solver` (the controller at the agent's UID through `nonet_exec.py`: no
  internet socket), one action per step until the row ends. The probe lock (`IdleAwareProbeLock`, rate 0.03, pass
  fraction 0.1): a probed step is asked twice with the same `t`, once with the robot's readings shifted
  (`wire.perturbed`: the joints by up to 0.01 rad, the tool poses and a wrist frame's pose those of the shifted
  joints, built from the cell's own angles before rounding), in a hidden order; pairs in which neither reply moves
  the arm or the cart are not counted. The schedule is a function of the row's index and seed.
- The yield (`rules.Rules.credit`): after the end and 2 s of settling (during which the rules still watch the
  fruits), a ready truss counts when it was cut free, is whole (no rachis cut), unbruised, unsqueezed, uncrushed, its
  plant stands, the tool does not hold it, every fruit's centre is inside one crate's inner footprint between the
  floor and the rim, and it neither lies on another cut truss nor has one lying on it. The row's yield is the count
  over the row's ready trusses; `mean_row_yield` is the mean over the six rows. A row that ends in any way keeps what
  was delivered by then. Nothing is subtracted anywhere, and there is no gate on a row or a plant beyond the lost
  plant. A controller that does nothing earns 0.
- The verdict's metrics count each ending, the trusses delivered, cut unready and damaged, plants lost, leaves cut,
  and the run's largest distance to each cliff (`margin_cut_y_mm`, `margin_cut_z_mm`: how far off the mouth's centre
  a cut member was; `margin_cut_force_ratio`, `margin_pad_ratio_max`: force over a peduncle's cutting force, against
  the crush ratio 1.4; `margin_dv_max` against 1.5 m/s; `margin_press_max` against 25 N).
- A failure of the cell itself, more than 64 processes of the agent's account, and the grader's own deadline are
  handled as in `arm/battery_pack_disassembly_to_modules` (the code is that task's): the controller stopped and its
  account emptied before one retry; a second failure scored when the controller ran beside it; a verdict from the
  rows graded when `verifier_deadline_s` runs out.

## 4. The reference program and the anchor

`solution/reference/controller.py` is a privileged reference in the sense of the pump skid, pantry and battery tasks:
the submission carries a 256-bit token, the grader then adds `truth` (every stem, leaf, peduncle and fruit where the
physics has it, which trusses are ready, each peduncle's cutting force, the cart's true place and slip, the exact
commanded targets) to its observations. With that it skips perception and identification; every motion, cut and
truss laid down goes through the public actions, rules and physics. For a ready truss: the cart to the plant; the
stated approach, followed onto the peduncle as it moves; a gentle closing (the opening reads the diameter), then 1.15
times the peduncle's cutting force; out by the way that leaves the tool and the hanging truss clear; over a crate by
the base joint with the tool as it was (the truss hangs as it hung); the tool upright over a free bay; the truss laid
down by feel (the clamp goes along the bay and comes down only while the wrench shows the truss not pushing back);
let go 0.10 m over the floor. Its displacements are taken from the observation it answers (the plan yields a function
of an observation), so the probe lock is honest.

The anchor: `tests/anchor.json` (the status log gives the runs).

## 5. Known limitations and open decisions

- **Validated on one host and one GPU model** (the author's validate host: one RTX 3090 on driver 535, below
  Omniverse RTX's R550 floor, hence `--/rtx/verifyDriverVersion/enabled=false` in `rowcell/server.py`; rootless
  Docker without a CPU quota). Not exercised: any cloud GPU model, a host with a CPU quota.
- **The cut and the hold are rules**, stated as such in the tool's data sheet: closing jaws do not move a body in
  PhysX, the blade's work is a threshold on force over cross-section, the pad's hold is a fixed joint.
- **Bruising is two thresholds** read once per control tick; a contact impulse is seen only when it falls on a tick's
  last physics step, so a marginal press can count on one host and not on another. The reference's distance from
  both is in the verdict.
- **No non-privileged baseline yet.** What was checked instead is the camera-only check (section 6;
  `review/camera_check/`): seven of nine peduncles located to under a millimetre from colour frames alone, two not
  found behind fruit. A perception program that harvests whole rows is open work. The frontier episode's program
  (section 6) delivered 2 of 27 trusses from the cameras alone: the job can be started from what the agent is
  given, and nothing yet shows it can be finished.
- **A probe's copy keeps the true observation's pictures**, as in the battery task.
- **Reproducibility.** PhysX on the CPU repeats a run on one machine; across CPUs it is not expected to be
  bit-identical, and the renderer is not repeatable even on one GPU. The yield reads where bodies lie and what
  happened to them.
- **The values of the thresholds** (1.5 m/s, 25 N, 1.4, the mouth's size) are the author's; they live in
  `rowcell/farm.py` and the documents follow them (`review/make_documents.py`).

## 6. Status log

**2026-11-27, 12:56 to 16:10 (the author's validate host; native Isaac Sim 6.2, RTX 3090, driver 535).** Built in
one session, with the generic modules of `arm/battery_pack_disassembly_to_modules`.

- *The cell.* A first design with a 0.20 m tool, a 28 mm mouth, three approach directions and stiff trusses did not
  let even the privileged reference work reliably, and was changed before any hidden value was drawn: the UR20's wrist
  under an upper truss (the tool is now 0.36 m long), a truss that jams between clamp and crate floor on stiff joints
  (a truss is now a floppy chain that turns about its own axis), trusses sprawling into the next lane (crates now have
  three bays), side approaches that swept the wrist through the neighbouring plant and flipped it over the crate (one
  stated approach). Per control tick the cell takes 10 to 15 ms (the physics engine's write-back to the stage off).
- *Rules before draws.* `review/select_set.py` with rules 1 to 4 was committed in `cd6cf609`; the four keys were
  drawn and the sets written in `43aaafc9`. No row was drawn again.
- *The reference on review rows* (cultivar Rossa, seeds 1 to 14, outside both sets; 71 ready trusses): every ready
  truss delivered on twelve rows, one truss each lost on two (pressed onto a crate's floor with 27 and 31 N while
  being laid down). The lay-down was changed (an arc about the truss's lowest fruit, followed by the wrench, 3 cm/s,
  guarded at 2.5 N); the four rows re-run after it, those two among them: all delivered, the largest press 12.8 N, the
  largest change of a fruit's velocity 0.83 m/s. On two review rows of the practice cultivar (Ambra, seeds 31 and
  32) the reference delivered every ready truss as well (6 of 6 and 4 of 4).
- *The grader end to end, natively* (`tests/verify.py` as an ordinary account, the canary included): no submission
  0 (no simulator started); the example program 0 on all six rows; the reference `mean_row_yield` 1.00000 (27 of 27
  ready trusses; 218, 241, 270, 356, 251, 300 steps; 214 to 328 s simulated; probe lock 39 of 40, 5 idle; the whole
  verify 599 s), twice, on the tree of `43aaafc9` and on the final tree; a blind replay of the reference's recorded
  actions on hidden row 0: probe lock 0 of 4, reward 0 (it delivered one truss of four).
- *The allowance.* 430 s of simulated time a row section, 1.31 times the reference's slowest hidden row (328 s).
- *Tightened before the first frontier episode* (the reference's margins untouched): the mouth's depth 20 to 14 mm,
  the crush ratio 1.6 to 1.4 (the reference cuts at 1.15 times a peduncle's cutting force), 400 frames a row (600).
- *The commissioning path, natively*: the daemon as an ordinary account, `run_session.py` with the example program
  on practice row 1 (Ambra, 5 ready trusses): the session opened in 33 s, the frame arrived, the count went 3 to 2.
- *The camera-only check* (`review/camera_check/`; frames saved from two review rows of the graded cultivar,
  seeds 21 and 22, with the truth beside them; CPU only; nine ready trusses). It asks one question: do the wrist
  camera's colour frames, with what a submission has, hold a peduncle's place to within what the mouth takes (about
  5 mm across, 7 mm along the tool's axis)? Step 1, `calibrate.py`: the wrist camera's mounting from six views of the
  plate with OpenCV's chessboard detector and `solvePnP` (the board's labelling taken from the nominal mounting): a
  point 0.30 m in front of the camera is 9.6 mm off through the nominal mounting (the review rows' test offset: 3.5,
  -4.2, 2.1 mm and 0.9, -1.2, 0.6 degrees) and 0.43 and 0.56 mm off through the identified one. Step 2,
  `peduncle.py`: eight colour views of each truss from 0.36 to 0.42 m (no depth); in every view the thin green stalks
  that leave the main stem downward, each as a line and a junction; the line in space on which most views agree (at
  least three), from the joint readings and the identified mounting. Seven of the nine peduncles are found, their
  cut points 0.0 to 0.4 mm off across the mouth and 0.0 to 0.6 mm off along the approach (3 to 6 views agreeing).
  Two are not found (fewer than three views agree; the check says so and gives no estimate): on both, a fruit of the
  truss above hangs in front of the junction in several views, and the peduncle runs close along the stem. So the
  information is in the frames at the needed accuracy, a careless view set loses trusses to occlusion, and nothing
  here grasps, cuts or lays down anything: a full program without the privileged view is open work. (The views
  were aimed with the truth: the check is about what frames hold once a truss is in view, not about finding it.)
  The frames come from `capture.py` (it needs the simulator) and are not in the repository; the renderer does not
  repeat a frame, and a second capture of the first row gave 0.49 mm for the mounting and three of its four peduncles
  (0.0 to 0.2 mm across, 0.2 to 0.4 mm along; the fourth not found, which the first capture had found).

**2026-11-27, 16:11 to 16:20: `ale validate` on the author's validate host.** Run `validate-bc3c4ffd` on the tree of
`c32222a7`, the pinned engine `bfb7505` (0.1.1), at the first attempt:

```
ok    grounded-intelligence-initiative/greenhouse_truss_tomato_harvesting: untouched={'reward': 0.0}, oracle={'reward': 1.0}
```

The image built on the battery task's layers (the Dockerfile is that task's up to the cell's own account); setup
started the commissioning service; the untouched sandbox scored 0.0 (no submission); the oracle episode's verify
stage (the cell as the account `rowcell` from a private copy, the seccomp launcher, the kit's account clearing, the
canary) took 435 s and measured `mean_row_yield` 1.00000: every row in the same steps as the native runs (218, 241,
270, 356, 251, 300), 27 of 27 ready trusses, probe lock 39 of 40, no protective stop, the margins as in
`tests/anchor.json`. Deviations of that host from an evaluation host: the NVIDIA driver 535 is below Omniverse RTX's
R550 floor (the cell's driver check is off, the canary judges the frames); rootless Docker cannot set a CPU quota
(the engine's `--cpus 8` was dropped; every exec'd process was pinned to 8 CPUs instead). `tests/anchor.json` keeps
the value 1.0; only its `source` changed after this run. Not exercised by a validate: the agent phase, any cloud GPU
model.

**2026-11-27, 16:22 to 16:24: the commissioning path inside the task's image** (what a validate does not run). One
container of the image that validate built, with the GPU; `setup.sh` as root; then, as the agent account,
`commission/run_session.py --row 1` with the example program: the session opened (139 s, the first one of a
sandbox), the overview frame arrived, the count went 3 to 2; the agent account cannot list
`/opt/rowcell-commission`. The whole repository's `pytest` passed on the branch.

**2026-11-27, 16:25 to 16:36: a first frontier episode that did not finish (no score).** Run `1ea8c16e` on the tree
of `64e415cf` (Claude Opus 5.5, claude-code 2.1.309, effort high, no web tools): `env_error`, "guest session closed",
634 s into the agent phase; the agent had read the documents, opened its first session interactively and was
identifying the wrist camera's mounting from the plate. The cause is memory on the validate host, not the agent and
not the engine: the host has 30.9 GiB (the task declares 32 GB, so the engine's limit does not bind there) of which
about 10 GB were in use outside the sandbox, and the first session's cell stood at 14.4 GB resident after the
renderer's cold shader compile; the host's memory pressure rose (60-second average 19 % just after the end) and
its out-of-memory daemon, which acts on pressure, ends a sandbox in that state. The kill itself is inferred: the
system's journal is not readable from the author's account; the same fault ended episodes of other tasks on this
host. Measured afterwards in the image, a session that takes 90 frames: first session of a sandbox, 158 s to open,
the cell 14.4 GB, the sandbox 13.4 GiB; second session, 17 s to open, the cell 6.0 GB, the sandbox 4.5 GiB. Natively
the cell stays at 6.1 GB over 300 steps with frames (no growth).

**2026-11-27, 16:46 to 16:53: the service warms up before the first session** (`commissiond.py` `warm_up`, above).
The first cell's 14.4 GB belong to the shader compile, so the compile now runs in a throwaway cell when the service
starts. In the image (the new daemon over the validated image's): the warm-up 138 s (its cell 14.4 GB, then gone);
a session asked for at once waited and opened 15 s after it, its cell at 5.8 GB, the sandbox at 4.4 to 5.1 GiB; the
next session opened in 13 s; the count went 3, 2, 1 (the warm-up counts nothing). Natively the same path: warm-up
65 s, the session counted once. The instruction and the interface document now say what an agent sees: opening a
session takes up to a minute, and one asked for in the sandbox's first minutes waits for the warm-up. Nothing of the
cell, the rules, the rows, the reference or the grader changed; `environment/` did, so `ale validate` ran again
(below). Also corrected: two stale comments (`task.toml`'s verifier estimate counted 600 frames; the Dockerfile's
header named another task's cell).

**2026-11-27, 16:56 to 17:04: `ale validate` again, on the tree with the warm-up.** Run `validate-75ad8c0a` on the
tree of `9734d8af`, the pinned engine `bfb7505`:

```
ok    grounded-intelligence-initiative/greenhouse_truss_tomato_harvesting: untouched={'reward': 0.0}, oracle={'reward': 1.0}
```

Untouched 0.0 (no submission). Oracle: verify 426 s (the canary 127 s, the six rows 31 to 47 s each),
`mean_row_yield` 1.00000 in the steps of every earlier run (218, 241, 270, 356, 251, 300), 27 of 27, probe lock 39 of
40, no protective stop, the margins unchanged. `tests/anchor.json`: the value stays 1.0; its `source` now names this
run (and the first one).

**2026-11-27, 17:05 to 17:19: a second frontier episode that did not finish (no score), and what ended both.** Run
`c11a3b0b` on the tree of `772f2003`, the same configuration: `env_error`, "guest session closed", 816 s into the
agent phase; 42 model calls, none cut. This time the host's memory was sampled every 30 s. The warm-up did what it
was made for: the sandbox peaked at 13.1 GiB during the service's warm-up and then stayed at 4.9 to 5.1 GiB while
the agent worked in a session (13.3 GiB and held, in the first episode). What ended the episode was outside the
sandbox: between 17:15:47 and 17:17:24 the host's use went from 13.4 to 22.8 GB with the sandbox unchanged at
5.0 GiB, a job of another project on the same machine standing at 9 to 10 GB resident and growing; the memory
pressure's 10-second average went from 0 to 8.9 % and on to 28.5 %, and the sandbox was ended at 17:19:09. So the
first episode's entry above names a real cost (a first session's cell at 14.4 GB, since removed) but probably not
the whole cause: its last two minutes saw no new activity in the sandbox either, and no sample was taken then.
Neither episode says anything about the task's difficulty. In its 13 minutes the second agent had identified the
wrist camera's mounting, graded colour, located peduncles from three views, read a peduncle's diameter with the
jaws, found its cutting force by raising the closing force in steps, laid a truss down by the wrench, and written a
whole controller, all in one interactive session.

**2026-11-27, 17:35: the validate host restarted** (a kernel update). Its Docker images, kept under `/tmp`, were
lost with it; the run records were restored from a copy. The task's image was built again from the tree of
`772f2003` with `ale prepare`; `ale validate` was not run a third time (`772f2003` differs from the validated
`9734d8af` in the anchor's `source` text and this file only).

**2026-11-27, 18:09 to 18:54: the frontier episode that scored (three commissioning sessions).** Run `4beb1baf` on
the tree of `772f2003`: Claude Opus 5.5 through the claude-code harness (CLI 2.1.309, effort high, WebSearch and
WebFetch off, subscription auth), the task's two-hour budget, one episode. **reward 0.09** (unrounded 0.0926;
`mean_row_yield` 0.0833).

- *The verdict.* 2 of the 27 ready trusses delivered: 2 of 4 on row 0, none on rows 1 to 5. Every row section ended
  by the program's own `done` after 260 to 361 s simulated (430 allowed) and 301 to 453 steps; no protective stop, no
  truss damaged, nothing cut that was not ready, no plant lost, no leaf cut; 156 frames in all; the program's own
  time 26 s; probe lock 38 of 39 (22 idle pairs not counted); the canary passed; verify 534 s. The yield was lost by
  not cutting, not by damage: 25 ready trusses were still on their plants at the end.
- *The agent.* It ended by itself 2175 s (36 min) into the two hours, with all three sessions spent. Session 1
  (interactive) and session 2 (a whole run of its program) both ended at the third protective stop, from one cause:
  a parking pose that brought the wrist over the robot's base. Session 3 (a whole run on a practice row) cut two
  trusses and held them (it read their weight, 2 and 3.5 N, and the same toughness from both cuts), missed a third
  five times (nothing in the jaws) and pressed a fourth onto a crate's floor into a protective stop. What it changed
  afterwards (the peduncle's estimate, the lay-down, the cart's centring) it could check only against a mock of its
  own and the frames it had saved: its final program never ran on the cell, and its closing message says so. Its
  method was sound throughout: the mounting from the plate (0.2 px), the light from the card, fruits and stems from
  wrist RGB-D frames, the diameter from a gentle closing, the cutting force found by raising the closing force in
  steps of 8 %, a guarded lay-down.
- *The transport.* 119 model calls, all forwarded, none stopped at `max_tokens`, the longest 174 s, no cut stream.
- *The host.* Memory sampled every 30 s (87 samples): the sandbox at most 12.2 GiB (the service's warm-up), 4.4 to
  5.5 GiB afterwards; never less than 14 GB available on the host; memory pressure at most 0.8 %; no job outside the
  task.
- *What it does and does not show.* One episode of one model. The agent used 36 of its 120 minutes and lost two of
  its three sessions to one motion mistake; another episode may go differently.

**2026-11-27, 18:55 to 19:03: `ale validate` on the rebuilt image.** Run `validate-23b8f549` on the tree of
`772f2003`, to confirm the image built again after the host's restart: `ok`, untouched 0.0, oracle 1.0
(`mean_row_yield` 1.00000 in the same steps, 27 of 27, probe lock 39 of 40, the margins unchanged, verify 416 s).
`tests/anchor.json` is unchanged.

**2026-11-27, 19:03 to 19:18: five commissioning sessions (were three), and `ale validate` on that tree.** The
contributor's decision after the episode above: its agent lost two of three sessions to one motion mistake, so its
final program never ran on the cell, and a score near zero for that reason says little. Only the count changed
(`db948835`: `review/select_set.py` `service()`, `dev_rows.json` with the same keys and rows, the instruction, the
interface document, `task.toml`'s comments, section 2 above); sessions stay never refunded, one at a time, without a
yield or a debrief; the cell, the rules, both sets, the reference, the grader and every threshold are as they were.
The tree before it, `7b57d4eb`, is the three-session version. Natively the count went 5, 4, 3, 2, 1, 0 over five
sessions of the example program and a sixth was refused ("no commissioning sessions left"); the warm-up counted
nothing. Run `validate-63a08ee8` on the tree of `db948835`, the pinned engine `bfb7505`:

```
ok    grounded-intelligence-initiative/greenhouse_truss_tomato_harvesting: untouched={'reward': 0.0}, oracle={'reward': 1.0}
```

Untouched 0.0; oracle `mean_row_yield` 1.00000 in the same steps, 27 of 27, probe lock 39 of 40, verify 415 s.
`tests/anchor.json`: the value stays 1.0; its `source` names this run and the three before it.

**2026-11-27, 19:19 to 19:56: the frontier episode on five sessions.** Run `cdc52771` on the tree of `910a7162`,
the same configuration (Claude Opus 5.5, claude-code CLI 2.1.309, effort high, WebSearch and WebFetch off,
subscription auth, two hours), one episode. **reward 0.44** (unrounded 0.4352; `mean_row_yield` 0.3917).

- *The verdict.* 11 of the 27 ready trusses delivered: 2 of 4, 2 of 4, 1 of 4, 3 of 6, 0 of 4 and 3 of 5. Every row
  section ended by the program's own `done` after 260 to 398 s simulated (430 allowed) and 467 to 664 steps; 3
  protective stops over the six rows (none latched); 4 trusses damaged; 4 trusses cut that were not ready (they earn
  nothing and take a bay); no plant lost, no leaf cut; 163 frames in all; the program's own time 20 s; probe lock 64
  of 64 (25 idle pairs not counted); the canary passed; verify 572 s. Its distance from the cliffs, where it cut: up
  to 5.1 mm off the mouth's mid-plane and 6.1 mm along the tool axis, the closing force up to 1.24 times a
  peduncle's cutting force.
- *The agent.* It ended by itself 1623 s (27 min) into the two hours with all five sessions spent. Session 1,
  interactive: calibration, colour, a peduncle probed with the jaws, the closing force raised until the cut, a carry.
  Sessions 2 and 3: protective stops again from poses with the wrist over the robot's base axis (the mistake of the
  three-session episode's agent as well). Session 4: the motion mended, no truss judged ready; it then ranked
  trusses by ripeness and took the best four to six. Session 5, a whole run on a practice row: no protective stop,
  296 s simulated, three trusses cut, held and laid down in six attempts. What it added after that ran only against
  a mock of its own; its closing message says so, and says it cannot tell its yield.
- *The transport.* 58 model calls, all forwarded, none stopped at `max_tokens`, the longest 133 s, no cut stream.
- *The host.* Memory sampled every 30 s (71 samples): the sandbox at most 13.7 GiB (the service's warm-up); never
  less than 13.8 GB available on the host; memory pressure at most 0.4 %; no job outside the task.
- *Read with the episode before it.* Two episodes of one model on two versions that differ in one number: three
  sessions, 2 of 27 trusses (0.09); five sessions, 11 of 27 (0.44). Each is a single episode, both agents stopped
  with more than three quarters of their time unused, and the difference between them is as much one agent's two
  extra sessions as it is chance: the spread of either version is not measured.

**2026-11-27, 20:00 to 20:13: a video of the reference, for people** (not in the repository). The privileged
reference on a review row of the graded cultivar (seed 7, outside both sets), natively: 6 of 6 ready trusses
delivered in 318.9 s simulated without a protective stop, the frames read from the render products between control
ticks (no frame asked through the wire). The repository's `pytest` passed on the tree of `bf7835a0`, with
`tests/test_longtail_pause_and_close.py` left out (its two `PauseTest` cases fail on the author's host at the
cleanup of a temporary directory on a network file system; the file tests another task's grader).

**2026-11-28: a cell that came up broken, once, and the canary's fresh start.** In a frontier episode of a trial
variant with four commissioning sessions (run `416d048a`), the verify stage ended as a task error 127 s in, with no
score: "the cell did not start in the grading environment". The canary's cell had taken 124.9 s to bring Isaac Sim
up (12 to 15 s in every other start) and then failed at its first log line: Kit replaces `sys.stderr` by an
interceptor that pushes every line onto a message queue of its scripting layer, the interceptor had no queue
(`AttributeError` in `omni/kit/app/_impl/__init__.py`, `self._mq.push`; Kit takes the queue once, when its scripting
layer starts), and Kit's own log said "carb.variant.plugin: Requested type py::object not known" at that moment. So
Kit's scripting layer had come up incomplete. Why is not established: the sandbox and Kit's log went with the
episode, and it was one of about sixty cell starts in grading sandboxes. The same image and tree graded the same
submission without incident afterwards (below). The grader already ran a row once more in a fresh cell when a cell
failed; the canary did not, and a cell that fails there ended the grading. Changed, in the cell and the grader and
in nothing an agent sees or is judged by:

- `rowcell/server.py`: the cell's own log lines go straight to the process's standard error and cannot raise; one
  line is sent through Kit's interceptor on purpose when the simulator is up, so that a Kit in that state is
  reported as a cell that did not start, and is not used.
- `verify.py`: a canary cell that does not start, or whose process fails during the check, is started once more in
  a fresh process and counted in `cell_retries`; twice is a task error, as is a cell that runs but does not render
  or move (no second start for that). `review/test_canary.py` checks this with stand-in cells.

*The submission of `416d048a`, graded afterwards.* The engine keeps a submission only while it verifies it, so the
agent's files were rebuilt from the episode's trajectory: its own file operations on `~/submission` replayed in
order (eighteen calls). The result has the sizes of the agent's last listing, to the byte (`controller.py` 20,796,
`percep.py` 9,952), and holds every line the agent printed from it. `tests/verify.py` of the trial tree then ran on
it as root in a fresh container of that tree's image, without the engine: the canary passed (15.3 s to start),
`mean_row_yield` 0.475 (13 of 27 ready trusses: 0 of 4, 1 of 4, 2 of 4, 3 of 6, 4 of 4, 3 of 5; one truss damaged,
nothing unready cut, one row section ended at its third protective stop), probe lock 27 of 27, which is reward
0.53. It is a grading of that agent's program, not an engine episode's score: the kit's clearing of the agent's
account did not run outside the engine, and the renderer does not repeat a frame, so a second grading of the same
program can differ by a truss or two.

## 7. Final report (2026-11-27)

**What this is.** Version 1 of the task, built in one day on the author's validate host. The final version has
five commissioning sessions: the task files of `910a7162`, plus this file's later entries. The contributor chose
it over the three-session version (the tree of `7b57d4eb`) after one frontier episode on each; the two differ in
the session count alone (`db948835`).

**Verified.**

- Static: `ale lint`, `scripts/lint_domain.py`, the task card (strict), task assets, GPU types, the kit's byte
  comparison, `review/check_copies.py`, `check_numbers.py` (33 statements), `select_set.py`, `test_rules.py` (8
  groups), `make_documents.py --check`, and the repository's `pytest` (one file of another task left out, above).
- Natively (`tests/verify.py` as an ordinary account): no submission 0, the example program 0, the reference
  `mean_row_yield` 1.00000 on the hidden set (27 of 27), a blind replay of the reference refused by the probe lock;
  the commissioning path with its count (5, 4, 3, 2, 1, 0, a sixth refused) and its warm-up.
- `ale validate`, four runs, each `ok` at the first attempt (untouched 0.0, oracle 1.0, the same steps):
  `validate-bc3c4ffd` on `c32222a7`, `validate-75ad8c0a` on `9734d8af`, `validate-23b8f549` on `772f2003` (the image
  rebuilt after the host's restart), `validate-63a08ee8` on `db948835` (the final task files).
- Frontier model, Claude Opus 5.5, one episode a version: **reward 0.44** on the final version (`cdc52771`, 11 of 27
  trusses), 0.09 on the three-session version (`4beb1baf`, 2 of 27). Two further episodes ended without a score
  when the host's memory was taken by a job outside the task.

**Not done, not exercised.**

- No program without the privileged view has harvested a whole row section; the camera-only check (section 6) and
  the two agents' programs (2 and 11 of 27 trusses) are what there is.
- One host, one GPU model (RTX 3090 on driver 535 with the renderer's driver check off; rootless Docker without a
  CPU quota; 30.9 GiB of memory against the declared 32 GB). No cloud GPU model, no supported driver, no host that
  limits the CPUs by a quota, no second CPU.
- `full_at` 0.9 is a margin for two lost trusses, not a measured spread: the reference repeats step for step on the
  one machine it ran on.
- One episode a version of one model. 0.44 is one episode, not far below the 0.5 that the contribution guide names;
  the spread from episode to episode is unknown, and both agents stopped with more than three quarters of their two
  hours unused.
- Ten of the fourteen review rows were not re-run after the reference's lay-down was changed.

**Open for the maintainers.**

- The session count. Two more sessions took one agent's yield from 2 to 11 of 27 trusses (one episode each); the
  three-session tree is kept (`7b57d4eb`), and `git revert db948835` restores it.
- The thresholds' values (1.5 m/s, 25 N, the crush ratio 1.4, the mouth's size) are the author's.
- Whether practice on another cultivar under other robot units is a fair commissioning picture (section 5).
- `gpu_types` beyond the one model the task ran on.

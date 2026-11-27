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
- **No non-privileged baseline yet.** What was checked instead is the camera-only check below; a perception program
  that harvests whole rows is open work, and whether the job fits the two hours is what the frontier episode measures.
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
  largest change of a fruit's velocity 0.83 m/s. Two review rows of the practice cultivar ran as well (below).
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
- *The camera-only check* (frames saved from two review rows with truth beside them; CPU only; first pass). The
  wrist camera's mounting from six views of the plate with OpenCV's chessboard detector and `solvePnP` (the board's
  labelling taken from the nominal mounting): a point 0.30 m in front of the camera is 9.6 mm off through the nominal
  mounting (a test offset of 3.5, -4.2, 2.1 mm and 0.9, -1.2, 0.6 degrees) and 0.43 and 0.56 mm off through the
  identified one. A peduncle from eight colour views 0.36 to 0.42 m away (no depth; a line fitted to the thin green
  stalk that leaves the stem, the planes through the cameras intersected): on the first row all four cut points
  within 1.7 mm across the mouth and 1.7 mm along the approach (medians 1.2 and 1.5 mm); on the second row two of
  five within 1.2 mm and three wrong by 10 to 22 mm, where the quick detector took another stalk in some views. The
  mouth takes about 5 mm across and 7 mm along. So the information is in the frames, and a detector has to be more
  careful than this one among leaves; a full program without the privileged view is open work.

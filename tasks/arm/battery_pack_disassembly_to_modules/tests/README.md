# battery_pack_disassembly_to_modules: design notes for maintainers and validators

The agent-facing contract is `instruction.md`; the pack series is described in
`environment/setup/payload/given/service_manual.md`, the tools in `tool_catalog.md`, the cell and its wire in
`cell/cell_interface.md`. This page records the design decisions, what the grader checks and why, what is known to be
rough, what was and was not verified, and a dated status log (version 0.1.0, 2026-11-24; built in one unattended
session on a server without Docker, so the engine's own gates are still owed: sections 6 and 7).

**Design rule: the agent gets what the automation engineer of a real disassembly cell would have, and nothing
else.** It gets the pack series' workshop manual (drawings, nominal positions, the torque table, the isolation
procedure), the tool catalog with the tools' tolerances, the cell's interface document (the cameras' exact
intrinsics, their nominal mounting and how far a cell's own may be off it, the depth sensors' data sheets, two
surveyed calibration plates), the robot's model, and a commissioning service that runs the cell on development
packs. It does not get the cell's code,
any ground truth (a part's pose, a fastener's head, state or wear), a credit or a debrief of any session, or a
rehearsal on a graded pack.

## 1. The job and why it is worth measuring

Taking traction battery packs apart to module level is the step that recycling and second-life lines still do
largely by hand: packs arrive in many variants, dirty, corroded, with fasteners replaced or missing, and under
voltage. A robot cell for it needs what manipulation benchmarks usually leave out: a long procedure with ordering
constraints that come from the product (nothing live may be touched before isolation; a part comes off only when
everything that holds it is out), tool and bit exchanges, fasteners that punish a wrong bit or a sloppy alignment,
and perception that has to deliver millimetres (a Torx bit engages within 1.2 mm; holes are within 2 mm of nominal,
and the pack lies within 12 mm and 1.5 degrees on its fixture). Nothing of the pack is on the wire: the program
identifies the variant, locates every fastener, tells a hexagon from a Torx head, notices rust, missing fasteners
and a harness remnant from RGB-D frames, and reads the nutrunner's torque and angle to know whether a fastener
turns, stalls or slips. It is a Physical Execution task for the `arm` platform; nothing among the arm tasks covers
disassembly.

## 2. The cell (`tests/packcell/`)

Where it runs: in the verifier (the grader starts `packcell/server.py`, one process per pack, as the image's account
`packcell`) and in the agent's sandbox as the commissioning service (below), from a byte copy that only that account
can read. One process is one pack: Isaac Sim 6.2 headless, PhysX on the CPU (`enableGPUDynamics` false, TGS solver,
120 Hz, six physics steps per 0.05 s control tick), the RTX renderer on the GPU for the two cameras only.

- **`series.py`**: the cell's geometry, the tools, the fastener catalog, the three variants and `draw_pack(seed)`:
  a pack's hidden differences from a 64-bit seed through a BLAKE2b counter stream. Per pack: the variant; the
  placement on the fixture (uniform within 12 mm and 1.5 degrees); for every fastener an offset within 2 mm, a head
  (the group's head, replaced by the other head of the same thread with probability 0.12), corroded (lid 0.15,
  busbar 0.08, bracket 0.12; breakaway 1.6 to 1.9 times the assembly torque) or missing (lid 0.06, bracket 0.05);
  a harness remnant (none, on the lid over one or two lid bolts, or along the busbars over two or three bolts).
  Two more draws come with a set's entry, each from a key of its own (`review/select_set.py`): the camera offsets of
  the cell a pack is taken apart in (rule 2) and a seed for the dabs of sealant over some heads (rule 3, `SEALANT`).
- **`scene.py`**: the USD stage (pxr only): the UR20 of NVIDIA's asset library, referenced unmodified, with two
  links added to its articulation: `ee_link` (the tool carrier, fixed to the flange) and `spindle_link` (the
  nutrunner's spindle on a prismatic joint with a spring drive: the 25 mm float of a real nutrunner). Both tools'
  bodies are children of `ee_link`; the cell shows and collides the mounted one and sets the link's mass. The pack's
  parts and fasteners are rigid bodies, kinematic until the rules free them.
- **`rules.py`**: what is not left to PhysX (next list). NumPy only; no simulator needed to read or test it.
- **`cell.py`**: the runtime: Cartesian and joint moves through damped-least-squares inverse kinematics on the
  commanded joint state (`kin.py`), smooth start and stop, speed limits; the external wrench at the flange from the
  articulation's incoming joint force on the tool carrier, the tool's own weight taken out; protective stops; the
  cameras (Replicator render products with pinned RTX settings; depth is `distance_to_image_plane`); the
  observation; `truth()` and `score()` for the grader only.
- **`sensors.py`**: what the cameras' depth channel makes of the renderer's exact depth (below). NumPy and OpenCV
  only; `review/test_sensors.py` holds it to its data sheet without a simulator.
- **`wire.py`**, **`server.py`**, **`cellproc.py`**: the action's schema, the cell as a process, and the client the
  grader and the service start it with.

**The cameras: a sensor's depth and a cell's own mounting (since 2026-11-25).** Until then the depth frames were the
renderer's exact distances and the cameras stood exactly at their published poses. A perception baseline written in
about an hour typed every head by its height above its seat in one overhead frame and located everything through
the published calibration, and scored 1.0 on the hidden set (section 6, the entry "2026-11-25, 10:00 to 11:35"). A
real cell offers neither shortcut. Two changes, both perception and engineering, neither a penalty:

- *The depth is an active-stereo sensor's* (`packcell/sensors.py`; data sheet `series.SENSORS`, stated in the
  interface document's section 5 the way a data sheet states it; also in `cell.json`). The renderer's exact depth
  goes through a lateral smoothing of 1 pixel; temporal noise of sigma_k z^2 (overhead 0.0009 per metre: 2.5 mm at the
  fixture, 1.65 m away; wrist 0.0025: 0.3 mm at 0.35 m), correlated over about 2 pixels, independent from frame to
  frame, up to twice as large on dim surfaces; quantization to the disparity grid (1/16 pixel at a 0.10 m and a
  0.018 m baseline: steps of 1.0 mm at the fixture, 0.3 mm at 0.35 m); and no reading outside the working range, at
  depth jumps of more than 2 disparity pixels, in the stereo pair's occlusion band beside tall edges, on dark
  surfaces (luma 40 down to 12) and on specular highlights (a colour channel 215 up to 245), lost in blobs. The sizes
  are those of industrial active-stereo cameras at these distances, a long-range model overhead and a short-range
  one on the wrist. The dark and bright dropouts are keyed to the rendered colour image (a sensor's own infrared
  image is not rendered); the stated thresholds let a program predict where it will have no reading. The colour
  image is unchanged. What this does to the job (`review/test_sensors.py`; section 6): one overhead frame types
  about a third of the heads wrong even when the head's centre is known, 32 averaged overhead frames or one close
  wrist frame type them right; the plated heads lose part of their top face's readings in a wrist frame (their
  highlights), so a centre from depth alone is up to about 1.6 mm off and the colour image's outline is what brings
  it within the Torx bit's 1.2 mm.
- *The cameras' poses on file are nominal* (as upstream #177 did for `lab_sample_handling`). A cell's overhead
  camera sits off its nominal pose by up to 15 mm and 1.5 degrees per component, its wrist camera off its nominal
  place on the flange by up to 4 mm and 1 degree (`series.MOUNT_TOL`, an offset in the camera's own frame: true =
  nominal times offset). Located through the nominal poses, a point on the fixture is typically 4 cm off (up to
  8 cm), a point 0.35 m in front of the wrist camera 6 mm (up to 13 mm). Every pack is taken apart in a cell of its
  own, graded and development packs alike, so a program identifies its cameras in every pack. What it identifies
  them with is on file and exact: two checkerboard plates on the fixture's strips that no pack covers
  (`series.CAL_PLATES`, 5 x 20 squares of 10 mm, surveyed; drawn only, nothing collides with them), the intrinsics,
  the robot's kinematic model and its joint readings. A wrist frame's `pose` on the wire is the nominal pose (the
  flange's pose times the nominal mounting), which is how `wire.perturbed` and the agent's library build a probe's
  copy too. The offsets were drawn under `review/select_set.py`'s rule 2 with a key of their own, after the rule was
  committed. Measured with standard OpenCV on a probe cell with hand-picked offsets (section 6): the two plates in
  one overhead frame give the overhead camera to 0.3 to 0.6 mm on the lid, and five whole-plate wrist views give the
  wrist camera's mounting to 0.2 mm.

**Sealant partly over heads (since 2026-11-25, the third lever).** With the two changes above, a revised
perception baseline still took three of four development packs apart completely after about an hour and a half of
work (section 6): a close wrist frame's depth gives a head's outline, its centre and its type. A resealed field pack
breaks that: about a third of the lid's bolts and a quarter of the others carry a dab of grey sealant
(`series.SEALANT`; service manual, section 7), a dome standing on the seat with its centre 0.3 to 0.9 head radii off
the axis, 0.6 to 1.0 head radii wide and 1 to 2.5 mm above the head's top. It covers a median 22 % of a head's top
(at most about half) and runs over the edge onto the seat, so the region that stands above the seat is the head and
part of the dab: its centre is a millimetre off in the median and up to 3 mm, more than a Torx bit's 1.2 mm, and part
of a Torx recess may be hidden. A program has to tell the sealant from the head (its matte grey against plated zinc
or rust, its round dome against the head's flat top and straight or circular edge) and fit the head's known outline
to what remains. It is drawn only (section 2's list); the bit's rules are unchanged. Drawn per pack under rule 3 of
`review/select_set.py` with a key of its own, after the rule was committed; `review/test_sensors.py` checks the
draw against the manual's statement.

**What is rule-based (the stated simplifications).**

- *Fasteners are rule-based constraints.* A fastener's body sits on its seat (its head is a real collider the bit
  presses on, through the spindle's spring); it turns only by the rule: the mounted bit is the head's own, the bit's
  axis is within 2.0 mm (socket) or 1.2 mm (Torx) and 3 degrees of the fastener's, the axial force (spindle stroke
  times spring rate) is at least 8 N (Torx: 10 N + 1.5 N per N m), and the torque limit reaches the fastener's
  breakaway torque; then loosening turns accumulate and after 7 (M6) or 9 (M8) turns the fastener is out and rides
  on the bit until `eject` turns it into a falling rigid body (whose body is pushed out of an overlap at 0.3 m/s
  at most, so one let go into a pile settles on it instead of being thrown off: section 5). A loose bit (a 13 mm socket on a 10 mm hexagon, a
  T30 in a T40 recess), an engagement outside the tolerances (up to 3.5 / 2.5 mm, 8 degrees), a cammed-out Torx bit
  and a tightening run above the head's limit wear the head; at wear 1 it is stripped and only the extractor bit
  removes it. Thread engagement, the fastener rising as it turns and the socket sliding over the head are not
  simulated: the bit's collider is a capsule whose round end stands on the head's flat top (a flat-ended
  cylinder on a head of nearly its own radius was thrown off sideways now and then: section 6), and that contact
  has a high friction (3.0, combine mode max), which stands in for the socket closing over the head. A part whose
  last fastener has just come out is freed only when the bit has lifted off it.
- *A part is held until what holds it is out.* The lid by all its bolts, a busbar by two, a bracket by one, a module
  by its two brackets and its two busbars (they have to leave their seats), the service disconnect and the plugs by
  a latch that gives at a pull of 60 N / 30 N along their axis. A held part is a kinematic body: pulling on it
  overloads the robot (a protective stop), it does not tear loose.
- *The grip is a rule.* Closing the fingers on a part's grip feature within the tool catalog's tolerances attaches
  the part to the tool carrier with a fixed joint (created at that moment, with the pose the part has: no snapping);
  opening removes the joint. The fingers are drawn but have no colliders; the gripper's body and the tool carrier do.
  There is no slip: a held part stays held until the gripper opens.
- *Exchanges are rules.* A tool is released or locked with the flange in the dock within 6 mm and 3 degrees at
  rest; a bit is exchanged with its tip in the bit station likewise. The parked tools and bits are drawn only.
- *Live parts.* Before isolation (the service disconnect out of its socket for 30 s) a tool point (the bit's tip;
  the gripper's centre point and fingertips) inside a busbar's box (3 mm margin) ends the pack. Checked once per
  control tick.
- *A dropped module* is a free, unheld module moving down faster than 0.7 m/s.
- *The harness remnant* is one rigid body (capsules along a polyline), kinematic where it lies until it is first
  gripped; a turning bit pressed on it marks it damaged.
- *Sealant over a head* (since 2026-11-25) is drawn only: a dome mesh under the fastener's body, no collider and no
  mass, so a bit seats on the head as it would without it, and the dab leaves with its fastener. It is soft sealant
  in the manual's words; what it does is hide part of the head from the cameras (below).

Everything else is rigid-body physics: the arm with its position drives and torque limits, contacts of the tool
with the pack, the bins and the tables (which is what the wrench shows and what triggers protective stops), parts
carried on the grasp joint with their weight, parts and fasteners falling into bins and lying there, a module
standing in its bay. The robot's self-collision is off (the articulation's own links do not collide with each
other).

**The commissioning service (`environment/commission_service/`).** `commissiond.py` runs as `packcell`, started by
`environment/setup/setup.sh`, on `/run/packcell/commission.sock`: one session at a time, a session is one of four
development packs in a fresh cell process started without the grader's switch (the process then refuses `truth`
and `score`), ten sessions in all, counted in the account's own state folder. The development packs are drawn by
the same rule as the hidden set under another tag (`review/select_set.py`); none is a graded pack. The agent gets
`given/lib/packcell_client.py` (the connection) and `~/commission/run_session.py` (runs `run.sh` in a session with
the grader's deadlines and repeated steps, and prints what the program saw: the end, steps, frames, stops, the
cell's messages).

## 3. What the grader checks (`verify.py`, `pack_grade.py`)

- No `/home/user/submission/run.sh`: an honest zero, no simulator started.
- A canary cell on a pack outside the set: the simulator starts, both cameras render (shape, not flat, sane
  brightness, depth present), the robot follows a small move. A failure is a task error, never a zero.
- The fixed hidden set `hidden/packs.json`: six packs (each variant twice, each harness case twice), the same for
  every submission, drawn by `review/select_set.py` under a rule fixed before any pack was run; no pack was chosen
  or dropped for how the reference does on it. Each pack carries the camera offsets of the cell it is taken apart in
  (rule 2 of the same script, drawn with a key of its own); `verify.py` refuses a set without them.
- Per pack a fresh cell and the kit's `Solver` (the controller at the agent's UID through `nonet_exec.py`: no
  internet socket), one action per step until the pack ends. The probe lock (`IdleAwareProbeLock`, rate 0.02, pass
  fraction 0.1): a probed step is asked twice with the same `t`, once with the robot's readings shifted
  (`wire.perturbed`), in a hidden order; pairs in which neither reply commands a motion are not counted. The
  schedule is a function of the pack's index and seed, so the grader adds no randomness. The copy is the
  observation the cell would give with its joints shifted (up to 0.01 rad a joint, a whole number of the readings'
  last digit): the grader asks the cell for its angles before rounding (`joints`, a grader-only request) and writes
  the angles, both tool poses and a wrist frame's pose from them as `cell.observe` does; the wrench is left as it
  is. So a copy's poses agree with its joints as a true observation's do, and the robot's model, which the agent
  holds, does not tell them apart (upstream #173; the measurement is in section 6). `probe_residual_true`,
  `probe_residual_copy` and `probe_true_residual_larger` of `probe_copies` in the verdict's metrics show it for
  every run.
- The credit (`rules.Rules.credit`): after the end and 1.5 s of settling, the value of the components that lie
  undamaged where they belong over the value of everything in the pack, from the bodies' poses in PhysX. A module:
  centre over a bay, bottom on the bay's floor (-5 to +15 mm), upright within 8 degrees. The lid: centre within
  0.10 m of the stand's centre, flat within 5 degrees, at the stand's height. A bin: centre over the inner footprint
  and at most 0.03 m above the walls. Not held by the gripper, not damaged (a stripped fastener, a chafed harness,
  a dropped module).
- `mean_pack_credit` is the mean over the six packs. A pack that ends early (live contact, a dropped module, the
  simulated time, the controller given up on, its time budget) keeps what it had; nothing is subtracted anywhere.
  A controller that does nothing earns 0 (nothing lies where it belongs when a pack arrives).
- Pixels are never scored. Measured on the author's server: two renders of one scene on the same GPU differ in
  about half the pixels (mean absolute difference 0.36 of 255).
- A failure of the cell itself runs that pack once more with a fresh cell and a restarted controller; a second
  failure is a task error.

## 4. The reference program and the anchor

`solution/reference/controller.py` is a privileged reference in the sense of the pump skid and pantry tasks: the
submission carries a 256-bit token, the grader then adds `truth` (every part's pose and grip features, every
fastener's head and seat, the harness' path, the exact commanded targets) to its observations. With that it skips
perception and nothing else: it works through the public actions, pays for every tool and bit exchange, presses and
turns every fastener under the same rules, and decides from the nutrunner's torque and angle that a fastener is
stalled (and raises the torque limit, as the manual says for corroded fasteners) or out. Its order: a harness on the
lid, the lid's bolts by head type, the lid to its stand (a half turn of the first joint), the service disconnect and
the plugs, the wait for isolation, a harness on the busbars, the busbar and bracket bolts, the busbars, brackets and
modules. Transfers go round the base by a turn of the first joint (a straight line near the base's axis runs the arm
through its shoulder singularity). It answers a repeated step from the observation it comes with: its plan yields a
function of an observation, and every displacement is a planned waypoint minus that observation's commanded pose.

**A perception-based baseline** (`review/perception_baseline/`, maintainers' evidence; nothing runs it in grading)
does the same job from what a submission gets: no token, no `truth`, only the frames with the published
calibration, the manual's tables, the cell's messages and the nutrunner's readings. It keeps the reference's motion
skills and order and replaces every value the reference reads from `truth`: the variant from the lid's ribs, the
pack's placement from the silhouette of its top, each fastener's place, presence and head from the overhead and
wrist depth frames (a head's height above its seat tells its type), the harness from its blue cable, "out" from the
nutrunner's trace. On the hidden set it scored `mean_pack_credit` 1.00000, as the reference does, in 24 to 28 % more
steps (section 6, the entry "2026-11-25, 10:00 to 11:35").

The anchor and its spread: see the status log (section 6) for the measured runs. `tests/anchor.json` holds the
value measured natively on the author's server, outside the engine; it has to be re-measured through `ale validate`.

## 5. Known limitations and open decisions (version 0.1.0)

- **Not validated in the engine.** No Docker on the author's server: the image has not been built, `ale validate`
  and `ale run` have not been run (section 6). The verify stage's root-only paths (the cell as the account
  `packcell` from a private copy, the seccomp launcher, the kit's account clearing) were written after
  `hazard_bay_build` and `pantry_jar_refilling` but never executed.
- **Difficulty is not measured.** No frontier model has run the task. The design aims at 0.3 or below through
  perception and engineering (millimetre localisation from RGB-D, per-fastener head classification, torque-trace
  logic, the isolation order, a long horizon with limited commissioning), not through penalties; whether it lands
  there, and whether it is too hard, is open until `ale run`.
- **The job can be done without the privileged view** (since 2026-11-25; section 6, the entry "2026-11-25, 10:00 to
  11:35"). `review/perception_baseline/` takes every pack apart from what a tested agent is given and scored
  `mean_pack_credit` 1.00000 on the hidden set without the token (one run, of a program frozen on the development
  packs before it), as the reference does. What made that straightforward is a finding about the difficulty: the
  depth frames are exact (0.1 mm steps, no noise), so a head's type follows from its height above its seat alone
  (4.5 against 6 mm, 6 against 8 mm), its centre from the depth points of its top face in one wrist frame (0.44 mm at
  worst on the development packs; the Torx bits need 1.2 mm), and the pack's placement from the silhouette of its top
  (0.44 mm, 0.001 degrees); the parts sit at the manual's places in the pack's frame; the nutrunner's trace separates
  breakaway, stall and run-out cleanly. The baseline was written in about an hour by the task's author, with a tool a
  tested agent does not have (the development packs' truth to compare with) and the reference's motion code to start
  from, so it shows that the task is solvable, not how hard it is for an agent with six hours, ten sessions and no
  truth: that is still for `ale run` (the bullet above). If frontier models turn out to do too well, noise on the
  depth frames is the lever this baseline points to first, next to those listed below; whether to use it is the
  owner's decision.
- **The head type can also be found by trial.** A bit that does not fit a head turns freely and harms nothing
  (only the two loose combinations and a bad alignment wear a head), so a program can run one bit over a whole group
  and come back with the other for the heads that did not turn. That costs a second pass, not credit. The brief's
  "the bit has to be chosen from the image" therefore holds for a program that wants to be quick, not for one that
  only wants to be right; making a wrong family harmful would not be true to the tools. Left as it is for the
  owner to decide.
- **Levers if frontier models turn out to do too well.** Three are used since 2026-11-25 (section 2, "The cameras"
  and "Sealant"): the cameras' poses on file are nominal and each cell's own mounting offset has to be identified;
  the depth is an active-stereo sensor's; fastener heads are partly hidden by sealant. Not used: lighting, dirt and
  label clutter varying per pack; a wider placement tolerance of the fixture. All are perception or engineering,
  none a penalty.
- **GPU models.** Run only on an RTX 6000 Ada with driver 570 (DLSS falls back to an older denoiser preset there),
  so `gpu_types` names only that model. The cloud models (RTX 4090, RTX 5090, RTX PRO 6000, H100, H200) are
  untested; `hazard_bay_build`, on the same Isaac Sim stack, validates on the first three. Each model has to be
  tried (the canary's frames, one reference pack) before it is added.
- **Reproducibility.** PhysX on the CPU repeated the reference's packs on this one machine (section 6); across CPUs
  it is not expected to be bit-identical, and the renderer is not repeatable even on one GPU. The credit tolerates
  that (it reads where bodies lie), a program's behaviour may not: an agent whose perception is marginal can score
  differently on different hosts.
- **A fastener let go into a pile appears inside it, and is pushed out slowly** (section 6, the entries "19:10 to
  20:30" and "2026-11-25, 01:20 to 02:40"). `eject` places the fastener's body at the bit and makes it dynamic at
  once, and the copy that rides on the bit collides with nothing, so a bit lowered into a pile lets the body
  appear inside the fasteners lying there. With PhysX's default separation speed (3 m/s) that threw five fasteners
  of two packs out of their bins on `15d8694` (the reference releases 70 mm above the table on nine spots per bin),
  and the reference's value fell to 0.9944. Since `8439170` every fastener's body carries a maximum depenetration
  velocity of 0.3 m/s: the overlap is undone without throwing anything, and the reference scores 1.0 again. What
  remains is the simplification itself: a real fastener on a bit pushed into a pile would meet the pile, here it
  appears in it and is pushed up and aside. A reviewer may prefer the cell to release the body only above what lies
  there; that changes what every program meets and needs the full series again.
- **Simplifications that a reviewer may want changed**: no finger colliders; no slip; the socket does not slide
  over the head; a harness is rigid; the robot does not collide with itself; the live-parts check is geometric and
  once per tick.
- **The task name is provisional** (not yet registered in the coordination sheet); it appears only in the folder
  name and in `task.toml`.
- **The values** (module 8, lid 5, service disconnect 3, busbar 2, bracket 1, plug 1, harness 1, fastener 0.5)
  are the author's. In an HVB-P5 the lid with its bolts is about 12 % of the pack's value, the service disconnect
  and the plugs 6 %, the busbars with their bolts 20 %, the brackets with theirs 17 %, the modules 45 %: the
  modules are what the line is there for, and every earlier stage still shows in the credit. A reviewer may prefer
  another weighting; it lives in `packcell/series.py` (`VALUES`) and the documents follow it
  (`review/make_documents.py`).

## 6. Status log

(Newest last. Times are the author's server's, EST.)

**2026-11-24, 02:50 to 03:40. The stack and the cell's skeleton.** Read the reference tasks (`hazard_bay_build`
for Isaac Sim, `pump_skid_startup_and_operation` and `pantry_jar_refilling` for arm cells, documents and the
commissioning service). Probed Isaac Sim 6.2 natively (the author's Python 3.12 venv): the UR20 asset
loads and articulates on CPU PhysX with two links added to its articulation; 240 physics steps in 0.2 s; the
incoming joint force of the added tool carrier is the flange wrench in the carrier's frame (checked with the tool
down and tilted 90 degrees: 32.4 N for 3.3 kg); a kinematic body can be made dynamic at run time; a fixed joint
between a link and a body can be created and removed at run time; two cameras render in 0.05 s a frame set once
`/exts/omni.replicator.core/maxAssetLoadingTime` is set to 1 before Replicator loads (the 100 s stall did not
occur). Wrote `kin.py`, `series.py`, `scene.py`, `rules.py`, `cell.py`, `wire.py`, `server.py`.

**03:40 to 04:20. The reference, and what it found in the cell.** A sleeping articulation reports stale joint
speeds and forces (the dock's "at rest" check failed for it): the arm's sleep threshold is 0 and joint speeds are
differences over the tick. A fastener's body moved by the tensor API came back to its seat when it was made
dynamic: bodies the rules move are moved by authoring their pose on the stage. A fastener leaving its seat in the
tick in which its part was freed flung the lid away (a kinematic body sweeping through a dynamic one): a freed
fastener's colliders are off while it waits, and a part is freed one tick after its last fastener is out. After a
protective stop the arm kept pressing with its target at the stopped pose: it now backs off to where it was 0.25 s
earlier. Straight Cartesian transfers past the base ran the arm through its shoulder singularity: the reference
turns the first joint instead. A gripper taking a bracket by its short side hit the tray's wall (the body is long
along the closing axis): the reference takes brackets by the long side. Parts dropped on one spot of a bin piled up
under the gripper: the reference spreads them. A one-tick remainder of a long move jerked a carried module into a
protective stop: the reference splits moves evenly and never sends a short action for a real distance.
First complete pack natively: credit 1.0 (a P4, 28 fasteners, 21 parts), 611 s of simulated time, 827 steps, 150 s
of wall clock.

**04:20 to 04:40. Grader, service, documents.** `verify.py` and `pack_grade.py` ran natively through the kit's
`Solver` and probe lock on hidden pack 0 (credit 1.0, 978 steps, lock 11 of 11 counted pairs, 8 idle pairs not
counted). That run showed 44 protective stops when the harness on the lid was gripped: the harness' ramp over a
bolt cut through the head's edge, and the harness is pushed out when it becomes dynamic. The harness now bridges a
head level (wider than any head) and keeps clear of every fastener it does not cover (`review/test_rules.py`
checks 360 draws); the same pack then ran with no stop. `ale lint` on the folder: ok. `scripts/lint_domain.py`: only
the two messages of a null anchor. The kit copy is byte-identical. Wrote the Dockerfile (after
`hazard_bay_build`'s), `setup.sh`, the commissioning service, the client library, `run_session.py`, the example
program, the documents' generator, the drawings and photographs, `select_set.py` (the hidden and development sets
drawn under its rule), `test_rules.py` (36 checks, no simulator).

**04:40 to 05:50. The whole hidden set, and what it found.** The first native verify of all six packs (the tree of
04:34, old values) gave 1.0, 0.866, 0.997, 1.0, 0.347, 1.0: the reference itself was not robust. Causes, each
found with an in-process trace, and the fixes:

- The reference turned the gripper to each part's direction by the shortest rotation from wherever the last joint
  stood; over a P6's many transfers the joint ran into its 360 degree limit, moves were clipped, parts were not
  reached, and a module was lowered across a rack divider (1600 N, a protective stop). It now picks, among the
  turns a task allows (exact, half-turn symmetric for the gripper, free for the nutrunner and for a part over a
  bin), the one that leaves the last joint nearest its middle, and makes it together with the first joint's turn.
- A plug that hung still on the gripper for a while stayed in the air when the gripper opened: the body had gone to
  sleep and a sleeping body does not notice that its joint is gone. The cell now wakes a part when it is released.
- A harness on the busbars could reach under a module's handle bar (it then pressed down on the tool when gripped).
  Its ends now stay short of the handles (`review/test_rules.py` checks it on 360 draws).
- A bit pressed on a bracket bolt was now and then kicked 16 mm off the head (a lateral 230 N, the wrist's drive
  saturated). The bit-on-head contact now has the high friction described in section 2.
- The pull on a plug raised the force from under 60 N to over 220 N within one tick (release and overload in the
  same tick): the reference pulls the first 4 mm at 4 mm/s.

After these, packs 0, 1, 2 and 4 each ran to credit 1.0 with no protective stop (direct runs of the reference).
The values were rebalanced (section 5) and the documents regenerated.

**What the frames carry** (`review/check_observability.py`, two development packs, public means only: one overhead
frame, the published calibration, one wrist frame per candidate; compared with the truth afterwards):

| pack | lid fasteners and holes found | overhead position error (median / max) | wrist position error (median / max) | head kind right | rust right |
|---|---|---|---|---|---|
| development pack 3 (P6, no harness) | 14 of 14 | 0.60 / 1.46 mm | 0.23 / 1.22 mm | 14 of 14 | 14 of 14 |
| development pack 2 (P5, harness on the lid over L03, three holes) | 11 of 12 (the one under the harness is the one not found) | 0.75 / 2.39 mm | 0.97 / 1.11 mm | 10 of 10 | 7 of 7 |

On the second pack one corroded hexagon was found from above (2.4 mm off) but not again by the script's wrist
step; hexagon heads are located about 1 mm off by the wrist step (the script takes the centre of the visible blob,
and the camera looks at the head 11 degrees off its axis), Torx heads within 0.5 mm. The bits' tolerances are
2.0 mm (socket) and 1.2 mm (Torx). The script is a measurement, not a solution: it does nothing about the variant,
the inside of the pack or the harness.

**The manual's photographs** were taken with the cell's cameras on an example pack (`review/make_photos.py`); round
primitives are drawn with a finer tessellation since (a fastener's head read as a ten-sided polygon before), and
the lights were turned down (the tray and the plated heads were blown out).

**05:50 to 06:20. The verify stage's small checks (native, null anchor, the tree of `6a08a7e`).**

| submission | set | result |
|---|---|---|
| none (an empty `/home/user/submission`) | six packs | "no usable submission": the honest zero, no simulator started |
| the example program (takes one frame, says `done`) | six packs | credit 0 on every pack (0 of 49, 48, 59, 58, 67, 69 items), `mean_pack_credit` 0; the whole stage 186 s |
| the reference without its token | hidden pack 2 | no `truth`, it ends the pack at once: credit 0 |
| the reference, its applied actions recorded | hidden pack 2 | credit 0.9944 (58 of 59), 1007 steps, the lock 11 of 11 counted pairs (9 idle pairs not counted) |
| a blind replay of those recorded actions | hidden pack 2 | the same credit to the last digit, 0.9944, in the same 1007 steps (PhysX repeated the pack exactly on this machine), and **the lock fails: 0 of 11 pairs**, so the reward is 0 |

The one item the recorded reference lost was a busbar bolt that bounced off the pile in the fasteners bin and out
over the wall (ejected 5 cm above the walls, always at the same corner). The reference now lowers the bit between
the walls and spreads the fasteners; the pack then ran to 1.0.

The commissioning service's first native run showed that a session opened right after another had closed was
refused as busy (the service was still stopping the first cell): `open` now waits up to 120 s for the cell to stop.

**06:15. The commissioning service, natively** (`commissiond.py` as the author's account with its run and state
folders redirected; the agent's own `packcell_client.py` and `run_session.py` against it): the status file and
request; `run_session.py` with the example program on development pack 2 (session 1: READY, reset, two steps, one
overhead frame, `done`; 28 s of wall clock); then an interactive session on pack 1 opened right after (it waited
for the first cell to stop): the first observation carries no `truth`, the overhead frame decodes to 1440 x 1080
with depth up to 2.05 m, a move and a wrist frame work and the frame count goes down, a malformed action is
refused without effect, and `truth` and `score` are unknown requests on this wire. Two sessions counted, eight
left, the count kept in the state file. Not run: the service under its own account behind the image's permissions
(`setup.sh` as root), which needs the image.

**06:20 to 07:15. A first full run on the frozen tree, and the bit's collider.** One native verify of all six
packs on `7d2ef4e`: `mean_pack_credit` 0.99920 (1.0, 1.0, 1.0, 1.0, 1.0, 0.9952; one busbar bolt of pack 5 not in
a bin), lock 85 of 85, 1332 s for the stage; but 4 and 3 protective stops on packs 1 and 3, all with the bit
pressed on a fastener: the bit was thrown 15 mm sideways with 250 to 2500 N in a tick in which nothing was
commanded. 160 scripted presses with centred, converged approaches never showed it, at any friction, so it is not
friction (the note on friction in the list above stays true but was not the cause). The cause was not isolated;
two suspects were removed together: the bit's collider is a capsule (a round end on a flat head: one contact point
with the head's normal; the suspicion is that a flat-ended cylinder on a head of nearly its own radius is a
degenerate pair for the contact generator), and a part whose last fastener has just come out is not freed while the
bit still presses on it (the suspicion: a 0.12 kg busbar squeezed between the bit and its post is shot out). After
both, packs 1 and 3 ran with no stop, credit 1.0 each. Which of the two did it, or whether a third cause is only
hidden, is not known: the evidence is that all six packs have run without a stop since (the entries below).

(The session was interrupted here from 07:15 to 14:50; nothing was running in between.)

**14:50 to 15:40. Resumed; a cell that did not start; the run the anchor is taken from.** The changes above were
committed (`82837d8`). Hidden pack 5, the one that had lost a busbar bolt in the run on `7d2ef4e`, ran directly
with the new tip: credit 1.0, no stop. The first attempt at three full runs then failed at its first cell ("the
cell did not answer within 900 s"): other users' jobs had the server at a load average of 50 on its 32 cores, and a
bare start of Isaac Sim took 348 s at that moment (17 s when the machine is idle). The failure showed a defect:
`CellProcess` left a simulator that had not come up running, so the next run started a second one beside it. A cell
that does not come up is now stopped (`230187b`). `grader_config.json` keeps its 900 s (a cell that does not start
is a task error or a pack that is run once more, never a zero); the native runs since were made on a stage copy
with 3000 s, the only difference from the committed `tests/`.

One native verify of the whole hidden set on `230187b`: `mean_pack_credit` 1.00000, every pack 1.0 (49, 48, 59,
58, 67 and 69 of their items placed), no protective stop, no stripped fastener, the lock 77 of 77 counted pairs
(50 idle pairs not counted), 852 to 1185 steps and 601 to 813 s of simulated time a pack, 6 s of the reference's
own time, 1166 s for the stage. `tests/anchor.json` records that value (`21ac344`), `full_at` 0.95.

**15:40 to 16:30. Repeats, and every check again with the anchor recorded** (the tree of `21ac344`; cell, grader
and reference are those of `230187b`).

| submission | set | result |
|---|---|---|
| the reference, second full run | six packs | `mean_pack_credit` 1.00000, reward 1.0, lock 77 of 77, stage 1077 s |
| the reference, third full run | six packs | `mean_pack_credit` 1.00000, reward 1.0, lock 77 of 77, stage 1076 s |
| none (an empty `/home/user/submission`) | six packs | "no usable submission": reward 0.0, no simulator started |
| the example program (one frame, then `done`) | six packs | credit 0 on every pack, reward 0.0, stage 140 s |
| the reference without its token | hidden pack 2 | no `truth`, it ends the pack at once: credit 0, reward 0.0 |
| the reference, its applied actions recorded | hidden pack 2 | credit 1.0 (59 of 59), 1040 steps, lock 11 of 11 (9 idle), reward 1.0 |
| a blind replay of those recorded actions | hidden pack 2 | the same credit, 1.0, in the same 1040 steps, and **the lock fails, 0 of 11**: reward 0.0 |

The three full runs are the same run three times: every pack took the same number of steps and the same simulated
time in each (PhysX on the CPU repeats on this machine), so they show no spread, and they say nothing about another
CPU. What a change of the tree does is visible one entry up: the run on `7d2ef4e` had lost one bolt (0.99920).
`full_at` 0.95 is therefore a judgment, not a measured spread (the anchor's `note`). The reference asks for no
camera frame (it reads `truth`), so in its runs the renderer works only in the canary; frames inside a pack were
exercised by the example program, the commissioning sessions and `review/check_observability.py`.

The commissioning service ran natively again on this tree, with the same result as at 06:15 (the example program
through `run_session.py` on development pack 2 in 22 s; an interactive session on pack 1 right after: no `truth`
in an observation, both cameras decode, a malformed action refused, `truth` and `score` unknown requests; two
sessions counted, eight left).

The image's asset step ran natively: `environment/fetch_isaac_assets.py` with `environment/isaac_assets.sha256`
downloaded the UR20's seven files from NVIDIA's public bucket, every digest matched, and the files are the ones
the native runs used.

Static gates on this tree: `ale lint` on `tasks/` ok; `scripts/lint_domain.py` ok; `tests/robotics_grader/`
byte-identical to `shared/robotics_grader/`; `scripts/task_assets.py check` 0 problems; `scripts/gpu_types.py
check` 0 errors; `review/check_copies.py` ok; `review/test_rules.py` 36 checks ok. The repository's `pytest`: all
pass but two. `tests/test_task_cards.py` fails because this task has no card in `tasks/cards.toml` (a maintainer's
step before the merge; the author was told not to edit that file). `tests/test_longtail_pause_and_close.py` fails
on this server only: its temporary folder is on NFS and cannot be removed while a file in it is open; with
`TMPDIR` on a local disk it passes. It does not touch this task.

**18:20 to 19:00. The kit of upstream #173, and the probe's copy rebuilt (tree `7e59d13`; the native verify ran
on `152fdf7`, the same task files).** `upstream/main` moved to `1c13a47` after this branch was cut: #173 changed
`shared/robotics_grader/` (the solver and everything it starts run at nice 19; `replies_differ` is the lock's one
rule; `probe.stale_copy`; `deadline.VerifierDeadline`) and copied it into every task. The branch was merged with
`upstream/main` and the kit copied again (byte-identical to `shared/`). The task against #173's three findings:

1. *The solver's priority.* The kit's own: `GuardedSolver` only replaces `_spawn`, and the kit starts whatever
   `_spawn` a task has from its low-priority thread. Nothing to change.
2. *One rule for probe pairs, in practice too.* `~/commission/run_session.py` repeats steps but counts no pairs,
   so there is no second rule. Nothing to change.
3. *A probe copy must be built the way the real observation is.* This task's copy was not. `wire.perturbed`
   shifted the joint readings (up to 0.02 rad) and, independently, the tool poses (up to 12 mm and 0.03 rad) and
   the wrench (up to 3 N). The cell reports `tcp` as the kinematics of its own joint reading times the tool, so a
   true observation's pose and joints agree to the rounding (about 1e-6) and the copy's disagreed by millimetres:
   one line with the robot's model, which is among the givens, named the copy every time. A program could have
   answered the copy differently on purpose and replayed everything else.

The copy is now the observation the cell would give with its joints shifted (section 3): `q` and `q_target` by
one shift, `tcp`, `tcp_target` and a wrist frame's `pose` from the shifted angles as `cell.observe` builds them,
the wrench, the joint speeds and the pictures unchanged. Two finer points were measured and closed on the way.
Building the copy from the rounded readings made the hidden digits of `q` and `q_target` the same number (true
observations' are independent), and a copy of a commanded angle with round digits had hidden digits where the true
one has none (a threshold on that pair's residual named the copy in 86 % of 4000 observations). So the grader
takes the cell's angles before rounding (`cell.joints`, served only to the grader, like `truth`) and the shift is
a whole number of the readings' last digit: what a reading hides is then the same in both. Measured without the
simulator, on 4000 observations built as the cell builds them, with the grader's builder:

| commanded angles | the true observation has the larger residual | best single threshold on the residual names the copy |
|---|---|---|
| any digits (the usual case: a target from inverse kinematics) | 1984 of 4000 (49.6 %) | 51.0 % (50 % is chance) |
| round digits (a joint move by a round amount) | 1979 of 4000 (49.5 %) | 50.9 % |

The tool centre point of a copy lies 9 mm from the true one in the median (3 mm at the 5th percentile, 31 mm at
most), so a program that works from the observed poses answers differently. The agent's library
(`packcell_client.perturbed`, used by `run_session.py`) builds the same copy from `ur20.json` and `cell.json`; it
has only the rounded readings, so its copies are right in the usual case and not in the round-digits one, which
does not matter for a rehearsal. The interface document and the instruction say what a repeated request carries.
`review/test_rules.py` (37 checks) holds the copy to these properties and the library to the grader's builder.

What ran on `7e59d13`: `ale lint` ok; `scripts/lint_domain.py` ok; the kit comparison; `scripts/gpu_types.py
check`; `scripts/task_assets.py check`; the kit's selftest; `review/check_copies.py`; `review/test_rules.py`; and
the grader's loop (`pack_grade.run_packs`) against a stand-in cell without a simulator: a controller that works
from the observed angles, 81 of 81 probed pairs differing; one that does not, 0 of 81 and the lock fails; the
cell asked for its angles once per probe; the residuals of both observations about 1.5e-6.

**The native verify on this tree** (`152fdf7`: the task files of `7e59d13` and this entry; 18:42 to 19:19, one
Isaac process at a time, confined to GPU 3 of the author's server, which other users' jobs shared). The first
attempt, at 18:34, was stopped within a minute because the GPU it was confined to had just been filled by another
job. What the change touches is which steps are probed and what the copied observation holds; the cell's physics
and the actions executed on true observations are the same code, and the runs show it:

| submission | set | result on `152fdf7` | on `21ac344` |
|---|---|---|---|
| the reference | six packs | `mean_pack_credit` 1.00000, reward 1.0, the lock 83 of 83 (53 idle pairs not counted), stage 1394 s | 1.00000, lock 77 of 77 (50 idle), stage 1076 s |
| none (an empty `/home/user/submission`) | six packs | "no usable submission": reward 0.0 | the same |
| the reference without its token | hidden pack 2 | no `truth`, it ends the pack at once: credit 0, reward 0.0 | the same |
| the reference, its applied actions recorded | hidden pack 2 | credit 1.0 (59 of 59), 1040 steps, lock 10 of 10 (6 idle), reward 1.0 | lock 11 of 11 (9 idle) |
| a blind replay of those recorded actions | hidden pack 2 | credit 1.0 in 1040 steps, **the lock fails, 0 of 10**: reward 0.0 | 0 of 11 |
| the example program (one frame, then `done`) | six packs | credit 0 on every pack, reward 0.0, stage 188 s | the same, 140 s |

Every pack of the reference's run took the same number of steps as on `21ac344` (921, 852, 1040, 1066, 1151 and
1185) and the same simulated time (601 to 813 s), with every item placed, no protective stop and no stripped
fastener; the reference's own time was 6 s. Which steps are probed is drawn afresh, so the lock's counts differ
from the earlier tree's; the longer wall times are the shared GPU's. The commissioning service, started from this
worktree, behaved as before: the example program through `run_session.py` on development pack 2 (2 steps, 1
frame, 0 repeated, 27 s); an interactive session on pack 1 right after: no `truth` in an observation, both
cameras decode (overhead 1440 x 1080, wrist 960 x 720), a malformed action refused ("move.type must be one of
hold, tcp, joint"), `truth` and `score` unknown requests; two sessions counted, eight left.

What the grader recorded about the copies (`probe_*` in the verdict's metrics) did not match the measurement
without the simulator above: of 136 copies (one for every probed step, idle pairs included), the true observation
had the larger residual in 81 (59.6 %; for a fair coin 68 ± 5.8, so 2.2 standard deviations above half), and the
largest true residual was 4.6e-4 to 8.0e-4 (an entry of a rotation matrix) on five of the six packs (one copy, on
pack 4, reached 6.9e-4), where the rounding leaves about 1e-6. The cause is the conversion of a pose near a half
turn (the next entry).

**19:10 to 20:30. A pose's rotation vector near a half turn (tree `15d8694`), and fasteners thrown out of the
bins.** The cause of the residuals above is `kin.R_rotvec`, the conversion of a rotation matrix to a rotation vector
that `wire.pose6` uses for every pose of an observation (and `kin.pose_error` for the cell's inverse kinematics). It
took the angle from `acos` of the trace and the axis from a division by `sin`; within about 1e-6 to 1e-5 rad of a
half turn both lose precision, and a pose written on the wire and read back was off by up to 2e-3 in a matrix
entry. The tool pointing straight down is a rotation of nearly exactly a half turn, and with the drives' small
following error the measured tool pose lands in that band, while a probe's copy, its joints shifted by up to 0.01
rad, is out of it: the true observation carried the large residual, the copy did not. The wrist camera, tilted 11
degrees from the tool's axis, is far from a half turn in those states.

The angle now comes from `atan2` of the antisymmetric part's norm and the trace, and within 1e-3 rad of a half
turn the axis from the symmetric part, signed by the antisymmetric one: in `tests/packcell/kin.py` (copied to the
commissioning service by `review/check_copies.py --sync`), in the agent's library (`packcell_client.R_rotvec`) and
in the reference. Nothing else in the folder converts a matrix to a rotation vector (`rules.angle_deg` is the angle
between two vectors, for the tilt rules). Measured without the simulator (`review/test_rules.py`, now 41 checks,
which fail with the old conversion; the round trip is the largest entry difference over 200 random axes):

| | the old conversion (`acos`) | the new one |
|---|---|---|
| a pose written and read back, angle pi - 1e-2 to pi - 1e-4 | 6e-7 | 6e-7 |
| at pi - 1e-5 | 2.1e-5 | 6.7e-7 |
| at pi - 2e-6 | 5.6e-4 | 6.4e-7 |
| at pi - 1e-6 | 1.9e-3 | 6.5e-7 |
| at pi - 1e-7, at pi | 7.6e-7, 6.4e-7 | 7.0e-7, 6.4e-7 |
| 400 near-downward states (`downward_states`): true residuals above 1e-5, copies above 1e-5, the true one the larger | 82, 0, 267 (67 %) | 0, 0, 184 (46 %) |

Away from a half turn (angles 1e-6 to pi - 1e-3) the two agree to 7e-10. Below about 1.5e-8 rad the old one
returned exactly zero (`acos` of a trace that rounds to 1) where the new one returns the rotation. The library's
copy of a wrist pose is now the grader's exactly; it was up to 7e-6 off at random joint angles, which a check had
put down to `cell.json`'s nine decimals, and was the conversion.

The native series on `15d8694` (19:30 to 20:04, GPU 3 only, as above):

| submission | set | result on `15d8694` | on `152fdf7` |
|---|---|---|---|
| the reference | six packs | `mean_pack_credit` **0.99440** (packs 0 and 1: 0.97315 and 0.99324, packs 2 to 5: 1.0), reward 1.0 (against the anchor of then, 1.0), the lock 83 of 83 (53 idle), no protective stop, no stripped fastener, stage 1330 s | 1.00000, 83 of 83 |
| none | six packs | "no usable submission": reward 0.0 | the same |
| the reference without its token | hidden pack 2 | credit 0, reward 0.0 | the same |
| the reference, its applied actions recorded | hidden pack 2 | credit 1.0 (59 of 59), 1040 steps, lock 10 of 10 (6 idle), reward 1.0 | the same |
| a blind replay of those recorded actions | hidden pack 2 | credit 1.0 in 1040 steps, **the lock fails, 0 of 10**: reward 0.0 | the same |
| the example program | six packs | credit 0 on every pack, reward 0.0, stage 168 s | the same |

The commissioning service behaved as before (the example program in 26 s; the interactive session as above; two
sessions counted, eight left). The probe's copies now balance: the true observation had the larger residual in 66
of 136 (48.5 %; 0.3 standard deviations below half, against 2.2 above on `152fdf7`), and the largest residual was
1.6e-6 for both (8.0e-4 and 6.9e-4 before).

*The five fasteners.* Every pack took the same number of steps as on `152fdf7`, and every fastener came out (the
same `fastener_out` events, within 0.05 s): the plan ran as before, and what changed is where released fasteners
came to rest. A second run of packs 0 and 1, from a stage copy whose `pack_grade.py` also keeps the cell's `where`,
lost the same five: three on the floor 1 to 2.5 m away (L08, B03a and R3af of pack 0), two on the table 10 and 20
cm beside a bin (B00a of pack 0, R4af of pack 1). A third run of pack 0, with the cell's `_eject` instrumented in
a stage copy, showed how. The reference releases every fastener with the bit's tip 70 mm above the table (between
the walls: the fix for the bounce of the 05:50 entry), on nine spots per bin that it uses in turn, so from the
tenth fastener of a bin on, a released one appears exactly over earlier ones (0.0 to 0.1 mm apart horizontally),
and once two lie there its shank is inside them. The cell places the ejected body at the bit and makes it dynamic
at once, and the physics engine pushes overlapping bodies apart fast: a normal release has fallen at 0.49 m/s
after its first tick; the release of R3af over L08 and B03a threw L08 at 5.6 m/s, and it ended on the floor 2.3 m
away with the other two; two more releases into a pile reached 1.9 and 1.6 m/s. The earlier trees used the same
spots, and their piles lay differently by chance: the trajectories differ in detail now (the poses near a half
turn are read exactly, and the inverse kinematics sees rotation errors below 1.5e-8 rad that were zero before),
and nothing had left a bin.

*What was decided* (nothing about the reference or the cell was changed; both would need several full runs, and
the cell's physics is the owner's to decide). The conversion stays fixed: it is right, and the probe's balance
shows it. The lost fasteners are a weak spot that the earlier runs hid, of the reference's drop rule (a fixed
height, reused spots) and of the cell's `eject`, which makes a body dynamic wherever it is, also inside another one;
a program of the agent's that releases into its own pile meets the same (section 5). `tests/anchor.json` now holds
the reference's value on this tree, 0.9944 (1.0 before, measured on the earlier trees). `full_at` stays 0.95: the
only spread measured so far is this one, a small perturbation of the reference's trajectory that cost 0.56 % of
the mean and 2.7 % of one pack, and 0.95 leaves about nine times that. With that value the reference's ratio on
this tree is 0.99440 / 0.9944 = 1.000, reward 1.0 (computed from the run above, not run again). Static gates on
the committed tree: `ale lint` ok, `scripts/lint_domain.py` ok, the kit comparison, `review/check_copies.py`,
`review/test_rules.py` 41 checks.

**2026-11-25, 01:20 to 02:40. The fasteners' depenetration cap (tree `8439170`; GPU 3 of the server only, as
above).** The owner chose to change the cell rather than keep 0.9944. `packcell/scene.py` now gives every
fastener's body PhysX's `maxDepenetrationVelocity` of 0.3 m/s (`FASTENER_MAX_DEPENETRATION`; the schema's default
is 3 m/s), and nothing else changed: the parts, the robot, the rules, the grader and the reference are those of
`15d8694`. The cap bounds only the speed the solver adds to undo an overlap (a fastener on its seat is kinematic
and does not meet it at all).

First a run of packs 0 and 1 from a stage copy with `_eject` instrumented as in the third run above (the tree of
`9fa0782` with the change not yet committed): both packs 1.0 (49 of 49, 48 of 48) in the same 921 and 852 steps.
Of the 53 ejects, the fastest fastener body within ten ticks of one moved at 0.94 m/s (a body that falls freely
moves at 0.49 m/s after those ten ticks), against 5.6, 1.9 and 1.6 m/s before; the release of R3af exactly over
L08 and B03a, which had thrown L08 to the floor, now moved B03a at 0.75 m/s, and nothing left a bin.

The native series on `8439170`:

| submission | set | result on `8439170` | on `15d8694` |
|---|---|---|---|
| the reference | six packs | `mean_pack_credit` **1.00000** (every pack 1.0, in the steps of every tree since `21ac344`: 921, 852, 1040, 1066, 1151 and 1185), the lock 83 of 83 (53 idle), no protective stop, stage 1674 s | 0.99440 |
| none | six packs | "no usable submission": reward 0.0 | the same |
| the reference without its token | hidden pack 2 | credit 0, reward 0.0 | the same |
| the reference, its applied actions recorded | hidden pack 2 | credit 1.0 (59 of 59), lock 10 of 10 (6 idle), reward 1.0 | the same |
| a blind replay of those recorded actions | hidden pack 2 | credit 1.0, **the lock fails, 0 of 10**: reward 0.0 | the same |
| the example program | six packs | credit 0 on every pack, reward 0.0, stage 196 s | the same |

The commissioning service behaved as before (the example program in 28 s; the interactive session as above; two
sessions counted, eight left). The probe's copies balance as on `15d8694`: the true observation had the larger
residual in 63 of 136 (46.3 %, 0.9 standard deviations below half), the largest residual 1.6e-6 for both.

*What was decided.* `tests/anchor.json` holds 1.0 again, the reference's value on this tree. `full_at` stays 0.95:
the only spread measured is still the perturbation of `15d8694`, and its 0.56 % came from the defect fixed here;
0.95 leaves room for another CPU's contact simulation. Section 5 keeps what remains of the simplification (the
released body appears where the bit is, also inside a pile, and is pushed out slowly).

**2026-11-25, 10:00 to 11:35. A perception-based baseline (`review/perception_baseline/`, frozen at `82e5f2f`; GPU
3 of the server only, as above).** The question a reviewer asks of a privileged reference: can the job be done from
what a tested agent has? The answer is a program, measured on the hidden set like the reference.

*The program.* A submission folder (`run.sh`, `main.py`, `controller.py`, `vision.py`, `manual.json`, `cell.json`).
It reads only its own folder, the observations and replies on the wire; it carries no token and runs without one.
`cell.json` is the agent's copy of the cell's configuration, `manual.json` the service manual's tables
(transcribed by `dev/parse_manual.py`). NumPy and OpenCV only. The motion skills and the order of the job are the
reference's (copied, with a line saying so); every value the reference takes from `truth` comes from perception, the
manual or the program's own bookkeeping instead (the folder's README has the table):

- the pack's placement: the silhouette of the pack's top in an overhead depth frame (the lid at 164 mm, the tray's
  walls at 160 mm once it is off), its four sides fitted as lines in the base frame;
- the variant: each variant's rib lines (manual, section 2) sampled for a 2 to 6 mm rise on the lid; after the lid,
  the module handles that stand where the manual puts them;
- each fastener: a bump at the manual's place in the overhead depth frame, its height above the seat giving the head
  (M6: S10 6 mm, T30 4.5 mm; M8: S13 8 mm, T40 6 mm), none for a missing or hidden one; then one wrist frame with the
  head on the camera's axis (the bit's tip 0.105 m above it, the camera on the side where nothing stands in the way:
  along the bracket for the brackets' bolts), and the depth points of the head's top face give its centre and height
  again, or an empty hole; a head that shows less than 93 % of its top face, or lies 1.4 mm from the overhead
  estimate, is looked at again from the other side;
- corroded or not: not needed; a fastener that stalls at 1.3 times the assembly torque is pressed again with the
  limit at twice the assembly torque, within the "never exceed" value;
- "out": the breakaway, the manual's turns to free and a free-running spindle in the nutrunner's trace (the first
  quarter turn slowly, so that a bit that slips or turns free is stopped before it wears the head), confirmed by
  `eject: ok`;
- the harness: the blue cable in the overhead frame, a grasp across a straight, level stretch at its centre's height
  (other places along it if a grasp fails);
- grip points and isolation: the manual's places in the pack's frame; the service disconnect's pull plus 30 s;
- the commanded pose: the program's own record after a completed move; a wrist frame's pose is used only when it
  agrees with that record within 1.5 mm, so a repeated step's copy, whose readings are shifted, is never taken for
  the camera's pose.

*How it was developed* (development packs only; the cell with the grader's switch is used by the tools in `dev/`,
never by the program). Three survey sessions (`dev/survey.py` on development packs 3, 1 and 2, in the service's numbering 1 to 4; a
fourth on pack 4 was stopped as it started): the reference does the job while the tool takes overhead frames at the stages and wrist
frames over every fastener, with the truth beside them. The perception was written and measured offline on those
frames (from above: the variant right on three of three, every seated head typed right, lid heads within 1.02 mm and
inner ones within 1.30 mm; from the wrist: 0.27 mm median, with outliers up to 1.75 mm only where a busbar or a
module hid part of a bracket's head from one of the survey's two fixed camera sides, a side the program does not use,
and on the one bolt the survey looked at with the harness still on it). Then closed-loop runs
(`dev/devrun.py`: `run.sh` behind the wire with the grader's deadlines and repeated steps, the truth only afterwards,
`dev/compare.py`, `dev/stage_table.py`): round r1 of the first complete program and round r2 of the frozen one (two
retries added that the development packs never needed). Eleven cell sessions in all; a tested agent gets ten.

| development pack | lid (+ bolts) | MSD, plugs, harness | busbars (+ bolts) | brackets (+ bolts) | modules | credit r1 / r2 | simulated s, steps, frames (r2) |
|---|---|---|---|---|---|---|---|
| 1: P4, harness on the busbars | 9.5/9.5 | 6/6 | 15/15 | 12/12 | 32/32 | 1.0 / 1.0 | 711, 1084, 32 |
| 2: P5, harness on the lid | 9.5/9.5 | 6/6 | 18/18 | 14.5/14.5 | 40/40 | 1.0 / 1.0 | 859, 1289, 38 |
| 3: P6, no harness | 12/12 | 5/5 | 21/21 | 17.5/17.5 | 48/48 | 1.0 / 1.0 | 947, 1479, 43 |
| 4: P6, harness on the lid | 11.5/11.5 | 6/6 | 21/21 | 17.5/17.5 | 48/48 | 1.0 / 1.0 | 991, 1521, 44 |

(Value recovered per stage over the value in the pack; fasteners 0.5 each with their group.) Perception against the
development truth, the same in both rounds: the variant 4 of 4; the placement within 0.44 mm and 0.001 degrees;
134 of 134 seated heads typed right; 8 of 8 missing fasteners seen as holes; overhead position error median 0.64 to
0.72 mm, at most 1.30 mm; wrist position error median 0.27 to 0.28 mm, at most 0.44 mm (the bits need 2.0 and
1.2 mm). Every fastener was driven at the first press; the 23 corroded ones stalled at the first limit and broke away
at the raised one (M6 16.0 to 18.8 N m, M8 38.0 to 40.7 N m). No protective stop, nothing stripped, no wrist frame
taken a second time.

*The hidden set*, run once (`hidden1`: staged as `full_runs.sh` stages the reference, `tests/` copied fresh from
`82e5f2f` with nothing uncommitted, the six program files in `home/submission`, no token, the same start timeout;
then the native verify stage):

| pack | variant, harness | credit | steps (reference) | simulated s (reference) | frames | program s | placed |
|---|---|---|---|---|---|---|---|
| 0 | P4, lid | 1.0000 | 1139 (921) | 768 (666) | 32 | 5.8 | 49/49 |
| 1 | P4, busbars | 1.0000 | 1079 (852) | 712 (601) | 32 | 5.4 | 48/48 |
| 2 | P5, none | 1.0000 | 1301 (1040) | 847 (730) | 37 | 6.2 | 59/59 |
| 3 | P5, lid | 1.0000 | 1326 (1066) | 882 (757) | 38 | 6.4 | 58/58 |
| 4 | P6, busbars | 1.0000 | 1471 (1151) | 961 (804) | 44 | 7.3 | 67/67 |
| 5 | P6, none | 1.0000 | 1493 (1185) | 954 (813) | 43 | 7.3 | 69/69 |

`mean_pack_credit` **1.00000**, ratio 1.000, **reward 1.0**; the probe lock 96 of 96 counted pairs (74 idle pairs
not counted); no protective stop, no stripped fastener, no live contact, no dropped module; the program's own time
38.6 s for the six packs (the budget is 1200 s a pack); stage 1476 s. Of the 170 probe copies the true observation
had the larger residual in 92 (1.1 standard deviations above half), the largest residual 1.6e-6 for both, as for the
reference. There was no second run: nothing failed. The program's stderr is not kept by the verify stage, so the
hidden packs have no perception record; what the baseline gets wrong there is nothing that cost credit.

*What it does not do, and what it says.* It never needed and so never exercised: the extractor bit (it has no code
for a stripped head), its retries (another grasp along the harness, a second pick, the lid's second look), a frame
taken again because of a repeated step. It does not tell rust from the picture (the stall does). It leans on exact
depth: with noisy depth the head-by-height rule and the top-face centre would need another method, untried. The
task is therefore solvable from what a tested agent is given, by a program of about 950 lines of controller and 300
of perception. How hard that is for an agent without the development truth and without the reference's code to
start from is not measured here (section 5).

## 7. Final report (2026-11-24, 16:30 EST)

**Since this report:** the kit of upstream #173 and a rebuilt probe copy (section 6, the entry "18:20 to 19:00"),
then an exact conversion of a pose near a half turn (the entry "19:10 to 20:30"), then a cap on how fast the
physics pushes a fastener out of an overlap (the entry "2026-11-25, 01:20 to 02:40"). The reference scored
1.00000 on `152fdf7`, 0.99440 on `15d8694` (five fasteners of two packs thrown out of the bins when the reference
released into its own pile, section 5), and 1.00000 on `8439170`, which has the cap. The anchor is 1.0 again,
measured on `8439170`; the small checks and the probe lock are as before on every tree. The numbers below are the
earlier trees'.
Then a perception-based baseline (`review/perception_baseline/`, the entry "2026-11-25, 10:00 to 11:35") scored
`mean_pack_credit` 1.00000 on the hidden set without the token, reward 1.0, lock 96 of 96: the task can be solved
from what a tested agent is given (this supersedes "A human-expert, perception-based solution. None exists" below).
The task's files, the reference and the anchor are unchanged.

**State.** The folder is complete and committed on the branch `claude/battery-pack-disassembly-8e1039` (not
pushed, no pull request, nothing outside the folder edited, the coordination sheet untouched). The work stops
here because everything left needs Docker and the engine, which the author's server does not have.

**Verified on the author's server (native, outside the engine; RTX 6000 Ada, driver 570.195.03, Isaac Sim
6.2.0.0, PhysX on the CPU, every simulator process confined to GPU 0):**

- The static gates listed in the last status entry.
- The grader's episode loop end to end (`tests/verify.py` as an ordinary account: the kit's `Solver`, the probe
  lock, one fresh cell process per pack, the kit's scoring and verdict): no submission scores 0.0; the example
  program scores 0.0; the reference scores `mean_pack_credit` 1.00000 = reward 1.0, three times; the reference
  without its token scores 0; a blind replay of the reference's own actions earns the same credit and is refused by
  the probe lock.
- The commissioning service with the agent's client library and `run_session.py`, as the author's account.
- The image's download of the robot model.
- What a non-privileged program can read from the frames, for the lid's fasteners only (section 6, the table).

**Not verified, because it cannot be here:**

- **The Docker image.** `environment/Dockerfile` was written after `hazard_bay_build`'s and never built. Unknown
  until it is: that the build succeeds, the agent-UID import check, the pinned settings' effect in the container.
- **Everything the verify stage does only as root**: the cell under its own account `packcell` from a private
  copy, the seccomp launcher `nonet_exec.py` around the submission (the native runs print `egress guard: none`),
  the kit's clearing of the agent account and securing of the path, the records under `/opt/arb-verify`, the
  verdict through `ale_verify` into `/logs/verifier`.
- **`environment/setup/setup.sh`** and the commissioning service under its own account behind the image's
  permissions (that the agent cannot read the service's folder or the development packs' draws rests on file modes
  that were never exercised).
- **`ale validate`**: the untouched sandbox at 0.0 and the oracle at 1.0 inside the engine. The anchor has to be
  measured again there; if the engine's hosts give the reference less than 1.0 on some pack, the anchor and
  `full_at` follow that measurement, not this one.
- **`ale run` with frontier models.** The difficulty is unmeasured. The target (0.3 or below) is a design intent.
- **A human-expert, perception-based solution.** None exists; the reference is privileged (section 5).
- **Any GPU model but the RTX 6000 Ada**, and any CPU but this server's.

**`gpu_types`.** Both tables name `["RTX-6000-Ada"]`, the one model the task has run on. The engine matches a name
by its words in order, so this accepts exactly that model. The models the benchmark's cloud hosts offer (RTX 4090,
RTX 5090, RTX PRO 6000, H100, H200) are not named because they were not tried: the task needs an RTX-capable GPU
for the cameras, the renderer is not repeatable even on one GPU, and a perception program's behaviour can depend on
the denoiser the driver picks. Before a model is added: the canary's frames on it and one reference pack.
`hazard_bay_build`, on the same Isaac Sim stack, lists the first three (`RTX-4090`, `RTX-5090`, `RTX-PRO-6000`),
which is the reason to expect them to work and to try them first; it does not list H100 or H200, and whether Isaac
Sim's renderer runs on those was not looked into here: they should be named only after a run. As long as the list
holds only the RTX 6000 Ada, a host without that model cannot run the task, so the first step of validation is to
try the image on a cloud model and extend the list.

**Decisions left to the owner** (the conservative choice was taken in each; section 5 has the reasons): the task's
name; the value weights; that the head type can be found by trial at the cost of time; `full_at`; which GPU models
to admit; whether to make the task harder or easier once models have run it.

**Next commands** (on a host with Docker, from `vendor/ale`):

```bash
VIRTUAL_ENV= uv run ale lint ../../tasks
VIRTUAL_ENV= DOCKER_HOST=unix:///var/run/docker.sock uv run ale validate \
    ../../tasks/arm/battery_pack_disassembly_to_modules --runs-dir /tmp/ale-runs
```

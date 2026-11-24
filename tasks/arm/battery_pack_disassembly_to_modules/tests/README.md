# battery_pack_disassembly_to_modules: design notes for maintainers and validators

The agent-facing contract is `instruction.md`; the pack series is described in
`environment/setup/payload/given/service_manual.md`, the tools in `tool_catalog.md`, the cell and its wire in
`cell/cell_interface.md`. This page records the design decisions, what the grader checks and why, what is known to be
rough, what was and was not verified, and a dated status log (version 0.1.0, 2026-11-24; built in one unattended
session on a server without Docker, so the engine's own gates are still owed: section 6).

**Design rule: the agent gets what the automation engineer of a real disassembly cell would have, and nothing
else.** It gets the pack series' workshop manual (drawings, nominal positions, the torque table, the isolation
procedure), the tool catalog with the tools' tolerances, the cell's interface document with exact calibration, the
robot's model, and a commissioning service that runs the cell on development packs. It does not get the cell's code,
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
- **`wire.py`**, **`server.py`**, **`cellproc.py`**: the action's schema, the cell as a process, and the client the
  grader and the service start it with.

**What is rule-based (the stated simplifications).**

- *Fasteners are rule-based constraints.* A fastener's body sits on its seat (its head is a real collider the bit
  presses on, through the spindle's spring); it turns only by the rule: the mounted bit is the head's own, the bit's
  axis is within 2.0 mm (socket) or 1.2 mm (Torx) and 3 degrees of the fastener's, the axial force (spindle stroke
  times spring rate) is at least 8 N (Torx: 10 N + 1.5 N per N m), and the torque limit reaches the fastener's
  breakaway torque; then loosening turns accumulate and after 7 (M6) or 9 (M8) turns the fastener is out and rides
  on the bit until `eject` turns it into a falling rigid body. A loose bit (a 13 mm socket on a 10 mm hexagon, a
  T30 in a T40 recess), an engagement outside the tolerances (up to 3.5 / 2.5 mm, 8 degrees), a cammed-out Torx bit
  and a tightening run above the head's limit wear the head; at wear 1 it is stripped and only the extractor bit
  removes it. Thread engagement, the fastener rising as it turns and the socket sliding over the head are not
  simulated.
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
  or dropped for how the reference does on it.
- Per pack a fresh cell and the kit's `Solver` (the controller at the agent's UID through `nonet_exec.py`: no
  internet socket), one action per step until the pack ends. The probe lock (`IdleAwareProbeLock`, rate 0.02, pass
  fraction 0.1): a probed step is asked twice with the same `t`, once with the robot's readings shifted
  (`wire.perturbed`), in a hidden order; pairs in which neither reply commands a motion are not counted. The
  schedule is a function of the pack's index and seed, so the grader adds no randomness.
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
- **No perception-based solution exists yet.** The reference is privileged. That a non-privileged program can do
  the job rests on the frames carrying the information (section 6 records what was looked at), not on a
  demonstration. A human-expert baseline (a perception pipeline plus the reference's motion skills) is the most
  valuable next piece of work, and the fairest calibration of the 2 mm / 1.2 mm tolerances.
- **GPU models.** Run only on an RTX 6000 Ada with driver 570 (DLSS falls back to an older denoiser preset there),
  so `gpu_types` names only that model. The cloud models (RTX 4090, RTX 5090, RTX PRO 6000, H100, H200) are
  untested; `hazard_bay_build`, on the same Isaac Sim stack, validates on the first three. Each model has to be
  tried (the canary's frames, one reference pack) before it is added.
- **Reproducibility.** PhysX on the CPU repeated the reference's packs on this one machine (section 6); across CPUs
  it is not expected to be bit-identical, and the renderer is not repeatable even on one GPU. The credit tolerates
  that (it reads where bodies lie), a program's behaviour may not: an agent whose perception is marginal can score
  differently on different hosts.
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
commissioning service). Probed Isaac Sim 6.2 natively (`/common/users/zl1308/venvs/arb-isaac`): the UR20 asset
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

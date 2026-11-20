# tensegrity_collapsed_warehouse_void_search: maintainers' notes

**Version 1.1** (2026-11-19; version 1.0 of 2026-11-17 was not published and is kept in the maintainers' development
folder). Platform `soft`, stage `behavior`. Hidden side: this folder is never staged into the agent's sandbox, so names
that the agent must not see may appear here; the attribution is in `NOTICE.md`.
The measurements these notes quote, and the task's design, are in the maintainers' development folder, which is not
part of this repository; a path below that starts with `tools/`, `build/`, `evidence/`, `text/` or `repo_tests/` is
a path in that folder.

**What changed from version 1.0.** One model episode on version 1.0 read 1.0 (void_search_score 0.97). Version 1.1
keeps the robot, the primitives, the cable controller, the station's calls and replies, the telemetry, the camera
model, the dwell rule, the retrieval rule, the contract, the session, the practice allowance and the debrief, and
changes five things: the site family (W2 -> W3: three like-looking mouths, a deeper void with dead ends and a chamber
in a back corner, rubble standing in the passages under the slab), the distance credit (0.33 / 0.66 m -> 0.20 / 0.40
m), the weights (0.15 / 0.25 / 0.60 -> 0.05 / 0.15 / 0.80), the time-limit factor (2.5 -> 1.6: 2100 s -> 1800 s) and
the numerical kernels of the program's own process (pinned, section 4). The hidden set, the practice sites and the
calibration site are new; the anchor was measured again (section 6).

**What the robot is made of.** The robot in the simulation and in the camera frames is the designers' published
whole-strut mesh, three times (a decimated copy for the agent camera, section 2); its mass and inertia are the
designers' simulation file's, its collision bodies are spheres, cylinders and a convex hull fitted to that mesh, and
its cables and stretch sensors are the 15 tendons of the designers' simulation model (lengths between two attachment
points, with no shape). Nothing of the robot was modelled, redrawn or completed, and there is no hidden variant of it:
every site has the same robot.

**The agent camera shows plain boxes.** Every site is made of plain boxes and the frames come from MuJoCo's classic
renderer. An art pass (the same collision boxes, dressed, with another render path) is planned; when it lands,
sections 7, 9 and 11 and the documents' cost figures change with it.

## 1. The task

A rescue team has lowered a mast with a fixed RGB-D search camera through the roof of a collapsed warehouse; the
camera looks down on the south 4.4 x 4.0 m of a 4.4 x 5.8 m site; the north part lies under a fallen slab. A 0.33 m,
0.73 kg three-strut tensegrity is set down on a pad. It carries no camera and no computer: a program on a PC outside
reads the camera's raw frames and the robot's telemetry and sends rolling primitives and cable-length targets, as in
the designers' own experiments. The agent writes that program (`operate.py`, `solve(station)`). A mission gives three
search points; at each the robot must stand still for 20 s and report. Points 1 and 2 lie in the camera's view (point
2 behind a ridge of rubble). Point 3 lies in a chamber under the slab, in one of the site's back corners, out of the
camera's view: three mouths that look the same open under the slab's front edge; one leads in, the other two are
plugged a short way inside. Behind the open mouth a straight passage with a dead end, a connecting passage and, on
some sites, a back passage with a second dead end lead to the chamber; the way from the last pose the camera sees to
point 3 is 4.2 to 5.9 m long with at least two right-angle bends (the ten scored sites), and one or two blocks of
fallen material stand against one wall of each straight passage, leaving beside them a gap as wide as the mouths.
Then the robot is to be brought back to the pad. The grader runs the program on ten hidden sites, each within
1800 s of simulated time.

## 2. Files

| path | what |
|---|---|
| `test.sh` | closes the stage and its parent (chmod 700) before Python starts; builds the root-owned givens at `/opt/arb` from `setup_payload/`; pins HOME, the user site, OSMesa (no shader cache), every thread count (8 rasteriser threads, OpenMP 1, BLAS 2) and the numerical kernels (`OPENBLAS_CORETYPE=Haswell`, `OPENBLAS_L2_SIZE=256`, `NPY_DISABLE_CPU_FEATURES="X86_V4 AVX512_ICL AVX512_SPR"`); `exec "${ALE_VERIFY_PYTHON:-python3}" -s verify.py` |
| `verify.py` | the grader (section 4) |
| `grader_config.json` | run settings only: `episodes` 10, the cell's budgets (600 frames, cable targets 100-240 mm, program time 600 s, silence 180 s, CPU 2400 s), `pose_source` (`estimated`), `render_backend` (`classic`), the privileged cap (3600 s), the route gate. No scoring threshold and no time limit |
| `anchor.json` | metric `void_search_score` (higher), `value` 1.0 = the reference's measured mean on the hidden set (two runs), `full_at` 0.95 |
| `voidscore/` | `acceptance.yaml` (version 2), the single source of every scoring threshold (weights, credit radii, dwell window and still tolerance, reports per point, retrieval radius and factors, the debrief's "stuck" rule); `scoring.py`; `debrief.py` (the practice ground's debrief; never imported by `verify.py`) |
| `voidcell/` | the cell: `server.py` (VoidCellServer, the station's wire side), `client.py` (byte-identical to the agent's `given/lib/voidcell/client.py`), `wire_guard.py`, `_lib/` (the vendored simulation base `ysim` 0.3.1 with its lock `ysim.lock.json` and data tables, the site loader, the executor of primitives and cable targets, the truth record, the camera, the telemetry, the budgets, `frames.md`), `data/meshes/` (the robot's visual meshes for the agent camera: the designers' strut mesh regrouped by material; four groups at full resolution, the group of small electronics decimated by 94 %; `MESHES.json` records tool, parameters and sha256) |
| `reference/` | the privileged reference program, voidref 1.1.1 (`reference.py`, run by the cell's `reference_run`) and its modules (`_lib/voidref/`: planner, controller, monitor, its primitive table) |
| `hidden/` | `sites/h01..h10.tar.xz` (the stored site packages of family W3, rubble included), `seeds.json` (the ten sites in episode order: archive, sha256, episode seed, the reference's final state and time), `selection.json` (the admission rule's hash and pin time, every candidate's value of every admission row, the set, the set-level checks), `limits.json` (`time_limit_s` 1800 and its rule), `oracle_token.sha256` |
| `setup_payload/`, `given_hashes.json` | the grader's byte copy of `environment/setup/payload/` (11 files, 226 795 bytes) and the sha256 of each |
| `cellkit/`, `robotics_grader/` | the verify-stage process kit (lockdown, runner as `nobody`, no-network seccomp filter, network audit, the JSON-lines wire with program-time accounting, the freezer, pinned seeds), byte-identical to `quadruped_hazardous_plant_inspection`'s copy; the benchmark's verdict kit, byte-identical to `shared/robotics_grader/` |
| `review/` | maintainers' tools that nothing runs at verify (its own `README.md` and `SHA256SUMS`): the site generators `w2gen` (the open ground) and `w3gen` (the slab, the three mouths, the void and the rubble), the calibration site's generator `calgen` 0.2.0, the QC scripts, the degenerate submissions, the maintainer variants of the reference with their expected scores. The admission rule and the selection, packaging and oracle-verify tools are not here: they stay in the development folder (section 8 describes what they did) |
| `README.md`, `NOTICE.md` | these notes; attribution |

Outside this folder: `solution/` (the oracle: a short `operate.py` and the 256-bit token), `environment/` (the
image; the payload; `practice_ground/` = the practice service `practiced.py`, a byte copy of `voidcell/` and
`voidscore/` without the reference and the token, the calibration site `cal`, the practice sites `p1..p3`).

## 3. Scoring

Per site, from the cell's own truth record (`voidscore/scoring.py`, every threshold from `voidscore/acceptance.yaml`
version 2): S = F x (0.05 c1 + 0.15 c2 + 0.80 c3).

* Position = the planar centroid of the six nodes (the end-cap centres).
* A report is a `report_point(i)` the station accepts (at most 3 per point); only the last accepted report of a point
  counts, also when an earlier one was better.
* c = dwell x distance. Dwell is 1 when the report comes at least 20 s after the episode's start, no command (a
  primitive or a `set_targets`, from acceptance to end) was executing at any moment of the 20 s before it, and every
  10 Hz truth sample of that window lies within 0.02 m of the position at the report. Distance is 1 within 0.20 m of
  the search point, linear to 0 at 0.40 m. The 0.20 m is a setting this task adds, not the designers' figure: an air
  reading is taken where the sampler stands, so the robot has to be brought to the spot in the void, not near it (the
  designers counted a stop within one body length, 0.33 m, as reaching a goal; version 1.0 used that radius).
* F = 1.0 when, at the instant the episode ends, the position is within 0.60 m of the pad centre and no command is
  executing; otherwise 0.85.
* No negative marks; the robot cannot be damaged; touching rubble costs nothing.

`void_search_score` = `contract_ok` x the mean of S over the ten sites; reward = clamp(score / 1.0 /
0.95, 0, 1). The two visible points and the retrieval alone give 0.20; all three points without the retrieval 0.85.
On every selected site no pose that touches a wall or a block and no pose the camera sees earns anything of point 3
(predicate G8a: the highest such credit is 0 on all thirteen sites, `hidden/selection.json`): point 3 stands at least
0.62 m from every solid and every site edge by rule, 0.640-0.697 m on the thirteen sites.

The cell announces six of the constants in `mission()` and holds its own copies (`voidcell/_lib/mission.py`,
`budgets.py`); every episode checks them against `acceptance.yaml` and a difference is a task error.

## 4. The verify stage (verify.py)

The stages and the kit of `quadruped_hazardous_plant_inspection`'s grader, with this task's cell, program and score:

0. `_early_lockdown` (stdlib only, first): the stage upload time; kill and freeze every process of the agent account
   until the verdict; lock the stage; a root-owned snapshot of the submission (regular files only); late writes into
   the submission zero the contract gate, late entries elsewhere in the home and the sticky folders are quarantined.
1. The givens at `/opt/arb` against `given_hashes.json`, and the cell's announced constants against
   `acceptance.yaml`: a mismatch is a task error.
2. Gates (`contract_ok`): `operate.py` is a regular file that parses and defines `solve`; the route gate (60 MB, no
   weight files, no training calls); every `.py` file readable by the parser; the static network audit (no networking,
   process or foreign-function import, no computed import or code, no program start; files byte-identical to the
   shipped client exempt); the import check as `nobody` under the no-network filter (120 s wall, 120 CPU-s); over the
   episodes at least one `step` call, no late write, no surviving process, no remote inet socket.
3. The oracle token: `.oracle_token` whose sha256 equals `hidden/oracle_token.sha256` makes the cells privileged (the
   call `reference_run`); a wrong token is ignored and recorded (`oracle_token_mismatch`).
4. The hidden set: `hidden/seeds.json` must pin `episodes` sites; every archive must be a regular file under
   `hidden/` whose sha256 matches (else a task error, also for an untouched submission). The simulated-time limit
   comes from `hidden/limits.json`, every other budget from `grader_config.json`.
5. Before the first episode: the frozen desktop settles, the no-network filter proves itself.
6. Per site, under an opaque tag `epNN`: a fresh `VoidCellServer` on the package (mode hidden; the robot set down
   with the pinned seed); the program as `nobody` from a read-only copy of the snapshot, `python -B -E -s`, the client
   library from `/opt/arb`, `prlimit --cpu`, `oom_score_adj` 1000, the seccomp no-network filter, its account's
   remote sockets sampled, and the same numerical kernels as the cell (`BLAS_PIN`: `OPENBLAS_CORETYPE=Haswell`,
   `OPENBLAS_L2_SIZE=256`, the NumPy SIMD cap; the practice ground's `~/training/run_mission.py` sets the same three;
   the verdict records that the pin held); the wire until the program finishes, calls `finish()`, reaches the
   simulated-time limit, falls silent (180 s), exceeds its program time (600 s of its own time: the wall clock minus
   the cell's simulation and rendering) or breaks the protocol. Whatever ends the episode, the reports accepted until
   then are scored and the retrieval is judged at that instant. Every process of the runner account is killed after
   each episode, which is why the episodes run one after another.
7. An episode in which the cell failed (an internal error while the episode was live, a frame it could not render)
   is run once more from scratch and the second run counts; at most one per verify (`MAX_RERUNS_PER_VERIFY`), a second
   one is a task error. A failure on the grader's side (the cell's build, the launch, the wire, the clean-up, a score
   that is not a finite number in [0, 1]) is a task error; a program that crashes or hangs ends its own episode.
8. The verdict through `robotics_grader`; records under the root-only `/opt/arb-verify` (`out/episodes.json`,
   `out/traj_<tag>.npz` with the 10 Hz truth samples, `logs/<tag>.log` with the program's stderr).

The kernel pin exists because OpenBLAS picks its kernels per CPU and NumPy its SIMD paths: without it the program's
numbers could differ between hosts, and the Haswell kernel alone, without the L2 size, crashes or returns wrong
products on Zen 4 / Zen 5 CPUs with numpy 2.5. A scored episode has no debrief: `verify.py` imports nothing of
`voidscore/debrief.py`, the cell writes none, and the reply to a report says only that it was accepted. Untouched
sandbox: no `operate.py`, `contract_ok` 0, reward 0.

## 5. Realism audit

What in the task's world is real (the designers' robot and experiments) and what this task adds. "Base assumption"
marks a value of the simulation base that no public measurement of the real robot fixes.

| in the task's world | real or added | source, or why it was added |
|---|---|---|
| The robot's body, 15 tendons (6 motor-wound lines, 6 short and 3 long stretch sensors), 6 gearmotors, one 6-axis IMU per strut | real | the designers' mesh, simulation file, bill of materials and firmware |
| No on-board camera or computer; planning, the cable-length PID and pose estimation on an off-board PC; a radio link | real | the designers' paper and control code |
| One fixed overhead RGB-D camera, 640 x 480, 7 Hz, calibrated; red, green and blue end caps, both ends of a strut the same colour | real | the paper; the intrinsics and the depth unit are those of the designers' tracking configuration |
| Eleven primitives, target shapes (six cable lengths), re-planning after each primitive, the symmetry table by the bottom triple | real | the paper and the control code |
| Navigation among obstacles; success = stopping within one body length of the goal | real | the paper. This task's credit is tighter and added: full within 0.20 m, none beyond 0.40 m (the sampler has to stand at the spot) |
| After a bad primitive the robot can end in an orientation from which it cannot roll on | real | the paper; in the simulation these are the two end triangles (16 of 7600 primitives among obstacles, 0 of 4640 on a free floor; the handbook prints the counts) |
| How far each primitive moves the robot, the stop band of 5 % (the designers' code: 15 %), which end-cap hook carries which cable, the line's stiffness, the 0.911 s rest between primitives, the IMU's axes | base assumption | no calibration data of the real robot is public. The handbook's primitive table is this simulated robot's own measurement on a flat floor, by start face, not the designers' published table (against that table: forward rolls 0.04-0.10 m off in translation, the clockwise turn qualitatively different) |
| A narrow passage and a low beam | the designers' simulation scenes, not the real robot | only their sizes are borrowed (the tight opening class T and the low slot XL, 0.15 m clear) |
| The scene: a collapsed warehouse after an earthquake | added | the maintainers' choice of setting |
| The camera on a mast through the roof, 4.95-5.25 m up, tilted 14-17 degrees (the lab: 2.34 m, straight down) | added | it has to cover the open part of the site and see the mouths |
| The program gets raw RGB-D only and estimates the pose itself (switch `pose_source`, default `estimated`) | added | in the real system a tracker supplies the pose; with a given pose the task would be the designers' published planner |
| The crawl space out of view: three mouths that look the same (one open), straight passages with dead ends meeting at right angles, a chamber in a back corner whose two walls are the site's own, a blind way of 4-6 m | added | the real experiments ran in view throughout; the paper names navigation on on-board sensing alone as open |
| Fallen material against one wall of a passage under the slab (1-2 blocks per straight passage, 0.20-0.30 m long, floor to slab; the gap beside each 0.559 m, the mouths' width) | added | a passage in a collapsed building is not clear; a program that rolls down the middle of a passage without knowing the site meets a block (the centre band of the widest roll overlaps every block by 0.04-0.11 m); the reference passes every block without contact |
| A search point = a 20 s dwell for one air reading (a gas sensor of a few grams on the board's expansion port; not modelled) | added | it gives a search point a meaning; only the dwell and the place are judged |
| 8 to 14 flat plastic chips in the end caps' colours on the floor; another light at every site | added | under the classic renderer one colour threshold would find the end caps; the paper reports a coloured surface disturbing the tracker |
| The radio holds everywhere (under the slab too); the telemetry's and the depth's noise | added (a simplification; rated values) | no public measurements; the documents call them rated values of this unit |
| The simulated-time limit (1800 s) as the team's working window; no energy term | added | endurance is not published; ten minutes of mixed primitives use about 6 % of the motor battery in the simulation |
| Loose ground = flush low-friction patches (dust 0.35, gravel 0.60); no relief, no granular medium; a level floor without steps or slopes | added (a simplification) | granular simulation is slow and unstable; this simulated robot climbs neither a 2 cm step nor a 4 degree slope, so a site has none |
| Lowered on a rope and hooked for retrieval; set down on a rollable face | added | no primitive starts from an end triangle |
| The floor friction of a site, drawn from 0.76-0.92 | range real, the draw added | the range of the designers' identified ground friction; its effect on a primitive is at most 3.3 mm and 1.9 degrees |
| Rolling backwards | not offered | the paper does not mention it and the designers' code says it was never run |

Given to the agent: the handbook (neutral model name RT-3; dimensions, numbering, the cable controller, the eleven
primitives with their target states and the symmetry table, the flat-floor table of every primitive from every
rollable face, what is not guaranteed, the telemetry with its noise, the IMU frames; three purchased parts under their
real vendors and part numbers; no picture of the robot), the station's API, the site briefing (the site rectangle and
its walls, the slab from wall to wall, three like-looking mouths of which one leads in, the passages' shape in words,
the chamber in a back corner with nothing within 0.6 m of point 3), the camera's data sheet, the calibration site's
pages (every dimension, among them a roofed passage narrowed by a block and a roofed chamber in a corner; nowhere
whether an opening can be passed), the client library, an example that runs and scores 0, and the practice ground (a
black-box service under its own account: the calibration site and three practice sites, 30 episodes, two at a time, a
debrief after each).

Not given: the simulator's code, the site generators and their ranges, the grading code, any scored site's layout,
which openings the robot can pass, which mouth is the open one, the passages' widths, lengths and turns, where the
blocks stand, the robot's motion on loose ground, the true pose, any score of a scored site, and every name of the
robot's designers, their schools, labs, papers, design and repositories. Left for the agent to measure on the practice
ground (`text/DISCLOSURE_CHECK.md`): the roll's swept width and peak height; that floor friction hardly matters; the
dust patch's numbers; that the robot gets out of a plugged mouth again; the clearance from walls and the footprint of
a turn; how far the table drifts when nothing corrects it; what "stuck" looks like in the telemetry.

## 6. The oracle and the anchor

`solution/solve.sh` installs `operate.py` (the mission, one frame, the telemetry and the status, then
`reference_run`) and the token. The reference program runs inside the cell with the privileged truth: the true node
positions (the pose, and the bottom triple it passes to each primitive) and the site's true occupancy (every box,
which mouth is open, the plugs, the rubble). It plans with a hybrid A* over the robot's own flat-floor primitive
table, re-plans after every primitive from the true pose, and sends every command through the station's own calls
with the same argument checks, budgets and time limit as an agent's program. It takes no camera frame for its
decisions and reads no telemetry, so its result does not depend on the renderer. Version 1.1.1 of the reference is
1.1.0 with comments and docstrings changed; its runs end bit for bit in the same final states.

The anchor (`anchor.json`): **1.0**, `full_at` 0.95 = the mean void_search_score of two full runs of `verify.py` with
the oracle submission on the ten frozen sites, on the development host, not as root (no lockdown and no account
switch; the givens copy built as `test.sh` builds it; `test.sh`'s numerics), two runs at once: every site S 1.000 in
both runs (c 1 / 1 / 1, F 1), every episode ended by `finish`, simulated time 942.7-1116.2 s (at most 62.0 % of the
limit), episodes 244-432 s of wall clock each; the ten final states equal bit for bit between the runs and to the
admission runs of the selection (`hidden/seeds.json` `reference_final_state_sha256`); no debrief file anywhere. A
first pair of the same runs, before a last text-only edit of the simulation base's parameter descriptions, gave the
same twenty episodes and final states.

| run | stage anchor | result | verify wall |
|---|---|---|---|
| oracleA | null (bootstrap: reports the metric, scores 0) | 10 x 1.000, void_search_score 1.000, reward 0.0 | 3375 s |
| oracleB | 1.0, full_at 0.95 | 10 x 1.000, void_search_score 1.000, reward 1.0 | 3398 s |

Validations in the task image (2026-11-19, the sandboxes started without a DNS search line): untouched 0.0, oracle
1.0, every one of the ten sites 1.000; the oracle's verify took 62 and 76 minutes in two of them (3730 s at host
load 35-66 of 48 cores; 4567 s). The run's id and the task's content digest depend on these files, so they are given in the pull request
that adds this version, not here. The first set's two validations (`validate-a7e732de`, `validate-cb45ff74`, both
oracle 1.0) were of version 1.0.

## 7. Timeouts

Agent: 14 400 s (4 h), no network. Verifier: **19 100 s** (`task.toml`). Its header derives it from numbers measured
in containers of the task image with the verifier's 8 CPUs and 16 GB, every input the largest measured in either
image: version 1.1's (2026-11-19 16:17-16:39 UTC, host load 19-56 of 48 cores; `evidence/validate_v1_1/costs.json`)
and version 1.0's (the same cell code with its own practice sites; 2026-11-17 21:18-21:31 UTC, host load 42-91;
`evidence/validate/costs.json`), with the 1800 s limit:

* fixed steps (the import check at most 120 s, the no-network self-test and the desktop settle at most 60 s each, the
  lockdown, the givens and the acceptance check): 260 s;
* per site at the bounds of the configuration: the cell's build and first frame 10 s; the program's 600 s program
  time; 1800 simulated seconds at 0.1922 s of wall clock each (the slowest whole-episode pace measured, a primitive
  running, the wire included): 346 s; 600 frames at 0.3552 s (the p95 of `get_frame`): 213 s; the wire's 60 s grace
  and 30 s of end, scoring and clean-up: 1259 s a site;
* eleven episodes (the ten sites and the one allowed re-run) plus the fixed steps: 14 109 s; 19 100 s is 1.35 times
  that.

In the version 1.1 image one episode alone costs 0.127-0.163 s of wall clock per simulated second while a primitive
runs (two runs; 0.099-0.146 s standing) and 0.225-0.228 s per frame (p95 0.25-0.30 s); two at once 0.128-0.130 s and
0.22-0.25 s. The documents' cost paragraphs (`tools/docs/gen_blocks.py` from `evidence/validate_v1_1/costs.json`)
print the larger of the two solo runs: about 0.16 s and 0.23 s. The version 1.0 image gave 0.146-0.161 s and
0.26-0.33 s per frame with the same bench. A program that spends every budget on every site would hold the verifier
for about 3.6 h. The oracle's verify took 3375 and 3398 s on the development host for the ten sites (two runs at
once) and 3730 s and 4567 s in two validations in the task image (section 6).

## 8. The hidden set and the measured programs

**Generator and candidates.** The site family W3 (`review/sitegen/w3gen`, version 0.1.0; it takes the open ground
from W2, `review/sitegen/w2gen`) builds a site from a seed: `site.json` (every coordinate and truth), `site.xml` (the
world only) and the generator's own QC (`qc.json`, not part of a frozen package). Two random streams: everything the
camera or `mission()` can show is drawn first and does not depend on which mouth is open; then a void is built for
each of the three mouths taken as the open one, and the open mouth is drawn last (`default_rng([seed, 12])`), so given
everything visible the open mouth is equally likely to be any of the three. The rubble under the slab is part of the
family: the void's straight passages get 1-2 blocks each from their own stream (`default_rng([seed, 13])`; the rule is
in `review/sitegen/w3gen/rubble.py`), each against one wall, 0.20-0.30 m long, floor to slab, the gap beside it the
mouths' width, not in the chamber, a dead end or a junction, at least 0.5 m from the mouth's inner end and every
corner, at least 0.8 m apart along a passage, at least 0.62 m from point 3. Sizes come from the measured capability
sheet of the simulation base (FINAL for ysim 0.3.1, revision 0.3.1-sheet1, sha256
`563efedf6128adacd9c1f246d90af1554445a0079fb735d467959be92e6b7f62`): class P 0.559 m wide (0.387 m clear in the first
ridge), T 0.459 m, the narrow decoy XN 0.257 m, the low decoy XL 0.559 m wide and 0.15 m clear; under the slab the
mouths 0.559 m wide, every passage and the chamber 0.407 m clear, the passages 0.779-0.919 m wide.

Predicates of a candidate (all from the package, never from the generator's own record of the rubble): G1-G11 as in
version 1.0 rewritten for three mouths and the deep void (camera coverage of the camera rectangle, projection against
the given intrinsics within 1 px, visibility of the open floor and of points 1 and 2, nothing of point 3 or the void
visible, the three mouths alike with no camera ray on a plug or any hidden box, point 3 at least 0.62 m from every
solid, the blind way 4.0-6.0 m, routes, minimum distances), G12-G16 for the void (at least two bends, at least one dead
end, passage widths, no straight line of sight from a mouth to point 3, the two known walls reachable from point 3),
G17-G19 for the rubble (the gap beside every block is the mouths' width and a route through the gaps exists; at least
one block on the way in, reaching at least 0.04 m into the centre band of the widest roll; clearances). G7c compares
the three counterfactual sites (each with its own rubble) ray for ray, frame for frame and `mission()` for `mission()`;
G9 and G17-G19 are judged on all three, so admission does not depend on which mouth is open.

**Rule first.** The admission rule (sha256 `a6b5f8beb814085a94206efaf46120aafd759ea4cffb818e07a35723f640cd33`,
26 641 bytes, in the development folder; its hash, pin time and a summary of every row are in
`hidden/selection.json`) was pinned on 2026-11-19 00:51:35 UTC, before any seed of the pools was generated, run or
opened (the first candidate result is of 00:52:31 UTC). Pools: hidden 101-180 (extension 181-260), practice 501-530
(extension 531-560). Of the hidden main pool 36 of 80 seeds pass G1-G19 (most failures: G18, a void with no room for a
block on the way in) and 28 are admissible; the used extension 181-235 adds 21 and 12; the practice pools 14 + 11 and
8 + 8. The reference ran every G-passing candidate through the station's calls (82 runs, all S = 1, F = 1; one stuck
period, seed 220) and a second time in a new process for the 56 candidates that pass every other row (56 of 56 equal
final states). R1-R8: full score without a stuck period or abort; at most 1300 s, and after the freeze at most
time limit / 1.6 = 1125 s; the primitive count inside [116, 146] (the 15th-85th percentile of the main pool's 50
candidates, computed once); no end triangle, set down on a rollable face; at least 16 primitives on the blind leg
going in; strut penetration at most 5 mm; the same final state in a second process; no contact with any rubble block
(106 blocks over the 82 runs: 0 contacts, nearest approach 0.171 m). A pool-level check on dead reckoning: chaining
the handbook's table over the reference's blind leg ends within 0.20 m of point 3 on 24 of the 81 G-passing
candidates (0.296; the rule's line 0.35: select as usual). Greedy in seed order without backtracking, quotas: third
opening of the first ridge T / XN / XL 3 / 3 / 4, corner E / W 5 / 5, rank of the open mouth by its distance to point
3 near / middle / far at most 4 each. The tenth hidden site came from the extension (seed 235: it had to be XN,
corner W and not far; the main pool had none). The practice greedy found no third site after 502 and 505 (the only
site with the missing strata, 550, lies outside the primitive window); by the rule's last fallback the selection
stopped there, and the practice set was taken as the first feasible triple in seed order with every quota kept: 502,
514, 553. Nothing was hand-picked. Four ways the rule departs from a literal reading are written in it before any
result: the set-level minimums are enforced inside the greedy (they changed nothing here), the cue check's gate is
family-wise, an extension is used as a prefix, "both corners" in practice is at most 2 of a corner.

| site | seed | third opening | corner | open mouth | rank (0 = nearest to point 3) | way under the slab | tunnel roof | blocks | reference time (s) | primitives (blind leg in) |
|---|---|---|---|---|---|---|---|---|---|---|
| h01 | 104 | tight opening | W | east | 2 | L | sloped | 1 | 1103.06 | 140 (23) |
| h02 | 105 | narrow decoy | E | east | 0 | U | sloped | 2 | 942.69 | 120 (26) |
| h03 | 107 | low decoy | E | centre | 1 | U | sloped | 2 | 1116.25 | 145 (28) |
| h04 | 108 | tight opening | W | centre | 1 | U | flat | 1 | 1083.90 | 138 (27) |
| h05 | 109 | tight opening | E | west | 2 | L | flat | 2 | 999.93 | 125 (25) |
| h06 | 113 | narrow decoy | W | east | 2 | L | sloped | 2 | 958.90 | 122 (22) |
| h07 | 118 | low decoy | E | west | 2 | L | flat | 1 | 1054.27 | 135 (22) |
| h08 | 127 | low decoy | E | centre | 1 | U | sloped | 1 | 954.90 | 121 (29) |
| h09 | 129 | low decoy | W | west | 0 | U | flat | 1 | 1006.22 | 128 (27) |
| h10 | 235 | narrow decoy | W | west | 0 | U | sloped | 2 | 1085.42 | 136 (34) |
| p1 | 502 | tight opening | E | east | 0 | U | flat | 2 | 1051.45 | 136 (33) |
| p2 | 514 | narrow decoy | E | west | 2 | L | sloped | 1 | 944.81 | 121 (24) |
| p3 | 553 | low decoy | W | centre | 1 | U | flat | 1 | 951.73 | 123 (31) |

Hidden strata: third opening 3 / 3 / 4, corner 5 / 5, rank 3 / 3 / 4; set-level checks: open mouth west / centre /
east 4 / 3 / 3 (at least 2 each), tunnel roof flat / sloped 4 / 6 (at least 4 each); the open mouth is the nearest one
to point 3 on 3 of 10 sites (chance: Binomial(10, 1/3)). The cue check (54 features per mouth that a program can get
without the truth: `mission()`, camera rays, classic-render brightness, the boxes the rays hit; 66 single-feature
rules on the 82 G-passing candidates; 20 000 permutations): the best rule hits 39 of 82 (raw p 0.0095, Bonferroni
0.62), permutation p 0.30; the gate is family-wise and holds; on the frozen 13 the permutation p is 0.82. Seeds,
archives, `site.json` and `site.xml` (geometry) of the hidden and practice sites are pairwise disjoint and differ from
the 360 development packages of the generator and from the calibration package. Every hidden episode's seed is its
site seed (the set-down and every noise stream derive from it), so a scored episode repeats exactly; a practice
episode draws a fresh set-down and noise. Each frozen package holds `site.json` and `site.xml`; `site.xml` is byte for
byte the one the admission runs used and `site.json` differs from it only in two provenance fields of the envelope
record (dropped when the generator's text was cleaned before the freeze). G17-G19 recomputed from the frozen bytes
hold on all three counterfactual sites of every one of the 13 (the true site from the frozen `site.json`; the other
two rebuilt from the seed, whose true site serialises to the frozen bytes); every frozen site has 1 or 2 blocks.
Rebuilding a package from its seed runs in the maintainers' development workspace, not from this folder alone: the
generator reads the capability tables of the development folder, and its G7c check uses the development folder's art
tools (the copies in `review/sitegen/` are for reading). There, with the generator's environment
(`OPENBLAS_CORETYPE=Haswell`, one BLAS thread), `python -I -B tools/sitegen/w3gen/cli.py --seed <seed> --cf-render
--pool --out <dir>` writes `<dir>/w3_s<seed>/{site.json, site.xml, qc.json}`, whose `site.json` and `site.xml` are the
frozen archive's members (member names `hNN/` and `pN/`). The packages were written in the format of the calibration
package (PAX tar, members `<tag>/site.json` and `<tag>/site.xml`, mtime 0, mode 0644, uid / gid 0, xz preset 6 with
CRC64; twice, byte for byte the same).

**Time limit.** 60 s x ceil(1.6 x the reference's longest time over the 13 selected sites / 60 s) = 1800 s (longest:
h03, 1116.25 s, 145 primitives); every selected site's reference time is at most 1800 / 1.6 = 1125 s. The practice
sites have the same limit; the calibration site max(900, 60 x ceil(1.6 x 872.5 / 60)) = 1440 s.

**Measured programs.** Only these have been run through the real `verify.py`. No pilot, probe, baseline ladder or
non-privileged solver was built. Besides them, one agent submission was run during the design of this version: version
1.0's saved episode program, under this version's settings on twelve development sites, as a diagnostic of whether an
unchanged program is still stopped (mean S 0.19, point 3 on none of the twelve).

| program | what it does | void_search_score | note |
|---|---|---|---|
| oracle | the reference with the token | 1.000 and 1.000 (two runs on the development host; reward 1.0 with the stage anchor 1.0) | this freeze, section 6 |
| D0 | `solve` returns at once | 0.000 | `contract_ok` 0 (the contract asks for at least one `step` call) |
| D1 | waits 25 s, reports all three points from the pad, ends | 0.000 | |
| D2 | never moves, steps to the time limit | 0.000 | F = 1, c = 0 |
| D3 | random primitives with a fixed seed to the time limit, no report | 0.000 | F 0.85 on 9 of 10 sites, 1.0 on one |
| D4 | imports `socket` | 0.000 | `contract_ok` 0 (network audit), no episode runs |
| D5 / D6 | the oracle's `operate.py` without a token / with a wrong token | 0.000 / 0.000 | the privileged call is refused in every episode; D6 records `oracle_token_mismatch` |
| D8 / D9 | the true pose: point 3 reported against the back wall and the corner's side wall / the robot pressed against the first rubble box, all three points reported there | 0.000 / 0.000 | every wall pose 0.436-0.583 m from point 3; every rubble box at least 0.85 m from point 3 on these sites |
| example | `example/operate.py` of the payload | 0.000 on the calibration site (`contract_ok` 1, c = 0 / 0 / 0, F 1.0) | not run on the hidden set |
| V1, V2, V3 and a fourth | maintainer variants of the reference (`review/qc/variants/`), expected values written before the runs | 0.200, 0.850, 0.600, 0.680 on 10 of 10 sites each | the two visible points with retrieval; all three without retrieval; point 3 at 0.30 m; the fourth: the remaining edge cases (15 s dwell, 0.41 m, pad at 0.61 m) |

D0-D3 were run on this version's code and time limit on a development W3 site (seed 7, 1800 s; the oracle there 1.0),
and D0-D9 on the frozen set (D7 is D1; section 9).

## 9. The grader's own QC

On version 1.1 so far (raw results in the development folder):

| # | check | result |
|---|---|---|
| Q1 | determinism | the oracle twice through `verify.py` on the frozen set: 10 of 10 final states equal between the runs and to the admission runs (`hidden/seeds.json` `reference_final_state_sha256`) |
| Q6 | the selection, recomputed from the stored packages | 13 of 13 sites: every archive, `site.json` and `site.xml` hash as pinned; `site.json` and `site.xml` regenerated byte for byte from the seed; G17-G19 on the three counterfactual sites from the frozen bytes; the set-level checks hold (`hidden/selection.json` `set_level_checks`) |
| Q5 | leak scan (`tools/scan/scan.py`, its word list stays in the development folder) | on the frozen bytes: 0 fail words in the instruction, the payload and the client (7 for review, as before: the robot's kind, `socket`, `osmesa`); 5771 hidden values and names of the 13 frozen sites against every number the static agent-visible files print: 104 waived as public (8 of them in the calibration site's plan, section 11, item 15), 2 for review, none a layout value of a scored site; no site name and no package hash; the development-text scan over every file of the task folder and its repository tests: 0 failing hits (4 kept in files byte-identical with merged tasks) |
| documents | every number of the agent's documents against the configuration | on the frozen bytes: 34 generated blocks with 1671 numbers and 2 derived files, 0 stale, 0 problems, 0 pending, 0 awaiting; 169 hand-written numbers (24 bound to the configuration, 145 structural), 0 mismatches; the 19 rows of this version's sentence table, 0 problems; `behaviour_check.py`: 220 statements about the station's behaviour (28 general, 192 on the 13 frozen sites and the calibration site) hold in the real cell |

Then on the frozen set (the grader's QC, 2026-11-19): the degenerate submissions D0-D9 (D7 is D1) all 0.000 (section
8); Q2 / Q3 with the new radii and weights, through the maintainer variants: a report 0.19 m from its point earns full
credit, 0.30 m half and 0.41 m none with a valid dwell, a 15 s dwell none, a fourth report is refused, the pad 0.59 m
away gives F 1 and 0.61 m away F 0.85, no retrieval F 0.85, and 0.030 m of motion in the dwell window voids the
point while 0.019 m does not (40 recorded episodes); the four variants equal their expected scores (0.200, 0.850,
0.600, 0.680) on 10 of 10 sites each; Q4, the interface: 141 of 141 checks over 199 replies (the mission on all ten
sites, without a layout value); Q6 again from the frozen bytes: G1-G19 (G17-G19 on the three counterfactual sites)
and R1-R8 on 13 of 13 sites as in `hidden/selection.json`; the counterfactual check (each other mouth taken as the
open one, with its own rubble: the same camera rays, hit names, public views, `mission()` and frames) on 13 of 13;
the cue check reproduced to the digit; the 20 frames of h03 equal in two processes on the development host. In the
version 1.1 image: 44 of 44 isolation checks (the agent account can neither list nor read the practice service's
files), the same 20 frames in two containers as on the development host, and the leak scans over the image's
agent-readable file list without a hit. The first set's QC (2026-11-17, on the unpublished version 1.0) is kept in the
maintainers' development folder; its results were: Q1-Q6 all held, 92 interface checks, 4892 hidden numbers 0 leaked, 44
isolation checks in the image.

## 10. Frozen files, and what to re-run after a change

Frozen on 2026-11-19 13:21 UTC (the freeze record in the development folder lists the sha256 of every frozen file, the
rule's hash and time, the two oracle runs and the set-level checks). The ones that decide a score, as they are after
the dated revisions of the same day (section 12; the freeze record lists the sha256 before and after each):

| file | sha256 |
|---|---|
| `tests/grader_config.json` | `ce3ef9793e87ceb501c3dd80204bfcf7a96a2bf05638b24ac9315bd46db61f61` |
| `tests/voidscore/acceptance.yaml` | `729f1bf381e1c0d1904c7ae88e496e9e8636016628f3eba8831b76c6d4f05c06` |
| `tests/voidscore/scoring.py` | `b458710b300f63ec4d38f645b6d54625822290ce15759c088806f198aa92a4c8` |
| `tests/verify.py` | `84be2754e1f6449ab9bce5a43f969c6a46a39cfbf39411fb0362f8f09313918c` |
| `tests/test.sh` | `e90b0f7e0ed7a9569f05a3134deeb202df27a3b1ff607be3ba23b08a74874264` |
| `tests/anchor.json` | `87ea02388539899bcadcd99d0621bcde933f74f155e73ae6d5a1bc62557c8f81` |
| `tests/hidden/seeds.json` | `c56c028b1e521bb3ec20faf2c07dbb2d04267e8cfc4ccae66540cf668d8b57f6` |
| `tests/hidden/selection.json` | `2976f38286b7f556e27209f5a67e4664e408f46c6c4e08fad154a5a505520686` |
| `tests/hidden/limits.json` | `3ac14df3395e86a309d55e6cb667e6e841a140434275eeecc9d8f98dfc0fb040` |
| `tests/hidden/oracle_token.sha256` | `9a16ed975ed3162d863de09226617582ee47af2c0fbfe95fd8a55f6250b6e754` |
| `tests/voidcell/_lib/ysim.lock.json` | `5676afc52392d337007a3751f1704e54780ee0af6171d6b1cd3d2fa853487fe7` |
| `tests/voidcell/server.py` | `0261294fd30da19298171d1cfaca7cd9d9c4332a4470b73d68bf66a01f8c5f1c` |
| `tests/voidcell/client.py` | `a903e92d8b4026912d330fbd642718dc90ca9629ee4822c6e4fc2e349fea2646` |
| `tests/reference/reference.py` | `25b9206db4474b569b5ceca1bb44fe86b0b6b6cffdd20abc68be8666a0ed6471` |
| `tests/given_hashes.json` | `38527c0768801eb4c72d43987a4ccde46de5f13be8ef20face7f6031f812f258` |
| `environment/practice_ground/sites.json` | `7cfdd778f2b0952b56d1c3525cc2c12e6b8a429f8b5a92c155602ddca2000b49` |
| `environment/practice_ground/cal/cal.tar.xz` | `0d7e42478b519c795bbe98265db9ac69437f5219091703a970f0bd4efabe1be6` |
| `instruction.md` | `15f6f49be97521d75bcae63c772da7aa968eae04f46257630cbd77f7d888a0cf` |
| `task.toml` | `4916fa6540bae37c6d951625e9e37859e61fab1dd428520810576b7d4124dc47` |
| `tests/reference/` (tree) | `ec0b4d44040ca68244737773b4e4c06157ecab9ae7a5fd750cee5e5be704d2ce` |
| `tests/voidcell/` (tree) | `4efb15c2dcfe55eec41940ed764bbcf5f894bcb5c2f821482d3947cfe2e424fb` |
| `tests/voidscore/` (tree) | `30990f979f2f20aa13eec98ad55a6403c29270e1d1e7db2ebe9b5182e260f0fd` |

A tree's sha256: of the text lines `<sha256>  tests/<tree>/<path>` (one per file, sorted by path, `__pycache__`
skipped). The admission rule pinned the trees as they were before the text-only revision; `hidden/selection.json`
(`pinned_inputs_revised_in_text_only`) holds both values.

Tools are in the development folder; Python is the development environment's (MuJoCo 3.14.0, numpy 2.5.4, scipy
1.18.1, PyYAML 6.0.3, the image's pins), with the three kernel variables of `test.sh` (`OPENBLAS_CORETYPE=Haswell`,
`OPENBLAS_L2_SIZE=256`, `NPY_DISABLE_CPU_FEATURES="X86_V4 AVX512_ICL AVX512_SPR"`) and `MESA_SHADER_CACHE_DISABLE=true`;
the generator runs with `OPENBLAS_CORETYPE=Haswell` and one BLAS thread.

| after a change of | run |
|---|---|
| `voidcell/` or `voidscore/` | `tools/degenerate/practice_sync.py sync` then `check` (the practice ground's copy); the cell's tests (`tools/celltests`, `tools/cellcore/tests`, `tools/reference/tests`, `build/scoring_tests`); the QC of section 9 (`tools/qc/qc_verify.py`, `check_runs.py`) |
| the client | the same, and `tools/image/make_payload_copy.py <task>` |
| any payload file or generated block | `tools/docs/gen_blocks.py`, `check_docs.py --strict`, `behaviour_check.py`, `make_payload_copy.py`, then lint and a real validation |
| the cell's speed or the image | `tools/image/run_image_checks.sh`, `make_costs.py`, the documents' chain above, and `task.toml`'s timeout by its header's arithmetic |
| the generator, a frozen size or the simulation base | back to the selection: a new admission rule pinned before any result, the candidates regenerated, the reference run on them twice, the set selected by that rule, the packages written (format: PAX tar, members `<tag>/site.json`, `<tag>/site.xml`, mtime 0, mode 0644, uid / gid 0, xz preset 6 with CRC64), the oracle through `verify.py` twice, then the anchor and the freeze record |
| a text-only edit of `reference/`, `voidcell/`, `voidscore/` or the generator | prove the final states unchanged (the reference on a development site, the regenerated packages byte for byte), record the trees' new sha256 next to the pinned ones; any byte of `tests/` after the freeze is a dated revision with the oracle through `verify.py` again |
| anything | `tools/scan/scan.py` (with its development-text scan; read its report with `tools/image/scan_summary.py`, which counts the rows that are only `tests/setup_payload/` copies as declared); the three repository tests of `repo_tests/` |

The oracle through the real `verify.py` outside a container (not root: no lockdown, every other gate as in the image):
the stage a byte copy of `tests/`, the submission made by `solution/solve.sh`, the root-owned givens copy built from
`setup_payload/` as `test.sh` builds it (`tools/select/run_oracle_verify.py --work <dir> --tag <name>` predates
`given_hashes.json` and stages only the client, which `verify.py` now refuses); expected: ten episodes of 1.000, the
ten final states of `hidden/seeds.json`; about 57 minutes on the development host, two runs at once, at load 35-100 of
48 cores.

## 11. Known limitations

1. **One model episode is the only measurement of difficulty.** By the maintainers' rule the only evidence built is the
   privileged reference, the degenerate submissions and the grader's QC. Version 1.0's one model episode read 1.0, and
   this version exists because of it. One model episode on this version (2026-11-19) read 0.23; the pull request that
   adds this version describes it. It ran on these files as they were before the verdict kit `robotics_grader/` was
   refreshed on 2026-11-20, a change that touches nothing this grader calls. One episode is one reading, not a mean.
2. **The time limit is tight by arithmetic.** With k = 1.6 the maintainers' arithmetic (an estimate, not a
   measurement of any program) leaves a careful program that does not know the site, at the reference's pace, about
   16 s for point 3 on an average site when it tries the mouths in the worst order, and about 12 s for the whole trip
   on an average site in the average case; on the longest sites both are negative. What such a program loses is
   mostly the retrieval (F 0.85) and the third mouth tried. k = 1.7 (1920 s) was the alternative.
3. **The simulated robot is not a calibrated twin.** Its primitives are this model's (section 5, base assumptions).
   It needs a stop band three times tighter than the designers' code to tip on every roll; among the 64 ways the
   assembly guide allows the cables to be hooked, the mean turn per forward roll spans -25.7 to +1.2 degrees, and the
   one in use was chosen by geometry alone. `roll_100_100` and `roll_100_120` are close to the tipping threshold.
4. **Perception is probably the easy part under the classic renderer.** Plain boxes, no texture, no colour noise;
   the coloured chips and the per-site light are a small counter-measure. The art pass will change the frames.
5. **Regularities the generator leaves in.** Version 1.0's (the mouth straight in front of point 3 was the plugged
   one, else the nearest was open) does not exist in W3: given everything visible the open mouth is equally likely to
   be any of the three. Two geometric ones remain, neither naming the open mouth: the passage behind a mouth runs away
   from the corner when the mouth is the one nearest to the corner and toward it when it is the farthest (the other
   side has no room); on admissible sites the rank of the open mouth sets the way's shape (ranks 0 and 1: U; rank 2:
   L, about one in ten Z).
6. **Dead reckoning alone ends inside the full-credit circle on about 30 % of the sites.** Chaining the handbook's
   table over the reference's blind leg, from the true pose at the mouth, ends within 0.20 m of point 3 on 24 of 81
   candidates; the chamber's two walls
   are the site's own and their coordinates are known, so a program can touch them and come back to the point (in
   the design's measurement on the cell, 22 of 24 trials of that manoeuvre ended within 0.20 m).
7. **The low decoy can be forced.** With the planner told that the 0.15 m slot is open, the robot scraped through in
   1 of 20 runs of 40 pushes; the capability sheet has 0 of 32. Scoring only looks at where the robot dwells, and no
   document promises that an opening can or cannot be passed.
8. **Determinism.** The physics is single-threaded and repeats bit for bit (verified across the development host and
   the task image on one machine for version 1.0, and again between the two oracle runs of this freeze). Frames are
   equal across host and image as well, but equality of frames on another CPU generation is not guaranteed and was not
   measured; the anchor comes from a reference that uses no frame.
9. **`pose_source = "cell"`** is implemented and unit-tested in process, but no verify run and no image check used it.
10. **The practice ground's protection is the account isolation alone.** Its copy of the cell (52 files) carries the
    prototype's names and the designers' repository names, exactly as `tests/voidcell/` does. Linux leaves the
    service's `/proc/<pid>/cmdline` readable (the interpreter and the service's path, nothing else).
11. **Host DNS search domain.** On the development host a sandbox inherits a `search` line that names the institution
    the host sits in. Model episodes on this host are run through a per-process `docker` wrapper that adds
    `--dns-search .`; the task itself does not depend on it.
12. **Serial verify, long worst case.** A program that spends every budget holds the verifier for about 3.6 h; the
    limit is 19 100 s. Host load slows frames and physics; nothing in the score depends on wall clock except the
    program's own 600 s of program time and the 180 s silence limit.
13. **Costs in the documents** are the version 1.1 image's, the larger of two one-episode runs on a host at load
    19-56 (0.163 s per simulated second, 0.228 s per frame); a busier host is slower. "rolling or standing alike"
    rounds a difference of up to 28 % (0.127 against 0.099 s, 0.163 against 0.146 s per simulated second).
14. **A neighbouring task.** A search-and-rescue task with a tracked vehicle in an earthquake-struck building was in
    review for this benchmark when this version was published; this one differs in the platform, the off-board camera,
    the unseen void and the rolling primitives.
15. **Numbers the leak scan reports.** In the calibration site's plan (`given/calibration_site/site_plan.json`) three
    printed numbers equal hidden values of the frozen sites to four decimals: 0.5593 m, class P's width, which is by
    rule the gap beside every rubble block (and the mouths' width; the instruction says the gap is as wide as the
    openings); 1.0066 m, the plan's x of an opening's edge, equal to a half-size of one fill block under the slab of p3
    (a coincidence); 0.2907 m, the plan's block width, equal to the across-size of p2's one rubble block (0.290656 m,
    a coincidence). The scanner's exemption table lists them, each with its reason, next to the family's three other
    class sizes, and the scan passes. For review: the handbook's spool
    radius 3.175 mm against a band coordinate of h02, and the count 127 in the handbook ("0 of 127") against h08's
    seed.

## 12. Record

| date (UTC) | what |
|---|---|
| 2026-11-17 | version 1.0: generator W2; cell, station, reference on 107 candidates (ysim 0.3.1); scoring, verify, practice ground, calibration site; selection and freeze (rule pinned 17:24:06, frozen 18:24:46, anchor 1.0); the grader's QC; the agent's documents; image, layout, two real validations; one model episode (read 1.0) |
| 2026-11-18 | version 1.1 designed: site family W3 with the rubble, credit 0.20 / 0.40 m, weights 0.05 / 0.15 / 0.80, time-limit factor 1.6, the kernel pin of the program's process; generator, calibration site (calgen 0.2.0), reference 1.1.0 |
| 2026-11-19 | admission rule pinned 00:51:35; 220 candidate packages, 139 reference runs; the agent's documents rewritten for version 1.1; a text-only revision of the cell, the reference (1.1.1), the generator and the maintainers' files (no computed value changed, final states proven unchanged); the 13 packages regenerated with the clean envelope; frozen 2026-11-19 13:21, anchor 1.0; after it, the grader's QC on the frozen set; the image rebuilt, and from its measurements the documents' cost figures and `task.toml`'s header and verifier timeout (19 100 s); dated revisions of `tests/`, each in the freeze record (the givens' copy and their pins with the cost figures; these notes; `hidden/seeds.json`'s rubble counts, a field nothing reads, and the two pins of its sha256); a real validation through the `docker` wrapper on the final bytes |
| 2026-11-19 | one model episode on the frozen version (read 0.23) |
| 2026-11-20 | the task's card; the verdict kit `robotics_grader/` refreshed to the repository's current copy (five files, nothing this grader calls changed); these notes brought up to date; a real validation on the final bytes |
| open | a later version with dressed sites and with heaps of rubble and of spilled fill on the open floor |

# endoscope_camera_assistance_from_video: design notes for maintainers and validators

The agent-facing contract is `instruction.md`; the device is described in
`environment/setup/payload/given/holder_manual.md` and implemented in `holder_model.py` beside it. This page records
the design decisions, what the grader checks and why, what is known to be rough, and, in section 7, a dated status
log: **what was verified, where, and what was not**. Version 0.1, 2026-11-24; the task name is provisional (not yet
registered in the coordination sheet; it appears only as the folder name and in `task.toml`).

**Design rule: the agent gets what the device's engineer would have, and nothing a camera holder in an operating
room would not.** It gets the holder's kinematics and manual, the intrinsics of both scopes, a labelled development
set of recorded procedures, and a commissioning service that plays development procedures for its controller. Its
controller gets the endoscope's frame and the holder's joint readings, and nothing else. It does not get the
simulator's code (the procedure generator, the scene, the judging), any graded procedure, ground truth on the wire,
or a score of anything it tries.

## 1. The job and why it is worth measuring

Holding the laparoscope is the assistant's job in laparoscopic surgery, and it is a function that surgical robots
already sell: voice-, head- and instrument-tracking camera holders (AESOP, EndoAssist, ViKY, AutoLap, Soloassist),
and the camera arm of every multi-arm system. The autonomous version, a holder that follows the work from the video
alone, is an old research problem that is still not a solved product, because the hard part is not the kinematics:
it is knowing, from a picture full of blood-red tissue, smoke and highlights, where the working instruments are,
which instruments matter, and when the surgeon needs the picture to stand still.

The task measures that in closed loop. A four-joint holder (pan, tilt, insertion, roll about the trocar's remote
centre of motion) carries a 0 or 30 degree laparoscope; a recorded procedure plays; the agent's controller sees the
endoscope's RGB frame (one control step old) and the four joint readings, and commands joint rates. It is scored on
the fraction of control steps with a usable view. The engineering is real end to end: perception that survives
unseen instruments and tissue, telling the surgeon's instruments from the assistant's, noticing an exchange, telling
which scope is fitted from how the picture moves, noticing a smeared lens from the picture, visual servoing through
a servo with lag and a video chain with latency, and standing still at the right times.

## 2. The simulator (`tests/endosim/`)

Three layers, split so that the part that scores never touches the part that draws:

| file | what | who can read it in the agent's sandbox |
|---|---|---|
| `model.py` | the holder, the scopes, the view rules' constants: the public model | the agent (`~/given/holder_model.py`, a byte copy) |
| `tissue.py`, `procedure.py` | the cavity's geometry; the generator of recorded procedures | the service account only |
| `episode.py`, `session.py` | one run: the servo, the lens, the judging; observations with latency | the service account only |
| `scene.py`, `looks.py`, `post.py`, `render_server.py` | the Isaac Sim process that draws frames | the service account only |
| `render_client.py` | the parent's end of the renderer (stdlib only) | the service account only |
| `recorder.py` | the development set's recorder and its scripted camera assistant | the service account only |

`environment/commission_service/endosim/` is a byte copy of this folder (the agent commissions against the simulator
that grades); `tests/review/check_copies.py` checks it, the two copies of `model.py` in the payload, and the vendored
kit.

**The holder** (`model.py`, manual sections 2-3). `R_WS = Ry(-pan) Rx(tilt)`, the lens at `insertion * a`, the camera
`R_WC = R_WS Rz(roll) Rx(-d)` with d = 0 or 30 degrees. Rate commands, clipped (0.5 rad/s, 0.5 rad/s, 0.05 m/s,
0.8 rad/s), a first-order servo lag of 0.12 s with acceleration limits, five servo ticks per 0.1 s control step. The
rate limits were sized against the procedures: the surgeon's long moves peak near 0.08 m/s, which a holder at
0.5 rad/s can follow with a short lag at a working distance of 0.2 m from the pivot, and no faster.

**The cavity** (`tissue.py`). A height field: a bowl (flat floor 170 mm below the pivot, rising into the walls), 5 to
8 smooth organ mounds, a fine ripple, under an ellipsoidal abdominal wall that carries the trocars. **The tissue is
a non-physical surface**: static geometry that moves kinematically with the patient's breathing (one z offset, 1.5
to 3.5 mm, 0.2 to 0.3 Hz). PhysX deformable bodies need GPU dynamics, and GPU dynamics do not repeat across GPU
models, which a benchmark's score has to; a camera holder does not push on tissue, so the job loses nothing. The
floor is 170 mm down so that the home view (scope withdrawn) never satisfies the zoom band: a camera that never
moves scores 0 on every procedure of the fixed sets (`hidden/selection.json`, `never_moves`).

**The procedures** (`procedure.py`). Instrument motion is generated from procedural primitives; **no surgeon
recording is used or shipped** (section 6, open items). A procedure is a path of working sites across the floor
(the working area travels: 0.3 to 1 m of path in 80 s), and per site a travel of both hands (minimum-jerk, 30 to
42 mm/s on average, the right hand leading) and a dwell made of gestures: hold, dissect (short strokes of the right
hand, regrasps of the left), suture bites (loops), pulls and stretches (one hand goes out 1 to 6 cm and comes back),
clips. Four kinds differ in the mix and in how far the work travels: dissection, suturing, exploration, clip and
cut. Events: an instrument exchange (one hand withdraws through its trocar, stays out 6 to 10 s, comes back,
possibly as another instrument; it does not count while away), the assistant's third instrument (never counts;
holds beside the work, follows a move late or not at all, dabs into the middle now and then), smoke bursts with
dissection. Drawn per procedure: the kind, the scope, the events, the port placement (within 12 mm of nominal), the
cavity, every instrument's look (jaw type, shaft colour and finish, marker bands, how much blood covers the working
end), the tissue's look (palette, fat, vessels, wetness, relief), what lies in the field (gauze swabs, clips), the
light, the camera's white balance and noise. The tips work at the tissue (1 to 9 mm above its resting surface), so
the breathing tissue buries a low tip from time to time. Everything comes from a keyed BLAKE2b stream per seed, in
plain float64 arithmetic, so a seed gives the same procedure on every machine and numpy version. The fixed sets pin
kind, scope and events per procedure in a balanced table and draw the rest (`tests/review/make_sets.py`: the rule,
written before the draw; no draw rejected).

**Two pools of looks.** `generate(..., pool="dev")` draws a development procedure's looks from a narrower pool than
a graded procedure's (`pool="all"`): 5 of the 11 shaft finishes, 3 of the 6 tissue palettes, staining of the working
end up to half-way to the tissue's colour (up to 0.9 when graded), less clutter. The geometry, the ports, the kinds of
procedure and the gestures are the same. The manual says so (section 7), without the lists. This is where the
perception difficulty is meant to come from: a detector fitted to the ten development recordings meets instruments
and tissue it has not seen, as a holder sold to another hospital would. It was added after the first pixel baseline
(status log): a small net trained for two minutes on eight development recordings found 99 % of the tips in the two
held-out ones, within 4 px in the median, when development and graded looks came from one pool.

**Rendering** (`scene.py`, `render_server.py`, `post.py`). One headless Isaac Sim 6.2 process draws the frames: the
RTX Real-Time path tracer through a Replicator render product at 640 x 480, one subframe per frame, the render
product's settings pinned (`RENDER_SETTINGS`), physics never stepped. The scene is procedural USD: the textured
floor and wall (colour and normal maps generated per procedure, `looks.py`), instruments built from primitives with
hinged jaws, a cannula at every trocar, the scope rig (camera, three small sphere lights round the lens as the
coaxial light). Nothing of NVIDIA's asset library is used. Image-space effects after the render (`post.py`): the
camera head's automatic exposure, smoke (a drifting haze and a plume above the instrument that makes it), motion blur
along the camera's own image motion (the shutter is open for half a control step), a smeared lens (blur and
stains), white balance, sensor noise, the endoscope's image disk, JPEG at quality 90. Facts measured while building
it (RTX 6000 Ada, driver 570, 2026-11-24):

* **The stage is in centimetres.** With `metersPerUnit = 1` the renderer ignored sphere lights a few millimetres
  across (the picture stayed black however high the intensity); with the same scene authored in centimetres the
  same lights work at once. `scene.K = 100`.
* **A frame can be the previous frame.** With one subframe, the annotator returned the frame before the last USD
  change on 20 to 35 % of captures (3 % with `/app/hydraEngine/waitIdle`, 0 % with three subframes at three times the
  cost). Hence the frame clock: a small emissive quad in the camera's own frame, in the top-left corner outside the
  image disk, whose slot encodes the frame's serial number modulo 8, in front of a black plate that reflects nothing.
  The server makes one plain app update after posing the stage, captures, reads the clock back and renders again
  until it shows the serial it was asked for (at most 8 renders; then the renderer is restarted). With the update,
  0.6 to 0.8 % of the frames need a second render. The clock sits 2 mm in front of the lens; when something at the
  lens hides it (the scope run through an instrument's shaft, which nothing prevents), the third capture of the
  unchanged stage is taken as it is (the lag was never more than one capture). The corner is masked before a frame
  leaves the server.
* **Pixels do not repeat.** Two renders of one state on one GPU differ in about half their pixels by a small amount
  (the brief's measurement: mean absolute difference 0.36 of 255). Nothing scored reads a pixel.
* **Replicator's first-frame wait.** The orchestrator waits for "asset loading" before its first frame, up to
  `/exts/omni.replicator.core/maxAssetLoadingTime` (30 s by default in this version; the smoke test saw about 100 s
  with physics playing). The server sets it to 5 s; with this scene the first frame came in under a second.
* **Kit opens a context on every GPU of the machine**, whatever `active_gpu` says: natively the renderer is run in a
  mount namespace that hides the other GPUs (status log). A sandbox has one GPU.
* **No network is needed, a loopback is.** In an empty network namespace with the loopback up, the renderer starts
  as fast as with a network (18 s on a warm cache) and renders the same; with the loopback down its start took 244 s.
* **Cost.** About 66 to 70 ms per delivered frame here (one app update, one capture, the image effects, the JPEG),
  7 s to build a procedure's scene (two texture pairs generated and loaded), 16 to 29 s to start Isaac Sim on a
  warm cache. A cold cache (a fresh container) was not measured here; `hazard_bay_build` measured about 7 minutes
  for the same stack.
* **The camera is what the model says.** The canary (below) puts a marker ball at known points and compares where
  it lands in the frame with `model.project`: within 1.1 px for both scopes in every run here.

## 3. What the grader checks (`tests/verify.py`, `endosim/episode.py`)

**The loop.** `test.sh` runs `verify.py` as root in a separate verifier sandbox. It returns the kit's zero verdict
at once when there is no `run.sh` (the untouched pass never starts Isaac Sim). Otherwise it starts the renderer as
the account `endosim` from a 0700 copy of the renderer's files (the scene and image code only: the generator and the
episode stay in the root-only stage folder), runs the canary, starts the controller through the kit's `Solver` (the
agent's account; the kit clears that account first), and plays the fixed hidden set
(`hidden/procedures.json`: 12 procedures x 800 steps, the same for every submission). Per step: the session renders
the current state, hands the controller the frame of one step earlier and the current joint readings, and applies
the reply through `model.clip_command`.

**The view rules** (`Episode._judge`, manual section 6). At every step reached, from the simulator's state only:
every counting tip in the central safe zone; every counting instrument's apparent shaft diameter in 22 to 44 px;
the horizon error within 15 degrees; during fine work (a rule on the tips' own motion, so that it is observable: every
counting tip within 9 mm of its mean over the last second) the working point's image moved by the camera no faster
than 48 px/s; the lens clean. A lens within 4 mm of the breathing tissue is smeared and the scope stops; it is clean
again after 4.0 s held at insertion <= 25 mm. `useful_view_fraction` = usable steps / steps over the whole set.
There is no other term, weight or penalty: a smeared lens costs exactly the steps it makes unusable.

**What standing still costs.** A reply that is late (10 s), malformed or missing, and a used-up reply-time budget
(0.25 s x 9600 steps = 2400 s in all, probe copies included), end the controller's control of that procedure: the
remaining steps run with the holder commanded to stop, without rendering, and are judged like any other step. The
kit restarts the controller before the next procedure (three restarts). Nothing is dropped from the average.

**The match lock.** On 6 % of the steps the step is asked twice with the same `t`: the true observation and a copy
whose picture is shifted inside the disk (128-224 px, 72-144 px) and whose pan, tilt and roll readings are offset
(0.2-0.4 rad), in a hidden order; a quarter of the pairs must differ in the action. The perturbation is large on
purpose: the right controller for this job stands still inside a dead zone, and a small perturbation would not move
it. It is also physically impossible (the holder cannot turn 0.2 rad in a step), and the instruction says so: a
controller keeps its memory by step index and lets only a possible observation become memory (`~/example` shows the
pattern; the reference's score is identical with and without probes). As everywhere, the lock catches a controller
that ignores its input and proves nothing more; isolation keeps the score honest.

**The canary.** Before the controller starts, the renderer draws a marker ball at five known points for each scope
and reports where it landed; `check_canary` requires every marker found within 2.5 px of `model.project` and a frame
that is neither black nor white. A failure is a task error (`fail`), never a zero: it means the sandbox cannot
render (no RTX GPU, a host driver without its graphics libraries) or renders another camera than the published
one. A renderer that dies or cannot bring the frame clock up to date is restarted and the frame drawn again (three
restarts in an evaluation), since the episode's state lives in the grader's process.

**Isolation.** Three accounts in the verifier: root (the grader, the procedures, the judging), `endosim` (the
renderer: scene and image code, the scene's current state, no future), the agent's account (the controller). Kit's
shared folders are made the renderer account's alone before it starts, its shared memory in `/dev/shm` is closed to
other accounts before the controller starts, and the renderer's wire is the pair of pipes the grader holds. The
controller receives frames and joint readings and nothing else.

## 4. The reference and the anchor

`solution/reference/controller.py`. The reference does not look at the picture: its submission carries a 256-bit
token (`hidden/oracle_token.sha256`), and for it alone every observation holds `privileged`: the scope fitted,
whether the lens is smeared, and each instrument's tip in the camera frame, with the frame's latency. That is all it
gets beyond the public wire. It is not told which instruments count, when the surgeon does fine work, where the
tissue is, or what comes next, and it moves the holder only by rate commands. It rebuilds the tips in the patient
frame from its own joint history, tells an exchange from the motion (a hand leaving along its shaft while the other
stays) and keeps the view where the work is until that hand is back, aims with the model's Jacobian at a working
distance set by the tips' spread, feeds the tips' velocity forward, levels the horizon in closed form, stands still
while the tips do (its own estimate of the fine-work rule), creeps under the image-speed limit when it has to move
during fine work, and withdraws to clean a smeared lens. The anchor is what perfect perception achieves under the
task's information rule; what it loses, it loses to the holder's speed on long moves and to the first seconds of
each procedure.

The anchor's value, its spread and their source are in `anchor.json` and in the status log. Because the reference
reads the state and never the picture, its score is the same number on every run and with or without rendering
(`tests/review/fast_eval.py` plays the same sessions with a null renderer in about a minute; that is how it was
tuned). A submission that reads the picture has no such luck: its frames differ from run to run in their last bits.

**A pixel baseline** (`tests/review/pixel_baseline/`, maintainers' evidence, not part of grading): what the task
looks like from the public interface. A 1.3 M parameter net, trained for 95 s on eight development recordings as an
agent could train it (tip heat maps for the left and the right instrument, their apparent size, the smeared lens),
feeds the reference's own control law in place of the privileged observation, works out the scope from how the
rebuilt tips drift when the camera moves, and gets no token. On the two held-out development recordings it finds
98 % of the tips, 3.6 px off in the median. On the hidden set it scores useful_view_fraction 0.342 (reward 0.39),
per procedure 0.04 to 0.67, against the reference's 0.903: the zoom band fails on 34 % of the steps (the apparent
size, and with it the depth, is the weakest thing it reads), the safe zone on 42 %, the steadiness rule on 23 % (a
detection that jitters moves the camera during fine work). So the task can be done from the frames, a first learned
detector is far from enough, and what is missing is engineering an agent can do: filtering in time, a better
reading of depth, looks the development set does not show. It was written once and never tuned in closed loop.

## 5. The agent's side

* `~/given/holder_manual.md`, `holder_model.py`: the device, the scopes, the set-up, the exact rules.
* `~/given/dev_procedures/`: ten development procedures recorded as video with labels, by the commissioning service
  on the sandbox's own GPU during the first minutes of the session (so the frames come from the renderer the grader
  will use; nothing is fetched or committed). The camera in them is held by a scripted, deliberately ordinary
  assistant (`recorder.Assistant`: its view is usable on 52 to 84 % of the steps); two recordings include a lens
  touching tissue and its cleaning (76 smeared frames). 4010 frames, about 320 MB.
* The commissioning service (`environment/commission_service/commissiond.py`, account `endosim`, started by
  `setup.sh`): the grader's simulator on the ten development procedures, 12 sessions, one at a time, the grader's
  observations and limits, no ground truth, no labels, no score, no summary. `~/commission/run_session.py` runs a
  controller folder through a session as the grader will, repeated requests included.
* `~/example/`: a controller that speaks the wire, keeps memory by step index and holds still.

The development procedures were drawn like the graded ones (same generator, same rule, other seeds); the graded
ones are in neither the image nor the sandbox.

## 6. Known limitations and open items (version 0.1)

* **Nothing here has run in the engine.** No Docker on the development server: the image has not been built, setup
  and the commissioning service have not run as their accounts, `ale validate` and `ale run` have not run, and the
  anchor was measured natively. See the status log for exactly what was and was not exercised.
* **Difficulty is unmeasured.** The target is a frontier-model reward of roughly 0.3 or below from perception and
  control; no frontier model has been run. The one data point is the pixel baseline of section 4 (reward 0.39 with
  the reference's control law handed to it, which an agent does not have). If a model saturates the task, the levers
  that keep it honest are in the generator (the looks pool, denser smoke, more assistant interference, longer
  exchanges, tips buried deeper), not penalties.
* **Numerics are not pinned.** The simulator and the reference compute with numpy's default kernels; on another CPU
  a threshold can flip on a step. The reference repeats to the last digit here; `full_at` 0.98 is the margin.
* **The controller is not confined to no network.** It runs through the kit's `Solver` as the agent's account; the
  engine opens egress during verify (`docs/security-model.md` section 4). `hazard_bay_build`'s `nonet_exec.py` is the
  model if a maintainer wants a seccomp filter here; it was not added because it could not be tested on this server.
* **No repository-level test.** The rule here was one folder per pull request, so the copy check lives in
  `tests/review/check_copies.py` instead of the repository's `tests/` (compare `tests/test_pump_skid_commissioning.py`).
* **GPU models.** `task.toml` names RTX-4090, RTX-5090 and RTX-PRO-6000 from `hazard_bay_build`'s validation of the
  same Isaac Sim stack; this task has run on none of them (status log).
* **Recorded surgeon motion.** The instruments move by procedural primitives. Real recordings (JIGSAWS kinematics,
  Cholec80 / CholecT50, EndoVis, SurgT and similar) would be more convincing, but no dataset's licence has been
  checked for redistribution in a public benchmark image, so none was downloaded or shipped. Open question for the
  maintainers: is there a recorded-kinematics dataset whose licence clearly allows redistribution? If so, the
  generator's gestures could be replaced or calibrated by it without touching the wire or the rules.
* The tissue is rigid and only breathes; instruments do not deform or cut it, there is no bleeding, and smoke and a
  smeared lens are image-space effects, not volumes. The instruments are built from primitives, not from CAD of real
  instruments. The picture is rectified: real laparoscopes have barrel distortion.
* The lens touches tissue only (not instruments, not the wall above); the scope's shaft collides with nothing.
* The instruments do not wait for the camera: a real surgeon slows down when the view lags.
* The fine-work rule is a rule on tip motion (observable), not a surgical phase label.
* The match lock's copy can be recognised by its joint readings; that is stated to the agent and is by design. It
  is not "built the way the real observation is" (upstream #173's rule for probe copies): the status log's entry
  on that kit says why it was left so and what the alternative is.
* The development set is rendered at setup time on the sandbox's GPU: it costs the first minutes of the session and
  differs in its pixels from run to run (the labels do not).

## 7. Status log

Newest first. Each entry says where the numbers come from; "here" is the development server (Ubuntu 22.04,
RTX 6000 Ada, NVIDIA driver 570.195.03, Isaac Sim 6.2.0.0 + Isaac Lab 3.0.0 in a native Python 3.12 venv, no root,
no Docker).

### 2026-11-24, evening: the kit of upstream #173 (tree `67c47c8`; the native runs have not been repeated on it)

`upstream/main` moved to `1c13a47` after this branch was cut: #173 changed `shared/robotics_grader/` (the solver
and everything it starts run at nice 19; `replies_differ` is the lock's one rule; `probe.stale_copy`;
`deadline.VerifierDeadline`) and copied it into every task. The branch was merged with `upstream/main` and the kit
copied again (byte-identical to `shared/`). The task against #173's three findings:

1. *The solver's priority.* The kit's own. `verify.TimedSolver` keeps the kit's `_spawn`, and the native tool's
   `LocalSolver` replaces `_spawn` only, which the kit starts from its low-priority thread like any other. Nothing
   to change. The controller now runs at nice 19 beside the renderer; on a loaded host its replies may take
   longer, and they are what the 2400 s budget counts.
2. *One rule for probe pairs, in practice too.* `~/commission/run_session.py` printed how many repeated requests
   changed the action and counted from the raw replies, while the lock compares the commands clipped to the
   holder's rate limits: two replies that differ only beyond a limit counted as changed there and do not for the
   grader. The launcher now has `differ(first, second)`, the lock's rule on the clipped commands, and
   `tests/review/check_runner_rule.py` holds it against the kit's `replies_differ` (8 pairs, no simulator).
3. *A probe copy must be built the way the real observation is.* **This task's copy is not, on purpose, and that
   is a point for the maintainers.** The copy is the same frame shifted in the image plane with the pan, tilt and
   roll readings offset by 0.2 to 0.4 rad: more than the holder can turn in a step, and unrelated to the shift of
   the picture. The instruction says so and tells the agent to keep such an observation out of its controller's
   memory (section 3 has the reason: the right controller stands still inside a dead zone, and a perturbation
   small enough to be possible would not move it). #173's rule is written for copies that are meant to pass for
   true observations; this one is declared. What it cannot do is catch a program that recognises the copy and
   answers it differently without reading the picture otherwise; nothing here needs that, since no procedure can
   be replayed (the graded ones are in neither the image nor the sandbox). If the maintainers want the rule to
   hold here as written, the way is to render the copy as a true second observation (the frame the camera would
   give with the holder a small, possible step away, with the joints of that pose) and to size the lock for it.
   Left as it is.

What ran on `67c47c8`: `ale lint` ok; `scripts/lint_domain.py` ok; the kit comparison; `scripts/gpu_types.py
check`; `scripts/task_assets.py check`; the kit's selftest; `review/check_copies.py`; `review/check_runner_rule.py`.

**Not run on this tree: the native runs.** The series (the reference, no submission, the camera that never moves,
random rates, a commissioning session through the changed launcher) was started and stopped within a minute,
because the server's free GPU had just been taken by another job. The numbers of the entry below are the earlier
tree's. The grader's own code is unchanged since then; the kit differs in the solver's priority.

### 2026-11-24: version 0.1 built; verified natively, not yet in the engine

**Verified here.**

* `ale lint` on the folder (pinned engine `bfb7505`, run from the main checkout's `vendor/ale`): ok.
* `scripts/lint_domain.py` on the folder: no problem. `scripts/task_assets.py check`: 0 problems (the task has no
  assets). `scripts/gpu_types.py check` (engine environment): the three names accept real models.
* The kit: `tests/robotics_grader/` byte-identical to `shared/robotics_grader/` (`diff -r`; also
  `tests/review/check_copies.py`, which checks the commissioning service's copy of `endosim/` and the two payload
  copies of `model.py` as well); the kit's selftest passes.
* The grader's entry point on an untouched home (`tests/verify.py` itself, natively): the kit's zero verdict, exit
  0, at once, no renderer started.
* **The grader's episode loop, natively** (`tests/review/run_native.py`: `verify.grade` and `verify.score` unchanged,
  the same hidden set, sessions, probes, budgets, renderer and canary; only the two launchers differ: the controller
  and the renderer run as the calling account instead of through `runuser`). Every run below was inside a mount
  namespace that shows GPU 0 only and an empty network namespace with a loopback (no route out):

  | submission | procedures | useful_view_fraction | reward | notes |
  |---|---|---|---|---|
  | none | - | (zero verdict) | 0 | no `run.sh`: the honest zero |
  | never moves (`~/example`) | 4 | 0.0000 | 0 | lock fails, 0 of 210 probes; render-free, all 12: 0.0000 each |
  | reference, run 1 | 12 | 0.9034375 | 1.0 | 8673 of 9600 steps; probes 570/570; 794 s wall, 68.7 ms a frame |
  | reference, run 2 | 12 | 0.9034375 | 1.0 | probes 578/578; 836 s wall |
  | reference, run 3 | 12 | 0.9034375 | 1.0 | probes 534/534; the renderer lost the GPU once and was restarted (below); 919 s wall |
  | pixel baseline (section 4) | 12 | 0.3422 | 0.39 | probes 585/589; 508 s of controller time |
  | `bad:diver` | 1 | 0.0000 | 0 | 1623 lens contacts, lens rule fails on 96 % of the steps; lock fails |
  | `bad:cleaner` | 1 | 0.0000 | 0 | one cleaning counted after 4 s in the cannula |
  | `bad:crash` | 2 | 0.0000 | 0 | aborted at step 100 twice, the rest stood still; one restart between |
  | `bad:garbage` | 1 | 0.0000 | 0 | malformed action at step 50: aborted, the rest stood still |
  | `bad:slow` | 1 | 0.0000 | 0 | reply budget (200 s for one procedure) used up at step 319, the rest stood still |
  | `bad:jitter` | 1 | 0.0013 | 0.00 | random rates pass the lock (56/56), as documented, and earn nothing |

  The reference's three runs are the same number: it reads the state, not the picture, and its memory is keyed by
  the step index, so neither the renderer nor the probes move it. The render-free loop (`fast_eval.py --hidden`)
  gives the same 0.9034 and the same per-procedure values (0.939 0.865 0.953 0.926 0.878 0.914 0.838 0.934 0.875
  0.928 0.926 0.868; `hidden/selection.json`). A camera parked at the best fixed pose found by a grid search with
  the hidden set in hand scores 0.111.
* The render canary in every run: all ten markers found, at most 1.11 px from where `model.project` puts them.
* **A renderer failure, recovered.** In the reference's third run the renderer lost the GPU after 833 s
  (`VkResult: ERROR_DEVICE_LOST`, "A GPU crash occurred"; the GPU is shared with other users' and another
  session's Isaac processes). The grader started a fresh renderer, rebuilt the scene, drew the frame again and
  finished the run on the same number (`renderer_restarts` 1 in the verdict's metrics). One loss in about
  85 000 frames rendered here; an evaluation allows three restarts.
* An earlier full run of the reference in a network namespace whose loopback was down: the same score, the
  renderer's start 244 s instead of 14 to 22 s.
* The commissioning service, natively (the daemon as the calling account, its paths overridden by environment
  variables): renderer up and canary passed in 24 to 40 s; the development set recorded in 7 min 15 s (ten
  recordings, 4010 frames, 322 MB; labels checked against the pictures by overlay); `status`, a session refused
  while recording, an unknown procedure refused, a session through `~/commission/run_session.py` with the example
  controller (the session counted, 12 to 11; repeated requests asked and compared; frames and log saved).
* `solution/solve.sh` against a copy of the payload: the reference submission is built and its `run.sh` speaks the
  wire.

**Not verified: none of this could be done on this server.**

* **The Docker image.** `environment/Dockerfile` has never been built. It follows `hazard_bay_build`'s line by line
  for the Isaac install; the account `endosim`, the 0700 service folder and the self-checks at its end are new and
  untested.
* **`environment/setup/setup.sh`** as root in a sandbox: staging, the service started through `runuser`, the wait for
  its renderer inside the engine's 1800 s bound. A cold cache in a fresh container was not measured.
* **`ale validate`** (the untouched and the oracle episode), and with it: the engine's stages, separate
  verification and the restore of `/home/user/submission`, `tests/test.sh` under `ALE_VERIFY_PYTHON`,
  `verify.engine_renderer` (the renderer as `endosim` through `runuser`, Kit's folders handed to that account, the
  shared memory sealed), the kit's `Solver` through `runuser` with the agent's account cleared, the timeouts of
  `task.toml`.
* **`ale run` with frontier models.** The difficulty target is unmeasured.
* **A cloud-listed GPU model** (RTX 4090, RTX 5090, RTX PRO 6000, H100, H200). Everything above was rendered on an
  RTX 6000 Ada with driver 570, which makes DLSS fall back to an older denoiser preset: the frames here are not
  the frames an evaluation host draws. The score does not read frames, but a model-specific failure (a crash at the
  first render, a darker picture, a renderer too slow for the budgets) would show only there.

**`gpu_types`.** `task.toml` names `["RTX-4090", "RTX-5090", "RTX-PRO-6000"]`, cheapest first, in both tables. The
reason is not a test of this task: it is the list on which `hazard_bay_build` validated the same Isaac Sim 6.2 RTX
stack, and it keeps the task off the H100 and H200, where that stack's rendering is unproven or known to fail for
other Isaac tasks. To make the list this task's own, per model: `ale validate` (the canary's numbers are in the
oracle episode's `verification.json` trace and the grader's log), a frame from `tests/review/render_preview.py`
compared with one from a validated model, and the verify wall time against `[verifier].timeout_sec`. A model is
added or dropped on that evidence; the key is left out only if all five run it.

**The anchor.** `anchor.json` holds 0.9034 with `full_at` 0.98, measured natively as above. It has to be measured
again through `ale validate` before the task is submitted; if the engine's value differs, the engine's is the anchor.

**Decisions made without the owner, to confirm.**

* The task name is provisional (folder name and `task.toml` only).
* `gpu_types` as above. Agent budget 4 h, verifier 9000 s, 8 CPUs, 32 GB, one GPU.
* The graded procedures draw their looks from a wider pool than the development set (section 2). It was added after
  the first pixel baseline showed that perception was easy when both came from one pool. The manual and the
  instruction say so.
* The match lock's perturbed copy is recognisable by its joint readings, and the instruction tells the agent how to
  keep it out of its controller's memory.
* The development set is recorded at setup on the sandbox's GPU instead of being fetched from the asset dataset
  (nothing to upload, the frames come from the renderer that grades; it costs the first 7 minutes of the session).
* No recorded surgeon motion and no dataset (section 6).

**Next, in order.** (1) Build the image on a machine with Docker and an RTX GPU; fix what the build shows. (2)
`ale validate`; read the canary and the timings; re-measure the anchor. (3) The same on each cloud GPU model;
settle `gpu_types`. (4) `ale run` with frontier models; if they are above about 0.3, raise the perception
challenge in the generator. (5) Register the name in the coordination sheet; open the pull request. A card for the
maintainers, as a suggestion: title "Hold a laparoscope by video alone", summary "Write the controller of a
four-joint camera holder that keeps a surgeon's instruments in a usable view through recorded procedures, from the
endoscope's video and its own joints alone.", platforms `["surgical"]`, stages `["behavior"]`. Until the card is in
`tasks/cards.toml`, the repository's `tests/test_task_cards.py::test_the_repository_has_a_card_for_every_task`
fails for this folder, as for any contributed task.

**How the native runs were isolated.** Isaac Sim opens a context of about 436 MiB on every GPU of the machine,
whatever `active_gpu` says. Every Isaac process of this task was therefore started inside
`unshare -r -m` with `/dev/null` bound over the other GPUs' device nodes (then mapped back to the caller's uid), and,
for the measurements above, inside an empty network namespace with the loopback up.

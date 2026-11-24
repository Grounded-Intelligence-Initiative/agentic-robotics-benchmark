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
cavity, every instrument's look (jaw type, shaft colour and finish, marker bands), the tissue's look (palette, fat,
vessels, wetness, relief), the light, the camera's white balance and noise. Everything comes from a keyed BLAKE2b
stream per seed, in plain float64 arithmetic, so a seed gives the same procedure on every machine and numpy
version. The fixed sets pin kind, scope and events per procedure in a balanced table and draw the rest
(`tests/review/make_sets.py`: the rule, written before the draw; no draw rejected).

**Rendering** (`scene.py`, `render_server.py`, `post.py`). One headless Isaac Sim 6.2 process draws the frames: the
RTX Real-Time path tracer through a Replicator render product at 640 x 480, one subframe per frame, the render
product's settings pinned (`RENDER_SETTINGS`), physics never stepped. The scene is procedural USD: the textured
floor and wall (colour and normal maps generated per procedure, `looks.py`), instruments built from primitives with
hinged jaws, a cannula at every trocar, the scope rig (camera, three small sphere lights round the lens as the
coaxial light). Nothing of NVIDIA's asset library is used. Image-space effects after the render (`post.py`): the
camera head's automatic exposure, smoke (a drifting haze and a plume above the instrument that makes it), a smeared
lens (blur and stains), white balance, sensor noise, the endoscope's image disk, JPEG at quality 90. Facts measured
while building it (RTX 6000 Ada, driver 570, 2026-11-24):

* **The stage is in centimetres.** With `metersPerUnit = 1` the renderer ignored sphere lights a few millimetres
  across (the picture stayed black however high the intensity); with the same scene authored in centimetres the
  same lights work at once. `scene.K = 100`.
* **A frame can be the previous frame.** With one subframe, the annotator returned the frame before the last USD
  change on 20 to 35 % of captures (3 % with `/app/hydraEngine/waitIdle`, 0 % with three subframes at three times the
  cost). Hence the frame clock: a small emissive quad in the camera's own frame, in the top-left corner outside the
  image disk, whose slot encodes the frame's serial number modulo 8, in front of a black plate that reflects nothing.
  The server reads the clock back from every frame and renders again until it shows the serial it was asked for (at
  most 8 renders; then the renderer is restarted). The corner is masked before a frame leaves the server.
* **Pixels do not repeat.** Two renders of one state on one GPU differ in about half their pixels by a small amount
  (the brief's measurement: mean absolute difference 0.36 of 255). Nothing scored reads a pixel.
* **Replicator's first-frame wait.** The orchestrator waits for "asset loading" before its first frame, up to
  `/exts/omni.replicator.core/maxAssetLoadingTime` (30 s by default in this version; the smoke test saw about 100 s
  with physics playing). The server sets it to 5 s; with this scene the first frame came in under a second.
* **Kit opens a context on every GPU of the machine**, whatever `active_gpu` says: natively the renderer is run in a
  mount namespace that hides the other GPUs (status log). A sandbox has one GPU.
* **The camera is what the model says.** The canary (below) puts a marker ball at known points and compares where
  it lands in the frame with `model.project`: within 0.75 px for both scopes here.

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

The anchor's value, its spread and their source are in `anchor.json` and in the status log.

## 5. The agent's side

* `~/given/holder_manual.md`, `holder_model.py`: the device, the scopes, the set-up, the exact rules.
* `~/given/dev_procedures/`: ten development procedures recorded as video with labels, by the commissioning service
  on the sandbox's own GPU during the first minutes of the session (so the frames come from the renderer the grader
  will use; nothing is fetched or committed). The camera in them is held by a scripted, deliberately ordinary
  assistant (`recorder.Assistant`); three recordings include a lens touching tissue and its cleaning.
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
  control; no frontier model has been run. If a model saturates the task, the levers that keep it honest are in the
  generator (more looks per procedure, denser smoke, more assistant interference, longer exchanges), not penalties.
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
* The match lock's copy can be recognised by its joint readings; that is stated to the agent and is by design.
* The development set is rendered at setup time on the sandbox's GPU: it costs the first minutes of the session and
  differs in its pixels from run to run (the labels do not).

## 7. Status log

(newest first; see the entries below)

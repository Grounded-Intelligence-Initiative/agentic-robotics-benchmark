# endoscope_camera_assistance_from_video: design notes for maintainers and validators

The agent-facing contract is `instruction.md`; the device is described in
`environment/setup/payload/given/holder_manual.md` and implemented in `holder_model.py` beside it. This page records
the design decisions, what the grader checks and why, what is known to be rough, and, in section 7, a dated status
log: **what was verified, where, and what was not**. Version 0.4, 2026-11-27 (versions 0.1 of 2026-11-24 and 0.2 of
2026-11-26 were too easy; 0.3 of 2026-11-26 was made harder again at the owner's request before its frontier episode:
section 7); the Task Name was registered in the coordination sheet on 2026-11-25.

**Design rule: the agent gets what the device's engineer would have, and nothing a camera holder in an operating
room would not.** It gets the holder's kinematics and manual, the intrinsics of both scopes, a development set of
recorded procedures with an annotation team's 2D labels, and a commissioning service that plays procedures of its own
for its controller. Its
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
endoscope's RGB frame (one to three control steps old) and the four joint readings, and commands joint rates. It is scored on
the fraction of control steps with a usable view. The engineering is real end to end: perception that survives
unseen instruments and tissue, telling the surgeon's instruments from the assistant's, noticing an exchange, telling
which scope is fitted, at what angle the camera head sits on it and how late the video is from how the picture moves,
how far away an instrument is when its size is not known, noticing a smeared lens from the picture, visual servoing
through a servo with lag and a video chain with latency, and standing still at the right times. Since version 0.3
the scene does not hold still either (the tissue breathes irregularly and is dragged by the instruments), tips work
inside the tissue and in smoke, the camera head is knocked during the procedure, and some mistakes end the procedure
for the holder (the surgeon takes the camera back).

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
a non-physical surface**: geometry that moves kinematically. Until version 0.2 it moved as one piece with the
breathing (one z offset, 1.5 to 3.5 mm, 0.2 to 0.3 Hz); since version 0.3 every point moves on its own (below,
"Version 0.3"). PhysX deformable bodies need GPU dynamics, and GPU dynamics do not repeat across GPU
models, which a benchmark's score has to; the kinematic field repeats to the last bit on every machine. The
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
light, the camera's white balance and noise. The tips work at the tissue (1 to 9 mm above its resting surface, or since version 0.3 2 to 6 mm in it) and ride on its
breathing. Everything comes from a keyed BLAKE2b stream per seed, in
plain float64 arithmetic, so a seed gives the same procedure on every machine and numpy version. The fixed sets pin
kind, scope and events per procedure in a balanced table and draw the rest (`tests/review/make_sets.py`: the rule,
written before the draw; no draw rejected).

**Two pools of looks.** `generate(..., pool="dev")` draws a development procedure's looks from a narrower pool than
a graded procedure's (`pool="all"`): since version 0.3, 3 of the 11 shaft finishes (5 before), 2 of the 6 tissue
palettes (3 before), staining of the working end up to 0.3 of the way to the tissue's colour (0.5 before; up to 0.9
when graded), less clutter. The geometry, the ports, the kinds of procedure and the gestures are the same. The manual
says so (section 7) and names the three finishes; the commissioning procedures use the graded pool. This is where the
perception difficulty is meant to come from: a detector fitted to the ten development recordings meets instruments
and tissue it has not seen, as a holder sold to another hospital would. It was added after the first pixel baseline
(status log): a small net trained for two minutes on eight development recordings found 99 % of the tips in the two
held-out ones, within 4 px in the median, when development and graded looks came from one pool.

**The rig of a procedure** (version 0.2; `procedure.draw_rig`, manual sections 4 and 5). Three things that a camera
holder moved from one operating room to the next meets, that version 0.1 kept fixed and published, and that the one
frontier episode on 0.1 (section 7, 2026-11-26) took as given:

| what | rule (drawn once per procedure) | what it changes |
|---|---|---|
| the camera head's angle h on the eyepiece | uniform in -40 to +40 degrees | `R_WC = R_WS Rz(roll) Rx(-d) Rz(h)`: the picture turned about the view direction; the horizon rule and which way the picture moves for a joint motion depend on it |
| the video chain's latency | 1, 2 or 3 control steps, equally likely | how old the frame is (`Session` takes the procedure's own) |
| each instrument's shaft | 10 mm with probability 0.35, else 5 mm; a 10 mm instrument is the 5 mm geometry built twice as large about its tip | the picture of an instrument no longer gives its distance: a 10 mm one at 2z looks like a 5 mm one at z, up to lighting and its context |

Why each one is there, and why it is not a penalty or hidden information:

* *The camera head.* In a real set-up the camera head is clamped on the scope's eyepiece by hand, and the angle is
  wherever the scrub nurse left it; "the horizon is off" is the commonest camera-holding complaint. Version 0.1 had
  it at 0 and published that, so `holder_model.level_roll` levelled the picture blind, and the episode's program did
  exactly that. The angle is observable: a known joint motion moves the picture in a direction that h turns, from
  the first second of a procedure (the scope is identified the same way). The horizon rule is unchanged; it is now
  judged on the picture the surgeon sees.
* *The latency.* Video chains differ between rooms (camera control unit, routing to monitors, recording). Version
  0.1 published one step; the episode's program hard-wired it. The latency is observable from when the picture
  starts to move after a commanded motion.
* *The instruments' sizes.* 5 and 10 mm instruments are both common. Version 0.1 had every shaft 5 mm, so the
  apparent width at the tip gave the depth (`fx * 5 mm / width`); the episode's program, and the pixel baseline,
  read depth that way. A 10 mm instrument built as a scaled 5 mm one removes that shortcut without hiding anything:
  depth still follows from the holder's own motion (parallax, the change of size with insertion), from the tissue
  the tip works on, from the light, and an instrument keeps its size until it is exchanged. Rule 2 was always a
  working distance; the manual now says so, and the judging (`fx * 5 mm / depth`) is unchanged.

Everything an expert needs is on file: the ranges in the manual (section 4: `HEAD_ANGLE_MAX_DEG`,
`LATENCY_STEPS`; section 5: the sizes), the model with the head angle (`camera_pose(q, scope, head)`,
`level_roll(pan, tilt, scope, head)`), each development recording's head angle and latency in its `meta.json`
(measured "in the lab"), and every instrument's `shaft_mm` in its labels. (Until version 0.2 the commissioning
service played each development procedure with its own rig; since 0.3 it plays procedures of its own.) The rule was committed (046ea06) before any value was drawn; the values were
drawn once by `tests/review/draw_rigs.py` from a 64-bit `os.urandom` key per set (e34d77e) and are recorded in
`hidden/procedures.json` and `dev_procedures.json` with the key (`draw_rigs.py check` recomputes them). A procedure
without a rig is nominal (head 0, one step, all 5 mm), which is version 0.1.

**Version 0.3** (2026-11-26; section 7 has why). Version 0.2 was saturated by one frontier episode (reward 0.92), and
every lever of 0.2 was an identification problem with a recipe. Version 0.3's levers are meant to stay hard when an
expert knows exactly what to do: perception of instruments that hide, tracking through what hides them, a scene that
does not hold still, a rig that changes during the procedure, consequences that cannot be undone, and development
material that is what a real team has rather than the simulator's truth.

| what | rule (version 0.3) | why it is there |
|---|---|---|
| tissue that moves, not as one piece (`tissue.py`) | irregular breaths (each 2.9 to 5.3 s, 2 to 10 mm: the patient's own period U(3.4, 4.6) s and depth U(3, 8) mm, each breath x U(0.85, 1.15) and x U(0.75, 1.25)), a lift of 0.45 to 1.4 of the breath's depth growing towards the head, and a head-ward component of U(0.3, 0.7) of the lift; traction: a hand that holds the tissue (the left while it dissects and when it stretches, the right while it clips) drags it round its grip, a gaussian of U(10, 18) mm; instruments ride on the breathing | the episode's program triangulated a rigid tissue map and used it for depth and for the lens's clearance; a real abdomen is not rigid. Kinematic and deterministic: no PhysX deformables (they do not repeat across GPU models) |
| tips worked into the tissue (`procedure._Site`) | at each site a hand's tip may be 2 to 6 mm under the resting surface (right p = 0.35, left p = 0.20) | tracking through what hides the instrument: the tip still counts where it is; only the shaft shows where it enters |
| smoke that hides (`post.py`) | the plume over the energy device's tip given a dense core (weight 1.0, 45 px, 15 px above the tip, opacity to 0.97): above a density of 0.6 it hides any tip within 50 px of its centre; the events as in 0.2 (density above 0.6 on 9 to 41 % of the steps of the five smoky hidden procedures) | real smoke hides what it comes from; 0.2's smoke only tinted the picture |
| the assistant | holds 2 to 4 cm beside the work (was 3 to 5.5), reaches into the work 3 times as often (0.012 a step) | telling the surgeon's instruments from the assistant's has to work in the middle of the work |
| the camera head knocked (`procedure.draw_head_changes`) | once or twice per procedure (p = 0.5 each): first at U(8, 40) s, second at U(first + 15, 76) s; to an angle U(+-40) at least 15 degrees from the old, over 3 control steps | adapting online, not a constant: a program that identifies the rig once (as the episode's did) levels the wrong horizon after a knock |
| irreversible consequences (`episode.Episode._take_over`) | the surgeon takes the camera back, and every later step of the procedure is not usable, when the lens reaches the tissue closing faster than 15 mm/s, when it touches the tissue a second time, or when a critical action (a clip applied: the jaws closed on it; the first half of a needle bite; the first second of an energy activation) is seen usable on fewer than a quarter of its steps; critical actions in the first 10 s, in the first 3 s at a new site, and overlapping the 4 s from a knock of the camera head are not judged. A procedure taken over counts 0 (a completion gate). A smeared lens needs 10 s withdrawn (was 4 s) | the owner's request (2026-11-26): a mistake that ends the job. In 0.2 every error cost only the steps it lasted. Each is a clinical consequence with a mechanism, a line in the manual and observable precursors; nothing is subtracted, and the gate copies merged tasks' completion gates ("a run that does not count scores 0", `arm/lab_sample_handling`) |
| the manual states facts, not methods | the sentences that said how the scope, the camera head and the latency are identified (from how the picture moves when the holder does) and that a map of the tissue goes out of date are gone; what a knock does to the picture, the ranges and what announces a critical action stay | merged tasks lowered frontier scores by taking methods out of the manual and keeping the facts (`arm/keyed_nut_peg_assembly`) |
| tighter rules where perception error bites | horizon within 10 degrees (was 15); fine-work picture speed 32 px/s (was 48) | they cost the reference 0.4 and 1.4 points render-free; a narrower zoom band (26-38 px) and safe zone (0.25) were tried and rejected: they cost the reference 16 and 5 points, through the tips' depth spread and the holder's speed, not perception |
| development material without 3D truth (`episode.Episode.annotation`, `recorder.py`) | labels on every 4th step, only the tips an annotator can see (inside the disk, out of the tissue, no tissue in the line of sight, not in a dense plume), marked with an error of 1.5 px per coordinate (at most 4); role, present, active, catalogue size, lens, joints, the lab's head angle; no depth, apparent size, fine-work flag, horizon error or judgement. The development look pool narrowed to 3 finishes, 2 palettes, staining to 0.3 | the episode's program rebuilt every development instrument's 3D path from the labels' depth, scored its commissioning sessions exactly against it and labelled its own session frames from it |
| a commissioning bench of its own (`commission_procedures.json`) | the service plays ten procedures of its own (the development table, new seeds, rigs and salts, the graded look pool), nothing about them on file; a session ends at a takeover without saying why | no free judge and no free labels: what a real team has is unlabelled video from the target rooms |
| scarce practice, a shorter session | 3 commissioning sessions in all (was 12), counted when they start, never given back; the agent's session 2 h (was 4 h), the development set's recording in its first minutes included | the owner's choice of 2026-11-26, as merged tasks with low frontier scores have it (`arm/kitchen_appliance_tending`: 2 h and one trial in total; `arm/lab_sample_handling`: 2 h and a commissioning step budget). Practice on the graded distribution, even blind, is a judge an agent debugs against until its program works; with three sessions a first-attempt error has to be avoided, not fixed. Constants and text only: no draw, the reference and the grader unchanged |
| the render canary | one renderer restart and a second canary before a task error (grader and service) | a validate run lost an evaluation to a renderer that hung at the canary (`Out of resource descriptors!`) |

Kept: the scoring formula, the probe lock's rule, the kit, the action set, the hidden and development seeds and
their tables, version 0.2's rig draws (first head angle, latency, shafts). The rules were committed (`1dcff25`) before
any value existed; `tests/review/draw_v3.py` then drew, once, a 64-bit `os.urandom` key per set (`d31ae95`): hidden
12757619781714779651, development 16688242537185592512, commissioning 3379130351603837604. From the key each procedure
gets a salt that is mixed into every stream of its generator (so everything 0.3 adds or changes is drawn after its
rule) and its head changes. Nothing was redrawn.

**Version 0.4** (2026-11-26/27; the owner asked for the hardest version, after a study of merged tasks with low
frontier scores). Two levers on top of 0.3, the rules committed (`a894f00`) before the units were drawn (`89c6415`,
`tests/review/draw_v4.py`: keys hidden 13615089563888208410, development 11486367590968804959, commissioning
3929384717746489771):

| what | rule | why it is there |
|---|---|---|
| the holder is one unit of a series (`model.ZERO_TOL`, `GAIN_RANGE`, `VIEW_TOL_DEG`; `procedure.draw_unit`) | each joint's reading = its true value + a fixed zero offset (pan, tilt +-0.015 rad; insertion +-6 mm; roll +-0.07 rad); each joint moves at its clipped command x a gain in 0.8 to 1.2; the scope looks within +-2.5 degrees of its nominal direction of view; a procedure starts at the home pose its readings show; the view rules are judged on the true pose, a lens cleaned by the true insertion. Development recordings give their units' values (lab-measured); commissioning procedures run on other units; the reference is told its unit | merged tasks fell when an exact executable model became a datasheet with units that deviate (a docking task, `arm/lab_sample_handling`). The tolerances are sized so that what is judged stays knowable: the absolute pan and tilt, which the camera barely reveals, deviate little; the insertion (parallax), the roll and the view direction (the picture's motion) and the gains (the joint readings) are observable. Measured render-free: a controller with perfect perception that ignores its unit loses nothing (it servos on the picture), so this lever bites only on what an agent computes from its model: the lens's clearance and cleaning position, depth from geometry, the rotation a joint motion should give |
| critical actions in every kind of procedure (`procedure._critical_actions`) | an inspection (an exploration's hold at a place) joins the clip, the needle bite and the energy activation; the settle after the work arrives at a new place is 2 s (was 3 s), and a critical action that starts within it counts from then on if half a second of it is left (it was dropped). Every hidden procedure now has 1 to 14 critical actions (0.3: six had none) | with the per-procedure gate of 0.3, a procedure without a critical action could only be lost to the lens; now every procedure is at stake. Suture pulls and retractions were tried and left out: the reference itself was taken over (a wide retraction cannot always be held in a usable view by any holder) |

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
the current state, hands the controller the frame of `latency` steps earlier (the procedure's rig) and the current joint readings, and applies
the reply through `model.clip_command`.

**The view rules** (`Episode._judge`, manual section 6). At every step reached, from the simulator's state only:
every counting tip in the central safe zone; every counting instrument's apparent shaft diameter in 22 to 44 px;
the horizon error within 10 degrees (15 until version 0.2); during fine work (a rule on the tips' own motion, so that
it is observable: every counting tip within 9 mm of its mean over the last second) the working point's image moved by
the camera no faster than 32 px/s (48 until 0.2); the lens clean. A lens within 4 mm of the moving tissue is smeared
and the scope stops; it is clean again after 10.0 s held at insertion <= 25 mm (4.0 s until 0.2).
`useful_view_fraction` = usable steps / steps over the whole set. There is no other term, weight or penalty.

**The surgeon takes the camera back** (version 0.3; `Episode._take_over`, manual section 6). When the lens reaches the
tissue closing faster than `LENS_INJURY_SPEED` (15 mm/s: the clearance's drop over the servo tick of the contact),
when it touches the tissue a second time in a procedure, or when a critical action (`procedure._critical_actions`) is
seen with a usable view on fewer than `CRITICAL_MIN_USABLE` (a quarter) of its steps, the episode records a takeover;
the grader stops asking the controller and runs the rest of the procedure idle, every step judged `takeover`
(not usable), and the procedure counts no usable step at all, the steps before the takeover included (the gate,
`Episode.summary`: a holder that had to be taken over did not assist that procedure; `usable_before_takeover` keeps
them for the record). The controller gets the next procedure's `reset` as usual. Nothing else is subtracted. `verification.json` reports `takeovers` and `takeover_<why>` counts, and each procedure's record
its takeover step and its critical actions (`critical_min_seen`).

**What standing still costs.** A reply that is late (10 s), malformed or missing, and a used-up reply-time budget
(0.25 s x 9600 steps = 2400 s in all, probe copies included), end the controller's control of that procedure: the
remaining steps run with the holder commanded to stop, without rendering, and are judged like any other step. The
kit restarts the controller before the next procedure (three restarts). Nothing is dropped from the average.

**The match lock.** On 6 % of the steps the step is asked twice with the same `t`, the true observation and a copy
in a hidden order, and a tenth of the pairs must differ in the action (after clipping to the rate limits). The copy
is built the way an observation is (upstream #173's rule; `Session.probe_copy`): the frame the observation shows,
rendered again at the same moment from the same camera pose, with the same joint readings, and with each instrument
in the picture where it would be had it moved over that frame's last step as it moved over another step of the
procedure, drawn among the steps with the same instruments in, the same fine work and the same counted hands (a
draw that would put a tip into the tissue is drawn again; a step with no instrument in has no probe). The joints
are left alone because they are predictable: they follow from the public holder model and the controller's own
commands, so any offset in them would name the copy. What differs is what only the surgeon decides; the tips move
by 16 px in the median (2 to 78 px, 10th to 90th percentile). Both observations are possible, so a controller with
memory keeps what each request leaves and chooses when the next step's frame arrives (`~/example` shows the
pattern; the reference and the pixel baseline take the memory whose tips the new frame continues). The fraction is
a tenth because the right controller for this job stands still inside a dead zone, and a still pair does not
differ: the reference stands still on both requests of about 70 % of the pairs, and when it moves it answers the
copy differently 97 % of the time. As everywhere, the lock catches a controller that ignores its input and proves
nothing more; isolation keeps the score honest.

**The canary.** Before the controller starts, the renderer draws a marker ball at five known points for each scope
and reports where it landed; `check_canary` requires every marker found within 2.5 px of `model.project` and a frame
that is neither black nor white. A failure is a task error (`fail`), never a zero: it means the sandbox cannot
render (no RTX GPU, a host driver without its graphics libraries) or renders another camera than the published
one. Since version 0.3 a canary that fails, or a renderer that dies or hangs during it, gets one fresh renderer and a
second canary first (`render_client.Renderer.canary`, `verify.grade`; the service does the same): a validate run on
the RTX 3090 host (`validate-a8caaec5`) had lost an evaluation to a renderer that hung at its first frames with
`Out of resource descriptors!`. A renderer that dies or cannot bring the frame clock up to date is restarted and the frame drawn again (three
restarts in an evaluation), since the episode's state lives in the grader's process.

**Isolation.** Three accounts in the verifier: root (the grader, the procedures, the judging), `endosim` (the
renderer: scene and image code, the scene's current state, no future), the agent's account (the controller). Kit's
shared folders are made the renderer account's alone before it starts, its shared memory in `/dev/shm` is closed to
other accounts before the controller starts, and the renderer's wire is the pair of pipes the grader holds. The
controller receives frames and joint readings and nothing else.

## 4. The reference and the anchor

`solution/reference/controller.py`. The reference does not look at the picture: its submission carries a 256-bit
token (`hidden/oracle_token.sha256`), and for it alone every observation holds `privileged`: the scope fitted, the
camera head's angle, the video latency,
whether the lens is smeared, and each instrument's tip in the camera frame, with the frame's latency. That is all it
gets beyond the public wire. It is not told which instruments count, when the surgeon does fine work, where the
tissue is, or what comes next, and it moves the holder only by rate commands. It rebuilds the tips in the patient
frame from its own joint history, tells an exchange from the motion (a hand leaving along its shaft while the other
stays) and keeps the view where the work is until that hand is back, aims with the model's Jacobian at a working
distance set by the tips' spread, feeds the tips' velocity forward, levels the horizon in closed form, stands still
while the tips do (its own estimate of the fine-work rule), creeps under the image-speed limit when it has to move
during fine work, and withdraws to clean a smeared lens. The anchor is what perfect perception achieves under the
task's information rule; what it loses, it loses to the holder's speed on long moves, to the first seconds of each
procedure, since version 0.2 to a video latency of up to three steps (its privileged observation includes the
rig: the camera head's angle and the latency, which it would otherwise have to identify), and since version 0.3 to
the tighter horizon and steadiness rules and to re-levelling after a knock of the camera head (its privileged
observation gives the head's angle of the frame it shows, so it learns of a knock with the frame's latency). The
moving tissue and the hidden tips cost it nothing; it never comes near a takeover (status log: no takeover in any
probe draw, every critical action seen on at least 0.73 of its steps against a rule of 0.25, the lens never within
37 mm of the tissue).

The anchor's value, its spread and their source are in `anchor.json` and in the status log. Because the reference
reads the state and never the picture, its score does not depend on the rendering (`tests/review/fast_eval.py`
plays the same sessions with a null renderer in about a minute; that is how it was tuned). It depends a little on
which steps are probed: when the memory it keeps after a repeated step is the copy's (about one time in ten), its
next commands differ. Over ten draws of the probes, render-free, its mean was 0.81104 to 0.81167 (version 0.2: 0.8752
to 0.8759; version 0.1: 0.9029 to 0.9037). A submission that
reads the picture has more spread: its frames differ from run to run in their last bits.

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
reading of depth, looks the development set does not show. It was written once and never tuned in closed loop. A
second run, on `763919b` (2026-11-25), scored 0.379 (reward 0.43) with the same weights; the per-procedure values
moved both ways, which is the run-to-run spread of a controller that reads the picture. On version 0.2, unchanged (its
weights, control law and holder model of `a5acf5e`), it scores 0.074 (reward 0.09): like the frontier episode's program
it reads depth from size and takes the camera chain as published. An adapted baseline, which identifies the rig, is
in the status log (2026-11-26). It was not run on version 0.3: its depth and its labels (`apparent_px`) are gone from
the development set, and the status log's evidence for 0.3 is the frozen episode program and a fresh-agent proxy.

## 5. The agent's side

* `~/given/holder_manual.md`, `holder_model.py`: the device, the scopes, the set-up, the exact rules.
* `~/given/dev_procedures/`: ten development procedures recorded as video with labels (since 0.3 an annotation team's:
  on every fourth step, only the visible tips, hand-marked with a small error, roles, counting, catalogue sizes, lens,
  joints, the lab's head angle; no depth and no judgement of the view; each `meta.json` gives the recording's camera head
  angle, its knocks and its video latency), by the commissioning service
  on the sandbox's own GPU during the first minutes of the session (so the frames come from the renderer the grader
  will use; nothing is fetched or committed). The camera in them is held by a scripted, deliberately ordinary
  assistant (`recorder.Assistant`: its view is usable on 52 to 84 % of the steps); two recordings include a lens
  touching tissue and its cleaning. 4010 frames (2010 labelled), about 320 MB.
* The commissioning service (`environment/commission_service/commissiond.py`, account `endosim`, started by
  `setup.sh`): the grader's simulator on ten commissioning procedures of its own (since 0.3:
  `commission_procedures.json`, the development table with new seeds, rigs and salts, the graded look pool; nothing
  about them in the agent's documents but their names), 3 sessions in all since 0.3 (12 before), counted when they
  start and never given back, one at a time, the grader's observations, limits and takeovers (a session ends there,
  without a reason), no ground truth, no labels, no score, no summary.
  `~/commission/run_session.py` runs a controller folder through a session as the grader will, repeated requests
  included.
* `~/example/`: a controller that speaks the wire, keeps memory by step index and holds still.

The development and commissioning procedures were drawn like the graded ones (same generator, same rule, other
seeds); the graded ones are in neither the image nor the sandbox.

## 6. Known limitations and open items (version 0.3)

* **Version 0.3 is measured natively only.** `ale validate` passed on versions 0.1 and 0.2 on an RTX 3090 with driver
  535 and rootless Docker that limits the CPUs by affinity instead of a quota (status log). Version 0.3 changes the
  image's inputs (the service, its procedures, the manual): a new image, `ale validate` and its oracle's anchor are
  due on the validate host. Not exercised for any version: a cloud GPU model, a host with a CPU quota.
* **Difficulty is measured by proxies only for version 0.3.** The target is a frontier-model reward of roughly 0.3 or
  below. Versions 0.1 and 0.2 were saturated by one Opus 5.5 episode each (rewards 1.0 and 0.92, status log). For
  0.3 the evidence is the frozen program of the 0.2 episode and a fresh-agent proxy adapted from it on development
  material only (status log). Only a new frontier episode settles it.
* **Cliffs make scores lumpy.** A takeover leaves the rest of a procedure unusable, so a submission's per-procedure
  score depends on when a single mistake happens, and two runs of a controller that reads the picture can differ
  more than in 0.2. That is the owner's choice (2026-11-26: a mistake that ends the job). The reference stays clear of
  every cliff with a margin (status log: its lens never comes within 15 mm of the tissue, it sees every critical
  action usable on most of its steps, against a rule of a quarter), so the anchor does not depend on them.
* **The rig's identification and its changes are part of the job.** A controller that does not identify the camera
  head's angle, or does not notice a knock, fails the horizon rule (10 degrees) until it does; one that takes every
  instrument for a 5 mm one and reads depth from size holds a 10 mm one at the wrong distance. Deliberate and stated
  in the manual.
* **Numerics are not pinned.** The simulator and the reference compute with numpy's default kernels; on another CPU
  a threshold can flip on a step. The reference repeats to the last digit here; `full_at` 0.98 is the margin.
* **The controller's network filter is tested natively only.** Since version 0.2 the controller starts through
  `tests/nonet_exec.py` (`hazard_bay_build`'s launcher, byte for byte: AF_UNIX sockets only; a self-test before the
  first start, the kit's plain launcher and `network_filter` 0 in the metrics if the filter cannot be installed).
  Natively a Torch-on-CUDA program ran under the filter and was refused internet sockets (EPERM); the account switch
  and the self-test run only as root, in the engine.
* **The renderer shares the sandbox with the controller.** Its shared memory is sealed after every start, with the
  agent's processes stopped across a restart (review S1); but the controller can still take GPU memory, GPU time or
  disk that the renderer needs, and a renderer that fails four times is a task error. `VerifierDeadline` (8400 s)
  writes a verdict from the steps judged so far, with every step not reached counted unusable, so a slow host ends in
  a score rather than in no verdict.
* **No repository-level test.** The rule here was one folder per pull request, so the copy check lives in
  `tests/review/check_copies.py` instead of the repository's `tests/` (compare `tests/test_pump_skid_commissioning.py`).
* **GPU models.** `task.toml` names RTX-4090, RTX-5090 and RTX-PRO-6000 from `hazard_bay_build`'s validation of the
  same Isaac Sim stack, then RTX-3090, the one model this task has run on in the engine; it has run on none of the
  three cloud models (status log).
* **Recorded surgeon motion.** The instruments move by procedural primitives. Real recordings (JIGSAWS kinematics,
  Cholec80 / CholecT50, EndoVis, SurgT and similar) would be more convincing, but no dataset's licence has been
  checked for redistribution in a public benchmark image, so none was downloaded or shipped. Open question for the
  maintainers: is there a recorded-kinematics dataset whose licence clearly allows redistribution? If so, the
  generator's gestures could be replaced or calibrated by it without touching the wire or the rules.
* The tissue moves kinematically (version 0.3): a breathing field and gaussian traction round a grip, no mechanics;
  the instruments do not cut it, a tip worked into it simply passes under the surface (nothing dents round it), there
  is no bleeding, and smoke and a smeared lens are image-space effects, not volumes. The lens's clearance is judged to
  first order (the surface's displacement taken at the lens's own x and y), which is exact for the breathing's
  vertical part and off by the slope times the lateral shift elsewhere (millimetres at most where the lens can be).
  The instruments are built from primitives, not from CAD of real instruments. The picture is rectified: real
  laparoscopes have barrel distortion.
* **The moving mesh costs renders.** The floor's 30,000 vertices and normals are rewritten every frame; the renderer's
  stale captures rose from under 1 % to 0.4 to 0.6 extra renders a frame (the frame clock catches them, section 2), and
  a frame took 142 to 204 ms natively beside other jobs (status log). The clock is posed in the same USD change block
  as the mesh, so a frame that shows the right clock shows the right tissue as far as Hydra applies one change block at
  once; this was not checked pixel by pixel.
* The lens touches tissue only (not instruments, not the wall above); the scope's shaft collides with nothing. A
  takeover for a clash of the scope with an instrument's shaft was considered for version 0.3 and left out: the
  reference's own scope, taken as a 10 mm cylinder, overlapped an instrument's shaft by up to 7 mm on test procedures
  (render-free), so it would not stay clear with a margin.
* **The units' deviations are small where the camera cannot see them.** Absolute pan and tilt are revealed only
  through the kinematics' nonlinearity, so their zero offsets are kept to +-0.015 rad (a horizon bias of about a
  degree against a limit of 10); a controller that servos on the picture barely feels the rest (status log). The lever
  is sized to stay knowable rather than to bite.
* The commissioning service ends a session at a takeover without saying why; a surgeon would say why. The service
  stays silent on purpose (no judgement of the view), and the manual gives the exact rules.
* The instruments do not wait for the camera: a real surgeon slows down when the view lags.
* The fine-work rule is a rule on tip motion (observable), not a surgical phase label.
* The match lock's copy is a possible observation (section 3), but it is drawn, not simulated: an instrument's
  motion over the frame's last step is taken from another step of the same procedure with the same kind of work.
  A controller that tracks the instruments closely can sometimes tell which of the two observations continues the
  previous frame better (the reference does so for its memory, and picks the copy's about one time in ten); what it
  could gain by it is only to answer the copy on purpose, which the lock does not claim to prevent. The lock's
  fraction is a tenth of the pairs (section 3 has why).
* The development set is rendered at setup time on the sandbox's GPU: it costs the first minutes of the session and
  differs in its pixels from run to run (the labels do not).

## 7. Status log

Newest first. Each entry says where the numbers come from; "here" is the development server (Ubuntu 22.04,
RTX 6000 Ada, NVIDIA driver 570.195.03, Isaac Sim 6.2.0.0 + Isaac Lab 3.0.0 in a native Python 3.12 venv, no root,
no Docker).

### 2026-11-26, evening: version 0.3, made harder after the second frontier episode (trees `1dcff25` to `a647378`; measured on `d31ae95` and `a647378`)

**What the episode showed.** One Claude Opus 5.5 episode on version 0.2 (`ale run` `da1ac3b2` on `0845db99`, the RTX
3090 host, claude-code 2.1.309, effort high, no web tools, the 4 h budget) scored `useful_view_fraction` 0.7897
against the anchor 0.876: ratio 0.901, reward 0.92. It ended by itself after 1 h 56 min and used all 12 commissioning
sessions; no transport fault. The engine kept its submission (`artifacts/0000/submission`, the hashes match the
snapshot), so nothing had to be reconstructed. Its failures, against the reference's (ref1 of version 0.2): zone 13.1 %
(8.2), motion 7.7 % (2.6), zoom 6.1 % (4.0), horizon 1.3 % (0.4), lens 0. What it relied on:

* **The labels' 3D truth, three ways.** The development labels gave every tip's pixel *and depth*, so it rebuilt every
  development instrument's 3D path. The commissioning service replayed exactly those procedures, so with that path it
  scored each of its sessions exactly ("Commissioning replays recorded procedures, so a session's log + dev labels
  gives a true offline score"), built an offline simulator with an exact judge to tune its control law, and labelled
  the frames of its own sessions from the truth for training (`sessdata.py`). A real team has none of that.
* **Learned depth.** Its net regressed log-depth from the labels' depth and the shaft's width; the 5 / 10 mm question
  was a likelihood over that, a prior on the tips' height (the floor's depth from the manual) and a rigid tissue map.
* **A rigid, static scene.** The rig was identified once, from optical flow against the model over the whole
  procedure (evidence never forgotten), and a tissue map was triangulated from the camera's motion: both assume a
  scene that only breathes a few millimetres.
* **Nothing final.** Every error cost only the steps it lasted.

The lesson of round 2 holds: version 0.2's levers were constants a careful reader identifies once. Version 0.3's are
meant to stay hard when the agent knows exactly what to do (section 2, "Version 0.3", has the table): tissue that moves
non-rigidly and is dragged by the instruments, tips hidden in the tissue and in smoke, the camera head knocked during
the procedure, irreversible takeovers (the owner's request), tighter horizon and steadiness rules, and development
material without 3D truth (2D labels of visible tips only, sparser, noisier, narrower looks; a commissioning bench of
its own with nothing on file).

**Rules and draws.** The rules were committed first (`1dcff25`, 16:33), the values drawn once from fresh keys after
it (`d31ae95`, 16:35; keys in section 2). Two corrections followed before any submission was measured, each found by
looking, not by a score: the plume's dense core (`9b5460b`: version `1dcff25`'s plume, 70 px wide, blanketed the
middle of the picture for much of a dissection, and the annotator marked an assistant's tip inside it as visible) and
the second-contact rule (`a647378`: it counted touches only after a cleaning; recordings are never taken over). Neither
moved a drawn value. Sizing before the rule commit, render-free on throwaway keys (`degraded`/`tol` tools, outside the
repository): a narrower zoom band (26 to 38 px) and safe zone (0.25) cost the reference 16 and 5 points and were
left out; a takeover for a scope-shaft clash was left out because the reference's own scope overlapped a shaft;
critical actions right after a travel and during a knock of the camera head were excluded because the reference
lost them (it needs a second or two to arrive, and re-levelling after a knock moves the picture).

**The reference** (privileged; the anchor). Native on the hidden set (`run_native.py`, tree `d31ae95`, whose state and
judging are the final tree's): **0.8110417** (7786 of 9600 steps; per procedure 0.8438 0.8037 0.8888 0.6913 0.7675
0.8287 0.7262 0.8512 0.8612 0.9113 0.8213 0.7375), probe pairs 213 of 557 differing, no takeover, 45 critical
actions all seen, no lens contact, no restart, 128.7 ms a frame, 0.32 extra renders a frame, 25 min. Render-free over
ten probe draws 0.81104 to 0.81167; in every draw no takeover, the least-seen critical action usable on 0.727 of its
steps (the rule: under 0.25 ends the procedure), the lens never closer than 37.1 mm to the tissue (contact: 4 mm).
Version 0.2's reference: 0.8760. What it loses beyond 0.2: the horizon after knocks (10 degrees now) and the tighter
steadiness rule; the moving tissue and the occlusion cost it nothing (it reads the state).

**A perfect-when-visible tracker** (render-free, `degraded_hidden.py` outside the repository: the reference fed each
tip where an annotator last saw it, held in the patient frame, its first sight free): 0.689 (0.85 of the reference)
with 3 takeovers for unseen critical actions; with the camera head's change known 20 steps late 0.695. Occlusion and
the cliffs alone cost a tracker with perfect detection about 15 %; detection, depth and the knocks come on top.

**The frozen program of the version 0.2 episode** (`da1ac3b2`'s submission, unchanged; its own `holder_model.py` of
0.2), native on the hidden set (tree `a647378`, before the per-procedure gate): 0.3586; **with the gate (the
procedures taken over counting 0, recomputed from the same run's records) 0.2310** (ratio 0.285 and reward 0.29
against the new anchor; version 0.2: 0.790). Per procedure 0.359 0.423 0.118 0.394 0.560 0.336 0.426 0.091 0.434 0.365 0.263 0.536. The surgeon
took the camera back in 6 of 12 procedures, every procedure with a critical action (all `critical_unseen`; 20 critical
actions judged); no lens contact; lock 381 of 434. Failed: zone 38 %, zoom 19 %, horizon 45 %, motion 14 %, takeover
22 %. What breaks it: it fits the rig once and never forgets the evidence, so every knock of the camera head leaves the
horizon wrong for the rest of the procedure; its tracks drop a tip hidden in the tissue or the plume within a second,
and the view that loses it during a clip or an energy activation ends the procedure. As in round 2, this is a lower
bound on a fresh agent, not an estimate: a fresh agent reads that the head is knocked and that tips hide.

**A fresh-agent proxy.** Written down before the work began: a careful agent that reads the version 0.3 manual
trains a tip detector on the sparse 2D labels with colour augmentation and self-trains on its own commissioning frames;
takes depth from geometry (the tip's ray meets the working floor), not from size; keeps a hidden tip's track in the
patient frame; identifies the rig at the start and then watches for knocks (a picture that turns about its centre);
keeps the lens away from the tissue and slow near it; holds still during fine work. I adapted the version 0.2
episode's program that way (outside the repository, as a model's work), on development material only (the
development recordings and labels, and the 12 commissioning sessions run blind: `run_native.py --commission --blind`
shows no judgement, only whether a session ended early), with no truth, no `tests/` and no token:

* detector: the episode's U-net retrained on the 2010 labelled frames (heat maps of counting and other tips and the
  smear, no depth), 60 epochs with its own colour, hue, clutter and blood augmentations; then three rounds of
  self-training on 1379 of its own commissioning frames, pseudo-labelled by its confident peaks (held-out
  development recordings: 20 % of tips missed and 27 % false detections before self-training);
* depth from the ray meeting the floor at z = -0.150 m; no 5 / 10 mm question;
* the rig: the episode's flow fit, latency and scope locked after 6 s, the head angle's evidence fading (0.93 per
  informative pair); a knock detector (a rotation search on the picture against the rotation the joints predict;
  a state machine from the manual: at most two knocks, 12 s apart, a turn under 10 degrees undone). Checked on the
  development recordings, whose head angles are on file: scope right on all ten, head within 0.1 to 3.6 degrees;
* tracking: a hidden tip's track kept 6 s; all tracks unseen for 2 s -> search; a tip lost near the border -> zoom
  out; stillness ignores a hidden tip; safe-zone margin 0.75;
* the lens: back off under 45 mm from the mapped tissue, advance at most 10 mm/s under 70 mm;
* the tighter rules: hold at 20 px/s, level within 6 degrees.

Twelve blind sessions (`c01` to `c10`, some twice): five played to the end and seven ended early. Its own logs and
frames named the causes it could see (instruments lost for seconds behind stale tracks, a camera that kept moving
during a clip, false knocks), but the clip-and-cut procedure `c08` ended at the same step in all four tries and
nothing on its side said why. That is the information rule working: no judge. It was frozen after about 1 h 20 min of
work from the moment the development set was ready (its sha256 are kept with it), then run once on the hidden set:
0.2871 before the per-procedure gate; **with it 0.2278** (ratio 0.281 and reward 0.29 against the new anchor). Per procedure 0.259 0.451 0.073 0.346 0.558 0.063
0.539 0.141 0.455 0.106 0.070 0.385. The surgeon took the camera back in 6 of 12 procedures, again every procedure with
a critical action (all `critical_unseen`); no lens contact; lock 282 of 384. Failed: zone 50 %, zoom 38 %, horizon
33 %, motion 12 %, takeover 33 % (the steps after a takeover count under every rule they fail as well). It lost most
to depth: the floor prior misses the zoom band on 38 % of the steps, which the episode's program, whose net still
carries the depth head it learned from version 0.2's labels, does not (19 %).

**The small checks** (native, tree `a647378`): no submission, reward 0.0 without starting the renderer; a camera that
never moves (the example) 0.0000 on procedure 0, the lock failing (0 of 11 pairs), and the surgeon took the camera back
at its first critical action; random rates (`bad:jitter`) 0.0000 on procedure 0, lock 12 of 12; render-free, a camera
that never moves scores 0 on all twelve hidden procedures. The commissioning service on `a647378`: the development
set recorded in 8.5 min (renderer start included; 10.5 to 16 min in the earlier runs beside other jobs), then a
session on `c05` with the example through the launcher, 200 steps, "14 asked, 0 changed the action", exit 0. The
canary found 10 of 10 markers in every native run of this entry, the worst at 0.70 to 1.13 px from the model.

**What this says about difficulty.** Everything measured points well under version 0.2's 0.90 of the reference: with
the per-procedure gate the unchanged episode program reaches 0.285 of it and the fresh-agent proxy 0.281 (0.44 and
0.35 without the gate), both taken over in every procedure with a critical action.
The proxy is not an estimate of a frontier agent (round 2: the adapted baseline reached 0.70 of the reference and the
fresh episode 0.90), and a fresh agent reads from the first minute that the head is knocked, that tips hide, and what
ends a procedure. What it then has to do is execution, not a constant to read off: depth without depth labels (the
shaft's width with the catalogue size, parallax, still tips triangulated in the recordings), tracking an instrument
through the tissue, the plume and the edge of the picture, noticing a knock while the holder moves, and holding a
usable view through every clip and every energy activation, because one lapse there ends the procedure. The upper
bound with perfect detection of every visible tip and a naive hold-last tracker is 0.85 of the reference; what keeps
a non-privileged controller below that is the quality of its perception and tracking. Whether a frontier agent lands
near 0.35 or near 0.6 of the reference only a frontier episode can say.

**Scarce practice and a shorter session, the owner's choice.** After the proxy's run, the owner chose on 2026-11-26
(19:30) to cut the commissioning sessions from 12 to 3, counted when they start and never given back, and the agent's
session from 4 h to 2 h, as merged tasks with low frontier scores have it (`arm/kitchen_appliance_tending`: 2 h and
one lab trial in total; `arm/lab_sample_handling`: 2 h and a commissioning step budget). The brief of this round had
excluded both as levers; the owner lifted that. Constants and text only (`task.toml` `[agent] timeout_sec`,
`commission_procedures.json` `sessions`, the instruction, the manual's section 9): no draw changes, and the reference
and `ale validate` are untouched. The development set fits: it was complete 11.5 min into the agent's session in the
version 0.2 episode on the RTX 3090 host, and took 8.5 to 16 min natively here. The proxy above had 12 sessions and
no time limit to speak of, so it is an upper bound for the new conditions; it was not run again.

**Not measured here:** anything in the engine (no Docker here: the image must be rebuilt, `environment/` changed;
the 2 h session and the 3 sessions are exercised only by a frontier episode);
the proxy's and the episode program's run-to-run spread (each was run once on the hidden set); the moving mesh's
stale captures beyond the frame clock's check (section 6). Next on the validate host: rebuild, `ale validate` (the
anchor from its oracle), one frontier episode.

**Added last, after the proxy's run (the owner's request, from a study of merged tasks).** The per-procedure gate (a
procedure taken over counts 0) and the manual's methods taken out (facts kept). The gate changes no run: the
reference is never taken over, and the episode program's and the proxy's gated values above are recomputed from their
records. Neither was measured by a new proxy run.

**Review items.** Version 0.2's review items stay as fixed. New in this version: the canary's renderer restart (the
`validate-a8caaec5` failure), the takeover's reporting in `verification.json` (`takeovers`, `takeover_<why>`,
`critical_actions`), and the service's early end of a session at a takeover.

### 2026-11-26, morning: version 0.2 through `ale validate`, on the RTX 3090 host (tree `88705800`)

Where: the host of the 2026-11-25 evening entry below (one RTX 3090 24 GB, NVIDIA driver 535.183.01, Intel
i9-12900K, the pinned engine `bfb7505`, Docker 29.10.1 in rootless mode), with the same two deviations from an
evaluation host: the renderer's driver check off (`5faaf03`, still in both copies of `endosim/render_server.py`), and
the CPUs limited by an affinity of 8 instead of a quota. The image was rebuilt from scratch (`environment/` changed).

**Run** `validate-7485b98b` (09:03 to 09:52, the image build included), first try:

    ok    grounded-intelligence-initiative/endoscope_camera_assistance_from_video: untouched={'reward': 0.0}, oracle={'reward': 1.0}

The untouched verdict: "no submission at /home/user/submission/run.sh", verify 6 s. Setup took 129 to 133 s in both
episodes (the commissioning service's renderer up, its canary passed, the development set recording). The oracle, in
the separate verifier sandbox: the renderer up in 12 s, the canary 10 of 10 markers (at most 0.83 px off for the
0-degree scope, 1.24 px for the 30-degree one), the agent account cleared (46 processes), the controller behind the
network filter (`tests/nonet_exec.py`, version 0.2's S2: on in the engine too). `useful_view_fraction` 0.8760417, ratio
1.00005, reward 1.0; per procedure 0.8788 0.8500 0.9137 0.9163 0.7950 0.8562 0.7887 0.9325 0.8750 0.9275 0.9113 0.8675;
probe pairs 187 of 580 differing; no aborted procedure, no lens contact, no renderer or solver restart (so the
shared-memory sealing after a restart, S1, did not have to act); 121 ms a frame, 0.81 extra renders a frame (the
probe copies), the controller 11.3 s over 10,180 requests; verify 1453 s. The mean is the native run's on `582152e4`
to the last digit; single procedures are a step or so apart either way, as different probe draws make them.

*What was decided.* `tests/anchor.json` keeps 0.876, its source now this run; `full_at` stays 0.98. RTX-3090 stays in
`gpu_types`. A confirmation `ale validate` on the commit of this entry follows; its `ok` line goes into the pull
request's description, and a frontier episode on the pushed commit after it. Not exercised: a renderer restart in the
engine (the S1 path), any cloud GPU model, a host with a CPU quota.

### 2026-11-26: version 0.2, made harder after the first frontier episode (trees `046ea06` to `582152e`; measured on `582152e`)

**What the episode showed.** One Claude Opus 5.5 episode (`ale run` `624ea132` on `a5acf5e`, claude-code 2.1.309,
effort high, no web tools, the 4 h budget; on the RTX 3090 host of the validate runs) scored `useful_view_fraction`
0.883 against the reference's 0.903: ratio 0.978, reward 1.0. The agent ended by itself after 2 h 07 min. Its
session (`trajectory.json`) and its program show what was easy:

* **Perception.** It trained an ensemble of three small heat-map CNNs on the development frames and labels (left,
  right and assistant tips and their apparent size), augmented towards what the graded looks add (recoloured shafts
  and tissue, blood, clutter, smoke). The wider look pool of the graded procedures cost it little.
* **Depth from size.** It read every tip's depth as `fx * 5 mm / apparent width`: with every shaft 5 mm, the
  apparent size was a range finder.
* **The published camera chain taken as exact.** The holder model, a video latency of exactly one step, and the
  camera head aligned with the scope (`level_roll` levelled the picture with no feedback from it). Only the scope
  had to be identified, which it did from optical flow against both scopes' predictions in the first steps.
* **An offline simulator from the labels.** With the labels' tips, depths and joints and the exact public model it
  rebuilt every development instrument's 3D path, wrote its own judge of the view rules and tuned its control law
  against it, without spending commissioning sessions on tuning.
* Where it lost its 2 %: the first 15 to 20 steps of each procedure (the approach), and some lost tips.

A real camera holder moved between operating rooms has none of those three givens. That is where version 0.2 adds
difficulty: engineering a real system has to do (identify the camera chain, find depth without a known size), not
penalties or hidden information.

**The levers** (section 2, "The rig of a procedure"; manual sections 4 and 5): the camera head's angle on the
eyepiece, uniform within +-40 degrees; the video latency, 1, 2 or 3 steps; each instrument 10 mm with probability
0.35, built at twice the size. Each is stated in the manual as a datasheet would, each development recording gives its
values, and each is observable from the holder's own motion. The rule and ranges were committed in `046ea06` before
any value existed; `tests/review/draw_rigs.py` then drew each set's rigs once from a 64-bit `os.urandom` key of its
own (`e34d77e`; hidden key 10556249565426065709, development key 16933116259670986098). The hidden set got heads from
-31.4 to +37.4 degrees (3 of 12 within +-15), latency 1 on 7 procedures, 2 on 2 and 3 on 3, and a 10 mm surgeon's
instrument in 7 of its 12 procedures (the development set: in 3 of 10). Nothing was redrawn. Kept from version 0.1: the hidden and development seeds, kinds, scopes,
events and looks (the rig is drawn from its own key and adds `shaft_mm` to the looks without moving any other draw),
the scoring, the probe lock's rule, the kit and the action set.

**Measurements** (all native, the grader's loop through `tests/review/run_native.py`, on the tree of `582152e4`, whose
task code is the final one; GPU 3 of the development server, an RTX 6000 Ada shared with other users' jobs, one Isaac
process at a time; 250 to 313 ms per frame; logs outside the repository):

| what | `useful_view_fraction` | ratio / reward against 0.876 | notes |
|---|---|---|---|
| the reference (privileged; the anchor) | **0.8760417** (8410 of 9600) | 1.0 / 1.0 | per procedure 0.79 to 0.93; lock 172 of 604 pairs; no restart. Render-free 0.8761; over ten probe draws 0.8752 to 0.8759. Version 0.1: 0.9030 |
| the frozen Opus 5.5 program of run `624ea132` (unchanged) | **0.1142** | 0.130 / **0.133** | 0.0000 on nine procedures (horizon failed on 90 to 100 % of their steps: the head angle was never identified); 0.885 on procedure 9 (head +5.5 degrees, latency 1, no 10 mm instrument: the conditions of version 0.1), 0.300 on 8 (head +11.6, a 10 mm instrument: zoom 69 %), 0.185 on 5 (head -4.6, latency 3: zone 49 %, steadiness 68 %). On development procedures dev01 to dev04: 0.0000 each. Lock 541 of 614; no restart |
| the pixel baseline of version 0.1 (unchanged: `a5acf5e`'s weights, control law and model) | **0.0743** | 0.085 / **0.087** | 0.78 on procedure 9 (near-nominal rig), 0.00 to 0.08 elsewhere; version 0.1: 0.379 (reward 0.43). It reads depth from size and takes the camera chain as published, like the Opus program. Lock 235 of 583 |
| an adapted baseline (below), frozen, once | **0.6121** | 0.699 / **0.713** | per procedure 0.30 to 0.85; scope, latency and head angle identified right on all 12 (head within 0.6 degrees); the zoom band is its main loss where a 10 mm instrument is in (procedures 4, 5, 7, 10: 0.30 to 0.53). Lock 431 of 544; no restart |
| no submission | reward 0.0, no renderer started | | |
| a camera that never moves (the example) | 0.0000 render-free on all 12 (`hidden/selection.json`); natively on procedure 0: 0.0000, lock 0 of 41 pairs | | |
| random rates (`bad:jitter`), procedure 0 | 0.0000, lock 50 of 50 | | |
| the commissioning service (setup's recording, then one session, the example controller, 200 steps) | recorded the ten development procedures with their rigs in 23 min (renderer start included); the launcher printed "18 asked, 0 changed the action"; exit 0 | | |

**The adapted baseline** shows that version 0.2 can be done without privileged information, and roughly how far an
engineer gets. It was built in about four hours from a copy of the frozen Opus program (kept outside the repository,
as a model's work), on development material only (development recordings and labels, the commissioning service's
procedures through the grader's loop with `run_native.py --dev`, nothing of `tests/` beyond that loop, no token), then
frozen and run once on the hidden set. What it adds:
a joint fit of scope, latency and head angle (a fixed 2 s excitation at home, Lucas-Kanade flow on the tissue against
the public model's prediction for each of 2 x 3 x 61 hypotheses, refined to 0.25 degree); frame poses taken from the
joint history at the fitted latency; a 5 / 10 mm test per instrument (an active 18 mm insertion probe while the size
is undecided; each size predicts how the tip's picture moves and grows, held still; a sequential likelihood ratio,
decided once, reset when the instrument comes back from an exchange); a lens contact told from the joints (a
commanded motion that did not happen, as the manual says the scope stops); and the size channels of its nets
fine-tuned on the development labels (`shaft_mm`; its 10 mm widths still read 19 % low, placed as 8.1 mm). On the
development procedures it measured 0.30 to 0.78 (dev01 0.30 to 0.38, dev02 0.60, dev03 0.78, dev04 0.44 to 0.52, dev07
0.12); render-free with true detections and the true rig 0.748 (with true sizes 0.811, taking every instrument for
5 mm 0.643).

**What this says about difficulty.** The three givens the episode relied on are gone: its program, unchanged, falls
from 0.883 to 0.114 (reward 0.13), and it scores where it used to only on the one procedure whose rig happens to be
near nominal. Each lever is a stated datasheet item that an expert identifies from the holder's own motion, and the
adapted baseline does exactly that (reward 0.71, 0.70 of the reference). A frontier agent now has to notice that
the camera chain must be identified, build the identification, and handle instruments whose size it cannot read;
whether it lands nearer 0.13 or 0.7 only a frontier episode can say.

**Not measured, for lack of time:** the frozen Opus program's run-to-run spread (review S6; it was run once on the
hidden set and once on four development procedures); the adapted baseline's spread; anything in the engine (no
Docker here). The next steps on the validate host: rebuild the image (`environment/` changed), `ale validate`, the
anchor from its oracle, one frontier episode.

**Review items** (`endoscope-533631eb`, the independent pre-PR review). Fixed: S1 (the renderer's shared memory is
sealed after every start, with the agent's processes stopped by SIGSTOP across a restart; `VerifierDeadline` at
8400 s writes a verdict from the steps judged, every step not reached counted unusable); S2 (the controller starts
through `tests/nonet_exec.py`, `hazard_bay_build`'s launcher byte for byte; natively a Torch-on-CUDA program ran
under its filter and was refused internet sockets); S3 (the instruction now says that a malformed action ends the
procedure without a restart, and that the reply which uses up the budget is still applied, N2); S4 (the practice
launcher writes without blocking under one deadline per request; it times the controller only, N7); S5 (this file,
`task.toml` and the draft brought up to date; the 2026-11-24 entry below is history and its "Decisions" and "Next"
lists are superseded: the name is registered, the probe copy is a possible observation since `763919b`, the anchor
is re-measured below). N3 ("a GPU of the same models") and N5 (the timing note) are in the instruction and
`task.toml`. Not fixed, in the known limitations: S1's GPU-memory, GPU-time and disk sharing; S6 (the frozen
program's run-to-run spread is measured below only as far as time allowed); N1, N4, N6, N8, N9.

### 2026-11-25, evening: the first `ale validate`, on an RTX 3090 (trees `5faaf03`, `4f91966`)

Where: the author's second server (Ubuntu 22.04, Intel i9-12900K, one RTX 3090 24 GB, NVIDIA driver 535.183.01), the
pinned engine `bfb7505`, Docker 29.10.1 in rootless mode (the account has no root). Two things of that host differ
from an evaluation host and belong to this record:

* The driver is below Omniverse RTX's floor ("R550 Omniverse RTX driver requirement on Linux ... unsupported driver
  range [0.00, 550.90.07)"), and the renderer refused to draw. `5faaf03` turns the check off in both copies of
  `endosim/render_server.py` (`--/rtx/verifyDriverVersion/enabled=false`, a setting NVIDIA's own streaming apps
  carry); the canary still judges the frames. A smoke scene rendered with the check off matched the RTX 6000 Ada /
  driver 570 frame (rgb mean 191.95 against 192.11, the same depths), natively and in this task's image as `endosim`.
* Rootless Docker there cannot set a CPU quota (the cgroup cpu controller is not delegated to user services), and
  the engine passes `--cpus 8` to every sandbox. A wrapper in front of the docker CLI dropped `--cpus` (once per
  sandbox), and a wrapper around runc pinned every process started through `docker exec` (guestd, and with it every
  stage command) to 8 CPUs, one hyperthread of each performance core, through the OCI field `execCPUAffinity`; the
  container's own init (the base image's desktop) was not pinned. The lock records `cpus = 8` as requested; the
  limit was an affinity, not a quota. Memory (32 GB) and the GPU lease were the engine's own.

**First run** (`validate-fa5430e3`, tree `5faaf03`): untouched 0.0; the oracle a task error at the canary, "the grading
environment could not render": omni.rtx "Failed to create texture cache folder /var/lib/endocam/.cache/ov/texturecache",
then Replicator's orchestrator missing at the first capture. The cause was the task's: `runuser -u endosim` sets HOME
from the account's passwd entry, `/var/lib/endocam`, which only setup creates, in the solver's sandbox; the separate
verifier sandbox has no such folder. The native runner never met it, since it starts the renderer without runuser.
`4f91966` ends the verifier's launch prefix with `env HOME=/opt/endosim-verify/home`, as `setup.sh` does for the
commissioning service (checked first in the image: the smoke scene renders as `endosim` with HOME set).

**Second run** (`validate-3f6c0b60`, tree `4f91966`):

    ok    grounded-intelligence-initiative/endoscope_camera_assistance_from_video: untouched={'reward': 0.0}, oracle={'reward': 1.0}

The untouched verdict: "no submission at /home/user/submission/run.sh". The oracle's `verification.json`:
`useful_view_fraction` 0.9030208 (8669 of 9600 steps; per procedure 0.9387 0.8662 0.9525 0.9250 0.8775 0.9125 0.8363
0.9337 0.8725 0.9275 0.9263 0.8675), ratio 1.00002, reward 1.0; probe pairs 179 of 555 differing; no aborted
procedure, no lens contact, no renderer restart; the canary found 10 of 10 markers, at most 1.10 px off (0.75 px
natively); the renderer up in 123 s in a fresh sandbox, 122.5 ms a frame, verify 1448 s; setup 127 s (the
commissioning service's renderer up and its canary passed, the development set recording). The mean is the native
run's on `763919b` to the last digit; six procedures are one step apart either way, as different probe draws make them.

*What was decided.* `tests/anchor.json` keeps 0.903, its source now this run; `full_at` stays 0.98. RTX-3090 stays in
`gpu_types`. Not exercised: an agent phase in the engine (the commissioning sessions and the development set as the
agent reads them), `ale run`, any cloud GPU model, a host with a CPU quota.

### 2026-11-25: the probe's copy made a possible observation (tree `763919b`)

The owner chose to make the copy follow #173's rule as written (the previous entry, item 3, had left it for the
maintainers). The joints could not be the perturbed part: they follow from the public holder model and the
controller's own commands, so a controller that runs the model knows them to the last digit, and any offset names
the copy. The copy now keeps the joint readings and the camera pose and renders the frame again with the instruments
moved differently over its last step (section 3; `Session.probe_copy`, the privileged view of the moved tips through
`episode.tip_view`, which `Episode.privileged` uses too). The renderer's picture-shift path is gone. Both
observations of a repeated step are possible, so the reference, the pixel baseline and `~/example` keep the memory
each request leaves and choose when the next step arrives (the first two: the memory whose tips the new frame
continues); the instruction, the example, the commissioning service's `probe_copy` (now without arguments) and the
launcher say what a repeated request carries. `probe_pass_fraction` is 0.1 (it was 0.25).

**Render-free first** (`tests/review/fast_eval.py`, which now counts the pairs as the lock does, on the hidden set):
the reference 0.9031 with the fixed seeds of that tool (0.9034 before), 161 of 539 pairs differing (29.9 %); a still
camera 0 of 539. Over three draws of 1693 pairs in all, the pairs that did not differ were 1184 in which the
reference stood still for both requests and 14 in which both commands were clipped to the same limits; the memory
it kept after a repeated step was the copy's 173 times in 1691. The tips moved in the picture by 16 px in the median
(2.3 px at the 10th percentile, 78 px at the 90th). Over ten draws of the probes the reference's mean was 0.90292 to
0.90365 (standard deviation 0.00026) and its differing pairs 26.4 % to 33.2 %, hence the lock's fraction of a tenth.

**The native runs** (02:35 to 04:23 on 2026-11-25, one Isaac process at a time, GPU 3 only as in the previous
entry, shared with another user's job). The series started on the clean tree of `763919b`; the jitter controller's
and the pixel baseline's runs also read this entry's new `anchor.json` value (0.903; nothing else that the runs read
had changed), so their reward is against 0.903, the others' against 0.9034:

| submission | procedures | on `763919b` | on `0d27fe0` (the previous entry) |
|---|---|---|---|
| reference | 12 | **0.9030208**, reward 1.0 against the anchor of then (0.9034); probe pairs 163 of 589 differing (27.7 %); 8669 of 9600 steps; 1922 s wall, 172.3 ms a frame | 0.9034375; 567 of 567 |
| none | - | the zero verdict, reward 0 | the same |
| never moves (`~/example`) | 4 | 0.0000, reward 0; the lock fails, 0 of 172 | 0.0000; 0 of 188 |
| `bad:jitter` | 1 | 0.0000, reward 0; random rates pass the lock (53 of 53) | 0.0000; 49 of 49 |
| pixel baseline (section 4) | 12 | 0.3794, reward 0.43; pairs 213 of 530 differing; 1585 s of controller time, 3527 s wall | 0.3422, reward 0.39 (`e73ccce`); 585 of 589 (the copy then); 508 s |

The reference per procedure: 0.9387 0.8650 0.9525 0.9250 0.8762 0.9137 0.8375 0.9337 0.8738 0.9275 0.9263 0.8662
(before: 0.9387 0.8650 0.9525 0.9263 0.8775 0.9137 0.8375 0.9337 0.8750 0.9275 0.9263 0.8675); four procedures lost
one step each, where the memory after a probe was the copy's. No probe copy failed to render, no renderer was
restarted, and the canary found all ten markers in every run (at most 0.75 px off). The pixel baseline is the same
net (the same weights file) and control law; only its memory over a repeated step changed, and that changes nothing
on a step without a probe. Its per-procedure values moved both ways (0.50 0.34 0.37 0.60 0.70 0.20 0.25 0.44 0.10
0.45 0.30 0.30, against 0.25 0.27 0.40 0.46 0.62 0.32 0.06 0.67 0.04 0.52 0.20 0.30): two runs of a controller that
reads the picture differ because the frames differ in their last bits and a lost track compounds, so 0.34 and 0.38
are two draws of its spread, not an effect of the copy. Its controller time tripled on the shared GPU (the net's
inference beside another user's job), still inside the 2400 s budget.

The session launcher against the commissioning service (the service's renderer up, its canary passed, the
development set already recorded; the example controller on `dev05`, stopped after 200 steps as asked): "repeated
requests: 10 asked, 0 changed the action (the grader wants at least a tenth to, over the whole evaluation)". The
service built each copy with the new `probe_copy`, and a controller that stands still answered both requests
alike, as the grader's 0 of 172 above.

*What was decided.* `tests/anchor.json` holds 0.903, the reference's value in this run (0.9034 before); `full_at`
stays 0.98, far above the spread over probe draws.

### 2026-11-24, evening: the kit of upstream #173 (tree `67c47c8`; run natively on `0d27fe0`, the same task files)

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
   is a point for the maintainers** (superseded on 2026-11-25: the entry above). The copy is the same frame shifted in the image plane with the pan, tilt and
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

**The native runs on this tree** (`0d27fe0`: the task files of `67c47c8` and this entry; 18:42 to 19:33, one Isaac
process at a time, inside a mount namespace that shows GPU 3 only and an empty network namespace with a loopback;
other users' jobs and the battery task's cell shared that GPU). A first attempt, at 18:34, was stopped within a
minute because the GPU it was confined to had just been filled by another job. `tests/review/run_native.py` as in
the entry below; the grader's own code is unchanged since then, the kit differs in the solver's priority:

| submission | procedures | on `0d27fe0` | on `e73ccce` (the entry below) |
|---|---|---|---|
| reference | 12 | 0.9034375, reward 1.0; probes 567/567; 2001 s wall, 193.9 ms a frame | 0.9034375 three times; probes 570/570, 578/578, 534/534; 794 to 919 s, 68.7 to 74.6 ms a frame |
| none | - | the zero verdict, reward 0 | the same |
| never moves (`~/example`) | 4 | 0.0000, reward 0; lock fails, 0 of 188 probes | 0.0000; 0 of 210 |
| `bad:jitter` | 1 | 0.0000, reward 0; random rates pass the lock (49/49) | 0.0013, reward 0.00; 56/56 |

The reference's twelve per-procedure values are the earlier ones to the last digit (0.9387 0.8650 0.9525 0.9263
0.8775 0.9137 0.8375 0.9337 0.8750 0.9275 0.9263 0.8675). No renderer was restarted; the canary of every run
found all ten markers, at most 0.77 px from where `model.project` puts them. The longer wall times are the shared
GPU's (about 195 ms a frame against about 70). The controller at nice 19 beside the renderer took 19.2 s for the
whole reference run (15.2 to 15.9 s before), its slowest reply 0.06 s (at most 0.025 s before), far inside the
2400 s budget. Which steps are probed is drawn afresh, so the probe counts differ; the jitter controller's rates are
unseeded random draws, so its 0.0013 and 0.0000 are two draws of nothing.

The session launcher against the commissioning service, the last step: the series' own attempt at 19:30 failed in
the series script, not in the task (a socket and a `COMPLETE` marker left from the 05:16 test made its wait end
before the daemon listened, and the launcher's connection was refused). Repeated at 19:32 with those removed: the
daemon's renderer up in 21.6 s, canary passed (ten of ten markers, at most 0.68 px), the development set already
recorded, the session counted (two used); the example controller on `dev05`, stopped after 200 steps as asked. The
launcher printed "controller time: 0.005 s a step on average, 0.018 s at most" and "repeated requests: 12 asked, 0
changed the action (the grader wants at least a quarter to)": the lock's rule (`differ`) on a controller that never
moves, which agrees with the grader's 0 of 188 above.

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

# Open the hidden-mechanism boxes

A Franka Panda arm stands at a LIBERO kitchen tabletop. On the table, a wooden puzzle box is bolted
in front of it. The box does not open by simply lifting its lid or pulling its drawer: its opening
is held shut by a hidden mechanism of internal interlocks between movable parts on its faces, and
the parts must be operated in the right order and the right way. There are four such boxes, `i0`,
`i1`, `i2` and `i3`. Each has its own mechanism. You do not get their models: you can reach them
only through a lab with a limited number of trials, seeing them through cameras and feeling them
through the arm.

You are scored on **one graded episode** that you play yourself, through the lab, while your session
runs: one of the four boxes at a hidden placement, drawn once before your session (the same episode for every
model). You open it from camera images, joint readings and the wrist force/torque sensor, step by step, with
whatever programs you write. You have **8 hours** to prepare (exploring, lab trials, building and testing your
tools) and must start the graded episode within them; from its start you have up to **3 hours** to play it (see
The graded episode).

## What is in your home directory

| Path | What it is |
|---|---|
| `box_env.py` | The complete robot environment: the LIBERO kitchen tabletop scene, the Panda arm and gripper (robosuite models), the operational-space controller, cameras, sensor noise and the credit rule. The lab runs this exact file, for its trials and for the graded episode; only the box it loads differs. Read its docstrings: frames, units, action and observation formats, camera matrices. |
| `practice_box.json` | A practice box (`xml`: its MJCF body, `meta`: its stages, `config`: its mechanism) for `BoxEnv(json.load(open(...)))`. Its mechanism: a front push button blocks a slide bolt; the bolt blocks the lid; the lever knob is a decoy. It is an example of how such boxes are built, not one of the four boxes. |
| `practice_grader/grade.py` | Runs a controller over the controller wire against the practice box (or any box spec you pass with `--box`) on visible seeds and prints the raw progress. It applies the rules of a controller run (deadlines, repeated step indices, restarts, a time budget); none of them apply to the graded episode. |
| `labclient.py` | Client for the lab (library and command line). |
| `helpers/lab_bridge.py` | Optional boilerplate: runs your controller (`run.sh`) for one lab trial and logs every observation and action. |
| `helpers/wire_skeleton.py` | Optional boilerplate for the controller wire (READY / reset / act / close, repeated step indices). |
| `helpers/graded_bridge.py` | Optional boilerplate: runs such a controller (`run.sh`) in the graded episode, or continues the episode with it, and logs every observation and action. |

Run Python as `/opt/venv/bin/python3` (it is not on your `PATH`). `box_env.py` renders its cameras
with MuJoCo's Filament renderer (physically based) on the CPU; the first image of a process takes
several seconds while the renderer compiles its shaders. `env.gt()` gives the practice box's ground
truth.

## The boxes (what is known about all four)

- A rigid case bolted to the table, between 0.16 and 0.21 m deep, 0.27 and 0.33 m wide and 0.12 and
  0.16 m tall, front face towards the robot. Each episode places it at a fresh random pose: centre
  (0.04, 0.00) m plus a uniform offset within +-0.04 m in x and +-0.05 m in y, yaw (about z) uniform
  within +-25 degrees (yaw 0: the front faces -x). The box appearance (colours) is fixed per box.
- It opens either by its **lid** (hinged along the top back edge, with a lip protruding about 28 mm
  over the front face) or by a **front drawer** (a vertical 40 mm handle bar standing 45 mm off the
  drawer front; travel 80 mm). Exactly one of the two is the box's opening.
- Movable parts sit on the front face or on the left/right faces (never on the back or under the
  lid). Kinds, with their ranges:
  - **push button**: round cap 18 mm across, protruding 16 mm, presses 10 mm straight into the face;
  - **slide bolt**: a plate with a 15 mm pin protruding about 28 mm, slides sideways by 25 to 35 mm in
    a track that is symmetric (the track does not show which way the bolt goes). A bolt may be
    **latched**: it then slides only while its pin is held pressed at least 4 mm into the face (the
    pin goes in up to 6 mm against a return spring of about 0.6 N/mm);
  - **lever knob**: a 32 mm round base with a 60 mm lever bar (vertical at the start), turns 90
    degrees about the face normal, clockwise or counter-clockwise;
  - **cover plate**: a plate standing about 2 cm off the face on rails, slides up or sideways by
    40 to 60 mm; it can hide another part behind it;
  - the **lid** or the **drawer** (above).
- Every part moves one way only from its start position (which way is part of what you must find
  out), is damped, and is held where you leave it by friction (no springs, no detents, except the
  return spring of a latched bolt's pin). The lid is the exception: let go below about 80 degrees
  open, it falls back towards closed under its own weight (from about 80 degrees on it stays open);
  the drawer stays where you leave it. A free part needs at most about 10 N to move slowly (knob:
  a few N at the lever end).
- Interlocks are purely mechanical: hidden internal stops. A part that is still blocked moves at
  most about 3 mm (a knob about 4 degrees, the lid about 1 degree) and then stops hard; once all of
  its blockers are in their release positions it moves freely. A plain blocker releases its
  dependents once it has moved past a hidden point along its travel (somewhere between about 30 %
  and 65 % of the travel) and keeps releasing them from there to the end of its travel; at 80 % of
  its travel (knob: 72 degrees) it always releases them. A blocker with a hidden **release window**
  inside its travel (a notch in the internal stop) releases its dependents only while it sits inside
  that window, and pushing it further blocks them again. Windows lie strictly inside the travel
  (never at its start or end) and are at
  least 3 mm wide for buttons, 8 mm for bolts and cover plates and 20 degrees for knobs. There are
  no timers, no electronics and no sequences that reset: the parts' positions are the only state.
  Some parts are free from the start; some parts are decoys that move but release nothing.
- Same as in your dev sim: the robot, controller, action and observation formats, cameras,
  sensor noise, physics settings, the scene. Different: the box itself (size, colours, which parts
  it has and where, their directions, travel and friction, the interlocks) and the placement draw.

## Observation and action

Observation (a JSON object; the lab replies with the same keys):
`t` (step index), `box` (`"i0"` ... `"i3"`), `q`, `dq` (7 arm joints), `ee_pos` (gripper site,
world frame, m), `ee_quat` (w, x, y, z), `gripper` (finger opening, m), `wrench` (force N and torque
N m at the wrist sensor, world frame, noisy), `T_world_wrist` (4x4 pose of the wrist camera, OpenCV
convention), and the RGB images `left`, `right` (fixed cameras, 320 x 240) and `wrist` (256 x 192),
each `{"dtype": "uint8", "shape": [H, W, 3], "b64": <raw row-major bytes, base64>}`. An image is
empty (shape `[0, 0, 3]`) unless the previous action asked for it; the first observation of an
episode carries all three. Camera intrinsics and the fixed cameras' poses: `box_env.py`
(`fixed_camera_matrices`, `WRIST`). There is no depth image and no box state of any kind.

Action: a list of 8 numbers `[dx, dy, dz, drx, dry, drz, grip, cams]` at 10 Hz: a gripper-site
displacement (world frame, clipped to 0.03 m), a world-frame rotation vector (clipped to 0.2 rad),
the gripper command (-1 open ... +1 closed) and a camera bit mask for the next observation
(1 = left, 2 = right, 4 = wrist). An episode lasts at most 2500 steps (250 s simulated).

## The lab (your only access to the four boxes)

`labclient.py` talks to a service on `/run/ale-lab/lab.sock` (JSON lines; `python3 labclient.py
--help`). A **trial** is one episode on one box: `reset` with the box id starts it (and charges 1
trial; you have **16 trials in total**, shared by all boxes, never refunded), then `step` with an
action returns the next observation (steps are free) until the episode reaches 2500 steps. A
trial ends when its episode ends, when you start a new trial or close the connection, after 3600 s
of wall time or after 900 s without a request; a step cap of 2600 requests (rejected actions
included) applies. Each trial draws a fresh box placement and fresh sensor noise (from OS entropy);
otherwise the boxes behave deterministically. Lab episodes never end early, and the lab never
reports scores, stages or part positions: only observations. A step takes about 0.01 s plus about
0.07 s per image; one with all three images returns about 0.8 MB. When the budget is used up,
`reset` answers `budget_exhausted`. The lab boxes are exactly the boxes you are scored on. The lab
also serves the graded episode (below); lab trials stay available for the whole session.

## Stages and credit

**The stages of a box** are: every part that must be operated to open it (the parts on its interlock chain, plus
any cover plate that hides one of them) and the opening itself (lid at least 60 degrees open, or drawer at least
56 mm out). The **credit position** of a part is at least 80 % of its travel (knob: 72 degrees), or inside its
release window if it has one (within 0.02 mm of either edge; knob: 0.05 degrees); that of the opening is the lid at
least 60 degrees open or the drawer at least 56 mm out. A stage **reaches** its credit position once it has been
there for 3 consecutive steps **while every part upstream of it in the chain (the parts that block it, the parts
that block those, and so on) was in its own credit position** during those steps (chain rule: for example, a part
whose window blocker has been moved out of its window reaches nothing until the blocker is back inside, and neither
does anything downstream of it). The opening and a part without a release window count once they reach their
credit position. A part with a release window counts only once a part it blocks reaches its credit position (by the
chain rule it was then inside its window): moving it into or through its window counts nothing by itself. A counted
stage stays counted. Decoys never count. Each box has 3 or 4 part stages. The episode score is 0.5 x (fraction of
the box's part stages done) + 0.5 x (1 if the opening is done, else 0). An episode in which the simulation becomes
unstable (any arm joint faster than 8 rad/s or any box part faster than 30 units/s) ends there and scores 0.

## The graded episode (the only thing scored)

The graded episode is one episode on one of the four boxes, drawn once before your session: a box chosen
uniformly at random, with one fixed seed for its placement (the draw described in The boxes) and its sensor noise.
Every model plays this same episode. Its first observation names the box (`box`). It is an episode like a lab trial on
that box (the same box, robot, controller, action and observation formats, cameras, sensor noise and physics, the
same 2500-step limit), except that it **ends as soon as all of the box's stages are done**, and it is not one of
your 16 trials.

The calls (`labclient.py`; the requests are JSON lines on the lab socket, `{"op": "graded_start"}` and so on):

- `obs = lab.graded_start()`: starts the graded episode and returns its first observation (all three images). It
  can be started **once per session**: the start is recorded before anything is simulated and is never given back,
  whatever happens afterwards (a crash of your script, a lost connection).
- `obs, done = lab.graded_step(action)`: one control step (0.1 s simulated) with an action in the format above;
  `obs` is the next observation and `done` is true once the episode has ended. A malformed action is refused
  (`bad_action`) and simulates nothing.
- `obs, done = lab.graded_observe()`: the latest observation again, as the last step returned it (images
  included); nothing is simulated. Use it to continue from a new script.
- `lab.graded_status()`: `started`, `ended`, `end_reason`, `steps`, `sim_time_s`, `prep_left_s` (seconds left to
  start it), `graded_left_s` (seconds left in it) and `holder` (`"you"`, `"other"` or none: the connection that
  holds it).
- `lab.graded_finish()`: ends the graded episode now.

From the shell: `python3 labclient.py graded_status`, `graded_finish`, `graded_start --out first.json` (prints the
reply with each image's data replaced by its shape and saves it whole), `graded_observe --out now.json`.
`helpers/graded_bridge.py` runs a wire controller (`run.sh`) in the graded episode, as `helpers/lab_bridge.py`
does for a trial (`--resume` continues the episode where it stands; `--steps N` stops after N steps; `--finish`
ends the episode if the run stops before its end). A short example:

```python
import sys; sys.path.insert(0, "/home/user")
from labclient import LabClient

with LabClient() as lab:
    obs = lab.graded_start()                                  # once per session
    for _ in range(20):
        obs, done = lab.graded_step([0, 0, -0.01, 0, 0, 0, -1, 0])

# later, from another script: continue where the episode stands
with LabClient() as lab:
    obs, done = lab.graded_observe()
    obs, done = lab.graded_step([0, 0, 0, 0, 0, 0, -1, 7])     # all three images in the next observation
    print(lab.graded_status())
    lab.graded_finish()                                       # when you are done
```

The episode lives in the lab, not in your connection or program: closing a connection or ending a script leaves it
where it is, and any later connection continues it. Simulated time advances only with your steps, never while you
think. One connection holds the episode at a time: while one holds it, graded calls from another one are answered
`graded_busy` (`graded_status` still answers) until the first one closes. The lab serves at most two connections at
a time and closes a connection that sent nothing for 900 s. If the lab itself fails during the episode, the episode
ends there and is scored as it stood.

**Two windows.** The graded episode must be started within 8 hours of the lab's start (a few seconds before your
session starts); a later `graded_start` is refused (`window_closed`), and then the task scores 0. Once started, the
episode ends:

- at its own end: all of the box's stages done, 2500 steps, or an unstable simulation (which scores 0);
- when you call `graded_finish`;
- 3 hours after its start (the lab ends it then, with or without a request);
- when your session ends (at the latest 11 hours after it started).

After its end the lab refuses further graded steps (`graded_over`).

**Score.** Your result is the graded episode's score (Stages and credit) at the moment it ends. Nothing is
simulated after the end: counted stages stay counted, so ending it early (a finish, the 3 hours, the end of your
session) scores what is counted at that moment, as holding still until step 2500 would. If your session ends while
the episode is running, it is scored as it stood after your last step. The lab never reports the score, the stages
or the box's state, only observations. Nothing else is scored: not the submission folder, not your lab trials, not
any controller you build.

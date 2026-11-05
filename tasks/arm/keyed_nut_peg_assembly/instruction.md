# Bayonet nut on the right peg

A Franka Panda arm stands at a table with three upright pegs and one nut. Pick up the nut by its
handle, fit it onto the peg named in the work order, work it down until it sits on the peg's base
and let go. The difficulty is that each peg carries a pin and each nut has a hidden **groove** in
the wall of its hole, a small bayonet maze: the nut only comes down while the pin travels along the
groove, which means lowering it, turning it about the shaft, lowering it again, turning it again.
Grooves also have dead ends and false entrances. Nothing about a nut, seen from outside, tells you
where its groove runs. There are four nuts to be scored, `N0`, `N1`, `N2` and `N3`, each with its
own groove, mass and surface friction. You reach them only through a lab with
a limited number of trials, through cameras and through the arm's sensors.

Deliver a controller that does the job from camera images, joint readings and the wrist
force/torque sensor. You have **8 hours** of wall time for the whole job (exploring, lab trials,
building and testing the controller). Have your controller in `/home/user/submission/` well before
the time is up. Your sandbox has 4 CPU cores and 12 GB of memory (processes that exceed it are
killed), no GPU and no network; each simulation with rendering takes about 0.5 GB and one core, so
do not start many in parallel.

## What is in your home directory

| Path | What it is |
|---|---|
| `keyed_env.py` | The complete robot environment: table, Panda arm and gripper (robosuite 1.5.2 models), the operational-space controller, the pegs, the nut model, cameras, sensor noise and the credit rule. The lab and the grader run this exact file; only the nut spec they load differs. Read its docstrings: frames, units, geometry, action and observation formats, camera matrices. |
| `practice_nut.json` | A practice nut `P` (a nut spec with its groove, for `NutEnv(json.load(open(...)))`). It is not one of the four scored nuts. `env.gt()` gives the dev sim's ground truth (nut pose, its groove, the peg poses). You can write nut specs of your own to practise on. |
| `practice_grader/grade.py` | Runs your controller over the grading wire against the practice nut (or any nut spec you pass with `--nut`) on visible seeds and prints which episodes were assembled and how far each got. |
| `labclient.py` | Client for the lab (library and command line). |
| `helpers/lab_bridge.py` | Optional boilerplate: runs your controller (`run.sh`) for one lab trial and logs every observation and action. |
| `helpers/wire_skeleton.py` | Optional boilerplate for the grading wire (READY / reset / act / close, repeated step indices). |

Python 3.12 is `/opt/venv/bin/python3` (NumPy, SciPy, OpenCV (headless), Pillow, MuJoCo 3.16.0 and
the robosuite 1.5.2 asset files for the robot model); call it by that path. MuJoCo renders the
cameras offscreen on the CPU (`MUJOCO_GL=egl`, already set).

## The scene (what is known about every episode)

- World frame: metres, radians; the table top is the plane z = 0.80, the Panda base stands at
  (-0.56, 0, 0.80) facing +x. Quaternions are (w, x, y, z).
- **Pegs.** Three pegs stand on the table. Each has a 50 x 50 x 10 mm base collar and on it a round
  shaft of radius 11 mm, 65 mm tall, with one **pin**: a small block on the shaft surface, 3 mm
  wide and 4 mm tall, reaching 17.2 mm from the shaft axis, centred 37.5 mm above the collar top.
  The pegs are identical except for 1, 2 or 3 dark bands near the top of the shaft. Each episode
  draws the peg positions (centres with x in [-0.02, 0.15], y in [-0.21, 0.21], at least 0.13 m
  apart) and their yaws (uniform over the full circle, which turns the pin with the peg) anew.
- **Nut.** A ring 45 mm tall (outer radius 28 mm) whose hole (inscribed radius 12.7 mm) fits over
  the shaft, with a handle bar (16 mm wide, from 25 to 41 mm above the nut's bottom face) reaching
  98 mm from the hole axis. The ring is 24 sectors (15 degrees each, counted counter-clockwise from
  the handle) by 9 layers (5 mm each). The groove is cut into the inner part of the hole wall (out
  to 19 mm from the axis); its top layer is always solid and its entrance is on the bottom face, so
  it cannot be seen. Every nut's groove belongs to the same family, documented exactly in
  `keyed_env.py`: an entrance riser at the bottom, a first turn of 2 to 4 sectors either way, a
  riser, a second turn of 2 to 4 sectors either way and a last riser, all 2 sectors wide; turn
  channels may continue 1 to 3 sectors the other way into a dead end; and there may be up to two
  false entrances (risers as deep as the real one that lead nowhere). A channel is 30 degrees wide,
but the pin's centre has only about +-7 degrees of play in it (less when the nut sits off-centre on
the shaft, which the 1.7 mm clearance allows). Resting on the pin, the nut's
  bottom is 39.5 mm above the collar; it can come down to 24.5 mm in an entrance riser, to 9.5 mm
  after the first turn and riser, and to the collar after the second. Each episode places the nut
  flat on the table with its centre at x in [-0.20, -0.07], y in [-0.20, 0.20] and its yaw
  (direction of the handle) equal to the nut's azimuth seen from the robot base plus 135 degrees
  plus a uniform offset within +-60 degrees.
- The fingers hold the handle by friction only, so a held nut slowly sags about the grip under its
  own weight; the hand is wide enough to touch a neighbouring peg if it passes close to it.
- The nuts differ in colour (fixed per nut), mass (0.05 to 0.15 kg) and surface friction
  coefficient (0.6 to 1.0), and in their groove. Everything else (robot, controller,
  action and observation formats, cameras, sensor noise, physics settings, peg geometry, layout
  ranges) is the same as in your dev sim.
- The **work order** of an episode is the nut's id (`nut`) and the target peg's band count
  (`peg_bands`, 1 to 3), both in every observation.

## Observation and action

Observation (a JSON object; the lab replies with the same keys): `t` (step index), `nut`,
`peg_bands`, `q`, `dq` (7 arm joints), `ee_pos` (gripper site, world frame, m), `ee_quat`
(w, x, y, z), `gripper` (finger opening, m), `wrench` (force N and torque N m at the wrist sensor,
world frame, with noise and a fixed per-episode bias), `T_world_wrist` (4 x 4 pose of the wrist
camera, OpenCV convention), and the RGB images `left`, `right` (fixed cameras, 320 x 240) and
`wrist` (256 x 192), each `{"dtype": "uint8", "shape": [H, W, 3], "b64": <raw row-major bytes,
base64>}`. An image is empty (shape `[0, 0, 3]`) unless the previous action asked for it; the
first observation of an episode carries all three. Camera intrinsics and the fixed cameras' poses:
`keyed_env.py` (`fixed_camera_matrices`, `WRIST`); the principal point is at (W/2, H/2), so the
centre of pixel (u, v) is at (u + 0.5, v + 0.5). There is no depth image and no object state of
any kind.

Action: a list of 8 numbers `[dx, dy, dz, drx, dry, drz, grip, cams]` at 10 Hz: a gripper-site
displacement (world frame, clipped to 0.02 m), a world-frame rotation vector (clipped to 0.15
rad), the gripper command (-1 open ... +1 closed; the finger force is capped at 20 N) and a camera
bit mask for the next observation (1 = left, 2 = right, 4 = wrist). An episode lasts at most 1200
steps (120 s simulated).

## The lab (your only access to the four nuts)

`labclient.py` talks to a service on `/run/ale-lab/lab.sock` (JSON lines; `python3 labclient.py
--help`). A **trial** is one episode with one nut: `reset` with the nut id starts it and charges
1 trial. You have **4 trials in total**, shared by all nuts and never refunded. `step` with an
action returns the next observation (steps are free) until the episode reaches 1200 steps. Each
trial draws a fresh layout, fresh sensor noise (from OS entropy) and a target peg (`peg_bands`)
at random. A trial ends when its episode ends, when you start a new trial or close the
connection, after 3600 s of wall time or after 900 s without a request; a cap of 1300 step
requests (rejected actions included) applies. Lab episodes never end early, and the lab never
reports scores, stages or object poses: only observations. A step takes about 0.02 s plus about
0.2 s per image. When the budget is used up, `reset` answers `budget_exhausted`. The lab nuts are
exactly the nuts you are scored on. The lab is gone when grading starts.

## Deliverable and grading

Leave a controller in `/home/user/submission/` with an executable `run.sh`. The grader starts it
as `bash run.sh` with that folder as working directory, as your user, with no network (only
AF_UNIX sockets work). Everything it needs must be inside that folder (at most 5000 files and
200 MiB; a larger folder scores 0), and the controller may write there; nothing else of your home
is available when it is graded. Protocol: one JSON object per line on stdin/stdout. Keep stdout
for protocol messages and send diagnostics to stderr.

```
                                  <- READY                  (when you are loaded)
{"type": "reset"}                 -> {"ok": true}
{"type": "act", "t": 0, "obs": {...}}
                                  -> {"action": [8 numbers]}
{"type": "close"}                 -> {"ok": true}, then exit
```

Deadlines: print `READY` within 120 s of launch, and answer every request within 30 s. That 30 s
is a single budget per request, timed from when the request is sent: blank or partial lines count
against the same 30 s and do not buy more time, so emit nothing but your one JSON reply. A missed
deadline, a reply that is not one JSON object with an `action` of 8 numbers, or a crash aborts the
episode, which then scores 0.

**Every episode is graded from scratch.** Before each episode the grader stops every process of
your account, deletes every file your account owns outside the submission folder (in `/tmp`,
`/var/tmp`, `/dev/shm`, `/run`, `/usr/local`, your home), replaces the submission folder with a
fresh copy of the one you left, and launches `bash run.sh` again (it must print `READY` again).
Nothing your controller writes or learns during one graded episode is available in the next one:
whatever it should know about the nuts has to be in the submission you leave. So `run.sh` has to
work when started again: no fixed ports, PID files or locks that an earlier run leaves behind.

`t` is the step index within the episode, starting at 0 after each `reset`. A step may be queried
more than once with the same `t`; answer each request from the observation it carries. Repeated
queries do not advance the episode, so do not count requests as steps; use `t`. At about 1 % of
the steps the grader asks the same `t` twice, once with the real observation and once with a
slightly perturbed copy (joint readings, gripper and wrist-camera pose, wrench; the images are
unchanged), in a hidden order, and applies only the reply to the real one. The grader records
whether the two replies differ; this record does not change your score. Your controller may keep
state between the steps of an episode (`helpers/wire_skeleton.py` shows one way to do that safely
with repeated `t`).

The grader runs **16 episodes**, 4 with each nut, in a fixed hidden order, each with its own hidden
layout and target peg. An episode counts as **assembled** when the nut sits on the **target** peg's
collar (its bottom within 3 mm of the collar top, tilt below 5 degrees) for 10 consecutive steps
with the fingers open at least 60 mm and not touching the nut. A nut on the wrong peg, a nut left
partway down the groove or still held, or a ruined nut is not assembled. The groove walls are
solid: forcing the pin through them (its centre more than 5 degrees sideways or 1.5 mm up or down
inside a solid part of the wall, for 3 consecutive steps, while the nut is on the target peg) ruins
the nut and ends the episode. The episode ends at 1200 steps or as soon as the nut is assembled.
Your result is the **fraction of the 16 episodes assembled**. (The grader also records how far each
episode got, for diagnosis only; it does not count.) An episode in which the controller crashes,
answers late or malformed, or makes the simulation unstable (any arm joint faster than 8 rad/s or
the nut faster than 5 m/s) scores 0. The whole grading run must finish within **6000 s** of wall
time, simulation and rendering included: budget it at about 375 s per episode, that is about
0.3 s per step on average including your controller's own time. On the grader a simulation step
takes about 0.02 s and each requested image about 0.2 s. The budget includes your controller's start-up in
every episode. The episode under way when the budget runs out, and the episodes after it, score 0.
Only the final controller is scored; trial usage is not.

## Checking your work

`/opt/venv/bin/python3 /home/user/practice_grader/grade.py` runs your controller against the
practice nut in your dev sim on visible practice seeds over the same wire: same request shape,
same deadlines, a few repeated-`t` queries, a new controller process from a fresh copy of the
submission for every episode, and the same network filter. It prints
which episodes it saw assembled, how far each got, and whether the run kept within the time
budget.

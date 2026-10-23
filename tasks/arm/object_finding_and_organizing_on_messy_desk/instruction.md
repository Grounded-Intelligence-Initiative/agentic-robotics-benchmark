# Find-and-organize on a messy desk

During moving or everyday home storage, objects are often placed into boxes without
carefully tracking their locations. Retrieving a specific item later is hard: it may not be
present at all; if it is, it may be buried beneath other objects and has to be searched for;
and the search must preserve the original organization of the storage space, both while it
runs and after it ends. This task is that chore, done by a robot, continuously.

You control a Franka Panda arm at a desk in NVIDIA Isaac Sim. The desk carries four KLT
containers in a loose row (their exact positions and headings vary a little from episode to
episode), and new objects keep landing on the free desk surface around them for the whole
episode. Your job has three parts:

- place every object in the container of its class: class **A** → the **gray** container, class **B** →
  the **light-blue** container, class **C** → the **blue** container. The desk must be organized when a
  task is finished (each request closed, and the end of the episode); it may be temporarily
  disorganized while you work, but every step spent disorganized costs score;
- whenever a *request* names a specific object, find it (it may be buried under other
  objects) and place it in the **red** container before the request's deadline; if the object is
  not on the desk at all, **declare it absent** instead;
- keep going: the environment never stands still. New objects keep arriving, objects get
  taken away, and a new request can open while you are still tidying up after the last one.
  Every object's state is scored at every step, and each request is judged on its own.

Two containers start full (A objects in gray, B objects in light-blue), the red and blue containers start
empty, and a few loose objects (mostly C, some A) lie on the desk. Classes are visual
categories of everyday objects (boxes, cans and bottles, tools and kitchenware), of which at most two instances (at slightly different sizes) are on the desk at once. There is no segmentation, no object pose and no
object list in the observation: you work from the cameras.

## Episode dynamics

An episode runs a fixed `${horizon}` control steps at 10 Hz and never ends early. During it:

1. **Arrivals.** New objects drop at random free spots on the desk surface (never inside a
   container) at random intervals, within the arm's reach. When too many objects are loose on the
   desk, arrivals pause until you clear some.
2. **Requests.** Once in every 5000 steps, at a random time, a request opens (four per
   20000-step episode; each has a 1200-step deadline): the observation carries a short text
   description of one specific object (for example `the mustard bottle`). Most
   requested objects are somewhere on the desk or in a container; some are not present at all.
   You close a request yourself with a *mark* in your action: **done: delivered** or **done:
   no such object**. The grader checks the physical truth at that moment: a delivered mark
   is right only if the requested object rests in the red container; a no-such-object mark is
   right only if the object is not on the desk. A right mark closes the request as a success,
   a wrong mark as a failure, and a request whose deadline passes without a mark is a failure
   too. At most one request is open at a time.
3. **Container service.** When a container is full, or a requested object has been delivered to the red
   container, a worker empties that container: its objects disappear from the scene.
4. **Disturbances.** Occasionally the world misplaces one object into a wrong container, or nudges
   a loose object across the desk. Correcting these is part of the job.

Every event is drawn from a hidden per-episode seed; the practice environment exposes the
same dynamics on visible seeds.

## What you must deliver

Your session has a hard limit of **1 hour** of wall-clock time from the moment it starts (run
`date` now and keep track). A session that is still running at the limit is cut off and **not
graded at all**, so keep a working controller in place as you go and end the session yourself,
with at least 10 minutes to spare. Whatever is in `/home/user/submission` when you end it is
what the grader runs.

Leave a controller at `/home/user/submission/run.sh`. The grader launches it as
`bash run.sh` with the working directory `/home/user/submission`, as your own user account,
with **no network access**, and talks to it over stdin/stdout, one JSON object per line.
Call interpreters by absolute path in `run.sh`: `/opt/venv/bin/python3` is Python 3.12 with
NumPy, Pillow and SciPy (the bare `python3` on your shell's `PATH` may be a different one), and
the simulator runs under `/opt/isaac/bin/python3`.

```
READY                                                     <- your first stdout line, within 120 s
{"type": "reset"}                        -> {"ok": true}
{"type": "act", "t": 0, "obs": {...}}    -> {"action": [dx, dy, dz, droll, dpitch, dyaw, grip, mark]}
{"type": "close"}                        -> {"ok": true}, then exit
```

- `t` is the step index inside the episode, owned by the grader and restarting at 0 after
  every `reset`. **A step may be queried more than once with the same `t`**; repeated queries
  do not advance the episode. Count steps by `t`, not by requests, and answer every request
  from the observation it carries.
- Every reply is due within 30 s. A missed deadline, a crash, non-JSON output or a reply
  without `action` aborts the episode; the next `reset` restarts `run.sh` (at most 3 restarts
  per run), so your controller must survive being started again.
- An aborted episode scores 0; it is never dropped.

## Observation

`obs` is a JSON object:

| key | content |
|---|---|
| `t` | step index |
| `request` | `null`, or `{"text": "...", "deadline_t": int}` while a request is open |
| `rgb` | `{"agentview": png_b64, "eye_in_hand": png_b64, "left_view": png_b64}` — 8-bit RGB PNG, base64 |
| `depth` | same three keys — 16-bit PNG in millimetres, base64 |
| `joint_pos`, `joint_vel` | 7 arm joints, radians and rad/s |
| `gripper_width` | metres |
| `ee_pose` | `[x, y, z, qw, qx, qy, qz]` of the tool centre point in the robot base frame |

Camera intrinsics and extrinsics, the desk and container footprints and the robot base pose are fixed for
the task and documented in `/home/user/task_card.md` in the practice environment; they do not
change between seeds.

## Action

`action` is eight numbers: a Cartesian delta for the tool centre point in the robot base
frame (metres and radians, clipped to ±0.05 m and ±0.25 rad per step), a gripper command in
`[0, 1]` (0 = open, 1 = closed), and a `mark` for the open request: `0` = nothing, `1` =
**done: delivered** (you claim the requested object rests in the red container), `2` = **done: no
such object** (you claim it is not on the desk). A mark is ignored while no request is open,
and a request accepts one mark only. The environment turns the delta into joint targets
through inverse kinematics, interpolated over the step (a 5 cm step completes about 96% within it); a
delta that is unreachable or would flip the arm to another joint-space branch is refused and
the arm simply stays put for that step.

## Scoring

The grader owns the simulator and evaluates the physical state at **every** step; nothing
your controller prints is a scoring input. Three detectors feed it: finger **contact** with an
object (physics contact report), **in container** (the object's centre inside a container's
interior, in that container's own frame), and **on table** (not in any container, resting,
not touched). An object counts as *placed* only when it rests and no finger touches it.

Each object `o` has a value `v_o(t)` per step:

| state | value |
|---|---|
| placed inside a right container (its class container; for the requested object also the red container) | +1 |
| on the table, in the gripper, or in flight | 0 |
| placed inside any other container | −1 |
| knocked off the table | −1 for the rest of the episode |
| removed by container service or delivered | frozen at its value when removed |

Every object also has a baseline `b_o`: its value when it entered the scene (0 for an arrival,
+1 for an object that starts in its pile) or when the world last moved it (−1 after the world
misplaces it). Only your net effect `v_o(t) − b_o` counts.

Requests end as a **success** (`done: delivered` while the requested object is placed in the red
container; `done: no such object` when it really is absent) or a **failure** (either mark when
the truth is the opposite, or the deadline passes unmarked).

Each request earns points in two stages: the first time its target is in `held` state earns
1 point, once per request; verified delivery adds the second point for a total of 2.
Correctly declaring absence also earns 2 points. Repeated holds add nothing. A failed or expired request keeps
any first-hold credit already earned, but gains no completion credit. Merely having
touched the object before the request opened does not count.

The episode score is

```
S = 1/2 · O + 1/2 · R
O = clip( Σ_t Σ_o (v_o(t) − b_o)  /  Σ_t Σ_o max(0, 1 − b_o) , 0, 1 )
R = sum of request points / (2 · opened requests)   (0 when no request opened)
```

`R` includes the ongoing request, so its first hold earns partial credit immediately.
A release onto the table or into a container is not a loss; only falling off the table
(centre below z = −0.2 m) triggers the permanent lost value.

`O` is the share of the sorting work that was available over the episode that you actually
did, counted every step: sustained tidiness pays, a last-minute clean-up does not, and an
untouched scene scores exactly 0. An episode the grader cuts short for exhausting its
wall-clock budget keeps what it earned so far, scaled by the fraction of the horizon that ran;
an aborted episode (protocol error, step deadline, malformed action) scores 0.

The task's reported metric is `mean_episode_score`, the mean of `S` over the hidden seeds. The
reward you are judged on is that mean relative to a hidden reference value.

## Practice material

- `/home/user/messy_desk_env.py` — the same environment the grader runs, usable on your own seeds
  (stdio JSON: `reset`, `step`, `info`, `close`). Every `step` reply also carries the running
  `score` (S if the episode were cut now, the unscaled ½·O + ½·R, O, R, requests closed and won),
  `{"cmd": "score"}` adds every object's current value and baseline, and
  `{"cmd": "request", "model": "005_tomato_soup_can"}` (or `"absent": true`) opens a request on
  demand so you can rehearse marks. None of this reaches your controller's observation; the
  hidden grader's wire is the one in **What you must deliver**;
- `/home/user/practice_grader/grade.py` — drives your `run.sh` over the real wire on visible
  seeds and prints the raw score, nothing else; `--live N` prints the running score every N
  steps and `--score-file PATH` writes it as JSON for your own tooling;
- `/home/user/example_controller/` — a complete, policy-free controller (`run.sh` + one Python
  file: READY, the reply loop, camera decoding, where the action goes). Copy it to
  `/home/user/submission/` and replace the policy; as shipped it holds still and scores 0;
- `/home/user/task_card.md` — camera calibration, container geometry, class catalogue.

The simulator runs headless on the GPU in this sandbox. Start it once and keep it alive;
each launch costs about two minutes.

# Find-and-organize on a messy desk

> **Draft v0.2 (2026-10-18).** The simulator, oracle and grader for this task are being built.
> The contract below (scene, wire, observation, action, scoring) is the intended one and is
> frozen only when the task reaches *Ready for Validation*.

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
2. **Requests.** At random times a request opens: the observation carries a short text
   description of one specific object (for example `the yellow mustard bottle`). Most
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

Leave a controller at `/home/user/submission/run.sh`. The grader launches it as
`bash run.sh` with the working directory `/home/user/submission`, as your own user account,
with **no network access**, and talks to it over stdin/stdout, one JSON object per line:

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
through inverse kinematics (a 5 cm step completes within the step inside the workspace); a
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

The episode score is

```
S = 1/2 · O + 1/2 · R
O = clip( Σ_t Σ_o (v_o(t) − b_o)  /  Σ_t Σ_o max(0, 1 − b_o) , 0, 1 )
R = successful requests / closed requests        (0 when no request closed)
```

`O` is the share of the sorting work that was available over the episode that you actually
did, counted every step: sustained tidiness pays, a last-minute clean-up does not, and an
untouched scene scores exactly 0. An episode the grader cuts short for exhausting its
wall-clock budget keeps what it earned so far, scaled by the fraction of the horizon that ran;
an aborted episode (protocol error, step deadline, malformed action) scores 0.

The task's reported metric is `mean_episode_score`, the mean of `S` over the hidden seeds. The
reward you are judged on is that mean relative to a hidden reference value.

## Practice material

- `/home/user/messy_desk_env.py` — the same environment the grader runs, usable on your own seeds;
- `/home/user/practice_grader/grade.py` — drives your `run.sh` over the real wire on visible
  seeds and prints the raw score, nothing else;
- `/home/user/task_card.md` — camera calibration, container geometry, class catalogue.

The simulator runs headless on the GPU in this sandbox. Start it once and keep it alive;
each launch costs about two minutes.

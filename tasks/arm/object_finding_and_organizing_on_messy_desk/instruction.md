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
bins, and new objects keep landing on the free desk surface around them for the whole
episode. Your job has three parts:

- keep every object in the bin of its class: class **A** → the **gray** bin, class **B** →
  the **light-blue** bin, class **C** → the **blue** bin;
- whenever a *request* names a specific object, find it (it may be buried under other
  objects) and place it in the **red** bin before the request's deadline; if the object is
  not on the desk at all, **declare it absent** instead;
- leave the storage logic intact: an object you dig out while searching goes back to its own
  bin, and nothing may end up in a wrong bin.

Two bins start full (A objects in gray, B objects in light-blue), the red and blue bins start
empty, and a few loose objects (mostly C, some A) lie on the desk. Classes are visual
categories of everyday objects (boxes, cans and bottles, tools and kitchenware), rendered
with per-episode colour and scale variation. There is no segmentation, no object pose and no
object list in the observation: you work from the cameras.

## Episode dynamics

An episode runs a fixed `${horizon}` control steps at 10 Hz and never ends early. During it:

1. **Arrivals.** New objects drop at random free spots on the desk surface (never inside a
   bin) at random intervals, within the arm's reach. When too many objects are loose on the
   desk, arrivals pause until you clear some.
2. **Requests.** At random times a request opens: the observation carries a short text
   description of one specific object (for example `the yellow mustard bottle`). Most
   requested objects are somewhere on the desk or in a bin; some are not present at all.
   The request closes when the object rests in the red bin (success), when you declare it
   absent (success if it is really absent, failure otherwise), or when its deadline passes
   (failure). At most one request is open at a time.
3. **Bin service.** When a bin is full, or a requested object has been delivered to the red
   bin, a worker empties that bin: its objects disappear from the scene.
4. **Disturbances.** Occasionally the world misplaces one object into a wrong bin, or nudges
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
{"type": "act", "t": 0, "obs": {...}}    -> {"action": [dx, dy, dz, droll, dpitch, dyaw, grip, absent]}
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

Camera intrinsics and extrinsics, the desk and bin footprints and the robot base pose are fixed for
the task and documented in `/home/user/task_card.md` in the practice environment; they do not
change between seeds.

## Action

`action` is eight numbers: a Cartesian delta for the tool centre point in the robot base
frame (metres and radians, clipped to ±0.05 m and ±0.25 rad per step), a gripper command in
`[0, 1]` (0 = open, 1 = closed), and an `absent` flag: send `1` to declare that the currently
requested object is not on the desk (ignored when no request is open; `0` otherwise). Inverse
kinematics and joint limits are handled by the environment.

## Scoring

The grader owns the simulator and evaluates the physical state at **every** step; nothing
your controller prints is a scoring input. Each object `o` gets a value `v_o(t)` per step:

**Non-target objects** (every object without an open request):

| state | value |
|---|---|
| resting inside the bin of its class | +1 |
| on the desk, in the gripper, or in flight | 0 |
| resting inside any other bin | −1 |
| removed by bin service | frozen at its value when removed |

**Target object** (the one named by the open request): the same three states, but its own
bin is the **red** bin while the request is open, every step counts **double**, and the first
contact between the gripper fingers and it earns a one-time bonus. When the request closes it
is a non-target object again.

**Absent requests:** declaring the requested object absent earns a one-time bonus when it is
really absent and costs the same amount when it is not.

The episode's raw score is

```
score = max(0, Σ_o Σ_t w_o · (v_o(t) − v_o(0))) / horizon
```

The subtraction of `v_o(0)` means an untouched scene scores exactly 0: objects that start in
their own bin earn nothing until you disturb them, and cost you if you leave them elsewhere.
Sustained tidiness pays every step; a last-minute clean-up does not.

The task's reported metric is `mean_episode_score`, the mean raw score over the hidden
seeds. The reward you are judged on is that mean relative to a hidden reference value.

## Practice material

- `/home/user/messy_desk_env.py` — the same environment the grader runs, usable on your own seeds;
- `/home/user/practice_grader/grade.py` — drives your `run.sh` over the real wire on visible
  seeds and prints the raw score, nothing else;
- `/home/user/task_card.md` — camera calibration, bin geometry, class catalogue.

The simulator runs headless on the GPU in this sandbox. Start it once and keep it alive;
each launch costs about two minutes.

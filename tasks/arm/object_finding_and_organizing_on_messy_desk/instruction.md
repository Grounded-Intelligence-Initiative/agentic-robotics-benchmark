# Find-and-organize on a messy desk

During moving or everyday home storage, objects are often placed into boxes without
carefully tracking their locations. Retrieving a specific item later is hard: it may not be
present at all; if it is, it may be buried beneath other objects and has to be searched for;
and the search must preserve the original organization of the storage space, both while it
runs and after it ends. This task is that chore, done by a robot, continuously.

You control a Franka Panda arm at a desk in NVIDIA Isaac Sim. The desk carries four KLT
containers: the red delivery container right in front of the robot base and the three class
containers behind it (their exact positions and headings vary a little from episode to
episode), and new objects keep landing on the free desk surface around them for the whole
episode. Your job has three parts:

- place every object in the container of its class: class **A** → the **gray** container, class **B** →
  the **light-blue** container, class **C** → the **blue** container. The desk is judged each
  time a request closes and when the episode ends; it may be temporarily disorganized while you
  work, but it should be back in order by then;
- whenever a *request* names a specific object, find it (it may be buried under other
  objects) and place it in the **red** container before the request's deadline; if the object is
  not on the desk at all, **declare it absent** instead;
- keep going: the environment never stands still. New objects keep arriving, objects get
  taken away, and a new request can open while you are still tidying up after the last one.
  Each request is a task, find plus organize, judged when it closes: at your mark, or at its
  deadline.

Two containers start full (A objects in gray, B objects in light-blue), the red and blue containers start
empty, and a few loose objects (mostly C, some A) lie on the desk. Classes are visual
categories of everyday objects (boxes, cans and bottles, tools and kitchenware), of which at most two instances (at slightly different sizes) are on the desk at once. There is no segmentation, no object pose and no
object list in the observation: you work from the cameras.

## Episode dynamics

An episode runs a fixed `20000` control steps at 10 Hz. During it:

1. **Arrivals.** New objects drop at random free spots on the desk surface (never inside a
   container) at random intervals, within the arm's reach. When too many objects are loose on the
   desk, arrivals pause until you clear some. While a request is open, no instance of the object
   it names arrives.
2. **Requests.** Once in every 5000 steps, at a random time, a request opens (four per
   20000-step episode; each has a 1200-step deadline): the observation carries a short text
   description of one specific object (for example `the mustard bottle`). Most
   requested objects are somewhere on the desk or in a container; some are not present at all.
   You close a request yourself with a *mark* in your action: **done: delivered** or **done:
   no such object**. The episode checks the physical truth at that moment: a delivered mark
   is right only if the requested object rests in the red container; a no-such-object mark is
   right only if no such object was on the desk or in a container when the request opened
   (while it is open the world neither brings one nor takes it away). A right mark closes the
   request as a success, a wrong mark as a failure that also costs a point (see Scoring), and a
   request whose deadline passes without a mark is a failure too. At most one request is open at
   a time.
3. **Container service.** When a class container is full (nine objects or more), or a requested object
   has been delivered to the red container, a worker empties that container: its objects disappear
   from the scene. A class container that holds the open request's object is emptied only after
   the request closes.
4. **Disturbances.** Occasionally the world misplaces one object into a wrong container, or nudges
   a loose object across the desk. Correcting these is part of the job. An object that comes to
   rest beside the robot's base (closer than 0.26 m, where no camera sees it and no hand reaches)
   is set back down on a free desk spot by the world a couple of seconds later, like a nudge.

Every event is drawn from a hidden per-episode seed. Nothing in your sandbox plays an episode
except the graded episode itself: the desk lab (see **Your sandbox**) is this desk without one, so
you meet these dynamics for the first time in the graded episode, which is also the only time they
are scored.

## The graded episode

You are scored on one episode, the **graded episode**, which you play yourself during this
session. It was drawn the way the hidden episodes are, from the same hidden set, and it runs under
exactly the rules of this page: the same desk, simulator, sensors, seed-driven world, `20000`-step
horizon, request deadlines and score. It runs in a service of its own on this machine, under
another account: you cannot read it, change it or see its score, and you drive it through
`/home/user/graded_episode.py` (Python 3, standard library only):

```
import sys; sys.path.insert(0, "/home/user")
from graded_episode import GradedEpisode
ep = GradedEpisode()                 # connect (one client at a time)
ep.status()                          # {"started", "ended", "steps", "horizon", "sim_time_s",
                                     #  "prep_left_s", "graded_left_s"}
obs = ep.start()                     # ONCE per session: the observation at t = 0
while not ep.done:
    obs = ep.step(policy(obs))       # one control step: an action in, the next observation out
ep.finish()                          # optional: end the episode now
```

From a shell, `/opt/venv/bin/python3 /home/user/graded_episode.py status` prints the status
(`start` and `finish --yes` also exist).

- **One shot.** `start()` succeeds once per session. The start is charged before anything is
  simulated and never refunded. It launches the episode's own simulator, which takes from seconds
  to a few minutes; if that simulator cannot start, nothing is charged and you can try again. It
  needs about 6 GB of GPU memory, as the desk lab does: if your lab holds the GPU, close it first.
- **The world waits for you.** It advances only inside `step()`: one call is one control step
  (0.1 s of simulated time). Between your calls nothing moves and no deadline runs, so thinking
  costs no simulated time; deadlines are in steps (`deadline_t`). The episode does not stand still
  while you step, though: arrivals, requests and disturbances come as described above.
- **Persistent.** The episode lives in the service, not in your connection or your script. Close
  the connection, let a script crash, or move to the next script: a new `GradedEpisode()` continues
  where the episode stands, and `observe()` returns the observation your next step acts on (nothing
  is simulated). One client at a time: while one is connected, a second gets `busy`, so let your
  program log its own progress and close its connection (or exit) before you connect another.
- **Steps.** `step(action)` takes the eight numbers of **Action** (plus the optional ninth); a
  malformed action is refused and nothing is simulated. Each observation is exactly the one
  described in **Observation**. A step takes about 0.15-0.25 s of wall-clock time, so a full episode
  is about an hour of simulation: drive it from a program, not by hand, and run long stretches in
  the background with a log.
- **Two windows.** You must start the graded episode within **1 hour** of the start of your
  session (`prep_left_s`); a later start is refused, and the task then scores 0. Once started, the
  episode has at most **4 hours** of wall-clock time (`graded_left_s`). Your session's hard limit is
  **5 hours**. Run `date` now and keep track.
- **How it ends.** The episode ends at step `20000`, when you call `finish()`, when its 4 hours run
  out, or when your session ends, whichever comes first. Nothing is simulated after it ends. An
  episode that ends before step `20000` is scored on the state it reached, scaled by the fraction of
  the horizon that ran (see **Scoring**), so an early end costs in proportion; if your session ends
  while the episode runs, it is scored as it stood after your last step.
- **Nothing else is graded.** `/home/user/submission` is not read and no program of yours runs
  after the session: the graded episode's record, kept by the service, is the whole result.

## Observation

`obs` is a JSON object:

| key | content |
|---|---|
| `t` | step index |
| `request` | `null`, or `{"text": "...", "deadline_t": int}` while a request is open |
| `rgb` | `{"agentview": png_b64, "left_view": png_b64, "oblique": png_b64, "eye_in_hand": png_b64}` — 8-bit RGB PNG, base64; three fixed cameras (front, left, oblique front-left) and the hand camera |
| `depth` | same four keys — 16-bit PNG in millimetres, base64; a real sensor's depth (noise, 0 where a pixel has none) |
| `rgb_hires`, `depth_hires` | only in the step after an action asked for them (see Action): the same cameras at 1920 x 1080, RGB and depth PNGs, keyed by camera name |
| `joint_pos`, `joint_vel` | 7 arm joints, radians and rad/s |
| `gripper_width` | metres |
| `ee_pose` | `[x, y, z, qw, qx, qy, qz]` of the tool centre point in the robot base frame |

Camera intrinsics, the cameras' nominal poses, the desk and container footprints and the robot base pose
are fixed for the task and documented in `/home/user/task_card.md`. The cameras are real sensors: the
three fixed ones are mounted within about 1 cm and 1 degree of their nominal poses, differently on every
desk (constant through an episode; nothing tells you the error), and depth is a stereo sensor's, noisy
and with missing pixels along object outlines. The hand camera's mount and `ee_pose` are exact. The task
card gives the figures.

## Action

`action` is eight numbers, plus an optional ninth: a Cartesian delta for the tool centre point in
the robot base frame (metres and radians, clipped to ±0.05 m and ±0.25 rad per step), a gripper
command in `[0, 1]` (0 = open, 1 = closed), and a `mark` for the open request: `0` = nothing, `1` =
**done: delivered** (you claim the requested object rests in the red container), `2` = **done: no
such object** (you claim it is not on the desk). The optional ninth number is a **hi-res request**:
a bitmask of cameras (1 = agentview, 2 = left_view, 4 = eye_in_hand, 8 = oblique; 15 = all four)
whose 1920 x 1080 RGB and depth frames come back in that step's observation as `rgb_hires` /
`depth_hires`. The 320 x 320 frames always arrive at 10 Hz; a hi-res frame costs wall-clock time
inside the graded window (the simulator's PNG encoding, about 0.1 s per camera, and receiving and
decoding it, about 3 MB per camera), so you choose, step by step, between picture quality and time.
Nothing else about the step changes. A mark is ignored while no request is open, and a request accepts one mark only; the
request, and the desk with it, are judged at the step of the mark. The environment turns the delta
into joint targets through inverse kinematics, interpolated over the step (a 5 cm step completes
about 96% within it); a delta with no solution is refused and the arm simply stays put for that
step, and one whose joint solution is more than a step away is approached over the next few steps
rather than jumped to.

## Scoring

The graded episode's service owns the simulator and tracks the physical state at every step;
nothing you print or write is a scoring input. Its detectors: finger **contact** with an object (physics
contact report), **held** (a real grasp: both fingers touch the object and it has been lifted,
its centre at least 3 cm above where it last rested; a finger brushing or pushing an object does
not hold it), **in container** (the object's centre inside a container's interior, in that
container's own frame), and **on table** (not in any container, resting, not touched). An
object counts as *placed* only when it rests and no finger touches it.

Each object `o` has a value `v_o(t)` at step `t`:

| state | value |
|---|---|
| placed inside its class container | +1 |
| on the table, held, touched, or in flight | 0 |
| placed inside any other container | −1 |
| knocked off the table | −1 for the rest of the episode |
| removed by container service or delivered | frozen at its value when removed |

While a request is open, its object belongs to the **red** container only: +1 placed in red
or held, −1 placed in any other container (its class container included), 0 on the table,
touched or in flight.

Every object also has a baseline `b_o(t)`: its value when it entered the scene (0 for an
arrival, +1 for an object that starts in its pile), when the world last moved it (−1 after the
world misplaces it), or when a request named it (its value under the red-only rule at that
moment; the previous baseline returns once the request has closed). Only your net effect
`v_o(t) − b_o(t)` counts.

Requests end as a **success** (`done: delivered` while the requested object is placed in the red
container; `done: no such object` when it really is absent) or a **failure** (either mark when
the truth is the opposite, or the deadline passes unmarked).

Each request earns points in two stages: the first time its target is **held** (both fingers on
it and lifted clear of its support, see the detectors) earns 1 point, once per request; verified
delivery adds the second point for a total of 2. Correctly declaring absence also earns 2 points.
Repeated holds add nothing, and merely having held the object before the request opened does not
count. An expired request keeps any first-hold credit already earned but gains no completion
credit. A **wrong mark costs a point**: the request keeps its first-hold credit minus 1, so it ends
at 0 or −1. This closes a loophole: without the cost a mark would be free to guess (one request
in five names an absent object, so "no such object" on every request would earn points for
nothing); with it, a guessed mark loses on average. A mark is worth sending only when you are
fairly sure: "no such object" pays above one chance in three of being right, "delivered" after
the first hold is banked above one chance in two.

The episode score is

```
S = O · R
O = mean of O(t) over the steps t at which a request closes, and the episode's last step
O(t) = clip( Σ_o (v_o(t) − b_o(t))  /  Σ_o max(0, 1 − b_o(t)) , 0, 1 )
R = max(0, sum of request points / (2 · opened requests))   (0 when no request opened)
```

`R` includes the ongoing request, so its first hold earns partial credit immediately.
A release onto the table or into a container is not a loss; only falling off the table
(centre below z = −0.2 m) triggers the permanent lost value.

`O(t)` is the share of the sorting work the episode has brought so far (every object that was
ever in the scene) that is done at step `t`. A request is a task, find plus organize: it
scores fully only when its object is delivered (or its absence declared) and the desk is in
order at the mark, so an object dug out of its container and not put back, or an arrival left
loose, counts against you then. The two terms multiply: a tidy desk earns nothing while no
request has earned a point (an episode whose request points add up to 0 or less scores 0,
however well sorted the desk), and request points earn only as much as the desk is in order.
An untouched scene scores exactly 0. An episode that ends before its last step (your `finish()`,
the end of its 4-hour window, the end of your session) is scored on the state it reached, scaled by
the fraction of the horizon that ran: `S · steps / 20000`.

The task's reported metric is `mean_episode_score`, here the `S` of the graded episode (the mean
over that one episode). The reward you are judged on is that value relative to a hidden reference
value.

## Your sandbox

- `/home/user/graded_episode.py` — the graded episode's client (see **The graded episode**);
- `/home/user/messy_desk_lab.py` — the desk lab: this desk in Isaac Sim, with the same robot,
  containers, objects, cameras, physics and action rule as the graded episodes, but without the
  episode. Nothing arrives, no request opens, nothing is misplaced, nudged or emptied, and nothing
  is scored or reported about the objects: `reset` sets out a starting desk the way an episode
  starts (the containers with their jitter, the two initial piles, a few loose objects), `place`
  puts any object of the catalogue anywhere, `step` takes your action, and every reply carries the
  observation your controller will get plus the arm's own `ik_ok` status. The commands are in
  the task card; `/home/user/lab_client.py` starts the lab and wraps them. It is where you try
  your perception and your grasps; the episode itself, its requests and its scoring are the ones
  described above, and the only run through them is the graded one;
- `/home/user/example_controller/` — a policy-free controller skeleton (`run.sh` + one Python
  file): camera decoding and where the action goes. It speaks a stdin/stdout wire of its own
  (READY, reset, act, close) that the graded episode does not use; take what helps and drive the
  graded episode with `graded_episode.py`;
- `/home/user/task_card.md` — camera calibration, desk and container geometry, the hand, the
  class catalogue, the desk lab's commands.

The lab runs headless on the GPU in this sandbox. Its first launch in a fresh sandbox compiles
the renderer's shaders and takes about ten minutes; later launches take seconds. Start it early
and keep it alive while you practise. The graded episode's simulator runs on the same GPU (about
6 GB each), and its shaders were compiled before your session started.

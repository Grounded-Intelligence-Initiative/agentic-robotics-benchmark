# Arm accuracy calibration with a touch probe

Six units of one 6-axis arm, `u1` to `u6`, each stand in a cell of their own. You are the commissioning engineer.
For each unit you have **one measuring session** in its cell, through a lab service: you drive the arm with a touch
probe against the cell's twelve surveyed datum spheres and take images with the wrist camera, within a budget of
virtual time, probes and images. Then, for the unit's published acceptance conditions, you predict where the tip of
the acceptance tool is and where five points given in the camera's frame are, in the cell's datum frame. Each cell
also holds a **gauge plate**, a finished part at a pose of its own that must never be touched and that only the
camera can see; you also write two **job programs** for the unit, joint targets that put the pin of the acceptance
tool into each of the plate's eight bores: one for the arm cold, one for the arm warm, from the start temperatures
published for the unit. After its measuring session each graded unit is **serviced**: one joint's drive is replaced, which
moves that joint's zero and changes its backlash (which joint, and by how much, is not told). You then have **one
short re-qualification session** on the serviced unit, with a small budget of probes and no images (the wrist
camera is off until the unit is released), and write a third job program for the serviced unit. You leave the
predictions and the programs in one file per unit. A unit is **accepted** when both acceptance tests are inside
their limits and its three jobs, each run once when you are graded, have done every bore. Your result is the
**fraction of the six units accepted**.

You have **6 hours** of wall-clock time for the whole job. Your sandbox has 4 CPU cores, 8 GB of memory, no GPU and
no network. Python 3.12 is `/opt/venv/bin/python3` (first on `PATH`); its only packages beyond the standard library
are NumPy 2.5.4 and SciPy 1.18.1.

## Your home directory (`/home/user`)

| Path | What it is |
|---|---|
| `cell.md` | The cell's data sheet: frames, units and joint numbering, the arm's drawing, the first-order model in words and the tolerances of its parameters, the datum spheres' survey, the probes' and tools' certificates (the pin of `process_c`), the contact and collision rules, motion and timing, readings, the figures stated only as ranges and how the link temperatures change (section 7), a session's budget, the acceptance tests, the gauge plate (section 10), the cold and warm jobs (section 11), the service, the re-qualification session and the serviced job (section 12). |
| `defects.md` | The six classes of defects a unit may have beyond the first-order model: their forms and bounds. |
| `camera.md` | The wrist camera's certificate and a description of its images. |
| `arm_model.py` | The manufacturer's first-order accuracy model (NumPy): the tool tip `tcp` and the camera pose `camera_pose` as functions of its 84 parameters, the encoder readings, the joint directions, the link temperatures and the tool; `gravity_torques`; the drawing's values `nominal()`; the joint limits `Q_MIN`, `Q_MAX`. |
| `cell_data.py` | The data sheet's numbers as Python constants (survey `surveyed_spheres()`, radii, `TOOLS`, `Q_HOME`, speeds, timing, ranges, tolerances, the camera's certified figures, `BUDGET`, the acceptance limits, the plate's certificate and pose ranges with `plate_frame()` and `plate_envelope()`, the jobs' figures). |
| `submission_format.py` | The submission's format and the grader's strict reader `parse`. |
| `labclient.py` | The lab's client (standard library; NumPy for `decode_image`). |
| `conditions/` | One file per unit, `p1.json` and `u1.json` to `u6.json`: the unit's acceptance conditions and its warm job's start temperatures. |
| `submission/` | Where your predictions go. |

The cell's simulator and the units are not readable from your account. The lab answers only what a cell gives a
commissioning engineer: encoder readings, link temperatures, probe triggers, images and the session's counters.

## The conditions files

`/home/user/conditions/<unit>.json` is one JSON object:

| Key | Value |
|---|---|
| `tool` | `"process_c"`: the acceptance tool (certificate in `cell_data.TOOLS` and `cell.md`, section 5) |
| `n` | 400, the number of conditions |
| `q` | 400 rows of 6 numbers: the joints' encoder readings (rad) |
| `s` | 400 rows of 6 integers, each +1 or -1: the direction each joint last moved in |
| `temp` | 400 rows of 6 numbers: the link temperatures (°C), without reading noise |
| `cam_points` | 5 rows of 3 numbers: the camera-frame points of the camera test (m) |
| `job_warm_temp` | 6 numbers: the link temperatures (°C, link 0 to 5) at the start of the unit's warm job, each between 45 and 55 °C (`cell.md`, section 11) |

Conditions 0, 2, 4, ... are cold (every link between 20 and 22 °C), conditions 1, 3, 5, ... warm (every link between
35 and 55 °C). In every condition `process_c` is mounted.

## The submission

Leave `/home/user/submission/u1.json` to `/home/user/submission/u6.json`. Each is one JSON object with exactly five
keys:

- `"tip"`: 400 rows of 3 numbers, the predicted tip of `process_c` for each condition (m, datum frame, in the order of
  the unit's conditions file);
- `"camera"`: 400 rows of 5 rows of 3 numbers: for each condition, the predicted datum-frame position (m) of each of
  the five `cam_points`, in the order of the file;
- `"job"`: the unit's cold job program, 1 to 200 rows of 6 numbers: joint targets (rad), run in this order, each inside
  its joint's limits (`arm_model.Q_MIN`, `Q_MAX`, the limits included); `cell.md`, section 11;
- `"job_warm"`: the unit's warm job program, run from the links at the unit's `job_warm_temp`: the same rules as
  `"job"`;
- `"job_serviced"`: the unit's serviced job program, run cold on the unit after its service (`cell.md`, section 12):
  the same rules as `"job"`.

The structure, shortened (each file has all 400 rows):

```json
{"tip": [[0.5444, 0.1057, 0.4537], [-0.3537, -0.7030, 0.5015], ...],
 "camera": [[[0.0845, 0.0427, 0.1460], [0.0633, -0.0907, 0.2280], [-0.0466, 0.1311, 0.1487],
             [0.1626, -0.0861, 0.0053], [0.1720, 0.1685, 0.2001]], ...],
 "job": [[0.0, -1.5708, 1.5708, 0.0, 0.7854, 0.0], ...],
 "job_warm": [[0.0, -1.5708, 1.5708, 0.0, 0.7854, 0.0], ...],
 "job_serviced": [[0.0, -1.5708, 1.5708, 0.0, 0.7854, 0.0], ...]}
```

This writes a complete, valid file for `u1` with the drawing's values and three one-row jobs that stay at home (it is
accepted on no unit):

```python
import json, sys
import numpy as np
sys.path.insert(0, "/home/user")
import arm_model as am, cell_data as cd

c = json.load(open("/home/user/conditions/u1.json"))
q, s, temp = (np.array(c[k], float) for k in ("q", "s", "temp"))              # (400, 6) each
tool = cd.TOOLS[c["tool"]]
tip = am.tcp(am.nominal(), q, s, temp, tool)                                     # (400, 3)
R, o = am.camera_pose(am.nominal(), q, s, temp, tool)                            # (400, 3, 3), (400, 3)
cam = np.einsum("nab,kb->nka", R, np.array(c["cam_points"])) + o[:, None, :]     # (400, 5, 3)
job = [cd.Q_HOME.tolist()]                                                        # 1 to 200 rows of 6
json.dump({"tip": tip.tolist(), "camera": cam.tolist(), "job": job, "job_warm": job, "job_serviced": job},
          open("/home/user/submission/u1.json", "w"))
```

Strict rules (`submission_format.parse` is the grader's reader): the file is UTF-8 JSON without a byte-order mark,
at most 1048576 bytes (`MAX_BYTES`), a regular file (not a symbolic link); the top level an object with exactly the
keys `tip`, `camera`, `job`, `job_warm` and `job_serviced`; no key twice; every list of exactly the stated length
(`job`, `job_warm` and `job_serviced`: 1 to 200 rows of 6 each); every number a JSON integer or decimal of magnitude at most 100 (`LIMIT`), the
jobs' inside the joint limits: no booleans, no strings, no `null`, no `NaN` or `Infinity`. Other files in
`submission/` are ignored.

## Grading

Nothing you leave is executed. The grader runs in a separate sandbox that receives only `/home/user/submission/`.
For each graded unit it reads `<unit>.json` with `submission_format.parse` and compares the predictions with the true
unit on that unit's 400 conditions (`cell.md`, section 9), and runs each job program once in its own simulator of
the cell (`cell.md`, sections 11 and 12). The predictions and the cold and the warm job are graded on the unit as you
measured it, before its service; the serviced job on the unit after its service:

- **Tool-tip test**: the error of a condition is the distance between your predicted tip and the true unit's tip of
  `process_c` there. Limits over the 400 errors: 95th percentile at most **0.20 mm**, largest at most **0.40 mm**.
- **Camera test**: each of the five camera-frame points is placed in the datum frame with the true unit's camera
  pose at the condition (`process_c` mounted); the error is the distance between your predicted position and that
  placement. Limits over all 2000 errors (400 conditions × 5 points, pooled): 95th percentile at most **0.40 mm**,
  largest at most **0.60 mm**.
- **Cold job** (`job`): `process_c` mounted, from `Q_HOME`, every link at 20 °C; passed when all eight bores are done
  (the pin in the bore, its apex between 8.0 mm below the mouth and the bottom) before the pin touches anything. The
  pin's radial clearance in a bore is 0.25 mm.
- **Warm job** (`job_warm`): the same, but every link starting at the unit's `job_warm_temp`; the links cool and warm
  along the program by the unit's own figures (`cell.md`, sections 7 and 11). Its own run, with its own
  repeatability draws.
- **Serviced job** (`job_serviced`): the cold job's rules, on the unit after its service. Its own run, with its own
  repeatability draws.

Percentiles are `numpy.percentile` with its default (linear) interpolation. A unit is accepted when all four limits
hold and its three jobs passed. A unit fails if its file is missing, is not a regular file, is larger than 1048576
bytes, does not parse under the strict rules (a file without `job_warm` or `job_serviced` does not), misses any
limit, or any of its jobs does not do every bore. Units are graded
independently: one unit's failure, or any error its file provokes in the grader, does not affect the others. The result is accepted units / 6.

## The lab

The lab service listens on the unix socket `/run/ale-lab/lab.sock`. `labclient.py` speaks its protocol: one JSON
object per line each way.

```python
import sys; sys.path.insert(0, "/home/user")
from labclient import LabClient, LabError, decode_image
with LabClient() as lab:
    print(lab.hello())                                        # free
    print(lab.budget())                                       # trials remaining (free)
    obs = lab.reset("u1")                                     # costs one trial: u1's measuring session
    obs, done = lab.step({"do": "probe", "q": [...]})
    obs, done = lab.step({"do": "image"}); picture = decode_image(obs["image"])   # (960, 1280) uint8
```

`python3 /home/user/labclient.py hello` and `... budget` do the same from the shell; `... raw '<request>'` sends one
request on a connection that closes when the command exits.

**Requests** (`op`, each with exactly the keys named):

| Request | Reply (`"ok": true`, `"op"` and) |
|---|---|
| `{"op": "hello"}` | `protocol` (1), `ops` (`hello`, `budget`, `close`, `reset`, `step`), `instances` (`p1`, `u1`, `u1-serviced`, ... `u6`, `u6-serviced`; there is no `p1-serviced`), `info` = a measuring session's budget `{"time_s": 1000.0, "probes": 120, "images": 51}`, `limits` (`max_line_bytes` 1048576, `trial_step_cap` 4000, `trial_wall_s` 7200.0, `max_connections` 2) |
| `{"op": "budget"}` | `remaining`: trials left |
| `{"op": "reset", "instance": id}` | `instance`, `obs`, `remaining`. Costs one trial; opens a session on this connection, ending any session this connection had: a measuring session on the unit (`u1`), or its re-qualification session after its service (`u1-serviced`) |
| `{"op": "step", "action": command}` | `obs`, `done` (always false), `step` (this session's step requests so far) |
| `{"op": "close"}` | nothing more; ends this connection's session and the connection |

**Commands** (the `action` of a step), JSON objects with exactly these keys:

| Command | What it does |
|---|---|
| `{"do": "move", "q": [6 numbers], "speed": v}` | Unguarded joint move to q, straight in joint space, the joint that moves farthest at v rad/s (0.001 ≤ v ≤ 2.0). |
| `{"do": "probe", "q": [6 numbers]}` | Guarded joint move to q at 0.01 rad/s for the joint that moves farthest; stops when the stylus touches a sphere, a post or the table (touching the gauge plate is a collision). |
| `{"do": "tool", "name": "probe_a"}` or `"probe_b"` | Change the tool; only at the home pose `Q_HOME`; 30 s. |
| `{"do": "wait", "seconds": t}` | Let t seconds pass (0.001 ≤ t ≤ 600). |
| `{"do": "image"}` | One image from the wrist camera, the arm at rest; 2 s. |

Every q entry must be a number inside that joint's limits (`arm_model.Q_MIN`, `Q_MAX`, the limits included). The
virtual time each command takes, how a probe stops, what a move may touch and how the direction signs change:
`cell.md`, section 5.

**Observations.** Every `obs` has exactly these fields:

| Field | Meaning |
|---|---|
| `q` | the six encoder readings (rad), the joint positions rounded to multiples of 2π / 2^22 rad |
| `temp` | the six link temperatures (°C), each with reading noise of 0.05 K standard deviation |
| `tool` | the mounted tool, `probe_a` or `probe_b` |
| `time_s`, `time_left_s` | the session's virtual time used and left (s), rounded to 1 ms; `time_left_s` is negative after a command that ran past the end |
| `probes_used`, `probes_left` | probes used and left of the session's (120; 12 in a re-qualification session) |
| `images_used`, `images_left` | images used and left of the session's (51; 0 in a re-qualification session) |
| `damaged` | true once a move has touched something, or a probe the gauge plate: the probe is broken |
| `triggered` | true when this probe stopped on a trigger (a touch or a false trigger) |
| `collided` | true when this move touched something, or this probe the gauge plate |
| `image` | `{"dtype": "uint8", "shape": [960, 1280], "b64": ...}` after an image command (row-major bytes, base64; `decode_image` turns it into an array); a 1 x 1 array otherwise |
| `refused` | `no`, or why the cell refused the command: `damaged`, `time_up`, `no_probes`, `no_images`, `not_home`, `no_session_left`, `serviced` (a measuring session of a unit that has been serviced) |

A refused command does nothing and charges nothing; the rules are in `cell.md`, section 8. A session's direction
signs s are not reported. A `no_session_left` or `serviced` reply carries the values of an untouched session, not
the unit's.

**Errors** are `{"ok": false, "error": code, "message": text}` (`labclient` raises `LabError`, with `.code`); an
error reply itself costs no trial: `bad_action` (a malformed command or a q outside the joint limits; it counts
toward the session's step cap), `bad_instance`, `bad_request`, `bad_json`, `unknown_op`, `unsupported` (an `execute`
request: the lab has no such operation), `no_trial` (a
step without an open session), `trial_over` (the session reached its step or wall-clock limit), `budget_exhausted`
(no trials left), `busy` (too many connections), `rate_limited` (more than 2000 requests per second on a connection),
`line_too_long`, `too_complex`, `internal` (a lab fault; what was already charged stays charged).

**Budgets.**

- **Trials: 13 for the whole job.** Every accepted `reset`, on any unit, costs one, and none is ever given back
  (not on a disconnect, an error, a lab restart, a refused command or an unused session). The six graded units need
  twelve (a measuring and a re-qualification session each), which leaves one for the practice unit's one measuring
  session: 6 + 6 + 1 = 13.
- **Sessions per unit: one measuring session for every unit, `p1` included; one re-qualification session for
  every graded unit** (`p1` is not serviced and has none).
  A unit is charged a session when the first valid command of a session arrives (a `reset` alone, or a
  command rejected as `bad_action`, does not charge it). A second session of either kind answers every command
  with `refused = no_session_left`; the `reset` reply does not tell whether the unit has a session left. A session
  cannot be repeated or resumed.
- **The service comes between the two kinds** (`cell.md`, section 12). When the first valid command of a unit's
  re-qualification session arrives, the unit has been serviced: its measuring session is over for good. One still
  open on another connection answers every later command with `refused = serviced`, and so does a measuring session
  of that unit opened later, also one it never used (nothing charged). A unit whose measuring session ended in a
  collision still has its re-qualification session, with a new probe.
- **Within a session**: a measuring session 1000 s of virtual time, 120 probes, 51 images; a re-qualification
  session 150 s, 12 probes, 0 images. The wrist camera is off in a re-qualification session: its cable was
  unplugged for the drive's replacement, and it is reconnected and re-verified only when the unit is released
  (the camera itself is unchanged by the service, `cell.md`, section 12). An `image` command there is refused as
  `no_images` (as `damaged` or `time_up` when one of those applies first), takes no time and charges nothing; it is
  still a valid command, so as a session's first command it charges the session and the unit is serviced.
- **Limits of the service**: at most 4000 step requests per session (rejected ones included) and 7200 s of
  wall-clock time from its `reset`; the next step after either answers `trial_over` and ends the session (later
  steps answer `no_trial`). At most 2 connections at a time (a third is answered `busy` and closed); a connection
  that sends nothing for 21600 s is closed; a request line longer than 1048576 bytes is answered `line_too_long`
  and the connection closed. The lab carries out one command at a time. A session's time is virtual: the lab does
  not wait it out.

**What ends a session for good**: its connection closing (`close`, your process ending, an idle connection or an
over-long line being closed, a lab restart), a new `reset` on the same connection (also one refused as
`budget_exhausted`), the step cap, the wall-clock limit, or an `internal` error raised by the cell during a step (one raised while
encoding a reply leaves the session open). A session whose probe is
damaged or whose time is used up stays open but can measure no more; with only the probes used up a measuring
session can still take images, and with only the images used up (a re-qualification session from its start) a
session can still probe.

## The practice unit `p1`

`p1` is a unit like the graded ones, drawn the same way, in a cell like theirs, with its own conditions file
`conditions/p1.json`. It is for **rehearsing a measuring session against the lab**: like a graded unit it has
**one measuring session** (one trial), at home, cold, with a measuring session's budget. `p1` is the shop's
demonstration unit and is not due for service: it is **never serviced** and has **no re-qualification session**.
There is no instance `p1-serviced`: a `reset` naming it is answered `bad_instance`, which costs no trial. You meet
the service and the re-qualification session for the first time on a graded unit. `p1` is not graded.

## No acceptance check before grading

Acceptance is the customer's inspection (the grader) after delivery; the lab has no acceptance rig. **Nothing about
predictions or job programs is ever reported before grading, on any unit**, `p1` included. The lab has no
`execute` operation: `hello` does not list it, and a request `{"op": "execute", "instance": ..., "plan": ...}` is
answered with the error `unsupported`, which costs no trial.

## Facts about the units and the cells

All figures are in `cell.md`, `defects.md` and `camera.md`; these facts are repeated here.

- **Before calibration.** With the drawing's values (the example above), the largest tool-tip error on a unit's
  acceptance conditions is between 11 and 22 mm for nine units in ten, and the largest camera-test error between 19
  and 34 mm. The first-order parameters of a unit lie within the tolerances of `cell.md`, section 3, which include
  ±5 mm and ±0.5 degrees per axis at the arm's base and ±3 mm and ±1 degree per axis at the camera's mounting.
- **The published model is not the whole truth.** `arm_model.py` is the manufacturer's first-order model. Each unit
  follows it with its own parameter values and has, in addition, some of the six defect classes of `defects.md`;
  which ones, on which joints and how large is not told, and the practice unit's set is its own.
- **A move that touches anything ends the session's measuring.** `move` is unguarded at every speed: if the stylus
  ball touches a sphere, a post or the table anywhere along the path, the arm stops there, the probe is broken
  (`collided`, `damaged`) and the unit can be measured no more. Only `probe` stops on contact. Only the stylus ball
  can touch anything.
- **The gauge plate is never touched.** Each cell holds a gauge plate, 160 × 100 × 20 mm on three shims, with eight
  blind bores (certificate and pose ranges: `cell.md`, section 10): its top face's centre within 30 mm of
  (0.230, -0.352) m on each axis, its yaw within 15 degrees of 33 degrees, tilted by up to 2 degrees about each
  horizontal axis, its bottom face's centre 6 to 10 mm above the table; where it is in a given cell is not told. A stylus that
  touches it or a shim, in a move or in a probe (guarded or not), collides (`collided`, `damaged`): the camera is the
  only way to find it. It always stays inside the vertical cylinder of radius 0.138 m about (0.230, -0.352) m, up to
  0.035 m above the table.
- **Knocked spheres.** In a unit's cell, 0 to 4 of the twelve datum spheres stand 0.5 to 3.0 mm from their surveyed
  centres, with their posts under them. How many and which is not told.
- **False triggers.** Each probe, with a probability between 0.01 and 0.10 that is fixed for a cell and not told,
  stops before contact and reports `triggered = true` exactly as a real touch does.
- **Repeatability.** Every arrival (move, probe, tool change) leaves the tip off its kinematic position by a fresh
  random translation of 1.5e-5 to 3.0e-5 m standard deviation per axis (per cell, not told), held until the next
  arrival; images taken there are shifted by the same translation. It is not part of the model and does not enter
  the acceptance.
- **Heating.** Each link warms as its joint turns (0.06 to 0.16 K per radian, per joint and cell, and a little,
  by an amount not told, as its neighbouring joints turn) and cools towards 20 °C with a time constant of 400 to
  900 s (per cell; the law: `cell.md`, section 7); every session starts with every link at 20 °C. The replies report
  the temperatures with noise. Half of the acceptance conditions are cold (20 to 22 °C) and half warm (35 to 55 °C);
  the warm job starts at the unit's published `job_warm_temp` (45 to 55 °C).
- **Joint directions.** s_j is +1 or -1: the direction joint j last moved in. At the start of a session s = +1 for
  every joint; each `move` or `probe` sets s_j to the sign of its commanded change (target minus present position)
  for every joint commanded to change by more than 1e-4 rad, even if the motion stops early, and leaves the other
  joints' s unchanged. The lab does not report s. The conditions files give s for every condition.
- **Gravity.** The weights of the links and of the mounted tool bend the arm. `probe_a` weighs 1.0 kg, `probe_b`
  5.0 kg and the acceptance tool `process_c` 3.0 kg, each with its own centre of gravity and tip offset. The lab
  mounts only `probe_a` and `probe_b`; `process_c` is never mounted in the cell (only in the jobs, in grading).
  Tools are changed only at the home pose `cell_data.Q_HOME`, where every session starts with `probe_a` mounted.
- **The jobs** (`cell.md`, section 11). `process_c` carries a pin of diameter 5.30 mm (a cylinder with a hemispherical
  end whose apex is the tip, along the flange's +z axis; the bores are 5.80 mm). Each program starts at `Q_HOME`,
  s = +1, every link at 20 °C (the cold job) or at the unit's `job_warm_temp` (the warm job); each row is a
  joint move at 0.25 rad/s plus 0.3 s, the links warming and cooling as in a session, a fresh repeatability error
  at each row's end; along a row the changed direction signs and the repeatability error go linearly from their old
  to their new values. A bore is done when, at a row's end, the pin is in it, its apex
  between 8.0 mm below the mouth and the bore's bottom; the first contact of the pin with anything stops that job
  (the check is continuous; a pin within 0.01 mm of a surface may count as touching it). A program whose contact
  check needs more than 15000 poses fails (every 0.002 rad of a row's farthest-moving joint counts; a program that
  keeps the pin clear needs a few thousand).
- **The service** (`cell.md`, section 12). After its measuring session the drive (motor and encoder) of one of joints
  0 to 4 of each graded unit is replaced and its gear mesh set anew: that joint's zero offset (the third rpy angle of its
  frame) moves by 0.10 to 0.25 degrees, either sign, and its backlash takes a new value in the range of section 3;
  nothing else of the unit or its cell changes. Which joint, by how much and the new backlash are not told. At given
  encoder readings the stylus can then stand a few millimetres from where the unit as measured puts it. In the
  re-qualification session the wrist camera is off (reconnected only when the unit is released): that session has
  probes but no images. The practice unit `p1` is not serviced: no service, no re-qualification session.
- **The camera is uncalibrated.** Its focal length (1400 px ± 1 %), principal point (within ±8 px of the image
  centre in each axis) and radial distortion (at most about 4 px at the corners) are per unit and not told; so are
  its focus, the lighting, the surfaces' finish and each image's exposure. `camera.md` gives the camera's frame and
  projection and describes its images in words; how the grey levels are formed is not given as a formula. It sees
  the spheres, their posts, the gauge plate, the table, the floor and the room, never the arm or the tool.

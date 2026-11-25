# Process a centrifuge worklist

You are a lab-automation engineer. A Franka Panda at a lab bench has to work through a worklist of capped 15 mL
sample tubes. It sees through cameras, feels with a wrist force sensor and can weigh on a bench scale. Write the
controller that does the job. It runs closed loop on the bench and is graded on centrifuge units and benches you
have not seen. The bench is simulated, and the simulator is not in your sandbox: you have the equipment's sheets
and a commissioning window on three other centrifuges of the same series. Then you run one graded episode yourself,
live: your score is the mean of your controller's credit on the hidden benches and that episode's credit ("How you
are scored" below). The session lasts 5 hours of wall-clock time: 2 hours for the controller and your preparation,
then the graded episode's 3 hours (see "Session time" below).

**The bench.** Nothing stands in exactly the same place twice:
- the incoming rack (one row, positions A1-A10);
- the cold rack (3 x 3, A1-C3);
- the centrifuge (its cabinet, its lid, its eight-position rotor and the shield over the rotor);
- the scale (one tube holder on a balance).

Each episode places each of these within a few centimetres of a nominal place, turned by up to 10-20 deg. The
rotor's hole circle and height also vary. `task_card.md` gives the nominal places and the ranges; your cameras
have to find the real ones.

The tubes are opaque and all look alike: nothing shows how full a tube is, and a tube is known only by where it
stands. 1 mL of sample weighs 1 g.

**The job.** Each episode's worklist names:
- `cold`: two or three incoming tubes and the cold-rack position each goes to, e.g. `{"A3": "B2", "A7": "C1"}`;
- `spin`: the incoming tubes to centrifuge;
- `preloaded`: zero to two tubes already seated in the rotor from another job, by rotor position, with their
  masses, e.g. `{"3": 22.41}`. They are spun too.

Load the rotor so that the spun tubes and the preloaded ones stand in opposite pairs, each pair's two masses
within 1 g. Positions k and k + 4 are opposite; the positions are numbered 1-8 counter-clockwise from the one
marked with a green dot. Such a pairing always exists, but the masses are known only by weighing. Which pair goes
to which positions is up to you, except that the preloaded tubes stay where they are. A fixed shield covers the
rotor's holes except at one loading port, so a tube goes in only at the hole the rotor has brought to the port.
Then close the lid: the job is the centrifuge run, and a rotor under an open lid cannot be started.

Some positions are not yours to touch:
- other people's tubes in the cold rack;
- incoming tubes on neither list;
- the preloaded tubes.

Leave all of these where they are. The lid may start open or shut.

**What is hidden per unit.** The rotor turns on its hub one way only: a ratchet lets it turn counter-clockwise
(seen from above) and holds it against the other way. You turn it by hand, by gripping the knob bar on its hub
inside the shield and moving it along its arc. A hole carried past the port comes back only after a full turn.
How the rotor answers a turn is a property of each centrifuge unit, and no unit's values are told:

| Property | Range over the series |
|---|---|
| Friction of the rotor's hinge | 0.10-0.45 N m |
| Play between the knob and the rotor | 0-20 deg |
| Rotor inertia | 0.010-0.040 kg m^2 |
| Radial clearance of the rotor's bores around a tube | 0.6-1.0 mm |
| Friction of the bores against a tube (coefficient) | 0.35-0.58 |
| Friction of the lid's hinge | 0.30-1.20 N m |
| Offset of each camera's pose on file (`T_world_<cam>`) from the equipment sheet's nominal pose | wrist: up to 4 mm and 1.5 deg per axis; fixed cameras: up to 20 mm and 2 deg per axis |
| Error of each camera's pose on file against the camera's true pose | wrist: up to 2 mm and 0.5 deg per axis; fixed cameras: up to 5 mm and 0.5 deg per axis |

A grip commanded to turn 45 deg may turn the rotor by much less, and a short grip may not turn it at all. In a
tight bore a tube that goes in tilted against the bore's axis, by as little as a degree, stops short of the floor. A stiff lid stays where it is
let go. No camera stands
where the equipment sheet's nominal pose puts it, and a point located through a camera's pose on file is still
millimetres off (up to about 2 cm from a fixed camera): the offset from the nominal pose is the unit's mounting and
does not tell the error. The six scored units and the
three commissioning units were drawn separately over these ranges: a commissioning unit shows what the series is
like, not what a scored unit's values are. Nothing measured on a commissioning bench is a scored unit's value:
your controller has to find out on site, in every episode, where things are, whether a tube is seated and
whether the lid is shut (`task_card.md` says what each sensor shows of it).

## How you are scored: your controller, and one episode you run live

The score is the mean of two items, each relative to its own hidden reference value:

1. **Your controller**: the mean credit of the controller in `/home/user/submission/` over the 12 hidden episodes
   (what you deliver and how it is graded: "Your controller and its grading" below).
2. **The graded episode**: one of those 12 episodes, which you run yourself, live, during this session, through a
   service in your sandbox, with the same bench, rules and credit ("The graded episode" below).

**The freeze.** The controller graded is the one in the submission folder at the earlier of two moments: when you
start the graded episode, or when the 2-hour start window below closes without a start. At that moment (before the
graded episode begins, if you start it) the graded episode's service copies the folder out of your reach (regular
files and folders of your account only, symbolic links skipped; over 5000 files or 200 MiB nothing is copied and the
controller scores 0, as a larger folder does at grading), and nothing you change in the folder afterwards reaches
the controller's grading: finish the controller before you start the graded episode. The folder as it is when your
session ends is graded only if the session ends before either moment. Without a start the graded episode scores 0.

**The commissioning service** (next section) closes when the graded episode starts.

**The windows.** You have 2 hours from the graded episode's service's start (it starts with your sandbox, moments
before your session begins) to write the controller and to prepare, commissioning included; start the graded
episode within them (later, `start()` answers `prep_window_closed`, and the controller is frozen as it is when they
close). The graded episode then has at most 3 hours. The session lasts 5 hours in all.

## Commissioning

There is no simulator of the bench in your sandbox, and no model of it other than what `task_card.md` states. What
you can do before you hand in is what an integrator can do before a method goes live: work on a spare bench for a
limited time.

- **The benches.** A commissioning service holds three development benches, units `d0`, `d1` and `d2`: other
  centrifuges of the same series (above), none of them a scored unit, with worklists of their own (none of them a
  scored episode's). It runs under another account, and its files are closed to yours. You reach it through its
  socket only, `/run/labbench/commission.sock`; `commission/client.py` is a client for it.
- **What it gives.** The observation a controller gets when it is graded (below), and nothing else: no pose of
  anything, no mass, no fill, no rotor angle, no unit property, no credit, and no word on how a worklist went. What
  happened on the bench you judge as your controller must: from the cameras, the scale and the wrist.
- **Its requests.**
  - `reset(unit, episode)` stages a bench: development worklist `episode` (0 to 4999) of that unit. The same
    unit and number stage the same bench again (the same places, tubes, fills and noise).
  - `step(action)` runs one control step and returns the observation.
  - `look(cams)` returns the cameras' images now, without a step.
  - `probe()` returns a perturbed copy of the last observation, as the grader's repeated requests carry (below).
  - `status()` returns the window's count.
- **How an episode ends there.** As a graded one does: after 20000 steps, 20 steps after the lid is shut, or at an
  unstable simulation. It does not end because a worklist is complete, and the service does not say whether it is.
- **The window.** 48,000 control steps in all, which is 80 minutes of bench time at 10 Hz (a spare bench for part
  of a morning; a graded episode is at most 20,000 steps, so the window holds few whole worklists: test the parts of
  the job, and plan what a whole run is spent on). Staging a bench costs 300 steps (half a minute to set it up). The count is kept by the service and survives reconnects; `/run/labbench/commission_status.json` shows
  it. Once it is used up, every `reset` and `step` answers `commissioning_window_used`. Nothing gives steps back.
- **Benches at a time.** Two connections, each with one bench; a third is answered `busy`. A connection silent for
  120 s is closed. The benches run on your sandbox's CPUs; a control step takes about 30 ms of wall time
  (more when images are drawn), on one bench as on two.
- **When the graded episode starts** the service closes for the rest of the session: an open bench ends at that
  moment, and every request is answered `commissioning_closed` (`commission_status.json` says
  `commissioning_closed`). A controller graded on its benches is never commissioned in the middle of its graded run.
  What is yours stays: the equipment's sheets, your own code and anything you recorded.
- **At grading** the service is not there: your controller meets the scored benches with what is in its folder.

`commission/run_bench.py` runs your submission folder on a development bench the way the grader runs it (a fresh
copy, `bash run.sh`, the grading wire with its repeated requests and its deadlines). It reports how the episode
ended, the steps, your controller's time against the grading budget and the repeated-request check; it cannot say
how the worklist went. `--images DIR` saves what the cameras show at the start and at the end.

## What is in your home directory

| Path | What it is |
|---|---|
| `task_card.md` | The equipment sheets: the robot and its control, the cameras, the tubes, the racks, the centrifuge, the scale; sizes, nominal places, and the range of everything that varies. |
| `commission/client.py` | Client of the commissioning service (`Bench`: `status`, `reset`, `step`, `look`, `probe`, `close`). |
| `commission/run_bench.py` | Runs your submission on a development bench over the grading wire. |
| `graded/client.py` | Client of the graded episode's service (`GradedEpisode`: `status`, `start`, `step`, `observe`, `finish`, `close`). |
| `graded/run_graded.py` | Runs a controller folder on the graded episode over the grading wire (it starts or resumes the episode). |
| `helpers/wire_skeleton.py` | Optional boilerplate for the wire (no control logic). |
| `session_time.sh` | Prints the minutes of your session and of its start window used and left (see "Session time"). |

Python is `/opt/venv/bin/python3` (MuJoCo 3.16, numpy, scipy, OpenCV, Pillow; robosuite 1.5.2's model files, the
Panda's among them). There is no network. The sandbox has 4 CPUs and 12 GB of memory, while grading too. Every
`/opt/venv` Python computes the same here and in the graded run, on whatever machine: the image sets, when the
variables are missing, NumPy's linear algebra on one fixed set of kernels (`OPENBLAS_CORETYPE=Haswell` with
`OPENBLAS_L2_SIZE=256`) and on one thread (`OPENBLAS_NUM_THREADS=1`, `OMP_NUM_THREADS=1`), NumPy without its
AVX-512 loops (`NPY_DISABLE_CPU_FEATURES`) and OpenCV's IPP at AVX2 (`OPENCV_IPP=avx2`). A variable that you set
yourself is kept, and your results may then depend on the grading machine's CPU.

## Credit

Each episode's credit (0 to 1) is computed from the simulation when the episode ends. The job is the centrifuge
run, so the credit is the run's.

A tube is **seated** in a hole (of a rack or of the rotor) when its tip is inside the bore and within 6 mm of the
hole's floor. The **run's tubes** are the worklist's `spin` tubes and the preloaded ones. A **balanced pair** is
two of the run's tubes seated in opposite rotor positions, their masses within 1 g.

1. **The run counts only if the centrifuge could be started on it.** All three must hold:
   - the lid is shut (its angle under 0.02 rad);
   - every tube seated in the rotor is half of a balanced pair;
   - the rotor holds at least one balanced pair.

   So a run does not count with the lid open, with a tube that has no partner opposite it (a preloaded tube whose
   partner has not been loaded is one), with a pair more than 1 g apart, or with a tube in the rotor that is not
   one of the run's (a cold tube, say).
2. **A run that counts has a share:** the balanced pairs in the rotor divided by the pairs the worklist needs
   (half the number of the run's tubes). A complete run has share 1.
3. **The credit is the share times (0.8 + 0.2 x the fraction of the cold tubes seated in their named cold-rack
   positions).** A complete run with every cold tube in place scores 1, a complete run with no cold tube in place
   0.8, and two of three pairs under a shut lid with every cold tube in place 0.67.
4. **A run that does not count scores 0**, whatever else was done. Cold tubes in their places are worth nothing
   without a run.
5. **If a tube that was not yours to move is no longer seated where it started, the episode scores 0.**

A tube anywhere else counts nothing.

An episode ends:
- after 20000 control steps (33 simulated minutes);
- 20 steps after the lid is shut, once it has been open (its angle under 0.02 rad, after having been over
  0.5 rad);
- at an unstable simulation (an arm joint faster than 8 rad/s), which scores 0.

Your controller's item is the mean credit over its 12 graded episodes; the graded episode's item is that one
episode's credit ("How you are scored" above).

## Observation and action

The controller runs at 10 Hz. The observation is a JSON object:

- **Robot:** `t` (step), `unit` (the unit's id), `q` and `dq` (7 joint angles and velocities), `ee_pos` and
  `ee_quat` (the gripper site, wxyz), `gripper` (the fingers' opening, m), `wrench` (the wrist's force and torque
  in the world frame, with noise: 0.3 N, 0.03 N m).
- **The scale:** `scale_g`, its display in grams (see `task_card.md`).
- **Cameras:**
  - `T_world_<cam>`: 4 x 4 pose in the OpenCV convention (x right, y down, z forward): the unit's calibration
    on file, which is off the camera's true pose by the unit's fixed error and off the equipment sheet's nominal
    pose by the unit's mounting (the table above);
  - `K_<cam>`: intrinsics, a pixel's centre at its index + 0.5;
  - `<cam>`: the image, `{"dtype": "uint8", "shape": [H, W, 3], "b64": ...}`.

  The four cameras are `left` and `right` (320 x 240, fixed, over the bench), `wrist` (256 x 192, on the hand)
  and `rack` (640 x 480, fixed, facing the incoming rack). An image is empty (`[0, 0, 3]`) unless the previous
  action asked for it; the first observation has all four. There is no depth image. The pictures show the bench in
  its laboratory; the room around it is not part of the task (`task_card.md`, section 3).
- **The worklist:** `worklist`, `{"cold": {incoming position: cold position}, "spin": [incoming positions],
  "preloaded": {rotor position: grams}}`.

The action is a JSON list in one of three shapes:

- 8 numbers `[dx, dy, dz, drx, dry, drz, grip, cams]`: the operational-space controller moves the gripper site's
  target by (dx, dy, dz) m (clipped to 0.03 m) and turns it by the rotation vector (drx, dry, drz) rad (clipped to
  0.2 rad), both in the world frame. `grip` runs from -1 (fully open, 80 mm) to +1 (closed).
- 15 numbers: the same, then seven joint angles. These are the posture the controller's null space pulls the arm
  toward from now on (the start posture until you set one).
- 9 numbers `[q1..q7, grip, cams]`: a joint step. A gravity-compensated joint PD moves toward q, each joint's
  target at most 0.1 rad from its current angle. q also becomes the null-space posture.

`cams` is a bitmask of the images the next observation carries: 1 left, 2 right, 4 wrist, 8 rack. Images are
large (the rack image alone is about 1.2 MB on the wire), so ask for one only when you need it.

## Session time

- This session is stopped 5 hours of wall-clock time after it started: the 2-hour start window for your controller
  and your preparation, then the graded episode's 3 hours. Your controller is graded as frozen at the graded start,
  or at the close of the start window if you do not start (the freeze, above): keep a complete, runnable submission
  in the folder from early on and replace it as you improve it. Should the session stop while the graded episode
  runs, that episode is credited as it stood after your last step.
- The only clock for these limits is the wall clock. `bash /home/user/session_time.sh` prints the minutes used and
  the minutes left of the session and of the start window, from a time stamp written when your sandbox was set up
  (moments before your session started and before the graded episode's service started, so you have slightly more
  than it says, never less); `date` shows the same clock, and the graded episode's `status()` gives both windows'
  seconds left. Run the script when you begin, again before every decision that depends on the time left, and
  whenever you pick the work up from notes or from a summary of what was done. Do not estimate the time used from
  how much you have done.
- Three other counts in this task are not your session's time: the commissioning window (48,000 control steps;
  its "80 minutes of bench time" pass several times faster than real time, at about 30 ms a step), an episode's
  length (20000 steps are 33 simulated minutes and about 10 minutes of wall time on a development bench) and
  the controller's 9000 s budget at grading. None of them says how much of the session or of the start window is
  left.

## Your controller and its grading (the first item)

Put your controller in `/home/user/submission/` with a `run.sh`. Everything it needs must be inside that folder (at
most 5000 files and 200 MiB; a larger folder scores 0), and the controller may write there. The folder graded is the
copy frozen at the graded start, or at the close of the start window if you do not start (the freeze, above). It is
graded after your session in a fresh sandbox: nothing else of your home directory is there (no `task_card.md`, no
`commission/`, no `graded/`: copy what you use into the folder), neither the commissioning service nor the graded
episode's service runs, there is no desktop session, and the only processes of your account are the ones `run.sh`
starts. A process of your account that was started any other way ends the run: that episode and every later one
score 0.

The grader starts the controller with `bash run.sh` in that folder, as your user, with only local (AF_UNIX) sockets,
and speaks one JSON object per line on its stdin and stdout:

- your controller prints `READY` within 120 s;
- `{"type": "reset"}` → `{"ok": true}` (an episode starts);
- `{"type": "act", "t": t, "obs": {...}}` → `{"action": [...]}`, within 30 s;
- `{"type": "close"}` → `{"ok": true}`, then exit.

Stdout carries only these messages; write logs to stderr. `run.sh` starts with only `PATH`, `HOME` and `LANG` set:
export anything else you need there (to render with MuJoCo, `MUJOCO_GL=osmesa`).

**Probes.** About 1 % of steps are asked twice with the same `t`: the real observation and a slightly perturbed copy
(joints by up to 1 mrad with the poses recomputed, force by up to 0.2 N; images and the scale unchanged), in a
hidden order. Only the reply to the real one is applied. A pair in which both replies command no motion (8 numbers whose
first six are all zero, within 1e-9) is not counted either way. If fewer than 10 % of the other pairs get replies that
differ (by more than 1e-4 in some number), the whole run scores 0: a controller that ignores its input, or answers a
repeated `t` from a cache, fails. The action has three shapes: a pair answered in two different shapes (8 numbers
to one request, 9 or 15 to the other) counts among the pairs and not as differing, whatever the numbers. Which steps are probed depends only on the episode, not on your replies.

**Grading.**
- There are 12 episodes: 2 for each of 6 hidden units, a fixed hidden set (each episode's seed is the same for
  every submission, so are its bench, worklist, fills, rotor start angle, lid start and noise), in a hidden order
  (the observation names the unit).
- Before the first episode the grader draws one test image of each camera. It is not charged to your controller's
  time.
- A crash, a late reply, a malformed action or an unstable simulation scores the episode 0. After 3 restarts the
  remaining episodes score 0.
- Your controller's time budget is 9000 s in all: the time it takes to print `READY` and to answer each request,
  summed over the run, plus 0.2 s for every image you ask for (a fixed price, whatever the grader's machine takes to
  draw it; the four images of an episode's first observation are free). The simulation itself is not counted. Once
  the budget is used up, the episode under way and the episodes not yet started score 0. `commission/run_bench.py`
  reports the same sum for one episode (`controller_seconds`).
- Your controller is one process for the whole run, so it may carry what it learns about a unit, or a rotor, from
  one episode to the next.
- This item is the mean credit relative to a hidden reference value; it is half of your score.

## The graded episode (the second item)

You run one episode yourself, live, during this session: the **graded episode**. It is one of the 12 hidden
episodes your controller is graded on (drawn once for this task, the same for every model), on a scored unit you
have not seen; you learn which unit from its first observation (`unit`). It is the bench of everything above, with
the same rules: the same bench, worklist, tubes, fills, rotor, lid, sensor noise and physics, the same observation
and action formats, the same end rules and the same credit ("Credit"). The simulation advances only inside your
`step` requests, so the time you spend thinking, looking at images or computing between steps costs the episode
nothing. No controller process is involved: there is no `READY`, no 30 s reply deadline, no controller-time budget
and no price per image, no repeated requests, no restarts and no rule on your processes; a malformed action is
refused (`malformed_action`) and simulates nothing, where at grading it scores the episode 0. As for a controller,
images come only with a step whose `cams` bits ask for them (the first observation has all four); there is no `look`
and no `probe`. You drive the episode from your session, with as many scripts of your own as you like
(`/opt/venv/bin/python3`):

```python
import sys; sys.path.insert(0, "/home/user")
from graded.client import GradedEpisode, GradedError, decode_image

g = GradedEpisode()          # the graded episode's service; once the episode has started, this resumes it
print(g.status())            # started, ended, steps, sim_time_s, steps_left, prep_window_left_s, graded_window_left_s
r = g.start()                # ONCE per session: r["obs"] is the first observation (all four images)
r = g.step([0, 0, 0, 0, 0, 0, -1, 0])     # one control step: r["obs"], r["done"], r["ended"], r["steps_left"]
g.close()                    # the episode stays where it is; the next GradedEpisode() resumes it

g = GradedEpisode()          # later, from another script
obs = g.observe()["obs"]     # the last observation again (images included); nothing is simulated
g.finish()                   # ends the graded episode: it is credited as it stands
```

`/opt/venv/bin/python3 ~/graded/run_graded.py --submission <folder>` runs a controller folder (a `run.sh`) on the
graded episode over the grading wire, as `commission/run_bench.py` does on a development bench: it starts the
episode (or, with `--resume`, continues it from where it stands: the controller then gets a `reset` and its first
`act` carries that observation's `t`), sends every step once, and stops at the episode's end or after `--max-steps`
steps, leaving the episode where it is; `--finish` ends the episode if the run stops before its end.

- **Once, and persistent.** `start()` is accepted once per session: it is charged the moment it is accepted and
  never refunded (later calls answer `already_started`). The episode lives in the service, not in your connection or
  script: a script that ends, crashes or closes its connection leaves the episode where it is (no simulated time
  passes), and the next `GradedEpisode()` resumes it (`observe()`). One connection at a time: `GradedEpisode()`
  raises `GradedError` with the code `busy` while another connection holds the episode (end that script first); a
  connection silent for 600 s is closed (the episode stays). The service records the episode as it stands after
  your requests (at least once a second while you work); should its own process fail, the episode ends as last
  recorded and is not resumed.
- **Starting it freezes your controller and closes the commissioning service** (above): finish the controller
  first.
- **Two windows.** Start the graded episode within 2 hours of the service's start (the service starts with your
  sandbox, moments before your session begins; `status()["prep_window_left_s"]` counts down); after that, `start()`
  answers `prep_window_closed`, this item scores 0, and your controller is frozen as it is then
  (`status()["program_frozen"]`). Once started, the episode lasts at most 3 hours of wall clock
  (`graded_window_left_s`).
- **How it ends.** At its own end, by the rules of a graded episode (after 20000 control steps, 20 steps after the
  lid is shut once it has been open, or at an unstable simulation, which scores 0: `r["done"]` and `r["ended"]` say
  which), at your `finish()`, when its 3 hours run out, or when your session ends, whichever comes first. It is then
  credited as it stands: nothing is simulated after the end, and a run counts only under a shut lid. If your session
  ends first, the episode counts as it stood after your last step. Once it has ended, the service answers `status()`
  and `finish()` only (`episode_over` to the rest).
- **How it is credited.** This item is the episode's credit by the rules of "Credit" above, relative to its own
  hidden reference value; it is the other half of your score. The service tells you nothing about the credit: judge
  the episode by what the cameras, the wrist and the scale show, as your controller does.

The controller's rules and budgets (the 30 s reply deadline, the 9000 s budget with its 0.2 s per image, the repeated
requests, the restarts, the rules on processes, the folder's caps) bind your controller in its 12 hidden episodes and
on the commissioning benches (`commission/run_bench.py` runs it the way the grader does); the graded episode has none
of them.

# Start up, operate and shut down a pump skid with a rail-mounted robot arm, seen only through its cameras

You are the automation engineer of a water plant that is unmanned at night. Its transfer and chlorine dosing skid has
to be run by a robot through the shift: a Franka Panda on a carriage that runs along a rail in front of the skid, with a
lift that sets its base height, a parallel-jaw gripper and a wrist RGB-D camera; one more RGB-D camera, fixed high
behind the rail, overlooks the whole skid. The robot works the skid the way a field operator does: it turns handwheels and levers, presses the
buttons and turns the selector of the local control station, and reads the dial gauges, the flow meter, the level
gauges, the sight glasses, the lamps and the analyser's display. Nothing on the API tells you where a device is, which
device is which, what a valve's position is, what a gauge reads or what credit you have earned: you must perceive it. The cameras'
calibration (intrinsics, the fixed cameras' poses, the wrist camera's mount) is exact and published.

Each episode is one shift on a skid of the series, following the operating procedure `~/given/sop.md`:

1. **Start-up** (before the operating window opens at 600 s of the cell's simulated clock): line the valves up (the
   previous shift may have left some of them out of place), prime the pump through its casing vent, start it on its
   minimum-flow line, open the discharge, fill the product tank T-201 into its band with the flow-control valve, and
   start the chlorine dosing.
2. **Operation** (600-1500 s): the consumer draws from T-201 at a rate no instrument shows. Hold T-201's level
   (LG-201) in 55-75 % and the residual chlorine (AI-201) in 1.8-2.6 mg/L, record six instrument readings on the log
   sheet in each of its two rounds (600-1050 s and 1050-1500 s), and handle whatever happens on the shift by the SOP's procedures: the consumer's draw may change, and several of
   the abnormal situations of the SOP may occur on one shift (a debris slug clogging the strainer, the supply tank's float valve
   sticking shut, an air lock in the dosing pump, the duty dosing drum running dry, a trip of the transfer pump), none
   announced. One common alarm lamp serves
   several faults, and some faults share their first symptoms: tell them apart on the instruments (`sop.md` section 3)
   before you act, since the wrong procedure does not help and some make things worse.
3. **Shutdown** (after 1500 s): stop the dosing, close the discharge, stop the pump, close the suction, selector OFF.

Every skid of the series has the same process and the same series tags (`~/given/pid.png`, `sop.md`,
`equipment_manual.md`), but each episode is a different site. A site numbers its equipment its own way: the tag plates
and the control station's labels show the **site tags**, and `describe()["site"]` is the site's equipment list (series
tag -> site tag, service, start-up position) and log sheet. A site also chooses how each valve is built (a handwheel or
a lever on the isolation valves, a lever or a small handwheel on the minimum-flow valve, a lever or a needle valve on
the vents) and the arrangement (the line either way along the skid, the flow meter before or after the flow control
valve, the control station's two pump groups either way round, positions and pipe heights). The fitted gauge scales
and units vary too, as does everything a shift brings: the state the previous shift left, the process (the pump, the
levels, the consumer's draw, the drums) and what goes wrong, and when. The tag plates are legible only from close up.

## How you are scored: your program, and one shift you run live

The score is the mean of two items, each relative to its own hidden reference value:

1. **Your program**: `operation_score` of the program in `/home/user/submission/` over 8 hidden episodes, eight sites
   of the series it has never run on (what you produce and how it is graded: the next sections).
2. **The graded episode**: one shift on a hidden site of the series that you run yourself, live, during this session,
   scored by the same procedure (its section below).

**The freeze.** The program graded is the one in the submission folder at the moment you start the graded episode. At
that moment, before the shift begins, the graded episode's service copies the folder out of your reach (regular files
and folders only, symbolic links skipped; above 60 MB, or 10,000 files and folders, nothing is copied and the program
scores 0), and nothing you change in the folder afterwards reaches the program's grading: finish the program before
you start the shift. If you
never start the graded episode, the folder as it is when your session ends is graded, and the graded episode scores 0.

**Practice** (commissioning, below) closes when the graded episode starts.

**The windows.** You have 4 hours from the service's start (it starts with your sandbox, a few minutes before your
session begins) to write the program and to prepare, commissioning included; start the graded episode within them
(later, `start()` answers `prep_window_closed`). The graded episode then has at most 3 hours. The session lasts 7 hours
in all.

## What you produce (in `/home/user/submission/`)

`operate.py` exposing `solve(cell) -> None` (plus any helper modules, optional `run.sh`). The grader runs it once per
hidden episode (8 episodes, eight sites of the series) in its own cell. It talks to the cell through `cell` (API in `~/given/cell/cell_api.md`):
the robot's own state (joints, tool pose, gripper width and grip signal, the wrist's force-torque estimate, the
protective stop, the rail and the lift), the horn, the two RGB-D cameras (`overview`, `wrist`), the site's equipment list, and `log_reading` for the log sheet (site
tags); nothing else about the skid. Per episode: 1800 s of
simulated time (the clock advances only inside `step`); 1200 s of program time, which is the episode's wall clock minus
the time the cell spends simulating (inside `step`) and rendering (inside `get_camera`), so it measures your program's
own computation and its other requests, and 4x that in CPU seconds (all threads together); at most 180 s from a reply
to your next request; 600 camera frames in total.

How the program is run: a fresh process under a dedicated unprivileged account that is not yours, from a per-run
directory with a read-only copy of your submission folder (regular files only), an empty writable HOME / TMPDIR, a
minimal environment (`PATH`, `HOME`, `TMPDIR`, `LANG`, `MUJOCO_GL`, `PYTHONPATH` = the given library `~/given/lib`,
`ARB_GIVEN_DIR` = a read-only copy of `~/given`) and the launcher `python3 -B -E -s -c <bootstrap> <lib> operate` (or
`bash run.sh`), with no network (a seccomp filter refuses every socket but a Unix-domain one, for the program and
everything it starts). Before the first episode the grader imports your module once under this regime to check that it
defines `solve`: the import must finish within 120 s of wall clock and 120 CPU seconds (all threads together), or every
episode scores 0; do heavy precomputation inside `solve`, not at module level. Resolve given files through `os.environ.get("ARB_GIVEN_DIR", os.path.expanduser("~/given"))`, write only
under the run's HOME / TMPDIR. A program that crashes ends that episode with what it achieved; the other episodes still
run. An episode in which the cell itself failed (an internal error, a physics fault, a frame it could not render) is run
once more from scratch, and the second run counts. Your program must be final when you start the graded episode (the
freeze, above): the frozen copy (or, if you never start the graded episode, the folder at your session's end) is graded
after your session in a fresh sandbox that holds nothing else of yours (no other file, no process you started), and a
surviving process of the run account voids the contract gate.

## What you are given

| path | content |
|---|---|
| `~/given/sop.md` | the operating procedure: start-up, operation, the abnormal-situation procedures, shutdown, what never to do, how the shift is scored |
| `~/given/pid.png` | the skid's P&ID (process and tags; schematic) |
| `~/given/equipment_manual.md` | every device: how it is built, operated and read; the gauge scales; what varies between skids |
| `~/given/cell/cell_manual.md` | the robot cell: frame, robot, rail and lift, control, the wrist load, the grip rule and how a turn reads, safety and protective stops, the cameras with their calibration, time and budgets, commissioning |
| `~/given/cell/cell_api.md` | the cell's programming manual (the wire API) |
| `~/given/cell/cell_config.json` | the cell's configuration as far as it is public |
| `~/given/lib/skidcell/` | the cell's client: `client` (the proxy, `decode_frame`, the program launcher, `connect_commissioning`, `connect_graded`); the cell itself runs in the grader, in the graded episode's service and in the commissioning service, not in your account |
| `~/given/catalog/` | the instrument catalog: every dial face and scale plate the series fits, two sample tag plates (`catalog.md`) |
| `~/given/panda/` | the robot's MuJoCo model (Apache-2.0) |
| `~/example/operate.py` | a minimal program (reads frames, moves the carriage, finishes; scores 0): the starting point |
| `~/commission/run_shift.py` | runs your program in one commissioning shift on a development site, the way the grader runs it (below; keep it out of your submission folder: it starts a process) |

Tools: Python 3.12 (`/opt/venv/bin/python3`; call it by that path, the bare `python3` on your shell's `PATH` may be
another interpreter) with numpy, scipy, OpenCV (`cv2`, headless), Pillow, MuJoCo 3.14. No network in your session.

## How your program is graded (the first item: verify runs 8 hidden episodes)

Gates (multiply the score): `contract_ok` = `operate.py` is a regular file that parses and defines `solve`, passes the
route gate (no weight files, no training calls, submission <= 60 MB), imports under the run regime above (the import check), makes at
least one `step` call over the episodes, holds no network socket during any episode, and your own `.py` files
import no networking module (`socket`, `ssl`, `urllib`, `http`, `requests`, `asyncio`, `ctypes`, `subprocess`, ...),
load no module under a computed name, execute no computed code (`exec` / `eval` / `compile` of a non-literal) and start
no other program (`os.system`, `os.exec*`, `os.spawn*`, ...).

`operation_score` = the mean over the episodes of the shift's score, `sop.md` section 6: start-up 0.20, operation in
band 0.25, the window's events 0.25, shutdown 0.15, the log sheet 0.15, judged on the cell's own state and the
process's true values. An episode in which the pump is deadheaded for 30 s or damaged by 300 s of cavitation, T-201
overflows, the emergency stop is pressed, or the robot is taken out of service scores 0. When the robot presses on the
skid harder than the safety limits (more than 40 N for 30 ms on anything but a handle or a button, more than 150 N on a
handle or a button), the cell makes a protective stop: the arm halts for 5 s, then moves again when commanded. A
contact above 400 N for 30 ms, or a sixth protective stop in the shift, takes the robot out of service. T-101
overflowing (to the drain) costs the start-up's line-up credit. This item is `operation_score` relative to a hidden
reference value; it is half of your score. Stats (not scored): the parts' means, event credits, frames per camera, sim
/ wall / program time, illegal calls, protective stops, safety events.

## The graded episode (the second item: one shift you run live)

You run one shift yourself, live, during this session: the **graded episode**, on a hidden site of the series drawn
like the sites your program is graded on (none of the development sites below: it has its own numbering, builds,
leftovers, faults and timings). It is the cell of everything below, with the same rules (`cell_api.md`, `cell_manual.md`, `sop.md`): 1800 s of
simulated time, at most 1000 ticks a `step`, 600 camera frames over both cameras, the safety watch and its protective
stops, the procedure's scoring. The simulated clock advances only inside your `step` calls, so the time you spend
thinking, perceiving or computing between calls costs the shift nothing. No program process is involved: there is no
program-time, CPU or silence budget, no import check and no program contract. You drive the episode from your session,
with as many scripts of your own as you like (`/opt/venv/bin/python3`):

```python
import os, sys
sys.path.insert(0, os.path.expanduser("~/given/lib"))
from skidcell.client import connect_graded, decode_frame, CellError

cell = connect_graded()      # the graded episode's service; once the episode has started, this resumes it
print(cell.status())         # started, ended, end, steps, sim_time_s, camera_frames_used / _left,
                             # prep_window_left_s, graded_window_left_s
cell.start()                 # ONCE per session: the shift begins (sim_time 0, the skid as the previous shift left it)
info = cell.describe()       # from here on the cell's calls of cell_api.md, as a program makes them
r = cell.step(100)
...
cell.close()                 # the episode stays where it is; the next connect_graded() resumes it
...
cell.finish()                # ends the graded episode: it is scored as it stands
```

A `solve(cell)` of yours runs on it as `sys.path.insert(0, os.path.expanduser("~/submission")); from operate import
solve; cell = connect_graded(); cell.start(); solve(cell)`, in a script of your own (`print()` goes to your terminal:
there is no stdio wire here). Mind `finish()`: it ends the graded episode at once, so a program that calls it at its end
(as `~/example/operate.py` does) ends the shift there.

* **Once, and persistent.** `start()` is accepted once per session: it is charged the moment it is accepted and never
  refunded (later calls answer `already_started`). The episode lives in the service, not in your connection or script:
  a script that ends, crashes or closes its connection leaves the shift where it is (no simulated time passes), and the
  next `connect_graded()` resumes it. One connection at a time: `connect_graded()` raises `CellError("busy")` while
  another connection holds the episode (end that script first). Before `start()` every cell call answers
  `not_started`; `reset` and any call outside `cell_api.md` answer `forbidden` and count as illegal calls, as in the
  cell. The service records the episode as it stands after your requests (at least once a second while you work);
  should its own process fail, the episode ends as last recorded and is not resumed.
* **Practice ends when the graded episode starts.** The moment `start()` is accepted, the commissioning service ends
  any open shift (it sends the event `shift_over` with the reason `practice_closed` and closes the connection; the
  client raises `CellError`) and refuses new shifts for the rest of the session (`CellError("practice_closed")`): a
  program at its site has no sister site to practise on in the middle of its shift, so commission first. What is yours
  stays available: the robot's model, your own simulations, and anything you wrote or recorded.
* **Two windows.** Start the graded episode within 4 hours of the service's start (the service starts with your
  sandbox, a few minutes before your session begins; `status()["prep_window_left_s"]` counts down); after that,
  `start()` answers `prep_window_closed` and this item scores 0. Once started, the episode lasts at most 3 hours of
  wall clock (`graded_window_left_s`).
* **How it ends.** At its own end (1800 s of simulated time, or when the cell ends it: a physics fault or an internal
  error of the cell; the graded episode is never run again), at your `finish()`, when its 3 hours run out, or when your
  session ends, whichever comes first. It is then scored as it stands, as a shift that stops at that moment: nothing
  is simulated after the end, so a shift ended at 900 s earns nothing for the rest of the operating window, for events
  still to come or for the shutdown. Finish only once the shutdown is done (after 1500 s). If your session ends first,
  the episode counts as it stood after your last request. Once it has ended, the service answers `status()` and
  `finish()` only (`episode_over` to the rest).
* **How it is scored.** This item is the graded episode's `operation_score`, its shift's score by `sop.md` section 6: start-up
  0.20, operation in band 0.25, the window's events 0.25, shutdown 0.15, the log sheet 0.15, judged on the cell's own
  state and the process's true values. The shift scores 0 if the pump is deadheaded for 30 s or damaged by 300 s of
  cavitation, T-201 overflows, the emergency stop is pressed, or the robot is taken out of service. When the robot
  presses on the skid harder than the safety limits (more than 40 N for 30 ms on anything but a handle or a button,
  more than 150 N on a handle or a button), the cell makes a protective stop: the arm halts for 5 s, then moves again
  when commanded. A contact above 400 N for 30 ms, or a sixth protective stop in the shift, takes the robot out of
  service. T-101 overflowing (to the drain) costs the start-up's line-up credit. The item is this `operation_score`
  relative to its own hidden reference value; it is the other half of your score. The service tells you nothing about
  the score: judge the shift by what you see. Your program is graded apart, on the copy frozen at the start (above).

The program's budgets and rules (`cell_api.md`: 1200 s of program time and 4x that in CPU seconds, 180 s of silence,
the import check, an episode run once more after a cell failure) bind your program in its 8 hidden episodes and in the
commissioning shifts (`~/commission/run_shift.py` runs it the way the grader does); the graded episode has none of them.

## Rules of the cell (details in `cell_manual.md` / `cell_api.md`)

* The gripper holds a handle by the documented grip rule: closing the fingers on a handle (a handwheel's spoke or rim,
  a lever, a needle valve's knob, the selector's wing) that lies in the pinch box, with the tool's approach and
  closing axes within 30 degrees of the handle's, attaches the handle's moving part to the hand; it then moves only as
  its own joint allows (move the hand on its circle about the stem, within about 1-2 mm), slips out above 60 N or 3 N m
  (the moment about the held part's pivot), and opening the fingers releases it. Buttons are pushed.
* The wrist load is the external wrench on the hand estimated from the joint torques, as on a real Franka (force and
  moment, world axes, at the tcp; noisy, with an offset to re-zero): it feels contacts, button presses and a held
  handle's load. What a normal turn and an end stop read: `cell_manual.md` section 3.
* A hard contact with the skid is a protective stop, as on a collaborative robot (`cell_manual.md` section 4).
* The process runs only while the cell is stepped; the clock is the cell's `sim_time`.
* Calls other than the documented public ones answer `{"error": "forbidden"}` and are counted.

## Commissioning on the series' sister sites (no rehearsal of the graded sites)

Your program is graded on eight sites it has never run on, as a robot program goes to sites it runs unattended. Before
that, like any automation engineer's, it can be commissioned on sister sites: the commissioning service in your
sandbox runs the series' cell on 9 development sites (`cell_manual.md` section 8).

* **What it is.** The cell of the graded episodes (the same robot and controller, grip rule, wrist load, safety watch,
  cameras, process and faults) behind the same calls (`cell_api.md`), plus `reset(site)`, which starts a shift on
  development site 1-9. You see what a commissioning engineer sees on a sister site: the cameras' real frames, the
  robot's state and wrist load, the grip, slip and protective-stop events, and through the cameras the lamps and the
  instruments. The development sites are skids of the series built and numbered within the equipment manual; between
  them they show every build option, gauge scale and abnormal situation the documents describe.
* **What it is not.** A rehearsal of the graded shifts or a grader: no development site is a graded site (the graded
  sites have their own numbering, builds, leftovers, faults and timings), and the service answers no ground truth (no
  device pose, valve position or process value), no score and no debrief. How a shift went, you judge from what your
  program saw and recorded. The simulator's code is not readable from your account.
* **Its budget.** 12 commissioning shifts in your session, at most 2 at a time, and none once the graded episode has
  started (its start ends commissioning for the session); each under the hidden episodes' budgets (1800 s of simulated
  time, 1200 s of program time, 600 frames, 180 s of silence). Plan them: teach-in and single steps first
  (`skidcell.client.connect_commissioning(site)` in a script of your own), then whole runs of your program
  (`/opt/venv/bin/python3 ~/commission/run_shift.py --site N`). The count: `/run/skidcell/commission_status.json`
  (`practice_closed` once the graded episode has started).

The robot's own model (`~/given/panda/`, MuJoCo) is yours to simulate the arm with.

## Notes

- Photos and camera frames are meant to be processed programmatically; view only the images you need.
- Do not modify the given files in your home directory; the grader reads its own copies.
- Environment facts (harness limits): a single tool call whose input runs above roughly 25-30k tokens is dropped by the
  harness (it is never executed and no error comes back), and a response that reaches the 64k output-token limit is lost
  and, after at most a few retries, ends the session (the program graded is then the copy frozen at the graded
  episode's start or, if it never started, whatever is in `/home/user/submission/` at that moment; the graded episode
  counts as it stood after your last request). Write large files in several parts (one function group per write), keep
  every single response well below that limit, write a first complete submission early and refine it.
- Session time: this session is stopped after 7 hours of wall-clock time from its start (the 4-hour window for the
  program and your preparation, then the graded episode's 3 hours). Should it stop while the graded episode runs, the
  episode is scored as it stood after your last request, with nothing simulated afterwards; should it stop before you
  start the graded episode, the submission folder is graded as it is then. Note the start time with `date` when you
  begin, keep a complete program in the folder from early on, start the graded episode in time, and when its shift is
  done (after the shutdown) call `finish()`, then end the session yourself (stop working and say that you are done).

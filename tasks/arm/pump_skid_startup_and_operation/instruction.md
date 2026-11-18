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

## The graded episode (the only thing that is scored)

You run one shift yourself, live, during this session: the **graded episode**, on a hidden site of the series drawn
like the graded sites (none of the development sites below: it has its own numbering, builds, leftovers, faults and
timings). It is the cell of everything below, with the same rules (`cell_api.md`, `cell_manual.md`, `sop.md`): 1800 s of
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
  `start()` answers `prep_window_closed` and the task scores 0. Once started, the episode lasts at most 3 hours of
  wall clock (`graded_window_left_s`).
* **How it ends.** At its own end (1800 s of simulated time, or when the cell ends it: a physics fault or an internal
  error of the cell; the graded episode is never run again), at your `finish()`, when its 3 hours run out, or when your
  session ends, whichever comes first. It is then scored as it stands, as a shift that stops at that moment: nothing
  is simulated after the end, so a shift ended at 900 s earns nothing for the rest of the operating window, for events
  still to come or for the shutdown. Finish only once the shutdown is done (after 1500 s). If your session ends first,
  the episode counts as it stood after your last request. Once it has ended, the service answers `status()` and
  `finish()` only (`episode_over` to the rest).
* **What is scored.** Only the graded episode. `operation_score` is its shift's score, `sop.md` section 6: start-up
  0.20, operation in band 0.25, the window's events 0.25, shutdown 0.15, the log sheet 0.15, judged on the cell's own
  state and the process's true values. The shift scores 0 if the pump is deadheaded for 30 s or damaged by 300 s of
  cavitation, T-201 overflows, the emergency stop is pressed, or the robot is taken out of service. When the robot
  presses on the skid harder than the safety limits (more than 40 N for 30 ms on anything but a handle or a button,
  more than 150 N on a handle or a button), the cell makes a protective stop: the arm halts for 5 s, then moves again
  when commanded. A contact above 400 N for 30 ms, or a sixth protective stop in the shift, takes the robot out of
  service. T-101 overflowing (to the drain) costs the start-up's line-up credit. The reward you are judged on is
  `operation_score` relative to a hidden reference value. Nothing else is graded: `/home/user/submission/` is not
  read, and the service tells you nothing about the score; judge the shift by what you see.

The givens also describe a program run by a grader: `cell_api.md`'s first paragraph and its budgets (a program process
on a pipe, its import check, 1200 s of program time and 4x that in CPU seconds, 180 s of silence, an episode run once
more after a cell failure) and `~/commission/run_shift.py`. Those hold for the commissioning shifts; the graded episode
is the one described here.

## What you are given

| path | content |
|---|---|
| `~/given/sop.md` | the operating procedure: start-up, operation, the abnormal-situation procedures, shutdown, what never to do, how the shift is scored |
| `~/given/pid.png` | the skid's P&ID (process and tags; schematic) |
| `~/given/equipment_manual.md` | every device: how it is built, operated and read; the gauge scales; what varies between skids |
| `~/given/cell/cell_manual.md` | the robot cell: frame, robot, rail and lift, control, the wrist load, the grip rule and how a turn reads, safety and protective stops, the cameras with their calibration, time and budgets, commissioning |
| `~/given/cell/cell_api.md` | the cell's programming manual (the wire API) |
| `~/given/cell/cell_config.json` | the cell's configuration as far as it is public |
| `~/given/lib/skidcell/` | the cell's client: `client` (the proxy, `decode_frame`, the program launcher, `connect_commissioning`, `connect_graded`); the cell itself runs in the graded episode's service and in the commissioning service, not in your account |
| `~/given/catalog/` | the instrument catalog: every dial face and scale plate the series fits, two sample tag plates (`catalog.md`) |
| `~/given/panda/` | the robot's MuJoCo model (Apache-2.0) |
| `~/example/operate.py` | a minimal program (reads frames, moves the carriage, then calls `finish()`; it earns nothing): a starting point |
| `~/commission/run_shift.py` | runs your program in one commissioning shift on a development site, the way a grader runs a program (below) |

Tools: Python 3.12 (`/opt/venv/bin/python3`; call it by that path, the bare `python3` on your shell's `PATH` may be
another interpreter) with numpy, scipy, OpenCV (`cv2`, headless), Pillow, MuJoCo 3.14. No network in your session.

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

## Commissioning on the series' sister sites (no rehearsal of the graded site)

Your graded shift runs on a site you have never seen, as a robot program goes to the sites it runs unattended. Before
that, like any automation engineer's, your program can be commissioned on sister sites: the commissioning service in
your sandbox runs the series' cell on 9 development sites (`cell_manual.md` section 8).

* **What it is.** The cell of the graded episode (the same robot and controller, grip rule, wrist load, safety watch,
  cameras, process and faults) behind the same calls (`cell_api.md`), plus `reset(site)`, which starts a shift on
  development site 1-9. You see what a commissioning engineer sees on a sister site: the cameras' real frames, the
  robot's state and wrist load, the grip, slip and protective-stop events, and through the cameras the lamps and the
  instruments. The development sites are skids of the series built and numbered within the equipment manual; between
  them they show every build option, gauge scale and abnormal situation the documents describe.
* **What it is not.** A rehearsal of the graded shift or a grader: no development site is a graded site (the graded
  site has its own numbering, builds, leftovers, faults and timings), and the service answers no ground truth (no
  device pose, valve position or process value), no score and no debrief. How a shift went, you judge from what your
  program saw and recorded. The simulator's code is not readable from your account.
* **Its budget.** 12 commissioning shifts in your session, at most 2 at a time, and none once the graded episode has
  started (its start ends commissioning for the session); each under a program's budgets (1800 s of simulated time,
  1200 s of program time, 600 frames, 180 s of silence; `cell_manual.md` calls them the graded episode's). Plan them:
  teach-in and single steps first (`skidcell.client.connect_commissioning(site)` in a script of your own), then whole
  runs of your program (`/opt/venv/bin/python3 ~/commission/run_shift.py --site N`). The count:
  `/run/skidcell/commission_status.json` (`practice_closed` once the graded episode has started).

The robot's own model (`~/given/panda/`, MuJoCo) is yours to simulate the arm with.

## Notes

- Photos and camera frames are meant to be processed programmatically; view only the images you need.
- Do not modify the given files in your home directory; the services use their own copies.
- Environment facts (harness limits): a single tool call whose input runs above roughly 25-30k tokens is dropped by the
  harness (it is never executed and no error comes back), and a response that reaches the 64k output-token limit is lost
  and, after at most a few retries, ends the session (the graded episode then counts as it stood after your last
  request). Write large files in several parts (one function group per write), keep every single response well below
  that limit, and keep your programs and notes in files.
- Session time: this session is stopped after 7 hours of wall-clock time from its start (the 4-hour start window, then
  the graded episode's 3 hours). A session stopped by that limit is not graded at all, so a late finish scores nothing.
  Note the start time with `date` when you begin, start the graded episode in time, and end the session yourself (stop
  working and say that you are done) once the graded episode has ended, at least 15 minutes before the limit.

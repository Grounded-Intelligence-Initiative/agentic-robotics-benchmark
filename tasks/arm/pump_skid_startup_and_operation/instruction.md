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
   sheet, and handle whatever happens on the shift by the SOP's procedures: the consumer's draw may change, and any of
   the abnormal situations of the SOP may occur (a debris slug clogging the strainer, the supply tank's float valve
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

## What you produce (in `/home/user/submission/`)

`operate.py` exposing `solve(cell) -> None` (plus any helper modules, optional `run.sh`). The grader runs it once per
hidden episode (4 episodes) in its own cell. It talks to the cell through `cell` (API in `~/given/cell/cell_api.md`):
the robot's own state (joints, tool pose, gripper width and grip signal, the wrist load, the rail and the lift), the
horn, the two RGB-D cameras (`overview`, `wrist`), the site's equipment list, and `log_reading` for the log sheet (site
tags); nothing else about the skid. Per episode: 1800 s of
simulated time (the clock advances only inside `step`); 2400 s of program time, which is the episode's wall clock minus
the time the cell spends simulating (inside `step`) and rendering (inside `get_camera`), so it measures your program's
own computation and its other requests; at most 180 s from a reply to your next request; 900 camera frames in total.

How the program is run: a fresh process under a dedicated unprivileged account that is not yours, from a per-run
directory with a read-only copy of your submission folder (regular files only), an empty writable HOME / TMPDIR, a
minimal environment (`PATH`, `HOME`, `TMPDIR`, `LANG`, `MUJOCO_GL`, `PYTHONPATH` = the given library `~/given/lib`,
`ARB_GIVEN_DIR` = a read-only copy of `~/given`) and the launcher `python3 -B -E -s -c <bootstrap> <lib> operate` (or
`bash run.sh`). Resolve given files through `os.environ.get("ARB_GIVEN_DIR", os.path.expanduser("~/given"))`, write only
under the run's HOME / TMPDIR. A program that crashes ends that episode with what it achieved; the other episodes still
run. Your submission must be final when your session ends: the submission folder is then copied out of your sandbox
and graded in a fresh sandbox that holds nothing else of yours (no other file, no process you started), and a
surviving process of the run account voids the contract gate.

## What you are given

| path | content |
|---|---|
| `~/given/sop.md` | the operating procedure: start-up, operation, the abnormal-situation procedures, shutdown, what never to do, how the shift is scored |
| `~/given/pid.png` | the skid's P&ID (process and tags; schematic) |
| `~/given/equipment_manual.md` | every device: how it is built, operated and read; the gauge scales; what varies between skids |
| `~/given/cell/cell_manual.md` | the robot cell: frame, robot, rail and lift, control, the grip rule, safety, the cameras with their calibration, time and budgets |
| `~/given/cell/cell_api.md` | the cell's programming manual (the wire API) |
| `~/given/cell/cell_config.json` | the cell's configuration as far as it is public |
| `~/given/lib/skidcell/` | the cell's client: `client` (the proxy, `decode_frame`, the program launcher); the cell itself runs in the grader, not in your account |
| `~/given/catalog/` | the instrument catalog: every dial face and scale plate the series fits, two sample tag plates (`catalog.md`) |
| `~/given/panda/` | the robot's MuJoCo model (Apache-2.0) |
| `~/example/operate.py` | a minimal program (reads frames, moves the carriage, finishes; scores 0): the starting point |

Tools: Python 3.12 (`/opt/venv/bin/python3`; call it by that path, the bare `python3` on your shell's `PATH` may be
another interpreter) with numpy, scipy, OpenCV (`cv2`, headless), Pillow, MuJoCo 3.14. No network in your session.

## How you are graded (verify runs 4 hidden episodes)

Gates (multiply the score): `contract_ok` = `operate.py` is a regular file that parses and defines `solve`, passes the
route gate (no weight files, no training calls, submission <= 60 MB), imports under the run regime above, makes at
least one `step` call over the episodes, causes no network activity during any episode, and your own `.py` files
import no networking module (`socket`, `ssl`, `urllib`, `http`, `requests`, `asyncio`, `ctypes`, `subprocess`, ...),
load no module under a computed name, execute no computed code (`exec` / `eval` / `compile` of a non-literal) and start
no other program (`os.system`, `os.exec*`, `os.spawn*`, ...).

`operation_score` = the mean over the episodes of the shift's score, `sop.md` section 6: start-up 0.20, operation in
band 0.25, the window's events 0.25, shutdown 0.15, the log sheet 0.15, judged on the cell's own state and the
process's true values. An episode in which the pump is deadheaded for 30 s or damaged by 300 s of cavitation, T-201
overflows, the emergency stop is pressed, or the robot presses on the skid harder than the safety limits (more than
40 N for 30 ms on anything but a handle or a button, more than 150 N on a handle or a button) scores 0. The reward you
are judged on is `operation_score` relative to a hidden reference value. Stats (not scored): the parts' means, event
credits, frames per camera, sim / wall / program time, illegal calls, safety events.

## Rules of the cell (details in `cell_manual.md` / `cell_api.md`)

* The gripper holds a handle by the documented grip rule: closing the fingers on a handle (a handwheel's spoke or rim,
  a lever, a needle valve's knob, the selector's wing) that lies in the pinch box, with the tool's approach and
  closing axes within 30 degrees of the handle's, attaches the handle's moving part to the hand; it then moves only as
  its own joint allows, slips out above 60 N or 6 N m, and opening the fingers releases it. Buttons are pushed.
* The process runs only while the cell is stepped; the clock is the cell's `sim_time`.
* Calls other than the documented public ones answer `{"error": "forbidden"}` and are counted.

## No rehearsal

Like a field operator who takes over a shift on a site of the series, your program works each site for the first time
in its graded episode: there is no simulator of the skid in your sandbox. What you know before the shift is what the
documents above say (the procedure, the P&ID, the equipment manual, the instrument catalog, the cell's manuals and
configuration); the rest your program learns on the shift through the cameras and the robot's state. The robot's own
model (`~/given/panda/`, MuJoCo) is yours to simulate the arm with.

## Notes

- Photos and camera frames are meant to be processed programmatically; view only the images you need.
- Do not modify the given files in your home directory; the grader reads its own copies.
- Environment facts (harness limits): a single tool call whose input runs above roughly 25-30k tokens is dropped by the
  harness (it is never executed and no error comes back), and a response that reaches the 64k output-token limit is lost
  and, after at most a few retries, ends the session with whatever is in `/home/user/submission/` at that moment. Write
  large files in several parts (one function group per write), keep every single response well below that limit, write
  a first complete submission early and refine it.
- Session time: this session is stopped after 4 hours of wall-clock time from its start. A session stopped by that limit
  is not graded at all, so a late finish scores nothing. Note the start time with `date` when you begin, keep a complete
  submission in place from early on, and end the session yourself (stop working and say that you are done) at least
  15 minutes before the limit.

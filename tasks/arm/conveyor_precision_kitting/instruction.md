# Commission a conveyor kitting station

Kitting station KS-7 feeds an assembly line with kits of six machined parts. Parts arrive one at a time on a moving
belt; a Franka Panda must take each needed part off the belt while it moves, turn over the ones that lie upside down
(a turnover station next to the belt does that), and seat every part in its own keyed pocket of a precision kit
tray: **1.2 mm of diametral clearance, a 1 mm lead-in chamfer and a key boss** that only lets the part in the
right way round. A kit with a part that is not fully seated stops the line downstream, and a part pressed into the tray
too hard is scrapped. The station has been installed; you are the engineer commissioning it. **Deliver the
controller that runs it**: it sees only what the station's cameras and sensors report, and it is judged on how many
pockets it fills correctly over hidden shifts of the station.

The integrator handed the station over with a survey, not a model: the data sheet gives nominal values and, for
everything it found to change from day to day (the command latency, how stiffly the arm tracks, the belt speed and
its drift, the lamps, the parts' surface condition, where the tray sits), the range it changes in. On any one shift
the station sits somewhere in those ranges and nothing tells your controller where: it has to find out on the
shift, while the belt runs.

## What you have

- `~/cell/datasheet.md`: the station's data sheet and hand-over survey: frames, the robot and its controller (the
  action, the command latency, the tracking law, with their ranges), the belt, the six part variants (drawings), the kit tray (pockets, clearances, keys,
  fiducials, the force limit), the turnover station, the sensors and cameras (calibration as drawn), what the
  downstream kit inspection counts, and the ranges within which the real station differs from its drawing every
  shift. Read it first. `~/cell/cell_data.py` holds its numbers as Python.
- `~/practice/run_practice.py`: runs your submission on **one practice shift of the real station** through the
  same wire and timing rules as the grader, and prints what the kit inspection found per pocket and how many parts
  were lost; it can also save the images your controller received (`--video out.mp4`) and a log
  (`--log out.jsonl`). `~/practice/client.py` lets you drive a practice shift yourself, step by step, but **the practice
  station does not wait either**: it applies the same 0.1 s reply rule as the grader (below), a step request later
  than 30 s ends the shift, and a practice shift ends after 90 s of wall-clock time (the reference controller needs
  about 40 s). **The whole session has 2 practice shifts**
  (`/run/kitcell/practice_status.json`); a shift is spent when it starts. There is no simulator of the station on
  this machine, and the station's model files are not readable from your account.
- Python: `/opt/venv/bin/python3` (Python 3.12) with numpy, scipy, opencv (headless) and pillow (no simulator: MuJoCo and the robot models are not available to your account). Nothing
  else can be installed: there is no network.
- 4 CPUs, 12 GB of memory, no GPU, and **6 hours** in total. The environment sets `OMP_NUM_THREADS=1` (the setting
  your controller is graded with). Leave a working submission in place early and improve it.

## What you submit

`/home/user/submission/run.sh`, which starts your controller, plus everything it loads, at most 100 MB in that
folder. Only that folder is kept. The grader starts `bash run.sh` with that folder as the working directory, as your
user, with the same Python and packages, no network and no GPU, and talks to it over standard input and output, one
JSON object per line:

1. Your program prints `READY` once it is ready (within 20 s of starting).
2. `{"type": "reset"}` at the start of each shift: answer `{"ok": true}`.
3. `{"type": "act", "t": <int>, "obs": <observation>}` for every control step: answer `{"action": [dx, dy, dz, dyaw,
   grip, cams, turn]}` (seven numbers; `cams` an integer from 0 to 7).
4. `{"type": "close"}` at the end: answer `{"ok": true}` and exit.

The observation and the action are the ones in `datasheet.md` (section 2 and 7); images arrive as `{"dtype":
"uint8", "shape": [H, W, 3], "b64": <base64 of the raw RGB bytes, row-major>}`.

**The station does not wait for your controller.** Every control step your controller has **0.1 s** to reply,
measured on the grader's clock from the moment the observation is sent until your reply is read. A reply that takes
longer is applied when it arrives: before it, the station runs one *hold* step (no motion, the gripper as last
commanded, no images, no turnover request) for every whole 0.1 s the reply took, and your controller never sees those
steps' observations (the next observation's `t` skips them). A reply later than 5 s ends the shift. The grader runs
on a machine with 4 CPUs (x86-64, about the speed of a current server core) with your controller alone on them while
it thinks; the reference controller needs a few milliseconds per step on average and under 30 ms at its slowest. On top of
this, the station's own command latency of two control steps applies (`datasheet.md`, section 2).

Now and then the grader asks the same step `t` a second time with a slightly different observation: the observation
of the arm a few milliradians away (joint readings, hand pose and the images taken) and a slightly different wrench;
only the reply to the real observation is applied and only that reply is timed, and a controller whose replies never
depend on its observation scores nothing. Answer each request from the observation it carries; a repeated `t` does
not advance the shift.

If your controller crashes, misses the 5 s limit or answers something that is not seven finite numbers, that shift
ends and keeps what is seated in its kit at that moment; `run.sh` is started again for the next shift (it must print
`READY` again). After three restarts every remaining shift ends the same way.

## How it is graded

Your controller runs **12 hidden shifts** of the station (each with its own belt speed, part sequence and poses,
kit placement, lamp brightness and real camera mounts, drawn like the station's own; the practice shifts are drawn
the same way), one after another, with the same `run.sh` process unless it had to be restarted. Each shift has one
kit of six pockets and eight parts on the belt (one of each variant and two spares). The graded quantity is the
**fraction of pockets that the kit inspection finds seated** (the right part, right side up, key aligned, fully
down, undamaged; `datasheet.md` section 8) over all shifts (72 pockets), compared with what a reference controller
built from the same material seats.

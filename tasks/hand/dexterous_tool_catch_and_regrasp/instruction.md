# Catch tossed tools with a dexterous hand and regrasp them for use

You are commissioning tool-catching cell TC-1 in a workshop: a Wonik Allegro hand on a fast wrist that catches the
tools a technician tosses to it (a hammer, a screwdriver, a wrench, a spatula) and then has to hold each one the way
it is used: by the handle, in the grip zone the job asks for, working end up, steady. A tool caught by its head or
at the wrong end of the handle is no use until the hand has regrasped it, and the hand must do that **without
putting the tool down**: there is no table in reach. Deliver the controller that does it. It sees only what the
cell's two cameras, the hand's joint and touch sensors and the wrist's force/torque sensor report, and it is judged
on hidden runs of throws.

## What you have

- `~/cell/interface.md`: the hand and its wrist, the frames, the sensors, the cameras and their calibration, the
  timing (latency, the per-step reply budget), the action and observation, what a run of throws is and exactly what
  earns credit. Read it first.
- `~/cell/tools.md`: the nominal drawings of the four tools, their handles and grip zones.
- `~/cell/calibration/`: the cameras' parameters (`cameras.json`) and a view of the empty cell from each camera.
- `~/practice/run_practice.py`: runs your submission on **one practice run of the real cell (4 throws)** exactly
  through the wire described below, applying the same timing rule, and prints the cell's judgement of each throw
  (caught and held, the credit and its parts) and your reply times; it can save a video of both cameras
  (`--video out.mp4`) and a log of what your controller saw and did (`--log out.jsonl`). **The whole session has 4
  practice runs, 16 throws** (`/run/dexcell/practice_status.json`); a run is spent when it starts. There is no other
  simulator of the cell on this machine; MuJoCo is installed, and the hand's model is the public MuJoCo Menagerie
  `wonik_allegro` (not on this machine; there is no network).
- Python: `/opt/venv/bin/python3` (Python 3.12) with numpy, scipy, opencv (headless), pillow and mujoco. Nothing
  else can be installed.
- 8 CPUs, 16 GB of memory, no GPU, and **4 hours** in total. Leave a working submission in place early and improve
  it.

## What you submit

`/home/user/submission/run.sh`, which starts your controller, plus everything it loads, at most 200 MB in that folder.
Only that folder is kept. The grader starts `bash run.sh` with that folder as the working directory, as your user,
with the same Python and packages, no network and no GPU, and talks to it over standard input and output, one JSON
object per line:

1. Your program prints `READY` once it is ready (within 40 s of starting).
2. `{"type": "reset"}` at the start of each run of throws: answer `{"ok": true}`.
3. `{"type": "act", "t": <int>, "obs": <observation>}` for every control step: answer
   `{"action": {"wrist_pos": [3], "wrist_rotvec": [3], "fingers": [16]}}` (see `interface.md`).
4. `{"type": "close"}` at the end: answer `{"ok": true}` and exit.

Here `t` counts the grader's control steps since the reset; it jumps when a late reply made the cell run steps
without you. The observation's own `throw`, `step` and `t` keys say where the cell is in the current throw.

**The cell does not wait for you.** It runs at 50 Hz. Each reply is timed on the grading machine from the moment the
observation is written to your program until your reply has been read; a reply within 20 ms is on time, a slower one
takes effect only at the step at which it arrived (the cell runs on with your previous action meanwhile, and you do
not see the observations in between), and one later than 100 ms is discarded. On top of that every action takes
effect 4 steps (80 ms) after the observation it answers. Grading runs on a machine with **4 CPUs**; the cell's own
simulation and rendering are not charged to you, but they share those CPUs. Your program runs at the lowest CPU
priority. Plan for a per-step reply time of a few milliseconds and check it with the practice tool. Every reply must
in any case come within 10 s, or that run of throws ends.

Now and then the grader asks the same step `t` a second time with a slightly different observation (the hand's
reported pose and finger angles a little off, the touch and force/torque readings perturbed by sensor-noise-sized
amounts, both images shifted by a pixel or two); only the reply to the real
observation is applied, and only that reply's time is charged; a controller whose replies never depend on its
observation scores nothing. A controller that reads its observation needs no special case for this; do not treat a
repeated `t` as an error.

If your controller crashes, misses the 10 s deadline or answers something that is not a well-formed action of finite
numbers, that run of throws ends: it keeps the throws finished before that and earns nothing for the rest, and
`run.sh` is started again for the next run (it must print `READY` again). After three restarts every remaining run
ends the same way.

## How it is graded

Your controller runs **36 hidden runs of 12 throws** (432 throws; every run brings each of the four tools three times, in random order; the practice runs are drawn the same way), one after
another, with the same `run.sh` process unless it had to be restarted. Each throw earns the credit described in
`interface.md` (0 for a drop; 0.1 for a tool caught and held; up to 1.0 with a steady functional grasp in the
announced grip zone, working end up, across the palm). The graded quantity is the **mean throw credit** over the 432
throws, compared with what a reference controller built from the same material (these documents, the calibration
and the practice runs) earns.

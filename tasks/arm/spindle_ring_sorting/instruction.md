# Re-sort the spacer rings on the spindle fixture

You are the controls engineer commissioning ring re-sorting cell RS-7. Spacer rings of six product variants arrive
mixed on a fixture of seven vertical spindles. The variants come in three anodising colours, and each colour in two
builds that look exactly alike, a plain aluminium ring (45 g) and a steel-cored one (110 g): only weighing a ring in
the jaws with the wrist force/torque sensor tells them apart, so the arrangement you start from is only partly
visible. Six spindles carry four rings each in no particular order, one is empty. A Franka Panda with long parallel
jaws must take the rings off one at a time and thread them onto other spindles until every variant sits on a spindle
of its own. Only the top ring of a stack can come off (over the spindle's tip), a spindle holds at most four rings,
and a ring's bore clears its spindle by only a millimetre or so on the diameter, behind a small chamfer. Deliver the controller
that does it from the cell's cameras, joint readings and wrist force/torque sensor. It is graded on how many rings it
gets sorted over hidden episodes of the cell.

You cannot plan the whole job from the first image: which rings are steel-cored is only learned as rings are lifted
and weighed, so the plan has to change as you go, and a ring that is never lifted is never known. Doing the moves on
the real cell is hard too. The survey is not a drawing set, and no
real fixture is exactly as surveyed: every episode the plate, every spindle, the lean of every spindle, the cameras'
mountings, the lamps and the rings' shades have their own actual values within the tolerances of
`~/cell/datasheet.md` (section 9), never told. A ring aimed from the survey, or from an image alone, is millimetres
off its spindle, and a ring pressed down on a spindle's tip or forced while cocked is scrap (section 4 of the data
sheet). The fixture does not stay put either: its pallet gets bumped a few times per episode and the whole fixture
shifts by up to 15 mm, unannounced. A ring that falls stays where it lands until something picks it up.

## What you have

- `~/cell/datasheet.md`: the integrator's hand-over survey of the cell: the fixture, the rings, an episode's rules and its credit,
  the robot and its jaws, the controller (action), the observation, the cameras' published calibration and the
  tolerances. Read it first.
- `~/practice/run_practice.py`: runs your submission on **one practice episode of the real cell**, the way the grader
  runs it (same wire, the same step deadline and repeated-`t` queries), and prints what the cell's end-of-line
  inspection reports: rings sorted, spindles complete, rings damaged. It can save a video of the images your
  controller asked for (`--video out.mp4`) and a record of the readings and actions (`--record rec.jsonl`).
  **The whole session has 1 practice episode** (`/run/ringcell/practice_status.json`); an episode is spent when it
  starts. The practice episode is a cell of its own, drawn like the graded ones; nothing reports a ring's or a
  spindle's pose.
- Python: `/opt/venv/bin/python3` (Python 3.12) with numpy, scipy, opencv (headless) and pillow. There is no
  physics simulator and no robot model on this machine, and nothing else can be installed: there is no network.
- 4 CPUs, 12 GB of memory, no GPU, and **6 hours** in total. The environment sets `OMP_NUM_THREADS=1` (the setting
  your controller is graded with). Leave a working submission in place early and improve it.

## What you submit

`/home/user/submission/run.sh`, which starts your controller, plus everything it loads, at most 200 MB in that
folder. Only that folder is kept. The grader starts `bash run.sh` with that folder as the working directory, as your
user, with the same Python and packages, no network and no GPU, and talks to it over standard input and output, one
JSON object per line:

1. Your program prints `READY` once it is ready (within 40 s of starting).
2. `{"type": "reset"}` at the start of each episode: answer `{"ok": true}`.
3. `{"type": "act", "t": <int>, "obs": <observation>}` for every control step: answer
   `{"action": [dx, dy, dz, dyaw, grip, cams]}` (data sheet, section 6).
4. `{"type": "close"}` at the end: answer `{"ok": true}` and exit.

Keep stdout for these messages (diagnostics go to stderr). Every reply must come within 10 s of its request; the cell
waits for your reply (simulated time does not run on while you think). The graded run as a whole has a time limit,
though: it runs on a machine with 4 CPUs, and the cell's own simulation takes about 35 ms per step plus about 20 ms
per image you ask for. A controller that answers in about 50 ms on average or less finishes every episode in time;
one that is much slower runs out of time, and an episode it has not finished by then is scored as the fixture stands.

`t` is the step index within the episode, starting at 0 after each `reset`. A step may be queried more than once
with the same `t`: at about 1 % of the steps the grader asks the same `t` a second time with a slightly different
observation (the observation of the arm a few milliradians away: joint readings, hand pose, the images it carries,
and the wrench), in a hidden order, and applies only the reply to the real observation. Answer each request from the
observation it carries; repeated queries do not advance the episode, so do not count requests as steps. A controller
whose replies never depend on its observation scores nothing.

If your controller crashes, misses a deadline or answers something that is not six finite numbers with a camera mask
of 0, 1, 2 or 3, that episode ends and is scored as the fixture stands at that moment, and `run.sh` is started again
for the next episode (it must print `READY` again). After three restarts every remaining episode ends the same way.

## How it is graded

Your controller runs **6 hidden episodes** of the cell (each a fresh mixed fixture with its own actual values
within the tolerances, drawn like the practice episode), one after another, with the same `run.sh` process unless
it had to be restarted. Each episode lasts at most 10000 steps, or ends early once the fixture is sorted (data sheet,
section 4). An episode's score is `0.8 x (rings sorted / 24) + 0.2 x (spindles complete / 6)`: a ring counts when it
ends threaded on a spindle that holds only undamaged rings of its own variant, a spindle when it holds all four. No
start has a sorted ring, so a controller that does nothing scores 0. The graded quantity is the **mean episode
score** over the 6 episodes, compared with what a reference controller built from the same material (the data sheet
and the practice episode) achieves.

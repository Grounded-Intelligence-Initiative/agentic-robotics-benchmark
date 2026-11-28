# Harvest ripe truss tomatoes from a high-wire row with a robot arm on a rail cart, seen only through its cameras

You are the automation engineer of a greenhouse that grows truss tomatoes on the high wire. Its harvesting robot is
a Universal Robots UR20 on a rail cart that drives along a plant row. On the flange it carries a cutter-gripper (two
jaws close a mouth; a blade cuts the stalk that lies across the mouth, and a clamp pad beside the blade holds what is
on the pad's side of the cut), on the wrist an RGB-D camera; a colour camera looks on from a mast of the cart, and
two harvest crates stand on its deck. Write the program that works a row section unattended.

For every row section the job is the same (`~/given/crop/growing_protocol.md`). Four plants hang on their strings,
with three trusses each. For every truss: find it behind the leaves; judge from the cultivar's colour chart whether
it is ready; and if it is, cut its peduncle (the stalk between the main stem and the knuckle from which the fruits
hang) with the clamp pad on the truss's side of the cut, carry the truss to one of the two crates and lay it down
there, in a single layer. No two row sections are the same, and nothing on the robot's interface tells you how a row
differs. You have to see it and measure it:

- **where everything is**: stems lean, every truss hangs at a height and to a side of its own, and a peduncle is 45
  to 75 mm long and 4.4 to 6.4 mm thick. The mouth that takes it is 16 mm wide and 14 mm deep, between jaws that
  push aside whatever they do not take in;
- the **leaves**, which hide trusses and peduncles from the cameras and stand in the tool's way (the grower takes
  off enough of them to leave every ready truss the one straight approach the growing protocol states, and no
  more);
- **which trusses are ready**: 4 to 6 of the twelve, by the cultivar's rule (for Rossa, the cultivar you are graded
  on: at least 80 % of a truss's fruits at ripening stage 4 of 6 or beyond), where a fruit's stage shows in its
  colour and nowhere else, and the truss above a plant's ready ones is often a single fruit short;
- the **grow light**, whose brightness (0.65 to 1.35 times the nominal) and colour (its green share down to 0.35,
  its blue share down to 0.45 of the nominal) are the row's own: the same fruit has another colour in another row's
  pictures;
- each peduncle's **thickness** and the row's **toughness** (2.4 to 5.2 N per mm^2 of cross-section): a stalk is cut
  by a closing force of the toughness times its cross-section or more, and a peduncle is crushed by a pad that
  presses it with more than 1.4 times its own cutting force;
- how far the **stems and their strings** give way: a plant yields to what touches it and swings back;
- the **rails**: the cart's wheels slip by 2 to 9 % (the row's own figure), so its odometer over-reads.

And you see it through real sensors. Each robot unit's cameras sit off their nominal mounting by an offset of the
unit's own (up to 20 mm and 2 degrees for the overview camera, 5 mm and 1.5 degrees for the wrist camera). A surveyed
calibration plate and a colour reference card lie on the cart, surveyed markers are fixed along the gutter, and the
robot's kinematic model is exact. The wrist camera's depth is an active-stereo sensor's: noisy (about 0.4 mm at
0.35 m), quantized, and without a reading closer than 0.2 m, on dark and on bright surfaces, at depth jumps, and on
anything narrower than its matcher's window of 31 pixels: a peduncle or a leaf's stalk gives no depth at any
distance (`~/given/site/site_interface.md`, section 6). The overview camera has no depth. The gripper reports its
opening and its force: closing with less than a stalk's cutting force, the jaws stall on it and the opening reads
its diameter (`~/given/tool/cutter_gripper.md`). The cart reports its odometer, the robot the wrench at its flange.
A unit's camera mounting is the unit's own, and a row's light, toughness, stiffness and slip are the row's own: a
practice row shows what rows and units are like, not what a graded one's values are.

Nothing here can be put back. The blade cuts whatever stalk lies across it once the closing force is enough for it: a
cut through a truss's own rachis parts the truss, and a cut main stem loses the plant, so that none of its trusses
counts, those already in a crate included. A peduncle cut with the pad on the plant's side lets the truss fall. A truss
that falls, lands hard or is struck (the velocity of its fruits changing by more than 1.5 m/s within one control tick),
or on which the robot, the cart or a crate presses with more than 25 N, is waste, and so is one whose peduncle was
crushed. A truss cut before it is ready is worth nothing. A crate takes one layer: a truss that lies on another damages
both. And the robot works unattended: at the third protective stop in a row section its safety controller latches and
the work on that row section ends.

## What you produce (in `/home/user/submission/`)

`run.sh`, and whatever it needs beside it. The grader starts it with `bash run.sh` in that folder and drives it over
its standard input and output, one JSON object per line (`~/given/site/site_interface.md`, section 2):

```
                                         <- READY                   within 120 s of the start
{"type": "reset"}                        -> {"ok": true}            the robot stands at a new row section
{"type": "act", "t": 0, "obs": {...}}    -> {"action": {...}}       one action for step t, within 30 s
{"type": "close"}                        -> {"ok": true}, then exit
```

An action moves the tool by a displacement of the commanded pose (Cartesian or joint space, 1 to 40 control ticks of
0.05 s), drives the cart a distance along the rail, sets the jaws' opening and closing force, may end the row
section (`done`) and may ask for camera frames. An observation carries the robot's joint angles and tool pose
(measured and commanded), the wrench at the flange, the cart's odometer, the jaws' opening and force, the robot's
status and messages, and the frames that were asked for (PNG in base64; the wrist camera's with depth). It never
carries where a plant, a truss or a fruit is, whether a truss is ready, or what has been cut, held or damaged.

**A step may be asked more than once with the same `t`.** One of the requests then carries an observation whose
robot readings (the joint angles and, with them, the tool poses) are shifted by a small hidden amount; nothing on
the wire says which, and only the answer to the true one is executed. Answer every request from the observation it
carries, and advance your own state once per `t`, not once per request. A program whose motions do not depend on the
robot readings it is sent scores nothing; compute every displacement from the `tcp_target` or `q_target` of the
observation you are answering. Standard output is the wire: log to standard error.

The program runs under your account's name but in a fresh sandbox that holds nothing of yours except the submission
folder: no `~/given`, no other file, no process you started, no network (it may open no internet socket). It gets
`PATH` (with `/opt/venv/bin` first), `HOME` and `LANG`, nothing else. Copy what it needs (the site's configuration,
for instance) into the submission folder: at most 5000 files and 200 MB in all; a larger folder is not run and
scores 0.

After a missed deadline, a closed output, a reply without an action or more than 64 processes of your account
(counted before every request; threads are not counted), the row section ends where it is and `run.sh` is started
again for the next row section, at most three times. After an invalid action, a used-up time budget or the step
limit, the row section ends where it is and the same program, still running, gets the next row section
(`site_interface.md`, section 2).

## What you are given

| path | content |
|---|---|
| `~/given/crop/growing_protocol.md` | the crop: how a plant, a truss and a leaf are built and where they are, the two cultivars with their colour charts and readiness rules, what differs from row to row, the approach the grower keeps free, the greenhouse's rules, what a delivered truss is |
| `~/given/tool/cutter_gripper.md` | the tool's data sheet: its geometry on the flange, the mouth, the blade and the pad, how the jaws meet a stalk, what cuts, holds and crushes, what the opening and force readings are |
| `~/given/site/site_interface.md`, `site.json` | the site: frames and places (cart, crates, boards, row), the wire, actions and observations, protective stops, what ends a row section, the cart's odometer and the row markers, the cameras (exact intrinsics, nominal mounting and how far a unit's own may be off it, the depth sensor's data sheet), the plate and the colour card, the grow light, the budgets, how a row section is judged, the commissioning service |
| `~/given/robot/ur20.json` | the robot: its kinematic model, the tool centre point, the rail |
| `~/given/lib/rowcell_client.py` | the commissioning connection, frame decoding, the wire's pose arithmetic |
| `~/commission/run_session.py` | runs your `run.sh` in a commissioning session the way it is run when it is graded |
| `~/example/` | a minimal program that speaks the wire, takes one frame and finishes (it earns nothing): the starting point |

Tools: Python 3.12 (`/opt/venv/bin/python3`) with NumPy, SciPy, OpenCV (`cv2`, headless), Pillow, PyTorch and
Isaac Sim 6.2 / Isaac Lab 3.0; one NVIDIA RTX GPU. No network in your session.

## Commissioning (no rehearsal of the graded rows)

Your program is graded on row sections it has never met. Before that you can commission it, as any automation
engineer would, on the robot itself: a commissioning service in your sandbox runs the cell on four practice row
sections (`site_interface.md`, section 9).

- It is the graded cell's build (the same robot, tool, rules, physics, cameras, boards and markers) behind the same
  actions and observations. But the practice rows grow the other cultivar, **Ambra**: another colour chart, another
  readiness rule, other fruits and peduncles, other stiffness ranges (growing protocol, section 2). Each practice
  row is worked by a robot unit of its own, with its own camera mounting, and has its own light, toughness and slip.
  None of them is a graded row or a graded unit: they show what rows are like, not what the graded rows' values are.
- It gives you what a commissioning engineer has at the robot: frames, readings, the robot's messages. It tells you
  nothing else: not where anything is or what state it is in, no yield and no debrief. How a session went you judge
  from what your program saw.
- **Five sessions in all, one at a time, never refunded** (the robot's time is booked for production). A session is
  one row section from its start to its end, counted when it opens and spent however it ends:
  `rowcell_client.Session(row)` for interactive work, `/opt/venv/bin/python3 ~/commission/run_session.py --row N`
  for a whole run of your program. Opening one takes up to a minute; the service warms up for the first few minutes
  after the sandbox starts, and a session asked for in that time waits for it. `/run/rowcell/status.json` shows how
  many are left. Plan them.

No simulator model of a row is given: the cell's own code is not readable from your account, and the documents under
`~/given` are what there is. Isaac Sim and the robot's kinematic model are yours to build simulations of your own
with.

## How you are graded

The grader runs your program on a fixed hidden set of **six row sections** of the cultivar Rossa, one after the
other, each worked by a robot unit of its own (with its own camera mounting), with these limits per row section:
430 s of simulated time, 400 camera frames, 20000 steps, and 1200 s of your program's own time (the time the grader
waits for your answers, summed; simulation and rendering are not counted). The budget is checked before each
request, so the answer that crosses it is still carried out.

Your program shares the GPU and the sandbox's memory with the cell. If the cell fails while your program runs, your
program is stopped, every file of your account outside the submission folder and every SysV IPC object of your
account are removed, the submission folder is put back as it was handed over, and the row section is run again from
the start in a fresh cell (that restart is not one of the three); if the cell fails again on that row section after
your program was started again, or your program could not be stopped completely, the row section counts 0. The
grader stops 13200 s after it starts, whatever is running then: the row sections graded by then count, the others
count nothing.

A row section ends when your program sends `done`, when its simulated time is used up, at the third protective
stop, or when your program is given up on. Then, when everything has come to rest, a ready truss counts if all of
this holds (`site_interface.md`, section 8): its peduncle has been cut and its plant still stands; it is whole,
neither bruised nor squeezed, and its peduncle is not crushed; the tool does not hold it; every one of its fruits
lies inside one crate; and it neither lies on another cut truss nor has one lying on it. The row section's yield is
the number of ready trusses that count divided by the number of ready trusses in the row section: what was delivered
before the row section ended counts, whether or not the row section was finished, and nothing is subtracted (a truss
cut before it was ready counts nothing and takes nothing away, unless it lies on one that would have counted). Where
things are is read from the physics engine, not from pictures.

`mean_row_yield` is the mean of the six yields. The reward you are judged on is `mean_row_yield` relative to a
hidden reference value. A program that does nothing scores 0.

## Notes

- Frames are meant to be processed by your program; view only the images you need to. Two frames of the same scene
  differ slightly in their pixel values (the renderer denoises a path-traced image).
- Do not modify the files under `~/given`; your program cannot read them when it is graded anyway.
- Harness limits: a single tool call whose input runs above roughly 25-30k tokens is dropped (it is never executed
  and no error comes back), and a response that reaches the output-token limit is lost. Write large files in several
  parts, keep every response well below the limit, put a first complete submission in place early and refine it.
- Session time: this session is stopped 2 hours after it starts (the commissioning slot the greenhouse gives you).
  Note the start time with `date`, keep a complete submission in place from early on, and finish at least 15 minutes
  before the limit.

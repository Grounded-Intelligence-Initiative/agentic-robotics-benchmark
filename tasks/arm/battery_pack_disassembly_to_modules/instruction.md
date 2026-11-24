# Take a battery pack apart to module level with a robot arm, seen only through its cameras

You are the automation engineer of a battery recycling line. One of its cells takes plug-in hybrid battery packs
(the HVB-P series) apart to module level: a Universal Robots UR20 with a tool changer, a nutrunner with exchangeable
bits and a parallel gripper stands in front of a fixture that holds one pack; an RGB-D camera looks down on the
fixture and another sits on the robot's wrist. Write the program that runs this cell unattended.

For every pack the job is the same (`~/given/service_manual.md`, section 6): remove the lid's bolts and the lid,
pull the service disconnect and wait for the pack to discharge, pull the low-voltage plugs, remove the busbars and
the module brackets with their bolts, lift every module into the module rack, and sort every other component into
its bin. No two packs are the same, and nothing on the cell's interface tells you how a pack differs. You have to
see it:

- the **variant** (4, 5 or 6 modules; another lid, another layout), which is not marked on the pack;
- the pack's **placement** on the fixture (within 12 mm and 1.5 degrees) and every **fastener's position** (within
  2 mm of the manual's nominal one), while a bit engages only within 1.2 to 2 mm;
- every fastener's **head type** (hexagon or Torx, per fastener: the bit has to be chosen from the picture);
- **corroded** fasteners, which need more torque than the default allows, and **missing** ones;
- a **harness remnant** that may lie on the lid or inside the pack over fasteners and has to go first.

A wrong bit or a bit that is not square on a head wears the head until it is stripped. A tool that touches a busbar
before the pack is isolated, or a module that falls, ends the work on that pack.

## What you produce (in `/home/user/submission/`)

`run.sh`, and whatever it needs beside it. The grader starts it with `bash run.sh` in that folder and drives it over
its standard input and output, one JSON object per line (`~/given/cell/cell_interface.md`, section 2):

```
                                         <- READY                   within 120 s of the start
{"type": "reset"}                        -> {"ok": true}            a new pack is on the fixture
{"type": "act", "t": 0, "obs": {...}}    -> {"action": {...}}       one action for step t, within 30 s
{"type": "close"}                        -> {"ok": true}, then exit
```

An action moves the tool by a displacement of the commanded pose (Cartesian or joint space, 1 to 40 control ticks of
0.05 s), runs the nutrunner's spindle, sets the gripper's opening, gives a command (eject a fastener, exchange the
tool or the bit, `done`) and may ask for camera frames. An observation carries the robot's joint angles and tool
pose (measured and commanded), the wrench at the flange, the nutrunner's torque, speed, angle and stroke, the
gripper's opening and whether it holds something, the cell's status and messages, and the frames that were asked
for (RGB and depth, PNG in base64). It never carries where a part is or what state a fastener is in.

**A step may be asked more than once with the same `t`.** One of the requests then carries an observation whose
robot readings (the joint angles and, with them, the tool poses) are shifted by a small hidden amount; nothing on
the wire says which, and only the answer to the true one is executed. Answer every request from the observation it carries, and
advance your own state once per `t`, not once per request. A program whose motions do not depend on the robot
readings it is sent scores nothing; compute every displacement from the `tcp_target` or `q_target` of the
observation you are answering. Standard output is the wire: log to standard error.

The program runs under your account's name but in a fresh sandbox that holds nothing of yours except the submission
folder: no `~/given`, no other file, no process you started, no network (it may open no internet socket). It gets
`PATH` (with `/opt/venv/bin` first), `HOME` and `LANG`. Copy what it needs (the cell's configuration, for
instance) into the submission folder. After a missed deadline, a closed output or an invalid action the pack ends
where it is and `run.sh` is started again for the next pack, at most three times.

## What you are given

| path | content |
|---|---|
| `~/given/service_manual.md`, `~/given/manual/` | the pack series: variants, the pack frame, every component's and fastener's nominal place, grip features, what holds what, the fasteners' heads and torque table, the isolation procedure, the order of work, what a field pack may look like, where everything goes and what it is worth; drawings of the three variants, photographs of an example pack and of the fastener heads |
| `~/given/tool_catalog.md` | the tool changer, the nutrunner and its bits (what each bit does on each head, the engagement tolerances, the forces), the gripper (how a grip works) |
| `~/given/cell/cell_interface.md`, `cell.json` | the cell: frames and places (fixture, bins, rack, lid stand, tool stand), the wire, actions and observations, protective stops, what ends a pack, the cameras with their exact calibration, the budgets, how a pack is judged, the commissioning service |
| `~/given/robot/ur20.json`, `~/given/robot/ur20_usd/` | the robot: its kinematic model and its USD description |
| `~/given/lib/packcell_client.py` | the commissioning connection, frame decoding, the wire's pose arithmetic |
| `~/commission/run_session.py` | runs your `run.sh` in a commissioning session the way it is run when it is graded |
| `~/example/` | a minimal program that speaks the wire, takes one frame and finishes (it earns nothing): the starting point |

Tools: Python 3.12 (`/opt/venv/bin/python3`) with NumPy, SciPy, OpenCV (`cv2`, headless), Pillow, PyTorch and
Isaac Sim 6.2 / Isaac Lab 3.0; one NVIDIA RTX GPU. No network in your session.

## Commissioning (no rehearsal of the graded packs)

Your program is graded on packs it has never met. Before that you can commission it, as any automation engineer
would, on the cell itself: a commissioning service in your sandbox runs the cell on four development packs of the
series (`cell_interface.md`, section 8).

- It is the graded cell (the same robot, tools, rules, physics and cameras) behind the same actions and
  observations. The development packs have their own variants, placements, heads, corrosion, missing fasteners and
  harness remnants; none of them is a graded pack.
- It gives you what a commissioning engineer has at the cell: frames, readings, the cell's messages. It gives no
  ground truth, no credit and no debrief. How a session went you judge from what your program saw.
- **Ten sessions in all, one at a time.** A session is one pack from its arrival to its end, counted when it opens:
  `packcell_client.Session(pack)` for interactive work, `/opt/venv/bin/python3 ~/commission/run_session.py --pack N`
  for a whole run of your program. Opening one takes one to two minutes (the first one several more).
  `/run/packcell/status.json` shows how many are left. Plan them.

The cell's own code is not readable from your account. Isaac Sim and the robot's model are yours to build
simulations of your own with.

## How you are graded

The grader runs your program on a fixed hidden set of **six packs**, one after the other, each in its own cell,
with these limits per pack: 2700 s of simulated time, 400 camera frames, 20000 steps, and 1200 s of your program's
own time (the time the grader waits for your answers, summed; simulation and rendering are not counted).

A pack ends when your program sends `done`, when its simulated time is used up, when a tool touches a live part
before isolation, when a module falls, or when your program is given up on. Then every component of the pack counts
if it lies undamaged where it belongs (service manual, section 9; `cell_interface.md`, section 7): a module standing
in a bay of the rack (value 8), the lid on its stand (5), the service disconnect in the electronics bin (3), a
busbar in the copper bin (2), a bracket in the steel bin (1), a low-voltage plug or the harness remnant in the
electronics bin (1 each), a fastener in a fasteners bin (0.5). The pack's credit is the value that counts divided
by the value of everything in the pack; nothing is subtracted for anything, and what was sorted before a pack ended
early still counts. Where things are is read from the physics engine, not from pictures.

`mean_pack_credit` is the mean of the six credits. The reward you are judged on is `mean_pack_credit` relative to a
hidden reference value. A program that does nothing scores 0.

## Notes

- Frames are meant to be processed by your program; view only the images you need to. Two frames of the same scene
  differ slightly in their pixel values (the renderer denoises a path-traced image).
- Do not modify the files under `~/given`; your program cannot read them when it is graded anyway.
- Harness limits: a single tool call whose input runs above roughly 25-30k tokens is dropped (it is never executed
  and no error comes back), and a response that reaches the output-token limit is lost. Write large files in several
  parts, keep every response well below the limit, put a first complete submission in place early and refine it.
- Session time: this session is stopped 6 hours after it starts. Note the start time with `date`, keep a complete
  submission in place from early on, and finish at least 15 minutes before the limit.

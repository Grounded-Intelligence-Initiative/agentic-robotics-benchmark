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
  2 mm of the manual's nominal one), while a bit engages only within 0.5 to 1.2 mm, and a bit that is on a head
  without driving it strips the head the moment it is loaded (no reading warns first);
- every fastener's **head type** (hexagon or Torx, per fastener);
- **corroded** fasteners, which need more torque than a clean one's breakaway, and **missing** ones;
- **seized** fasteners (up to four a pack, clean-looking or rusty) that no torque the shank survives turns, and
  **weakened shanks**: corrosion has eaten into every corroded and seized fastener's shank by an amount nothing on the
  outside shows (its shear torque 70 to 100 % of the thread's). A bit that holds a shank's shear torque twists it off
  and the stub keeps its part for good. A shank's yield point shows in the nutrunner's torque and angle readings,
  which are a real transducer's, noisy and calibrated per cell; a seized fastener comes loose only by working
  (`service_manual.md`, section 5);
- **foam pads** glued over some lid and busbar fasteners, which hide the head (or the empty hole) from both cameras;
  the bit goes through them;
- a **swollen** module (a bulge of 0.45 to 0.8 mm on its top, where a healthy module's top is crowned by up to
  0.33 mm and the criterion is 0.4 mm), which must stay in its seat: lifted or pushed 5 mm it vents;
- **sealant** dabbed over part of some heads, which hides part of the head from the cameras;
- the **look** of the pack: grime, dust and labels on the lid, dull heads, and the hall's light (brightness and colour);
- a **harness remnant** that may lie on the lid or inside the pack over fasteners and has to go first.

And you see it through real sensors. Every cell's cameras sit off their nominal mounting by an offset of their own
(up to 15 mm and 1.5 degrees overhead, 4 mm and 1 degree on the wrist): through the nominal poses a point is
centimetres off. Two surveyed calibration plates lie on the fixture, and the robot's kinematic model is exact. The
depth images are an active-stereo sensor's: noisy (about 2.5 mm at the fixture from the overhead camera, 0.3 mm from
the wrist camera close up), quantized, with no reading on dark and shiny surfaces, at edges and in the stereo pair's
shadows (`cell_interface.md`, section 5). Every cell's nutrunner has a torque transducer of its own: it reads a
gain of 0.88 to 1.12 times the torque plus an offset of -0.3 to 0.3 N m, and the spindle stops where that reading
meets the limit you command (`tool_catalog.md`, section 2). A cell's camera mounting and transducer are its own: a
development cell shows what the series of cells is like, not what a graded cell's values are.

A bit one size off (a 13 mm socket on a 10 mm hexagon, a T30 bit in a T40 recess), the extractor on a sound head, a
bit that is on a head but not square on it, or a Torx bit not pressed hard enough for the torque it holds strips the
head at once when it is loaded (a torque limit of 1 N m or more); a stall above the head's limit wears it through in
half a second; any other bit turns freely on it and harms nothing. A reamed Torx recess cannot be extracted: its part
stays held. A freed fastener rides out on the magnetic bit only when it is drawn straight out of its hole first and
carried gently (`service_manual.md`, section 5); one that falls onto a busbar shorts it.

Some mistakes cannot be undone, and the facility treats four of them as **safety events**: a tool touching a live
part before isolation, a loose fastener on a busbar, a module that falls, a swollen module moved off its seat. Each
ends the pack and **condemns** it: the pack goes to quarantine whole, with everything already sorted from it, and
counts 0 (`service_manual.md`, section 9). The cell also runs unattended: at the third protective stop in a pack its
safety controller latches and the work on that pack ends.

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
`PATH` (with `/opt/venv/bin` first), `HOME` and `LANG`, nothing else (`cell_interface.md`, section 9). Copy what it
needs (the cell's configuration, for instance) into the submission folder: at most 5000 files and 200 MB in all; a
larger folder is not run and scores 0.

After a missed deadline, a closed output, a reply without an action or more than 64 processes of your account
(counted before every request; threads are not counted), the pack ends where it is and `run.sh` is started again for
the next pack, at most three times. After an invalid action, a used-up time budget or the step limit, the pack ends
where it is and the same program, still running, gets the next pack (`cell_interface.md`, section 2).

## What you are given

| path | content |
|---|---|
| `~/given/service_manual.md`, `~/given/manual/` | the pack series: variants, the pack frame, every component's and fastener's nominal place, grip features, what holds what, the fasteners' heads and torque table, the isolation procedure, the order of work, what a field pack may look like, where everything goes and what it is worth; drawings of the three variants, photographs of an example pack and of the fastener heads |
| `~/given/tool_catalog.md` | the tool changer, the nutrunner and its bits (what each bit does on each head, the engagement tolerances, the forces), the gripper (how a grip works) |
| `~/given/cell/cell_interface.md`, `cell.json` | the cell: frames and places (fixture, bins, rack, lid stand, tool stand, calibration plates), the wire, actions and observations, protective stops, what ends a pack, the cameras (exact intrinsics, nominal mounting and how far a cell's own may be off it, the depth sensors' data sheets), the budgets, how a pack is judged, the commissioning service |
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

- It is the graded cell's build (the same robot, tools, rules, physics, cameras and calibration plates) behind the
  same actions and observations. The development packs have their own variants, placements, heads, corrosion,
  missing fasteners and harness remnants, and each is taken apart in a cell of its own, with its own camera mounting;
  none of them is a graded pack or a graded cell. They differ from the graded packs on purpose: each shows the
  series' conditions, but none carries them in the combinations and numbers a graded pack does (more seized and
  weakened fasteners, padded lid bolts, a swollen module beside them).
- It gives you what a commissioning engineer has at the cell: frames, readings, the cell's messages. It gives no
  ground truth, no credit and no debrief. How a session went you judge from what your program saw.
- **Three sessions in all, one at a time, never refunded** (fewer than there are development packs: the cell's time
  is booked for production). A session is one pack from its arrival to its end, counted when it opens, and spent
  however it ends: `packcell_client.Session(pack)` for interactive work, `/opt/venv/bin/python3
  ~/commission/run_session.py --pack N` for a whole run of your program. Opening one takes one to two minutes (the
  first one several more). `/run/packcell/status.json` shows how many are left. Plan them.

The cell's own code is not readable from your account. Isaac Sim and the robot's model are yours to build
simulations of your own with.

## How you are graded

The grader runs your program on a fixed hidden set of **six packs**, one after the other, each in its own cell (with
its own camera mounting), with these limits per pack: 1500 s of simulated time, 400 camera frames, 20000 steps, and 1200 s of your program's
own time (the time the grader waits for your answers, summed; simulation and rendering are not counted). The budget
is checked before each request, so the answer that crosses it is still carried out.

Your program shares the GPU and the sandbox's memory with the cell, which takes about 4 GB of GPU memory and 6 GB of
memory: keep your program below 12 GB of GPU memory and 16 GB of memory. If the cell fails while your program runs,
your program is stopped, every file of your account outside the submission folder and every SysV IPC object of your
account are removed, the submission folder is put back as it was handed over, and the pack is run again from the
start in a fresh cell (that restart is not one of the three); if the cell fails again on that pack after your
program was started again, or your program could not be stopped completely, the pack counts 0; and if the processes
of your account cannot all be stopped when grading ends, the run scores 0. The grader stops 20,400 s after it
starts, whatever is running then: the packs graded by then count and the others count 0 (six packs at every limit
above take about 12,500 s).

A pack ends when your program sends `done`, when its simulated time is used up, at the third protective stop, at a
safety event (a tool on a live part before isolation, a loose fastener on a busbar, a module that falls, a swollen
module moved off its seat), or when your program is given up on. Then every component of the pack counts if it lies
undamaged where it belongs (service manual, section 9;
`cell_interface.md`, section 7): a module standing in a bay of the rack, or a swollen one freed and left in its seat
(value 8), the lid on its stand (5), the service disconnect in the electronics bin (3), a busbar in the copper bin
(2), a bracket in the steel bin (1), a low-voltage plug or the harness remnant in the electronics bin (1 each), a
fastener in a fasteners bin (0.5). A condemned pack counts 0. Every other pack's credit is the value that counts
divided by the value of everything in the pack: what was sorted before the pack ended counts, whether or not the
pack was taken apart completely, and nothing is subtracted. Where things are is read from the physics engine, not
from pictures.

`mean_pack_credit` is the mean of the six credits. The reward you are judged on is `mean_pack_credit` relative to a
hidden reference value. A program that does nothing scores 0.

## Notes

- Frames are meant to be processed by your program; view only the images you need to. Two frames of the same scene
  differ slightly in their pixel values (the renderer denoises a path-traced image).
- Do not modify the files under `~/given`; your program cannot read them when it is graded anyway.
- Harness limits: a single tool call whose input runs above roughly 25-30k tokens is dropped (it is never executed
  and no error comes back), and a response that reaches the output-token limit is lost. Write large files in several
  parts, keep every response well below the limit, put a first complete submission in place early and refine it.
- Session time: this session is stopped 2 hours after it starts (the commissioning slot the line gives you). Note
  the start time with `date`, keep a complete submission in place from early on, and finish at least 15 minutes
  before the limit.

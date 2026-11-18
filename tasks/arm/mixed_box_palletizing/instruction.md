# Palletize a mixed order

A 4-axis palletizing arm with a vacuum gripper stands between an infeed conveyor, a pallet and a small buffer
table. Cartons of different sizes and weights arrive on the conveyor one at a time, in an order nobody announces.
The arm has to build one pallet from them that survives transport: stable, inside the pallet's edges and height
limit, with no carton crushed, dropped or knocked. The cartons are made to tile the pallet: an order fits only
when they stand a few millimetres apart. And the cell is never exactly its drawing: the arm's joint zeros, the
cameras' mounts and the place where the forklift set the pallet down differ in every order, and nobody reports
them.

Deliver a controller that does this from camera images, joint readings and the wrist force/torque sensor. You
have **3 hours** of wall time for the whole job; have your controller in `/home/user/submission/` well before the
time is up. A session still running at the limit is cut off.

There is no simulator of the cell on this machine. You have its description, reference images of the marks, the
arm's kinematics, and a **lab**: the cell itself behind a socket, for three practice orders in total.

## What is in your home directory

| Path | What it is |
|---|---|
| `task_card.md` | The cell: frames, layout, arm, gripper, cameras, cartons, marks, rules and numbers. |
| `arm_kinematics.py` | The arm's forward and inverse kinematics as drawn, as code. |
| `marks/` | Reference images of the six handling marks as a supplier's specification shows them: `max5.png`, `max10.png`, `max20.png`, `max40.png` (stacking limit by mass), `nostack.png` (do not stack), `fragile.png`. |
| `labclient.py` | Client for the lab (library and command line; `python3 labclient.py --help`). |
| `helpers/wire_skeleton.py` | Optional boilerplate for the grading wire. |

Run Python as `/opt/venv/bin/python3` (Python 3.12 with NumPy, SciPy, OpenCV, Pillow and MuJoCo 3.16; it is first
on `PATH`). The sandbox has 4 CPUs and 8 GB of memory, while grading too.

## The job

An order is a sequence of cartons. A carton appears in the pick station at the end of the conveyor, skewed and a
little off centre; the next one arrives 2 s after the station is clear. For each carton the controller decides
where it goes: onto the pallet, or onto the buffer table to be placed later (the table is just a surface; what
stands on it at the end of the order does not ship). The order ends when the controller says so (`done`) or when
the step allowance runs out (400 control steps per carton of the order plus 400; the observation counts them down).

Everything the controller knows about a carton it has to sense:

- **pose and size** from the cameras (three fixed RGB-D cameras: above the pick station, above the pallet, above
  the buffer table);
- **handling marks**, printed on the carton's top face, anywhere on it beside the sealing tape and turned by any
  multiple of 90 degrees: a stacking limit by mass (`5`, `10`, `20` or `40` kg), "do not stack" and "fragile".
  A carton carries at most one stacking mark; one without a stacking mark carries 80 kg. Print differs from
  carton to carton as it does between suppliers and print runs: the marks' size (38 to 85 mm on a side), the ink
  (black to dark grey, brown or blue), how strongly it took, and worn patches. Shipping labels and brand text are
  printed too and mean nothing;
- **mass** from the wrist force/torque sensor, once the carton hangs on the gripper;
- **centre of gravity**: up to 0.12 m off the centre of the top face, and nothing printed says where. The torque
  a hanging carton puts on the wrist does.

Nothing in the observation names the carton, its mass, its marks or where anything stands.

## What differs from the drawing

`task_card.md` is the cell as drawn. The cell as built differs from it, anew in every order, in the lab and when
graded, and nothing in the observation says by how much:

- **Joint zeros.** Each joint's encoder reads its angle with a constant error: up to 0.5 degrees at the base yaw,
  0.3 at the shoulder, 0.3 at the elbow, 1.0 at the tool yaw. `q` and `tcp` are the readings, and the arm's own
  controller works in the readings too: your velocity command moves a target in the frame of the readings. So
  `tcp` is not where the pad is: at full reach 0.5 degrees is about 20 mm.
- **Camera mounts.** Each camera hangs up to 30 mm (per axis) from its drawn position and points up to 1.5
  degrees off its drawn direction. The intrinsics are exact.
- **The pallet.** Its centre is up to 25 mm (per axis) from the drawn place and it is turned by up to 2 degrees.
  The shipping limits are the real pallet's edges.

Exact, in every order: the link lengths, the conveyor, the buffer table, twelve survey marks painted on the
conveyor, the table and the floor around the pallet, and a mark on top of the tool (all in `task_card.md`).

## What ships

At the end of an order the gripper lets go, the cell settles for 2 s, and the pallet goes through a tilt test:
gravity is tilted by 10 degrees toward +x, -x, +y and -y in turn (1 s up, 2 s held, 1 s down each). A carton
**ships** when, after the test, all of these hold:

- it stands upright (its top face up, level within 3 degrees) on the pallet's stack: no part of it more than
  5 mm beyond the real pallet's edge, none below the deck, none higher than 1.00 m above the deck;
- during the tilt test its centre moved at most 30 mm and it turned at most 5 degrees;
- it was never **damaged**, at any time from its arrival to the end of the test:
  - *shock*: the change of its velocity over any 20 ms window, divided by 20 ms, never exceeded 60 m/s^2
    (20 m/s^2 for a carton marked fragile). A free drop of about 70 mm (8 mm for fragile) onto something rigid
    reaches the limit;
  - *stacking load*: the weight resting on its top face from cartons above it, low-pass filtered with a 0.5 s
    time constant, never exceeded its limit: the printed number of kilograms, 0.5 kg for "do not stack", 80 kg
    for a carton without a stacking mark. Pressing a held carton down onto another one counts as load.

An order **ships complete** when every one of its cartons ships. An order with even one carton that does not
ship (left on the floor, on the buffer table or on the conveyor, damaged, or moved in the tilt test) is a short
shipment and counts as nothing. Your result is the fraction of the graded orders that ship complete. Every
graded order, and every lab order, can be shipped complete: the reference controller has done it, deciding about
each carton when it arrives as yours must. The reference is told what yours has to measure (the joint zeros, the
pallet's real pose, and each arrived carton's true pose, size, mass, centre of gravity and limits), and your
result is measured against it.

The vacuum seal has limits of its own (`task_card.md`): it holds only so much pull and so much moment, less when
the pad covers the carton's top only partly. A heavy carton held far from its centre of gravity hangs on most of
what the seal holds while it is at rest, and falls off when it is moved.

## Observation and action

The controller runs at 10 Hz. Observation (a JSON object):

| Key | Meaning |
|---|---|
| `t` | step index within the order |
| `q`, `dq` | the four joint angles and velocities (base yaw, shoulder, elbow, tool yaw) |
| `tcp` | `[x, y, z, yaw]`: where the drawing's kinematics put the centre of the gripper pad's face (m) and the tool's yaw (rad) for the readings `q`, with the cup's spring extended. Off by the joint zeros |
| `wrench` | `[Fx, Fy, Fz, Tx, Ty, Tz]`: force (N) and torque (N m, about the pad-face centre) that the cup and whatever hangs on it exert on the arm's flange, in world axes, noisy. A mass m hanging at rest reads `Fz = -9.81 m`; the empty cup reads about -9.81 N; pressing the pad onto a surface reads a positive `Fz` |
| `vacuum`, `sealed` | the vacuum command, and whether the pad is sealed on a carton |
| `in_reach` | 0 when the last command asked for a pose the arm cannot take (it then holds its target) |
| `infeed_present` | the station's photo-eye: a carton rests in the pick station |
| `delivered`, `total` | cartons delivered to the station so far, and cartons in the order |
| `steps_left` | control steps left in the order |
| `images` | for `infeed`, `pallet`, `buffer`: `rgb` (uint8, H x W x 3) and `depth` (uint8, H x W x 2: the low and the high byte of the depth in millimetres along the optical axis, 0 = no reading), each `{"dtype": "uint8", "shape": [...], "b64": <raw row-major bytes, base64>}`. An image is empty (H = W = 0) unless the previous action asked for its camera; the first observation of an order carries all three |

The nesting, with the numbers of one observation shortened (here the previous action asked for the infeed camera
only, so the other two images are empty):

```json
{"t": 37, "q": [0.61, 0.12, 0.44, -0.61], "dq": [0.0, 0.0, 0.0, 0.0], "tcp": [1.21, 0.85, 1.02, 0.0],
 "wrench": [0.1, -0.2, -9.8, 0.0, 0.0, 0.0], "vacuum": 0, "sealed": 0, "in_reach": 1, "infeed_present": 1,
 "delivered": 1, "total": 18, "steps_left": 7563,
 "images": {
   "infeed": {"rgb":   {"dtype": "uint8", "shape": [600, 800, 3], "b64": "..."},
              "depth": {"dtype": "uint8", "shape": [600, 800, 2], "b64": "..."}},
   "pallet": {"rgb":   {"dtype": "uint8", "shape": [0, 0, 3], "b64": ""},
              "depth": {"dtype": "uint8", "shape": [0, 0, 2], "b64": ""}},
   "buffer": {"rgb":   {"dtype": "uint8", "shape": [0, 0, 3], "b64": ""},
              "depth": {"dtype": "uint8", "shape": [0, 0, 2], "b64": ""}}}}
```

The lab's replies carry the same object (plus `result`); `labclient.py` returns it as it is.

The lighting (direction and strength of the main light) and a colour cast of up to 10 % per channel change from
order to order. Depth is noisy (about 1.5 mm per metre of distance) and about 0.5 % of its pixels read 0.

Action: a list of 7 numbers `[vx, vy, vz, wyaw, vacuum, cams, done]`:

- `vx, vy, vz`: velocity of `tcp`, the pad-face centre as the readings have it (m/s, each clipped to 1.2); `wyaw`: tool yaw rate
  (rad/s, clipped to 2.5). Inside the step the commanded velocity ramps linearly from your previous command to this
  one, so the commanded acceleration is the change of your command divided by 0.1 s; the joints servo to the moving
  target. The target is kept between 0.80 m and 2.15 m from the base axis and between 0.10 m and 2.10 m high;
- `vacuum`: 1 on, 0 off. A seal that broke is re-armed by switching the vacuum off;
- `cams`: which cameras the next observation carries (1 infeed, 2 pallet, 4 buffer, added up): an integer 0 to 7;
- `done`: 1 ends the order after this step.

## The lab

`labclient.py` talks to a service on `/run/ale-lab/lab.sock` (JSON lines). The lab is the cell, with twelve
practice kinds of carton and three practice orders, `p0`, `p1` and `p2` (16, 18 and 18 cartons). It serves `hello`,
`budget`, `reset`, `step` and `close`. A **trial** is one order: `reset` with the order's id starts it and charges
one trial; you have **3 trials in total**, never refunded. `step` with an action returns the next observation
(steps are free) until the order ends: when you send `done`, or when its step allowance runs out. Every trial
draws fresh printing, arrival poses, lighting, sensor noise, joint zeros, camera mounts and pallet pose. A trial also ends when you send another `reset` or
close the connection, after 3600 s of wall time or after 900 s without a request. The lab serves at most two
connections at a time.

The lab's observation is the observation of the grading wire, with one more key, `result`: -1 while the order
runs and, in the reply to the step that ends it, the fraction of the order's carton volume that shipped after the
tilt test (1.0 means the order shipped complete). The lab
tells nothing else: no carton's identity, mass, marks or state, and not which cartons shipped. The lab is not
there when your controller is graded.

## The scored orders

Eight hidden orders of 16 to 20 cartons each. Same as in the lab: the cell, the arm, the gripper, the cameras, the
sensor noise, the physics, the marks, every rule above, and how far the cell may differ from its drawing.
Different: the cartons (a hidden catalogue of 14 kinds, none of them a practice carton), the orders and the seeds.

All cartons, practice and hidden, are modular: made to tile a 1200 mm x 800 mm pallet. Their nominal footprints
are 600 x 400, 600 x 300, 600 x 200, 400 x 400, 400 x 300, 400 x 200 and 300 x 200 mm; a real carton is 2 to 4 mm
under nominal in length and in width (the same for all cartons of a kind). Heights are 0.16 to 0.32 m, masses 1.5
to 25 kg, the centre of gravity up to 0.12 m off the centre of the top face. An order's cartons add up to between
0.48 and 0.63 m^3 (the stack may be 1.00 m high, so up to two thirds of what the pallet can hold).

The practice orders and the hidden orders were chosen the same way: drawn at random, and kept when the reference
controller shipped them complete. It leaves 4 mm between neighbouring cartons. A planner that leaves 8 mm finds
room for every carton in none of them: three cartons of 397 mm are 1191 mm, and the pallet is 1200 mm.

## Deliverable and grading

Leave a controller in `/home/user/submission/` with a `run.sh` (started as `bash run.sh` with that folder as
working directory, as your user, with no network). Everything it needs must be inside that folder: nothing else
of your home is there when it is graded (copy `arm_kinematics.py` and `marks/` in if you use them); the image's
Python and its packages are. Protocol, one JSON object per line on stdin/stdout (stdout carries only protocol messages; use
stderr for logs): print `READY` within 120 s of the start; then answer `{"type": "reset"}` with `{"ok": true}`,
`{"type": "act", "t": T, "obs": {...}}` with `{"action": [7 numbers]}` within 30 s, and `{"type": "close"}` with
`{"ok": true}` before exiting. `reset` starts a new order.

At about 1 % of the steps the grader asks the same `t` twice, once with the real observation and once with a
slightly perturbed copy, in a hidden order, and applies only the reply to the real one. The copy differs in `q`
(each joint by up to 0.001 rad), in `tcp` (recomputed from it) and in the wrench's force (up to 0.3 N per axis);
everything else, the images included, is the same. Answer every request from the observation it carries; a
repeated `t` does not advance the order. A pair in which both replies command no motion (the first four numbers
all zero) is not counted; if fewer than 10 % of the other pairs get two replies that differ (by more than 1e-4 in
one of the first four numbers), the whole run scores 0: a controller whose motion commands ignore the joint
readings, or that answers a repeated `t` from a cache, fails.

The grader runs the eight orders in a hidden sequence, each with a fixed seed (the same for every submission). An
order in which the controller crashes, answers late or sends a malformed action, or in which the simulation
becomes unstable, scores 0. After a crash, a late answer or a broken protocol the controller is started again
before the next order, at most 3 times; the order that would need a fourth start and every later one score 0. A
controller that does not print `READY` at its first start scores 0 for the whole run. The whole grading run must
finish within 5400 s of wall time, simulator, rendering and your controller together; the order under way when it
runs out and the orders after it score 0. For scale: on the task's 4 CPUs the cell needs roughly 5 ms per control
step without images, and one camera's colour and depth images add roughly 0.1 s; a full run of the eight orders is
about 40,000 to 60,000 control steps.

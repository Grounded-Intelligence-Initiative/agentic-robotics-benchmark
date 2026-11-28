# Build a wooden toy car from its picture manual

A Franka Panda stands at a wooden table. Loose on the table lie the parts of a children's wooden toy car kit:

- a red chassis board with two notches across its top;
- two birch axle dowels;
- four black wheels;
- a blue cabin block.

The box also held parts of a sibling model of the same toy family. Each of those part types is there as the
sibling's full set:

| Sibling part | How many |
|---|---|
| axles | 2 |
| wheels | 4 |
| cabin | 1 |

So the table holds the same number of kit parts and sibling parts of each such type. They have the same paint and
fit the same notches, axle ends and seat, but their sizes differ.

**Every episode is a different car.** The toy family makes many cars of the same build. Each episode draws its own
car, with its own chassis, axle, wheel and cabin sizes, and its own sibling sets. The car's numbers are not written
anywhere. The only description of the episode's car is its picture manual: six pictures without text, each with a
millimetre ruler drawn to scale. Your controller receives that manual with the episode's first observation (t = 0)
and never again, so it has to read the manual at run time.

Write a controller that, in every episode, reads that episode's manual and builds its car on the cell's assembly
stand from what the cell's two cameras see. Use only the kit's own parts. When the controller is done, the car is
pushed along the table and has to roll.

## What you deliver

Leave `/home/user/submission/run.sh`. The grader starts it as `bash run.sh`, as your user, with the working directory
`/home/user/submission`, and talks to it over stdin/stdout (see "The wire").

Everything it needs must be inside `/home/user/submission`, because only that folder is carried into a fresh grading
machine. That machine runs the same image as yours, with the Python environment at `/opt/venv` (numpy, scipy,
pillow, mujoco), and has no network and no practice cell.

## What you have

- **The manual** (it reaches your controller at t = 0, see "Observation"; examples below): six pictures,
  `step_0_parts` to `step_5_done`.
  - `step_0` shows every part of the kit in one row, seen from above at true scale.
  - Steps 1 to 4 each show the assembly after the step in a three-quarter view. That step's parts are in full
    colour and earlier parts are faded.
  - Beside it is an orthographic view of the same assembly, drawn at the ruler's scale. It is from above for steps
    1 and 2, from the front (along the car's length) for step 3, and from the side for step 4.
  - `step_5` shows the finished car, with an orthographic view from above.
  - Ruler ticks are 1 mm apart, with longer ticks every 5 mm and 10 mm. Within one picture the ruler and the
    orthographic view share one scale.
- **`~/examples/car_1` … `car_6`**: six cars of the family. Each holds `kit.json` (the car's numbers, see "What a
  kit is made of") and `manual/step_*.png` (its manual, exactly as the controller receives it at t = 0). Use them to
  check how you read a manual. Your practice episodes build `car_1` to `car_3`; graded episodes draw other cars of
  the same family.
- **The practice cell**: the real cell, unseen, behind the unix socket `/run/toycell/practice.sock`.
  - Its episodes are graded episodes in every respect but the seed: each builds its own car (practice episode k
    builds the car of `~/examples/car_k`) on a fixed practice layout, with sibling parts mixed in. The
    car's manual comes with the observation at t = 0.
  - The grader's deadlines hold too: each request must come within 30 s of the previous reply, and the time
    between replies and requests adds up to at most 600 s per episode. Otherwise the episode ends with an error.
  - You get **3 practice episodes for the whole session**. `/run/toycell/practice_status.json` shows how many are
    left. An episode is spent as soon as it starts.
  - `~/practice/run_practice.py` runs your `submission/run.sh` through one practice episode over the real wire,
    with the grader's deadlines, and prints the cell's counts at the end. `--frames DIR` saves the camera images.
  - You may also drive the socket yourself. It takes one JSON line per request and answers one JSON line:

    | Request | Reply |
    |---|---|
    | `{"method": "reset"}` | `{"result": {"obs": ..., "episodes_left": n}}` |
    | `{"method": "step", "params": {"action": [...]}}` | `{"result": {"obs": ..., "done": bool}}` |
    | `{"method": "close"}` | `{"result": {"steps", "mates_with_kit_parts", "rolls"}}` |

    Errors come back as `{"error": ...}`. One episode per connection, under the deadlines above.
  - The cell answers nothing else: no poses and no latch events.

## The cell (world frame, metres, radians)

**Table, robot and stand**

- The table top is at z = 0.80. The Panda base is at (-0.56, 0, 0.80), facing +x.
- The assembly stand is a post with a dark grey square foot (90 mm) and a light grey top plate (60 × 32 mm), 0.10 m
  above the table.
  - Its nominal place is the top centre at (0.06, 0.00, 0.90), with the plate's long side along world x.
  - Every episode, practice episodes included, puts it somewhere else: up to 25 mm off in x and in y, and turned
    by up to 8° about the vertical. Find it.
  - The chassis goes onto it with its bottom face on the plate, centred, and its long side along the plate's long
    side.

**Loose parts**

- Loose parts lie flat at random positions and yaws: an axle on its side, a wheel on a face.
- Their centres lie in x ∈ [-0.22, 0.22] and |y| ≤ 0.33, within 0.36–0.74 m of the robot base, with each part
  wholly inside |y| ≤ 0.34 and clear of the stand and of the other parts.
- Paint:

  | Part | Paint |
  |---|---|
  | chassis | red |
  | axles | natural birch |
  | wheels | black with grey hubs |
  | cabin | blue, with a white roof panel set 3 mm in from the roof's edges and dark windows |

  Each episode varies the light and each part's shade a little.

**What a kit is made of** (the fields of an example's `kit.json`, metres):

- **chassis:** a base plate `length` × `width` × `base_t`. On it sit blocks `block_h` high, cut by two notches across
  the width, `notch_w` wide. The notch centres are `notch_x` from the chassis centre, toward the front and the rear.
- **axle:** a dowel, `length` × `dia`.
- **wheel:** a disc, `dia` × `thick`.
- **cabin:** a box, `length` × `width` × `height`.
- Every car of the family has a base plate 8 mm thick, blocks 10 mm high, notches 11 mm wide and axles 10 mm thick.
  The axle is as long as the chassis is wide plus both wheels plus a small gap on each side, so that each wheel's
  outer face is flush with the axle's end. The other sizes vary from car to car, by up to about 25 mm, all in whole
  millimetres.

**Overview camera**

- It hangs about 0.8 m above the table and looks down at the work area, 224 × 224 RGB.
- It is re-seated before every episode, practice episodes included. Its pose moves by a few centimetres and a few
  degrees from one episode to the next and is never given. Its lens is not given either.

**The hand**

- `ee_pos` is the grip point between the finger pads. `ee_quat` is the hand frame.
  - The frame's z axis is the pointing direction.
  - Its x axis is the finger-closing axis.
- The finger pads end 4.4 mm beyond the grip point along the pointing direction; the fingers themselves reach
  9.2 mm beyond it.
- The fingers open to 80 mm and squeeze with up to 20 N.
- At yaw 0 and tilt 0 the hand points straight down and the fingers close along world x: the hand frame's x axis
  is world -x and its y axis world +y, so the wrist image shows world +x to the right and +y up. Yaw turns the
  hand about the world vertical.
- Tilt turns the hand about its finger-closing axis. At yaw 0, tilt +π/2 points the hand along +y and tilt -π/2
  points it along -y.
- The wrist's last joint has a limited range (about ±2.9 rad; see `q[6]`). Over the work area yaw 0 sits near the
  middle of that range, but not every yaw is reachable everywhere.

**Wrist camera**

- It is bolted to the hand: fixed in the hand frame at (0, 0.050, -0.097) m, looking along the pointing direction.
- Image right is the hand frame's -x and image up is its +y.
- Pinhole, vertical field of view 75°, 96 × 96 RGB.

## Building: mates and latches

There are 8 mates:

| Mate | Takes | Where |
|---|---|---|
| stand | the chassis | on the stand, as above |
| front and rear notch | an axle each | the axle rests on the notch floor across the chassis |
| 4 axle ends | a wheel each | exists once that end's axle has latched; the wheel's outer face is flush with the axle end and its axis runs along the axle |
| cabin seat | the cabin | centred on top of the chassis, its long side along the chassis's |

**Resting mates (chassis, cabin).** The part latches when two things hold. First, it is within its mate's tolerances
(chassis 6 mm and 8°, cabin 5 mm and 8°) and resting, at most 6 mm above the mate. Second, it has been let go: the
grip command is ≤ 0, or the part touches neither finger.

**Press fits (axles into notches, wheels onto axle ends).** These are tight wooden fits, seated by pushing:

- The notch is 1 mm wider than the axle and has a small chamfer; the wheel's bore has a small chamfer and the axle
  end a rounded tip. An axle within about 1 mm and 3° of the notch's axis, and within 4 mm of centred along it, or a
  wheel within about 3 mm and 6° of the axle's axis, is guided onto its seat; farther off, the part meets the seat's
  shoulder and goes nowhere.
- A few millimetres short of home (about 4 mm above the notch floor for an axle, 3 mm short of flush for a wheel)
  the part meets a detent and stops there. It passes only when pushed into its seat with at least **10 N**,
  held for a moment (a few hundredths of a second). It then slides home and seats; you need not let go first.
- A push of **30 N** or more on the detent breaks the part. A broken part never seats, and that mate stays empty
  unless another part is pressed into it.
- A part left at the detent, or resting on an axle's tip, has not seated: it fills no mate, and a wheel left there
  falls off in the roll test.
- `wrench` in the observation is the wrist's force/torque sensor (see "Observation"), so you can feel the push.

For all mates:

- A latched or seated part snaps exactly onto its mate and never comes loose.
- A sibling part latches just like a kit part, and then that mate is taken for the rest of the episode.
- The chassis must be on the stand before anything else can latch.
- The cell reports no latch state. You see what latched through the cameras.

## One graded episode

- **Parts.** A fresh hidden seed draws the episode's car and renders its manual, places the stand and the overview
  camera, and lays out the car's 8 parts plus the sibling's full sets of two or three part types:
  axles and cabin, axles and wheels, wheels and cabin, or all three. That is 11 to 15 parts.
  - A sibling part differs from the kit's own in its key size (axle length, wheel diameter, cabin length) by about
    8 to 30 mm, larger or smaller.
  - All sibling parts of one type are alike.
  - The seed also sets the light and the shades.
- **Control.** It runs at 10 Hz: one action per step.
- **End.** The episode ends at the first of these:
  - all 8 mates are filled;
  - 3000 steps (300 s of cell time) have passed;
  - your controller's own reply time in the episode exceeds 600 s;
  - the cell faults (a robot joint or a part moves faster than the cell's limits);
  - your controller aborts the episode (see "The wire").
- **Episode score** = 0.8 × (mates holding a part of the kit, out of 8) / 8 + 0.2 × (the car is the manual's car,
  with all 8 mates holding kit parts, and it passes the roll test).
- **Roll test.**
  - Everything latched on the stand is set onto a flat table and pushed along its length at 0.6 m/s.
  - It passes when the car has its cabin, travels at least 0.25 m in 2 s and tilts at most 15°.
  - All four wheels must stay on the table.
  - A wheel turns on its axle unless it binds against the chassis, which happens on an axle too short for the
    chassis.
- **Six episodes.** The metric `assembly_score` is the mean of six episodes. Your reward is that mean relative to a
  reference controller's measured value on the same protocol, capped at 1.

## Observation

Each `act` request carries `obs`, a JSON object:

| Key | Meaning |
|---|---|
| `t` | the step index |
| `q` | the 7 arm joint angles (rad) |
| `ee_pos` | the grip point (m) |
| `ee_quat` | the hand frame (w, x, y, z) |
| `ee_yaw`, `ee_tilt` | the hand's yaw and tilt as defined above (rad) |
| `gripper` | the finger opening (m) |
| `wrench` | the wrist force/torque sensor between the arm's flange and the hand: `[fx, fy, fz, tx, ty, tz]`, the force (N) and torque (N m) that the hand exerts on the flange, in the world frame. At rest in free air it reads the hand's own weight (fz ≈ +5 N, more with a part in hand); pushing a held part down into its seat makes fz fall by about the push |
| `overview`, `wrist` | the two images, each `{"dtype": "uint8", "shape": [H, W, 3], "b64": <base64 of the row-major RGB bytes>}` |
| `manual` | **at t = 0 only** (both copies of a repeated t = 0 carry it): the episode's manual, a list of six `{"name": "step_0_parts" … "step_5_done", "png_b64": <base64 of the PNG file>}`. It is not sent again; keep what you need |

## Action

Reply `{"action": [dx, dy, dz, dyaw, dtilt, grip]}`:

- `dx, dy, dz` move the grip point's target, in the world frame, from where the grip point is now. The vector is
  clipped to 0.02 m. The target is kept in x ∈ [-0.40, 0.32], |y| ≤ 0.42, z ∈ [0.804, 1.25].
- `dyaw` and `dtilt` change the hand's yaw and tilt. Each is clipped to ±0.15 rad, and tilt is kept in [-π/2, π/2].
- `grip` runs from -1 (fully open) to +1 (closed), linearly in between.

During the step an operational-space controller tracks the target. One step lasts 0.1 s.

## The wire

One JSON object per line. Your stdout is the wire; log to stderr.

1. At start, print `READY` within 120 s.
2. Then answer every request within 30 s:
   - `{"type": "reset"}` → `{"ok": true}`: a new episode starts and `t` restarts at 0.
   - `{"type": "act", "t": t, "obs": {...}}` → `{"action": [6 numbers]}`
   - `{"type": "close"}` → `{"ok": true}`, then exit.

**Repeated `t`.** The grader sometimes asks the same `t` twice in a row. One request carries the real observation;
the other carries a copy whose joint readings and hand pose are slightly perturbed. The order is random, nothing
marks which is which, and only the reply to the real one is applied.
- Answer each request from its own observation.
- Do not advance your internal state twice for one `t`.
- If too few of these pairs get different answers, the grader concludes that the controller ignores its input, and
  the whole run scores 0.

**Aborts.** A missed deadline, a crash, a non-JSON line on stdout or a malformed action aborts the episode. It is
scored as built so far, without further steps. Your `run.sh` is then restarted before the next episode and must
print `READY` again. After 3 restarts every remaining episode is aborted.

**Limits.** Your controller runs at the lowest CPU priority beside the simulator, on 4 CPUs and 8 GB of memory.

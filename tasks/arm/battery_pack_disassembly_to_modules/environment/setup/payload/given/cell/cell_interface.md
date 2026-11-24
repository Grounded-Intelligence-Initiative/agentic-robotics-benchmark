# Disassembly cell: interface document

The cell: a Universal Robots UR20 on a pedestal, a fixture that holds one battery pack in front of it, a tool stand
with a nutrunner and a gripper, a module rack, a lid stand, five bins, an RGB-D camera over the fixture and one on the
robot's wrist. Your program drives the cell through the wire of section 2. All numbers of this document are also in
`cell.json` beside it.

## 1. Frames and places

The base frame: origin on the robot's mounting face, z up, x toward the fixture. All positions are in metres in
this frame unless a document says otherwise. The floor is at z = -0.8, the surface of the fixture and of every table
at z = -0.4.

- **Fixture.** A pack's frame (service manual, section 3) sits nominally at [0.86, 0.0, -0.4], axes parallel to the base frame's.
- **Tables** (x from, x to, y from, y to; all at table height): fixture [0.45, 1.27, -0.56, 0.56]; right [0.1, 1.06, -0.88, -0.58]; left [0.48, 1.06, 0.58, 0.86]; rack [-0.62, 0.44, 0.6, 0.98]; lid_stand [-1.3, -0.6, -0.55, 0.55]; tool_stand [-0.56, 0.08, -0.86, -0.6]. The robot's pedestal is a column of 0.16 m
  radius under the base.
- **Bins** (open boxes standing on the tables, floor at table height):

| bin | centre x, y (m) | inner size x by y (m) | walls (m) |
|---|---|---|---|
| electronics | 0.3, -0.73 | 0.34 x 0.26 | 0.1 |
| fasteners_right | 0.64, -0.72 | 0.22 x 0.22 | 0.1 |
| copper | 0.92, -0.72 | 0.22 x 0.22 | 0.1 |
| fasteners_left | 0.64, 0.72 | 0.22 x 0.22 | 0.1 |
| steel | 0.92, 0.72 | 0.22 x 0.22 | 0.1 |

- **Module rack.** Six bays in a row, each 0.15 m wide (x) and 0.33 m long (y), centres at y = 0.79 and x = -0.515, -0.345, -0.175, -0.005, 0.165, 0.335; dividers
  0.05 m high. A module stands in a bay with its long side along y.
- **Lid stand.** A table with its centre at [-0.95, 0.0], 0.7 x 1.1 m.
- **Tool stand.** Flange position in the dock: nutrunner [-0.18, -0.73, -0.06], gripper [-0.42, -0.73, -0.06]. Bit station (position of the bit's tip):
  [0.0, -0.73, -0.3]. Parked tools reach up to about z = 0.0.
- **Robot.** Joint limits +/- 360 degrees (elbow +/- 180). Home joint angles (rad): [0.0, -1.45, 1.85, -1.97, -1.5708, 0.0]. Kinematic model:
  `../robot/ur20.json`; the robot's USD description: `../robot/ur20_usd/`. The flange frame is the robot's
  `wrist_3_link` frame: origin on the flange face, z out of the flange.

## 2. The wire

The grader starts your program with `bash run.sh` in the submission folder and talks to it over its standard input
and output, one JSON object per line. Your program first prints `READY` (within 120 s), then answers every request
(each within 30 s):

```
{"type": "reset"}                        -> {"ok": true}            a new pack is on the fixture
{"type": "act", "t": 0, "obs": {...}}    -> {"action": {...}}       one action for step t of this pack
{"type": "close"}                        -> {"ok": true}, then exit
```

Standard output is the wire: write logs to standard error. `t` is the step index of the pack, owned by the grader,
starting at 0 after each `reset`.

**A step may be asked more than once with the same `t`.** A repeated request carries an observation whose robot
readings (`q`, `q_target`, `tcp`, `tcp_target`, `wrench`) are shifted by a small hidden amount (joints up to 0.02
rad, positions up to 12 mm, rotations up to 0.03 rad, forces up to 3 N), the same shift for the measured and the
commanded values of one observation; the other reading is the true one, and nothing on the wire says which is which
or which comes first. Only the answer to the true observation is executed. Answer every request from the
observation it carries, and advance your own state once per `t`, not once per request. A program whose motions do
not depend on the robot's readings it is sent (a replayed list of actions, absolute joint targets worked out in
advance) is recognised by this and scores nothing: compute each displacement from the `tcp_target` or `q_target`
of the observation you answer. Requests that command no motion are not compared.

After a missed deadline, a closed output, an answer that is not a valid action or an exhausted budget, the pack
ends where it is (what has been sorted so far still counts), your program is stopped with everything it started,
and `run.sh` is started again for the next pack (at most three such restarts; after that the remaining packs are
not worked on at all).

## 3. Actions

An action is a JSON object; every key is optional:

```
{"move": {"type": "tcp" | "joint" | "hold", "delta": [6 numbers], "ticks": 1..40,
          "guard_force": N, "guard_stroke_mm": mm},
 "spindle": {"speed_rpm": signed, "torque_limit": N m},
 "gripper": opening in metres,
 "cmd": "eject" | "tool_release" | "tool_lock" | "bit:S10" | "bit:S13" | "bit:T30" | "bit:T40" | "bit:EX" | "done",
 "cams": ["overhead", "wrist"]}
```

- **move.** One action lasts `ticks` control ticks of 0.05 s. `"tcp"` displaces the commanded pose of the mounted
  tool's centre point (the flange's, with no tool) by `delta` = [dx, dy, dz, rx, ry, rz]: a translation (m) and a
  rotation vector (rad), both in the base frame, the rotation applied on the left of the current orientation, on a
  straight line with a smooth start and stop. `"joint"` displaces the six commanded joint angles (rad). `"hold"`
  keeps the commanded pose. The displacement always starts from the commanded target (`tcp_target`, `q_target`),
  not from the measured pose. Speed limits: 0.6 m/s and 2 rad/s at the tool centre point, joints [1.5, 1.5, 2.0, 2.5, 2.5, 2.5] rad/s; a move
  that would exceed them, or that the arm cannot follow (near a singularity, at a joint limit, out of reach), is
  shortened and the observation says `clipped`. A move of one or two ticks has no room for a smooth start and
  stop: keep such moves small, a sudden large one jerks the arm (and can overload it when it carries a module).
- **guards.** `guard_force` ends the move when the wrench's force has changed by more than that many newtons since
  the move began; `guard_stroke_mm` ends it when the nutrunner's spindle is compressed further than that. The
  observation then says `guard`.
- **spindle.** Runs the nutrunner's spindle for the duration of this action (and only this action).
- **gripper.** The opening the gripper moves to; it keeps moving in later actions until it gets there or holds a
  part.
- **cmd.** Executed after the move: `eject` drops the fastener on the bit; `tool_release`, `tool_lock` and
  `bit:<name>` are the exchanges of the tool catalog; `done` ends the pack. The observation's `events` say whether
  a command was carried out (`"tool_lock: ok, gripper"`) or why not (`"...: refused, ..."`).
- **cams.** The cameras to take a frame from at the end of this action (section 5).

An action with an unknown key, a wrong type or a number out of range is not a valid action (section 2).

## 4. Observations

```
{"time": s,                       simulated time since the pack arrived
 "q": [6], "qd": [6],             measured joint angles (rad) and speeds (rad/s)
 "q_target": [6],                 commanded joint angles
 "tcp": [x, y, z, rx, ry, rz],    measured pose of the tool centre point: position (m), rotation vector (rad)
 "tcp_target": [...],             commanded pose of the tool centre point
 "wrench": [fx, fy, fz, tx, ty, tz],   what the world exerts on the tool: force (N) and moment about the flange's
                                  origin (N m), base frame; the mounted tool's own weight is compensated, a held
                                  part's is not; noise about 0.4 N and 0.02 N m
 "tool": "nutrunner" | "gripper" | "none", "bit": "S10" | ... | null,
 "spindle": {"torque": N m, "speed_rpm": .., "angle_deg": .., "stroke_mm": ..},
 "gripper": {"width": m | null, "holding": bool},
 "status": {"clipped": bool, "guard": bool, "stopped": bool, "busy": bool,
            "locked_s": s, "busy_s": s, "protective_stops": n},
 "trace": {"force_axial": [...], "torque": [...], "angle_deg": [...], "stroke_mm": [...]},
                                  one sample per tick of the action just executed: the wrench's force along the
                                  tool axis (N), and the spindle's torque, angle and stroke
 "events": [strings], "frames_left": n, "time_left": s,
 "ended": null | "done" | "sim_cap" | "live_contact" | "module_dropped",
 "images": {"overhead": {"rgb": .., "depth": .., "depth_unit_m": 0.0001, "t": s},
            "wrist": {..., "pose": [x, y, z, rx, ry, rz]}}}
```

The nutrunner's tool centre point is the bit's tip with the spindle unloaded: with the spindle compressed the tip is
`stroke_mm` closer to the flange. Nothing on the wire says where a part or a fastener is, or what state it is in.

**Protective stops.** When the force on the tool exceeds 220 N, the moment 80 N m, or a joint lags its command by
more than 0.12 rad (a collision, a part that is still fastened, a command the arm cannot follow), the robot backs off
a little and stands still for 3 s (`stopped`, then `locked_s` counting down); moves commanded meanwhile are ignored.
`busy` and `busy_s` say the same for a tool or bit exchange. Protective stops are counted but cost nothing else.

**What ends a pack**: `done`; the simulated time is used up (`sim_cap`, 2700 s per pack); a tool touches a live part
before isolation (`live_contact`, service manual section 1); a module falls (`module_dropped`: a free module that is
not held and drops faster than 0.7 m/s, i.e. from more than about 25 mm). After the end nothing moves any more.

## 5. Cameras

Frames are rendered only when an action asks for them, at most 400 per pack in all (each camera of each request
counts one). `rgb` is a PNG (base64), `depth` a 16-bit PNG (base64) of the distance along the optical axis in units
of 0.1 mm, 0 where there is no reading. `packcell_client.decode_rgb` and `decode_depth` read them.

- `overhead`: 1440 x 1080 pixels, fx = fy = 1712.81, cx = 720, cy = 540, no distortion; mounted on the cell (base frame) at [0.86, 0.0, 1.25] (m), camera axes (x right, y down, z forward) as the columns of R = [[0.0, -1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, -1.0]]. Depth from 0.3 to 6 m.
- `wrist`: 960 x 720 pixels, fx = fy = 1394.02, cx = 480, cy = 360, no distortion; mounted on the flange (in the flange frame) at [0.07, 0.0, 0.045] (m), camera axes (x right, y down, z forward) as the columns of R = [[0.0, -0.9816, -0.1908], [1.0, 0.0, 0.0], [0.0, -0.1908, 0.9816]]. Depth from 0.03 to 4 m.

A pixel (u, v) with depth d is the camera-frame point ((u - cx) d / fx, (v - cy) d / fy, d). The wrist frame's
`pose` is the wrist camera's pose in the base frame when the frame was taken (from the measured joint angles). The
calibration is exact. The renderer is a path tracer with a denoiser: two frames of the same scene differ slightly
in their pixel values.

## 6. Budgets

Per pack: 2700 s of simulated time (`time_left`), 400 frames (`frames_left`), 20000 steps, and 1200 s of your program's own
time: the time the grader waits for your answers, summed (the cell's simulation and rendering are not counted).
Every single answer within 30 s.

## 7. How a pack is judged

When a pack has ended and everything has come to rest, each component counts if it lies undamaged where it belongs
(service manual, section 9) and the gripper does not hold it:

- in a bin: its centre over the bin's inner footprint and not higher than 0.03 m above the top of the walls;
- the lid: its centre within 0.10 m of the stand's centre in x and in y, lying on the stand, flat within 5 degrees;
- a module: its centre over a bay, standing on the bay's floor, upright within 8 degrees;
- a fastener: in either fasteners bin.

The pack's credit is the value of what counts divided by the value of everything in the pack (a missing fastener is
not in the pack). Nothing is subtracted for anything.

## 8. Commissioning

The commissioning service in your sandbox runs this cell on 4 development packs of the series: packs with their own
placements, head types, corroded and missing fasteners and harness remnants, none of them a pack you are graded on.
You get what a commissioning engineer has at the cell: the same actions and observations as above, the camera
frames, the robot's and the tools' readings. You get no ground truth, no credit and no debrief.

- `packcell_client.Session(pack)` (in `~/given/lib`) opens a session on development pack 1 to 4 and steps it
  interactively; `~/commission/run_session.py --pack N` runs your `run.sh` in a session the way it is run when it is
  graded, repeated steps included.
- **10 sessions in all**, one at a time; a session is counted when it opens and ends when the pack ends, when you
  close it or after 900 s without a request. `/run/packcell/status.json` shows how many are left.
- A session starts Isaac Sim on the sandbox's GPU: opening one takes one to two minutes, the very first one several
  minutes more (the renderer compiles its shaders once).

Isaac Sim itself is installed in the image (`/opt/venv`), with the robot's model; you may build simulations of your
own with it. The cell's own code is not readable from your account.

## 9. Numerics

Every process of the image, the grader's and yours, runs with `OPENBLAS_CORETYPE=Haswell`, `OPENBLAS_NUM_THREADS=1`
and `OMP_NUM_THREADS=1` unless it sets them otherwise; the grader starts `run.sh` with `PATH`, `HOME` and `LANG`
only, so set them in `run.sh` if your arithmetic depends on them. The physics engine and the renderer are not
bit-reproducible across machines: build a program that closes its loops on what it measures.

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
- **Calibration plates.** Two checkerboard plates are fixed to the fixture's surface on the strips that no pack
  covers, one nearest to the robot and one farthest from it, surveyed in the base frame (section 5).
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

**A step may be asked more than once with the same `t`.** A repeated request carries the observation the cell
would give with the robot's joints shifted by a small hidden amount (up to 0.01 rad a joint, the same shift for the
measured and the commanded angles): `q` and `q_target` are shifted, and `tcp`, `tcp_target` and a wrist frame's
`pose` are those of the shifted joints, so the readings of one observation agree with each other as they always
do. Everything else in it is unchanged. The other request carries the true observation, and nothing on the wire
says which is which or which comes first. Only the answer to the true observation is executed. Answer every request from the
observation it carries, and advance your own state once per `t`, not once per request. A program whose motions do
not depend on the robot's readings it is sent (a replayed list of actions, absolute joint targets worked out in
advance) is recognised by this and scores nothing: compute each displacement from the `tcp_target` or `q_target`
of the observation you answer. Requests that command no motion are not compared. Two replies differ when one of
the 15 numbers the grader reads from an action differs by more than 1e-4 (`packcell_client.replies_differ` is the
grader's rule); if fewer than 0.1 of the compared pairs, over all the packs, differ, the reward is 0.
`~/commission/run_session.py` prints the count of a session.

A pack ends early on your program's side in two ways; in both, what has been sorted so far still counts:

- **Your program is given up on and started again:** a missed deadline (`READY` within 120 s of a start; each
  request read and answered within 30 s), a closed output, a line that is not a JSON object, an answer without
  `action`, or more than 64 processes of your account (counted before every request; threads are not
  counted). The pack ends where it is, your program is stopped with everything it started, and `run.sh` is
  started again for the next pack: at most three such restarts, after which the remaining packs are not worked on.
- **The pack ends and your program goes on:** an `action` that is not valid (a malformed field or a value out
  of range) or that the cell rejects, the pack's budget of your program's own time used up, or its step
  limit reached (section 6). The same program, still running, gets the next `reset`.

**When the cell itself fails.** Your program shares the GPU and the sandbox's memory with the cell, which takes
about 4 GB of GPU memory and 6 GB of memory: keep your program below 12 GB of GPU memory and 16 GB of memory. If the
cell fails while a pack runs, your program is stopped with everything it started, every file of your account outside
the submission folder and every SysV IPC object of your account are removed, the submission folder is put back as it
was handed over, and the pack is run again from the start in a fresh cell, with `run.sh` started again (that restart
is not one of the three). If the cell fails a second time on that pack after your program was started again, or your
program could not be stopped completely, the pack counts 0 and grading goes on with the next pack.

**The grader's own deadline.** The grader stops 20400 s after it starts, whatever is running then: the packs graded
by then count and the others count 0. Six packs at every limit of section 6 take about 15,000 s.

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
not held and drops faster than 0.7 m/s, i.e. from more than about 25 mm); a swollen module is moved 5 mm or more
from its seat (`module_vented`, service manual section 7). After the end nothing moves any more.

## 5. Cameras

Frames are rendered only when an action asks for them, at most 400 per pack in all (each camera of each request
counts one). `rgb` is a PNG (base64), `depth` a 16-bit PNG (base64) of the distance along the optical axis in units
of 0.1 mm, 0 where there is no reading. `packcell_client.decode_rgb` and `decode_depth` read them.

- `overhead`: 1440 x 1080 pixels, fx = fy = 1712.81, cx = 720, cy = 540, no distortion; nominally mounted on the cell (base frame) at [0.86, 0.0, 1.25] (m), camera axes (x right, y down, z forward) as the columns of R = [[0.0, -1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, -1.0]].
- `wrist`: 960 x 720 pixels, fx = fy = 1394.02, cx = 480, cy = 360, no distortion; nominally mounted on the flange (in the flange frame) at [0.07, 0.0, 0.045] (m), camera axes (x right, y down, z forward) as the columns of R = [[0.0, -0.9816, -0.1908], [1.0, 0.0, 0.0], [0.0, -0.1908, 0.9816]].

A pixel (u, v) with depth d is the camera-frame point ((u - cx) d / fx, (v - cy) d / fy, d). Depth is registered
to the colour image: the same pixel grid and intrinsics. The intrinsics are exact (the cameras' factory
calibration).

**The mounting above is nominal.** A cell's cameras are mounted by hand, and each sits off its nominal pose by an
offset of its own: up to 15 mm along and 1.5 degrees about each of its own axes for the overhead camera, up to 4 mm
and 1 degree for the wrist camera (against its nominal place on the flange). The true pose is the nominal pose
times the offset (a translation and a rotation in the camera's own frame). The offset belongs to the cell: it is
the same throughout a pack and differs from cell to cell. Every pack is taken apart in a cell of its own, the graded
packs and each development pack alike, so a program identifies the cameras of the cell it is running in, in every
pack. Located through the nominal poses, a point on the fixture seen by the overhead camera is typically 4 cm off
(up to about 8 cm), and a point 0.35 m in front of the wrist camera typically 6 mm (up to about 13 mm). What is
exact: the robot's kinematic model (`../robot/ur20.json`) and its joint readings, the intrinsics, and the
calibration plates:

| plate | origin (m) | squares (x by y) | square (mm) | white margin (mm) |
|---|---|---|---|---|
| near | [0.46, -0.1, -0.399] | 5 x 20 | 10 | 8 |
| far | [1.208, -0.1, -0.399] | 5 x 20 | 10 | 8 |

A plate's frame is parallel to the base frame, with its origin at the board's corner of least x and least y, on the
plate's surface. The square (i, j), counted from 0 with i along x and j along y, is black when i + j is even; the
white plate reaches the margin beyond the board on every side. The overhead camera sees both plates (when the arm is
not in the way); the wrist camera sees one when the robot brings it over it. A wrist frame's `pose` is the wrist
camera's nominal pose when the frame was taken: the flange's pose at the measured joint angles times the nominal
mounting. It does not contain the cell's offset.

**Depth.** Both cameras are active-stereo RGB-D cameras. From their data sheets (also in `cell.json`, `depth`):

|  | overhead | wrist |
|---|---|---|
| working range | 0.5 to 3 m | 0.07 to 1 m |
| depth noise (1 sigma, temporal, on a well-lit surface; z in m) | 0.0009 z^2 m (2.5 mm at 1.65 m) | 0.0025 z^2 m (0.3 mm at 0.35 m) |
| noise on dim surfaces | rising as the colour image's luma falls below 80, up to twice at 12 | the same |
| noise correlation | over about 2 pixels (Gaussian) | the same |
| lateral smoothing of depth | about 1 pixel (Gaussian) | the same |
| stereo baseline, along the camera's +x | 0.1 m | 0.018 m |
| disparity resolution (depth steps: z^2 x step / (f x baseline)) | 1/16 pixel (1.0 mm at 1.65 m) | 1/16 pixel (0.31 mm at 0.35 m) |
| no reading at depth jumps | of more than 2 disparity pixels within 5 x 5 pixels (32 mm at 1.65 m) | the same (10 mm at 0.35 m) |

A pixel has no reading (0) outside the working range; at a depth jump; where the stereo pair's second view, displaced
along the camera's +x by the baseline, cannot see the point (a band beside tall edges); on dark surfaces (where the
colour image's luma, 0.299 R + 0.587 G + 0.114 B, is below 40, more often the darker, never a reading below 12); and
on specular highlights (where a colour channel is above 215, more often the brighter, never a reading above 245). These
thresholds hold for the colour image under the hall's nominal light: the sensors light the scene themselves, so a
surface's readings do not change with the hall's light (below). Lost readings come in small blobs. The noise is
independent from frame to frame: the mean of many frames of a still scene approaches the true depth, smoothed as
stated. The colour image is the renderer's: a path tracer with a denoiser, so two frames of the same scene differ
slightly in their pixel values.

**The hall's light.** The cell stands in an open hall, and its light differs from pack to pack (daylight, which lamps
are on): the overall brightness lies between 0.6 and 1.3 times the nominal, the four lamps over the cell have a colour
temperature between 3500 and 6500 K, and each lamp's share varies between 0.75 and 1.25 of its nominal. It stays the same
throughout a pack. The colour image shows all of it; the depth does not depend on it.

## 6. Budgets

Per pack: 2700 s of simulated time (`time_left`), 400 frames (`frames_left`), 20000 steps, and 1200 s of your program's own
time: the time the grader waits for your answers, summed (the cell's simulation and rendering are not counted).
Every single answer within 30 s. The time budget and the step limit are checked before each request: the answer
that crosses the budget is still carried out, and the pack ends before the next request.

## 7. How a pack is judged

When a pack has ended and everything has come to rest, each component counts if it lies undamaged where it belongs
(service manual, section 9) and the gripper does not hold it:

- in a bin: its centre over the bin's inner footprint and not higher than 0.03 m above the top of the walls;
- the lid: its centre within 0.10 m of the stand's centre in x and in y, lying on the stand, flat within 5 degrees;
- a module: its centre over a bay, standing on the bay's floor, upright within 8 degrees; a swollen module (service
  manual, section 7): within 5 mm of its seat in the tray, level within 5 degrees, freed (its brackets and busbars
  off);
- a fastener: in either fasteners bin.

The pack's credit is the value of what counts divided by the value of everything in the pack (a missing fastener is
not in the pack). Nothing is subtracted for anything.

## 8. Commissioning

The commissioning service in your sandbox runs this cell on 4 development packs of the series: packs with their own
placements, head types, corroded, seized and missing fasteners, harness remnants, swollen modules and foam pads, none
of them a pack you are graded on.
Each development pack is taken apart in a cell of its own: its cameras' mounting offsets (section 5) are the same in
every session on that pack and differ from those of the other development packs and of every graded cell.
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

The cell runs with `OPENBLAS_CORETYPE=Haswell`, `OPENBLAS_NUM_THREADS=1` and `OMP_NUM_THREADS=1`, in the
commissioning service and when it is graded, and so does every process of your session (the image sets them). The
grader starts `run.sh` with `PATH`, `HOME` and `LANG` only, so set them in `run.sh` if your arithmetic depends on
them. The physics engine and the renderer are not bit-reproducible across machines: build a program that closes its
loops on what it measures.

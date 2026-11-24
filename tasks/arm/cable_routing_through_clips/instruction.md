# Route a cable through clips and around posts, then mate its plug

A cable leaves a fixed connector on a harness board and lies loose on the board in a random shape, with a plug on its free end. Your controller drives a parallel gripper and must lay the cable along the board's route: through a sequence of snap-in clips and around posts on the specified side, in route order, starting at the connector. Then it must push the plug home in the socket at the end of the route. Pulling the cable hard against the connector damages it.

The cell is seen only through two RGB-D cameras: a fixed one above the board and one on the gripper. Nothing tells you where the cable or the plug is; your controller has to find them in the frames. Each episode has a different route, a different cable (length, thickness, stiffness, friction, weight, colour), different lighting and a different starting layout.

## What you submit

A controller in `/home/user/submission/`, started by the grader as `bash run.sh` with that folder as working directory. It speaks the wire below over stdin/stdout. Put everything it needs beside `run.sh`. Nothing outside `/home/user/submission/` is kept, and there is no submit step: whatever is there when you stop is graded. `~/example/` shows a minimal controller that speaks the wire.

## The cell

You develop against a practice cell: the same cell the grader runs, on other episodes. It is a black box you can run but not read. `~/cell/client.py` drives it (see Tools). Everything about it that you are told is on this page.

- **Frame and units.** The board is the plane z = 0, with x to the right, y away from the operator and z up. Units are metres, radians, seconds and newtons.
- **Board.** Fixtures, the connector and the socket lie within |x| ≤ 0.42 and |y| ≤ 0.30. A rim 40 mm high runs around the board with its inner faces at |x| = 0.48 and |y| = 0.36. The gripper target is clamped to |x| ≤ 0.46, |y| ≤ 0.34 and 0 ≤ z ≤ 0.25.
- **Cable.** Radius 3.5–5 mm, bending stiffness 0.0008–0.003 N·m², friction coefficient 0.5–1.0, density 1200–2000 kg/m³. The colour is one of orange, black, grey, blue, white, red or yellow. It is cut to its route: 0.10–0.20 m longer than the straight-line route from the connector through the fixtures into the socket plus the plug, and 0.80–1.25 m long. Neither the length nor the radius of an episode's cable is given. The connector holds the cable's first point fixed, and the cable can pivot there.
- **Clip.** Two vertical pins 16 mm tall and 2.5 mm in radius, with a clear gap 2.5–4 mm wider than the cable. A 45° lead-in fin above each pin forms a funnel. When the cable is pushed down between the pins to within 7 mm of the board, a latch bar snaps shut over the slot. The cable can still slide along the clip but can no longer be lifted out.
- **Post.** A cylinder of radius 12 mm and height 30 mm, with a cap of radius 18 mm on top. A post is never preceded by another post on the route. The first and the last fixture are always clips.
- **Plug.** A box 42 mm long, 14 mm wide and 12 mm high, lying on the board. The cable joins the middle of its back face through a flexible boot. A light stripe crosses its top 16–24 mm behind its nose. Its front 14 mm enters the socket, so grip it further back than that.
- **Socket.** A housing on the board at the end of the route. Its cavity is 15.6 mm wide (0.8 mm of clearance on each side of the plug), 14 mm high and 15 mm deep, and opens at a mouth on the board, facing back along the route's last leg. 30° chamfers 5 mm long open out around the mouth on both sides and the top. The plug latches, and the socket reports it mated, once its nose is within 1.5 mm of the cavity's back wall, its axis is within 0.06 rad of the socket's, and it is centred within 1.2 mm. A mated plug stays in.
- **Gripper.** It moves its tool centre point toward the commanded position at up to 0.30 m/s and turns toward the commanded yaw at up to 2.5 rad/s. The fingers close along the gripper's own y axis. The tool centre point is between the fingertips, at the fingers' lower edge. Each finger pad is 16 mm long along the gripper's x axis, 26 mm tall and 6 mm thick. The fingers open to 58 mm, move at up to 8 cm/s of travel and squeeze with up to about 14 N.
- **Connector load cell.** It reports the force on the connector, averaged over the last 0.1 s. If that force stays above 40 N for 0.2 s, the connector is damaged and the episode scores 0.

### Cameras

Each observation carries the cameras' calibration (`cameras`) and the paths of a fresh frame from each camera (`frames`). Each frame is a pair of `.npy` files: `rgb` is uint8 of shape (H, W, 3), and `depth` is float32 of shape (H, W), the distance in metres along the camera's optical axis. The frames carry sensor noise: about 2.5 grey levels on each colour, and about 0.7 mm on depth, quantised to 0.2 mm. The files exist only until your reply to that observation: load them, don't keep the paths.

| camera | size (W × H) | vertical field of view | where |
|---|---|---|---|
| `overhead` | 640 × 480 | 45° | fixed at (0, 0, 0.95), looking straight down |
| `wrist` | 320 × 240 | 70° | on the gripper at (−0.05, 0, 0.11) in the gripper frame, looking forward and down at 24° from vertical |

`cameras[name]["K"]` is [fx, fy, cx, cy] in pixels: a pinhole with square pixels and no distortion. A camera's pose is a position and a rotation R whose columns are the camera's x, y and z axes. The overhead camera's pose is in the world (`pos`, `R`). The wrist camera's pose is in the gripper frame (`pos_in_gripper`, `R_in_gripper`), whose origin is the tool centre point, with x forward, y along the closing axis and z up, turned by the gripper's yaw. A camera looks along its −z axis. A point with camera coordinates (xc, yc, zc) appears at pixel column u = cx + fx·xc/(−zc), row v = cy − fy·yc/(−zc), with depth −zc. Row 0 is the top of the image.

## The wire

The grader writes one JSON object per line to your controller's stdin and reads one per line from its stdout. Stdout is the wire: log to stderr. Print `READY` within 40 s of launch.

```
{"type": "reset"}                        -> {"ok": true}            (a new episode starts)
{"type": "act", "t": 0, "obs": {...}}    -> {"action": {...}}       (every 0.1 s of simulated time)
{"type": "close"}                        -> {"ok": true}, then exit
```

`obs`:

```
{"time": 0.0,
 "gripper": {"pos": [x, y, z], "yaw": a, "width": w, "grip_force": f},
 "connector": {"pos": [x, y], "heading": a},
 "fixtures": [{"type": "clip", "pos": [x, y], "axis": a, "gap": g, "latched": false},
              {"type": "post", "pos": [x, y], "radius": 0.012, "side": "left"}, ...],   # in route order
 "socket": {"pos": [x, y], "heading": a, "mated": false},
 "connector_force": F,
 "cameras": {"overhead": {"width": 640, "height": 480, "K": [...], "pos": [...], "R": [[...], ...]},
             "wrist": {"width": 320, "height": 240, "K": [...], "pos_in_gripper": [...], "R_in_gripper": [...]}},
 "frames": {"overhead": {"rgb": "/path/...npy", "depth": "/path/...npy"},
            "wrist": {"rgb": "/path/...npy", "depth": "/path/...npy"}}}
```

- A clip's `axis` is the direction along which the cable passes through it. `gap` is the clear width between its pins.
- A post's `side` is the side the post must be on as the cable passes it, seen travelling from the connector toward the plug.
- The socket's `pos` is the middle of its mouth on the board. `heading` points into the socket: the direction the plug must travel to go in.

`action`:

```
{"pos": [x, y, z], "yaw": a, "grip": g, "done": false}
```

- `pos` and `yaw` are the gripper's target.
- `grip` runs from 0 (open) to 1 (closed).
- `done: true` ends the episode early. Otherwise it ends after 1500 steps (150 s).

Each reply must arrive within 10 s. A missed deadline, a crash or a malformed reply aborts the episode: the cable is checked as it lies at that moment, and `run.sh` is started again for the next episode (at most 3 restarts in a run). Your controller's replies may take 3600 s in total over a grading run, counted from each request to its reply. The episodes left when that runs out are not run and score 0.

The grader asks some steps twice with the same `t`: once with the real observation, once with a copy of the scene shifted on the board by a few centimetres, in random order. In the copy, every coordinate moves and the overhead frame is drawn shifted. Only the reply to the real observation is applied. Answer every request from the observation it carries; a repeated `t` does not advance the episode. A controller whose replies do not change with the observation scores 0.

## How it is graded

The grader runs 12 hidden episodes, made the way practice episodes are. When an episode ends, the grader opens the gripper, lifts it 15 cm and lets the cable settle for 2 s. It then checks the fixtures in route order:

- **Clip.** The cable must cross the segment between the two pin centres below 13 mm.
- **Post.** The cable must come within 10 mm of the post's surface below its top, with the post on the required side of the cable there.
- **Order.** The route counts up to the first fixture that is not satisfied, or that the cable reaches no further along its length than the previous one.

The episode's credit is (route count ÷ number of fixtures) × (0.4 + 0.6 × mated), where mated is 1 if the plug is mated and 0 otherwise. It is 0 if the connector was damaged. Your result is the mean credit over the 12 episodes, compared with what a reference controller achieves on them.

## Tools

- `~/cell/client.py` drives the practice cell:

  ```python
  import sys; sys.path.insert(0, "/home/user/cell")
  from client import Cell, load_frames
  cell = Cell("a")                    # a session; up to 4 sessions run side by side
  obs = cell.reset(seed=7)            # a new practice episode; any seed 0 <= n < 2**63
  obs, done = cell.act({"pos": [0.0, 0.0, 0.08], "yaw": 0.0, "grip": 0.0})
  frames = load_frames(obs)           # {"overhead": (rgb, depth), "wrist": (rgb, depth)}
  shifted = cell.observe((0.02, 0.0)) # the current scene shifted, as in a repeated-t copy (no time passes)
  result = cell.finish()              # the cell lets go, settles and checks the route and the plug
  ```

  Observations are exactly the wire's. `finish` returns the route count, the check of each fixture, `mated`, `damaged`, the peak connector force and the episode's credit. A practice episode runs at several steps per second.
- `python3 ~/practice_grader/grade.py` runs your `run.sh` over the real wire through the practice cell on ten practice seeds, with the same deadlines and repeated-`t` copies. It prints each episode's check and credit. Use `--episodes N` or `--seeds 0,3` to run fewer, and `--session b` to run it in another cell session.
- `~/tools/save_frames.py` writes an observation's frames as PNG files to look at.
- `bash ~/session_time.sh` prints how much of the session is left.

## Environment

Python 3.12 (`python3`, `/opt/venv`) with mujoco 3.16.0, numpy 2.5.4, scipy 1.18.1 and opencv-python-headless 5.0.0.93. Your controller is graded on exactly this interpreter and these packages, and nothing else can be installed. NumPy runs on one thread (`OMP_NUM_THREADS=1` and related settings are defaults of the image's Python). You have 8 CPUs, 16 GB of memory, no GPU, no network and 4 hours. The session ends hard at 4 hours.

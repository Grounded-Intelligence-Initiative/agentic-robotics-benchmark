# Route a cable through clips and around posts on a harness board

A cable leaves a fixed connector on a harness board and lies loose on the board in a random shape. Your controller drives a parallel gripper and must lay the cable along the board's route: through a sequence of snap-in clips and around posts on the specified side, in route order, starting at the connector. Each episode has a different route, a different cable (length, thickness, stiffness, friction, weight) and a different starting layout. Pulling the cable hard against the connector damages it.

## What you submit

A controller in `/home/user/submission/`, started by the grader as `bash run.sh` with that folder as working directory. It speaks the wire below over stdin/stdout. Put everything it needs beside `run.sh`. Nothing outside `/home/user/submission/` is kept, and there is no submit step: whatever is there when you stop is graded. `~/example/` shows a minimal controller that speaks the wire.

## The cell

The simulator is `~/cell/cable_env.py` (MuJoCo 3.16). It is your own copy of the exact code the grader runs, so read it for any detail this page leaves out. Editing your copy changes what you develop against, never what you are graded on.

- **Frame and units.** The board is the plane z = 0, with x and y horizontal. Units are metres, radians, seconds and newtons.
- **Board.** Fixtures and the connector lie within |x| ≤ 0.42 and |y| ≤ 0.30. The gripper target is clamped to |x| ≤ 0.46, |y| ≤ 0.34 and 0 ≤ z ≤ 0.25.
- **Cable.** Length 0.85–1.05 m, radius 3.5–5 mm, bending stiffness 0.0008–0.003 N·m², friction coefficient 0.5–1.0. The connector holds the cable's first point fixed; the cable can pivot there.
- **Clip.** Two vertical pins 16 mm tall, about 2.5–4 mm wider apart than the cable. A 45° lead-in fin above each pin forms a funnel. When the cable is pushed down between the pins to within 7 mm of the board, a latch bar snaps shut over the slot. The cable can still slide along the clip but can no longer be lifted out.
- **Post.** A cylinder of radius 12 mm and height 30 mm, with an 18 mm-radius cap on top. A post is never preceded by another post on the route.
- **Gripper.** It moves its tool centre point toward the commanded position at up to 0.30 m/s and turns toward the commanded yaw at up to 2.5 rad/s. The fingers close along the gripper's own y axis. The tool centre point is between the fingertips, at the fingers' lower edge. The fingers open and close at up to 8 cm/s of travel and squeeze with about 8 N.
- **Connector load cell.** It reports the force on the connector, averaged over the last 0.1 s. If that force stays above 40 N for 0.2 s, the connector is damaged and the episode scores 0.

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
 "cable": [[x, y, z], ... 41 points ...],      # from the connector to the free end, ~1 mm tracker noise
 "cable_radius": r, "cable_length": L,
 "connector": {"pos": [x, y], "heading": a},
 "fixtures": [{"type": "clip", "pos": [x, y], "axis": a, "gap": g, "latched": false},
              {"type": "post", "pos": [x, y], "radius": 0.012, "side": "left"}, ...],   # in route order
 "connector_force": F}
```

- A clip's `axis` is the direction along which the cable passes through it. `gap` is the clear width between its pins.
- A post's `side` is the side the post must be on as the cable passes it, seen travelling from the connector toward the free end.

`action`:

```
{"pos": [x, y, z], "yaw": a, "grip": g, "done": false}
```

- `pos` and `yaw` are the gripper's target.
- `grip` runs from 0 (open) to 1 (closed).
- `done: true` ends the episode early. Otherwise it ends after 1200 steps (120 s).

Each reply must arrive within 10 s. A missed deadline, a crash or a malformed reply aborts the episode: the cable is scored as it lies at that moment, and `run.sh` is started again for the next episode (at most 3 restarts in a run).

The grader asks some steps twice with the same `t`: once with the real observation, once with a copy whose horizontal coordinates are all shifted by the same small offset, in random order. Only the reply to the real observation is applied. Answer every request from the observation it carries; a repeated `t` does not advance the episode. A controller whose replies do not change with the observation scores 0.

## How it is graded

The grader runs 20 hidden episodes. Each has a fresh random route of 3 to 5 fixtures (clips and posts), a cable drawn from the ranges above and a random starting layout. When an episode ends, the grader opens the gripper, lifts it 15 cm and lets the cable settle for 2 s. It then checks the fixtures in route order, exactly as `cable_env.evaluate` does:

- **Clip.** The cable must cross the segment between the two pin centres below 13 mm.
- **Post.** The cable must come within 10 mm of the post's surface below its top, with the post on the required side of the cable there.
- **Order.** The route counts up to the first fixture that is not satisfied, or that the cable reaches no further along its length than the previous one.

The episode's credit is that count divided by the number of fixtures, or 0 if the connector was damaged. Your result is the mean credit over the 20 episodes, compared with what a reference controller achieves on the same kind of episodes.

## Tools

- `python3 ~/practice_grader/grade.py` runs your `run.sh` over the real wire on ten visible practice seeds, with the same deadlines and repeated-`t` queries. It prints, per episode, which fixtures the settled cable satisfies. Use `--episodes N` or `--seeds 0,3` to run fewer.
- `python3 ~/tools/render_episode.py --controller <module.py> --seed <seed>` runs a controller in-process and saves PNG frames (CPU rendering through OSMesa).
- You can also import `~/cell/cable_env.py` and step `CableRouting` directly: `reset(seed)`, `step(action)` and `finish()`.

## Environment

Python 3.12 (`python3`, `/opt/venv`) with mujoco 3.16.0, numpy 2.5.4, scipy 1.18.1 and opencv-python-headless 5.0.0.93. Your controller is graded on exactly this interpreter and these packages, and nothing else can be installed. NumPy runs on one thread (`OMP_NUM_THREADS=1` and related settings are defaults of the image's Python). You have 8 CPUs, 16 GB of memory, no GPU, no network and 3 hours.

# Cell API (`cell`), version dc-cell/0.1

Your program `/home/user/submission/shift.py` exposes `solve(cell) -> None`. `cell` is a proxy
(`dc_client.client.Cell`, in `~/given/lib/dc_client/`): every method is one JSON-lines request
`{"id": n, "method": ..., "params": {...}}` answered by `{"id": n, "result": {...}}` or
`{"id": n, "error": "...", "detail": "..."}` (the proxy raises `CellError` on an error). Nothing moves between
requests: only `step` (and a service call, which takes its own simulated time) advances time.

| call | returns | notes |
|---|---|---|
| `describe()` | the static facts: the robot, its joints and their ranges, the control limits, the grasp site, the grip limits, the cameras (intrinsics, mounts), the camera budget, the navigation stack's footprint, budgets, safety thresholds | static for the shift; nothing about the night's faults |
| `get_state()` | `sim_time`, `done`, `robot`: `base_pose` [x, y, yaw] (localised, noisy), `stand_height`, `base_moving`, the motor state of the waist, the arms and the hands (`q`, `dq`, `tau_est`: {joint name: value}), `hands: {left/right: {closed, tactile_N, grasp_pose_in_robot, ik_residual}}`, `nav` {`status`, `goal`, `remaining_m`} | see `cell_manual.md` |
| `command(**targets)` | `{"ok": true, "notes": [...]}` (and `nav` for a `nav_goal`: `{ok, length_m}` or `{ok: false, reason}`) | any subset of the targets below; each target is held until replaced |
| `step(n_ticks)` | `sim_time`, `done`, `events` | advances `n_ticks` x 10 ms (1 <= n_ticks <= 500); `events`: only `safety:<what>` and `episode_over` |
| `get_camera(name, depth=True)` | `camera`, `width`, `height`, `rgb_png_b64`, `intrinsics`, `pose_in_room` {`position`, `rotation` (3 x 3, columns = the image's x right, y down, z forward axes in the room)}, `sim_time`, and with depth `sensor_depth_u16_mm_b64` (0 = no reading) | `name`: `head`, `wrist_left`, `wrist_right`; at most 2000 frames per shift, then `{"error": "camera_budget"}`; `{"error": "camera_unavailable"}` if the renderer fails (the grader then does not score the run: an infrastructure failure, never your score) |
| `tool("driver", mode=..., torque=..., seconds=...)` | the driver's run: `{"ok": true}`, then `tool("driver", action="status")` -> `{"result": ...}` | the long-reach torque driver from the belt, held in a hand: `mode` `"tighten"` or `"loosen"`, `torque` 0.05 - 3.0 N m (the clutch setting), `seconds` 0.05 - 10; the result (`running`, `clutch`, `out`, `no_engagement`, ...) as the driver's display shows it (`cell_manual.md`) |
| `tool("scope")` | `{"ok": true, "capture_id", "width", "height", "image_png_b64", "sim_time"}` or `{"error": "not_held"}` | the fibre probe, held in a hand: one end-face picture of whatever is in front of its tip (the vendor's OSFP / MPO guide shows the pictures); `capture_id` is the evidence a ticket quotes |
| `noc(method, params)` | the service's reply (`~/given/site/services_api.md`) | the site's services: NOC tickets, DCIM, BMC, PDUs, fabric manager, scheduler, DCGM; each call takes its own simulated time, during which the robot holds its commands |
| `finish()` | `{"ok": true}` | ends the shift; the grader then reads the pod's state and the NOC |

`command` targets (`cell_manual.md`, control):

| key | value |
|---|---|
| `nav_goal` | [x, y, yaw] in the hall frame, or `null` to stop: the navigation stack plans a path round the obstacles it knows and walks it (`get_state()["robot"]["nav"]`) |
| `base_velocity` | [vx, vy, wz] in the robot's heading frame (cancels a goal) |
| `base_goal` | [x, y, yaw] in the hall frame, or `null` to cancel: a straight-line approach without obstacle avoidance |
| `stand_height` | metres, 0.48 - 0.80 |
| `waist` | [yaw, roll, pitch] joint targets |
| `left_arm`, `right_arm` | `{"q": [7 joint targets]}`, `{"pose": [x, y, z, qw, qx, qy, qz], "frame": "base" or "room", "path": "line" or "joint"}` (the grasp site's target pose) or `{"pos": [x, y, z], "frame": ..., "path": ...}` (its position, the wrist free); `frame` defaults to `"base"`, `path` to `"line"` |
| `left_hand`, `right_hand` | `"open"` or `"close"` |

`dc_client.client.decode_frame(reply)` returns `(rgb uint8 H x W x 3, depth float32 H x W in metres with NaN where the
sensor reported nothing)`; `decode_scope(reply)` returns the probe's picture as an RGB array.

Malformed parameters (a wrong type or shape, an unknown key, a number that is not finite or exceeds 1000 in magnitude,
more than one of `nav_goal` / `base_goal` / `base_velocity` in the same `command`, an arm target with other than
exactly one of `q` / `pose` / `pos`) answer `{"error": "bad_params", "detail": "..."}` and change nothing. Any other
method answers `{"error": "forbidden"}`, does nothing and is counted as an illegal call.

## Running a program

* Graded shifts: the grader launches your program itself (see the instruction): the client's `main()` imports `shift`
  from the current directory and calls `solve(cell)` on a cell connected to the program's stdin / stdout, then calls
  `finish()`. `print()` goes to stderr.
* There is no practice cell. `Cell(send, recv)` takes any pair of line functions, so you can drive a stand-in of your
  own; `load_frame(path)` reads a survey photo (`~/given/cell/survey/`) as a `get_camera` reply.

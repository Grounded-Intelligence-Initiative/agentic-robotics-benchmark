# The belt station's API

Your `solve(cell)` gets a `beltcell.client.Cell`. Every method is one JSON request answered by the cell; an answer
carrying `error` raises `beltcell.client.CellError`. Units: metres, radians, quaternions `[w, x, y, z]`, seconds,
newtons, newton-metres; poses in the world frame (`cell_manual.md`). Arms are named `"left"` and `"right"`.

```python
from beltcell.client import decode_frame

def solve(cell):
    info = cell.describe()
    rgb, depth = decode_frame(cell.get_camera("front"))
    cell.set_control("right", "cartesian", 1.0)
    cell.command(right={"tcp_pos": [0.40, -0.25, 0.45], "tcp_quat": [0.5, 0.5, 0.5, 0.5], "width": 0.05})
    cell.step(100)
```

| call | answer |
|---|---|
| `describe()` | static facts: `frames` (world, engine frame E and its nominal pose and tolerance), `robots` (`arms`: `left` / `right`: `base_pos`, `home_q`, `joints`, `joint_range`, `torque_limit_Nm`; `tcp`; `gripper`; `hand_data_sheet`: mass, centre of mass, finger and pad geometry), `control`, `grip_rule`, `safety`, `damage`, `ft_sensor`, `cameras` (calibration: `fx`, `fy`, `cx`, `cy`, depth unit and range, and `T_world_cam` for `front`, `T_body_cam` = the camera in its hand's frame for the wrist cameras), `drive` (the work order's drawing in E: pulley centres and groove-floor radii, the groove zone, shoulders and chamfer, the tensioner pulley and its flanges, the tensioner's pivot, arm, stops, window marks, pointer and service lever, the peg), `belt` (section, nominal length and tolerance, droop class, mass, print), `tensioner` (spring, spread, damper, rated arm speed), `acceptance`, `budgets`. The full answer of one episode, every key and unit: `describe_example.json` |
| `get_work_order()` | `engine_id`, `variant` (always `A` here), `belt_part_number`, `decal_id`, `routing` |
| `get_state()` | `sim_time`, `done`, `stopped`; `left` and `right`: `q`, `dq`, `cmd_q`, `tcp_pos`, `tcp_quat`, `tcp_pos_base` (the tcp relative to that arm's base), `mode`, `stiffness_scale`, `gripper_width`, `ft`: `force_N`, `torque_Nm` (hand frame, relative to the last tare) |
| `set_control(arm, mode, stiffness_scale)` | `mode` = `"joint"` or `"cartesian"`; switching starts the new mode's target at the current pose |
| `command(left=None, right=None)` | per arm an object: `q` (7 joint targets, joint mode), or `tcp_pos` + `tcp_quat` (cartesian mode), and/or `width` (the gripper's target opening, 0-0.08 m). Both arms' targets take effect on the same tick. Targets hold until the next command |
| `tare(arm)` | zeroes that arm's force/torque sensor at its current reading |
| `step(n_ticks=1)` | advances n x 10 ms (at most 1000 per call); `{sim_time, done, stopped}` |
| `get_camera(name, depth=True)` | `name` in `front`, `left_wrist`, `right_wrist`; `{rgb_png_b64, depth_u16_b64, depth_unit_m, depth_invalid_value, width, height, sim_time, frame_index, T_world_cam}` (`T_world_cam`: 4x4, OpenCV convention, at capture). `decode_frame(reply)` gives (uint8 HxWx3, float32 HxW metres with NaN where invalid). Each call uses one frame of the episode's budget |
| `finish()` | ends the episode (the launcher calls it when `solve` returns); the grader then runs the drive in and inspects it |

Nothing else about the station is on the wire: not the belt's state, not the tensioner's angle, not whether a grip
slipped. Calls other than these answer `{"error": "forbidden"}` and are counted.

## Errors

Every refusal is an answer carrying `error` (the client raises `CellError` with it); nothing is ignored silently.

| error | when |
|---|---|
| `bad_params: <detail>` | a missing, malformed, out-of-range or non-finite argument; an unknown arm; a `q` target outside `joint` mode, a `tcp_pos`/`tcp_quat` target outside `cartesian` mode or without its partner; `step(n)` with n < 1 or n > 1000; a `stiffness_scale` outside [0.2, 1.0]; a `width` outside [0, 0.08] m |
| `episode_over` | `set_control` or `command` after the episode ended (`step` and `get_state` keep answering, with `done`) |
| `camera_budget`, `camera_unavailable` | the 600 frames used; a frame the cell could not render (not counted) |
| `forbidden` | a call outside this table (counted as an illegal call) |
| `internal_error` | the cell failed inside a call (the episode may end) |

A line that is not a JSON object with a `method` (or carries non-finite numbers) ends the episode.

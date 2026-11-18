# Skid cell API (`cell`), version pump-skid-cell/0.6.0

Your program (for instance `/home/user/submission/operate.py` exposing `solve(cell) -> None`) talks to a cell through
`cell`. Two services run the cell: the graded episode's service, the one shift that is scored, which you drive
yourself from scripts of your own through `skidcell.client.connect_graded()` (no program is run for you there, and it
has no program-time, CPU or silence budget and no import check: the instruction gives its rules), and the
commissioning service on the series' development sites (below). `~/commission/run_shift.py` runs a program in a
commissioning shift the way a grader runs one: in a fresh process, from a per-run directory holding a copy of your
submission folder (regular files only: symbolic links and `__pycache__` are not copied), with an empty writable
`$HOME` / `$TMPDIR`, a minimal environment (`PATH`, `HOME`, `TMPDIR`, `LANG`, `MUJOCO_GL=osmesa`, `PYTHONPATH` =
`~/given/lib`, `ARB_GIVEN_DIR` = `~/given`, BLAS threads 2) and the launcher `python3 -B -E -s -c <bootstrap> <lib>
operate` (the bootstrap puts `<lib>` on `sys.path` and calls `skidcell.client.main`, which imports `operate` from the
current directory), or `bash run.sh` when you ship one; its CPU time is capped at 4x its program time.
`cell` is a proxy (`skidcell.client.Cell`): every method is one JSON-lines request, on the process's original stdout
and answered on its stdin in a program that `run_shift.py` runs (`print()` then goes to stderr), on a Unix socket
through `connect_graded()` and `connect_commissioning()`. A reply carrying `error` raises
`skidcell.client.CellError`.

Units: metres, radians, seconds, quaternions `wxyz`; the world frame is the skid's frame (`cell_manual.md` section 1).
The robot's own state is exact on the wire; **nothing about the skid is**: no valve position, no gauge, lamp, level
or display reading, no device pose, not even which device the gripper holds. You see the skid through the cameras. The
one document on the wire is the site's equipment list (`describe()["site"]`).

| call | returns | notes |
|---|---|---|
| `describe()` | `cell_version`, `units`, `robot` (joints and ranges, home posture, tcp, gripper), `rail` (axis, ranges, speeds), `control` (tick, rate limits, stiffness range, modes, the Cartesian frame), `grip_rule` (the pinch box, the axis tolerance, the slip limits), `wrist_load` (what the estimate is, its frame and noise), `cameras` (size, exact intrinsics, the fixed cameras' poses, the wrist camera's mount), `skid` (front plane, extent), `site` {`name`, `equipment`: one entry per device {`series_tag`, `site_tag`, `service`, and for the valves `startup_position`}, `log_sheet`: the site tags to log}, `schedule` (startup deadline, shift end, sim cap, the bands), `budgets`, `safety` (the protective stop's limits, its lock, the damage limit, the stops a shift allows) | static for the episode |
| `get_state()` | `sim_time`, `done`, `arm` {`q`[7], `dq`[7], `cmd_q`[7], `tcp_pos`[3], `tcp_quat`[4], `mode`, `stiffness_scale`, `gripper_width`, `holding`, `wrist_load` {`force_N`[3], `torque_Nm`[3]}, `protective_stop` {`active`, `remaining_s`, `count`, `locked_out`}}, `rail` {`x`, `lift`, `x_goal`, `lift_goal`}, `horn` | `holding`: the gripper holds a handle (the grip rule engaged); `wrist_load`: the external wrench on the hand estimated from the joint torques, world axes, force at the tcp and moment about the tcp, what the surroundings exert on the robot (contacts and a held handle alike; noise and a posture-dependent offset: `cell_manual.md` section 2); `protective_stop`: the robot's stop state and the shift's count (`cell_manual.md` section 4); `horn`: an alarm sounds (which one: the panel's lamps and the instruments) |
| `set_control(mode, stiffness_scale)` | ok | `mode` in `joint`, `cartesian`; `stiffness_scale` in [0.2, 1.0] scales the seven joints' position gains and damping |
| `command(q=[7])` or `command(tcp_pos=[3], tcp_quat=[4])`, optional `width` | ok / error | targets are held until the next command; joint targets are clamped to the ranges and rate-limited to 1.5 rad/s; Cartesian targets (world frame) go through the cell's damped-least-squares differential IK on the `panda/tcp` site (<= 0.5 m/s, 1.5 rad/s, the joint rate limit); `width` (m, 0..0.08) ramps at 0.05 m/s; closing on a handle engages the grip rule, opening wider releases it. In a protective stop a command with a target answers `{"ok": false, "error": "protective_stop", "remaining_s": ...}` (`locked_out` once the robot is out of service) and moves nothing; `width` alone still works |
| `command_rail(x=None, lift=None)` | ok, `x_goal`, `lift_goal` | the carriage's goal along the rail (x) and the lift's height (the Panda base's z); clamped to the ranges, ramped at 0.30 m/s and 0.15 m/s; refused in a protective stop, as `command` |
| `step(n_ticks)` | `sim_time`, `done`, `events[]` | advances n x 10 ms (5 x 2 ms physics steps each) of the cell AND the process; nothing moves and no time passes outside `step`; n <= 1000 per call; events: `grasped` / `released` / `slipped` (the grip rule; the hand feels the grip, it does not know the device), `protective_stop:<robot link>:<force>N` / `protective_stop_cleared` (the safety watch's stop and its end, 5 s later), `safety:damage:<robot link>:<force>N` / `safety:locked_out` (the robot is out of service for the rest of the shift), `horn_on` / `horn_off`, `window_open` (600 s), `shift_end` (1500 s), `physics_fault`, `episode_over` |
| `get_camera(name, depth=True)` | `camera`, `frame_index`, `sim_time`, `width`, `height`, `format` (`png`), `rgb_png_b64`, with depth `depth_u16_mm_b64`, `depth_unit_m` (0.001), `depth_invalid_value` (0); for `wrist` also `T_world_cam` (4x4, OpenCV convention) | one frame of `overview` or `wrist` at the current state; at most `budgets.camera_frames` frames per episode over all cameras, then `{"error": "camera_budget"}`; rendering takes about 0.2-1 s of wall clock per frame (not charged to your program time) |
| `log_reading(tag, value, unit)` | ok, `tag`, `sim_time`, `round`, `in_window` | one entry on the operator's log sheet (`sop.md` 2.3): `tag` the SITE tag of one of PI-101, PI-103, PDI-111, FI-105, LG-201, AI-201 (`describe()["site"]["log_sheet"]`); `unit` one of bar, psi, kPa, m3/h, L/min, %, mg/L; `round` (1, 2 or null) and `in_window` follow from the clock alone: the reply says neither whether the value is right nor whether the entry counts (only an entry made while P-101 runs does) |
| `finish()` | ok | ends the episode; the grader then scores the episode's record |

`skidcell.client.decode_frame(reply)` returns `(rgb uint8 H x W x 3, depth float32 H x W metres, NaN = no reading)`.

Errors: a malformed request answers `bad_params: ...`; a call outside the list answers `forbidden` (counted as an
illegal call); `episode_over` after the episode ended (`step` still answers, nothing moves). An internal error of the
cell while it answers a call that changes the cell ends the episode. A frame the cell could not render answers
`camera_unavailable` and is not charged to the frame budget: ask again. The graded episode is never run again: an
episode the cell ended (an internal error, a physics fault) is scored as it stands.

Budgets per episode: 1800 s of simulated time and 600 camera frames. A program that `run_shift.py` runs in a
commissioning shift also has 1200 s of **program time** (the wall clock minus the cell's own time inside `step` and
`get_camera`), 4x that in CPU seconds, and 180 s of silence (no request) ends its shift; after the shift is over it may
still read state and must finish. The graded episode has none of these three (`describe()["budgets"]` lists them as a
program run has them): its clock advances only inside `step`, the time between your requests costs it nothing, and
once it is over its service answers `status()` and `finish()` only.

No call returns ground truth.

**Commissioning** (`cell_manual.md` section 8; development only). The commissioning service answers the same calls on
the series' development sites, plus `reset(site)`, the first call of a connection: it starts the connection's one
shift on development site `site` (1-9) and answers `ok`, `site`, `site_name`, `shift` (its number in the window) and
`shifts_left`. `skidcell.client.connect_commissioning(site)` opens a connection and calls it. Errors: `bad_params` (no
such site), `no_shift` (a call before `reset`), `one_shift_per_connection`, `commissioning_window_used` (every shift of
the window is used), `busy` (as many shifts open as the service takes at a time), `practice_closed` (the graded
episode has started: commissioning has ended for the session, and an open shift ends with the event below, reason
`practice_closed`). When a budget ends the shift the
service sends `{"id": null, "event": "shift_over", "reason": ...}` and closes the connection (the client raises
`CellError`). The commissioning window's count: `/run/skidcell/commission_status.json`. Nothing else differs from the
graded cell: no truth, no score, no debrief.

# Finishing cell API (`cell`), version shoe-lacing-and-boxing-cell/0.1.0

Your program `/home/user/submission/packer.py` exposes `solve(cell) -> None`. The grader runs it once per hidden
episode in a fresh process under a dedicated unprivileged account, from a per-run directory holding a root-owned
READ-ONLY copy of your submission folder (regular files only: symbolic links and `__pycache__` are not copied), with an
empty writable `$HOME` / `$TMPDIR`, a minimal environment (`PATH`, `HOME`, `TMPDIR`, `LANG`, `MUJOCO_GL=osmesa`,
`PYTHONPATH` = a read-only copy of `~/given/lib`, `ARB_GIVEN_DIR` = a read-only copy of `~/given`, BLAS threads 2) and
the launcher `python3 -B -E -s -c <bootstrap> <lib> packer` (the bootstrap puts `<lib>` on `sys.path` and calls
`shoecell.client.main`, which imports `packer` from the current directory), or `bash run.sh` when you ship one.
`cell` is a proxy (`shoecell.client.Cell`): every method is one JSON-lines request on the process's original stdout,
answered on stdin by the grader's physics server; `print()` goes to stderr. A reply carrying `error` raises
`shoecell.client.CellError`. The same proxy serves your development and the graded episode:
`LocalCell(seed=...)` (`shoecell.local`: a practice cell in your own process, plus `truth()`), `~/practice/grade.py`
(runs your program over the real wire against the practice cell, as the grader runs it) and
`shoecell.client.connect_graded()` (the graded episode, section "The graded episode" below; the program graded is
your submission folder as it was when the graded episode started).

Units on the API: metres, radians, seconds, quaternions `wxyz`; the cell frame = the bench-top centre, z up, x along
the bench, y across it, the arms behind the +y edge (see `cell_manual.md`). `arm` is `"left"` (the gripper arm) or
`"right"` (the hook arm). The arms' joint and tool state is exact; nothing about the objects is on the wire.

| call | returns | notes |
|---|---|---|
| `describe()` | `cell_version`, `units`, `robots` (per arm: base pose, joint names and ranges, home posture, tcp site, tool), `control` (tick, rate limits, stiffness range, modes), `tools` (the attach rules), `cameras` (name -> size, exact intrinsics, the fixed cameras' poses, the wrist cameras' mounts), `zones` (infeed tray, blank spot, cradle nominal pose and seat, outfeed line), `objects` (the batch: shoe, lace, box and lid dimensions), `work_order` (pairs per episode, credits, what arrives), `budgets`, `safety` | static for the episode; no object pose, no eyelet position, no lace, lock or lid state |
| `get_state()` | `sim_time`, `done`, `arms {left: {q[7], dq[7], cmd_q[7], gripper_width, tcp_pos[3], tcp_quat[4], mode, stiffness_scale, holding}, right: {q, dq, cmd_q, tcp_pos, tcp_quat, mode, stiffness_scale, hook_loaded}}`, `clamp_engaged`, `pairs_shipped`, `infeed_pair` (the number of the pair at work, or null) | joint angles, the tcp poses and the gripper width are exact; `holding` is the gripper's grasp signal (the fingers stopped on an attached object); `hook_loaded` is the hook clip's sensor |
| `set_control(arm, mode, stiffness_scale)` | ok | `mode` in `joint`, `cartesian`; `stiffness_scale` in [0.2, 1.0] scales the seven joint position gains |
| `command(arm, q=[7])` or `command(arm, tcp_pos=[3], tcp_quat=[4])`, optional `width` (left only) | ok / error | targets are held until the next command; joint targets are clamped to the ranges and rate-limited to 1.5 rad/s; Cartesian targets go through the cell's damped-least-squares differential IK on the arm's `tcp` site (<= 0.5 m/s, 1.5 rad/s, joint rate limit); the gripper `width` (m, 0..0.08) ramps at 0.05 m/s and the cell grasps what the documented attach rule allows (`cell_manual.md`) |
| `hook(on)` | ok, `loaded` | closes (`on=true`) or opens the hook tip's clip: closing catches an aglet whose centre is within 12 mm of the tip segment and draws it into a seat along the tip over 0.3 s (an aglet pulled with more than 6 N for 30 ms slips out of the clip or the pinch); opening releases it (so does closing the gripper on the aglet while it sits on the tip) |
| `release_clamp()` | ok | opens the cradle clamp (a seated shoe is clamped automatically 0.3 s after it comes to rest in the seat) |
| `step(n_ticks)` | `sim_time`, `done`, `events[]` | advances n x 10 ms of physics (5 x 2 ms inside); nothing moves outside `step`; n <= 500 per call; events: `safety:<what>`, `pair_arrived:<k>`, `box_shipped:<k>`, `aglet_slipped:<arm>`, `physics_fault`, `episode_over` (no grasp, seat, lace, lock or pack events) |
| `get_camera(name, depth=True)` | `camera`, `frame_index`, `width`, `height`, `format` (`png`), `rgb_png_b64`, and with depth `depth_u16_mm_b64`, `depth_unit_m` (0.001), `depth_invalid_value` (0) | one frame of `overhead`, `front`, `side`, `wrist_left` or `wrist_right` rendered at the current physics state: the PNG colour image and a 16-bit PNG depth image (z-depth along the optical axis, millimetres, registered to the colour pixels, 0 = no reading); at most `budgets.camera_frames` frames per episode over all cameras, then `{"error": "camera_budget"}`; rendering takes about 0.1-0.4 s of wall clock per frame and counts against the episode's wall-clock cap |
| `finish()` | ok | ends the episode; the grader then reads the cell's own state |

`shoecell.client.decode_frame(reply)` returns `(rgb uint8 H x W x 3, depth float32 H x W metres, NaN = no reading)`.

Anything else answers `{"error": "forbidden"}`, counts as an illegal call and does nothing. In particular `truth()`
and `reset(seed)` (the practice cell's development calls), `get_state_privileged()` and `reference_run()` (the
grader's reference controller) are not available to your program.

## Budgets and aborts

Everywhere, per episode: 900 s of simulated time, at most 500 ticks per `step`, 450 camera frames (the cell's own
limits). A program (on the grader's hidden episodes and on the practice wire) also has 1200 s of program time per
episode (the episode's wall clock minus the cell's own time inside `step`, simulating, and inside `get_camera`,
rendering); 120 s from a reply to the next request (silence) aborts the episode; a CPU-time backstop of 4 x the
program-time cap applies per process; every process of the run's account is killed when the episode ends; a program
that breaks the wire (crash, malformed line, exit, or not reading a reply within the silence limit) ends ITS episode
with what it achieved so far and the other episodes still run. Once the episode is over (sim cap, physics fault) the
program may still read its state and `finish`; after 50 more `step` calls, 200 more requests or 60 s the episode
closes.

## The graded episode (`connect_graded`)

`shoecell.client.connect_graded(path=None, timeout=None)` connects to the graded service (unix socket
`/run/arb-online/cell.sock`) and returns a `shoecell.client.GradedCell`: the proxy above, from any process of yours
(every call of the table, the same replies and errors; `timeout`: seconds per request, None = as long as the request
takes), plus:

| call | returns | notes |
|---|---|---|
| `status()` | `started`, `ended`, `end_reason` (`done` = the episode's own end, `finish`, `graded_window` or `worker_restart`; null while it runs), `steps` (your `step` calls), `sim_time`, `camera_frames_used`, `prep_window_s`, `graded_window_s`, `prep_left_s` (time left to start the graded episode), `graded_left_s` (time left in its window once started), `can_start` | answers always, before, during and after the episode; never the score |
| `start()` | `ok`, `started`, `graded_window_s` | starts the graded episode: once per session, charged before anything is simulated, never given back; refused a second time (`already_started`) and after the start window (`prep_window_closed`) |
| `close()` | | closes the connection (so does leaving a `with connect_graded() as cell:` block); the episode stays where it is |

Before `start()` every call of the table answers `not_started`. One client at a time: while another connection is
open, `connect_graded()` raises `CellError("busy: ...")`; a connection that sends no request for 15 minutes is closed
by the service. The episode lives in the service: a later connection resumes it as it was (the last commands stay
commanded; nothing moves between `step` calls). It ends at its own end (sim cap, physics fault), at `finish()`, when
its window (`graded_window_s` from the start) runs out, or when the service's worker restarts (it is then not
resumed); after that every call but `status()` answers `episode_over` (after the episode's own end the cell still
answers reads, and up to 50 more `step` calls / 200 requests, as on the wire). An ended episode is scored as it
stands, on the cell's own state, as an episode whose program stopped at that moment; nothing is simulated afterwards.
There is no program-time, silence or CPU limit on the graded episode: a request takes the wall clock its simulation
and rendering take, and the time between your requests costs no simulated time. Its errors carry a short
explanation: `CellError("<code>: <what it means>")`.

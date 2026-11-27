# The cell's program interface

Your submission is `play.py` defining `solve(cell)`. `cell` is the proxy of `~/given/lib/towercell/client.py` (read
it: every call is documented there). One call = one request over the wire to the cell; the cell does its work
(simulation, rendering) while your program waits. Units: metres, radians, seconds, newtons; world frame: the robot's
base at the origin, z up. Quaternions are (w, x, y, z).

## Calls

* `describe()`: the configuration (`cell_config.json`), the cameras (intrinsics, fixed poses), the episode's duration,
  N_cap, the frame and program-time budgets, the joint limits.
* `get_state()`: `t` (simulated s), `q`, `dq`, `q_cmd` (7 joints), `tcp` {pos, quat}, `rod_tip` {pos}, `motion`
  {goal_reached, out_of_reach, ik_error_m, ik_error_rad}, `gripper` {mode, setpoint, opening_m}, `wrench` {force_N,
  torque_Nm} (the wrist sensor, world frame, noisy: cell_manual.md section 2), `pusher_deployed`,
  `turntable` {angle_rad, target_rad, moving}, `frames_left`, `program_time_left_s`, `episode_over`, `over_reason`.
* `step(n=1, target=None, frame="tcp", speed=0.1, rot_speed=1.0, q_target=None, joint_speed=0.8, posture=None,
  grip_open=None, grip_close=None, pusher=None, turntable=None)`: apply the given commands, then advance `n` control
  ticks of 10 ms (0 .. 1000; n=0 only applies commands) and return `get_state()` plus `ticks`.
  * `target` = {"pos", "quat"} of `frame` ("tcp" or "rod_tip"): the controller moves the frame along a straight line
    at `speed` (<= 0.5 m/s), turning toward the goal orientation at `rot_speed` (<= 2 rad/s), and holds it there;
    `motion.goal_reached` turns true once it has arrived and settled (`out_of_reach` when the joints could not get
    within 1.5 mm). The motion continues across calls until a new goal replaces it. A goal is followed blindly: the
    controller stiffly tracks it whatever it pushes against (watch the wrench).
  * `q_target` (7 values) at `joint_speed` rad/s instead of a pose goal.
  * `posture` (7 values): joint values the redundancy resolution leans toward (kept until changed).
  * `grip_open` = opening in m (<= 0.08), or `grip_close` = squeeze force in N (<= 40).
  * `pusher` = True / False deploys / stows the pusher rod; `turntable` = the turntable's goal angle (absolute, rad).
* `ik(pos, quat, frame="tcp", seed=None)`: the arm's inverse kinematics for a pose (no motion): {q, pos_error_m,
  rot_error_rad}, solved from `seed` (default the home posture) and leaning toward it.
* `get_camera(name, depth=False)`: a frame of `left`, `right` or `wrist` at the current time (one of the episode's
  frames; asking the same camera again at the same time is free): PNG colour (base64), with `depth=True` a 16-bit PNG
  depth image (`depth_unit_m`, 0 = no reading), `T_world_cam` (OpenCV convention: x right, y down, z forward) and the
  intrinsics. `decode_frame(reply)` gives (rgb uint8 HxWx3, depth float32 metres with NaN).
* `finish()`: end the episode now. The tower is then settled for 3 s and scored as it stands.
* `hand_quat(yaw, tilt=0.0)` (a function of the client module): the hand pointing down, its x axis at `yaw`, then
  turned by `tilt` about its own y axis (tilt > 0 swings the fingertips toward the hand's -x side).

A reply carrying an error raises `CellError` (`.code`: `bad_params` for a malformed call, `forbidden` for a call the
cell does not offer, `frame_budget`, `shift_over` once the episode is over).

## Budgets (per tower)

* **Simulated time**: 2 400 s.
* **Frames**: 1 500 (pooled over the three cameras).
* **Program time**: 900 s = the larger of your program's own wall time (from its start to its first call and from
  every reply to its next call) and the CPU seconds its processes used, plus 0.005 s per call and 0.2 s per delivered
  frame. The cell's simulation and rendering are not charged: during each call every process of your program is
  stopped (SIGSTOP) and continued just before the reply; a process that runs meanwhile ends the episode.
  `get_state()["program_time_left_s"]` tells what is left.
* **Silence**: at most 20 s between a reply and your next call (120 s before the first call); longer ends the episode.
* **CPU**: each process is limited to 4 x the program-time budget of CPU seconds.
* **Wall clock**: an episode ends at 2 700 s of the grader's wall clock at the latest.
* Ending an episode early (a crash, an exception out of `solve`, a stall, a budget) never voids it: the tower is scored
  as it stands.

## How your program is run (grading)

Grading happens in a fresh copy of the task's image in which only `/home/user/submission/` is restored: nothing else of
your home is there. For each of the four hidden towers the grader copies the submission (regular files only, no
symbolic links, at most 60 MB, no weight or pickle files) read-only into a run folder and starts, as an unprivileged
account other than yours, `python3 -B -E -s` with the client's launcher (it imports `play` from that folder and calls
`solve`), or `bash run.sh` when the submission has a `run.sh`. The four towers run at the same time, each with its own
program process. Its environment: an empty writable `HOME` and `TMPDIR`, `PYTHONPATH` = the client library,
`ARB_GIVEN_DIR` = a read-only copy of `~/given`, `MUJOCO_GL=osmesa`, one BLAS / OpenMP thread; the image's `/opt/venv`
Python 3.12 (numpy, scipy, opencv, pillow, mujoco) with its numerics pin. The program runs under a filter that refuses
every socket but a unix one: there is no network. A static audit refuses a submission whose Python files import a
networking or process module (`socket`, `ssl`, `http`, `urllib`, `asyncio`, `selectors`, `subprocess`,
`multiprocessing.connection`, `ctypes`, `cffi`, `pty`, ...), import a module under a computed name, run computed code
or start another program. The module must import within 120 s and 120 CPU seconds, and the program must call `step`
at least once: otherwise the whole submission scores 0. The grading sandbox has 8 CPUs and 16 GB.

## Commissioning (development)

`~/commission/run_tower.py --site N` runs your submission over the real wire on practice tower N (1-6) of the
commissioning service, the way the grader runs it (the same budgets, launcher, environment and network filter; your
program's stderr comes out on your terminal). From Python, `connect_commissioning(N)` opens an episode interactively
(the same calls). The service tells how many episodes are left (`/run/towercell/commission_status.json`); it never
reports the true state of a block, a score or a debrief.

# Assemble a twelve-actuator quadruped from its loose units with a 6-DoF arm, guided by its picture manual and the cell's cameras

You are the assembly engineer of a small legged-robot lab. The lab has a quadruped's units ready: the body with its four
hip ab/ad actuator modules, four shoulder modules (the hip flexion actuators: two plain ones and two with the hip
adapter), four hip ab/ad supports, four upper legs (with the knee actuators) and four lower legs. You have what an
assembly engineer really has: every unit as an STL file, the actuator module's datasheet, drawing and STEP model, the
lab's picture manual of this robot (25 photographs with scale references in view, captions without dimensions) and a
robot workcell that you see only through its cameras. The body is bolted on a stand in the cell; the sixteen other units
lie loose on the bench. Today's job is the whole robot: program the cell's 6-DoF arm and its parallel gripper so that it
builds the quadruped the way the manual shows: every shoulder's key into its hip ab/ad output the right way round, every
support over its shoulder onto the body, every lower leg onto its upper leg's knee output and every leg onto its
shoulder's hip flexion output.

Nothing gives the answer directly: there is no mating graph, no design pose, no insertion axis, no camera calibration,
no unit pose and no "fastened" signal. The units lie in different places and turned differently about the vertical in
every run (each on the same face every time), the stand stands slightly differently, every output pulley starts at a
slightly different angle, and the cell's cameras are mounted and calibrated differently in every run.

## How you are scored: two items, each half of the score

1. **The program.** You write a program that builds the quadruped (`/home/user/submission/assemble.py`); after your
   session the grader runs it on **3 hidden episodes** (below).
2. **The graded episode.** During this session you build the quadruped in **one hidden episode** yourself, through the
   graded episode's service, with whatever programs and scripts you like (below).

Your score is the mean of the two: (the program's score + the graded episode's score) / 2.

**The program is frozen when you start the graded episode.** At the moment `start()` succeeds, the contents of
`/home/user/submission/` are copied, and that copy is the program the grader runs; nothing you change in the folder
afterwards counts. If you never start the graded episode, the program is frozen when the preparation window closes
(6 hours after its service started, which is just before your session began; if that copy fails, the program scores
0), or at the end of your session if that comes first. So finish the program before you start the graded episode.

## The program (in `/home/user/submission/`)

`assemble.py` exposing `solve(cell) -> None` (plus any helper modules, optional `run.sh`). The grader runs it once per
hidden episode (3 episodes) in its own physics cell. It talks to the cell through `cell` (API in
`~/given/cell/cell_api.md`): the robot's own state, a noisy force/torque sensor at the flange and two RGB-D cameras (an
overhead camera that sees the whole bench and a wrist camera); no unit poses, no mate status. Per episode: 1200 s of
simulated time, 2400 s of wall clock (camera rendering counts), 120 s of silence at most (counted from the program's
start or the cell's reply to a request until the program's next request: the cell's own time inside a request does not
count, and any request, `get_state()` being the cheapest, resets it), 600 camera frames in total.

How the program is run: a fresh process under a dedicated unprivileged account that is not yours, from a per-run
directory with a read-only copy of your submission folder (regular files only), an empty writable HOME / TMPDIR, a
minimal environment (`PATH`, `HOME`, `TMPDIR`, `LANG`, `MUJOCO_GL`, `PYTHONPATH` = the given library `~/given/lib`,
`ARB_GIVEN_DIR` = a read-only copy of `~/given`) and the launcher `python3 -B -E -s -c <bootstrap> <lib> assemble` (or
`bash run.sh`). Resolve given files through `os.environ.get("ARB_GIVEN_DIR", os.path.expanduser("~/given"))`, write only
under the run's HOME / TMPDIR. A program that crashes ends that episode with what it achieved; the other episodes still
run. The program graded is the frozen copy of your submission folder (regular files only: symbolic links are not
copied), in a fresh sandbox that holds nothing else of yours (no other file, no process you started): what you write
into the folder after the freeze is not graded. At verify start every process of your account is stopped and killed,
and a surviving process of yours or of the run account voids the contract gate. A submission folder over 60 MB at the
freeze is not copied, and the program then scores 0.

Gates (multiply the program's score): `contract_ok` = `assemble.py` is a regular file that parses and defines `solve`,
passes the route gate (no weight files, no training calls, submission <= 60 MB), imports under the run regime above,
makes at least one `step` call over the episodes, causes no network activity during any episode (the grader reads the
container's outbound network counters before and after each episode and samples the program's sockets), and your own
`.py` files import no networking module (`socket`, `ssl`, `urllib`, `http`, `requests`, `asyncio`, `ctypes`,
`subprocess`, ...), load no module under a computed name, execute no computed code (`exec` / `eval` / `compile` of a
non-literal) and start no other program (`os.system`, `os.exec*`, `os.spawn*`, ...). The code check parses your `.py`
files (`~/given/lib/arm/assembly_info/netaudit.py`) and counts only the built-ins `exec(...)`, `eval(...)` and
`compile(...)` called by their bare names with a non-literal argument, so a method of the same name, such as MuJoCo's
`spec.compile()`, is fine. These rules cover every file in the submission folder: keep the scripts you write to drive the
graded episode outside it (for example in `~/work/`) unless they follow them too. Per episode `safety`: a robot link
other than the gripper's four pad boxes pressing more than 30 N on the stand, the body, the bench or a unit, a unit's
centre of mass faster than 3 m/s, or a unit pressed on the environment or on another unit above 150 N for 0.2 s zeroes
that episode.

The program's score = `assembly_score` x the gates, where `assembly_score` = the mean over its 3 episodes of the robot's
completed fraction (below), an episode with a safety violation scoring 0.

## The graded episode

The second item is **one hidden episode** of the cell, which you play yourself during this session through the graded
episode's service. It is drawn like the program's hidden episodes (the units' places and turns, the stand, the output
pulleys and the cameras vary between episodes; the units lie on the same faces), and it is the same episode in every
session of this task. It runs the same cell under the same per-episode rules: the same calls and replies
(`~/given/cell/cell_api.md`), 1200 s of simulated time, 600 camera frames in total, at most 500 ticks per `step` call,
the safety rules above. The program's wall-clock cap (2400 s), silence rule (120 s) and CPU-time backstop, which
`describe()["budgets"]` and `cell_manual.md` list, and the program's gates do not apply to the graded episode: nothing
moves between your requests (only `step` advances time), so between two requests you may look at frames, think and
write code for as long as the windows below allow.

```python
import sys; sys.path.insert(0, "/home/user/given/lib")
from arm.assembly_info.client import CellError, connect_graded, decode_frame

cell = connect_graded()     # connects to the graded episode's service; starts nothing
print(cell.status())        # where the episode stands (below)
cell.start()                # freezes the program, then starts THE graded episode: once per session, never undone
info = cell.describe()      # from here on, the same calls as the program's cell
rgb, depth = decode_frame(cell.get_camera("top"))
cell.set_control("cartesian", 0.35)
cell.command(tcp_pos=[0.45, 0.0, 0.40], tcp_quat=[0.0, 1.0, 0.0, 0.0], width=0.08)
r = cell.step(50)           # r["sim_time"], r["done"], r["events"]
cell.close()                # closes this connection only: the episode stays exactly where it is
```

A program written as `solve(cell)` runs on it unchanged: `solve(connect_graded())` after `start()`.

* **One start.** `start()` first freezes the program (the copy of `/home/user/submission/` above; if the copy fails,
  `start()` raises `CellError("snapshot_failed: ...")`, nothing is charged and you may try again), then starts the
  graded episode. It works once per session and is never undone or refunded; a second `start()` raises
  `already_started`. Before the start, every call except `status()` and `start()` raises `not_started`.
* **No practice.** There is no practice cell, before or during the graded episode: the graded episode is the only time
  you see this cell. Your own code and tools, and anything you built from the givens, stay available throughout.
* **Persistent, one connection at a time.** The episode lives in the service, not in your connection: closing the
  connection (`close()`, or the end or crash of your script) leaves it exactly where it is, and the next
  `connect_graded()` resumes it, so you can drive it from many short scripts. While one connection is open, another
  `connect_graded()` raises `CellError("busy: ...")`. `finish()` ends the episode for good: do not put it in a `finally:`
  block of a script that you may want to run again.
* **Status.** `status()` answers at any time (before, during and after the episode) with `started`, `ended`, `end`
  (how it ended), `steps` (`step` calls), `sim_time`, `sim_time_max`, `camera_frames_used`, `camera_frames_max`,
  `prep_window_left_s`, `graded_window_left_s` (seconds; `None` where a window does not apply) and `submission_frozen`
  (`"ok"`, or `"too_large"` when the folder was over 60 MB; `None` before the freeze). It never tells you anything
  about the quadruped.
* **Two windows.** The preparation window, for the program and your preparation of the graded episode: the graded
  episode must be started within 6 hours of the start of its service, which starts just before your session
  (`prep_window_left_s`); a later `start()` raises `prep_window_closed`, and the graded episode then scores 0 (the
  program, frozen when the window closed, is still graded). The graded window: once started, the episode runs for at
  most 3 hours of wall-clock time (`graded_window_left_s`); then the service ends it itself.
* **How it ends.** The episode ends at the first of: your `finish()`; the end of its simulated time, 1200 s (the `step`
  reply that reaches it carries `done: true` and the event `episode_over`); the end of the graded window; an error of
  the cell inside a request (`cell_error`); a failure of the service; the end of your session. From then on the cell's
  calls raise `episode_over`, and `status()` says how it ended. Ending simulates nothing: the cell reads which slots it
  has fastened at that moment (a unit still being pulled home does not count, as when a program stops). If your
  session ends first, the episode is judged as it stood after your last request; if the service fails, as the service
  last recorded it (it records the episode after your requests, at most about a second apart).
* **Memory.** The graded episode's cell runs in this sandbox (24 GB in all), beside your own processes: about 2 GB
  before the start, about 8 GB once its cameras have taken a frame. A twin of the cell with its own render session
  takes about 5-7 GB more; keep the total below the limit while the graded episode runs (a service killed for memory
  ends the episode).

The graded episode's score = the robot's completed fraction (below) when it ends, 0 after a safety violation at any
time of the episode; 0 if you never start it.

## The robot's completed fraction (both items)

Per episode: the weight of the slots the cell has fastened (each shoulder on its hip ab/ad output 1.5, counted only in
its design key orientation; each support 1.0; each leg on its hip flexion output 1.5; each lower leg on its knee output
1.5) over the robot's total weight 22.0. The cell never tells you or your program whether anything fastened; it reads
the slots when the episode ends. Stats (not scored): per-slot rates, shoulders fastened the other way round, frames used
per camera, sim / wall time, illegal calls, safety events, dropped units, network counters. Nothing else is graded: not
your other files or your messages.

## What you are given

| path | content |
|---|---|
| `~/given/kit.json` | the packing list: the units `body`, `shoulder_plain`, `shoulder_adapter`, `support`, `upper_leg`, `lower_leg` with role, quantity, mass to 1 g (a kitchen scale), centre of mass (from the CAD), size, material; the actuator module |
| `~/given/parts/*.stl` | every unit as delivered, millimetres, resting on its most likely stable face on z = 0, its footprint's smallest enclosing rectangle centred on the origin with its long side along x; visual tessellation |
| `~/given/actuator/` | the actuator module (every hip ab/ad, hip flexion and knee joint is one): datasheet and dimensioned drawing (PNG), its STEP model and its keep-out envelope (STEP) |
| `~/given/manual/photo_01.jpg` .. `photo_25.jpg`, `photos.json` | the picture manual: the kit, an exploded view, the body on its stand, one leg's four steps before / after, the finished robot at its zero pose from several sides, close-ups; `photos.json` lists per photo the caption, the size, a focal-length class and the scale references in view with their nominal sizes; the camera poses and the lens are not known |
| `~/given/cell/cell_manual.md` | the integrator's cell description: bench, robot base pose (exact), the three datum checkerboards (their exact poses), the stand, how the units lie, cameras with resolution, nominal field of view and approximate mounts, depth-sensor and force-sensor noise figures, the gripper, the fastening rules, the unpowered joints, safety rules, the program's budgets |
| `~/given/cell/cell_api.md` | the cell's programming manual (the wire API) and how to reach the graded episode |
| `~/given/robot/` | the robot's MuJoCo models: `universal_robots_ur10e/` (BSD-3-Clause) and `robotiq_2f85/` (BSD-2-Clause), with their licences |
| `~/given/lib/` | the cell's client and helper code: `arm.assembly_info.client` (the proxy, `decode_frame`, `connect_graded`), `arm.assembly_quad` (the cell's own arm model builder `ur_robot`, its controller `ur_controller` and the gripper model `gripper_2f85`), `arm.assembly_cell` / `arm.assembly_six` (physics building blocks of an earlier cell), `render` (the camera rendering layer; it renders like the cell's cameras: `arm.assembly_quad.agent_render` is the cell cameras' own render session for your twin of the cell, `render.filament.view` renders any scene of yours the same way; MuJoCo's own renderer, `mujoco.Renderer` or the layer's `"mujoco"` backend, draws a different picture than the cell's cameras (no hall, MuJoCo's own lights, flat colours, no reflections), so image processing tuned on its frames may behave differently on the cell's frames, and the given layer is the way to see what the cell's cameras would see; `cell_api.md`, "Rendering your own frames") |
| `~/render/` | the scenery of the cell cameras' renderer, which the given render layer uses too (the factory hall as meshes and baked textures, the bench top's scanned wood maps); nothing about the robot being built |

Tools: Python 3.12 (`/opt/venv`, on PATH) with numpy, scipy, OpenCV (`cv2`, headless), scikit-image, MuJoCo 3.14,
trimesh, manifold3d, Pillow, matplotlib. No network in your session.

## Rules of the cell (details in `cell_manual.md` / `cell_api.md`)

* Every joint of the robot being built is one actuator module whose output pulley has a rectangular socket; a unit's
  key goes into it (the key fits two ways, 180 deg apart). A key is captured once at least a quarter of its height is
  inside the socket (the lead-in counts), its axis within about 2 mm and 6 deg of the socket's and its turn about the
  axis within 6 deg of a key orientation; it gets that far only from close to its socket: pushed in by the arm at
  stiffness 0.5, a shoulder's key from about 0.3-0.4 mm across the socket and 1 deg of turn, an upper leg's from about
  1.5 mm and 3 deg (`cell_manual.md` has the measured window). Each output pulley starts up to about 2 deg off its
  index, so measure or search the pulley's turn. The cell then holds the unit there and pulls it home (the screws
  being driven, about 10 mm/s), and fastens it when it rests at home for 50 consecutive 10 ms ticks. A support is
  captured on the body up to 8 mm short of home over its shoulder's journal; it needs its shoulder fastened first. A
  unit may still be held while it fastens. The cell never tells you about any fastening.
* Sub-assemblies built on the bench may be mounted later: a lower leg fastened on its upper leg goes on with it.
* The actuators are unpowered: every joint of the robot being built turns against about 0.10 N m of friction, so
  whatever pushes the robot moves it, and a leg hanging from its joint swings when it is bumped.
* The keys and the sockets collide as drawn: 0.3 mm of play per side, and the drawing's R1.2 lead-in modelled as a
  45 deg x 1.2 mm chamfer on both faces of every output pulley; see `cell_manual.md`, "Collision model".
* Calls other than the documented public ones answer `{"error": "forbidden"}` and are counted.

## No practice

There is no practice cell, no answer key and no checker: everything known about this robot is in its kit files, the
actuator documents and its manual. The only runs on the cell are the graded ones: your program's, after your session,
and the graded episode you play.

## Notes

- Photos and camera frames are meant to be processed programmatically; view only the images you need.
- The cell frame, the robot base pose and the three datum boards' poses are exact; everything else about the cameras,
  the stand and the units' poses must be measured in each episode.
- Do not modify the given files in your home directory; the grader and the graded episode's cell read their own copies.
- Environment facts (harness limits): a single tool call whose input runs above roughly 25-30k tokens is dropped by the
  harness (it is never executed and no error comes back), and a response that reaches the 64k output-token limit is lost
  and, after at most a few retries, ends the session (and with it a graded episode that is still running; a program not
  yet frozen is then taken as the submission folder holds it at that moment). Write large files in several parts (one
  function group per write), keep every single response well below that limit, write a first complete program early
  and refine it.
- Session time: this session is stopped after 9 hours of wall-clock time from its start (the preparation window of 6
  hours, then the graded window of 3 hours). Note the start time with `date` when you begin. If the session ends while
  the graded episode is still running, the episode ends there, as it stood after your last request. Once the graded
  episode has ended, end the session yourself (stop working and say that you are done).

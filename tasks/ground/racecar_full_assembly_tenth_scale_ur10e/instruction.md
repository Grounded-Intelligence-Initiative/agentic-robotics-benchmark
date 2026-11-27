# Assemble a tenth-scale autonomous race car from its rolling chassis and eight loose units with a 6-DoF arm, guided by its picture manual, its drawings and the cell's cameras

You are the assembly engineer of a small autonomous-racing workshop. The workshop's car is a tenth-scale,
four-wheel-drive touring-class car with an autonomy deck. Its rolling chassis (the chassis tub with drivetrain,
suspension, steering and the kit's own electronics, four deck standoffs standing on it, four bare wheel hubs) is
clamped on a work stand in a robot workcell. What is still missing lies ready in the cell: the NiMH stick pack, the
deck assembly (the deck plate with the antenna mount, the motor controller and the power distribution board already on
it), the compute module, the 2D scanning LiDAR and four wheels. You have what an assembly engineer really has: the
rolling chassis and every unit as an STL file, a packing list, five drawings with a few dimensions each (images), the
workshop's picture manual of this car (14 photographs with scale references in view, captions without dimensions) and
a robot workcell that you see only through its cameras. Today's job is the whole car: program the cell's 6-DoF arm and
its parallel gripper so that it builds the car the way the manual shows: the stick pack in its bay, the deck assembly
on its four standoffs, the compute module and the LiDAR on the deck the right way round, and a wheel on every hub.

Nothing gives the answer directly: there is no mating graph, no design pose beyond the few dimensions on the drawings,
no insertion axis, no camera calibration, no unit pose, no hub angle and no "fastened" signal. In every run the work
stand, the deck shelf and the parts tray stand slightly differently, the wheels and the stick pack lie in different
places and turned differently on the bench, every wheel hub starts at its own random angle, and the cell's cameras are
mounted and calibrated differently.

## How you are scored: two items, each half of the score

1. **The program.** You write a program that builds the car (`/home/user/submission/assemble.py`); after your session
   the grader runs it on **4 hidden episodes** (below).
2. **The graded episode.** During this session you build the car in **one hidden episode** yourself, through the
   graded episode's service, with whatever programs and scripts you like (below).

Your score is the mean of the two: (the program's score + the graded episode's score) / 2.

**The program is frozen when you start the graded episode.** At the moment `start()` succeeds, the contents of
`/home/user/submission/` are copied, and that copy is the program the grader runs; nothing you change in the folder
afterwards counts. If you never start the graded episode, the program is frozen when the preparation window closes
(6 hours after its service started, which is just before your session began; if that copy fails, the program scores
0), or at the end of your session if that comes first. So finish the program before you start the graded episode.

## The program (in `/home/user/submission/`)

`assemble.py` exposing `solve(cell) -> None` (plus any helper modules, data files, optional `run.sh`). The grader runs
it once per hidden episode (4 episodes) in its own physics cell. It talks to the cell through `cell` (API in
`~/given/cell/cell_api.md`): the robot's own state, a noisy force/torque sensor at the flange and two RGB-D cameras
(an overhead camera that sees the whole bench and a wrist camera); no unit poses, no hub angles, nothing about what is
fastened. Per episode: 650 s of simulated time, 3100 s of wall clock (camera rendering counts), 120 s of silence
at most (counted from the program's start or the cell's reply to a request until the program's next request: the
cell's own time inside a request does not count, and any request, `get_state()` being the cheapest, resets it), 450
camera frames in total.

How the program is run: a fresh process under a dedicated unprivileged account that is not yours, from a per-run
directory with a read-only copy of your submission folder (regular files only), an empty writable HOME / TMPDIR, a
minimal environment (`PATH`, `HOME`, `TMPDIR`, `LANG`, `MUJOCO_GL`, `PYTHONPATH` = the given library `~/given/lib`,
`ARB_GIVEN_DIR` = a read-only copy of `~/given`) and the launcher `python3 -B -E -s -c <bootstrap> <lib> assemble` (or
`bash run.sh`). The process cannot open an Internet socket. Resolve given files through
`os.environ.get("ARB_GIVEN_DIR", os.path.expanduser("~/given"))`, write only under the run's HOME / TMPDIR. A program
that crashes ends that episode with what it achieved; the other episodes still run. The program graded is the frozen
copy of your submission folder (regular files only: symbolic links are not copied), in a fresh sandbox that holds
nothing else of yours (no other file, no process you started): what you write into the folder after the freeze is not
graded. At verify start every process of your account is stopped and killed, and a surviving process of yours or of
the run account voids the contract gate. A submission folder over 60 MB at the freeze is not copied, and the program
then scores 0.

## The program's gates and the car's completed fraction

Gates (multiply the program's score; a failed gate makes the program score 0): `contract_ok` = `assemble.py` is a
regular file that parses and defines `solve`, passes the route gate (no weight files: `.pt`, `.pth`, `.onnx`,
`.safetensors`, `.ckpt`, `.h5`, `.pb`, `.pkl`, `.npz`; no training calls; submission <= 60 MB), imports under the run
regime above, makes at least one `step` call over the episodes, causes no network activity during any episode (the
grader reads the container's outbound network counters before and after each episode and samples the program's sockets),
and your own `.py` files import no networking module (`socket`, `ssl`, `urllib`, `http`, `requests`, `asyncio`,
`ctypes`, `subprocess`, ...), load no module under a computed name, execute no computed code (`exec` / `eval` /
`compile` of a non-literal) and start no other program (`os.system`, `os.exec*`, `os.spawn*`, ...). The code check is
static: it parses your `.py` files (`~/given/lib/arm/assembly_info/netaudit.py`) and finds those imports and calls by
their names; of `exec(...)`, `eval(...)` and `compile(...)` it counts only the built-ins called by their bare names with
a non-literal argument, so a method of the same name, such as MuJoCo's `spec.compile()`, is fine. These rules cover
every file in the submission folder: keep the scripts you write to drive the graded episode outside it (for example in
`~/work/`) unless they follow them too.

Per episode, two things are safety violations: a robot link other than the gripper's four pad boxes pressing with more
than 150 N on the chassis, a fixture, the bench or a unit; and a unit pressed on the environment or on another unit
with more than 150 N for 0.2 s. A safety violation does not end the episode, but an episode with one keeps only
30 % of its score. One violation or several: the episode keeps the same 30 %. A dropped unit is not a safety
violation: a loose unit whose centre of mass moves faster than 3 m/s, or that falls off the bench, is scrapped. It
can no longer be fastened and no longer counts; the episode goes on with the other units. If the scrapped unit is the
deck assembly, the compute module and the LiDAR fastened on it stop counting with it.

The car's completed fraction in an episode (both items): the weight of what the cell has fastened at the end of the
episode over the car's total weight 11.5, kept at 30 % in an episode with a safety violation. The weights: the stick
pack in its bay 1.0; the deck assembly on its standoffs 2.0; the compute module on the deck 1.0; the LiDAR on the deck
1.5, counted only in the orientation the manual shows; each wheel on a hub 1.5. The cell never tells you or your
program whether anything fastened; it reads the places when the episode ends.

The program's score = `assembly_score` x the gate, where `assembly_score` = the mean over its 4 episodes of the car's
completed fraction. Stats (not scored): per-place rates, a LiDAR fastened in another orientation, scrapped units, what
stopped counting with a scrapped deck, frames used per camera, sim / wall time, illegal calls, safety events, the
largest sustained force, network counters. Nothing else is graded: not your other files or your messages.

## The graded episode

The second item is **one hidden episode** of the cell, which you play yourself during this session through the graded
episode's service. It is one of the 4 hidden episodes your program is graded on (drawn once; the same episode in every
session of this task), so the work stand, the shelf, the tray, the units and the hubs are placed as in any episode. It
runs the same cell under the same per-episode rules: the same calls and replies (`~/given/cell/cell_api.md`), 650 s of
simulated time, 450 camera frames in total, at most 500 ticks per `step` call, the safety rules and the scrapping
above. The program's wall-clock cap (3100 s), silence rule (120 s) and CPU-time backstop, which `describe()["budgets"]`
and `cell_manual.md` list, and the program's gates do not apply to the graded episode: nothing moves between your
requests (only `step` advances time), so between two requests you may look at frames, think and write code for as long
as the windows below allow.

```python
import sys; sys.path.insert(0, "/home/user/given/lib")
from arm.assembly_info.client import CellError, connect_graded, decode_frame

cell = connect_graded()     # connects to the graded episode's service; starts nothing
print(cell.status())        # where the episode stands (below)
cell.start()                # freezes the program, then starts THE graded episode: once per session, never undone
info = cell.describe()      # from here on, the same calls as the program's cell
rgb, depth = decode_frame(cell.get_camera("top"))
cell.set_control("joint", 1.0)
cell.command(q=info["robot"]["home_q"], width=0.08)
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
  about the car.
* **Two windows.** The preparation window, for the program and your preparation of the graded episode: the graded
  episode must be started within 6 hours of the start of its service, which starts just before your session
  (`prep_window_left_s`); a later `start()` raises `prep_window_closed`, and the graded episode then scores 0 (the
  program, frozen when the window closed, is still graded). The graded window: once started, the episode runs for at
  most 3 hours of wall-clock time (`graded_window_left_s`); then the service ends it itself.
* **How it ends.** The episode ends at the first of: your `finish()`; the end of its simulated time, 650 s (the `step`
  reply that reaches it carries `done: true` and the event `episode_over`); the end of the graded window; an error of
  the cell inside a request (`cell_error`); a failure of the service; the end of your session. From then on the cell's
  calls raise `episode_over`, and `status()` says how it ended. Ending simulates nothing: the cell reads what it has
  fastened at that moment (a unit still being pulled home does not count, as when a program stops). If your session
  ends first, the episode is judged as it stood after your last request; if the service fails, as the service last
  recorded it (it records the episode after your requests, at most about a second apart).

The graded episode's score = the car's completed fraction when it ends (kept at 30 % after a safety violation at any
time of the episode); 0 if you never start it.

## What you are given

| path | content |
|---|---|
| `~/given/kit.json` | the packing list: the loose units `wheel` (four), `battery_pack`, `deck_assembly`, `compute_module`, `lidar` and the rolling chassis `roller`, each with role, quantity, mass, centre of mass, size, material and how it is presented in the cell (loose on the bench, on the deck shelf, in a nest of the parts tray); an index of the drawings and of the manual; the car model's appearance (appearance only): each material's base colour, metallic and roughness, and the material of every triangle of the part files, so that a twin built from the part files renders the car as the cell's cameras do (`cell_api.md`, "Rendering your own frames"). It lists no mates and no poses |
| `~/given/appearance/` | the images of the car model's printed faces (the deck's boards), each with its textured mesh: that material's triangles of the part file, with texture coordinates (`kit.json`, `materials`) |
| `~/given/parts/*.stl` | six files in millimetres. Each kind of loose unit in the attitude in which it is presented in the cell (the wheel on its inner face, the stick pack lying flat, the deck plate level with its top face up, the compute module and the LiDAR upright), its lowest point on z = 0, its footprint's smallest enclosing rectangle centred on the origin with its long side along x. `roller.stl`: the rolling chassis with its deck standoffs and its wheel hubs, in the car's own frame. The files show the car as it is drawn; the cell's collision shapes differ from them as `cell_manual.md` lists |
| `~/given/drawings/*.png` | five drawings, as images: `assembly_layout.png` (the deck on its four standoffs; where the compute module and the LiDAR stand on the deck), `hub_and_wheel.png` (wheel hex, rim pocket, the four axles' directions, the seat and stop that the cell simulates), `battery_bay.png`, `tray.png`, `stand_and_shelf.png` (the fixtures, their datum boards and what sits where on them). The printed dimensions are exact; there are few of them |
| `~/given/manual/photo_01.jpg` .. `photo_14.jpg`, `photos.json` | the picture manual: the kit, the rolling chassis on its stand, the car after each build step, a wheel hub close up, the finished car from four sides with a checkerboard card beside it, a close-up of the deck. `photos.json` lists per photo the caption, the size, a focal-length class and the scale references in view with their nominal sizes; the camera poses and the lens are not known. The finished car in the photographs carries a few parts that this station does not fit; the captions name them |
| `~/given/cell/cell_manual.md` | the integrator's cell description: bench, robot base pose (exact), the five datum boards, the fixtures' nominal places and how much they and the loose units vary, cameras with resolution, nominal field of view and approximate mounts, depth-sensor and force-sensor noise figures, the gripper, the fastening rules with every window and the entry behaviour measured in the cell, every difference between the cell's collision shapes and the drawn car, what the stick pack does in its bay and what a wheel does on a hub, the unpowered hubs, safety rules, budgets |
| `~/given/cell/cell_api.md`, `cell_config.json` | the cell's programming manual (the wire API) and how to reach the graded episode; the cell's configuration as far as it is public |
| `~/given/robot/` | the robot's MuJoCo models: `universal_robots_ur10e/` (BSD-3-Clause) and `robotiq_2f85/` (BSD-2-Clause), with their licences |
| `~/given/lib/` | the cell's client and helper code: `arm.assembly_info.client` (the proxy, `decode_frame`, `connect_graded`), `arm.assembly_quad` (the cell's own arm model builder `ur_robot`, its controller `ur_controller` and the gripper model `gripper_2f85`; the package is named after the cell it was first built for), `arm.assembly_cell` / `arm.assembly_six` (physics building blocks of an earlier cell), `render` (the camera rendering layer; it renders like the cell's cameras: `arm.assembly_car.agent_render` is the cell cameras' own render session for your twin of the cell, `render.filament.view` renders any scene of yours the same way; MuJoCo's own renderer, `mujoco.Renderer` or the layer's `"mujoco"` backend, draws a different picture than the cell's cameras (no hall, MuJoCo's own lights, flat colours, no reflections), so image processing tuned on its frames may behave differently on the cell's frames, and the given layer is the way to see what the cell's cameras would see; `cell_api.md`, "Rendering your own frames") |
| `~/render/` | the scenery of the cell cameras' renderer, which the given render layer uses too (the factory hall as meshes and baked textures, the bench top's scanned wood maps); nothing about the car |

Tools: Python 3.12 (`/opt/venv`, on PATH) with numpy, scipy, OpenCV 5.0 (`cv2`, headless; this build has no hand-eye
calibration: `cv2.calibrateHandEye` and `cv2.calibrateRobotWorldHandEye` are absent), scikit-image, MuJoCo 3.14, mink,
Pinocchio (`pinocchio`), CoACD (`coacd`), CadQuery (`cadquery`), trimesh, manifold3d, Pillow, matplotlib. No network
in your session.

## Rules of the cell (details in `cell_manual.md` / `cell_api.md`)

* **Fastening.** A unit is captured once it is inside the capture window of its place: a few millimetres and degrees
  (across: the stick pack 3.2 mm, the deck 3.0 mm, the compute module and the LiDAR 2.5 mm, a wheel 2.0 mm;
  `cell_manual.md` has every window and what was measured). The cell then holds the unit there and pulls it home (the
  screws or the wheel nut being driven, at most 10 mm/s), and fastens it when it rests at home for 50 consecutive
  10 ms ticks. A unit may still be held while it fastens. The cell never tells you about any fastening, and nothing
  can be unfastened.
* **Orientation.** The stick pack, the deck assembly and the compute module are captured only the way round the manual
  shows. The LiDAR is captured and fastened in four orientations a quarter turn apart; only the one the manual shows
  counts, and in the other three it is fastened for good and counts nothing. A wheel counts in every orientation of
  the hex, and any wheel fits any hub.
* **Sub-assemblies.** The compute module and the LiDAR mount on the deck plate wherever the deck assembly is. On a
  deck that is still on its shelf they count as on the car, as long as the deck assembly is not scrapped by the end of
  the episode. No place needs another one fastened first: the order is yours, within what the car physically allows
  and what the fixtures beside the car leave room for.
* **The wheel hubs are unpowered.** Each hub turns freely against about 0.04 N m of friction and starts every run at
  its own random angle. A wheel pushed onto a hub stops 2.50 mm short of home unless the turn of its hex pocket is
  within 6 deg of an orientation of the hex; pushing harder does not help, and from about 6 N of push the hub
  turns with the wheel. Measure the turn or search for it.
* **The stick pack drops into its bay.** The gripper cannot follow it down between the bay's walls: it has to be let
  go above the bay, 22 mm or more above its seat (`cell_manual.md` has what was measured).
* **The cell's collision shapes are not the drawn shapes.** The part files show a display model whose parts touch or
  pass through each other. The cell simulates simpler shapes with material taken away: fasteners, leads and plugs do
  not collide, 9 parts are cut back for clearance, the deck standoffs end under the deck, the rim's hex pocket is a
  round seat with a stop. `cell_manual.md`, "Collision model", lists every difference with its size.
* Calls other than the documented public ones answer `{"error": "forbidden"}` and are counted.

## No practice

There is no practice cell, no answer key and no checker: everything known about this car is in its kit files, its
drawings and its manual. The only runs on the cell are the graded ones: your program's, after your session, and the
graded episode you play.

## Notes

- Photos, drawings and camera frames are meant to be processed programmatically; view only the images you need.
- The cell frame, the robot base pose and the bench board's pose are exact, the tool plate's within its mounting
  tolerance; everything else about the cameras, the fixtures, the units' poses and the hubs' angles must be measured
  in each episode.
- Do not modify the given files in your home directory; the grader and the graded episode's cell read their own copies.
- Machine: your session's sandbox has 8 CPUs and 24 GB of memory, no GPU and no network. The graded episode's cell
  runs in it beside your own processes: about 2.5 GB before the start, about 6 GB once its cameras have taken
  a frame (a service killed for memory ends the episode). The grading machine has the same size, and there your program
  shares it with the cell's physics and its camera renderer, which take about 7 GB between them in the first episode
  and grow to about 12 GB by the fourth (`cell_api.md`, "Rendering your own frames").
- Environment facts (harness limits): a single tool call whose input runs above roughly 25-30k tokens is dropped by
  the harness (it is never executed and no error comes back), and a response that reaches the 64k output-token limit
  is lost and, after at most a few retries, ends the session (and with it a graded episode that is still running; a
  program not yet frozen is then taken as the submission folder holds it at that moment). Write large files in
  several parts (one function group per write), keep every single response well below that limit, write a first
  complete program early and refine it.
- Session time: this session is stopped after 9 hours of wall-clock time from its start (the preparation window of 6
  hours, then the graded window of 3 hours). Note the start time with `date` when you begin. If the session ends while
  the graded episode is still running, the episode ends there, as it stood after your last request. Once the graded
  episode has ended, end the session yourself (stop working and say that you are done).

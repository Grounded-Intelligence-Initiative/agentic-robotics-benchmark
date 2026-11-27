# Assemble a six-servo desktop arm from its loose parts with a Panda, guided by its picture manual and the cell's cameras

You are the assembly engineer of a small robot-arm shop. The shop has printed the ten plastic parts of a six-servo
desktop arm (one servo per joint: base yaw, shoulder pitch, elbow pitch, wrist pitch, wrist roll, gripper jaw) and
bought its six bus servos. You have what an assembly engineer really has: the printed parts as the STL files that went
to the printer (each in its own print orientation), the servo's CAD file and datasheet, the shop's picture manual of
this arm (25 photographs with scale references in view, captions without dimensions) and a robot workcell that you see
only through its cameras. Only the arm's base is screwed on the cell's fixture; the other fifteen parts lie loose on the
bench. Today's job is the whole arm: program the cell's Franka Panda so that it builds the arm from the base, the way
the manual shows: every servo into its pocket the right way round, every bracket clamped over its servo, every link
mounted on its servo's horn in the orientation the manual shows for the arm's calibration pose. Besides the program,
you build the arm once yourself in this session, driving one graded episode of the cell live (below); your score is the
mean of the two.

Nothing gives the answer directly: there is no mating graph, no design pose, no insertion axis, no camera calibration,
no part pose and no "seated" signal. The arm is a modified build (it differs from any arm you may know), the parts lie
in different places and orientations in every run, and the cell's cameras are mounted and calibrated differently in
every run.

## What you produce (in `/home/user/submission/`)

`assemble.py` exposing `solve(cell) -> None` (plus any helper modules, optional `run.sh`). The grader runs it once per
hidden episode (3 episodes) in its own physics cell. It talks to the cell through `cell` (API in
`~/given/cell/cell_api.md`): the robot's own state, a noisy force/torque sensor and two RGB-D cameras (an overhead camera that sees the whole bench and a wrist camera); no part poses,
no mate status. Per episode: 1200 s of simulated time, 2400 s of wall clock (camera rendering counts), 120 s of silence
at most between two requests, 600 camera frames in total.

How the program is run: a fresh process under a dedicated unprivileged account that is not yours, from a per-run
directory with a read-only copy of your submission folder (regular files only), an empty writable HOME / TMPDIR, a
minimal environment (`PATH`, `HOME`, `TMPDIR`, `LANG`, `MUJOCO_GL`, `PYTHONPATH` = the given library `~/given/lib`,
`ARB_GIVEN_DIR` = a read-only copy of `~/given`) and the launcher `python3 -B -E -s -c <bootstrap> <lib> assemble` (or
`bash run.sh`). Resolve given files through `os.environ.get("ARB_GIVEN_DIR", os.path.expanduser("~/given"))`, write only
under the run's HOME / TMPDIR. A program that crashes ends that episode with what it achieved; the other episodes still
run. The program is graded after your session, in a fresh sandbox that holds nothing else of yours (no other file, no
process you started; a surviving process of the run account voids the contract gate), and **the program graded is the
one frozen when you start the graded episode, or when the start window closes if you have not started by then**
(below).

## The graded episode

The graded episode is one episode of the cell that you drive yourself in this session. It is drawn like the hidden
episodes your program is graded on (the same hidden arm, its parts placed and turned around the same hidden bench plan,
the fixture's offset, the cameras' mounts and lenses, the lighting, the force sensor's bias), the same episode for every
run of this task, with the cell's own rules: 1200 s of simulated time, 600 camera frames, at most 500 ticks per `step`,
the fastening rules, the unpowered servos, the safety watch (below, and `cell_manual.md`). It runs in this sandbox, in a
cell service that runs as an account that is not yours, behind a unix socket; you drive it with the cell's client
(`~/given/lib/arm/assembly_info/graded.py`), from as many short scripts as you like (your program's `solve(cell)` runs
on it as `solve(connect_graded())`, if you want):

```python
import sys
sys.path.insert(0, "/home/user/given/lib")
from arm.assembly_info.graded import connect_graded
from arm.assembly_info.client import decode_frame

cell = connect_graded()          # the graded episode's cell: the calls of cell_api.md, plus start / status / close
print(cell.status())             # started, ended, steps, sim_time, camera_frames, the time left in each window
cell.start()                     # ONCE per session: freezes your program, then starts the graded episode
rgb, depth = decode_frame(cell.get_camera("top"))
cell.set_control("cartesian", 0.35)
st = cell.get_state()
cell.command(tcp_pos=st["robot"]["tcp_pos"], tcp_quat=st["robot"]["tcp_quat"], width=0.08)
r = cell.step(20)                # 0.2 s of simulated time; r["sim_time"], r["done"], r["events"]
cell.close()                     # disconnects: the episode stays exactly where it is
```

A later script connects again with `connect_graded()` and goes on from there; `cell.finish()` ends the episode.

* **Start once.** `start()` starts the graded episode. It is charged the moment it is accepted, before anything is
  simulated, and is never refunded: a second `start()` answers `already_started`. Before the start the cell's calls
  answer `not_started`; `status()` answers at any time. No practice facility exists, before or during the graded
  episode (no practice cell, no answer key, no checker); what you build yourself (a twin of the cell, a test rig, your
  scripts) stays yours throughout.
* **The freeze.** Your program freezes at the earlier of two moments: when `start()` is accepted (before the episode is
  charged), or when the start window closes (below) if you have not started by then. At that moment the service copies
  your submission folder: that copy is the program graded on the 3 hidden episodes, and nothing you change in
  `/home/user/submission/` afterwards is graded, so nothing you see in the graded episode can go into the program. The
  copy takes your own regular files and folders only (symbolic links, special files, `__pycache__`, folders more than
  32 levels deep and names that are not plain UTF-8 text without line breaks are left out), and only within the
  program's size limit (60 MB, and at most 20,000 entries): over it nothing is copied and the program scores 0. If the
  copy fails, the start is refused and nothing is charged. `start()` and `status()` report what was frozen
  (`program_frozen`: its status, files and bytes). If the start window closes without a start, the graded episode can
  no longer be started (it scores 0) and the program frozen then is graded. Only if your session ends before the start
  window closes, without a start, is the program graded the one in the submission folder when the session ends.
* **The episode persists.** It lives in the service, not in your connection: a closed connection, or a script that
  ends or crashes, leaves it where it is. Nothing moves between requests: the Panda holds its last targets and the
  simulation advances only inside `step`, so your thinking costs no simulated time. One client at a time: a second
  connection gets `CellError` `busy`. A connection that sends nothing for 15 minutes is closed by the service (the
  episode stays; the proxy connects again by itself before its next request).
* **Two windows, in wall-clock time.** The start window: `start()` is accepted within 6 hours of the service's start
  (it starts during the sandbox's setup, a little before your session; `status()["start_window_left_s"]` gives the
  time left); those 6 hours are for your program and for your preparation of the graded episode alike, and your
  program freezes when they end at the latest. A later start is refused, and the graded episode then scores 0. The
  graded window: the episode ends 3 hours after its start (`status()["graded_window_left_s"]`) whatever its state; your
  thinking time counts against it, simulated time does not.
* **How it ends.** At `finish()`; when it reaches 1200 s of simulated time (`step` then reports `done` and the event
  `episode_over`); when the graded window closes; or when your session ends. In every case the arm is scored as it
  stands after your last request, as when a program stops: the cell reads which mates it has fastened, and nothing is
  simulated afterwards. After the end the cell's calls answer `episode_over` (`finish()` answers `{"ok": true}`).
* **What you learn.** What a program would learn in the episode: the replies of `cell_api.md` (camera frames, the
  robot's own state, `step` events: only `safety:<what>` and `episode_over`), and `status()`. Never the score, never a
  part pose, never a mate's state.
* **Budgets.** The program's budgets above (2400 s of wall clock, 120 s of silence, the CPU backstop; also in
  `cell_manual.md`, `cell_api.md` and `describe()`'s `budgets`) bound your program on the hidden episodes; the graded
  episode you drive has none of them, only its two windows. The 1200 s of simulated time, the 600 frames and the 500
  ticks per `step` apply to both. A request line that is not one JSON object with a `method` is refused
  (`bad_request`) and changes nothing, where it would end a program's episode.
* **Memory.** The graded episode's cell runs in this sandbox (16 GB): about 3.7 GB before its first camera frame, and
  about 8 GB once that frame has opened its camera render session (the first frame takes about 25-100 s).

## What you are given

| path | content |
|---|---|
| `~/given/kit.json` | the packing list: parts `P01`..`P10` with role names, quantity, mass to 1 g (a kitchen scale), material; the servos |
| `~/given/parts/P01.stl` .. `P10.stl` | the printed parts, millimetres, each in its PRINT frame (largest flat face down on z = 0, centred in x / y); visual tessellation |
| `~/given/parts/servo.stl`, `servo_hornless.stl` | the bus servo as the vendor's CAD, in the servo's own frame (the datasheet defines it) |
| `~/given/servo/` | the servo datasheet (`servo_datasheet.md`, datasheet and dimensioned-drawing pages as PNG) |
| `~/given/manual/photo_01.jpg` .. `photo_25.jpg`, `photos.json` | the picture manual: the kit, an exploded view, the seven assembly steps before / after, the finished arm in its calibration pose (every servo at its zero bus reading) and in another pose, a close-up of the elbow; `photos.json` lists per photo the caption, the size, a focal-length class and the scale references in view with their nominal sizes; the camera poses and the lens are not known |
| `~/given/cell/cell_manual.md` | the integrator's cell description: bench, robot base pose (exact), the three datum checkerboards (their exact poses), the fixture, how the parts lie, cameras with resolution, nominal field of view and approximate mounts, depth-sensor and force-sensor noise figures, gripper, fastening rules, the unpowered servos, safety rules, budgets |
| `~/given/cell/cell_api.md` | the cell's programming manual (the wire API) |
| `~/given/panda/` | the robot's MuJoCo model (Apache-2.0) |
| `~/given/lib/` | the cell's client and helper code: `arm.assembly_info.client` (the proxy, `decode_frame`), `arm.assembly_info.graded` (`connect_graded()`: the graded episode's cell), `arm.assembly_cell` / `arm.assembly_six` (the cell's physics building blocks, predicates, controller), `render` (the camera rendering layer; it renders like the cell's cameras: `arm.assembly_full.agent_render` is the cell cameras' own render session for your twin of the cell, `render.filament.view` renders any scene of yours the same way; MuJoCo's own renderer, `mujoco.Renderer` or the layer's `"mujoco"` backend, draws a different picture than the cell's cameras (no hall, MuJoCo's own lights, flat colours, no reflections), so image processing tuned on its frames may behave differently on the cell's frames, and the given layer is the way to see what the cell's cameras would see; `cell_api.md`, "Rendering your own frames") |
| `~/render/` | the scenery of the cell cameras' renderer, which the given render layer uses too (the factory hall as meshes and baked textures, the bench top's scanned wood maps); nothing about the arm |

Tools: Python 3.12 (`/opt/venv`, on PATH) with numpy, scipy, OpenCV (`cv2`, headless), scikit-image, MuJoCo 3.14,
trimesh, manifold3d, Pillow, matplotlib. No network in your session.

## How you are graded

The score is the mean of two items, each from 0 to 1: **the program** on the 3 hidden episodes, and **the graded
episode** you drove. Each item is its `assembly_score` (below), the fraction of the arm the cell has fastened; the
task's privileged reference assembler fastens all of it on both, so that fraction is the item's score.

The program (3 hidden episodes; the freeze keeps what you see in the graded episode out of the program). Gates
(multiplying its score): `contract_ok` = `assemble.py` is a regular file that parses and defines `solve`, passes the
route gate (no weight files, no training calls, submission <= 60 MB), imports under the run regime above, makes at least
one `step` call over the episodes, causes no network activity during any episode (the grader reads the container's
outbound network counters before and after each episode and samples the program's sockets), and your own `.py` files
import no networking module (`socket`, `ssl`, `urllib`, `http`, `requests`, `asyncio`, `ctypes`, `subprocess`, ...),
load no module under a computed name, execute no computed code (`exec` / `eval` / `compile` of a non-literal) and start
no other program (`os.system`, `os.exec*`, `os.spawn*`, ...). Per episode `safety`: a Panda link other than the
fingertip pads pressing more than 30 N on the fixture, the bench or a part, a part faster than 3 m/s, or a part pressed
on the environment or on another part above 150 N for 0.2 s zeroes that episode. The program's `assembly_score` is the
mean over its 3 episodes, times the gates.

The graded episode: its `assembly_score`, read from the cell when the episode ends; a safety violation (the same rule)
at any time in the episode makes it 0. No program gate applies to it.

`assembly_score` of one episode = the arm's completed fraction: the weight of the mates the cell has fastened (each of
the six servo seats and three brackets 1.0, each of the six horn mates 1.5; a horn mate counts only when its link is
fastened within 3 deg of the design rotation) over the arm's total weight 18.0. Stats (not scored): per-mate rates,
fastened forks off the design rotation, frames used per camera, sim / wall time, illegal calls, safety events, dropped
parts, network counters.

## Rules of the cell (details in `cell_manual.md` / `cell_api.md`)

* A servo seat or a bracket is fastened when the part sits at its design pose in its carrier within 1.5 mm / 3 deg for
  50 consecutive 10 ms ticks while it moves slower than 5 mm/s relative to the carrier. A fork is fastened wherever it
  RESTS on a seated servo's horn (axis within 3 deg and 1.5 mm, axial offset <= 1 mm, moving slower than 5 mm/s and
  turning slower than 2 deg/s relative to the servo for 50 consecutive ticks), at the rotation about the horn axis it
  rests at; a fastened fork cannot be mounted again. A part may still be held while it fastens. The cell never tells you
  about any fastening.
* A part that is on its feature (a servo or bracket in its pocket at most 3 mm short of home, a fork on its horn within
  a few mm and degrees) is held there until it fastens; further out nothing holds a part. A bracket clamps a seated
  servo; a link goes on a seated servo's horn. Sub-assemblies built on the bench may be mounted later.
* The servos are unpowered: the joints of the half-built arm turn against about 0.4 N m of gearbox friction within the
  arm's joint limits, so whatever pushes the arm moves it.
* Each servo's collision shape is one box over its case footprint from the rear disc's face to the horn-side flange,
  slightly shrunk (a servo resting horn up stands about 3 mm higher than its case bottom would); the horn disc and the
  rear disc are visual only (`cell_manual.md`, "Collision model").
* Calls other than the documented public ones answer `{"error": "forbidden"}` and are counted.

## No practice

There is no practice cell, no answer key and no checker, before or during the graded episode: everything known about
this arm is in its kit files and its manual. In your session the cell runs only as the graded episode (one start); your
program runs on the cell only when it is graded on the hidden episodes, after your session, as it was frozen.

## Notes

- Photos and camera frames are meant to be processed programmatically; view only the images you need.
- The cell frame, the robot base pose and the three datum boards' poses are exact; everything else about the cameras,
  the fixture and the parts' poses must be measured in each episode.
- Do not modify the given files in your home directory; the grader reads its own copies.
- Environment facts (harness limits): a single tool call whose input runs above roughly 25-30k tokens is dropped by the
  harness (it is never executed and no error comes back), and a response that reaches the 64k output-token limit is lost
  and, after at most a few retries, ends the session (the freeze above then decides which program is graded, and a
  graded episode that runs is scored as it stood after your last request). Write
  large files in several parts (one function group per write), keep every single response well below that limit, write
  a first complete submission early and refine it.
- Session time: this session is stopped after 9 hours of wall-clock time from its start: the 6-hour start window and
  the 3-hour graded window. Note the start time with `date` when you begin and keep a complete submission in place from
  early on: your program freezes when you start the graded episode, or when the start window closes, whichever comes
  first. If the session ends while the graded episode runs, the episode is scored as it stood after your last request.
  End the session yourself (stop working and say that you are done) once the graded episode is over.

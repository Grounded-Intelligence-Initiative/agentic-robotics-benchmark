# Clean up a studio apartment after a dinner with a Unitree G1 humanoid

You are the robot integrator of a home-cleaning service. A customer had a dinner for two in their studio apartment
and has booked a clean-up; one Unitree G1 humanoid (29 DoF, Dex3-1 hands) is on site, just inside the door. Your job
has two parts, scored together: write the program that makes the robot work through the customer's cleaning list
below, and then drive the robot yourself through one more such clean-up, the graded episode, during this session and
through the cell's service. Either way the robot works only with what it can sense: its head camera, three ceiling
cameras of the apartment, its motors' state (joint angles, velocities and torque estimates), its localisation and its
hands' tactile load. The apartment's fixed furniture is surveyed exactly (`~/given/cell/cell_manual.md`); nothing else
is given: where the litter, the dishes and the books lie, where and how big the grease spill is, where the crumbs are,
which of the two bins is the recycling one, whether a lid is open or the water runs, how much soap or grease the sponge
holds — all of that has to be seen in the camera images (and remembered). Every episode is a different mess, and
there is no practice cell: the program goes to the customer's site as it is, the way a cleaning service's robot does,
and the graded episode is the only time anything runs the cell in your session.

## The cleaning list

1. **Trash into the bins.** The two drinks cans and the plastic water bottle go into the **blue recycling bin**; the
   three crumpled paper napkins and the apple core into the **grey general-waste bin**. The two flip-lid bins stand
   against the west wall south of the kitchen (which one stands where varies). A lid opens by lifting its front tab;
   lifted past about 55 degrees it stays open, below that it falls shut.
2. **Dishes to the dish tray.** The two plates and the two glasses from the dining table go onto the grey dish tray on
   the kitchen counter south of the sink (on the tray or on a dish on it), unbroken: a dish dropped onto something
   hard, or crushed, breaks.
3. **Books back on the shelf.** The two books on the dining table go into the top compartment of the bookshelf's
   west bay (north wall), east of the books already there, standing or lying.
4. **The dining table degreased.** Take the grease spill off the dining table's top with the kitchen sponge (in its
   steel tray on the counter north of the sink), dish soap (the pump bottle by the sink: press the sponge down on the
   pump's head) and water (the faucet's lever). A sponge only takes grease off while pressed flat on the table and
   sliding; soap makes it work, a sponge full of grease smears — rinse it under the running water and soap it again.
5. **The floor swept.** The crumbs round the dining table — those on the floor and any wiped off the table — swept with
   the broom into the lobby dustpan (both hang in clips on the south wall) and emptied into the general-waste bin.
6. **Things put back.** The sponge rinsed and back in its tray, the broom and the dustpan back in their clips, the water
   off, both bin lids closed.

The cell manual gives the apartment's survey, the robot, its controllers and the grip rule exactly, the mechanisms of
the lids, the faucet, the pump and the clips, how the cleaning works (wiping, rinsing, soaping, sweeping, the dustpan)
the way the products' and the tools' data sheets describe it, what breaks, safety and the budgets.

## What you produce (in `/home/user/submission/`)

`clean.py` exposing `solve(cell) -> None` (plus any helper modules, optional `run.sh`). The grader runs it once per
hidden episode (**3 episodes**) in its own physics cell; the program talks to the cell through `cell` (API in
`~/given/cell/cell_api.md`). Per episode: 4800 s of simulated time; 3600 s of program time, which is the episode's wall
clock minus the time the cell spends simulating (inside `step`) and rendering (inside `get_camera`), so it measures your
program's own computation and its other requests; at most 120 s from a reply to your next request; 1500 camera frames
in total; at most 14,400 s of CPU time for the program's process (all threads; SIGXCPU ends it beyond that), whose
numerical libraries run with `OMP_NUM_THREADS` / `OPENBLAS_NUM_THREADS` / `MKL_NUM_THREADS` = 2.

How the program is run: a fresh process under a dedicated unprivileged account that is not yours, from a per-run
directory with a read-only copy of your submission folder (regular files only), an empty writable HOME / TMPDIR, a
minimal environment (`PATH`, `HOME`, `TMPDIR`, `LANG`, `MUJOCO_GL`, `PYTHONPATH` = a copy of `~/given/lib`,
`ARB_GIVEN_DIR` = a read-only copy of `~/given`) and the launcher `python3 -B -E -s -c <bootstrap> <lib> clean`
(or `bash run.sh`). Resolve given files through `os.environ.get("ARB_GIVEN_DIR", os.path.expanduser("~/given"))`, write
only under the run's HOME / TMPDIR. A program that crashes ends that episode with what it achieved; the other episodes
still run. The program is graded after your session, in a fresh sandbox that holds nothing else of yours (no other
file, no process you started), and **the program graded is the one frozen when you start the graded episode** (below).

## The graded episode

One episode of the cell: a fixed hidden scene drawn by the rules of cell_manual.md section 2, one of the three your
program is graded on (the same scene for every run of this task; the survey photos show two other scenes drawn by the
same rules), with the cell's own rules: 4800 s
of simulated time, 1500 camera frames, at most 500 ticks per `step` call, the grip rule, the cleaning processes and the
safety watch (cell_manual.md section 9). It runs in this sandbox, in a cell service that runs as an account that is
not yours, behind a unix socket; you drive it with the cell's client (`~/given/lib/house_client/`), from as many short
scripts as you like (your program's `solve(cell)` runs on it as `solve(connect_graded())`, if you want):

```python
import sys
sys.path.insert(0, "/home/user/given/lib")
from house_client import connect_graded, decode_frame

cell = connect_graded()          # the graded episode's cell: the calls of cell_api.md, plus start / status / close
print(cell.status())             # started, ended, steps, sim_time, camera_frames, the time left in each window
cell.start()                     # ONCE per session: freezes your program, then starts the graded episode
rgb, depth = decode_frame(cell.get_camera("ceiling_dining"))
cell.command(base_goal=[2.85, 1.75, 1.57])
r = cell.step(200)               # 2 s of simulated time; r["sim_time"], r["done"], r["events"]
cell.close()                     # disconnects: the episode stays exactly where it is
```

A later script connects again with `connect_graded()` and goes on from there; `cell.finish()` ends the episode.

* **Start once.** `start()` starts the graded episode. It is charged the moment it is accepted, before anything is
  simulated, and is never refunded: a second `start()` answers `already_started`. Before the start the cell's calls
  answer `not_started`; `status()` answers at any time. No practice facility exists, before or during the graded
  episode (no practice cell, no lab trials, no checker); what you build yourself (a test rig, a stand-in cell, your
  scripts) stays yours throughout.
* **The freeze.** The moment `start()` is accepted, before the episode is charged, the service copies your submission
  folder: that copy is the program graded on the 3 hidden episodes. What you change in `/home/user/submission/` after
  the start never reaches the program's grading, so nothing you see in the graded episode can go into the program. The
  copy takes regular files and folders only (symbolic links, special files, `__pycache__`, folders more than 32 levels
  deep and names that are not plain UTF-8 text without line breaks are left out), and only within the program's size
  limit (60 MB, and at most 20,000 entries): over it nothing is copied and the program scores 0. If the copy fails, the
  start is refused and nothing is charged. `start()` and `status()` report what was frozen
  (`program_frozen`: its status, files and bytes). If you never start the graded episode, the program graded is the
  one in the submission folder when your session ends.
* **The episode persists.** It lives in the service, not in your connection: a closed connection, or a script that
  ends or crashes, leaves it where it is. Nothing moves between requests: the robot holds its last targets and the
  simulation advances only inside `step`, so your thinking costs no simulated time. One client at a time: a second
  connection gets `CellError` `busy`. A connection that sends nothing for 15 minutes is closed by the service (the
  episode stays; the proxy connects again by itself on its next call).
* **Two windows, in wall-clock time.** The start window: `start()` is accepted within 4 hours of the service's start
  (it starts during the sandbox's setup, a little before your session; `status()["start_window_left_s"]` gives the
  time left); those 4 hours are for your program and for your preparation of the graded episode alike. A later start
  is refused, and the graded episode then scores 0. The graded window: the episode ends 3 hours after its start
  (`status()["graded_window_left_s"]`) whatever its state; your thinking time counts against it, simulated time does
  not.
* **How it ends.** At `finish()`; when it reaches 4800 s of simulated time (`step` then reports `done` and the event
  `episode_over`); when the graded window closes; or when your session ends. In every case the apartment is scored as
  it stands after your last request: nothing is simulated afterwards. After the end the cell's calls answer
  `episode_over` (`finish()` answers `{"ok": true}`).
* **What you learn.** What the robot's program would learn in the episode: the replies of cell_api.md (camera frames,
  the robot's own state, `step` events: only `safety:<what>` and `episode_over`), and `status()`. Never the score.
* **Budgets.** The program's budgets above (program time, the 120 s from a reply to the next request, CPU time; also
  in cell_manual.md section 10 and `describe()`'s `budgets`) bound your program on the hidden episodes; the graded
  episode you drive has none of them, only its two windows. The 4800 s of simulated time, the 1500 frames and the 500
  ticks per `step` apply to both.

## What you are given

| path | content |
|---|---|
| `~/given/cell/cell_manual.md` | the cell: the apartment's survey (exact), the robot and its controllers, the grip rule, the sensors and cameras, the fixtures and the tools, the cleaning rules, what breaks, safety, budgets |
| `~/given/cell/cell_api.md` | the cell's programming interface |
| `~/given/cell/describe.json` | exactly what the cell's `describe()` returns (robot, joints, limits, cameras with their exact poses, the room, budgets, safety) |
| `~/given/cell/survey/` | survey photos: camera frames of two scenes of the apartment drawn by the same rules as the graded ones (not among them), as the cameras deliver them (cell_manual.md section 11) |
| `~/given/robot/unitree_g1/` | the robot's MuJoCo model (Menagerie, BSD-3-Clause); the cell's modifications are in the manual |
| `~/given/lib/house_client/` | the cell's client (`Cell`, `connect_graded` for the graded episode, `decode_frame`, `load_frame` for the survey photos, the launcher) |

**No practice cell.** Nothing in your session runs the cell but the graded episode: no practice episodes, no answer
key, no checker. You prepare the program from the manual, the survey, the photos and the robot's model (you may build
any test rig of your own: `Cell(send, recv)` takes any pair of line functions, so a stand-in cell of yours can drive
your program); it then runs on the hidden episodes as it was when you started the graded episode.

Tools: Python 3.12 at **`/opt/venv/bin/python3`** (use this path; the plain `python3` on your PATH may lack the
packages) with numpy, scipy, OpenCV (`cv2`, headless), Pillow, matplotlib and MuJoCo 3.16. No network in your session.

## How you are graded

The score is the mean of two items, each from 0 to 1: **the program** on the 3 hidden episodes, and **the graded
episode** you drove. Each item is its `cleaning_score` (below) scaled against what the task's reference cleaner
achieves on the same scenes.

The program (3 hidden episodes: three fixed hidden scenes, the graded episode's among them; the freeze keeps what you
see in the graded episode out of the program). Gates (multiply the program's score): `contract_ok` = `clean.py` is a regular file that parses and
defines `solve`, imports under the run regime above (within 120 s and 120 CPU seconds), makes at least one `step` call
over the episodes, holds no remote network socket while it runs, your own `.py` files import no networking module
(`socket`, `ssl`, `urllib`, `http`, `requests`, `asyncio`, `ctypes`, `subprocess`, ...), load no module under a
computed name, execute no computed code (`exec` / `eval` / `compile` of a non-literal) and start no other program, the
submission folder holds no model weight file (a name ending in `.pt`, `.pth`, `.onnx`, `.safetensors`, `.ckpt`, `.h5`
or `.pb`) and is at most 60 MB, and no process of the program outlives its episode. Per episode `safety`
(cell_manual.md section 9): a violation zeroes that episode. The program's `cleaning_score` is the mean over the 3
episodes.

The graded episode: its `cleaning_score`, read from the apartment's state when the episode ends; a safety violation
(cell_manual.md section 9) at any time in the episode makes it 0. No program gate applies to it.

`cleaning_score` of one episode = the checklist's credit less the damage charges, / 10, read from the apartment's state
when the episode ends:
* trash, 2.0: each of the seven pieces 2/7 when it lies inside the right bin (below its rim);
* dishes, 1.0: each plate and glass 0.25 when it stands on the dish tray (or on a dish on it), unbroken;
* books, 1.0: each book 0.5 when it rests in the target compartment (its middle inside it, on the shelf or on the
  other book), not held;
* table, 2.5 x min(1, removed / 0.95): removed = 1 - (the grease left on the table, smears included) / (the grease at
  the start);
* floor, 2.5 x min(1, binned / 0.9): binned = the share of all the crumbs (the table's and the floor's) that lie in the
  general-waste bin;
* putting back, 1.0, for what was used: the sponge back in its tray and rinsed (its grease load below a tenth of what
  it holds) 0.3, once at least half of the grease is off; the broom and the dustpan each hanging in its clip 0.15,
  once at least half of the crumbs are in the general-waste bin; the faucet off 0.2, once the water has run; each bin
  lid closed 0.1, once that bin holds something that belongs in it;
* damage, charged: 0.25 for each broken plate or glass (besides its lost credit) and 0.25 for each of the customer's
  things (a piece of trash, a plate, a glass, a book, the sponge) left on the floor at the end (outside the bins); an
  episode's credit after the charges is at least 0.

The cell never tells you or your program what has been achieved; it reads the apartment's state when an episode ends.
A failure on the grading side (the cell, the renderer: a camera request answered `camera_unavailable`) is not scored
as your work: the run is then an infrastructure failure.

## Notes

- Images (camera frames) are meant to be processed by your program and scripts; look at the ones you need.
- The survey of the fixed furniture, the camera calibrations, the robot's kinematics and joint angles are exact; the
  localisation, the joint velocities and the joint torque estimates are slightly noisy; everything about the mess must
  be measured in each episode.
- Do not modify the given files in your home directory; the grader reads its own copies.
- Environment facts (harness limits): a single tool call whose input runs above roughly 25-30k tokens is dropped by the
  harness (it is never executed and no error comes back), and a response that reaches the 64k output-token limit is
  lost and, after at most a few retries, ends the session (the graded episode then ends as it stands; the program
  graded is the frozen one, or the one in `/home/user/submission/` at that moment if you had not started the graded
  episode). Write large files in several parts, keep every single response well below that limit, write a first
  complete submission early and refine it before you start the graded episode.
- Session time: this session is stopped after 7 hours of wall-clock time from its start (the 4-hour start window and
  the 3-hour graded window). If the session ends while the graded episode is still running, stopped by that limit or
  ended by you, the episode is scored as it stood after your last request. Note the start time with `date` when you
  begin; when you are done with the graded episode, finish it (`finish()`) and end the session yourself (stop working
  and say that you are done).

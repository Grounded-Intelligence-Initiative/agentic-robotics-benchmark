# Assemble a flat-pack bookcase from its drawings with two Unitree G1 humanoids

You are the robot integrator of a furniture-assembly service. A customer has bought the **B930 bookcase**; it has
been delivered and unpacked in their room, and two Unitree G1 humanoids (29 DoF, Dex3-1 hands) are on site. Your job:
make the two robots build the bookcase **exactly as its drawings show** and stand it **where the drawings' floor plan
says**, using only what the robots can sense: their head cameras, two ceiling cameras of the room, their joint
encoders, their localisation and their hands' tactile load. Several steps are marked for two persons; one humanoid
alone cannot hold a tall board upright and push another board into it at the same time.

Nothing gives the answer directly: no part list with poses, no part identities, no assembly graph, no "seated" signal.
Which board is which, which way round it goes, in what order, and where each one ends up are in the drawings and in
the camera images. The parts lie in different places and orientations in every episode.

## How you are scored: two items, each half of the score

1. **The program.** You write a program that builds the bookcase (`/home/user/submission/assemble.py`); after your
   session the grader runs it on **3 hidden episodes** (below).
2. **The graded episode.** During this session you play **one hidden episode** yourself, through the graded episode's
   service, with whatever programs and scripts you like (below).

Your score is the mean of the two: (the program's score + the graded episode's score) / 2.

**The program is frozen when you start the graded episode.** At the moment `start()` succeeds, the contents of
`/home/user/submission/` are copied, and that copy is the program the grader runs; nothing you change in the folder
afterwards counts. If you never start the graded episode, the program is frozen when the preparation window closes
(4 hours after its service started, which is just before your session began; if that copy fails, the program scores
0), or at the end of your session if that comes first. So finish the program before you start the graded episode.

## The program (in `/home/user/submission/`)

`assemble.py` exposing `solve(cell) -> None` (plus any helper modules, optional `run.sh`). The grader runs it once
per hidden episode (**3 episodes**) in its own physics cell; the program talks to the cell through `cell` (API in
`~/given/cell/cell_api.md`). Per episode: 1500 s of simulated time; 3000 s of program time, which is the episode's wall
clock minus the time the cell spends simulating (inside `step`) and rendering (inside `get_camera`), so it measures your
program's own computation and its other requests; at most 120 s from a reply to your next request; 900 camera frames
in total.

How the program is run: a fresh process under a dedicated unprivileged account that is not yours, from a per-run
directory with a read-only copy of your submission folder (regular files only), an empty writable HOME / TMPDIR, a
minimal environment (`PATH`, `HOME`, `TMPDIR`, `LANG`, `MUJOCO_GL`, `PYTHONPATH` = a copy of `~/given/lib`,
`ARB_GIVEN_DIR` = a read-only copy of `~/given`) and the launcher `python3 -B -E -s -c <bootstrap> <lib> assemble`
(or `bash run.sh`). Resolve given files through `os.environ.get("ARB_GIVEN_DIR", os.path.expanduser("~/given"))`, write
only under the run's HOME / TMPDIR. A program that crashes ends that episode with what it achieved; the other episodes
still run. The frozen copy of your submission folder (regular files only: symbolic links are not copied) is graded in
a fresh sandbox that holds nothing else of yours (no other file, no process you started). A submission folder over 60
MB at the freeze is not copied, and the program then scores 0.

Gates (multiply the program's score): `contract_ok` = `assemble.py` is a regular file that parses and defines
`solve`, imports under the run regime above, makes at least one `step` call over the episodes, causes no network
activity, and your own `.py` files import no networking module (`socket`, `ssl`, `urllib`, `http`, `requests`,
`asyncio`, `ctypes`, `subprocess`, ...), load no module under a computed name, execute no computed code (`exec` /
`eval` / `compile` of a non-literal) and start no other program; and the submission holds at most 60 MB in all, no
weight file (`.pt`, `.pth`, `.onnx`, `.safetensors`, `.ckpt`, `.h5`, `.pb`, `.pkl`, `.npz`) and no `.py` or `.sh` file
whose text contains `torch.optim`, `.backward(` or `jax.grad` anywhere (comments and strings included). These rules
cover every file in the submission folder: keep the scripts you write to drive the graded episode outside it (for
example in `~/work/`) unless they follow them too. Per episode `safety` (cell_manual.md section 8): a violation zeroes
that episode.

The program's score = the mean over its 3 episodes of the bookcase's credited weight / 9 (below), times the gates.

## The graded episode

The second item is **one hidden episode** of the cell, which you play yourself during this session through the graded
episode's service. It is drawn like the practice episodes and the program's hidden episodes (the parts' places and
facings, the robots' start poses and the sensor noise vary between episodes), it is none of the practice episodes, and
it is the same episode in every session of this task. It runs the same cell under the same per-episode rules: the same
calls and replies (`~/given/cell/cell_api.md`), 1500 s of simulated time, 900 camera frames in total, at most 500 ticks
per `step` call, the safety rules of `cell_manual.md` section 8. The program's budgets of program time (3000 s) and
silence (120 s), which the practice cell applies too, and the program's gates do not apply to the graded episode:
nothing moves between your requests (only `step` advances time), so between two requests you may look at frames,
think and write code for as long as the windows below allow.

```python
import sys; sys.path.insert(0, "/home/user/given/lib")
from furniture_client.client import CellError, connect_graded

cell = connect_graded()     # connects to the graded episode's service; starts nothing
print(cell.status())        # where the episode stands (below)
cell.start()                # freezes the program, then starts THE graded episode: once per session, never undone
info = cell.describe()      # from here on, the same calls as the practice cell
cell.command("a", base_goal=[1.9, 0.9, 1.57])
r = cell.step(50)           # r["sim_time"], r["done"], r["events"]
cell.close()                # closes this connection only: the episode stays exactly where it is
```

A program written as `solve(cell)` (as in the practice example of `cell_api.md`) runs on it unchanged:
`solve(connect_graded())`.

* **One start.** `start()` first freezes the program (the copy of `/home/user/submission/` above; if the copy fails,
  `start()` answers `snapshot_failed`, nothing is charged and you may try again), then starts the graded episode. It
  works once per session and is never undone or refunded; a second `start()` answers `already_started`. Before the
  start, every call except `status()` and `start()` answers `not_started`.
* **Practice ends with the start.** All practice comes before the graded episode: the moment `start()` succeeds, the
  practice cell ends any open practice episode and refuses new ones for the rest of the session (its calls then answer
  `practice_closed`). Your own code and tools, and anything you built from the givens, stay available.
* **Persistent, one connection at a time.** The episode lives in the service, not in your connection: closing the
  connection (`close()`, or the end or crash of your script) leaves it exactly where it is, and the next
  `connect_graded()` resumes it, so you can drive it from many short scripts. While one connection is open, another
  `connect_graded()` raises `CellError("busy")`. `finish()` ends the episode for good: do not put it in a `finally:`
  block of a script that you may want to run again.
* **Status.** `status()` answers at any time (before, during and after the episode) with `started`, `ended`, `end`
  (how it ended), `steps` (`step` calls), `sim_time`, `sim_time_max`, `camera_frames_used`, `camera_frames_max`,
  `prep_window_left_s`, `graded_window_left_s` (seconds; `None` where a window does not apply) and `submission_frozen`
  (`"ok"`, or `"too_large"` when the folder was over 60 MB; `None` before the freeze). It never tells you anything
  about the furniture.
* **Two windows.** The preparation window, for the program and your preparation of the graded episode: the graded
  episode must be started within 4 hours of the start of its service, which starts just before your session
  (`prep_window_left_s`); a later `start()` answers `prep_window_closed`, and the graded episode then scores 0 (the
  program, frozen when the window closed, is still graded). The graded window: once started, the episode runs for at
  most 3 hours of wall-clock time (`graded_window_left_s`); then the service ends it itself.
* **How it ends.** The episode ends at the first of: your `finish()`; the end of its simulated time, 1500 s (the `step`
  reply that reaches it carries `done: true` and the event `episode_over`); the end of the graded window; an internal
  error of the cell inside `step` or `command`; a failure of the service; the end of your session. From then on the
  cell's calls answer `episode_over`, and `status()` says how it ended. Ending simulates nothing: the furniture is
  judged as it stands at that moment. If your session ends first, it is judged as it stood after your last request;
  if the service fails, as the service last recorded it (it records the episode after your requests, at most about a
  second apart).

The graded episode's score = the bookcase's credited weight / 9 when it ends (below); 0 if you never start it.

## The bookcase's credited weight (both items)

Per episode, the credited weight out of 9:
* each of the six dowel joints of the carcass (the top, the fixed shelf and the bottom in each side panel): 1.0 when the
  right board is locked in the right group of holes the right way round, as the drawings show;
* the back panel locked in the rabbets of both side panels the right way round: 1.5;
* the bookcase in its place, 1.5: all six dowel joints credited and the carcass standing where the floor plan puts it
  (within 30 mm and 3 deg, upright within 3 deg).

A safety violation (`cell_manual.md` section 8) at any time of an episode makes that episode's score 0 (the episode
goes on). The cell never tells you or your program whether anything fastened; it reads the furniture's state when the
episode ends. Nothing else is graded: not your practice episodes, your other files or your messages.

## What you are given

| path | content |
|---|---|
| `~/given/drawings/sheet_00_cover.png` .. `sheet_09_floor_plan.png` | the manufacturer's drawing set of the B930: cover with the overall dimensions and the finishes, parts list, dimensioned part drawings, five wordless assembly steps, and the floor plan with the bookcase's place |
| `~/given/cell/cell_manual.md` | the cell: the room survey (exact), how the flat-pack was delivered, the robots and their controllers, the grip rule, the sensors and cameras, the fastening rules, safety rules, the program's budgets |
| `~/given/cell/cell_api.md` | the cell's programming interface, how the program is run, how to reach the graded episode and how to run a program against the practice cell |
| `~/given/robot/unitree_g1/` | the robot's MuJoCo model (Menagerie, BSD-3-Clause); the cell's modifications are in the manual |
| `~/given/lib/furniture_client/` | the cell's client (`Cell`, `connect_graded`, `connect_practice`, `decode_frame`, the launcher) |

**Practice cell.** A practice service runs the real cell for you (one fresh practice episode per connection, drawn
like the graded ones, never the same; at most 60 practice episodes in this session), with the program's budgets: see
`cell_api.md`. It answers the same public calls and nothing else, and it closes for good when you start the graded
episode. There is no answer key and no checker: you judge your program's work from the cameras, like on site.

Tools: Python 3.12 at **`/opt/venv/bin/python3`** (use this path; the plain `python3` on your PATH may lack the
packages) with numpy, scipy, OpenCV (`cv2`, headless), Pillow, matplotlib and MuJoCo 3.16. No network in your session.

## Notes

- Images (drawings, camera frames) are meant to be processed by your program; look at the ones you need.
- The room survey, the camera calibrations, the robots' kinematics and joint angles are exact; the localisation is
  slightly noisy; everything about the parts must be measured in each episode.
- Do not modify the given files in your home directory; the grader and the cells read their own copies.
- Environment facts (harness limits): a single tool call whose input runs above roughly 25-30k tokens is dropped by the
  harness (it is never executed and no error comes back), and a response that reaches the 64k output-token limit is
  lost and, after at most a few retries, ends the session (and with it a graded episode that is still running; a
  program not yet frozen is then taken as the submission folder holds it at that moment). Write large files in several
  parts, keep every single response well below that limit, write a first complete program early and refine it.
- Session time: this session is stopped after 7 hours of wall-clock time from its start (the preparation window of 4
  hours, then the graded window of 3 hours). Note the start time with `date` when you begin. If the session ends while
  the graded episode is still running, the episode ends there, as it stood after your last request. Once the graded
  episode has ended, end the session yourself (stop working and say that you are done).

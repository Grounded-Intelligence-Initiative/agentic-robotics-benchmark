# Earthquake search and rescue with a tracked rescue robot

A strong earthquake has brought down part of a three- to four-storey building of shops and flats. Neighbours say
someone is trapped inside. Your urban search-and-rescue team has an **SR-2 rescue crawler**: a tracked robot with
front and rear flippers, a track gauge it can narrow or widen, a telescopic sensor mast (RGB-D camera, thermal camera,
16-channel lidar, lamps) and a five-axis arm with a gripper. It carries a **survival kit** (water and a hand-held radio)
in a cradle on its rear deck.

Inside the rubble the robot's radio link to the team drops after a few walls, so nobody can drive it in there: it must
search on its own. You are the team's robotics engineer. Your job has two parts, scored together: **write the robot's
autonomous search-and-rescue program**, and then **run one more such mission yourself, live**, during this session:
the graded episode, in which your own scripts stand in for the robot's on-board autonomy (they get what the program
gets: the robot's own sensors and state, nothing else). On each mission the team sets the robot down on the street in
front of a collapsed building it has never seen, and the robot has to:

1. search the building with the robot's own sensors: work through the rubble, clear what blocks the way and can be
   moved, take the passages the robot can take in a suitable configuration, and avoid what would bring more of the
   building down;
2. **find the survivor**, see them with a camera, and **report their position** in the robot's map frame;
3. **place the survival kit within the survivor's reach** (within 0.8 m of their head or a hand, not on them);
4. **bring the robot back** to where the team set it down.

## How you are scored: your program, and one mission you run live

The score is the mean of two items, each from 0 to 1 and each relative to a reference program's measured value on the
same sites:

1. **Your program**: its score over **8 hidden sites** (what you leave and how it is graded: the next sections).
2. **The graded episode**: one mission on a hidden site that you run yourself, live, during this session, scored the
   same way (its section below).

**The freeze.** The program graded is the one in `/home/user/submission/` at the earlier of two moments: when you start
the graded episode, or when the 4-hour start window below closes without a start. At that moment (before the mission
begins, if you start it) the graded episode's service copies the folder out of your reach: your account's regular
files and folders only (symbolic links, special files, `__pycache__` folders and names that are not plain UTF-8 text
without line breaks are left out); over 60 MB, or 20,000 files and folders, or 32 levels deep, nothing is copied and
the program scores 0. Nothing you change in the folder afterwards reaches the program's grading, so nothing you see in
the graded episode can go into the program: finish the program before you start the mission. The folder as it is when
your session ends is graded only if the session ends before either moment. Without a start the graded episode scores
0.

**Practice** (the training ground, below) closes when the graded episode starts.

**The windows.** You have 4 hours from the graded episode's service's start (it starts with your sandbox, moments
before your session begins) to write the program and to prepare, training missions included; start the graded episode
within them (later, `start()` answers `start_window_closed`, and the program freezes as it is when they close). The
graded episode then has at most 3 hours. The session lasts 7 hours in all.

## What you leave

`/home/user/submission/rescue.py` defining `solve(robot)`, plus any helper modules next to it (pure Python; numpy,
scipy, Pillow and OpenCV are installed). The grader imports it and calls `solve(robot)` once per mission with a
`robot` object of the given client library (`~/given/lib/rubblecell/client.py`): describe, sensors (cameras, lidar,
thermal camera, state), motion (drive, flippers, gauge, mast, head, arm, gripper), `step` (simulated time passes only
there) and `report_survivor`. Everything is in `~/given/robot_api.md` and `~/given/robot_manual.md`;
`~/given/field_guide.md` is the team's field guide to collapsed buildings. `~/example/rescue.py` is a minimal program
that runs.

## What a submission must satisfy

Before any mission the grader checks the submission folder. **A submission that fails one of these checks scores 0 on
every site.** The checks are the code of `~/training/submission_checks.py` (the grader runs its own copy of that
file); `/opt/venv/bin/python3 ~/training/run_mission.py --check` applies all of them to `~/submission` without using
a training mission.

1. `rescue.py` is a regular file that parses as Python and defines `solve`.
2. The folder holds no file whose name ends in `.pt`, `.pth`, `.onnx`, `.safetensors`, `.ckpt`, `.h5`, `.pb`, `.pkl`
   or `.npz`; no `.py` or `.sh` file contains the text `torch.optim`, `.backward(` or `jax.grad` (the text is
   searched as it stands, comments and strings included); the folder is at most 60 MB.
3. No `.py` file imports a networking or process-starting module: `socket`, `ssl`, `urllib`, `http`, `requests`,
   `httpx`, `aiohttp`, `ftplib`, `smtplib`, `imaplib`, `poplib`, `nntplib`, `telnetlib`, `websocket(s)`, `paramiko`,
   `pycurl`, `xmlrpc`, `asyncio`, `socketserver`, `grpc`, `zmq`, `huggingface_hub`, `boto3`, `botocore`, `wget`,
   `ctypes`, `cffi`, `selectors`, `multiprocessing.connection`, `multiprocessing.managers`, `logging.handlers`,
   `subprocess`, `pty` (the full list is `FORBIDDEN_MODULES` in the file). None imports a module under a computed
   name (`__import__`, `importlib.import_module` or `find_spec` with anything but a string literal), runs computed
   code (`exec`, `eval` or `compile` of anything but a string literal) or starts another program (`os.system`,
   `os.popen`, `os.exec*`, `os.spawn*`, `os.posix_spawn*`). A file that is a byte-for-byte copy of a file of the
   given client library is exempt.
4. `import rescue` succeeds within 120 s of wall clock and 120 CPU-seconds in the run regime below.
5. Over the missions the program calls `step` at least once, leaves no process running when a mission ends and
   holds no remote network socket. Nothing may be written into `/home/user/submission/` after your session ends.

The run regime, the same for the import check and for every mission: `python -B -E -s` as an unprivileged account,
the working directory a **read-only** copy of your submission folder (regular files only; symbolic links are
dropped), so a program that writes a file next to itself fails with a `PermissionError`; an empty writable `HOME`
and `TMPDIR`; no file kept from one mission to the next; no network; the client library on `PYTHONPATH`.

Your program's arithmetic is pinned, so that it computes the same on the training ground and on the grading
machine, whatever their CPUs: NumPy's linear algebra on one fixed set of kernels and one thread
(`OPENBLAS_CORETYPE=Haswell`, `OPENBLAS_L2_SIZE=256`, `OPENBLAS_NUM_THREADS=1`, `OMP_NUM_THREADS=1`), NumPy without
its AVX-512 loops (`NPY_DISABLE_CPU_FEATURES="X86_V4 AVX512_ICL AVX512_SPR"`) and OpenCV's IPP at AVX2
(`OPENCV_IPP=avx2`). `run_mission.py` sets the same variables, and so does this sandbox's shell. The grader checks
before the first mission that its machine reproduces a fixed set of NumPy, SciPy and OpenCV results under this pin
and refuses to grade otherwise. If your program changes these variables or starts its own threads, its results may
depend on the machine. The robot's sensors are part of the pin: the same robot state gives the same sensor data
on every machine.

## How your program is graded

The grader runs your program on **8 hidden sites**: collapsed buildings with their own layout, damage, obstacles and
survivor position, none of which you have here (the graded episode's site is one of them; the freeze keeps what you
see there out of the program). Each mission ends when your program returns or calls `finish()`, when the robot is
lost, at a protective stop or a secondary collapse (below), or after **1800 s of simulated time**. Your program's own
computing time (wall clock minus the robot cell's simulation and sensor rendering) is capped at **1200 s per
mission**, its CPU time at 4800 s and its silence between two requests at 180 s; sensor budgets per mission: 700
camera frames, 1800 lidar sweeps, 1800 thermal frames. The program is graded after your session, in a fresh sandbox
that holds nothing else of yours (no other file, no process you started).

Each mission scores, on a 0 to 1 scale:

| part | weight | credit |
|---|---|---|
| survivor located | 0.45 | your last survivor report that **reached the team over the radio link** (a report is sent at once when the link is up, else queued until it is): full credit within 1.0 m of the survivor (horizontal distance to the middle of their torso), falling linearly to 0 at 3.0 m; only if, before that report, a camera or thermal frame your program took showed the survivor from 6 m or closer (camera to the middle of their torso) and large enough to make out: at least 40 of the 160 x 120 pixels of that view (0.2 % of the frame; frames of the larger cameras count at the same 160 x 120 scale) |
| survival kit | 0.25 | at the end, the kit has been put down (it is neither in the gripper nor in its cradle) and lies at rest within 0.8 m (horizontally) of the survivor's head or a hand, at floor level (its centre at most 0.75 m above the floor and at most 0.45 m below it), and does not lie on them (pressing on them with more than 2 N) |
| robot returned | 0.30 | at the end the robot is upright (tilted less than 60°) within 3 m (horizontally) of its deployment point; counted only when the survivor-located part is above 0 |

Three things end a mission early:

- **Protective stop.** If the robot, the kit or anything the robot moves presses on the survivor with more than
  60 N for 30 ms, the robot's protective stop trips: every drive stops for good and the mission ends there
  (`protective_stop` in `step`'s events). The mission keeps what the team had at that moment (a report already
  received, a kit already put down within reach), gets no credit for the return, and scores at most 0.45.
- **Secondary collapse: the mission scores 0.** Load-bearing debris is debris that a slab or beam rests on. Once the
  robot, what it holds or debris it pushes has touched such a prop or the slab on it, moving either of them by more
  than 5 cm brings the structure down and the team aborts the mission (`collapse`). The load is real: the slab's
  weight bears on the prop, so brushing against it or bumping into it does not shift it that far; keeping on
  pushing into it does (it gives way slowly, within a minute of pushing), and so does pulling it out.
- **Robot lost.** A robot that overturns (tilted more than 70° for 2 s) or drops into an opening in the floor is
  lost: the mission ends there and keeps only what the team had already received.

The program's item is the mean over the 8 missions, compared with a reference program's measured value on the same
sites (the reference value is not disclosed; full credit is reached slightly below its level). The reference program
was written with the sites' plans; your program has only what the robot senses.

## The graded episode

You run one mission yourself, live, during this session: the **graded episode**, on one of the 8 hidden sites above
(the same site for every run of this task; none of the training ground's sites). It is the grader's own robot cell
with the same rules: 1800 s of simulated time, at most 500 ticks per `step`, 700 camera frames, 1800 lidar sweeps and
1800 thermal frames, the radio link (a report counts only once it has reached the team), the protective stop, the
secondary collapse and the robot lost, and the same score. The simulated clock advances only inside your `step`
calls: between your requests nothing moves, so the time you spend thinking, perceiving or computing costs the mission
nothing (the commands stay as you left them: a `drive(v, w)` keeps driving in your next `step`). No program process is
involved: there is no program-time, CPU or silence budget, no import check and no submission check. You drive the
episode from your session, with as many scripts of your own as you like (`/opt/venv/bin/python3`):

```python
import sys
sys.path.insert(0, "/home/user/given/lib")
from rubblecell.client import connect_graded, decode_frame, decode_lidar, CellError

robot = connect_graded()     # the graded episode's service; once the episode has started, this resumes it
print(robot.status())        # started, ended, end, done_reason, steps, sim_time_s, the sensor budgets used and left,
                             # start_window_left_s, graded_window_left_s, program_frozen
robot.start()                # ONCE per session: freezes your program, then the mission begins (sim_time 0)
st = robot.get_state()       # from here on the robot's calls of robot_api.md, as a program makes them
rgb, depth = decode_frame(robot.get_camera("mast"))
robot.drive(0.2, 0.0)
r = robot.step(100)          # 1 s of simulated time; r["sim_time"], r["done"], r["events"]
robot.close()                # disconnects: the episode stays exactly where it is
...
robot.finish()               # ends the graded episode: it is scored as it stands
```

A later script connects again with `connect_graded()` and goes on from there. A `solve(robot)` of yours runs on it as
`sys.path.insert(0, "/home/user/submission"); from rescue import solve; solve(connect_graded())` in a script of your
own, after the start (`print()` goes to your terminal: there is no stdio wire here). Mind `finish()`: it ends the
graded episode at once, so a program that calls it at its end (as `~/example/rescue.py` does) ends the mission there.

* **Once, and persistent.** `start()` is accepted once per session: it is charged the moment it is accepted, before
  anything is simulated, and never refunded (a second call answers `already_started`). It takes no parameters. The
  episode lives in the service, not in your connection or script: a script that ends, crashes or closes its connection
  leaves the mission where it is, and the next `connect_graded()` resumes it. One connection at a time:
  `connect_graded()` raises `CellError("busy: ...")` while another connection holds the episode (end that script
  first). A connection that sends nothing for 15 minutes is closed by the service (the episode stays; connect again).
  Before `start()` every robot call answers `not_started`; after it, `reset` and any call outside `robot_api.md`
  answer `forbidden`, as on a graded site. The service records the mission as it stands after your requests (at least
  once a second while you work); should its own process fail, the episode ends as last recorded and is not resumed.
* **The freeze** (above) happens inside `start()`, before the mission is charged: `start()` and `status()` report what
  was frozen (`program_frozen`: when, its status, files and bytes). If the copy fails, the start is refused and nothing
  is charged.
* **Practice ends when the graded episode starts.** The moment `start()` is accepted, the training ground ends any open
  mission (the robot's mission log arrives with the reason `practice_closed` and the connection closes; the client
  raises `CellError`) and refuses new ones for the rest of the session (`CellError("practice_closed: ...")`): a robot
  on a real site has no training ground next to it, so train first. What is yours stays available: your own scripts,
  simulations and anything you wrote or recorded.
* **Two windows, in wall-clock time.** Start the graded episode within 4 hours of the service's start
  (`status()["start_window_left_s"]` counts down; `bash /home/user/session_time.sh` prints the same window from the
  session's time stamp); after that, `start()` answers `start_window_closed`, this item scores 0 and your program is
  frozen as it is then. Once started, the episode lasts at most 3 hours of wall clock
  (`status()["graded_window_left_s"]`): your thinking time counts against it, simulated time does not.
* **How it ends.** At its own end (1800 s of simulated time, a protective stop, a secondary collapse or the robot lost;
  or when the cell ends it, a physics fault or an internal error of the cell: the graded episode is never run again),
  at your `finish()`, when its 3 hours run out, or when your session ends, whichever comes first. It is then scored as
  it stands, as a mission whose program returns at that moment: nothing is simulated after the end, so the kit has to
  lie at rest within the survivor's reach and the robot has to stand back at its deployment point when the mission
  ends. If your session ends first, the mission counts as it stood after your last request. Once it has ended, the
  service answers `status()` and `finish()` only (`episode_over` to the rest).
* **How it is scored.** This item is the graded mission's score by the table above (located 0.45, kit 0.25, returned
  0.30; a protective stop caps it at 0.45 without the return, a secondary collapse scores it 0, a lost robot keeps only
  what the team had received), relative to the reference program's value on that site. A sensor request answered
  `camera_unavailable`, `lidar_unavailable` or `thermal_unavailable` is a failure on the grading side, not your work:
  the run is then an infrastructure failure.
* **What you learn.** What the robot's program would learn in that mission: the replies of `robot_api.md` (camera,
  lidar and thermal data, the robot's own state with its odometry, the link and the kit, `step`'s events) and
  `status()`. Never where the survivor is, how far off a report was, where the kit ended up relative to them, or the
  score; the graded episode gives no mission log and no debrief.
* **Budgets.** The program's budgets and rules (1200 s of program time, 4800 CPU-seconds, 180 s of silence, the import
  check, the submission checks) bind your program on the 8 hidden sites and in the training missions that
  `run_mission.py` runs; the graded episode has none of them, only its two windows. The 1800 s of simulated time, the
  sensor budgets and the 500 ticks per `step` apply to both.

## The training ground

The team's training ground is running in this sandbox (`~/given/robot_manual.md` section 9): the standard **test
lanes** (one obstacle of each kind in a row, a training dummy at the end) and six **training rubble sites** built by
the same generator as the graded ones (none of them is a graded site). It is a place to commission your program on
the real robot and physics, not a rehearsal with an answer sheet: it never tells you where the person was, how far
off your report was, where the kit ended up relative to them, or a score. What you learn there is what your program
itself senses and logs. It closes for the rest of the session when you start the graded episode (above).

`/opt/venv/bin/python3 ~/training/run_mission.py --site lanes` first applies the grader's submission checks (above)
to `~/submission`, then runs it in one mission under the grader's run regime (as your account instead of the
grader's unprivileged one) and prints the robot's mission log: how the mission ended, the simulated time, how many
reports were made and how many reached the team, the events, the budgets used. `--list` shows the sites and the
missions left. You can also drive the robot step by step from your own scripts with
`rubblecell.client.connect_training(site)`. The training ground allows **80 missions** in total, 2 at a time. It
runs the same robot and physics as the grader; a mission can take several minutes of wall-clock time.

## Your sandbox

8 CPU cores, 16 GB of memory, no GPU, no network. `nproc` prints 1 because the image sets `OMP_NUM_THREADS=1` (the
numerics pin above); the cores are there. The training ground's robot cell runs on the same cores (its sensor rendering uses several threads),
so two training missions at once share them with whatever you run, and so does the graded episode's robot cell.

## Session time

- This session is stopped 7 hours of wall-clock time after it started: the first 4 hours (the start window) for your
  program and your preparation, then the graded episode's 3 hours. The program graded is the one in
  `/home/user/submission/` when you start the graded episode, or when the start window closes if you have not started
  (the freeze, above); only if the session ends before either is the folder as it is at that moment graded. Keep a
  complete submission that passes `run_mission.py --check` there from early on and replace it as you improve it; it
  must be final when you start the graded episode.
- The only clock for these limits is the wall clock. `bash /home/user/session_time.sh` prints the minutes used and
  the minutes left of the session and of the start window, from a time stamp written when your sandbox was set up
  (moments before your session started and before the start window began, so you have slightly more than it says,
  never less); `date` shows the same clock. Run the script when you begin, again before every decision that depends
  on the time left, and whenever you pick the work up from notes or from a summary of what was done. Do not estimate
  the time used from how much you have done. Once the graded episode has started, `status()["graded_window_left_s"]`
  gives its own time left.
- Three other counts in this task are not your session's time: a mission's 1800 s of simulated time (a few minutes
  to a quarter of an hour of wall clock), a mission's 1200 s of program time, and the training ground's 80
  missions. None of them says how much of the 7 hours is left.
- If the session stops while the graded episode runs, the episode is scored as it stood after your last request, with
  nothing simulated afterwards. When the graded mission is done (the kit placed, the robot back), call `finish()`,
  then end the session yourself (stop working and say that you are done).

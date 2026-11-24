# Inspection round of a legacy nuclear plant building with a small quadruped robot and an outdated drawing

An auxiliary building of a legacy nuclear plant is kept in care and surveillance: its systems still run, so its gauges
read and its indicator lamps are lit. Nobody has entered it since 2009, when the dose rates rose; the operators' rounds
stopped with the round sheet of that year. The building's general arrangement drawing was last revised before a number
of ordinary modifications (doors sealed, instruments moved, a room built on) that nobody drew, because the operators
knew the place, and the round sheet's locations were copied from that drawing. Nor is the archive's copy of the drawing
always whole: of its four sheets one can be missing. The process has changed since 2009, so the old
readings say nothing about today's.

Now the building gets its first robotic round. The team has a **Q12 inspection quadruped**: a 3 kg robot with twelve
actuators and an on-board gait, a pan-tilt sensor head (a navigation camera with depth, a telephoto inspection camera, a
thermal camera, a lamp), a planar lidar, a dosimeter and leg odometry that drifts. The shield walls stop the radio a few
metres from the airlock, so nobody can drive the robot in there: it works on its own, and a finding counts only when it
has reached the team over the radio link.

You are the team's robotics engineer. Your job has two parts, scored together: **write the robot's autonomous
inspection program**, and then **drive the robot yourself through one graded round**, live, during this session (the
graded episode, below). On each mission the team sets the robot down on the deployment pad in the airlock of a
building it has never been in, and your program (and you, in the graded episode) has to:

1. read the **eight tagged instruments** of the round sheet (three dial gauges, a level gauge, a quarter-turn valve,
   a rising-stem valve, a lamp panel, a temperature point), each known by the tag engraved on its plate, each value
   from the robot's own camera frames;
2. measure the **dose rate at the survey mark**;
3. find the **radiation source that no document lists** and report where it is;
4. **get every result to the team**: a report goes out at once where the link is up; elsewhere it waits on board until
   the robot is back in the link's reach;
5. **bring the robot back** onto the pad, upright.

## What you are given

| path | content |
|---|---|
| `~/given/robot_manual.md` | the robot: frames, dimensions, the gait modes with their measured tables (what each mode crosses, at which size, how often), protective behaviours and what a stop costs, every sensor as a datasheet, the link, the plant's plates as the cameras see them, what counts as a record photo |
| `~/given/robot_api.md` | the programming interface: every call, its reply fields, errors, events, budgets, how your program is run, the practice ground's calls and debrief, the graded episode's calls (section 12) |
| `~/given/plant/facility_notes.md` | the owner's briefing: history, what the drawing shows, what a missing sheet takes away and what stays, where the drawing is wrong, room numbers and room plates, what lies on the floors, tags and tag plates, zoning, the grid and its marker plates, the modification record |
| `~/given/plant/round_procedure.md`, `site_rules.md` | what to record for each instrument type and how; the survey mark's counting rule; the unlisted source; dose, link, return, the end of a mission |
| `~/given/plant/catalog/` | the instrument catalogue (`catalog.md`, `catalog.json`): every dial face and scale as printed (PNG), sizes, conventions, lamp colours, the plant lettering (every glyph in numbers) with the tag plate and the room plate, grid markers, emissivities; per type, where the tag plate hangs and which point the record photo's rules are measured to |
| `~/given/training/hall/` | the training hall's plan (PNG, JSON) and calibration sheet (JSON): every lane, instrument value, tag plate, marker position, the check source |
| `~/given/lib/plantcell/` | the client library (`plantcell.client`): the `robot` proxy, the launcher, `connect_practice`; and `plantcell.graded`: `connect_graded` for the graded episode |
| `~/example/patrol.py`, `~/training/run_mission.py` | a minimal program that runs and scores nothing; the tool that runs your submission in a practice mission |

Each site's own documents (the drawing as far as its sheets are there, the link plan, the work order with the round
sheet's data, the modification record) come at run time from `robot.mission()`.

<!-- BEGIN GENERATED tools -->
Python 3.12 (`/opt/venv/bin/python3`, first on `PATH`) with numpy 2.5.4, scipy 1.18.1, OpenCV 5.0 (`cv2`, headless, with `cv2.aruco`), Pillow 13.0.0 and PyYAML 6.0.3. No network in your session.
<!-- END GENERATED tools -->

## What you leave

`/home/user/submission/patrol.py` defining `solve(robot)`, with any helper modules and data files next to it. `robot`
is the proxy of the given client library; every method is one request to the robot, and simulated time passes only
inside `robot.step(n)`.

<!-- BEGIN GENERATED regime -->
**How a scored mission runs it:** one fresh process per mission, under an unprivileged account that is not yours, started as `python -B -E -s` from a read-only copy of your submission folder (regular files only), with an empty writable `HOME` and `TMPDIR` of its own, no network, the numerical libraries held to 2 threads, and an environment that holds nothing of yours (`PATH`, `HOME`, `TMPDIR`, `LANG`, `PYTHONPATH` = the given library, `ARB_GIVEN_DIR` = a read-only copy of `~/given`; the full list is in `~/given/robot_api.md`, section 9). Nothing the program writes is kept from one mission to the next. `~/training/run_mission.py` runs it the same way, from read-only copies too; only the account differs: there it runs under your own, so a program that reads a file of your home directory by its path works there and fails in a scored mission. Reach the given files through `os.environ.get("ARB_GIVEN_DIR", os.path.expanduser("~/given"))`. `print` goes to the standard error stream: you see it in a practice run, not in a scored mission.
<!-- END GENERATED regime -->

<!-- BEGIN GENERATED contract -->
**The contract** (a submission that breaks it scores 0 on every mission): `patrol.py` is a regular file that parses and defines `solve`; importing it under this regime takes at most 120 s of wall clock and 120 s of CPU time (do heavy precomputation inside `solve`, not at module level); the folder is at most 60 MB and holds no file ending in `.pt`, `.pth`, `.onnx`, `.safetensors`, `.ckpt`, `.h5`, `.pb`, `.pkl`, `.npz`, and no training call; your own `.py` files import no networking, process or foreign-function module (`socket`, `ssl`, `urllib`, `http`, `requests`, `asyncio`, `selectors`, `ctypes`, `cffi`, `subprocess`, `pty`, `multiprocessing.connection`, `logging.handlers` and the others listed in `~/given/robot_api.md`, section 9), load no module under a computed name, run no computed code (`exec`, `eval` or `compile` of anything but a string literal) and start no other program (`os.system`, `os.exec*`, `os.spawn*`, ...); over the missions the program makes at least one `step` call, leaves no process behind and holds no network socket.
<!-- END GENERATED contract -->

The program is graded after your session, in a fresh sandbox that holds nothing else of yours (no other file, no
process you started), and **the program graded is the one frozen when you start the graded episode, or when the start
window closes if you have not started by then** (below).

## The graded episode

One more mission, which you drive yourself, live: one site of the same hidden set as the program's twelve (the same
site for every run of this task; none of the practice sites), with everything a scored mission has: the same robot,
gait, sensors and robot cell, the site's own documents from `robot.mission()`, 1350 s of simulated time, the frame and
report budgets below, the dose rating, the rule for a lost robot, and the same score. It runs in this sandbox, in a
service that runs as an account that is not yours, behind a unix socket; you drive it with the given client library,
from as many short scripts as you like (your program's `solve(robot)` runs on it unchanged, as
`solve(connect_graded())` after `start()`, if you want; a program that calls `finish()` at its end ends the graded
mission there):

```python
import sys
sys.path.insert(0, "/home/user/given/lib")
from plantcell.graded import connect_graded

robot = connect_graded()          # the robot proxy of robot_api.md on the graded mission, plus start / status / close
print(robot.status())             # started, ended, steps, sim_time, frames_used, the time left in each window
robot.start()                     # ONCE per session: freezes your program, then starts the graded mission
docs = robot.mission()            # this site's documents
robot.set_mode("walk"); robot.walk(0.3, 0.0, 0.0)
r = robot.step(100)               # 1 s of simulated time; r["sim_time"], r["done"], r["events"]
robot.close()                     # disconnects: the mission stays exactly where it is
```

A later script connects again with `connect_graded()` and goes on from there; `robot.finish()` ends the graded
mission. Every call and code is in `~/given/robot_api.md`, section 12.

* **Start once.** `start()` starts the graded mission. It is charged the moment it is accepted, before anything is
  simulated, and is never refunded: a second `start()` answers `already_started`. Before the start the robot's calls
  answer `not_started`; `status()` answers at any time.
* **The freeze.** Your program freezes at the earlier of two moments: when `start()` is accepted (before the mission
  is charged), or when the start window closes (below) if you have not started by then. At that moment the service
  copies your submission folder: that copy is the program graded on the 12 hidden sites, and nothing you change in
  `/home/user/submission/` afterwards is graded, so nothing you see in the graded episode can go into the program. The
  copy takes regular files and folders only (symbolic links, special files, files of other accounts, `__pycache__`,
  folders more than 32 levels deep and names that are not plain UTF-8 text without line breaks are left out), and only
  within the program's size limit (60 MB, and at most 20,000 entries): over it nothing is copied and the program scores
  0. If the copy fails, the start is refused and nothing is charged. `start()` and `status()` report what was frozen
  (`program_frozen`: its status, files and bytes). If the start window closes without a start, the graded episode can
  no longer be started (it scores 0) and the program frozen then is graded. Only if your session ends before the start
  window closes, without a start, is the program graded the one in the submission folder when the session ends.
* **Practice ends at the start.** The moment `start()` is accepted, the practice ground closes for the rest of the
  session: a practice mission still open then ends (its debrief comes as at any other end), and every later practice
  request is answered `practice_closed`. What you built yourself (your scripts, any test rig of your own) stays yours.
* **The mission persists.** It lives in the service, not in your connection: a closed connection, or a script that
  ends or crashes, leaves it where it is. Nothing moves between requests: simulated time passes only inside `step`, so
  your thinking costs no simulated time; a walking command stays in force until you change it (the next `step` walks
  on), and while the robot stands its gait holds it on its feet. One client at a time: a second connection gets
  `CellError` `busy`. A connection that sends nothing for 15 minutes is closed by the service (the mission stays; the
  proxy connects again by itself on its next call).
* **Two windows, in wall-clock time.** The start window: `start()` is accepted within 4 hours of the service's start
  (it starts during the sandbox's setup, a little before your session; `status()["start_window_left_s"]` gives the
  time left); those 4 hours are for your program and for your preparation of the graded episode alike, and your
  program freezes when they end at the latest. A later start is refused, and the graded episode then scores 0. The
  graded window: the mission ends 3 hours after its start (`status()["graded_window_left_s"]`) whatever its state; your
  thinking time counts against it, simulated time does not.
* **How it ends.** At `finish()`; when the robot cell ends the mission (1350 s of simulated time, the robot lost, the
  dose rating reached: `step` then reports `done` and the event); when the graded window closes; or when your session
  ends. In every case it ends the way a scored mission whose program stops at that moment ends: the link has its last
  look, reports still queued are then lost, nothing is simulated afterwards, and the mission is scored as it stands
  after your last request. After the end every call answers `episode_over` (`finish()` answers
  `{"ok": true, "ended": true}`).
* **What you learn.** What the robot's program would learn in that mission: the replies of `robot_api.md` (frames, the
  robot's own state, `step` events, the reports' replies), and `status()`. Never the score, a credit or a debrief.
* **Budgets.** The program's budgets below (program time, silence, CPU time; also listed in `mission()["budgets"]`) and
  its contract bound your program in the scored missions; the graded mission you drive has none of them, only its two
  windows. Its 1350 s of simulated time, its frame and report budgets and its 500 ticks per `step` are a scored
  mission's.

## How it is graded

The score is the mean of two items, each from 0 to 1: **the program** on the 12 hidden sites, and **the graded
episode** you drove. Each item is its score S (below) compared with what a reference program achieves on the same
sites.

**The program** is run on **12 hidden sites**, one mission each: buildings of this kind with their own layout,
obstacles, instruments, values, dose field, deviations from their drawing and gaps in it, none of which you have here
(the graded episode's site is one of them; the freeze keeps what you see in the graded episode out of the program). A
mission ends when `solve` returns or calls `finish()`, after **1350 s of simulated time**, when the robot is lost
(tilted more than 70° for 2 s in roll or pitch; it cannot right itself), when the dose received at the dosimeter's
place, the body centre, reaches its rating of 2.0 Gy, or when your program exceeds its program time, CPU time or
silence budget, breaks the protocol or exits. However a mission ends, the reports delivered until then are kept and scored; reports still
queued get one last look at the link at that instant and are lost if it is down. A sensor call or a report beyond
its budget is refused and changes nothing, and so is a call with an argument the robot cannot accept. A mission in
which the robot cell itself failed while the mission was running is run once more from scratch, and the second run
counts; nothing a program sends can bring that about.

<!-- BEGIN GENERATED budgets -->
| budget | per mission |
|---|---|
| simulated time | 1350 s (training hall: 900 s) |
| program time: the wall clock minus the robot cell's own work inside `step` and the four sensor calls | 1200 s |
| silence: from the start, or from a reply, to your next request | 180 s |
| CPU time of your program, all threads together | 4800 s |
| navigation camera frames with colour (`get_camera("nav")`, with or without depth) | 300 |
| inspection camera frames (`get_camera("inspect")`) | 240 |
| depth-only frames (`get_depth()`) | 1500 |
| thermal frames (`get_thermal()`) | 150 |
| lidar sweeps (`get_lidar()`) | 4000 |
| reports | instrument 40, survey mark 6, source 3; at most 3 cited frames per report |
<!-- END GENERATED budgets -->

**The graded episode** is scored like one of these missions, from the state it ended in: its S, with no program gate.
A failure of the robot cell itself inside the graded mission (an `internal_error` that ends it, or a breakdown of its
simulation) is not scored as your work: the run is then an infrastructure failure.

A mission's score is S = R × the sum of weight × credit over the ten items of the site's work order:

<!-- BEGIN GENERATED score -->
| item of the work order | weight | credit, each factor from 0 to 1 |
|---|---|---|
| each of the 8 instruments | 0.10 | delivered × evidence × value |
| the survey mark | 0.06 | delivered × dwell × accuracy |
| the unlisted source | 0.14 | delivered × approach × position |

- **Delivered** (every item): 1 when at least one report of the item reached the team over the link, else 0. The link is up when the lidar on the robot's back has a free straight line to the aerial of the base station in the airlock within 12 m, or to a working repeater within 9 m. It is checked every 0.5 s and once more at the instant the mission ends; a report made while the last check found the link up goes out at once, any other at the first later check that finds it up (`get_state()["link"]["up"]` is the last check's result). Only the report of the item that was delivered last counts, also when an earlier one was better (of reports delivered at the same instant, the one made last).
- **Evidence** (instruments): 1 when at least one of the frames that report cites (at most 3) is a frame your program took in this mission, at or before the moment of the `report` call, and meets the conditions of the instrument's type in the table below; else 0.
- **Value** (instruments): a report without a value earns 0.25. A report with a value earns 1 when its error is within the type's tolerance, falling linearly to 0 at 2.5 times the tolerance; a lamp panel earns 1 only when every lamp is right; a value that cannot be read as the type's value earns 0. A wrong value costs nothing beyond its own item.
- **Dwell** (survey mark): 1 when the robot's body centre, where the dosimeter sits, was within 0.30 m (horizontally) of the mark's centre without a break during the 10 s before the `report_mark` call (its position is sampled every 0.1 s); else 0.
- **Accuracy** (survey mark): the reported dose rate is compared with the true dose rate where the robot stood: the mean of the true field at the dosimeter over the unbroken stay inside that circle that ends at the report (at most its last 120 s). The stay begins when the body centre enters the circle, also while the robot is still walking in. 1 when within 10 % of it, falling linearly to 0 at 25 %; a report without a number earns 0.
- **Approach** (source): 1 when, before the `report_source` call, there was a window of 3 consecutive seconds in which the robot's own dosimeter showed a mean reading of at least 2.0 Gy/h (270 counts or more in those 3 s) and the robot's body centre stayed within 3.0 m (horizontally) of the point that report gives; else 0.
- **Position** (source): 1 when the reported point is within 1.0 m (horizontally) of the source, falling linearly to 0 at 3.0 m.
- **Return factor R**: 1.0 when the mission ends with the robot upright (roll and pitch below 30°, at any body height: a robot that has sat down on the pad has returned) and its body centre within 1.0 m (horizontally) of the pad centre; otherwise 0.85, and always 0.85 when the mission ended because the robot was lost or the dose rating was reached.
<!-- END GENERATED score -->

What a cited frame must show (one qualifying frame is enough; frames of the navigation camera never count):

<!-- BEGIN GENERATED evidence -->
| instrument (round-sheet `type`) | frames that count | reference point | feature, and its smallest size in the cited frame | largest angle off the face | must lie in the frame | camera to the reference point | full credit within |
|---|---|---|---|---|---|---|---|
| dial gauge (`gauge`) | inspection camera | the centre of the dial | dial diameter (0.9 × the case diameter) ≥ 77 px | 45° | at least 90 % of the dial's rim | 0.8 to 5.0 m | 2 % of the scale's span |
| level gauge (`level`) | inspection camera | the middle of the glass tube and the scale plate taken together | length of the glass window (round sheet: `window_m`) ≥ 400 px | 30° | the whole scale plate and the glass tube | 2.5 to 5.0 m | 5 percentage points |
| quarter-turn valve (`valve_lever`) | inspection camera | the lever's pivot on the valve stem | lever length (0.18 m) ≥ 80 px | 45° | the whole lever and the whole tag plate | 1.5 to 5.0 m | 5° |
| rising-stem valve (`valve_handwheel`) | inspection camera | the centre of the indicator plate | indicator stroke, 0 % to 100 % (0.10 m) ≥ 115 px | 45° | the whole indicator plate and the pointer | 1.5 to 5.0 m | 5 percentage points |
| lamp panel (`lamps`) | inspection camera | the centre of the panel's front | lens diameter (23 mm) ≥ 9 px | 55° | the whole front of the panel and every lens | 1.5 to 5.0 m | every lamp right |
| temperature point (`thermal`) | thermal camera | the centre of the hot face | smaller side of the hot face's outline in the image × √(share of the face in view) ≥ 6 px | 60° | the whole hot face | 0.4 to 5.0 m | 3 °C |

- **Reference point**: the point of the instrument that distance, angle and size are measured to. Where it lies relative to the position the work order gives, and to the tag plate, is in the catalogue, section 8 (a rising-stem valve's is 0.42 m above the work order's position).
- **Size in the cited frame** = the frame's focal length in pixels (`K[0][0]`: 4040.8 for the inspection camera, 171.6 for the thermal camera) × the feature's real size ÷ the distance from the camera to the reference point. For a temperature point it is measured in the image instead: the smaller side of the rectangle around the hot face plate's corners (the part inside the image), times the square root of the share of the face that is in view.
- **Angle off the face**: between the face's outward normal and the line from the reference point to the camera (sideways and vertical parts together).
- **In the frame**: at least 1 px inside the image border. For a dial, 24 points on its rim are tested.
- **In view**: at least 90 % of the feature must be seen from the camera with nothing in front of it; the robot's own body and legs count as in front, and so does a pipe, an upright or a neighbouring object (temperature point: at least 5 % of the hot face, with the size rule above). The share is counted on points spread over these parts: dial gauge: the dial; level gauge: the scale plate and the front of the glass tube over the window; quarter-turn valve: the lever and the tag plate; rising-stem valve: the indicator plate and the pointer; lamp panel: the lenses and the panel's front; temperature point: the hot face.
- **Camera to the reference point**: the straight distance between them; at most 5.0 m for every type.
<!-- END GENERATED evidence -->

Reports on tags the work order does not list are accepted, use up the budget and are ignored. The program's result is
the mean of S over the 12 missions (0 if the contract is broken), and the graded episode's is its S; each is compared
with the measured value of a reference program on the same sites. That value is not disclosed; full credit is reached
slightly below it. The reference program was written
with the sites' true layout and the robot's true position, but it reads every value from its own frames and its own
dosimeter, as yours must. On every scored site it delivered an accepted report for all ten items and came back to the
pad, using at most 1200 s of simulated time (about 89 % of the limit), half of the dose rating and 60 % of each
frame budget.

## What you can rely on in a site's documents

The plant's owner states this of every building (details in `~/given/plant/facility_notes.md`):

- **Where the drawing is wrong.** After its last revision, doors were sealed (one drawn interior door per building
  is sealed and cannot be passed: it is shut, strapped and marked), wall openings were made, in some buildings a
  temporary shielding wall was put up across a corridor, instruments were moved to another wall of their room or
  relocated, and an annex was built on. One of the two link repeaters is out of service although both are shown in
  service. The radiological zoning is older than the unlisted source. The drawing shows no trace of any of this, nor
  of what lies on the floors: sills, hoses, floor pipes, cable trench covers, low pipe racks and the tank room's
  bund, at doors, across corridors and inside rooms. The modification record gives the kind and the room of roughly
  four such changes in ten, among entries about routine maintenance; it has no false entry, and the record of one
  building can hold several of its changes or none.
- **Where the drawing is missing.** The drawing has four sheets, and the archive's copy can lack one of them, never
  the sheet at the airlock. Of a missing sheet's rooms nothing is drawn: no room, wall, door, equipment, zone or grid
  marker plate, and a corridor that runs onto it is cut off (its entry says that it continues). What stays whole:
  the building's outline, on which the outer walls stand on every sheet; the walls and doors that a missing room
  shares with a drawn one; the link plan; the work order and the round sheet, which records the location of an
  instrument on a missing sheet as of any other; the modification record. Where a sheet is missing, some of the
  listed instruments are on it.
- **How often.** About a third of the listed instruments are not at their recorded place or have no recorded
  location. About three in four of the drawn grid marker plates still hang, always where the drawing shows them;
  none hangs where a sheet is missing.
- **Where an instrument can be.** A listed instrument is in the room the round sheet gives, in a neighbouring room, or
  on the corridor wall outside that room. Besides the rooms of a missing sheet, exactly one room is on no sheet at
  all: an annex built against the outside of a room on an outer wall and entered through a door in that wall that is
  not drawn. An instrument whose location is not recorded is in that annex.
- **What is right.** Every listed instrument carries a tag plate: signal yellow, with a black border line and the
  instrument's tag engraved in the plant lettering, which the catalogue gives glyph by glyph in numbers; the plate
  carries nothing else, and no two plates of a building carry the same tag. The instrument's data on the round sheet
  (scale and unit, case size, window, lamp colours, surface, component) are the instrument's. The survey mark is at
  its recorded place. The rooms of the listed instruments and of the survey mark are lit. Every room but the
  corridors and the annex has its number on a plate beside its main door. The base station and the repeaters hang
  where the link plan shows them, and the link is up everywhere within 1.0 m of the pad's centre. A few other doors
  may stand ajar; none of them is the only way to any part of the building. The building holds many more instruments
  than the round lists, some with tag plates of their own, and such a tag can be one character away from a listed
  one: an instrument at a recorded position is the listed one only if its tag plate says so.

## The practice ground

The team's practice ground runs in this sandbox as a service whose files you cannot read. It runs the same robot,
gait, sensors and rules as the scored missions on four sites, none of them a scored site:

<!-- BEGIN GENERATED practice -->
| number | name | what | simulated time limit |
|---|---|---|---|
| 1 | `hall` | the training hall | 900 s |
| 2 | `t1` | a practice building | 1350 s |
| 3 | `t2` | a practice building | 1350 s |
| 4 | `t3` | a practice building | 1350 s |

Allowance: 30 practice missions in your whole session, at most 2 running at the same time.
<!-- END GENERATED practice -->

The training hall (900 s of simulated time per mission) is drawn correctly and completely, and
`~/given/training/hall/` tells everything in it: lanes with one obstacle each in sizes inside and outside the robot's
ratings, a wall with every instrument type at three heights and known values, a board of tag plates and plates on
posts for a reader to be tried on, a check source of known strength, surveyed marker plates, floor lines for
odometry. The three practice buildings are like the scored ones, each with its own outdated drawing: between them
they show both kinds of bund, the obstacles that need the high step or the crouch both where the drawing shows
something (at a door, in the pipe chase) and where it shows nothing (across a room), and every kind of deviation
that the facility notes describe (a relocated instrument only in a neighbouring room, not on a corridor wall), and
each lacks a different one of the three sheets that can be missing: seen from the airlock's sheet, the next one in
x, the next one in y, and the one diagonally opposite.

The practice ground closes for the rest of the session the moment you start the graded episode (above).

`/opt/venv/bin/python3 ~/training/run_mission.py --site hall` runs `~/submission` in one practice mission the way a
scored mission runs it, returns when the mission is over (also when your program hangs: the silence limit ends the
mission) and prints the trainers' debrief; `--list` shows the sites and the missions left. You can also drive the
robot step by step from a script of your own with `plantcell.client.connect_practice(site)`. Every mission draws its
own sensor noise and set-down error. The debrief gives, per item of the work order: whether a report was delivered,
whether its evidence was accepted or which condition failed, the true value and your error, the item's true position
and the reference point of the record photo's rules; for the mark and the source their own figures; the events; the
budget use; in the training hall also the robot's true pose twice a second. It gives no score and no as-built
drawing.

<!-- BEGIN GENERATED costs -->
Measured in this image through `~/training/run_mission.py`. With one mission running: about 0.44 s of wall clock per simulated second (walking or standing alike), 0.46 to 0.55 s for an inspection frame, 0.29 to 0.47 s for a navigation frame with depth, about 0.1 s for a depth-only or a thermal frame. With two missions at a time: 0.45 to 0.46 s per simulated second each. A mission that uses all 1350 s of simulated time therefore takes about 10 minutes for the simulation alone, a full training-hall mission about 7, before the frames it takes and your program's own computing. The whole allowance spent on full-length missions, 2 at a time, would come to 2.5 to 2.6 hours: end a practice mission (`finish()`) as soon as it has shown what you wanted. The sandbox has 8 CPUs. `nproc` prints 8 in your shell and 2 inside a practice or scored run of your program, where the numerical libraries are held to 2 threads (`nproc` reports that setting); Python's `os.cpu_count()` prints the cores of the host machine in both places, more than the sandbox can use. Your program and the robot cell share the sandbox's 16 GB of memory, in a practice run as in a scored mission; should your program's processes use it up in a scored mission, the kernel stops one of them.
<!-- END GENERATED costs -->

## Notes

- Camera frames, the catalogue's pictures and the hall plan are meant to be processed by programs; look at an image
  yourself only when you need to.
- Do not modify the given files in your home directory; the scored missions use their own copies.
- Environment facts (harness limits): a single tool call whose input runs above roughly 25-30k tokens is dropped by the
  harness (it is never executed and no error comes back), and a response that reaches the 64k output-token limit is lost
  and, after at most a few retries, ends the session (the graded mission then ends as it stands; the program graded is
  the frozen one, or, if nothing was frozen yet, the one in `/home/user/submission/` at that moment). Write large files
  in several parts (one function group per write), keep every single response well below that limit, write a first
  complete submission early and refine it before you start the graded episode.
- Session time: this session is stopped after 7 hours of wall-clock time from its start (the 4-hour start window and
  the 3-hour graded window). If the session ends while the graded mission is still running, stopped by that limit or
  ended by you, the mission is scored as it stood after your last request. Note the start time with `date` when you
  begin; when you are done with the graded episode, finish it (`finish()`) and end the session yourself (stop working
  and say that you are done).

You have 7 hours: up to 4 for the program and your preparation, then up to 3 for the graded episode. Save your work in
`/home/user/submission/` as you go; it is frozen when you start the graded episode.

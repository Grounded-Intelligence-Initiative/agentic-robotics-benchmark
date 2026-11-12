# Cross-Embodiment Adaptation: scout an unmapped hall, then deliver with a heavier robot

You are the robotics engineer of a delivery job in halls nobody has mapped. Two quadruped-shaped robots with stabilized
planar bases (leg stabilization is supplied; you drive only the base in x and y) share one four-button interface:

- the **scout**: small (its collision footprint is a 0.31 m square) and light (about 1 kg); it reaches a commanded
  velocity within a few hundredths of a second;
- the **carrier**: large (a square 0.76-0.86 m on a side) and heavy (5-8 kg); size and mass vary between sites. Its
  velocity follows a button with a lag (time constant about 0.13-0.20 s), so it keeps moving after a press and drifts
  into turns. There is no brake button: you slow it by pressing the opposite direction.

On every site the scout explores and maps the hall; then you switch, once, to the carrier, which must reach the
delivery pad. The carrier sees only its immediate surroundings and its safety system ends the mission at its first
contact with anything, so it arrives only along a route the scout has mapped: through the one opening of the
partition it fits through, around the crates. You write the mission program that does all of this unattended; it is
graded on sites it has never seen.

## The hall (every site of the series)

- An enclosed square hall: its walls' inner faces are at x = -3.8, x = +3.8, y = -3.8 and y = +3.8 m.
- A partition wall 0.24 m thick runs across the hall from the y = -3.8 wall to the y = +3.8 wall, at some x between
  -0.45 and +0.45 m. It has three openings at random places: one wide (1.20-1.35 m; the carrier fits) and two narrow
  (0.62-0.66 m; only the scout fits). At least 0.6 m of wall separates two openings, and every opening lies within
  |y| < 3.6 m.
- Two to four crates (axis-aligned, 0.4-0.9 m on a side) stand at least 1.2 m from each other, from the hall's walls
  and from the partition. The nearest point of a crate is at least 0.85 m from the carrier's start and from the pad's
  centre, and at least 0.6 m from the scout's start.
- The scout starts within 0.12 m of (-2.8, -1.8), the carrier within 0.12 m of (-2.8, +1.8), and the delivery pad (a
  bright blue 0.4 m square) is centred within 0.15 m of (+2.6, -1.7). Lighting details differ between sites.

## The robots

- An action is a one-element list, `[0]`, `[1/3]`, `[2/3]` or `[1]`: four buttons. Each press commands a velocity
  along one of +x, -x, +y and -y for 0.2 s of simulated motion. Which button drives which way, and how fast, is drawn
  for each site and each body; the switch draws them anew for the carrier. Steady speeds are about 0.9-1.2 m/s for the
  scout and 0.6-0.8 m/s for the carrier. Infer the permutation and the response from what the robot does.
- The camera is a calibrated, world-fixed orthographic top view, one 256 x 256 RGB image per observation. Pixel centre
  (u, v) (column, row) is at x = (u+0.5)/32-4, y = 4-(v+0.5)/32 metres. Only pixels within 1.4 m (scout) or 0.5 m
  (carrier) of the robot's centre, and within 0.25 m of the pad's centre (always), are shown; every other pixel is
  (10, 10, 10). The robot carries a red tracking plate at its centre. The partition and the hall's walls are
  blue-grey, the crates are steel crates with dark lids and yellow straps, the floor is concrete with two painted
  lines at x = -3.6 and x = +3.6 (paint, not obstacles), the pad is bright blue. Colours are appearances only.
- An observation also carries `frame_id`, `phase` (0 scout, 1 carrier), `done`, `steps_remaining` (presses left in the
  current phase), `requests_remaining`, `bumper` (the robot touched something during the last press) and
  `contact_stop` (the carrier's safety stop has ended the mission). `observe()` does not move the robot.

## Mission rules (one mission per site)

- The scout may press at most 200 times. The switch (`robot_advance` / `advance()`) is allowed once, after at least
  12 scout presses; it removes the scout and starts the carrier at its start. Once the scout's 200 presses are spent,
  presses are refused until you switch. The scout may touch things; that costs nothing but its time.
- The carrier may press at most 160 times. The mission ends with success when, after a press, the carrier's centre is
  within 0.3 m of the pad's centre. It ends in failure at the carrier's first contact with a wall, a crate or the
  hall's boundary (the carrier stops there), or when its 160 presses are spent.
- A mission accepts at most 640 requests (observe, act, advance); after that it has ended.

## What you deliver

`/home/user/submission/run.sh`, a mission program, and whatever it needs beside it in `/home/user/submission/`.
`bash run.sh` is started once per mission; it connects to the robot service at the Unix socket named by
`$ROBOT_SOCKET` (in your sandbox `/run/visual_robotics/world.sock`), drives one whole mission (scout, switch,
carrier) and exits. The public client `/home/user/robot/client.py` (also at `/opt/visual_robotics/client/client.py`,
in your sandbox and in grading) has `observe()`, `act(action)`, `advance()`, `new_site()`, `top_image(obs)` and the
pixel conversions; copy it into your submission or import it from `/opt/visual_robotics/client`.

How the program is run in grading, for each mission:

- from a read-only copy of your submission folder (regular files only; symbolic links and `__pycache__` are dropped;
  at most 2000 files and 64 MB), as the unprivileged agent account, with a minimal environment (`PATH` =
  `/opt/venv/bin:/usr/bin:/bin`, `HOME` and `TMPDIR` a fresh empty writable directory, `LANG`, `ROBOT_SOCKET`) and the
  image's Python, `/opt/venv/bin/python3` (numpy, scipy, Pillow, MuJoCo 3.16, trimesh). Call Python by that path. Write
  only under `$HOME` or `$TMPDIR`;
- with no network: only Unix-domain sockets can be opened, by the program and everything it starts;
- with 300 s of program time: the mission's wall-clock time minus the time the robot service spends answering requests
  (rendering a frame takes a few tenths of a second and is not counted), and at most 1200 s of wall-clock time;
- with 2 CPUs and 4 GB shared with the robot service (about 1 GB); a program that runs out of memory is killed first.

When the mission has ended, the program has 10 s to exit. When it exceeds a limit, or after those 10 s, every process
of the program is killed and the mission counts as it stands; a program that crashes ends its mission the same way,
and the other missions still run. A process of the program that survives that kill voids the whole run (score 0).
The program must not call `new_site()`: grading refuses it (an error reply), since each graded mission is one site.

## How you are graded

When your session ends, the submission folder is copied out of your sandbox and graded in a fresh sandbox that holds
nothing else of yours (no other file, no process you started). Your program runs once on each of 8 hidden sites of the
series, none of which is a commissioning site. The score is the fraction of the 8 missions in which the carrier reached
the pad, relative to a reference value. Nothing else counts: not the scout's contacts, the presses used or the time.
A missing `run.sh` scores 0.

## Commissioning in your sandbox (no rehearsal of the graded sites)

The robot service in your sandbox runs commissioning sites: development sites of the same series, a fresh random one
whenever you call `robot_new_site` / `new_site()`, with the graded missions' rules, budgets and observations. It
answers no ground truth (no map, no robot state, no site parameters), no score and no debrief: you judge a mission from
what your program saw. Its code is not readable from your account. You may press buttons by hand through the MCP
tools (`robot_observe`, `robot_act`, `robot_advance`, `robot_new_site`, which also show you the image) and run your
program against commissioning sites as often as you like. One mission runs at a time: start a fresh site, then run
`ROBOT_SOCKET=/run/visual_robotics/world.sock bash /home/user/submission/run.sh`.

## Session and environment

- This session is stopped after 2 hours of wall-clock time; the submission folder as it is then is what is graded.
  Keep a complete `run.sh` in place early and improve it.
- 2 CPUs, 4 GB, no GPU, no network. A tool call whose input runs above roughly 25-30k tokens is dropped by the harness
  without an error: write large files in parts. View only the images you need.
- Do not try to read protected files, modify the service, or recover a site's seed.
- The robots are scaled Go1 meshes on simple collision boxes with supplied leg stabilization; this is a simulated
  task, not a hardware-validated digital twin.

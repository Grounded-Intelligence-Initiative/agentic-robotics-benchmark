# Cross-Embodiment Adaptation: one robot maps a hall, another must find itself in that map and deliver

You are the robotics engineer of a delivery job in halls nobody has mapped. Two quadruped-shaped robots with stabilized
planar bases (leg stabilization is supplied; you drive only the base) share one four-button interface:

- the **scout**: small (a 0.31 m square footprint) and light (about 1 kg), with a camera on a mast that looks down on
  the floor, the walls and the crates around it;
- the **carrier**: large (a square 0.76-0.86 m on a side) and heavy (5-8 kg); size and mass vary between sites. Its
  only camera looks at the floor under its own base: it never sees a wall or a crate, and its safety system ends the
  mission at its first contact with anything.

Nothing tells either robot where it is in the hall or how it is turned: there are no global coordinates. On every site
the scout explores and maps the hall in its own frame; then you switch, once, to the carrier, which is set down near
where the scout started, at a heading of its own. The carrier arrives only if it finds its own position and heading
in the map the scout made (the floor is the one thing both cameras see), keeps track of them while it drives, and
follows a route the scout has seen: through the one opening of the partition it fits through, around the crates. You
write the mission program that does all of this unattended; it is graded on sites it has never seen.

## The hall (every site of the series)

Dimensions are given in the hall's own frame, which no sensor reports.

- An enclosed square hall, 7.6 m by 7.6 m between the inner faces of its walls. The walls are about 0.7 m tall.
- A partition wall, 0.24 m thick and as tall, runs from one wall to the opposite one, parallel to the other two and
  at most 0.45 m off the hall's middle. It has three openings at random places: one wide (1.50-1.65 m) and two narrow
  (0.62-0.66 m; only the scout fits). At least 0.6 m of wall separates two openings, and every opening ends at least
  0.2 m from the hall's walls.
- Two to four crates, 0.6 m tall and 0.4-0.9 m on a side, their sides parallel to the hall's walls, stand at least
  1.2 m from each other, from the hall's walls and from the partition. The hall is in use: one of them stands within
  0.45 m of the straight way from where the carrier is set down to the middle of the wide opening and on to the pad.
- The **dock** is a point of the floor (not marked) in one of the two rooms, at least 1.5 m from every wall, the
  partition and every crate. The scout is set down with its centre on it. The carrier is set down with its centre
  within 0.8 m of it. Each is set down at its own, arbitrary heading.
- The **delivery pad**, a bright blue square 0.4 m on a side and 4 cm thick, lies in the other room, its centre at
  least 1.0 m from the hall's walls and the partition and 1.15 m from the nearest point of a crate.
- At any heading the carrier lies inside a circle of radius 0.60 m about its centre (the scout: 0.21 m). From every
  place the carrier may be set down there is a route to the pad along which its centre stays at least 0.70 m from
  every wall, the partition and every crate.
- The floor is cast concrete, laid differently on every site: its pattern repeats every 2.4-3.75 m in two
  perpendicular directions of its own, which are not the hall's. Two yellow lines are painted on it 0.2 m inside the
  two walls parallel to the partition (paint, not obstacles). Daylight falls from a direction that differs between
  sites; walls and crates cast shadows. The floor continues outside the hall's walls, where neither robot can go.

## The robots

- An action is a one-element list, `[0]`, `[1/3]`, `[2/3]` or `[1]`: four buttons. Each press commands, for 0.2 s of
  simulated motion, a velocity along one of the body's own four directions: its camera image's right, left, up and
  down. Which button drives which way, and how fast, is drawn for each site and each body; the switch draws them anew
  for the carrier. Steady speeds are about 0.9-1.2 m/s for the scout, which reaches them within a few hundredths of a
  second, and 0.6-0.8 m/s for the carrier, whose velocity follows a button with a lag (time constant about
  0.13-0.20 s): it keeps moving after a press and drifts into turns. There is no brake button and no button that
  turns the body. Infer the buttons and the response from what the robot does.
- **Slip.** On some presses a foot slips: between 12% and 25% of the presses of a body on a site (the rate is drawn
  for each site and body). A slipping press gives the body only 20-70% of the commanded velocity, and during it the
  body turns by 1.5-6 degrees to one side or the other. Every other press loses up to 3%. The heading changes in no
  other way: the stabilization holds it, also against a contact.
- **Odometer.** `odometer` is `[x, y]` in metres: the running total of the leg odometer since this body started, each
  press added in the body's frame of that moment (x to the image's right, y up the image). It counts strides: it
  reads what the body moved plus what a slipping press lost, so it over-reads slips; it is otherwise accurate to 2%
  of a press. It does not sense a turn.
- **The scout's camera** is a pinhole camera 2.0 m above the floor, looking straight down, centred on the scout and
  turning with it: one 256 x 256 RGB image per observation. The floor plane fills it exactly, 2.8 m across: pixel
  centre (u, v) (column, row) shows the floor point x = (u - 127.5) x 2.8 / 256 to the image's right of the scout's
  centre and y = (127.5 - v) x 2.8 / 256 up the image, in metres. A point at height h above the floor appears at
  2 / (2 - h) times its true distance from the image centre: walls and crates lean away from the centre, show their
  near sides and hide the floor behind them. The scout itself, with a red tracking plate, is at the centre. The
  partition and the hall's walls are blue-grey, nearly black where the light does not reach; the crates are rusted
  steel, the rust broken by pale patches near the colour of the floor, with a dark lid (3 cm smaller than the crate
  all round, its top 0.62 m above the floor) and two yellow straps; the pad is bright blue. Colours are appearances
  only.
- **The carrier's camera** is a floor camera under its base: one 256 x 256 RGB image per observation, an orthographic
  view of the square of floor beneath the carrier's centre, image right and up being the body's own. Its span is
  between 0.56 and 0.64 m across, fixed for a mission and not reported (the camera rides lower under a heavier
  load). It is another sensor than the scout's: its exposure (0.85-1.15 times), its colour balance (each channel
  within 7%) and a fall-off of brightness toward the corners (25-40%) are drawn for the site, and it has sensor noise
  (about 5 grey levels). It shows the floor, its paint and the pad, and nothing that stands on the floor.
- An observation also carries `frame_id`, `phase` (0 scout, 1 carrier), `done`, `steps_remaining` (presses left in the
  current phase), `requests_remaining`, `bumper` (the robot touched something during the last press) and
  `contact_stop` (the carrier's safety stop has ended the mission). `observe()` does not move the robot.

## Mission rules (one mission per site)

- The scout may press at most 500 times. The switch (`robot_advance` / `advance()`) is allowed once, after at least
  12 scout presses; it removes the scout and sets the carrier down. Once the scout's 500 presses are spent, presses
  are refused until you switch. The scout may touch things; that costs nothing but its time.
- The carrier may press at most 240 times. The mission ends with success when, after a press, the carrier's centre is
  within 0.3 m of the pad's centre. It ends in failure at the carrier's first contact with a wall, the partition or a
  crate (the carrier stops there), or when its 240 presses are spent.
- A mission accepts at most 1000 requests (observe, act, advance); after that it has ended.

## What you deliver

`/home/user/submission/run.sh`, a mission program, and whatever it needs beside it in `/home/user/submission/`.
`bash run.sh` is started once per mission; it connects to the robot service at the Unix socket named by
`$ROBOT_SOCKET` (in your sandbox `/run/visual_robotics/world.sock`), drives one whole mission (scout, switch,
carrier) and exits. The public client `/home/user/robot/client.py` (also at `/opt/visual_robotics/client/client.py`,
in your sandbox and in grading) has `observe()`, `act(action)`, `advance()`, `new_site()`, `camera_image(obs)` and the
pixel conversions; copy it into your submission or import it from `/opt/visual_robotics/client`.

How the program is run in grading, for each mission:

- from a read-only copy of your submission folder (regular files only; symbolic links and `__pycache__` are dropped;
  at most 2000 files and 64 MB), as the unprivileged agent account, with a minimal environment (`PATH` =
  `/opt/venv/bin:/usr/bin:/bin`, `HOME` and `TMPDIR` a fresh empty writable directory, `LANG`, `ROBOT_SOCKET`) and the
  image's Python, `/opt/venv/bin/python3` (numpy, scipy, Pillow, MuJoCo 3.16, trimesh). Call Python by that path. Write
  only under `$HOME` or `$TMPDIR`;
- with no network: only Unix-domain sockets can be opened, by the program and everything it starts;
- with 600 s of program time: the mission's wall-clock time minus the time the robot service spends answering requests
  (rendering a frame takes one to two tenths of a second and is not counted), and at most 1800 s of wall-clock time;
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

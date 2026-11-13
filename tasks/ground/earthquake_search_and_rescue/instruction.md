# Earthquake search and rescue with a tracked rescue robot

A strong earthquake has brought down part of a three- to four-storey building of shops and flats. Neighbours say
someone is trapped inside. Your urban search-and-rescue team has an **SR-2 rescue crawler**: a tracked robot with
front and rear flippers, a track gauge it can narrow or widen, a telescopic sensor mast (RGB-D camera, thermal camera,
16-channel lidar, lamps) and a five-axis arm with a gripper. It carries a **survival kit** (water and a hand-held radio)
in a cradle on its rear deck.

Inside the rubble the robot's radio link to the team drops after a few walls, so nobody can drive it in there: it must
search on its own. You are the team's robotics engineer. **Write the robot's autonomous search-and-rescue program.**
On each mission the team sets the robot down on the street in front of a collapsed building it has never seen, and
your program has to:

1. search the building with the robot's own sensors: work through the rubble, clear what blocks the way and can be
   moved, take the passages the robot can take in a suitable configuration, and avoid what would bring more of the
   building down;
2. **find the survivor**, see them with a camera, and **report their position** in the robot's map frame;
3. **place the survival kit within the survivor's reach** (within 0.8 m of their head or a hand, not on them);
4. **bring the robot back** to where the team set it down.

## What you leave

`/home/user/submission/rescue.py` defining `solve(robot)`, plus any helper modules next to it (pure Python; numpy,
scipy, Pillow and OpenCV are installed). The grader imports it and calls `solve(robot)` once per mission with a
`robot` object of the given client library (`~/given/lib/rubblecell/client.py`): describe, sensors (cameras, lidar,
thermal camera, state), motion (drive, flippers, gauge, mast, head, arm, gripper), `step` (simulated time passes only
there) and `report_survivor`. Everything is in `~/given/robot_api.md` and `~/given/robot_manual.md`;
`~/given/field_guide.md` is the team's field guide to collapsed buildings. `~/example/rescue.py` is a minimal program
that runs. Your program runs without network access, as an unprivileged user, with no file left behind between
missions; it must not start other processes.

## How it is graded

The grader runs your program on **4 hidden sites**: collapsed buildings with their own layout, damage, obstacles and
survivor position, none of which you have here. Each mission ends when your program returns or calls `finish()`, when
the robot is lost, or after **1800 s of simulated time**. Your program's own computing time (wall clock minus the
robot cell's simulation and sensor rendering) is capped at **1200 s per mission** and its silence between two
requests at 180 s; sensor budgets per mission: 700 camera frames, 1800 lidar sweeps, 1800 thermal frames.

Each mission scores, on a 0 to 1 scale:

| part | weight | credit |
|---|---|---|
| survivor located | 0.45 | your last survivor report that **reached the team over the radio link** (a report is sent at once when the link is up, else queued until it is): full credit within 1.0 m of the survivor (horizontal distance to the middle of their torso), falling linearly to 0 at 3.0 m; only if, before that report, a camera or thermal frame your program took showed the survivor from 6 m or closer (camera to the middle of their torso) and large enough to make out: at least 40 of the 160 x 120 pixels of that view (0.2 % of the frame; frames of the larger cameras count at the same 160 x 120 scale) |
| survival kit | 0.25 | at the end, the kit has been put down (it is neither in the gripper nor in its cradle) and lies at rest within 0.8 m (horizontally) of the survivor's head or a hand, at floor level (its centre at most 0.75 m above the floor and at most 0.45 m below it), and does not lie on them (pressing on them with more than 2 N) |
| robot returned | 0.30 | at the end the robot is upright (tilted less than 60°) within 3 m (horizontally) of its deployment point; counted only when the survivor-located part is above 0 |

A mission scores **0** if the robot, the kit or anything the robot moved **presses on the survivor** with more than
60 N, or if the robot moves **load-bearing debris** (debris that a slab or beam rests on) by more than 5 cm, which
brings the structure down (a secondary collapse; the mission is aborted). A robot that overturns (tilted more than
70° for 2 s) or drops into an opening in the floor is lost: the mission ends there and keeps only what the team had
already received. Your score is the mean over the 4 missions, compared with a reference program's measured value on
the same sites (the reference value is not disclosed; full credit is reached slightly below its level). The
reference program was written with the sites' plans; your program has only what the robot senses.

## The training ground

The team's training ground is running in this sandbox (`~/given/robot_manual.md` section 9): the standard **test
lanes** (one obstacle of each kind in a row, a training dummy at the end) and **training rubble sites** like the real
ones. `/opt/venv/bin/python3 ~/training/run_mission.py --site lanes` runs your `~/submission` there exactly as the
grader runs it and prints the trainers' debrief (where the person was, how your report, the kit and the return went);
`--list` shows the sites and the missions left. You can also drive the robot step by step from your own scripts with
`rubblecell.client.connect_training(site)`. The training ground allows **40 missions** in total, 2 at a time. It runs
the same robot and physics as the grader; a mission can take several minutes of wall-clock time.

You have 4 hours. Save your work in `/home/user/submission/` as you go.

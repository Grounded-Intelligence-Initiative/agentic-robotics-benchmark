# Catch falling rods in mid-air under an overhead feeder, with a mobile dual-arm robot that sees only through its cameras

You are the line worker at the unloading station of a small packing cell, and you do the job through a robot. The parts
are rods in sponge sleeves. Ten of them hang upright under an overhead feeder's plate at all times, in a rack of two
rows of five. Every few seconds the feeder lets one of them go, **and nothing shows beforehand which one**: it falls
straight down and, unless it is caught, hits the floor. Your program must make the robot catch each rod **while it
falls** and set it down in the tote beside it.

**This is a highly dynamic task.** Nothing waits for the robot: from the moment a rod is let go it is in free fall and
reaches the floor in about half a second. No rod may be touched while it hangs, nor before it has fallen 0.25 m: a rod
taken out of the rack at rest counts as lost. When it may first be touched it is moving at more than 2 m/s, and the
whole of it passes any one height within about a tenth of a second. A hand can wait under one place of the rack, and
there are ten: seeing which rod has been let go, getting a hand under it in the fraction of a second that is left and
closing on it as it passes is the whole job.

A Galaxea R1Pro mobile dual-arm robot works the cell: a holonomic base, a four-joint torso, two seven-joint arms with
parallel grippers (opening 0.127 m; **a part stays in a hand by finger friction alone**: the simulator does not
attach it to the hand; the finger pads are rubber and grip well, and a part may be taken with either hand or
with both), a head camera and one camera in each wrist. The space under the feeder and the tote
are within reach of the right arm from where the robot starts. The feeder's plate is about 0.5 m in front of the robot, 0.3 m
to its right and 1.55 to 1.70 m above the floor: higher than the head camera looks from the starting posture, so a rod
hanging there may be seen only in part, or not at all, until a camera is pointed at it (a wrist camera turned up, or the
torso leaned back).

## What you produce (in `/home/user/submission/`)

`run.sh`, which starts your controller. The grader talks to it over standard input and output, one JSON object per
line: your program prints `READY`, then answers `{"type": "reset"}` at the start of each shift and
`{"type": "act", "t": ..., "obs": ...}` for every step, until `{"type": "close"}`. `~/example_controller/` is a complete
controller on this wire that holds still.

- Observation: the robot's 61 proprioceptive values, base odometry, the step index, and the camera frames you asked
  for in your previous reply (RGB as PNG, depth in millimetres, the intrinsic matrix and the camera's pose in the robot
  base frame). Nothing else: no part pose, speed, kind or mass, no signal that a part is coming, no credit so far.
- Reply: `{"action": [26 numbers]}`. The first 23 are one row of the robot's native command at 30 Hz (base velocity 3,
  torso joint targets 4, left arm 7, left gripper 1, right arm 7, right gripper 1; grippers -1 closes, 1 opens). The
  last 3 are camera flags (head, left wrist, right wrist): above 0.5 puts that camera's frame in the next observation.
- A step may be asked more than once with the same `t`; repeated requests do not advance the cell. Answer each from
  the observation it carries. After an aborted shift `run.sh` is started again and must print `READY` again.

## Time: the cell does not wait for your program

The cell runs at 30 Hz, and it runs while your program computes. A reply that arrives within one control period
(1/30 s of your program's own processor time, summed over its processes; a reply that takes more than 2 s on the
clock counts by the clock) is applied on the next step.
**Every further period your reply takes is one more step the cell runs first**, with the arms and grippers on their
previous targets and the base stopped, and the rod doing whatever it was doing: a reply that takes 0.3 s meets a cell
0.3 s older than the observation it was computed from. Nothing is scored for being fast or slow; time only decides
what state of the cell your action meets. `practice.py` applies the same rule and prints how many steps went by this way.

**Every camera frame you ask for also costs 0.1 s**: the cell advances three more steps before your next action is
applied. How often to look, and with which camera, is yours to decide.

## A shift

A graded shift is 20 rods let go, one at a time. Every rod is different: a metal core in a sponge sleeve, 25 to
60 mm across, 0.28 to 0.42 m long, 0.05 to 0.40 kg, each in its own colour. The sleeve is soft and grips well: what
closes on it firmly holds it. The rack has ten places, two rows of five, 0.10 m apart both ways, under a plate 0.26 m
deep and 0.56 m wide; the rods hang by their tops, so their lower ends are at different heights. When a rod's event
is over, a new rod is hung in the place it left, so ten always hang. Three household items lie on a side table and
are not parts: leave them where they are.

Each shift has its own constants, fixed for the whole shift and hidden: where the feeder stands, how high its plate
is, how heavy this shift's rods are. **Which place of the rack lets go next is drawn anew every time, every one of the
ten places equally likely**, and nothing announces it: there is no favourite place to wait under. It has to be seen,
and the hand has to get there while the rod falls.

A part's event ends when one of these happens:

- it rests in the tote with no finger on it;
- it touches the floor;
- **a finger touches it before it has fallen 0.25 m, or touches any rod that still hangs** while this rod's event runs
  (taking a rod out of the rack is not a way to catch it; keep the hands below the rack);
- the place it left is refilled (see below), or 10 s have passed since it was let go.

**The feeder keeps its own timetable; your robot has no say in it.** Rods are let go 7 to 11 s apart, at moments fixed
before the shift starts, whether or not the robot is ready. The place a rod left is refilled 1 to 4 s before the next
rod is let go, and with that the previous rod's time is up wherever it is: in the hand, in the air or half-way to the
tote, it is cleared away and counts as lost (or as held, if it had been).

## Credit

Judged on the cell's own physical state, per part:

| outcome | credit |
|---|---|
| resting in the tote, no finger on it | 1.0 |
| held for 1 s (both fingers of one gripper on it, clear of the feeder and the tote, not slipping), but not in the tote at the end | 0.5 |
| anything else | 0 |
| a bystander item moved more than 5 cm | -0.5 each, once |

`part_pickup_score` = the mean credit over the shift's 20 parts, not below 0. The reward is its mean over the hidden
shifts, relative to the reference recorded with the task. The grader also reports, unscored, the score of each shift's
first six and last six parts.

## What you are given

- `~/lab/`: the cell itself, as source (`cell_env.py` and the files it uses), with the same robot, feeder, tote, items,
  cameras, physics and prices as the graded shifts. `python3 ~/lab/practice.py --seed N` runs your controller on a
  practice shift exactly as the grader will and prints each part's outcome. Practice seeds are yours to choose; the
  graded shifts use seeds that are not in the image, with their own constants. `kin.py` is the robot's forward and
  inverse kinematics from its URDF.
- `~/example_controller/`: a complete controller without a policy (`run.sh` and one Python file). It scores 0.

The simulator runs under `/opt/sim/bin/python`; `python3` is a separate interpreter with numpy, scipy, pillow, OpenCV,
PyTorch and torchvision. There is no detector, tracker or pose estimator in the image, and no pretrained weights:
seeing the parts is part of the task.

**Training models is allowed.** The lab is yours to use as a data source: because you run the cell yourself there, you
can read every part's true pose from `cell_env.py` next to the camera frames and build whatever labelled data, model
or policy you want within your time, on the machine's GPU. Anything your controller needs at grading time (weights
included) must be inside `/home/user/submission/`; the grader gives it camera frames and proprioception only.

## Limits

4 h of wall time for you. No single reply may take more than 30 s. The network is closed.

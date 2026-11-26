# Take moving parts off an inclined chute with a mobile dual-arm robot that sees only through its cameras

You are the line worker at the unloading station of a small packing cell, and you do the job through a robot. Parts come down a gently inclined transfer chute one at a
time: each is released near the top and rolls or slides toward the low end. Your program must make the robot take each
part off the chute **while it moves** and set it down in the tote beside the chute.

**This is a highly dynamic task.** Nothing waits for the robot: every part is in motion from the moment it is released, it is gone within seconds, and it must be taken while it moves. A part that is stopped and picked up at rest counts as lost. Seeing where a part is, predicting where it will be and having the hand there in time is the whole job.

A Galaxea R1Pro mobile dual-arm robot works the cell: a holonomic base, a four-joint torso, two seven-joint arms with
parallel grippers (opening 0.127 m; **a part stays in a hand by finger friction alone**: the simulator does not
attach it to the hand), a head camera and one camera in each wrist. The chute's low end and the tote are
within reach of the right arm from where the robot starts.

## What you produce (in `/home/user/submission/`)

`run.sh`, which starts your controller. The grader talks to it over standard input and output, one JSON object per
line: your program prints `READY`, then answers `{"type": "reset"}` at the start of each shift and
`{"type": "act", "t": ..., "obs": ...}` for every step, until `{"type": "close"}`. `~/example_controller/` is a complete
controller on this wire that holds still.

- Observation: the robot's 61 proprioceptive values, base odometry, the step index, and the camera frames that have
  arrived (asked for some steps ago, see Time) (RGB as PNG, depth in millimetres, the intrinsic matrix and the camera's pose in the robot
  base frame). Nothing else: no part pose, speed, kind or mass, no signal that a part is coming, no credit so far.
- Reply: `{"action": [26 numbers]}`. The first 23 are one row of the robot's native command at 30 Hz (base velocity 3,
  torso joint targets 4, left arm 7, left gripper 1, right arm 7, right gripper 1; grippers -1 closes, 1 opens). The
  last 3 are camera flags (head, left wrist, right wrist): above 0.5 puts that camera's frame in the next observation.
- A step may be asked more than once with the same `t`; repeated requests do not advance the cell. Answer each from
  the observation it carries. After an aborted shift `run.sh` is started again and must print `READY` again.

## Time: the cell runs in real time, what you see is already past, and your program has to keep up

The cell runs in simulated time at 30 Hz: one step per action.

**Looking takes time.** Every camera frame you ask for costs 0.1 s: the cell advances three more steps, with the arms
holding their targets and the base stopped, before your next action is applied, and the part keeps moving through all
of them.

**What you see is late.** A frame does not come back with the next observation. It is taken when its 0.1 s is over
and reaches you a fixed number of steps later, as a real camera and its processing would: between 0.10 and 0.20 s (3 to
6 steps), the same for every frame of a shift and hidden. Each frame carries `t_capture`, the step it was taken at, and
the camera's pose at that step; the observation it arrives in carries the current step `t` and the current joint
readings. A part moving at 0.4 m/s is 4 to 8 cm past where a frame shows it by the time you can act on it. The first
observation of a shift is the one exception: its frames are current.

**Your program must keep pace, at every step.** The grader measures your program's own time (the time it waits for
your replies, not the time the simulator takes):

- **Each reply may use 0.1 s.** The time counted is your program's CPU time, summed over all its processes and
  threads (so a busy grading machine cannot make you late, and more threads do not buy more time); a reply that takes
  more than 10 s of wall time counts in full, however little CPU it used. A reply over 0.1 s is a late reply; three are
  forgiven in a shift, and **the fourth ends the shift**, which scores what it had earned. The first reply of a shift
  may take up to 5 s.
- Over a shift, your program's time (counted the same way) may not exceed the simulated time elapsed plus 10 s, or the shift ends the same
  way. On average, then, a reply has 33 ms, and each frame you ask for buys 100 ms more.

You cannot stop the world to think: a controller has to decide in a control period or two, as on a real robot. These
are thresholds, never scored times: a program that stays within them scores the same on a fast and a slow machine.
`practice.py` applies the same rules and prints your times, your late replies and the shift's latency.

## A shift

A graded shift is 20 parts, one at a time, released at moments you are not told. The 20 parts of a shift are 20
different items (cans, boxes, bottles, a bowl, a mug, a drill, scissors, a marker, clamps, a banana, a foam brick and
others, sizes and masses in `/opt/cell_items/items.json`), each with its real shape, mass and printed surface: round ones roll, flat ones slide, and
no two sit in a gripper alike. Some are wider than one gripper opens in the pose they come down in: both arms are yours to use, on any part and in
any way (one hand, or the two together). Three more items
lie on a side table and are not parts: leave them where they are.

Each shift has its own constants, fixed for the whole shift and hidden: the chute's slope, its surface friction, the
speed parts are released at and how heavy this shift's parts are. A program that works them out from the first parts
can do better on the later ones.

A part's event ends when one of these happens:

- it rests in the tote with no finger on it;
- it reaches the low end of the chute, where the end stop is;
- it touches the floor;
- **it stands still on the chute for 1 s without being held** (a jam: blocking a part and picking it up at leisure is
  not a way to take it);
- the next part is put on the chute (6 to 12 s after this one's release).

**The line feeds at its own pace.** Its schedule is fixed before the shift starts and nothing your controller does
changes it: a new part is let go every 9 to 13 s, at random, and is put at the top of the chute 1 to 3 s before that.
A part's event is over at the latest when the next part is put on the chute: whatever is still in the hand or on the
chute then is gone. You are not told the schedule.

## Credit

Judged on the cell's own physical state, per part:

| outcome | credit |
|---|---|
| resting in the tote, no finger on it | 1.0 |
| held for 1 s (both fingers of one gripper on it, clear of the chute and the tote, not slipping), but not in the tote at the end | 0.5 |
| anything else | 0 |
| a bystander item moved more than 5 cm | -0.5 each, once |

`part_pickup_score` = the mean credit over the shift's 20 parts, not below 0. The reward is its mean over the hidden
shifts, relative to the reference recorded with the task. The grader also reports, unscored, the score of each shift's
first six and last six parts.

## What you are given

- `~/lab/`: the cell itself, as source (`cell_env.py` and the files it uses), with the same robot, chute, tote, items,
  cameras, physics and prices as the graded shifts. `python3 ~/lab/practice.py --seed N` runs your controller on a
  practice shift exactly as the grader will and prints each part's outcome. Practice seeds are yours to choose. The
  graded shifts are drawn with a secret that is not in the image: no seed reproduces one of them in the lab, so a
  graded shift cannot be recognised from its first parts. `kin.py` is the robot's forward and
  inverse kinematics from its URDF.
- `~/example_controller/`: a complete controller without a policy (`run.sh` and one Python file). It scores 0.

The simulator runs under `/opt/sim/bin/python`; `python3` is a separate interpreter with numpy, scipy, pillow, OpenCV,
PyTorch and torchvision. There is no detector, tracker or pose estimator in the image, and no pretrained weights:
seeing the parts is part of the task.

**Training models is allowed.** The lab is yours to use as a data source: because you run the cell yourself there, you
can read every part's true pose from `cell_env.py` next to the camera frames and build whatever labelled data, model
or policy you want within your time, on the machine's GPU. Anything your controller needs at grading time (weights
included) must be inside `/home/user/submission/`; the grader gives it camera frames and proprioception only.

## Rules: the cameras are the only way to know about a part

At grading your controller runs alone, as your account, on the machine where the grader runs the cell. It may use
what it is sent (joint readings, odometry, the step index, the frames it asked for), the files in
`/home/user/submission/`, and the public item list. Everything else is out of bounds, and a submission that does any
of the following is disqualified (reward 0), whatever it scored:

- reading, or trying to read, anything the grader or the simulator writes or holds: their files, temporary folders,
  logs, processes, memory, the GPU's state, the grader's folder, another account's files;
- obtaining a part's pose, kind, timing or a shift's constants in any way other than from its own observations of that
  shift (no reproduction of graded shifts, no guessing of the grader's secret, no side channel);
- talking to anything but the grader's pipe: no network, no other process's sockets, no simulator of its own driven
  with information it should not have;
- tampering with the grader, its files, its timing or the machine (changing clocks or priorities, exhausting
  memory or the GPU, leaving background processes from your session).

A shift in which parts are taken although not one frame was ever asked for is void. Predicting from physics what
you have seen (a part you saw twice will keep accelerating down the chute), learning a shift's constants from its first
parts, and using the item list are all fair: that is the task. If you find a way to score without seeing the parts,
it is a defect of the task, not a solution: say so in `/home/user/submission/NOTES.md` and do not use it.

## Limits

4 h of wall time for you. The real-time rules above bound every reply. The network is closed.

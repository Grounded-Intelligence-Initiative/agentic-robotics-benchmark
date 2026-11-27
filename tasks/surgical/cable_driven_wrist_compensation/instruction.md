# Make any cable-driven surgical wrist go where it is told

A research group is adding autonomous suturing to a laparoscopic robot. Its needle drivers
are 8 mm instruments whose three-joint wrist (pitch and two jaws) is driven through cables by
three motors in the robot's drive unit. Inside a patient nothing measures the wrist: the only
sensors are the motor encoders, so where the jaws end up depends entirely on the motor
commands the robot sends. With the stock firmware, the jaw tips miss the requested positions
by several millimetres, and even a careful recalibration of the cable relations leaves errors
of a few tenths of a millimetre, which is too much for placing a needle.

The instruments are reusable and swapped between procedures, and no two are alike. So every
time an instrument is mounted, the robot makes a two-minute calibration recording: an operator
moves the wrist about through the stock firmware while the robot's stereo endoscope films it.

Write the **self-calibrating feed-forward compensator**: a program that, given a freshly
mounted instrument's calibration recording and the wrist trajectories the planner wants,
returns the motor commands that make that instrument follow them as closely as possible. It
will meet instruments it has never seen.

You have 3 hours.

## What you have (in `/home/user`)

- `datasheet.md`: the instrument model's kinematics, nominal cable relations and
  tolerances, the drive's servo and command timing, the endoscope and the instrument's shape,
  and the format of a calibration recording. Read it first; it is part of this task's
  specification.
- `instrument_model/`: the instrument's CAD model (triangle meshes, one per rigid part), as the
  datasheet describes.
- `instruments/instrument_a/`, `instrument_b/`, `instrument_c/`: three instruments, each a
  folder exactly as your program will receive one: `calibration/` (the recording: motor log,
  stereo endoscope video, about 85 MB, and the endoscope pose the robot reports) and
  `request.json` (four trajectories). Nothing tells you where these wrists actually went.
- `tools/check_response.py`: `python3 ~/tools/check_response.py <request> <response>` checks a
  response's shape (not its quality).

Python 3.12 with numpy, scipy, pandas, matplotlib, Pillow and MuJoCo (with software OpenGL
for offscreen rendering, no GPU) is installed (`python3`). There is no network, no simulator
of the instruments and no score feedback in your session.

## What you submit

A program in `/home/user/submission/` that runs as

```
bash /home/user/submission/run.sh <instrument_dir> <response.json>
```

with `/home/user/submission/` as its working directory. `<instrument_dir>` is a folder in the
format of `instruments/instrument_a/`; the program reads its `calibration/` and its
`request.json` and writes the response. Put everything it needs in `/home/user/submission/`;
nothing else of your session is kept, and the program cannot read `~/instruments` when it is
graded. There is no submit step: what is in the folder when your session ends is graded.

**Request** (`request.json`):

```json
{"dt_s": 0.005, "columns": ["pitch", "yaw", "grip"],
 "episodes": [{"id": "ep_00_3f2a91", "desired": [[pitch, yaw, grip], ...]}, ...]}
```

Each episode's `desired` is N rows (N = 4000 for a 20 s episode) of the wrist pose wanted at
`t_k = k * dt_s`: pitch and yaw in radians, grip (the opening between the jaws) in radians.
`jaw 1 = yaw + grip / 2`, `jaw 2 = yaw - grip / 2`.

**Response:**

```json
{"episodes": [{"id": "ep_00_3f2a91", "motor_cmd": [[m_pitch, m_jaw1, m_jaw2], ...]}, ...]}
```

One entry per requested episode with the same `id`, and exactly N rows of three finite motor
commands in radians. Row k is the command the drive sends at `t_k` and holds until `t_{k+1}`.
The episodes are run on the instrument right after its calibration recording, back to back
in the request's order; the drive does not settle the wrist in between (see the datasheet).

## How it is graded

After your session, eight new instruments are mounted, one after the other, from the same
production as the three you have (every parameter within the datasheet's tolerances). For
each, the robot makes its calibration recording exactly as it made yours, and your program
is run once, at your account's privileges, on that instrument's folder with a request of six
hidden 20-second trajectories drawn from the same distribution as the example requests:
reaches, holds, small fast corrections, grasps and suturing arcs across the wrist's working
range. It must finish within 10 minutes per instrument on 2 CPU cores and 4 GB of memory.

Each episode's commands are then played on its instrument. From run to run the cables and the
servos behave slightly differently, by a few percent. At every command instant after the
first 0.25 s the positions of both jaw tips are compared with the positions the requested
pose puts them at (the datasheet's kinematics with that instrument's true dimensions). The
score is the **root-mean-square jaw-tip position error in millimetres** over all instruments,
episodes, both jaws and all those instants: lower is better, and it is compared with what a
careful reference compensator achieves.

An episode that is missing from the response, has the wrong number of rows or contains a
non-finite value is played with the stock firmware's commands for that trajectory instead
(the datasheet's nominal relations), which misses by millimetres; so is every episode of an
instrument whose program run does not finish in time without a response. Commands beyond
+-8 rad are clipped by the drive. If the program produces no usable commands at all, the
score is zero.

# Find the TV remote

A Franka Panda arm on a floor rail stands in a living room. Somewhere in that room is the TV
remote, and nobody remembers where it was left: when an episode starts, nothing of it shows in
either of the robot's cameras. Write the controller that finds the remote and takes it out.

You are told what the robot is, how to talk to it and what the remote looks like. You are **not**
told where a remote may be in this room, and no list of places exists in anything you are given:
where to look, in what order, how to get at each place and what in the room could help you reach
it are yours to work out, the way a person who has lost a remote works it out. Whatever is in
the room may be moved, opened or used.

**There is no simulator of the room and no practice episode.** Nothing in your sandbox runs the
room, replays an episode or scores a controller. You prepare the controller from this page, the
robot's model, a few recorded first observations of the room and the remote's model (you may
build any test rig of your own); it then runs on the graded episodes as it is, like a program
sent to a customer's home.

You have **6 hours** of wall time. Have the controller in `/home/user/submission/` before the time
is up. The sandbox has 4 CPU cores, 12 GB of memory, no GPU and no network.

## What you are given (`/home/user/given/`)

| Path | What it is |
|---|---|
| `robot/panda_on_rail.xml` | The robot alone, as a MuJoCo model: the rail, the pedestal, the Panda with its hand, the wrist camera, the fixed overview camera, the actuators and the wrist sensor, exactly as they are in the room (joints `rail`, `joint1` to `joint7`, `finger_joint1`, `finger_joint2`; site `grip_site` is the point between the fingertips). Nothing of the room is in it. |
| `survey/site_1`, `site_2`, `site_3` | The first observation of three episodes that are not graded, as the wire delivers it (`observation.json`, see "Observation and action") and as pictures (`overview_rgb.png`, `wrist_rgb.png`; `*_depth_mm.png` are 16-bit, in millimetres). The graded episodes are drawn by the same rules. These frames are all you see of the room before your controller is in it. |
| `remote/` | The remote: its model (`remote.obj`, metres, with `remote.png`) and two pictures of it. |
| `example_controller/controller.py` | A minimal controller for the wire that holds still and decodes one pair of images. |
| `wire_check.py` | Starts your `run.sh` the way the grader does and checks the protocol and the deadlines against recorded observations. Nothing is simulated and no score is printed. |

Python 3.12 is `/opt/venv/bin/python3` (NumPy, SciPy, OpenCV (headless), Pillow, MuJoCo 3.16.0 and
the robosuite 1.5.2 asset files of the Panda, which the robot's model refers to); call it by that
path. MuJoCo renders offscreen on the CPU (`MUJOCO_GL=egl`, already set).

## What is known about the room

World frame: metres and radians, quaternions (w, x, y, z). The rail runs along y at x = 0, x points
from the rail into the room, y to the robot's left, z up; the floor is z = 0.

- **Robot.** The Panda's base stands 0.30 m above the floor on a pedestal that rides the rail
  (pedestal y from -1.75 to 1.10). The rail's beam is 14 cm wide and 4 cm tall. The hand opens to
  8 cm; each finger presses with at most 35 N. The arm is position controlled (see "Action").
- **Cameras.** Two RGB-D pinhole cameras, 320 x 240, principal point in the middle of the image
  (the centre of pixel (u, v) is at (u + 0.5, v + 0.5)). The overview camera is fixed at
  (-0.80, -0.15, 1.80) and looks at the point (0.55, -0.15, 0.25), image up being world up; its
  vertical field of view is 58 degrees (fx = fy = 216.5 pixels). The wrist camera sits on the
  hand's side and looks along the fingers, tilted 15 degrees toward the fingertips; its vertical
  field of view is 70 degrees (fx = fy = 171.4 pixels), and its pose comes with every observation.
- **Furniture.** A two-seat settee and a chest of drawers stand in front of the rail, on its +x
  side, as the survey frames show them. They are the same pieces in every episode, but they do
  not stand in the same place twice: each is shifted by a few centimetres and turned by a few
  degrees from one episode to the next, and no observation says by how much. Neither moves when
  pushed. Everything else in the room is loose and behaves as such: what is pushed moves, what is
  dropped falls where it falls, and the arm cannot pass through anything.
- **The remote.** 17 x 4.4 x 1.8 cm, 0.20 kg, dark (`remote/`). It lies flat, in any direction.
- **Other small things** lie about the room, some of them dark and about the remote's size.
  Taking one of them out earns nothing: only the remote counts.
- **What changes between episodes:** where the remote is, where the furniture stands, where the
  small things lie, how the room looks (the tint of the fabric, the woods, the floor and the
  wall, the floor's boards, the lamp) and the sensor noise.

Nothing else about the room is given: its sizes, its mechanisms and its contents have to be
measured from the survey frames and, in an episode, from what your controller sees and feels.

## Observation and action

An observation is a JSON object:

- `t`: the step index.
- `q`, `dq`: position and velocity of the rail (m) and the seven arm joints (rad), 8 numbers each.
- `gripper`: the opening between the fingers (m).
- `ee_pos`, `ee_quat`: the pose of the point between the fingertips (the hand's z axis is its
  approach direction, its x axis the fingers' closing direction).
- `wrench`: force (N) and torque (N m) at the wrist, in that point's frame turned to the world's
  axes, 6 numbers.
- `T_world_wrist`: the 4 x 4 pose of the wrist camera (x right, y down, z forward).
- `images`: `{"overview": {"rgb": ..., "depth": ...}, "wrist": {...}}` with only the cameras the
  previous action asked for (the first observation of an episode carries both). An image is
  `{"dtype", "shape", "b64"}`: raw row-major bytes in base64; `rgb` is uint8 240 x 320 x 3, `depth`
  is uint16 240 x 320 in millimetres along the optical axis (0: nothing within 6 m).

Readings carry a little noise. There is no object state.

An action is a list of 10 numbers `[rail, q1, ..., q7, grip, cams]` at 20 Hz: position targets
for the rail and the seven joints, the finger opening (0 to 0.08 m) and a camera mask for the next
observation (1 = overview, 2 = wrist, 3 = both, 0 = none). Targets are clipped to the joint ranges
and followed no faster than 0.5 m/s (rail), 1.4 rad/s (joints 1 to 4), 1.8 rad/s (joints 5 to 7)
and 0.10 m/s (fingers). An episode lasts at most **3600 steps** (180 s simulated).

## How an episode is judged

The grader's own simulator decides, from the state of the room; nothing your controller prints
counts, and the controller is never told how it is doing.

- **Retrieved (score 1, and the episode ends):** the remote is held by both finger pads, touches
  nothing else, is at least 8 cm above the surface it lay on when the episode began, has nothing
  but the robot above it, and stays so for 20 steps in a row.
- **Laid open (score 0.4):** nothing but the robot is above the remote, for 5 steps in a row.
- **Seen (score 0.2):** the remote shows (at least 60 pixels of it) in an image your controller
  asked for.
- Otherwise 0. An episode in which the controller crashes, answers late or malformed, or drives
  an arm joint faster than 8 rad/s scores 0.

Your result is the **mean score over 12 graded episodes**, in a fixed order and with seeds you do
not know. It is compared with a reference controller's result on the same episodes; the reference
retrieves the remote in every one of them.

## Deliverable and the wire

Leave a controller in `/home/user/submission/` with a `run.sh`. The grader starts it as
`bash run.sh` with that folder as working directory, under your user account, in a fresh sandbox
that holds this image and your submission folder and nothing else of your home: everything the
controller needs, files of `given/` included, must be in that folder. One JSON object per line on
stdin and stdout; keep stdout for the protocol and send diagnostics to stderr.

```
                                  <- READY                  (when you are loaded)
{"type": "reset"}                 -> {"ok": true}
{"type": "act", "t": 0, "obs": {...}}
                                  -> {"action": [10 numbers]}
{"type": "close"}                 -> {"ok": true}, then exit
```

Deadlines: print `READY` within 120 s of launch and answer every request within 30 s. That 30 s
is one budget per request, timed from when the request is sent: blank or partial lines count
against it. A missed deadline, a reply that is not one JSON object with an `action` of 10 finite
numbers (the last an integer from 0 to 3), or a crash aborts the episode, which then scores 0;
the grader then stops your process and starts `bash run.sh` again for the next episode (it must
print `READY` again), and after three such restarts every remaining episode aborts. So `run.sh`
has to work when started again.

`t` is the step index within the episode, starting at 0 after each `reset`. A step may be asked
more than once with the same `t`; answer each request from the observation it carries. A repeated
request does not advance the episode, so do not count requests as steps; use `t`. At about 1 % of
the steps the grader asks the same `t` twice, once with the real observation and once with a copy
whose joint readings are shifted slightly, in a hidden order, and applies only the reply to the
real one. Whether the two replies differ is recorded and does not change your score.

**Time.** Every episode has a controller budget of **900 s**: the time the grader waits for your
replies plus 0.2 s for every image you ask for (the two images of an episode's first observation
are free). The simulation's own time is not charged. An episode whose budget runs out stops there
and is scored as it stands.

## Checking your work

`/opt/venv/bin/python3 /home/user/given/wire_check.py` starts your `run.sh` twice, the way the
grader does, and checks every reply's form and the deadlines, with some steps asked twice. It
feeds the recorded first observation of a survey site over and over: the robot in it never
moves. It shows that the protocol works, not how the controller would do in the room.

A controller written without a room to try it in has to protect itself: it should measure before
it moves, move slowly near things, watch the wrist force, check after each step whether what it
meant to do has happened, and go on to the next place when something does not work.

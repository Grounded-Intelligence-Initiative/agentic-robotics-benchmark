# Find the TV remote

A Franka Panda arm on a floor rail stands in a living room, in front of a two-seat settee and a
chest of drawers. Somewhere in the room is the TV remote, out of sight. Write the controller that
finds it and takes it out: in every episode the remote is in **one of six places**, and nothing of
it shows in either camera when the episode starts.

| Hiding spot | Where the remote is | What it takes |
|---|---|---|
| `left_cushion`, `right_cushion` | on the seat deck, under one of the settee's two loose seat pads (left is the robot's left) | get the pad off it, then pick it up |
| `upper_drawer`, `lower_drawer` | in the top or the bottom drawer of the chest (the chest has three drawers; the remote is never in the middle one) | pull the drawer out by a knob, then pick it up |
| `under_hat` | on top of the chest, under the crown of the sun hat that lies there (the hat is there in every episode; its brim stands out 4 to 5.5 cm over the top's front edge) | take the hat off it, then pick it up |
| `under_sofa` | on the floor under the settee, 15 to 19 cm behind its front face, where the gap to the floor is 6 cm: the hand does not fit in | a wooden clothes hanger hangs on a peg on the chest's outer side: fetch it, bring the remote out with it, then pick it up |

**The remote is not the only thing put away in this room.** In every episode a phone and a glasses
case lie somewhere a remote could be: the phone under a seat pad, in one of the three drawers or
(when the remote is not there) under the hat; the case in one of the three drawers. Either may
lie in the same place as the remote. Taking one of them out earns nothing: only the remote counts.

Your controller sees the room through two RGB-D cameras (a fixed overview and one on the wrist),
the joint readings and a wrist force/torque sensor. It is told nothing about where the remote is
and gets no object poses of any kind. **The furniture does not stand in the same place twice**: in
every episode the settee and the chest are each shifted and turned a little from the nominal
layout (see "The room"), and no observation says by how much. The remote's spot, its position and
direction there, where the phone and the glasses case lie, how the room looks (the tint of the fabric, the woods, the floor and the wall,
the floor's boards, the lamp) and the sensor noise change too.

You have **6 hours** of wall time. Have the controller in `/home/user/submission/` before the time
is up. The sandbox has 4 CPU cores, 12 GB of memory, no GPU and no network.

## What is in your home directory

| Path | What it is |
|---|---|
| `remote_env.py` | The simulator, complete: the room's geometry as named constants, the robot and its controller, the cameras, how an episode is drawn from a seed (`make_episode(seed, spot)`), how an episode is judged (`Env.progress`, `Env.score`). The grader runs a byte-identical copy. Read it: it is the data sheet. Copy it into your submission if your controller imports it. |
| `practice_grader/grade.py` | Runs your controller over the grading wire on practice episodes of your choice and prints the raw metric (see "Checking your work"). |
| `example_controller/controller.py` | A minimal controller for the wire that holds still and decodes one pair of images. |

Python 3.12 is `/opt/venv/bin/python3` (NumPy, SciPy, OpenCV (headless), Pillow, MuJoCo 3.16.0 and
the robosuite 1.5.2 asset files of the Panda); call it by that path. MuJoCo renders offscreen on
the CPU (`MUJOCO_GL=egl`, already set). The room's meshes and textures are in `/opt/room`.

## The room

World frame: metres and radians, quaternions (w, x, y, z). x points from the rail to the settee, y
to the robot's left, z up; the floor is z = 0. All numbers below are constants of `remote_env.py`.

The positions below are the **nominal layout**. In an episode the settee, with its pads and
whatever lies on or under it, is that layout turned about the vertical through the middle of its
front face and then shifted; the chest, with its drawers, the hanger, the hat and whatever lies in
or on it, likewise about the middle of its front. For each of the two, independently and
uniformly: the shift is -1 to +5 cm along x and -5 to +5 cm along y, the turn -0.09 to +0.09 rad
(about 5 degrees) (`LAYOUT_RANGE`, `place`). A point 0.3 m from a pivot can so be 6 cm from where
the nominal layout has it. The robot, its rail and the overview camera do not move.

- **Robot.** The Panda's base stands 0.30 m above the floor on a pedestal that rides a rail along
  y at x = 0 (pedestal y from -1.75 to 1.10). The rail's beam is 14 cm wide and 4 cm tall. The
  hand opens to 8 cm; each finger presses with at most 35 N.
- **Settee.** Its front face is the plane x = 0.36, its armrests' outer faces are at y = -0.32 and
  0.82. The seat deck is flat, at z = 0.25; the two seat pads (46 x 49 x 6.5 cm, 0.55 kg,
  loose) lie on it side by side, their front edges 3 cm proud of the front face. A remote under
  a pad lies on the deck under the pad's front half. A pad is soft: it lies flat over the
  remote and hides it (in the simulator a pad and the remote do not touch each other). The
  settee's underside is 6 cm above the floor, on four legs at its corners.
- **Chest.** Its front is at x = 0.38 and its middle at y = -0.85; it is 0.57 m wide and 0.73 m
  tall. Each of its three drawers slides out toward the robot by up to 0.36 m and has two round
  knobs. A remote in a drawer lies on the drawer's floor, toward its front.
- **Hanger.** A flat wooden hanger, 44 cm long, hangs by its wire hook on a peg on the chest's
  outer side (the side away from the settee), 0.56 m above the floor.
- **Hat.** A cloth sun hat, 41 x 44 cm across its brim and 15.5 cm tall, 0.12 kg, lies on the
  chest's top. Its crown is hollow (inner radius 10 cm), and its brim is wavy: 2 to 3 cm above
  the top at the ends toward and away from the robot.
- **Remote.** 17 x 4.4 x 1.8 cm, 0.20 kg, dark.
- **Phone.** 15 x 7.4 x 0.9 cm, 0.18 kg, dark. **Glasses case.** 16 x 7 x 3.4 cm, 0.09 kg, dark
  blue. Both lie flat, in any direction; things that share a place are at least 4 cm apart and
  none lies on another. A pad is as soft over the phone as over the remote.

Things are solid, and their collision shapes follow what the cameras show (the settee's and the
hat's are convex decompositions of the drawn models): the arm cannot pass through the furniture,
a pad or a hat that is pushed falls where it falls, and the pedestal cannot drive through a
drawer that stands open. The settee and the chest themselves do not move when pushed.

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
  is uint16 240 x 320 in millimetres along the optical axis (0: nothing within 6 m). Both cameras
  are pinhole cameras with the principal point in the middle (`camera_intrinsics(name)`); the
  overview camera is fixed (`OVERVIEW_POS`, `OVERVIEW_LOOKAT`), the wrist camera sits on the hand's
  side and looks along the fingers.

Readings carry a little noise. There is no object state.

An action is a list of 10 numbers `[rail, q1, ..., q7, grip, cams]` at 20 Hz: position targets
for the rail and the seven joints, the finger opening (0 to 0.08 m) and a camera mask for the next
observation (1 = overview, 2 = wrist, 3 = both, 0 = none). Targets are clipped to the joint ranges
and followed no faster than 0.5 m/s (rail), 1.4 rad/s (joints 1 to 4), 1.8 rad/s (joints 5 to 7)
and 0.10 m/s (fingers). An episode lasts at most **3600 steps** (180 s simulated).

## How an episode is judged

The grader's own simulator decides; nothing your controller prints counts.

- **Retrieved (score 1, and the episode ends):** the remote is held by both finger pads, touches
  nothing else, is at least 8 cm above the surface it was hidden on (the floor, for
  `under_sofa`), has nothing but the robot above it, and stays so for 20 steps in a row.
- **Laid open (score 0.4):** nothing but the robot or the hanger is above the remote for 5 steps
  in a row: the pad or the hat is off it, its drawer is out with no other drawer over it, or it
  has been brought out from under the settee.
- **Under the settee only:** the remote showing in an image your controller asked for (at least
  60 pixels of it) scores 0.2, and scores 0.4 once the hanger has also been taken (held by both
  finger pads and touching nothing else for 5 steps in a row).
- Otherwise 0. An episode in which the controller crashes, answers late or malformed, or drives
  an arm joint faster than 8 rad/s scores 0.

Your result is the **mean score over 12 graded episodes**, two for each hiding spot, in a fixed
order and with seeds you do not know. It is compared with a reference controller's result on the
same episodes; the reference retrieves the remote in every one of them.

## Deliverable and the wire

Leave a controller in `/home/user/submission/` with a `run.sh`. The grader starts it as
`bash run.sh` with that folder as working directory, under your user account, in a fresh sandbox
that holds this image and your submission folder and nothing else of your home: everything the
controller needs must be in that folder. One JSON object per line on stdin and stdout; keep
stdout for the protocol and send diagnostics to stderr.

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

`/opt/venv/bin/python3 /home/user/practice_grader/grade.py --spots under_sofa --seeds 1,2` runs
your controller over the same wire, with the same deadlines and a few repeated-`t` queries, on the
episodes you name, and prints the raw metric with how each episode went. The graded episodes have
other seeds, and the practice grader does not know what the reference achieves.

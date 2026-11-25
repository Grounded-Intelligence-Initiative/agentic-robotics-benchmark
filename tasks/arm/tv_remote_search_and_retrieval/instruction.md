# Find the TV remote

A Franka Panda arm on a floor rail stands in a living room, in front of a two-seat settee and a
chest of drawers. Somewhere in that room is the TV remote, and nobody remembers where it was
left: when an episode starts, nothing of it shows in either of the robot's cameras. Write the
controller that finds the remote and takes it out.

**You are not told where a remote may be in this room.** No list of places exists in anything
you are given: the simulator in your home puts the remote wherever its caller says, and the rule
the graded episodes follow is the grader's alone. Where to look, in what order, how to get at
each place and what in the room could help you reach it are yours to work out, the way a person
who has lost a remote works it out. Whatever is in the room may be moved, opened or used. In the
graded episodes the remote is always in a place a remote could plausibly have got to in a living
room, and the robot can get it out of every one of them with what is in the room.

**The remote is not the only small thing in this room.** In every graded episode other small
things lie somewhere too, some of them dark and about the remote's size (the simulator's model
has them). Taking one of them out earns nothing: only the remote counts.

Your controller sees the room through two RGB-D cameras (a fixed overview and one on the wrist),
the joint readings and a wrist force/torque sensor. It gets no object poses of any kind. **The
furniture does not stand in the same place twice**: in every episode the settee and the chest are
each shifted and turned a little from the nominal layout (see "The room"), and no observation
says by how much. How the room looks (the tint of the fabric, the woods, the floor and the wall,
the floor's boards, the lamp) and the sensor noise change too.

You have **6 hours** of wall time. Have the controller in `/home/user/submission/` before the time
is up. The sandbox has 4 CPU cores, 12 GB of memory, no GPU and no network.

## What is in your home directory

| Path | What it is |
|---|---|
| `remote_env.py` | The simulator, complete: the room's geometry as named constants, the robot and its controller, the cameras, how a scene is drawn from a seed (`make_scene(seed, remote)`: you say where the remote is), how an episode is judged (`Env.progress`, `Env.score`). The grader runs a byte-identical copy and adds only its own rule for where the remote and the other small things are. Read it: it is the data sheet. Copy it into your submission if your controller imports it. |
| `practice_grader/grade.py` | Runs your controller over the grading wire on scenes of your own, with the remote where you put it, and prints the raw metric (see "Checking your work"). |
| `example_controller/controller.py` | A minimal controller for the wire that holds still and decodes one pair of images. |

Python 3.12 is `/opt/venv/bin/python3` (NumPy, SciPy, OpenCV (headless), Pillow, MuJoCo 3.16.0 and
the robosuite 1.5.2 asset files of the Panda); call it by that path. MuJoCo renders offscreen on
the CPU (`MUJOCO_GL=egl`, already set). The room's meshes and textures are in `/opt/room`.

## The room

World frame: metres and radians, quaternions (w, x, y, z). x points from the rail to the settee, y
to the robot's left, z up; the floor is z = 0. All numbers below are constants of `remote_env.py`.

The positions below are the **nominal layout**. In an episode the settee, with everything that
belongs to it, is that layout turned about the vertical through the middle of its front face and
then shifted; the chest, with everything that belongs to it, likewise about the middle of its
front. For each of the two, independently and uniformly: the shift is -1 to +5 cm along x and -5
to +5 cm along y, the turn -0.09 to +0.09 rad (about 5 degrees) (`LAYOUT_RANGE`, `place`). A point
0.3 m from a pivot can so be 6 cm from where the nominal layout has it. The robot, its rail and
the overview camera do not move.

- **Robot.** The Panda's base stands 0.30 m above the floor on a pedestal that rides a rail along
  y at x = 0 (pedestal y from -1.75 to 1.10). The rail's beam is 14 cm wide and 4 cm tall. The
  hand opens to 8 cm; each finger presses with at most 35 N.
- **Settee.** Its front face is the plane x = 0.36, its armrests' outer faces are at y = -0.32 and
  0.82.
- **Chest.** Its front is at x = 0.38 and its middle at y = -0.85; it is 0.57 m wide and 0.73 m
  tall.
- **Remote.** 17 x 4.4 x 1.8 cm, 0.20 kg, dark. It lies flat, in any direction.
- **Everything else** the room holds, fixed or loose, with its sizes, weights and how it moves,
  is in the simulator's model: `remote_env.py` is the whole description, and this page does not
  repeat it.

Things are solid, and their collision shapes follow what the cameras show: the arm cannot pass
through the furniture, a loose thing that is pushed falls where it falls, and the pedestal cannot
drive through something that stands in its way. The settee and the chest themselves do not move
when pushed.

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
  nothing else, is at least 8 cm above the surface it lay on when the episode began, has nothing
  but the robot above it, and stays so for 20 steps in a row.
- **Laid open (score 0.4):** nothing but the robot is above the remote, for 5 steps in a row.
- **Seen (score 0.2):** the remote shows (at least 60 pixels of it) in an image your controller
  asked for.
- Otherwise 0. An episode in which the controller crashes, answers late or malformed, or drives
  an arm joint faster than 8 rad/s scores 0.

Your result is the **mean score over 14 graded episodes**, in a fixed order, with seeds and
places for the remote that you do not know. It is compared with a reference controller's result on the
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

`/opt/venv/bin/python3 /home/user/practice_grader/grade.py --remote 0.25,0.30,0.02,0.4 --seeds 1,2`
runs your controller over the same wire, with the same deadlines and a few repeated-`t` queries,
on scenes of your own, and prints the raw metric with how each episode went. `--remote` is where
you put the remote (x, y, z, yaw in the world frame; it is put there and let go), `--scenes` a
JSON list of scenes for more than one place, with the other small things placed too if you want
them (the script's head says how). The example above has the remote lying in the open on the floor.
Which places are worth practising is your call: the practice grader knows neither the graded
episodes' places nor what the reference achieves.

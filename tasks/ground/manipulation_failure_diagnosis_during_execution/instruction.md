A mobile manipulator is working through kitchen tasks, and sometimes it fails. Write a **failure monitor**: a program
that watches a run while it happens, from the robot's cameras and its arm and gripper commands, and raises one
alarm as soon as the run has failed, saying what kind of failure it is and which object it concerns. A repair or
recovery step then takes over, so an alarm is worth most when it comes early, and an alarm on a run that has not failed
stops a good run.

Leave the program runnable at `/home/user/submission/run.sh`. It is graded on hidden recorded runs that it never sees
during development, from kitchen tasks that are not in your practice set, with no model calls allowed during grading.

## How a run reaches your program

The grader plays back one recorded run at a time to your program, in chunks of 10 frames (0.5 s; frames are 1/20 s
apart, frame 0 is the initial state and frame `k` is the state after control step `k`). Your program answers each
chunk before it gets the next one. It never sees frames after the chunk it is answering, and it is never told how long
a run is or will be.

Each run is one of two kinds:

* **A failed run.** It contains the failure event defined below. Its playback stops 3.0 to 5.0 s after the failure's
  onset.
* **A successful run.** The robot completed its task. Its playback stops 3.0 to 5.0 s before the frame at which the
  task was complete (the recordings end at completion). No failure event happens in the part you are shown.

Where in that range a run stops is a fixed random draw per run, the same for every submission, and is independent of
what happens in the run. Every graded run starts at frame 0, with the robot at the start of its task. You never see the
stop coming: the end of a run arrives as the next `reset`.

## The wire

`run.sh` is launched as `bash run.sh` with the working directory `/home/user/submission`, under your own user account,
with standard input and output connected to the grader, and with only three environment variables set:
`PATH=/opt/venv/bin:/usr/local/bin:/usr/bin:/bin`, `HOME=/home/user` and `LANG=C.UTF-8` (set anything else your program
needs, such as thread counts, in `run.sh`). It must start a long-lived process that speaks one JSON object
per line on standard output. Keep standard output for replies; send diagnostics to standard error (it is discarded).

```
                                         <- READY                     (your first line on stdout)
{"type": "reset"}                        -> {"ok": true}              (a new run starts)
{"type": "act", "t": 0, "obs": {...}}    -> {"action": {"alarm": false}}
{"type": "act", "t": 1, "obs": {...}}    -> {"action": {"alarm": true, "failure_type": "grasp_missed", "object": "sponge"}}
{"type": "close"}                        -> {"ok": true}, then exit
```

`t` is the chunk index within the run, starting at 0 after each `reset`; chunk `t` holds frames `10 t` to `10 t + 9`
(the last chunk of a run may be shorter). A chunk may in principle be sent more than once with the same `t`; answer each
request from the observation it carries, and count chunks by `t`, not by requests. Once your program raises an alarm,
that run's playback ends (the next message is a `reset` or `close`): one alarm per run, and it cannot be taken back.

`obs` holds:

| key | content |
|---|---|
| `instruction` | the task the robot was given, in English (the same in every chunk of a run) |
| `candidates` | the sorted list of object categories present in the scene, strings such as `"sausage"` or `"plate"`, with no indication of which the task needs (the same in every chunk) |
| `camera_model` | the agent-camera mounts, below (the same in every chunk) |
| `fps` | 20 |
| `frames` | `[start, stop)`: the frame indices in this chunk |
| `images` | path of a NumPy `.npy` file holding a `uint8` array of shape `(stop - start, 256, 768, 3)`, RGB: three 256 x 256 views side by side, left agent view, right agent view, wrist view. The file exists only until you reply; read it before replying |
| `log` | the robot's readings for those frames, one row per frame (below) |

`log` has these keys, each a list with one entry per frame of the chunk (numbers rounded to 6 decimals):

| key | entry | meaning |
|---|---|---|
| `gripper_command` | number | the policy's gripper decision for the step that produced this frame: 1 = commanded closed, 0 = commanded open (0 for frame 0) |
| `ee_pos_rel` | 3 numbers | gripper position relative to the mobile base, metres, base frame |
| `ee_rot_rel_rotvec` | 3 numbers | gripper orientation relative to the base, axis-angle |
| `base_pos` | 3 numbers | mobile base position in the world frame, metres |
| `base_rotvec` | 3 numbers | mobile base orientation in the world frame, axis-angle |

Not streamed: the fingers' state (their joint positions or the gap between them) and the wrist force and torque.
Whether the gripper holds something, closed on nothing, or let an object slip has to be judged from the camera views,
mainly the wrist view, where the fingers are visible at the bottom of the image.

`camera_model` gives the two agent cameras, which are rigidly mounted on the mobile base: `camera_model.left` and
`camera_model.right` each hold `pos` (camera position in the base frame, metres) and `quat_wxyz` (camera orientation in
the base frame, quaternion w, x, y, z). Both have the same intrinsics: image 256 x 256, vertical field of view 60 degrees,
focal length `focal_px` = 128 / tan(30 deg) = 221.70 pixels, principal point (128, 128). A world point `P` projects into a
camera at frame `t` like this, with `R(·)` the rotation matrix of a quaternion or an axis-angle vector:

```
R_b  = R(base_rotvec[t])
R_w  = R_b * R(quat_wxyz)
t_w  = base_pos[t] + R_b * pos
v    = R_w^T * (P - t_w)                  # the camera looks along -z, x right, y up
u    = 128 + focal_px * v[0] / (-v[2])
w    = 128 - focal_px * v[1] / (-v[2])    # (u, w) in image pixels, origin at the top left, valid when v[2] < 0
```

The gripper's world position at frame `t` is `base_pos[t] + R_b * ee_pos_rel[t]`. These readings are what a real robot records; no simulator
state is shown. Indices `t` here are frame indices.

## The failure events

"Held" means the gripper closed on the object so that the object moved with the gripper. A failed run's failure is
one event of the three kinds below. A drop or a missed grasp happens to an object the instruction concerns (one that
the task's goals name), and when the recording ended that object's goal, or the goal of another object of the same
category that the task treats as interchangeable (for example "one sausage on each plate"), was still unmet. A wrong
grasp happens to an object the instruction does not concern, in a run whose goals were still unmet when the recording
ended. The run's other grasps, releases and placements are not the failure.

* `dropped_after_grasp`: the gripper held the object for at least 1 s and lifted it by more than 3 cm during that hold,
  then lost it while the gripper was still commanded closed; 1.5 s after the loss the object was more than 5 cm lower
  than at the loss, and the robot never held it again. **Onset**: the first frame at which the simulator no longer
  counts the object as held (it keeps counting a hold through short contact gaps while the object stays within 4 cm of
  where the gripper carried it).
* `grasp_missed`: the object was never held at any time in the recording, and the gripper, open by at least 2 cm just
  before, was commanded closed for at least 6 frames and closed to an aperture below 8 mm, with the gripper within 8 cm
  of the object during that closure, while it held nothing else. **Onset**: the first frame of the first such closure
  (its first frame with `gripper_command` > 0). **Object**: among the objects the instruction concerns, the one whose
  centre is nearest the gripper at the first frame of that closure with an aperture below 8 mm (graded runs where that
  nearest object is not the one that was never held are left out of the set).
* `grasped_wrong_object`: the gripper closed on (both fingers touching) and lifted by at least 3 cm one of the scene's
  distractors: an object the scene holds besides the ones the task is about, which the instruction does not name and
  the task's success conditions do not involve (never a container, plate, pan or other receptacle, and never an object
  of the same category as one the task needs, such as a second apple in a task that moves an apple). **Onset**: the
  first frame at which that object is at least 3 cm above where it was when this grasp began. **Object**: the object
  that was lifted. What the robot does with it afterwards (keeps it, puts it back, grasps the right object later) does
  not change the event.

The rules were applied by program to the simulator's record of each run (contacts, object poses, the finger aperture,
which is not streamed to you, and the task's success conditions). The thresholds in them (hold, lift, fall distances)
and the mapping of each task's success conditions to the objects they concern were set by the task author.

What the graded (hidden) runs contain:

* A failed run contains exactly one event of the three kinds above: its failure. Before the onset it contains no drop
  or missed grasp of these kinds on any object and no lifted distractor, and before the onset no object the
  instruction concerns was put down and left where its goal stayed unmet. A run with a drop or a missed grasp contains
  no lifted distractor anywhere in its recording, and a wrong-grasp run has its wrong grasp at least 5 s before the
  recording ends, as do the other kinds.
* A successful run contains no drop, no missed grasp and no lifted distractor anywhere in its recording.
* Runs are left out where, before the onset, a goal condition that had held stopped holding for 2 s or more (for
  example the meat lifted out of its container: the visible mistake comes earlier than the keyed event), or an object
  the instruction concerns was put down and left without meeting a goal.
* Drops are left out where the gripper was commanded open within 0.5 s after the onset (a set-down that missed its
  place rather than an object slipping from a closed gripper).
* Runs are left out where the failed object is not clearly the nearest candidate: at the event (the closing frame for a
  missed grasp, the last held frame for a drop, the onset of a wrong grasp), the centre of some object of another
  category was less than 3 cm farther from the gripper than the failed object's centre. Not counted as rivals: the
  receptacles the task's goals name, and the object the failed object rests on or in (an object under it, touching it,
  with the failed object's centre inside its outline seen from above: its plate, bowl or container).

Practice runs follow the same rules for their failure, but may also contain drops and missed grasps from which the
robot recovered (it held that object again later, or completed the task anyway), before the failure or in a
successful run. Those are not failures.

Some scenes, hidden and practice, are RoboCasa's pick-and-place tasks with one extra object, drawn from the same pool
of objects as the one to move, placed within 12 cm of it, so that the gripper can close on the wrong one; the
instruction and the success conditions are the task's own.

## How an alarm is scored

Let `a` be the time of your alarm, the last frame of the chunk your alarm answers divided by 20, and `o` the onset time
of the run's failure (onset frame / 20).

* **Failed run.** No alarm scores 0. An alarm with `a < o - 0.5 s` is a false alarm and scores 0. Otherwise the alarm
  scores `correct x timeliness(max(0, a - o))` (an alarm up to 0.5 s early counts as at the onset), where `correct` is 1 if `failure_type` and `object` are both right (`object` equal to
  the failed object's category, one string out of `candidates`) and 0 otherwise, and

  ```
  timeliness(d) = 1                    for 0 <= d <= 1.0 s
                = (3.0 - d) / 2.0      for 1.0 s < d < 3.0 s
                = 0                    for d >= 3.0 s
  ```

  Full credit within 1.0 s: the recorded policy commits a new 0.8 s action chunk at a time, so an alarm within 1.0 s
  stops the robot before it acts on more than one more chunk. No credit after 3.0 s: in recordings of practice tasks
  the robot starts its next gripper closure a median 2.5 to 2.9 s after the onset of a missed grasp (2.92 s over 30
  missed grasps keyed without the success-term check, 2.55 s over 23 with it), so by then it has acted on the failure
  without help. The same window is used for all three types.
* **Successful run.** Any alarm is a false alarm.

Your score is `P x (1 - F)`. `P` is the mean alarm score over the failed runs of each failure type, averaged over the
three types. `F` is the fraction of the successful runs on which your program raised an alarm. Any `action` that is not
exactly one of the two forms above (`{"alarm": false}` with no other key, or `{"alarm": true}` with exactly the keys
`failure_type`, one of the three names, and `object`, a string) counts as an alarm with `correct` = 0 and ends that
run's playback.

The hidden set is fixed: 88 runs: 48 failed (16 `dropped_after_grasp`, 16 `grasp_missed`, 16 `grasped_wrong_object`) and 40 successful, 2117 s of playback in all. Your program receives no feedback on its score during grading and is scored once.

## What you get

`/home/user/practice/<id>/` holds 155 recordings: 67 failed (29 `dropped_after_grasp`, 20 `grasp_missed`, 18 `grasped_wrong_object`) and 88 successful, each a whole recording in three files plus its answer:

* `episode.json`: `instruction`, `candidates`, `camera_model`, `fps`, `n_frames` (the recording's length; the grader
  never sends this).
* `video.mp4`: all frames, 768 x 256, the same three views (decode with OpenCV, which gives BGR).
* `logs.npz`: the `log` arrays above, one row per frame.
* `key.json`: `kind` (`"failure"` or `"success"`); for a failure also `failure_type`, `onset_frame`, `onset_s`,
  `subject` (the object category); for both, `stream_frames`, the number of frames the grader would play back, and
  `tail_frames`, the drawn stop distance (frames after the onset, or before completion).

Practice runs come from other kitchen tasks than the hidden ones, so objects and instructions differ; identifiers are
random strings. Their answers were made by the same rules as the hidden ones.

`/opt/venv/bin/python3 /home/user/practice_grader/grade.py [--limit N] [--kind failure|success]` plays practice runs
to your program over the same wire, with the same playback rule, deadlines and time accounting, and prints what
happened in each run and the raw `P`, `F` and per-type fractions. It uses no information about the hidden set and
reports no score.

Python 3.12 at `/opt/venv/bin/python3` has NumPy, SciPy, OpenCV (headless), scikit-learn, PyTorch (CPU) and Transformers.
The weights for CLIP ViT-B/16 are at `/opt/models/clip-vit-base-patch16` and load with `from_pretrained` from that path.
This sandbox and the grading sandbox have the same Python, the same libraries and the same model; call the interpreter by
that path. There is no network here, and none at grading.

## Time and other limits

* **Your program's own time** is what is counted: for each request, the time from the moment the grader starts writing
  it (a request is a few kilobytes; the frames are in the file) to the moment your reply line arrives. The grader does
  nothing while it waits, and the time it spends reading the recording and writing the next chunk file is not counted.
* Each reply must arrive within 30 s of that moment (a request your program has not read by then counts as a missed
  deadline), and `READY` within 120 s of launch (load your models first, then
  print it). A line your program writes may be at most 65,536 characters long; a longer one ends the stream, as a
  closed stream does (below).
* These limits were sized on the authoring host, an Intel Core i9-14900KF, with the sandbox's 4 CPUs.
* Within a run your program must keep up with the robot: the sum of its reply times to the run's chunks must stay at
  most 20 s more than the time it has been shown (0.5 s per chunk answered). The `reset` reply, and any restart and
  `READY` wait before it, are not counted. A run where it falls further
  behind ends there and scores 0 (a successful run counts as alarmed), without restarting your program.
* A missed deadline, a crash, a closed stream or a reply without an `action` aborts the run: it scores 0 (a successful
  run counts as alarmed). The grader then stops your process and starts `bash run.sh` again for the next run, so it must
  work when started a second time: no fixed ports, PID files or locks that a previous start leaves behind. After three
  restarts, or a restart that fails, every remaining run is aborted.
* The grader runs one run at a time and keeps nothing of your program's between runs except what your process keeps
  itself. During grading your program has no IP network and no model service: only `/home/user/submission` and the
  libraries above. No hidden run is ever labelled or shown to you.
* The whole grading run has a budget of 7100 s of wall-clock time: runs not started within it score 0 (a successful run counts as
  alarmed). A program that keeps up with real time never reaches it. The verify stage is cut off at 8300 s.
* The sandbox has 4 CPUs and 16 GB of memory, no GPU. Your total working time here is 8 hours. Grading runs in a fresh
  sandbox of the same size where only `/home/user/submission` is restored. Do not start many parallel jobs here: an
  out-of-memory kill ends your work.
* `/home/user/submission` may hold at most 300 MB and 20,000 files, and no symlink may leave it.

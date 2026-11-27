# Joint angles and gripper opening of a robot arm from video

A ROBOTIS OpenManipulator-X (four revolute arm joints and a parallel gripper) was filmed by one fixed camera during a
data-collection campaign, while its controller logged the joint encoders. For 24 of the recordings the logs were lost:
only the videos remain. Recover the four arm joint angles and the gripper's aperture in every frame of these 24
videos.

You have 4 hours, 4 CPU cores, 16 GB of memory (processes that exceed it are killed), no GPU and no network.
Python 3.12 is `python3` (NumPy, SciPy, OpenCV (headless, with FFmpeg), MuJoCo 4.0.1, Pillow, scikit-image,
scikit-learn, Matplotlib, trimesh). There is no display; MuJoCo's offscreen renderer (`mujoco.Renderer`) works on the CPU (`MUJOCO_GL=osmesa` is set).

## What you get (`/home/user`)

```
robot/open_manipulator_x.xml     the robot model (MuJoCo MJCF); its meshes are in robot/assets/
labelled/<id>/video.mp4          32 recordings that still have their log: 8 of each of the motions
labelled/<id>/log.csv            circular_cw, circular_ccw, backward_forward and go_up_and_stay_still
test/T01.mp4 ... test/T24.mp4    the 24 recordings without a log
recordings.json                  every recording, with its number of frames
tools/score.py                   the scoring rule on a labelled recording, and a format check of your files
tools/jointscore.py              the scoring module (the grader uses the same module)
submission/                      write T01.csv ... T24.csv here
```

- **Videos.** Real footage, 1706 x 1440 pixels, 24 frames per second (constant), H.264. Frame k is at video time
  k/24 s; OpenCV decodes exactly the number of frames that `recordings.json` gives. The camera does not move during a
  recording. No calibration of it exists: its intrinsics and its pose are unknown, and it was put in place by hand for
  each session, so its pose differs between recordings.
- **Logs.** CSV, about 59 rows per second. `timestamp` is in seconds. `pos1` ... `pos4` are the measured angles of the
  model's `joint1` ... `joint4`, in radians; the zero positions of the encoders are close to the model's, but not
  exactly equal. `aperture` is the position of the model's two finger joints (`gripper_left_joint` and
  `gripper_right_joint`, which the model couples) in millimetres; larger is more open. In the labelled recordings the
  gripper stays closed. The other columns are the rest of the controller's telemetry (commands `goal_*`, currents,
  velocities, temperatures, PWM, voltages, force channels).
- **Clocks.** Video and log were started separately. The log starts about one second before the video: frame k of a
  video is at log time k/24 + d, where d is close to 1.0 s and differs a little between recordings. d is not given for
  any recording.
- **Test recordings.** The same robot. They show other motions than the four labelled ones, and in many of them the
  gripper opens and closes. Half of them were filmed in other sessions: there the camera stands where no labelled
  recording has it, other objects lie on the table, the gripper picks one up in some, and in some a bar is fixed to
  the arm.

## Deliverable

For every test recording `Txx`, a file `/home/user/submission/Txx.csv`: the header line `q1,q2,q3,q4,aperture`, then
one line per video frame, in frame order, with the angles of `joint1` ... `joint4` at that frame in radians and the
gripper's aperture in millimetres, as the lost log would give them (`pos1` ... `pos4`, `aperture`). The number of
lines after the header must be the recording's `frames` in `recordings.json`. `python3 tools/score.py --check`
checks the format of the files in `submission/`.

## Scoring

The grader holds the lost logs. For each test recording:

1. It compares your frame k with the log at time k/24 + d (linear interpolation), for every time offset d from 0.5 s
   to 1.5 s in steps of 1/240 s. Frames outside the time span of the log are left out.
2. A frame is correct when each of your four angles is within 5.0 degrees of the log and your aperture is within
   3.0 mm of the log. A line with a number that is not finite is a wrong frame.
3. The score of the recording is the fraction of correct frames, at the offset d that makes this fraction largest.

A missing or malformed file scores 0 for its recording. The metric, `state_accuracy_5deg_3mm`, is the mean of the 24
recording scores (higher is better; 1.0 means that every frame of every recording is correct).
`python3 tools/score.py <your csv> <labelled id>` computes the same recording score for a labelled recording.
The grader reads only the 24 CSV files; nothing you write is executed.

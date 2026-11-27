# Joint angles of a humanoid robot from a hand-held video

A Unitree G1 humanoid (29 joints) hangs on a test stand by its pelvis and moves its legs, waist and arms, while a
hand-held camera films it. The recordings are simulated: the motion is a MuJoCo simulation of the robot's own
position servos, and the videos are rendered from it. For 24 recordings the joint logs were lost: only the video
remains. Recover the 29 joint angles in every frame of these 24 recordings.

You have 4 hours, 4 CPU cores, 16 GB of memory (processes that exceed it are killed), no GPU and no network.
Python 3.12 is `python3` (NumPy, SciPy, OpenCV (headless, with FFmpeg), MuJoCo 4.0.1, Pillow, scikit-image,
scikit-learn, Matplotlib, trimesh). There is no display; MuJoCo's offscreen renderer (`mujoco.Renderer`) works on the CPU (`MUJOCO_GL=osmesa` is set).

## What you get (`/home/user`)

```
robot/g1.xml                 the robot model (MuJoCo MJCF of MuJoCo Menagerie's unitree_g1); meshes in robot/assets/
labelled/L01 ... L06/        6 recordings that still have their joint angles: video.mp4, joints.csv
test/T01 ... T24/            the 24 recordings without joint angles: video.mp4
recordings.json              every recording, with its number of frames
tools/score.py               the scoring rule on a labelled recording, and a format check of your files
tools/humanoidscore.py       the scoring module (the grader uses the same module and tools/g1_keypoints.xml)
submission/                  write T01.csv ... T24.csv here
```

- **Robot.** The 29 hinge joints of the model, in the order of `JOINTS` in `tools/humanoidscore.py` (the order of
  `g1.xml`). The model's free joint is not used: the stand holds the pelvis fixed and upright, facing the model's +x
  direction, in every recording.
- **Motion.** Every joint's servo tracks a target that moves smoothly to a new random value every one to two
  seconds, now and then staying where it is. Limbs and body touch each other at times (the simulation includes the
  robot's self-collisions).
- **Video.** One camera per recording: 640 x 480 pixels, 30 frames per second, MPEG-4. Frame k is at time k/30 s,
  and row k of the joint angles is the robot's state at that time. Each frame is the mean of two renders 1/60 s apart
  (motion blur), with added noise. The camera is hand-held: it moves slowly round the robot during a recording and
  shakes a little; it films the robot from the front or the sides. Neither its pose nor its intrinsics are given (its
  field of view stays the same during a recording). The lighting, the floor, the sky and the tone of the robot's
  paint differ between recordings; the stand (a post behind the robot and a beam to its pelvis) is in every one.
- **Labelled recordings.** The same kind of motion, camera and scenes as the test recordings. Their `joints.csv`
  is in the deliverable's format.

## Deliverable

For every test recording `Txx`, a file `/home/user/submission/Txx.csv`: the header line with the 29 joint names of
`JOINTS`, comma-separated, then one line per video frame, in frame order, with the 29 joint angles at that frame in
radians. The number of lines after the header must be the recording's `frames` in `recordings.json`.
`python3 tools/score.py --check` checks the format of the files in `submission/`.

## Scoring

The grader holds the true joint angles. For each frame of a test recording it places 21 points of the robot by
forward kinematics, with the pelvis fixed as on the stand, once from your angles and once from the true angles: the
knee, ankle, toe and heel of each leg, the head, and the shoulder, elbow, wrist, hand centre, hand tip and the thumb
side of the hand of each arm (the sites of `tools/g1_keypoints.xml`).

1. A point is correct when it lies within 5.0 cm of its true place. A line with a number that is not finite has
   all its points wrong.
2. The score of a recording is the fraction of correct points over all its frames.

A missing or malformed file scores 0 for its recording. The metric, `keypoint_accuracy_5cm`, is the mean of the 24
recording scores (higher is better; 1.0 means that every point of every frame is correct).
`python3 tools/score.py <your csv> <labelled id>` computes the same recording score for a labelled recording.
The grader reads only the 24 CSV files; nothing you write is executed.

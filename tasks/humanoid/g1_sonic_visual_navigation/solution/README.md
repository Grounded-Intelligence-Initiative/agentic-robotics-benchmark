# Reference solver

The controller walks to the first visually marked stop, waits for the turn light, pivots to face the yellow heading beacon and walks to the final green stop. It receives the same JSONL camera and robot observations as a participant. It uses the released GEAR-SONIC V1 encoder to turn a feedback gait into 64 integer codes every 20 ms. The environment applies the released decoder and advances the robot.

**Vision.** The controller separates colored components in the 128 × 96 RGB image. For each component it unprojects valid optical-axis depths, fits the public 0.12 m beacon sphere, compensates for the camera translation and projects the center onto the horizontal pelvis axes using measured gravity. On a walking leg, the requested stop is 1.50 m before the beacon along the heading at which that leg began.

**Feedback.** Body velocity and yaw rate maintain a local position and heading estimate between camera measurements. The bearing of a visible stationary walking beacon corrects yaw drift. The turn light supplies the initial direction while the yellow marker is outside the field of view. Once visible, its bearing determines the final yaw. A slower approach near the target and a constant standing reference inside the stop region stabilize the extra approach dwell. The new beacon starts the next leg.

**Same-tick observations.** Each action is computed from the estimate associated with its preceding committed observation. Several packets can arrive for a single tick. They each start from the same estimate and the next packet’s `last_observation_id` identifies which result advanced the robot.

In the task environment install the reference and run a public practice course:

```sh
bash solution/solve.sh
python /home/user/practice.py --seed 7 --turn left
```

The periodic leg and arm motion is adapted from the public G1 SONIC command-sequence reference. The optical localization and robot-local state estimation in this solution use only the published calibration, visible lights and measured proprioception.

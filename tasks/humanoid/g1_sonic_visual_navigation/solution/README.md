# Reference solver

The G1 sidesteps through the visible cabinet openings, stops at the blue beacon, pivots toward the yellow heading beacon and walks to the final green stop. Every 20 ms the controller reads the public egocentric RGB-D image and local proprioception. The public GEAR-SONIC V1 encoder converts a ten-pose gait reference into 64 integer codes. The environment applies the released decoder and advances the robot.

**Obstacle detection.** In the 128 × 96 RGB image, the controller finds vertical edges of teal cabinet panels. It checks that the adjacent depth is at least 30 cm farther away or beyond the sensor range. This can recover the second opening through the first even when both cabinets have similar colors. With the published opening width and camera calibration, each visible edge estimates the center of its own opening. The controller tracks those centers in a horizontal odometry frame using body velocity, gravity and yaw rate.

**Passing through an opening.** The gait controls forward stride and sideways stride independently. The pelvis faces down the aisle so that the next cabinet stays visible. As the lateral error grows, the controller reduces forward speed and increases sideways stride. It keeps tracking the same opening until the pelvis is 31 cm beyond its estimated plane, allowing the trailing foot and arm to clear the cabinet before the next lateral maneuver.

**Stopping and turning.** The controller unprojects colored beacon pixels and fits the public 0.12 m sphere to metric depth. The walking stop lies 1.50 m before the beacon along the leg’s entry heading. The turn light supplies the initial direction while the yellow target is outside the field of view. Once visible, its bearing determines the final yaw. For the blue stop, forward and sideways braking preserve the current aisle heading. The controller parks after three consecutive samples satisfy the position, heading, speed and yaw-rate margins, and resumes when the next light appears.

Install the reference in the task environment and run a public practice layout with opposite openings.

```sh
bash solution/solve.sh
python /home/user/practice.py --seed 7 --layout-seed 7 --gaps LR --turn left
```

The periodic leg and arm motion is adapted from the public G1 SONIC command-sequence reference. The solver uses the participant wire and published dimensions. It receives its observations and returns codes over JSONL.

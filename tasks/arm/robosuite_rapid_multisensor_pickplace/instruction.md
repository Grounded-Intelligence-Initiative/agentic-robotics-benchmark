# Rapid visual and tactile pick and place

Write a closed-loop controller for the Panda arm in robosuite 1.5.2's native
`PickPlaceCan` scene. Move the red can from the source bin into its designated
quadrant of the destination bin, release it, and move the gripper clear. The
simulator uses MuJoCo 3.3.7. The task deadline is **115 actions at 20 Hz**
(5.75 seconds of simulated time). A stock-scene reference timing probe completed
20 sampled scenes in 4.6–7.05 seconds, and a camera-based controller completed
9 of 12 fresh-seed scenes within 115 steps. The stock Panda setup did not
support a reliable 3-second deadline. Only simulated steps count toward the
deadline, so machine load does not alter the result.

The public environment wrapper is `/home/user/rapid_sort_env.py`. You can inspect
it and run `python3 /home/user/practice_grader/grade.py` for visible-seed practice.
Each observation provides:

- `external_rgb_png_base64`: 128×128 external camera PNG.
- `wrist_rgb_png_base64`: 128×128 wrist camera PNG.
- `wrist_contact_png_base64`: the wrist image with cyan contact-force highlights
  projected from MuJoCo finger-pad contacts into the same camera frame. This is a
  visualization of simulated contact force, not a separate optical tactile sensor.
- `external_camera`: calibrated origin, right/down/forward-to-world basis, and
  vertical field of view for image-based localization.
- `eef_pos`, `eef_quat`, `joint_pos`, and `gripper_qpos`: robot proprioception.

The goal bin centre is `(0.1, 0.28, 0.8)` metres. Its can quadrant is the
positive-X, positive-Y half; the nominal target centre is `(0.1975, 0.4025)`.
The red can's exact pose and the random seed are unavailable. A can counts only
when robosuite's native placement test is satisfied and the arm is clear for
five consecutive control steps.

Create `/home/user/submission/run.sh`. It is launched as an unprivileged process
from `/home/user/submission`. Print `READY` once, then exchange one JSON object
per line:

```text
{"type":"reset"}                  -> {"ok":true}
{"type":"act","t":0,"obs":{...}} -> {"action":[x,y,z,rx,ry,rz,grip]}
{"type":"close"}                  -> {"ok":true}
```

Actions are seven normalized OSC pose and gripper commands, each clipped to
`[-1, 1]`. The grader tests fresh hidden seeds and scores the fraction of scenes
completed by the deadline. Missing, malformed, or late replies fail an episode.
Reply within 10 seconds per step and print `READY` within 40 seconds. A step
may be queried twice with the same `t`; answer from that query's observation.
Diagnostics belong on stderr.

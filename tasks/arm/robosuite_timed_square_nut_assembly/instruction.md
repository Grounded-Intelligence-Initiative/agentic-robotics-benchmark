# Timed square nut assembly in MuJoCo

Write a closed-loop controller for a Panda arm in robosuite's unmodified
`NutAssemblySquare` scene. Pick up the square nut by its handle, align its hole
with the square peg, lower it onto the peg, and release it before the time limit. Success requires
robosuite's native `_check_success()` to hold for 10 consecutive control steps.

The simulator is robosuite 1.5.2 with MuJoCo 3.3.7. You have 170 actions at 20 Hz.
Each action is a seven-dimensional normalized OSC pose and gripper command vector.
The environment, controller, exact observation filter, and reset scheme are
documented in `/home/user/square_nut_env.py`. You may inspect that file and run
`python3 /home/user/practice_grader/grade.py` for visible seed practice.

Create `/home/user/submission/run.sh`. It is launched as an unprivileged process
with its working directory set to `/home/user/submission`. Print `READY` once,
then exchange one JSON object per line:

```text
{"type":"reset"}                  -> {"ok":true}
{"type":"act","t":0,"obs":{...}} -> {"action":[x,y,z,rx,ry,rz,grip]}
{"type":"close"}                  -> {"ok":true}
```

`obs` contains `eef_pos`, `eef_quat`, `joint_pos`, `gripper_qpos`,
`nut_pos`, `nut_quat`, `handle_pos`, `peg_pos`, and `rgb_png_base64`. Pose coordinates are
world-frame metres. The nut pose has a small fixed per-scene vision offset and
does not expose the exact MuJoCo body state. The image is a 128×128 PNG from
the `agentview` camera. The action is clipped elementwise to [-1,1].
Only the current observation may be used; the hidden seed is never exposed.

The grader scores the fraction of fresh hidden episodes completed in the
170-step horizon. Missing, malformed or late replies fail that episode.
Diagnostics belong on stderr. Reply within 10 seconds per step, and print
`READY` within 40 seconds. A step can be queried twice with the same `t`;
answer each query from its observation. The grader can restart the process
after a failed episode.

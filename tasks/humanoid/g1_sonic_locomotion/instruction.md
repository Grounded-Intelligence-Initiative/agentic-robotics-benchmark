# Walk, turn and stop a Unitree G1 using SONIC tokens

Write a closed-loop controller for a free-standing Unitree G1/Dex3 on level ground. In `straight_stop`, walk the requested distance along the robot’s initial heading and come to rest facing that heading. The public default is **5 metres**. In `right_turn_stop`, turn clockwise by the requested angle, stay near the starting position and come to rest. The public default is **90 degrees right**. The starting world position and yaw vary, and the requested goal is public on every decision.

Submit 64 integer codes per 20 ms of motion. The original GEAR-SONIC V1 decoder turns each code vector and the current history into targets for 29 body motors. MuJoCo advances the floating-base robot with ten 2 ms physics substeps per row. Both hands stay open.

## Run your first episode

Put your program and its dependencies in `/home/user/submission/`. The evaluator launches `bash run.sh` from that directory as an unprivileged user. These commands install the neutral starter and exercise both routes offline.

```bash
cd /home/user
mkdir -p submission
cp starter.py submission/controller.py
cat > submission/run.sh <<'SH'
#!/bin/sh
exec /opt/venv/bin/python3 -u controller.py
SH

/opt/venv/bin/python3 practice.py --submission submission --task straight_stop --seed 7 --steps 25
/opt/venv/bin/python3 practice.py --submission submission --task right_turn_stop --seed 7 --steps 25
```

Replace the starter’s `act(obs)` function with your controller. Omit `--steps` to run an entire episode. Practice also accepts `--distance-m 5` for straight walking and a signed `--turn-deg -90` for a right turn. The JSON report includes the stopping reason, physical measurements, elapsed simulated seconds and the number of FSQ scalars actually applied. The neutral starter sends zeros in normalized token space and exercises the wire.

## What counts as a completed route

Keep the robot upright throughout the course. On the straight route, the pelvis must remain within 0.45 m of the initial heading line and finish within 0.25 m of the waypoint. On the right turn, stay within 0.45 m of the starting pivot, finish within 0.30 m of it, and keep the accumulated leftward turn from the initial heading at or below 15°. Both routes require a final heading error of at most 10°.

Hold the endpoint continuously for 0.75 s with planar speed at or below 0.15 m/s and absolute yaw rate at or below 0.25 rad/s. The host checks motion and contact every 2 ms and records the first terminal event. A fall, path violation or timeout ends the episode. Successful stops across all scheduled starts determine the physical success rate. Same-tick trials also check command following and sensitivity to measured state.

In the practice report, `simulated_s` is time to the terminal event or cutoff. `issued_control_s` counts complete 20 ms rows, including the row containing a terminal event. `scalar_codes` counts 64 FSQ scalars per applied row, and `code_change_fraction` is the fraction of coordinates that changed between adjacent rows. `joint_accel_rms_rad_s2` measures smoothness from all 29 measured joint velocities using differences across 0.10 s. The first eligible sample is at 0.10 s. The report also gives terminal errors, stop dwell, pelvis path length and left/right foot load transitions.

## Solver protocol

Print `READY` followed by a newline and flush within 60 seconds of launch. Read one JSON object per line and flush one response per request within 15 seconds. Use stderr for diagnostics. The process can serve multiple episodes. Clear controller memory at reset and support a fresh process after an abort.

| Host request | Solver reply |
| --- | --- |
| `{"type":"reset"}` | `{"ok":true}` |
| `{"type":"act","t":t,"obs":obs}` | `{"action":{"rows":[{"codes":[...]}]}}` |
| `{"type":"close"}` | `{"ok":true}`, then exit |

Each `codes` array contains exactly 64 integers in `[0,31]`. Code `c` represents the native FSQ value `(c - 16) / 16`. Every accepted row advances one physical tick. A chunk can contain between one row and `min(5 - t % 5, obs["horizon"] - t)` rows. For example, at `t = 3` you may submit one or two rows. The host checks the entire chunk before applying its first row and ends it on the first terminal event. A short practice run can stop partway through a valid chunk at the requested tick.

The host may query different observations at the same `t`, or repeat an identical observation. Answer each using the memory committed *before* that tick. A stateful controller can store candidate next memories under `obs["observation_id"]`. When `t` advances, select the candidate named by `obs["last_observation_id"]`. The initial receipt is `null`. Repeated queries consume zero simulated time and zero applied FSQ scalars.

## Read the goal and robot state

The full goal, command, robot measurements and decoder history refresh on every decision. A one-row reply gives a new measurement at 50 Hz. A five-row reply at a five-tick boundary gives a new measurement after 100 ms, with the intervening rows executed open loop.

| Observation field | Meaning |
| --- | --- |
| `protocol`, `tick`, `horizon` | Protocol `g1-sonic-locomotion/1`, current physical tick (equal to `t`) and maximum physical ticks in the episode. |
| `goal.distance_m`, `goal.heading_delta_rad` | Fixed endpoint displacement along the initial forward direction in metres and signed heading change from reset in radians. |
| `command.route` | `straight_stop` or `right_turn_stop`. |
| `command.target_forward_m`, `command.target_left_m` | Remaining horizontal displacement to the endpoint along the current pelvis heading and its leftward perpendicular, in metres. |
| `command.heading_error_rad` | Desired final heading minus the current continuous pelvis heading. Left is positive. A full rotation changes the error by $2\pi$. |
| `observation.q`, `observation.dq` | 29 measured body joint positions and velocities in Unitree hardware order, in rad and rad/s. |
| `observation.base_pos`, `torso_pos`, `left_foot_pos`, `right_foot_pos` | World-frame positions in metres. All these fields are inside `observation`. |
| `observation.base_quat`, `torso_quat`, `base_gyro`, `gravity_body` | World orientations as `[w,x,y,z]`, body-frame pelvis angular velocity in rad/s, and body-frame unit gravity. |
| `observation.base_velocity_world`, `heading_rad`, `yaw_rate_rad_s` | Measured world linear velocity in m/s, continuous pelvis heading in rad and signed yaw rate in rad/s. |
| `observation.left_foot_contact`, `right_foot_contact` | Boolean sole-contact flags. |
| `decoder_history` | 930 numeric values packed for the next native decoder call. |
| `observation_id`, `last_observation_id` | Opaque identifiers for the current and last applied observations. |

## Query the released encoder

The environment includes the original vision-free G1 reference-motion encoder, decoder and robot assets. A controller can emit codes directly or encode reference joint trajectories. The encoder consumes ten reference frames, five physical ticks apart, in Unitree joint order. Each 6D anchor is the first two columns of the reference pelvis rotation expressed in the current pelvis frame, flattened in row-major order.

The following replacement for the starter’s `act(obs)` encodes a stationary reference and produces one valid action row. Initialize the encoder once, before `READY`.

```python
import numpy as np
from sonic_sim.encoder import ReferenceEncoder

encoder = ReferenceEncoder("/opt/sonic/assets/policy/model_encoder.onnx")

def act(obs):
    q = np.tile(np.asarray(obs["observation"]["q"], dtype=np.float32), (10, 1))
    dq = np.zeros_like(q)
    anchor = np.tile([1., 0., 0., 1., 0., 0.], (10, 1))
    codes = encoder.codes_g1(q, dq, anchor).tolist()
    return {"rows": [{"codes": codes}]}
```

The local simulator can evaluate candidate codes with the same measured history. For offline work, `sonic_loco.LocomotionSession("/opt/sonic/assets", seed)` starts a native episode and `loco_io.LiveEpisode(session, loco_io.practice_route("straight_stop"))` supplies the same observation and action interface used by practice.

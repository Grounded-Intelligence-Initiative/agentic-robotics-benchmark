# Walk, turn on command, then walk again

Write a controller for a free-standing Unitree G1. Walk to the first waypoint and hold still. Once that stop is confirmed, the host reveals a left or right pivot. After the robot turns and settles, the host reveals a shorter forward walk and one final stop. All three commands act on the same robot, with continuous simulation, joint velocities, contacts and SONIC decoder history.

You control the body with 64 integer finite scalar quantization (FSQ) codes at 50 Hz. The released SONIC decoder converts each row into targets for 29 joints. Each row drives ten 2 ms MuJoCo steps. Both Dex3 hands stay open.

## Run a public practice trial

The deliverable is `/home/user/submission/run.sh` and its dependencies under `/home/user/submission/`. The task supplies Python, NumPy, MuJoCo, ONNX Runtime, the G1 model and the original SONIC encoder and decoder. Plan for an offline, CPU-only development environment with four cores, 8 GiB memory and one hour.

```sh
cd /home/user
mkdir -p submission
cp starter.py submission/controller.py
cat > submission/run.sh <<'SH'
#!/bin/sh
exec /opt/venv/bin/python3 -u controller.py
SH
/opt/venv/bin/python3 practice.py --turn left --seed 7 --submission submission --steps 20
```

The starter emits neutral body codes. Replace its `act(obs)` function. Omit `--steps` to run a full trial. Use `--turn right` to check the opposite pivot with the same start. The practice host also accepts `--first-m` (4.5–5.5), `--angle-deg` (75–105, positive magnitude) and `--last-m` (1.0–2.0). It sends each target to your process only when that leg begins. Use `--trace /tmp/course.jsonl.gz` to record the observations selected for execution, proposed rows and the number actually applied.

## Physical criteria

| Stage | Newly revealed target | Required motion |
| --- | --- | --- |
| `approach` | Walk 4.5–5.5 m along the initial heading. | Stay within a 0.45 m lateral corridor. Finish within 0.25 m of the target facing forward. Hold the qualifying stop for 0.75 s, then maintain it for another 1.50 s before the turn is revealed. |
| `pivot` | Turn 75–105 degrees to the left or right. | Stay within 0.45 m of the pivot and finish within 0.30 m. Excursion more than 15 degrees in the wrong direction fails the route. Hold the new heading for 0.75 s. |
| `departure` | Walk another 1.0–2.0 m along the new command heading. | Stay in the 0.45 m corridor, finish within 0.25 m, and hold the final stop for 0.75 s. |

At each qualifying stop the robot must remain upright, within 10 degrees of its requested heading, at planar speed at most 0.15 m/s and absolute yaw rate at most 0.25 rad/s. The stage limits are 1,300, 450 and 650 rows respectively. The episode allows at most 2,400 rows (48 simulated seconds). Falling, violating the path or exhausting a limit ends the attempt.

The raw score is complete three-stage routes divided by 16 fresh trials, eight per turn direction. A completed first or second stage alone earns zero. The host records applied body-code counts, stage timings, final errors and joint acceleration as diagnostics. Grading also checks that your actions respond to both the current robot state and the revealed turn. For each check it subtracts the fraction of changed first code rows on identical repeated observations from the fraction changed on alternatives. The required difference is at least 0.10 for robot-state alternatives and 0.50 for opposite turn commands. The panel needs at least 16 state pairs, eight turn pairs and eight identical repeats for each check. Failure of either check yields zero reward for the panel. The frozen reference completed 15/16 routes. With both feedback checks passed, the final reward is `min(1, completed_courses / 12)`. For example, nine full routes give a raw 9/16 and reward 0.75.

## Process and action format

The host runs `bash run.sh` in the submission directory as an unprivileged user. Print `READY` and flush within 60 seconds. Reply to each JSON line within 15 seconds, using one flushed JSON object on stdout. Direct diagnostics to stderr. Each reply must fit in 65,536 decoded characters including the newline.

| Host message | Required reply |
| --- | --- |
| `{"type":"reset"}` | `{"ok":true}` and clear per-episode memory. |
| `{"type":"act","t":t,"obs":obs}` | `{"action":{"rows":[...]}}` |
| `{"type":"close"}` | `{"ok":true}`, then exit. |

The reset acknowledgment uses the JSON boolean `true`. Each act reply has exactly one key, `action`. Its `rows` array contains one to `min(5 - t % 5, obs["horizon"] - t)` rows. Each row has exactly one key, `codes`, holding 64 integers in `[0,31]`. A bin `c` becomes the native scalar `(c - 16) / 16`. One valid Python reply follows.

```python
{"action": {"rows": [{"codes": [16] * 64}]}}
```

The host validates the full proposed chunk before applying any row. It stops a chunk on a stage transition, terminal event or debugging cutoff. Read the next absolute tick to learn how many rows actually ran. Sending a single row gives fresh feedback after 20 ms.

Several `act` requests can arrive at the same tick. They consume no physical time. Evaluate each from the same pre-tick controller memory. For recurrent policies, save candidate memories by `observation_id` and commit the one named by `last_observation_id` when time advances. A crash, invalid reply or timeout counts as a failed trial.

## Reading the live observation

The protocol string is `g1-sonic-command-sequence/1`. The solver receives the current command, robot measurements and decoder history. Future targets become available only after physically qualified stops.

| Field | Meaning |
| --- | --- |
| `tick`, `horizon`, `stage`, `stage_tick` | Elapsed physical rows since reset, episode row limit, currently revealed stage, and rows elapsed in that stage. |
| `command.route` | `straight_stop` in either walk, or `turn_stop` during the pivot. |
| `command.target_forward_m`, `command.target_left_m` | Remaining planar displacement to the current waypoint in the pelvis heading frame, in metres. |
| `command.heading_error_rad` | Desired heading minus the continuous pelvis heading, in radians. Left is positive. |
| `observation.q`, `dq` | 29 joint angles and velocities in Unitree hardware order, in rad and rad/s. |
| `observation.base_pos`, `torso_pos`, `left_foot_pos`, `right_foot_pos` | World-frame positions in metres. |
| `observation.base_quat`, `torso_quat`, `base_gyro`, `gravity_body` | Unit quaternions `[w,x,y,z]`, pelvis-frame angular velocity in rad/s and unit gravity direction. |
| `observation.base_velocity_world`, `heading_rad`, `yaw_rate_rad_s` | World-frame linear velocity (m/s), continuous heading (rad) and yaw rate (rad/s). |
| `observation.left_foot_contact`, `right_foot_contact` | Boolean foot-contact measurements. |
| `decoder_history` | 930 current non-token inputs for the released body decoder. |
| `observation_id`, `last_observation_id` | Opaque IDs for the current sensor packet and the previous physically selected packet. The latter starts as `null`. |

Within table entries beginning with `observation.`, each listed field belongs to that same dictionary. Use proprioception to estimate progress, begin braking and maintain balance through each reveal. The public reference-motion encoder is available as `sonic_sim.encoder.ReferenceEncoder`. Its `codes_g1` method converts ten reference joint-position, velocity and pelvis-orientation samples into a native 64-bin body row. The body decoder operates locally on CPU.

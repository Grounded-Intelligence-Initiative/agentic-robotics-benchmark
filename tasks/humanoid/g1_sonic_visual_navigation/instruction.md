# Follow the lights on a G1

Walk a Unitree G1 to a blue waypoint, hold still, pivot in the direction shown by a newly illuminated signal, then walk to a green waypoint and stop. All three maneuvers take place in one continuous MuJoCo world, with the same floating-base robot, contacts and SONIC decoder history. Your executable receives a pelvis-mounted RGB-D camera and robot-local sensors. It sends 64 integer finite scalar quantization (FSQ) codes for each 20 ms of motion. The released SONIC V1 decoder drives 29 body joints. The hands stay open.

## Read the scene

Each illuminated sphere has a 0.12 m radius and a center 1.03 m above the floor. Waypoint distance and turn direction vary between episodes. World origin and reset yaw vary independently of the course.

| Visible lights | Required maneuver |
|---|---|
| Blue | Walk along the pelvis heading at the start of this leg. Stop 1.50 m before the sphere center along that axis. Evaluation varies the walk between 3.25 and 5.75 m. |
| Red or magenta at the old forward bearing, plus a yellow heading beacon | Red means pivot left, magenta means pivot right. The yellow sphere is 6 m from the pivot origin on the required heading, 80–100° from the leg-entry heading. It comes into view as the robot turns. Stay near the pivot origin. |
| Green | Walk along the pelvis heading at entry to this last leg. Stop 1.50 m before the sphere. This walk ranges from 0.95 to 2.25 m. |

To qualify, the robot must remain upright, face within 10° of the required heading, travel at most 0.15 m/s horizontally, and rotate at most 0.25 rad/s for 0.75 s continuously. The walking stop disk has radius 0.25 m and the entire walk must stay within a 0.45 m lateral corridor. Maintain the first qualifying stop another 1.50 s for the turn signal. Signals change at the first five-tick boundary after the physical hold, adding up to four settled rows. The pivot stop disk has radius 0.30 m, with a 0.45 m maximum excursion and at most 15° of travel in the wrong direction. A fall, path violation, interrupted additional hold or timeout ends the course.

## Public practice

Put your runnable controller in `/home/user/submission/run.sh`, with dependencies in the same submission directory. Development is CPU-only and offline, with four CPUs, 8 GiB memory and one hour. The environment includes the public SONIC encoder/decoder, robot assets and Python libraries. To inspect the camera and exercise the starter:

```sh
cd /home/user
python practice.py --turn left --seed 7 --inspect /tmp/nav-view
mkdir -p submission
cp starter.py submission/controller.py
cat > submission/run.sh <<'SH'
#!/bin/sh
cd "$(dirname "$0")"
exec /opt/venv/bin/python3 -u controller.py
SH
chmod +x submission/run.sh
python practice.py --turn left --seed 7 --steps 20 --submission submission
```

The inspect directory contains `rgb.png`, `depth-mm.png` and `calibration.json`. Omit `--steps` for a full course. Change `--turn` to `right` for the opposite signal. Other public host flags are `--first-m` (3–6), `--angle-deg` (75–105) and `--last-m` (0.8–2.5). The host prints the physical outcome, per-leg endpoint errors, dwell times, native row count, code changes and joint acceleration. `--trace FILE.jsonl.gz` saves selected observations and proposed and applied row counts. Practice command-line arguments are private to the simulation process during evaluation.

## Controller process

Print `READY` and flush. Read one JSON object per line and flush one reply per request. A reset or close request takes `{"ok":true}`. Act requests are `{"type":"act","t":t,"obs":{...}}`. Return only `{"action":{"rows":[{"codes":[...]}]}}`. Each row is an array of exactly 64 JSON integers in [0,31]. Code 16 represents zero. A normalized body token coordinate is `(code-16)/16`. Supply one through `min(5-t%5,horizon-t)` rows. Every applied row runs the frozen decoder once and advances ten 2 ms native physics steps. A valid chunk continues across a light change until it ends or the course terminates.

The public observation protocol is `g1-sonic-visual-navigation/1`. It contains:

- `tick` and `horizon` (2,400 native rows). The leg time limits are 1,300, 450 and 650 rows.
- `camera.rgb_png`, a base64-encoded 128 × 96 RGB PNG, and `camera.depth_mm_png`, a colocated 16-bit PNG. Depth measures optical-axis distance in millimetres. Zero marks invalid or more than 12 m. Pixel center is (63.5,47.5) with square pixels and a 75° vertical field of view. Camera origin in the pelvis frame is (0.28,0,0.33) m, looking along +x with image right along −y and up along +z. RGB and depth are rendered at the decision tick.
- `observation.q` and `dq`, 29 joint angles and velocities in G1 hardware order. `base_gyro`, `gravity_body` and `velocity_body` are angular velocity, projected unit gravity and ideal linear velocity expressed in the pelvis frame. `yaw_rate_rad_s` is the measured vertical heading rate. Boolean `left_foot_contact` and `right_foot_contact` indicate more than 1 N of floor load. These are ideal simulator sensors.
- `decoder_history`, the released decoder's public flat history of local measurements and preceding motor outputs.
- `observation_id`, a content hash of the current public packet, and `last_observation_id`, the preceding packet selected for application (null on reset). On every five-tick boundary the verifier sends four requests in randomized order, including a scene intervention, relighting and an exact repeat. Other decisions have one live query. Public practice sends one live query per decision. A stateful controller should commit its preceding decision using that selected ID.

The grader samples 16 fresh courses, balanced across turn directions. It counts complete physical courses on all assigned starts. It also measures whether the first native code row responds more often to a changed signed signal and displaced waypoint than to relighting the same beacon or repeating the same packet at the same pose. A response requires at least two changed coordinates. Full credit requires at least 15 of 16 strictly completed courses and both visual checks. With valid checks, partial credit is the completion count divided by 15, capped at one. Timing, smoothness and code use are diagnostics.

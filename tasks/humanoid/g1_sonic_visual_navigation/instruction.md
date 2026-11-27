# Navigate a G1 through offset openings

Drive a free-standing Unitree G1 through the openings in a row of storage barriers. Stop at a blue waypoint, turn in the direction signalled by a light, then walk to a green waypoint and stop. A route only counts when the robot clears every opening in order, avoids contact and completes all three stops. The same robot, physical world and SONIC V1 decoder history continue across all three maneuvers.

Your executable receives a pelvis-mounted RGB-D camera and robot-local sensors. It sends 64 integer finite scalar quantization (FSQ) codes for each 20 ms of motion. The released SONIC V1 decoder drives 29 body joints. The hands stay open.

## Read the scene

| What the camera shows | What the robot must do |
|---|---|
| Storage barriers and a blue waypoint | Find each opening and cross it in order. Openings can be on either side of the initial heading. After the last barrier, return to the route centerline and stop 1.50 m before the blue sphere. Face the heading at the start of the route. |
| A red or magenta signal and a yellow heading beacon | Pivot in place. Red means left and magenta means right. The yellow sphere is 6 m from the pivot origin on the required heading and comes into view during the turn. |
| A green waypoint | Walk along the new heading and come to rest 1.50 m before the green sphere. |

Each 0.72 m high barrier has a real opening, 0.95–1.08 m wide, between two cabinet sections. The center of an opening lies 0.57–0.76 m to either side of the route centerline. Cabinet color can vary; the traversable space is visible in both RGB and depth. The first leg contains one or two barriers. With two barriers, the second opening may stay on the same side or switch sides. The first walk spans 5.5–7.25 m. During the approach, keep the pelvis within 1.58 m of the initial centerline. During the final walk, stay within 0.45 m of the new heading line. Walk past the center plane of each opening in order. Turn magnitude (80–100°), final distance (0.95–2.25 m), gate placement and robot reset pose vary across courses. The colored waypoint spheres have 0.12 m radius and sit 1.03 m above the floor. The camera provides geometry through aligned depth in millimetres.

The left and right cabinet sections are each 2.4 m wide and 0.18 m deep. Measure the free passage from depth and leave room for the robot’s arms and legs. A stop requires an upright posture, heading error at most 10°, horizontal speed at most 0.15 m/s and yaw rate at most 0.25 rad/s for 0.75 s continuously. The walking stop disk has radius 0.25 m. Hold the first stop for another 1.50 s to reveal the turn. The pivot stop disk has radius 0.30 m with a maximum excursion of 0.45 m and at most 15° of travel in the wrong direction. A fall, collision, leaving the course, losing the additional standing hold or timing out ends the trial.

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

The inspect directory contains `rgb.png`, `depth-mm.png` and `calibration.json`. To compare different physical passages on the same robot reset, run:

```sh
python practice.py --seed 7 --layout-seed 7 --gaps LR --turn left --submission submission
python practice.py --seed 7 --layout-seed 7 --gaps RL --turn left --submission submission
```

`L` and `R` mean left and right as seen from the initial robot heading. The six practice patterns are `L`, `R`, `LL`, `LR`, `RL` and `RR`. The layout seed varies the physical furniture separately from the robot seed. Omitting `--gaps` lets the layout generator choose the pattern. Run `python practice.py --help` for turn, distance, step-limit and trace options. The practice host reports the raw physical outcome and motion costs. Evaluation supplies each participant process with sensor packets and accepts actions over standard input and output.

## Controller process

Print `READY` and flush. Read one JSON object per line and flush one reply per request. A reset or close request takes `{"ok":true}`. Act requests are `{"type":"act","t":t,"obs":{...}}`. Return only `{"action":{"rows":[{"codes":[...]}]}}`. Each row is an array of exactly 64 JSON integers in [0,31]. Code 16 represents zero. A normalized body token coordinate is `(code-16)/16`. Supply one through `min(5-t%5,horizon-t)` rows. Every applied row runs the frozen decoder once and advances ten 2 ms native physics steps. A valid chunk continues across a light change until it ends or the course terminates.

The public observation protocol is `g1-sonic-visual-navigation/1`. It contains:

- `tick` and `horizon` (3,100 native rows). The approach, pivot and departure time limits are 2,000, 450 and 650 rows.
- `camera.rgb_png`, a base64-encoded 128 × 96 RGB PNG, and `camera.depth_mm_png`, a colocated 16-bit PNG. Depth measures optical-axis distance in millimetres. Zero marks invalid or more than 12 m. Pixel center is (63.5,47.5) with square pixels and a 75° vertical field of view. Camera origin in the pelvis frame is (0.28,0,0.33) m, looking along +x with image right along −y and up along +z. RGB and depth are rendered at the decision tick.
- `observation.q` and `dq`, 29 joint angles and velocities in G1 hardware order. `base_gyro`, `gravity_body` and `velocity_body` are angular velocity, projected unit gravity and ideal linear velocity expressed in the pelvis frame. `yaw_rate_rad_s` is the measured vertical heading rate. Boolean `left_foot_contact` and `right_foot_contact` indicate more than 1 N of floor load. These are ideal simulator sensors.
- `decoder_history`, the released decoder's public flat history of local measurements and preceding motor outputs.
- `observation_id`, a content hash of the current public packet, and `last_observation_id`, the preceding packet selected for application (null on reset). Each decision receives one live observation.

## Scoring

The primary metric is the fraction of assigned courses completed with zero obstacle contacts, all required passages and all three physical stops. Every part of the robot counts for obstacle contact, including feet and arms. The verifier checks native MuJoCo contacts at 2 ms intervals between 50 Hz commands. The terminal report records passages, contacts and stage completion so a reviewer can distinguish a missed opening from a failed stop. It also reports simulated time, applied FSQ rows and scalar codes, changes between successive rows and joint acceleration RMS. Each 16-course evaluation includes four single-gate courses, six with two same-side openings and six with opposite openings. Both initial gap sides and both pivot directions are balanced.

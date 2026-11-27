# Navigate a G1 through moving gates and a low service rack

Drive a free-standing Unitree G1 through a warehouse junction with two motorized sliding gates. Use the camera to find a passable aisle and choose when to cross. Some routes also have a stationary cabinet barrier. After the blue stop, pivot on a light signal, find the offset passage beneath the service rack, and walk through to the green inspection station. Finish standing upright at the station. A route counts only when the entire robot avoids contact and completes all three stops. The same robot, physical world and SONIC V1 decoder history continue across all three maneuvers.

Your executable receives a pelvis-mounted RGB-D camera and robot-local sensors. It sends 64 integer finite scalar quantization (FSQ) codes for each 20 ms of motion. The released SONIC V1 decoder drives 29 body joints. The hands stay open.

## Read the scene

| What the camera shows | What the robot must do |
|---|---|
| Two sliding gates, then a blue waypoint | Cross the junction through either passable opening. Leave enough time for both feet and arms to clear before the gate closes. If there is another cabinet barrier, cross its opening next. Return to the route centerline and stop 1.50 m before the blue sphere, facing the initial heading. |
| A red or magenta signal and a yellow heading beacon | Pivot in place. Red means left and magenta means right. The yellow sphere is 6 m from the pivot origin on the required heading and comes into view during the turn. |
| A green waypoint beyond a roofed rack | Enter the offset opening in the indicated aisle, keep the whole robot clear of the roof and side cabinets, and stand at the inspection station 1.50 m before the green sphere. |

The two amber shutters slide sideways on independent schedules. The motor commands a working shutter to open over 1.0 s, holds the fully retracted position for 4.2 s, closes over 1.0 s, then holds closed for 5.8 s. The moving plate can lag the command during transitions. The 12 s cycle repeats. Each shutter has its own starting phase. The central island is 0.44 m wide, and each side offers 0.96–1.06 m when fully open. The junction is 2.20–2.55 m from the start. At least one side opens fully every cycle. On some courses, one side has a mechanical stop that leaves too little room to pass even at maximum travel. Watch the actual free space through time. You may wait, change sides or cross either physically clear opening. Contact with a jamb, the central island, a stationary cabinet or a moving shutter ends the course, including contact after the pelvis has crossed the gate plane.

Walk through the gate planes in order. The first leg is 5.5–7.25 m long. Keep the pelvis within 1.58 m of its initial centerline. If a stationary cabinet follows the shutters, its opening may lie on either side. The outgoing aisles are perpendicular to the initial approach. After completing the signalled 80–100° pivot, align to the selected aisle. Both aisles have real racks and marked station areas from reset. The outgoing walk ends 3.8–4.3 m from the junction, on the aisle centerline. Each waypoint sphere has radius 0.12 m and sits 1.03 m above the floor. Depth is aligned with the RGB image and measured in millimetres along the optical axis.

The outgoing service rack has a golden roof and two cabinets flanking an offset opening. Use RGB-D to locate the opening, the roof underside and its back edge. The rack begins 1.72–1.95 m along the outgoing aisle and its roof extends 0.64–0.80 m. The opening is 0.96–1.06 m wide, and its center is offset by 0.58–0.72 m to either side of the outgoing aisle. Keep the pelvis within 1.58 m of the aisle centerline. Low roofs have 1.205–1.225 m of clearance; high roofs have 1.36–1.42 m. Use the measured space to choose a body posture and leave room for walking motion. Allow for the head, arms and trailing foot as you pass beneath the entire roof, then return to the aisle centerline. At the station, stand facing outward and keep the top of the physical head at least 1.25 m above the floor throughout the final 0.75 s stop. Any valid SONIC token sequence can earn full credit.

A stop requires an upright posture, heading error at most 10°, horizontal speed at most 0.15 m/s and yaw rate at most 0.25 rad/s for 0.75 s continuously. The walking stop disk has radius 0.25 m. Hold the first stop for another 1.50 s to reveal the turn. The pivot stop disk has radius 0.30 m with a maximum excursion of 0.45 m and at most 15° of travel in the wrong direction. A fall, collision, leaving the course, losing the additional standing hold or timing out ends the trial.

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

The inspect directory contains `rgb.png`, `depth-mm.png` and `calibration.json`. To test whether your controller responds to the moving gates, run the same robot and physical layout twice with different shutter phases:

```sh
python practice.py --seed 7 --layout-seed 7 --service both --tail right --right-phase-s 6.3 --left-phase-s 11.8 --turn left --submission submission
python practice.py --seed 7 --layout-seed 7 --service both --tail right --right-phase-s 11.8 --left-phase-s 6.3 --turn left --submission submission
```

Use `--service left` or `--service right` to give only that side its full stroke. The restricted plate moves 0.06–0.12 m. With `--service both`, both shutters have a full stroke. `--tail none|left|right` selects whether there is a later stationary opening and where it sits. `--layout-seed` chooses the furniture geometry. `--phase-seed` independently resamples the motion schedules. To reproduce exact phases, pass both `--right-phase-s` and `--left-phase-s` in [0,12). Run `python practice.py --help` for distance, turn, tracing and step-limit options. The live controller receives only sensor packets. The practice host reports the physical outcome and motion costs.

To compare overhead clearance with the same robot reset and lateral opening, change only the roof in the aisle selected by the left turn:

```sh
python practice.py --seed 7 --layout-seed 7 --rack-seed 23 --turn left --left-rack low-left --right-rack high-right --last-m 4.0 --submission submission
python practice.py --seed 7 --layout-seed 7 --rack-seed 23 --turn left --left-rack high-left --right-rack high-right --last-m 4.0 --submission submission
```

In `--left-rack low-left`, the option selects the **left-turn aisle** and the value selects a **low roof with an opening on the robot’s left when facing down that aisle**. Both aisles already exist when the robot starts. The public controller must identify the correct aisle from the signalled turn.

## Controller process

Print `READY` and flush within 60 seconds. Read one JSON object per line and flush each reset or act reply within 15 seconds. The verifier also has a **3,540-second wall-clock budget for all 16 courses combined**, including simulator and process overhead. Run the full practice course and time your controller on the four-CPU configuration. Most of the per-reply allowance is available for occasional slow calls. If the whole-panel budget expires, completed successes retain their credit out of the 16 assigned courses.

You may return up to five consecutive rows per decision. A reset or close request takes `{"ok":true}`. Act requests are `{"type":"act","t":t,"obs":{...}}`. Return only `{"action":{"rows":[{"codes":[...]}]}}`. Each row is an array of exactly 64 JSON integers in [0,31]. Code 16 represents zero. A normalized body token coordinate is `(code-16)/16`. Supply one through `min(5-t%5,horizon-t)` rows. Every applied row runs the frozen decoder once and advances ten 2 ms native physics steps. A valid chunk continues across a light change until it ends or the course terminates.

The public observation protocol is `g1-sonic-visual-navigation/1`. It contains:

- `tick` and `horizon` (4,350 native rows). The approach, pivot and departure time limits are 2,600, 450 and 1,300 rows (52, 9 and 26 simulated seconds).
- `camera.rgb_png`, a base64-encoded 128 × 96 RGB PNG, and `camera.depth_mm_png`, a colocated 16-bit PNG. Depth measures optical-axis distance in millimetres. Zero marks invalid or more than 12 m. Pixel center is (63.5,47.5) with square pixels and a 75° vertical field of view. Camera origin in the pelvis frame is (0.28,0,0.33) m, looking along +x with image right along −y and up along +z. RGB and depth are rendered at the decision tick.
- `observation.q` and `dq`, 29 joint angles and velocities in G1 hardware order. `base_gyro`, `gravity_body` and `velocity_body` are angular velocity, projected unit gravity and ideal linear velocity expressed in the pelvis frame. `yaw_rate_rad_s` is the measured vertical heading rate. Boolean `left_foot_contact` and `right_foot_contact` indicate more than 1 N of floor load. These are ideal simulator sensors.
- `decoder_history`, the released decoder's public flat history of local measurements and preceding motor outputs.
- `observation_id`, a content hash of the current public packet, and `last_observation_id`, the preceding packet selected for application (null on reset). Each decision receives one live observation.

## Scoring

The primary metric is the fraction of assigned courses completed with zero obstacle contacts, a passage through an actual clear opening at each barrier, the whole body beyond the service rack’s rear edge and all three physical stops. Every part of the robot counts for obstacle contact, including feet and arms. The verifier checks native MuJoCo contacts at 2 ms intervals between 50 Hz commands. The terminal report records passages, contacts and stage completion so a reviewer can distinguish a missed opening from a failed stop. It also reports simulated time, applied FSQ rows and scalar codes, changes between successive rows and joint acceleration RMS. Every 16-course evaluation includes eight courses with both shutters in full service, four with only the left in full service and four with only the right in full service. Eight courses have an additional stationary cabinet opening, four on each side. The pivot directions and the outgoing opening sides are balanced. Twelve service racks have a low roof and four have a high roof. A safe crossing by either side of the first junction counts. The final rack requires both an entrance and an exit before the standing inspection stop.

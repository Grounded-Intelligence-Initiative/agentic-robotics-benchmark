# Calibrate a robot's camera, LiDAR and IMU from its driving logs

A wheeled warehouse robot (a Clearpath Husky with a sensor arch) carries a spinning LiDAR, an RGB camera and an IMU. After transport and a repair, the camera and IMU brackets were refitted. They now sit a few degrees and centimetres away from where the CAD model puts them. The camera and the LiDAR each run on their own clock, offset from the robot's clock and drifting against it. Nobody can build a calibration target room for this robot. What you do have are ordinary driving logs: several sequences per robot, recorded in the warehouse it works in. A few stretches of these logs are corrupted by sensor or driver faults.

Write a **reusable calibration program**. Given one robot's logs, it must recover the camera-LiDAR and IMU-LiDAR extrinsics, both sensor clocks and the IMU biases, and it must report the faulty stretches. After you finish, your program is run on six robots it has never seen: other mounts, clocks, biases, lenses, floor layouts, drives and faults. Its output is checked against their true parameters and against a held-out drive of each robot that your program never sees.

You have 2 hours (see "Session time").

## What you submit

A program in `/home/user/submission/` that runs as

```
bash run.sh --input <rig_dir> --output <out_dir>
```

and writes two files into `<out_dir>` (which exists and is empty): `calibration.yaml` and `fault_report.json`, described below. Put everything the program needs beside `run.sh`. Nothing outside `/home/user/submission/` is kept, and there is no submit step: whatever is there when your session ends is graded.

## The rig folder (input)

Three practice rigs in exactly the format of the hidden ones are in `/opt/rigcal/practice/rig_a`, `rig_b` and `rig_c` (read-only, about 110 MB each).

```
<rig_dir>/
├── manifest.json        sensors, camera intrinsics and distortion, LiDAR beam angles, T_body_lidar, the sequence names
├── rough_prior.yaml     CAD values of T_lidar_camera and T_lidar_imu, with their error bounds
└── seq_00/, seq_01/, seq_02/
    ├── camera/timestamps.csv    frame,stamp_s       (frames/000000.jpg ... : 720x540 RGB JPEG, 10 Hz on the camera's clock)
    ├── camera/frames/*.jpg
    ├── lidar/timestamps.csv     scan,stamp_s        (scans/000000.npz ... : 10 Hz on the LiDAR's clock)
    ├── lidar/scans/*.npz        range_mm (32x1024 uint16, 0 = no return), reflectivity (32x1024 uint8)
    ├── imu.csv                  stamp_s,gx,gy,gz,ax,ay,az    (200 Hz, rad/s and m/s^2)
    └── wheel_odom.csv           stamp_s,v_mps,yaw_rate_rps   (50 Hz)
```

- **Sequences.** Each is 24 to 28 s long. Each starts and ends with the robot standing still for about 2 to 3 s, and some also stop in the middle. They are one to two minutes apart within one recording session of about ten minutes. `manifest.json` lists them with their approximate start times. The robot drives on a flat floor; its suspension rolls and pitches the body by a few degrees when it moves, and the body vibrates while it drives.
- **The site.** A warehouse hall (60 x 40 m) with painted floor lanes, doors, poles and partitions, and goods standing around the robot's working area: crates, cartons, pallets, tool cabinets, carts, racks, trucks. All rigs work in this hall; the working area and what stands in it differ from rig to rig.
- **LiDAR** (an Ouster OS1-32). Ring `i` points at elevation `manifest.lidar.elevation_deg[i]` (32 rings over +-22.5 degrees). Column `j` points at azimuth `360*j/1024` degrees, counter-clockwise from +x. A return is `range * (cos(el) cos(az), cos(el) sin(az), sin(el))` in the LiDAR frame **at the moment the column was taken**: a sweep takes 0.1 s, its stamp is the time of column 0, and column `j` is taken `j/1024 * 0.1` s later (on the LiDAR's clock). The driver does **not** motion-compensate, so a sweep taken while the robot moves is skewed. Ranges have 1 cm of noise up to 20 m, rising to 3 cm at 60 m. Returns nearer than 0.5 m are dropped (the sensor arch's legs blank two sectors), as are weak, grazing and a few random returns; at depth edges some returns lie between the two surfaces. Reflectivity is the surface material's near-infrared reflectance, weaker at grazing incidence. It is a property of the material, not of its visible colour or print.
- **Camera** (a FLIR Blackfly S, global shutter, 2x2 binned, 4 mm lens). Pinhole with the `K` in the manifest and Brown-Conrady distortion `[k1, k2, p1, p2, k3]` (OpenCV's order), both exact. A frame's stamp is the middle of its 2 ms exposure. Frames carry vignetting, sensor noise and JPEG compression. The robot itself is never in view.
- **IMU** (a Bosch BMI088). The gyro measures angular velocity plus a bias plus noise (about 0.0035 rad/s per sample). The accelerometer measures specific force plus a bias plus noise (about 0.024 m/s^2 per sample); at rest it reads +9.80665 m/s^2 along "up". Both are in the IMU's own frame. Each axis has a scale error of up to 0.1 % and the axes are coupled by up to 0.1 %; the biases wander slowly (far less than the tolerances below over a session). The IMU and the wheel odometry are stamped by the robot's reference clock.
- **Wheel odometry.** The body's forward speed and yaw rate from the wheel encoders. The speed has up to 3 % of scale error and reads a few percent high while the wheels slip (accelerating, turning). The robot is skid-steered: the yaw rate reads only 0.80 to 1.00 of the true rate, a factor that is constant for a rig.

## Frames and conventions

- World: z up, gravity (0, 0, -9.80665) m/s^2. Body: x forward, y left, z up, origin on the floor. LiDAR: x forward, y left, z up. `T_body_lidar` in the manifest is exact.
- Camera: the OpenCV optical frame (x right, y down, z forward).
- `T_a_b` is the 4x4 pose of frame `b` in frame `a`: `p_a = R_a_b p_b + t_a_b`, with the last row `[0, 0, 0, 1]`. Metres and radians throughout.
- **Clocks.** At reference time `t` (seconds since the start of the session, the clock in `imu.csv`) a sensor's clock with offset `o` (s) and drift `d` (ppm) reads `s = t + o + d * 1e-6 * t`. The camera and the LiDAR each have their own `o` and `d`, and each runs free on its own clock: its frames are 0.1 s apart **on that clock** (plus a trigger jitter of 0.3 ms for the camera and 0.05 ms for the LiDAR, and 0.2 ms of stamping jitter), whatever the drift. A sensor's log of a sequence starts a few tenths of a second to a second after the sequence starts and ends as long before it ends, differently for each sensor and sequence. On the hidden rigs, `|o|` is at most 0.1 s for the camera and 0.06 s for the LiDAR, and `|d|` is at most 200 ppm for the camera and 100 ppm for the LiDAR.
- `rough_prior.yaml` is off from the truth by up to 10 degrees and 8 cm for `T_lidar_camera`, and up to 5 degrees and 4 cm for `T_lidar_imu`. Gyro biases are up to 0.015 rad/s per axis and accelerometer biases up to 0.15 m/s^2 per axis.

## calibration.yaml (output)

```yaml
T_lidar_camera: [[r00, r01, r02, tx], [r10, r11, r12, ty], [r20, r21, r22, tz], [0, 0, 0, 1]]   # camera in the LiDAR frame
T_lidar_imu:    [[...], [...], [...], [0, 0, 0, 1]]                                             # IMU in the LiDAR frame
time:
  camera: {offset_s: 0.0, drift_ppm: 0.0}
  lidar:  {offset_s: 0.0, drift_ppm: 0.0}
imu:
  gyro_bias_rps:   [bx, by, bz]     # in the IMU frame; measured = true + bias (the mean bias over the given sequences)
  accel_bias_mps2: [bx, by, bz]
```

Every key is required. Each rotation block must be orthonormal with determinant +1, to within 1e-3; it is re-orthonormalised before use. `~/tools/prior_baseline.py` writes a valid example from the rough prior.

## fault_report.json (output)

```json
{"faults": [{"sequence": "seq_01", "sensor": "camera", "type": "camera_frozen", "start_s": 123.45, "end_s": 125.10}]}
```

Each rig's logs contain one to three fault intervals, at most one of each type. Each lasts 1.5 to 3.5 s while the robot is moving, and two can fall in the same sequence. The five types and their sensors:

| type | sensor | what happens |
|---|---|---|
| `camera_frozen` | camera | the driver re-sends the last good image (byte-identical files) with fresh stamps |
| `camera_time_jump` | camera | the frames' stamps are shifted by a constant 35 to 80 ms (either sign) |
| `lidar_degraded` | lidar | a large share of returns are replaced by spurious short, dim returns |
| `imu_axis_swap` | imu | two axes of both the gyro and the accelerometer are exchanged |
| `odom_glitch` | wheel_odom | the forward speed reads about zero or jumps by a large offset |

`start_s` and `end_s` are stamps from the affected sensor's own log: the camera's or LiDAR's `timestamps.csv`, or `stamp_s` in `imu.csv` or `wheel_odom.csv`. They are the first and last affected samples. At most 50 entries.

## How it is graded

The six hidden rigs are a fixed set. Your program is run once per rig, **all six at the same time** on an 8-CPU machine with 32 GB of memory. Each run has the working directory `/home/user/submission/`, runs as your user at the lowest scheduling priority, and has **no network** (AF_UNIX sockets only). Its limits:

- **900 seconds of its own processor time**: user plus system time of all its processes together. Waiting, and whatever else the machine is doing, does not count. A run that uses several cores uses its allowance that much faster.
- a wall-clock stop at 3600 s, which a run inside its processor-time allowance does not reach on a machine that is not overloaded;
- a **6144 MB address-space limit** per process.

A run that reaches a limit is killed, and the rig is scored on whatever files exist by then. NumPy, SciPy and OpenCV are pinned to one thread and to one set of kernels (`OMP_NUM_THREADS=1`, `OPENBLAS_CORETYPE=Haswell` and related settings, defaults of the image's Python; do not change them), so plan on one core per run, and a program computes the same on every machine.

Each rig earns a credit in [0, 1]:

```
credit = 0.10 * faults + 0.90 * (c_projection * c_motion * c_time) ^ (1/3)
```

The three factors are measured on the held-out drive, and each needs several parameters right together; their geometric mean is zero when one of them is zero, so a part left at the prior is not made up for by the others. Each error `e` becomes a credit `c(e) = clamp(log(zero/e) / log(zero/full), 0, 1)`: 1 at or below `full`, 0 at or above `zero`, log-linear in between.

| factor | measured as | full | zero |
|---|---|---|---|
| `c_projection` | `projection_px`: median pixel distance (undistorted pixels of `K`) between where held-out LiDAR points land in held-out camera frames with your `T_lidar_camera` and both clocks, and where they really appear | 3 px | 8 px |
| `c_motion` | mean of `c(rpe_rot_deg)` and `c(rpe_trans_m)`: the mean error of the LiDAR's motion over 1 s windows of the held-out drive, predicted by integrating the held-out IMU samples with your `T_lidar_imu` and biases | 0.2 deg, 0.03 m | 0.5 deg, 0.25 m |
| `c_time` | `time_rms_s`: RMS error of the reference times your clock models give for the held-out camera and LiDAR stamps (the worse of the two sensors) | 3 ms | 30 ms |
| `faults` | F1 of your reported intervals; a report matches a true fault with the same sequence and sensor and interval IoU >= 0.3 (each interval padded by 0.05 s), counting 1 with the right type and 0.5 with a wrong one | | |

More precisely:

- **Projection.** For a held-out camera frame and LiDAR sweep, each LiDAR point is placed in the world with the LiDAR's true pose at the reference time your LiDAR clock gives for its column (the sweep's stamp plus the column's delay). It is then projected into a camera whose pose is the LiDAR's true pose at the time your camera clock gives for the frame's stamp, composed with your `T_lidar_camera`. The result is compared with the projection under the true times and extrinsic. Points behind your camera count as 1000 px.
- **Motion.** Each window starts from the true LiDAR pose and velocity. Its IMU pose is the LiDAR pose composed with your `T_lidar_imu`. It integrates your bias-corrected gyro and accelerometer samples at 200 Hz, then maps the result back to a LiDAR pose through your `T_lidar_imu`. (The IMU's small scale errors are part of the data; the true parameters leave about 0.05 deg and 0.01 m.)
- **Held-out drive.** It is recorded later in the same session than the sequences you are given, so clock drift matters.
- **Missing or malformed files.** A missing or unparsable `calibration.yaml` earns 0 on the three factors for that rig. A missing or malformed `fault_report.json` earns 0 on the fault term.

The graded number is the mean credit over the six rigs. The true parameters score 1.0 on every rig: each full point is several times what the true parameters leave on the held-out drive.

## Tools

- `python3 ~/practice_grader/grade.py` runs your `run.sh` on the three practice rigs the way grading does: all at once, with the same limits, working directory `/home/user/submission/`. It then prints every raw metric above for each rig, computed with the same measurement code against the practice rigs' truth and their own held-out drives, and each run's processor and wall time. Use `--rigs rig_b` to run one rig, and `--no-run --outputs DIR` to measure existing outputs. Free and unlimited.
- `/opt/rigcal/practice_truth/<rig>/` holds each practice rig's true parameters (`ground_truth.yaml`, in the output format), its faults (`faults.json`) and its held-out data (`heldout.npz`). A calibration engineer has reference rigs; the hidden rigs are others, and every parameter of theirs is drawn anew.
- `~/tools/rig_io.py` has readers for the format, `~/tools/overlay.py` draws a sweep over a camera frame for a given calibration, and `~/tools/prior_baseline.py` writes the output format from the prior.
- `bash ~/session_time.sh` prints the minutes of your session used and left.

## Environment

Python 3.12 (`python3`, `/opt/venv`) with numpy 2.5.4, scipy 1.18.1, opencv-python-headless 5.0.0.93, PyYAML 6.0.2 and matplotlib 3.11.2. Your program is graded on exactly this interpreter and these packages, and nothing else can be installed. Your sandbox has 8 CPUs, 16 GB of memory, no GPU and no network.

## Session time

Your session is stopped 2 hours after it started, and what is in `/home/user/submission/` at that moment is graded.

- **How to read the time.** `bash /home/user/session_time.sh` prints the minutes used and the minutes left, from a time stamp written when your sandbox was set up (a few minutes before your session started, so you have slightly more than it says, never less). Run it whenever you plan your next step; do not estimate the time from memory or from how much work you have done.
- **Use the time.** Stopping early earns nothing: the score is the program's, not the session's. Keep a working `run.sh` in the submission folder from early on and replace it as you improve it, so that the stop never finds the folder empty or half-edited.
- **Budget your runs.** One pass of the practice grader over three rigs takes as long as your program's slowest run; a program near its processor-time limit takes a quarter of an hour per pass.

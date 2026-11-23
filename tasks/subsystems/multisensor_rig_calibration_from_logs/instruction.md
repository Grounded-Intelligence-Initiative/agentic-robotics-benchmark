# Calibrate a robot's camera, LiDAR and IMU from its driving logs

A wheeled indoor robot carries a spinning LiDAR, an RGB camera and an IMU. After transport and a repair, the camera and IMU brackets were refitted. They now sit a few degrees and centimetres away from where the CAD model puts them. The camera and LiDAR clocks are also offset from the robot's own clock, and they drift. Nobody can build a calibration target room for this robot. What you do have are ordinary driving logs: several sequences per robot, recorded in whatever building it worked in. A few stretches of these logs are corrupted by sensor or driver faults.

Write a **reusable calibration program**. Given one robot's logs, it must recover the camera-LiDAR and IMU-LiDAR extrinsics, both sensor clocks and the IMU biases, and it must report the faulty stretches. After you finish, your program is run on six robots it has never seen: different buildings, mounts, clocks, biases, trajectories and faults. Its output is checked against their true parameters and against a held-out sequence of each robot that your program never sees.

## What you submit

A program in `/home/user/submission/` that runs as

```
bash run.sh --input <rig_dir> --output <out_dir>
```

and writes two files into `<out_dir>` (which exists and is empty): `calibration.yaml` and `fault_report.json`, described below. Put everything the program needs beside `run.sh`. Nothing outside `/home/user/submission/` is kept, and there is no submit step: whatever is there when you stop is graded.

## The rig folder (input)

Three practice rigs in exactly the format of the hidden ones are in `/opt/rigcal/practice/rig_a`, `rig_b` and `rig_c` (read-only).

```
<rig_dir>/
├── manifest.json        sensors, camera intrinsics K, LiDAR beam angles, T_body_lidar, the sequence names
├── rough_prior.yaml     CAD values of T_lidar_camera and T_lidar_imu, with their error bounds
└── seq_00/, seq_01/, seq_02/
    ├── camera/timestamps.csv    frame,stamp_s       (frames/000000.png ... : 320x240 RGB, about 10 Hz)
    ├── camera/frames/*.png
    ├── lidar/timestamps.csv     scan,stamp_s        (scans/000000.npz ... : about 10 Hz)
    ├── lidar/scans/*.npz        range_mm (32x1024 uint16, 0 = no return), reflectivity (32x1024 uint8)
    ├── imu.csv                  stamp_s,gx,gy,gz,ax,ay,az    (200 Hz, rad/s and m/s^2)
    └── wheel_odom.csv           stamp_s,v_mps,yaw_rate_rps   (50 Hz)
```

- **Sequences.** Each is 24 to 28 s long. Each starts and ends with the robot standing still for about 2 to 3 s, and some also stop briefly in the middle. They are minutes apart within one recording session. `manifest.json` lists them with their approximate start times. The robot drives on a flat floor; its suspension rolls and pitches the body by a few degrees when it moves.
- **LiDAR.** Ring `i` points at elevation `manifest.lidar.elevation_deg[i]`. Column `j` points at azimuth `360*j/1024` degrees, counter-clockwise from +x. A return is `range * (cos(el) cos(az), cos(el) sin(az), sin(el))` in the LiDAR frame. Each scan has already been motion-compensated by the driver to its stamp, so treat a scan as taken at one instant. Ranges have about 1 cm of noise. Reflectivity measures the surface's near-infrared reflectance and depends on the incidence angle. It follows the visible brightness of a surface only in part.
- **Camera.** Pinhole with the `K` in the manifest, no distortion, global shutter, 8-bit RGB with a little noise. The scenes are rooms with boxes, shelves, pillars and textured surfaces.
- **IMU.** The gyro measures angular velocity plus a constant bias plus noise. The accelerometer measures specific force plus a constant bias plus noise; at rest it reads +9.80665 m/s^2 along "up". Both are in the IMU's own frame. The IMU and the wheel odometry are stamped by the robot's reference clock.
- **Wheel odometry.** The body's forward speed and yaw rate, with a few percent of scale error and some wheel slip when accelerating.

## Frames and conventions

- World: z up, gravity (0, 0, -9.80665) m/s^2. Body: x forward, y left, z up, origin on the floor. LiDAR: x forward, y left, z up. `T_body_lidar` in the manifest is exact.
- Camera: the OpenCV optical frame (x right, y down, z forward).
- `T_a_b` is the 4x4 pose of frame `b` in frame `a`: `p_a = R_a_b p_b + t_a_b`, with the last row `[0, 0, 0, 1]`. Metres and radians throughout.
- Clocks: an event at reference time `t` (seconds since the start of the session, the clock in `imu.csv`) is stamped by a sensor with offset `o` (s) and drift `d` (ppm) as `stamp = t + o + d * 1e-6 * t`. The camera and the LiDAR each have their own `o` and `d`. On the hidden rigs, `|o|` is at most 0.1 s for the camera and 0.06 s for the LiDAR, and `|d|` is at most 400 ppm for the camera and 150 ppm for the LiDAR. Stamps also carry about 0.2 ms of jitter.
- `rough_prior.yaml` is off from the truth by up to 10 degrees and 8 cm for `T_lidar_camera`, and up to 5 degrees and 4 cm for `T_lidar_imu`. Gyro biases are up to 0.015 rad/s per axis and accelerometer biases up to 0.15 m/s^2 per axis.

## calibration.yaml (output)

```yaml
T_lidar_camera: [[r00, r01, r02, tx], [r10, r11, r12, ty], [r20, r21, r22, tz], [0, 0, 0, 1]]   # camera in the LiDAR frame
T_lidar_imu:    [[...], [...], [...], [0, 0, 0, 1]]                                             # IMU in the LiDAR frame
time:
  camera: {offset_s: 0.0, drift_ppm: 0.0}
  lidar:  {offset_s: 0.0, drift_ppm: 0.0}
imu:
  gyro_bias_rps:   [bx, by, bz]     # in the IMU frame; measured = true + bias
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
| `camera_frozen` | camera | the driver re-sends the last good image (bit-identical) with fresh stamps |
| `camera_time_jump` | camera | the frames' stamps are shifted by a constant 35 to 80 ms (either sign) |
| `lidar_degraded` | lidar | a large share of returns are replaced by spurious short, dim returns |
| `imu_axis_swap` | imu | two axes of both the gyro and the accelerometer are exchanged |
| `odom_glitch` | wheel_odom | the forward speed reads about zero or jumps by a large offset |

`start_s` and `end_s` are stamps from the affected sensor's own log: the camera's or LiDAR's `timestamps.csv`, or `stamp_s` in `imu.csv` or `wheel_odom.csv`. They are the first and last affected samples. At most 50 entries.

## How it is graded

Six hidden rigs are generated fresh for every grading run. Your program is run once per rig, **all six at the same time** on an 8-CPU machine. Each run has the working directory `/home/user/submission/`, runs as your user, has **no network** (AF_UNIX sockets only), and has a **900 s wall-clock limit** and a **6144 MB address-space limit**. A run still going at its limit is killed, and the rig is scored on whatever files exist by then. NumPy, SciPy and OpenCV are pinned to one thread (`OMP_NUM_THREADS=1` and related settings, also defaults of the image's Python), so plan on one core per run. Nothing your program prints is read.

Each rig earns a credit in [0, 1]:

```
credit = 0.35 * alignment + 0.30 * downstream + 0.15 * time_sync + 0.10 * imu + 0.10 * faults
```

Each error `e` becomes a credit `c(e) = clamp(log(zero/e) / log(zero/full), 0, 1)`: 1 at or below `full`, 0 at or above `zero`, log-linear in between.

| term | measured as | full | zero |
|---|---|---|---|
| alignment | `projection_px`: median pixel distance between where held-out LiDAR points land in held-out camera frames with your `T_lidar_camera` and clocks, and where they really appear | 0.5 px | 8 px |
| downstream | mean of `c(rpe_rot_deg)` and `c(rpe_trans_m)`: the mean error of the LiDAR's motion over 1 s windows of the held-out sequence, predicted by integrating the held-out IMU samples with your `T_lidar_imu` and biases | 0.02 deg, 0.012 m | 0.5 deg, 0.25 m |
| time_sync | `time_rms_s`: RMS error of the reference times your clock models give for the held-out camera and LiDAR stamps (mean of the two) | 1 ms | 30 ms |
| imu | mean of `c` for the IMU rotation error, the gyro bias error (norm) and the accelerometer bias error (norm) | 0.1 deg, 3e-4 rad/s, 0.015 m/s^2 | 3 deg, 0.01 rad/s, 0.15 m/s^2 |
| faults | F1 of your reported intervals; a report matches a true fault with the same sequence and sensor and interval IoU >= 0.3 (each interval padded by 0.05 s), counting 1 with the right type and 0.5 with a wrong one | | |

More precisely:

- **Projection.** For a held-out camera frame and LiDAR scan, each LiDAR point is placed in the world with the LiDAR's true pose at the reference time your LiDAR clock gives for the scan's stamp. It is then projected into a camera whose pose is the LiDAR's true pose at the time your camera clock gives for the frame's stamp, composed with your `T_lidar_camera`. The result is compared with the projection under the true times and extrinsic. Points behind your camera count as 1000 px.
- **Downstream.** Each window starts from the true LiDAR pose and velocity. Its IMU pose is the LiDAR pose composed with your `T_lidar_imu`. It integrates your bias-corrected gyro and accelerometer samples at 200 Hz, then maps the result back to a LiDAR pose through your `T_lidar_imu`.
- **Held-out sequence.** It is recorded later in the same session than the sequences you are given, so clock drift matters.
- **Missing or malformed files.** A missing or unparsable `calibration.yaml` earns 0 on the first four terms for that rig. A missing or malformed `fault_report.json` earns 0 on the fault term.

The graded number is the mean credit over the six rigs, compared with what a reference calibrator scores on the same kind of rigs.

## Tools

- `python3 ~/practice_grader/grade.py` runs your `run.sh` on the three practice rigs the way grading does: all at once, 900 s and 6144 MB each, working directory `/home/user/submission/`. It then prints every raw metric above for each rig, computed with the same measurement code against the practice rigs' truth and their own held-out sequences. Use `--rigs rig_b` to run one rig, and `--no-run --outputs DIR` to measure existing outputs. Free and unlimited.
- `/opt/rigcal/practice_truth/<rig>/` holds each practice rig's true parameters (`ground_truth.yaml`, in the output format), its faults (`faults.json`) and its held-out data (`heldout.npz`).
- `~/tools/rig_io.py` has readers for the format, `~/tools/overlay.py` draws a scan over a camera frame for a given calibration, and `~/tools/prior_baseline.py` writes the output format from the prior.

## Environment

Python 3.12 (`python3`, `/opt/venv`) with numpy 2.5.4, scipy 1.18.1, opencv-python-headless 5.0.0.93, PyYAML 6.0.2 and matplotlib 3.11.2. Your program is graded on exactly this interpreter and these packages, and nothing else can be installed. You have 8 CPUs, 16 GB of memory, no GPU, no network and 2 hours.

# How the rigs' logs were generated (verify never runs this)

The practice and hidden rigs are generated ONCE, offline, and stored: the practice rigs as one archive of the task's
assets (fetched at image build), the hidden rigs' logs as one archive fetched at verify (`tests/verify.py`), their
truth in `tests/hidden/`. This folder is the generator, kept for review and for regenerating a rig.

| file | what |
|---|---|
| `rigsim.py` | a rig's parameters from its seed; motion, clocks, IMU, odometry, faults; the LiDAR sweep model; the camera's lens, noise and JPEG; the log writers; the held-out data |
| `rigsite.py` | the site: the hall, the props placed per rig, the robot's body; ray casting (Embree) on their triangles; occupancy and prop placement; the near-infrared reflectance table |
| `assets.py` | triangles and material names of a USD file (the files the renderers draw) |
| `bl_render.py` | Blender (EEVEE, headless): the pinhole source image of every camera frame from the same USD files |
| `gen_rigs.py` | one rig end to end |

```
<python 3.12: numpy 2.5.4, scipy 1.18.1, opencv-python-headless 5.0.0.93, pyyaml, embreex, usd-core> -I -B gen_rigs.py \
    --set practice --index 1 --name rig_b --hall <site>/hall.usdc --props <site>/props --husky <site>/husky \
    --cache <dir> --out <root> [--blender blender]        # Blender 5.3
```

## One scene, two renderers

- **Site.** A warehouse hall (60 x 40 x 11 m; 24 materials with their maps: polished concrete floor with painted
  lanes, panel walls, roller doors, poles, movable partitions, cable trays, ducts, lamps) and 34 kinds of warehouse
  and factory props at their own size. Per rig a working zone of 24-34 x 16-24 m is drawn and 30-45 small and 3-6
  large props are placed in it by rejection sampling, each at least 0.9 m clear of the rig's four drives; every
  kind is used once before any repeats.
- **Camera frames** are rasterised by Blender EEVEE from those USD files; the demonstration pictures are path
  traced (OVRTX) from the same files. The frame is drawn as a pinhole image at 1.5 x the focal length and then goes
  through the rig's lens distortion, vignetting, exposure, sensor noise and JPEG (`rigsim.finish_frame`).
- **LiDAR** rays are cast on the triangles of the same USD files, every column from the LiDAR's pose at its own
  time, plus the robot's own body (Clearpath Husky A200 meshes with the 510 mm sensor arch). Reflectivity comes
  from a table of near-infrared reflectance per material class (`rigsite.NIR`), not from the colour maps.

## Sensors (datasheet figures used)

| sensor | model | figures |
|---|---|---|
| camera | FLIR Blackfly S BFS-U3-16S2C (Sony IMX273, global shutter), 2x2 binned, 4 mm lens | 720 x 540, 6.9 um binned pixel, fx 565-595 px (about 64 deg horizontal), Brown-Conrady k1 -0.14..-0.08, k2 0.02..0.07, p1, p2 within 0.0008; 10 Hz free-running; exposure 2 ms (motion blur under about 1 px, not modelled); vignetting 25 % at the corner; noise 1.2-2.0 levels; JPEG quality 92 |
| LiDAR | Ouster OS1-32 (uniform beams), 1024 x 10 Hz | 32 beams over +-22.5 deg; one sweep 100 ms, each column at its own time; minimum range 0.5 m; range noise 1 cm to 20 m rising to 3 cm at 60 m; a return is lost under the level of a 10 % target at 60 m, at grazing incidence (cosine under 0.06) and for 0.5 % at random; mixed returns at depth edges |
| IMU | Bosch BMI088, 200 Hz | gyro bias within 0.015 rad/s (datasheet zero-rate offset 1 deg/s), noise 0.0035 rad/s per sample (0.014 deg/s/sqrtHz); accelerometer bias within 0.15 m/s^2 (datasheet 20 mg), noise 0.024 m/s^2 per sample (175 ug/sqrtHz); scale and cross-axis within 0.1 % (after factory trim); bias walk 2e-5 rad/s and 2e-4 m/s^2 per sqrt(s) |
| wheel odometry | Husky A200 encoders, 50 Hz | speed scale within 3 %; yaw-rate factor 0.80-1.00 (skid steer); slip up to 4 % accelerating and 3 % turning; noise 0.01 m/s, 0.01 rad/s |

## Time

The camera and the LiDAR run free on their own clocks for the whole session: frame n happens at sensor time
`phase + n * 0.1 s + trigger jitter` and is stamped with that time plus a stamping jitter; the reference time of the
event is `(s - offset) / (1 + drift)`. A sensor's log of a sequence starts 0.2-1.0 s after the sequence starts and
ends as long before its end, drawn per sensor and sequence. So neither the stamps' spacing nor where a log starts
says anything about offset or drift (`tests/review/stamp_leak.py` measures that on every rig).

## Rigs

- Seed of rig i of a set: the first four 32-bit words of
  `sha256("ale-robotics/multisensor_rig_calibration_from_logs/v2/<set>/<i>")` (`rigsim.rig_seed`), set `practice`
  (rig_a, rig_b, rig_c) or `hidden` (rig_00 .. rig_05). A rig that fails the acceptance check (the generator's own
  placement checks; every scored parameter pinned inside its full point by `tests/review/ident.py`) is replaced by
  index `<i>.1`. Used: practice `0.1`, `1`, `2` (index `0` was rejected: its camera clock was pinned to 4.8 ms
  only, one of its sequences has little rotation); hidden `0` .. `5` (none rejected).
- Everything but the camera frames is a pure function of the seed and the asset files (byte-identical on two
  machines; .npz members carry fixed dates). A second rendering of one sequence gave 256 of 260 identical JPEG
  files; the other four differ by at most 4 levels in under 100 pixels. The stored logs are the task: the practice
  archive and the hidden archive are pinned by sha256, and every hidden file again by `tests/hidden/<rig>/sha256.json`.

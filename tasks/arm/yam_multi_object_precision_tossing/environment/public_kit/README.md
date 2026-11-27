# RGB-D controller development kit

The read-only kit at `/opt/public-kit` is available during development and in the separate verification environment. Python can import `rgbd_client` directly. Development-home copies are working files. Place every edited runtime helper and its data files in `/home/user/submission` so artifact transfer includes them.

This kit decodes the recorded observation in `~/sensor_sample.json`, projects depth into the calibrated world frame, computes the robot's grasp pose, and checks a controller's stdio protocol. Physical task success is measured during evaluation.

## Inspect the recorded sensor observation

```bash
python3 /opt/public-kit/rgbd_client.py ~/sensor_sample.json --output ~/sensor_views
```

For each camera, this writes an RGB PNG, a lossless float32 depth `.npy`, and a depth-preview PNG. `camera_metadata.json` records calibration and the preview's metre range. Use the `.npy` depth for calculations. Invalid depth is non-finite or non-positive. Preview brightness is scaled for inspection.

The sample is an observation object with `time`, `arms`, and `cameras`. The four camera names are `left_overhead`, `left_oblique`, `right_overhead`, and `right_oblique`. Each camera supplies:

- `rgb_png_base64`: base64-encoded RGB PNG.
- `depth_f32_zlib_base64`: base64-encoded, zlib-compressed, row-major little-endian float32 depth.
- `width`, `height`, `K` (row-major 3×3), and `T_world_camera` (row-major 4×4).
- `depth_units: "m"`, `depth_convention: "camera_z"`, `depth_dtype: "<f4"`.

Depth is axial camera z in metres. Camera coordinates use x right, y down, and z forward. Integer `(u,v)` indices denote pixel centres. `T_world_camera` maps camera coordinates to world coordinates.

```python
import json
import numpy as np
from rgbd_client import decode_camera, unproject, project, forward_kinematics

obs = json.load(open("/home/user/sensor_sample.json"))
camera = obs["cameras"]["left_overhead"]
rgb, depth = decode_camera(camera)
u, v = 320, 240
if np.isfinite(depth[v, u]) and depth[v, u] > 0:
    point_world = unproject(camera, [u, v], depth[v, u])
    pixel_uv, camera_z = project(camera, point_world)
tcp_world = forward_kinematics(obs["arms"]["left"]["joints"], "left")
```

The two arm records contain `joints`, `joint_velocity`, `finger_positions`, `finger_velocity`, `reference`, and `jaw_reference`. Angles are radians; finger positions and jaw half-gap are metres. `robot_kinematics.json` provides fixed own-robot link transforms, joint axes and limits, base calibration, and the grasp TCP. `forward_kinematics` returns a 4×4 grasp-TCP transform from six measured joint angles. It is a geometric calculation.

## Identify the static action IDs

Identify pieces from the initial images and track their identities after motion.

For the left arm, sort the six initial pickup centres by increasing polar azimuth about the left robot base in the world XY plane: `atan2(y-base_y, x-base_x)`. This order is `left_item0`, `left_item1`, …, `left_item5`. Initial world-y order can differ from this azimuth order. Colors, shapes, and poses are inferred from the images.

For the right arm, initial increasing world-y order is:

| Initial order | Action ID |
|---|---|
| 1 | `right_slot_large_play_piece` |
| 2 | `right_slot_medium_play_piece` |
| 3 | `right_slot_small_play_piece` |
| 4 | `right_ring_large_play_piece` |
| 5 | `right_ring_medium_play_piece` |
| 6 | `right_ring_small_play_piece` |
| 7 | `right_wood_play_piece` |

The hollow rings, solid disks, and rectangular wood block also provide visual identity cues. Keep each initial ID attached to the tracked piece throughout the round.

## Check the controller protocol

```bash
python3 /opt/public-kit/sensor_smoke.py --controller ~/submission/run.sh --sample ~/sensor_sample.json
```

The smoke test launches the controller, waits up to 60 seconds for `READY`, and sends `reset`, one `act` request containing the recorded observation, then `close`. Each request has a 15-second complete-response deadline. It validates JSON replies, the static item vocabulary, waypoint structure, joint limits, and jaw range. It prints `physical_success: null` because the result measures protocol validity.

The action schema is unchanged: select `arm`, `item`, and 2–60 waypoint rows `[time,q1,q2,q3,q4,q5,q6,jaw_half_gap]`, or return `{"finish":true}`. Waypoint time starts at zero, increases strictly, and ends by six seconds. The evaluation applies the final command through the remainder of that action. A round contains at most 20 actions. Follow `instruction.md` for the complete task and acceptance rules.

Repeated step indices can occur during responsiveness checks. Compute each reply from its supplied observation; request count is not an action clock. The recorded sample smoke checks one request and does not measure task performance or responsiveness across layouts.

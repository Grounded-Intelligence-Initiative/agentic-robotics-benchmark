# Control a Unitree G1 in SONIC token space

Write a controller that picks up the orange object and places it upright on the green pad. The free-standing G1 has 29 body joints and two Dex3 hands. You control the body by choosing **64 integer codes per 20 ms** for the public SONIC decoder. You also command the left and right hand closures. The task runs in MuJoCo on CPU.

Each episode contains one box or one cylinder. The starting object position, pad position, object yaw, mass and friction vary. Keep the robot upright throughout the lift and release.

- **Lift.** Raise the object at least 4 cm. For 40 consecutive ms, two parts of the same hand must make opposing contacts (the palm can count as one). The hands must carry at least half the object's weight, and other supports at most 5%.
- **Place.** Release the object fully inside the green pad. Either end of the box or cylinder can face up. For 250 consecutive ms the object must stay nearly vertical and still, with no hand contact, while the pad carries at least half its weight.

An episode ends on successful placement, a fall or at 750 control ticks (15 simulated seconds). The raw metric is the fraction of episodes with a successful placement. The verifier also checks that your body codes respond to the observations.

## Start here

Your deliverable is **`/home/user/submission/run.sh`** and everything it needs under `/home/user/submission/`. The environment supplies Python 3.12, NumPy, SciPy, Pillow, MuJoCo, ONNX Runtime, the robot model and the original SONIC decoder. You have one hour, four CPU cores and 8 GiB of memory for development. The model and robot assets are already installed. Your submitted controller operates offline.

These commands create a working wire example in a fresh submission directory, save the public practice cameras, and run ten control ticks. The starter sends the neutral code and opens both hands. Replace its action selection to solve the task.

```bash
cd /home/user
cp starter.py submission/controller.py
cat > submission/run.sh <<'SH'
#!/bin/sh
exec /opt/venv/bin/python3 -u controller.py
SH
/opt/venv/bin/python3 practice.py inspect --task box_to_pad --seed 7 --out /tmp/g1-views
/opt/venv/bin/python3 practice.py run --submission submission --task box_to_pad --seed 7 --steps 10
```

For a full rollout omit `--steps`. Try both `box_to_pad` and `cylinder_to_pad` with different seeds. The practice driver exercises your submitted process and prints the raw physical outcome. Look at the saved `hand` and `side` views together to see where the gripper and object sit relative to the pad.

## Process and actions

The host starts `bash run.sh` in your submission directory as an unprivileged user. Print the literal line `READY` and flush it within 60 seconds. Then read one JSON request per line from stdin, emit one JSON reply per line within 15 seconds, and flush stdout after each reply. Send diagnostics to stderr.

| Request | Reply |
| --- | --- |
| `{"type":"reset"}` | `{"ok":true}` and clear per-episode state |
| `{"type":"act","t":t,"obs":obs}` | `{"action":{"rows":[...]}}` |
| `{"type":"close"}` | `{"ok":true}`, then exit |

Each action row has exactly two fields. `codes` is a length-64 list of integers in `[0,31]`. Each code `c` becomes the native finite scalar quantization (FSQ) value `(c - 16) / 16`; code 16 is zero. `hand` is `[left,right]`, each in `[0,1]` from open to fully closed. For example, this Python expression builds one valid neutral reply:

```python
reply = {"action": {"rows": [{"codes": [16] * 64, "hand": [0.0, 0.0]}]}}
```

One row advances one physical tick. You may send one to five rows, up to the next camera boundary or the end of the episode. At `t=0` the limit is five, at `t=3` it is two, and at `t=4` it is one. The host validates the entire chunk before applying any row and stops on a physical terminal event. The next request's `t` is the number of ticks actually executed since reset.

The host may ask about several observations at the **same** tick, in either order. Produce an action for each input from the same pre-tick state. If your policy carries recurrent memory, cache each candidate next state by `obs["observation_id"]`. When `t` advances, `obs["last_observation_id"]` identifies which candidate the host applied. At reset it is `null`. A timeout, malformed action or crash aborts the episode. Make the process safe to start again.

## Observations

`obs["protocol"]` is `g1-sonic-rgb/1`. `obs["task"]` identifies the shape (`box_to_pad` or `cylinder_to_pad`), and `obs["tick"]` equals the request's `t`. The remaining fields are:

| Field | Contents |
| --- | --- |
| `observation.q`, `observation.dq` | 29 body joint positions (radians) and velocities (rad/s) in Unitree hardware order. |
| `observation.base_pos`, `observation.torso_pos`, `observation.left_hand_pos`, `observation.right_hand_pos` | World-frame positions in metres; the hand positions are the grasp sites. |
| `observation.base_quat`, `observation.torso_quat` | Unit quaternions in `[w,x,y,z]` order. |
| `observation.base_gyro`, `observation.gravity_body` | Pelvis angular velocity in rad/s and the unit gravity direction, in pelvis coordinates. |
| `last_hand` | Last physically applied left and right hand commands. |
| `vision`, `planner_vision` | Three calibrated, synchronized RGB views rendered at 128×96 and 480×360 respectively. Both are present at ticks 0, 5, 10, … and `null` between captures. `vision_tick` is the most recent capture tick. |
| `decoder_history` | 930 float values already packed in the order required by the released decoder at this tick. |
| `observation_id`, `last_observation_id` | Opaque identifiers for managing recurrent state as described above. |

Each camera packet has `tick`, `mime` (`image/png`), `images` and `calibration`. Both dictionaries use `overview`, `hand` and `side`. All three viewpoints are fixed in the world; `hand` is a close-up of the work area. Decode an image with:

```python
import base64, io
from PIL import Image

frame = obs["planner_vision"]
if frame is not None:
    image = Image.open(io.BytesIO(base64.b64decode(frame["images"]["hand"]))).convert("RGB")
    camera = frame["calibration"]["hand"]
```

Calibration gives the optical centre `position` (world metres), unit world axes `right`, `up`, `forward`, `focal_px`, `center_px` and the raster dimensions. Pixel coordinates start at the centre of the top-left pixel and increase rightward and downward.

## Using the released decoder

A direct token policy can produce codes from the images and proprioception. A visual planner can use pixel geometry and evaluate candidate codes with the same decoder that will drive the robot. Both approaches submit the same action rows. For a local one-step query, initialize this decoder once and use the supplied history for the current tick:

```python
import numpy as np
from sonic_sim.decoder import Decoder

decoder = Decoder("/opt/sonic/assets/policy/model_decoder.onnx")

def body_targets(codes, obs):
    token = (np.asarray(codes, dtype=np.float32) - 16) / 16
    packed = np.concatenate((token, np.asarray(obs["decoder_history"], dtype=np.float32)))
    return decoder.joint_targets(decoder.decode_observation(packed))  # 29 targets in radians
```

A local query predicts the decoder's body targets. The applied codes then drive a feedback-controlled, free-base robot, so grasp, slip, contact and placement must be observed over time. The next physical observation supplies the updated history.

# Walk a Unitree G1 to a table and place an object

Write a controller for a free-standing G1 with 29 body joints and two Dex3 hands. It starts about a metre behind the work area. Walk forward, stop upright at the table, grasp the orange box or cylinder, then release it upright on the green pad. One continuous MuJoCo episode carries the robot's motion, decoder history and contacts through all four stages. The robot's initial pose and the object's location, pad location, mass, friction and object yaw vary across episodes.

| Stage | What the physics checks |
| --- | --- |
| Walk | Advance the pelvis at least 0.72 m in world +x and each ankle at least 0.60 m. Each foot must rise at least 0.020 m. Establish at least three alternating single-support phases of at least 0.040 s each. |
| Dock | After walking, hold the pelvis within `abs(x) <= 0.100 m`, `abs(y) <= 0.110 m` and `abs(yaw) <= 0.30 rad` for 0.50 s. Maintain height of at least 0.62 m, up-axis cosine of at least 0.94 and horizontal speed of at most 0.10 m/s throughout. Touching the object with a hand or bracing on a surface before docking ends the episode. |
| Lift | Raise the object at least 4 cm while two parts of the same hand make opposing contacts. The hands must support at least half its weight, with other supports carrying at most 5%, continuously for 40 ms. |
| Place | Keep the whole object footprint inside the green pad for 250 ms. The object must be nearly vertical, nearly still and clear of both hands. The pad must bear at least half its weight and other supports at most 5%. Either end may face up. |

One action row advances 20 ms. The episode ends on a strict placement, a fall, an approach violation or after 1,100 rows (22 simulated seconds). The raw score is the number of strict placements divided by 16 independent evaluation episodes, eight per shape. Each episode contributes one or zero. Intermediate stages, motion smoothness and code usage are diagnostics. The practice summary’s `motion` object reports applied rows, individual body codes, simulated seconds and the fraction of code coordinates that changed between consecutive applied rows. It also gives the 29-joint measured acceleration RMS over 0.10 s velocity differences. A run with fewer than five rows reports zero RMS and zero acceleration samples.

The verifier also checks whether the body commands respond to the current sensors. Across the 16 episodes, it requires at least 16 same-tick comparisons between different sensor packets and at least eight comparisons of identical packets. A response counts as changed when any of the 64 body codes in its first row differs by at least one bin. Subtract the fraction of identical-packet comparisons that changed from the fraction of different-packet comparisons that changed. This difference must be at least 0.12. Only the response to the live packet advances the real robot. Missing the comparison counts or the 0.12 threshold gives a panel reward of zero.

With the observation check passed, the final reward is the number of strict placements divided by the calibrated placement count for full credit, capped at 1. Calibration records the physical outcome on independent reference panels.

## Start here

Your deliverable is **`/home/user/submission/run.sh`** and everything it needs under `/home/user/submission/`. The environment supplies Python 3.12, NumPy, SciPy, Pillow, MuJoCo, ONNX Runtime, the robot model and the original SONIC decoder. You have one hour, four CPU cores and 8 GiB of memory for development. The released SONIC reference-motion encoder, body decoder and robot assets are already installed. Your submitted controller operates offline.

These commands create a working wire example in a fresh submission directory, save the public practice cameras, and run ten control ticks. The starter sends the neutral code and opens both hands. Replace its action selection to solve the task.

```bash
cd /home/user
mkdir -p submission
cp starter.py submission/controller.py
cat > submission/run.sh <<'SH'
#!/bin/sh
exec /opt/venv/bin/python3 -u controller.py
SH
/opt/venv/bin/python3 practice.py inspect --task box_to_pad --seed 7 --out /tmp/g1-mobile-views
/opt/venv/bin/python3 practice.py run --submission submission --task box_to_pad --seed 7 --steps 10
```

For a full rollout omit `--steps`. Try both `box_to_pad` and `cylinder_to_pad` with different seeds. The practice driver exercises your submitted process and prints the raw physical outcome. The `repeats` fields count how often the first body-code vector changed for a second observation at the same tick and for an identical observation. The neutral starter should report zero changes. Use the overview to follow the approach. The hand and side views show the table, object and pad. Try the cylinder with public seed 19 as a second practice case. The practice summary includes walking, docking, lift, placement, approach violations and the terminal reason; these fields stay on the host.

## Process and actions

The host starts `bash run.sh` in your submission directory as an unprivileged user. Print the literal line `READY` and flush it within 60 seconds. Then read one JSON request per line from stdin, emit one JSON reply per line within 15 seconds, and flush stdout after each reply. A reply may contain up to 65,536 decoded characters including the newline. Send diagnostics to stderr.

| Request | Reply |
| --- | --- |
| `{"type":"reset"}` | `{"ok":true}` and clear per-episode state |
| `{"type":"act","t":t,"obs":obs}` | `{"action":{"rows":[...]}}` |
| `{"type":"close"}` | `{"ok":true}`, then exit |

The reset reply must have exactly the key `ok` with the JSON boolean `true`. The act reply must have exactly the key `action`.

Each action row has exactly two fields. `codes` is a length-64 list of integers in `[0,31]`. Each code `c` becomes the native finite scalar quantization (FSQ) value `(c - 16) / 16`; code 16 is zero. `hand` is `[left,right]`, each in `[0,1]` from open to fully closed. For example, this Python expression builds one valid neutral reply:

```python
reply = {"action": {"rows": [{"codes": [16] * 64, "hand": [0.0, 0.0]}]}}
```

One row advances one physical tick. You may send one to five rows, up to the next camera boundary or the end of the episode. At `t=0` the limit is five, at `t=3` it is two, and at `t=4` it is one. The host validates the entire chunk before applying any row and stops on a physical terminal event. The next request's `t` is the number of ticks actually executed since reset.

The host may ask about several observations at the **same** tick, in either order. Produce an action for each input from the same pre-tick state. If your policy carries recurrent memory, cache each candidate next state by `obs["observation_id"]`. When `t` advances, `obs["last_observation_id"]` identifies which candidate the host applied. At reset it is `null`. A timeout, malformed action or crash aborts the episode and it counts as a failed placement. The host can restart your process up to three times for subsequent episodes. It must print `READY` on each launch.

The feedback check compares changes in the **first row's body codes** across two observations with changes on identical repeated inputs. If your controller samples actions, reusing the random draw for all observations at one tick lets the check distinguish your response to the scene from sampling noise.

## Observations

`obs["protocol"]` is `g1-sonic-mobile-rgb/1`. `obs["task"]` identifies the shape (`box_to_pad` or `cylinder_to_pad`), and `obs["tick"]` equals the request's `t`. The remaining fields are:

| Field | Contents |
| --- | --- |
| `observation.q`, `observation.dq` | 29 body joint positions (radians) and velocities (rad/s) in Unitree hardware order. Import `BODY_JOINTS` from `sonic_sim.physics` for the names in that exact order. |
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

## Generating body codes

The optional GEAR-SONIC reference encoder accepts a short motion that you design. Import `ReferenceEncoder` from `sonic_sim.encoder` and construct it once. Its `codes_g1(joint_pos, joint_vel, anchor_rot6d)` method returns 64 integer bins. The first two arguments are ten 29-joint poses (radians) and velocities (rad/s) in Unitree hardware order, spaced five control ticks apart. The last argument is a `[10, 6]` array. For each pose it contains the first two columns of `R_current_pelvis.T @ R_reference_pelvis`, flattened in row-major order. Call `.tolist()` before writing the result to JSON. The encoder and decoder run locally on CPU.

### Inspecting a decoder target

Use the same code-and-hand interface for walking, stopping and manipulation. A direct token policy can produce codes from the images and proprioception. A visual planner can use pixel geometry and evaluate candidate codes with the same decoder that will drive the robot. Both approaches submit the same action rows. For a local one-step query, initialize this decoder once and use the supplied history for the current tick:

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

# Deliver the object to the illuminated station

Control a free-standing Unitree G1 in one continuous MuJoCo episode. An orange box or cylinder rests on a table with two gray receiving pads. Start either within reach or about one metre away. Approach if needed, stand quietly at the work position, and watch for one pad to turn teal. Lift the object with the hand, place it fully on the illuminated pad, release it and let it settle. A qualifying release on the other pad ends the episode. Lighting first changes **after the physical dock**. Positions, orientations, materials and robot starts vary across layouts.

## What counts

The far start requires at least 0.72 m of forward pelvis travel, 0.60 m of forward travel and 0.020 m of rise by each ankle, and at least three alternating single-support phases of 0.040 s each. Both start modes require 0.50 s of uninterrupted quiet standing at the work position. The pelvis must be within 0.10 m forward/back and 0.11 m sideways of the work position, face within 0.30 rad of the table, stay at least 0.62 m high and within an upright cosine of 0.94, and move horizontally at at most 0.10 m/s. Hand contact with the object before docking or using the table or body for support during the approach invalidates that episode. Normal sole-floor contact supplies balance.

After docking, support the object against gravity using opposed hand contacts, lift it at least 0.040 m above its initial height and hold the loaded lift for 0.040 s. The object must then fit fully over the receiving surface, touch it, carry at least half its weight there, and remain upright, hand-free and quiet for 0.25 s. Maximum linear and angular speeds during release are 0.08 m/s and 1.2 rad/s. The native simulation ends on qualifying placement, an approach violation, a fall, or 1,100 applied control rows (22 s).

Evaluation runs three separate episodes on each physically matched layout. One destination occurs twice and the other once, with all six possible sequences equally likely. The score is the fraction of layouts where **all three episodes** complete their required delivery. The grader also reports individual deliveries and the number of layouts where each destination was served at least once. Stage counts, motion cost and response to camera variations are diagnostics. The grader launches a fresh controller process with an empty temporary work area for each episode. It runs with no network, four CPU cores and 8 GiB of RAM. Write episode-local files under `$TMPDIR`. The agent development budget is one hour.

## Sensors and control

The verifier sends JSON Lines over standard input. Write one JSON line per request to standard output. Send logs to standard error. Start `/home/user/submission/run.sh` from its submission directory. It must print `READY` within 60 s, acknowledge `{"type":"reset"}` with `{"ok":true}`, and answer each `{"type":"act","t":t,"obs":obs}` within 15 s. A `close` request ends the process. A malformed or late response loses the episode.

| Observation field | Contents |
| --- | --- |
| `protocol`, `task`, `tick` | Constants `g1-sonic-visual-delivery/1`, `visual_delivery`, and the 50 Hz control tick. |
| `observation.q`, `observation.dq` | Measured angles in radians and velocities in rad/s for 29 body joints in the published G1 MuJoCo order. |
| `observation.base_gyro`, `observation.gravity_body` | Pelvis-frame angular velocity in rad/s and unit downward gravity, three coordinates each. |
| `vision` | Every fifth tick, three native 128×96 RGB PNGs in `images`, keyed by `overview`, `hand` and `side`. |
| `planner_vision` | The same three fixed views at 320×240. Each has an RGB PNG in `images`, an optical-axis depth PNG in `depth_mm`, and pixel intrinsics in `intrinsics`. The grayscale depth is unsigned 16-bit millimetres. Zero denotes background or a sample outside the 12 m range. PNGs are base64-encoded. |
| `vision_tick`, `last_hand` | Most recent scheduled camera tick and the last applied left/right hand commands. Both image fields are null between captures. |
| `observation_id`, `last_observation_id` | SHA-256 sensor packet identifier and the identifier associated with the last action actually applied. The latter starts as null. |

Respond with `{"action":{"rows":[{"codes":[...],"hand":[left,right]}]}}`. Each row supplies **64 integers from 0 through 31** to the released SONIC V1 body decoder, plus two finite closure commands in [0,1] for the left and right hands (0 opens). A row advances ten 0.002 s physics substeps. A request accepts one to five rows, up to the next camera capture or episode horizon. The grader stops applying rows when the episode ends. It may repeat or vary camera inputs at the same tick. The simulator advances on the row(s) for its actual observation. Use the identifiers to commit recurrent state only after the applied branch is known. The physical scorer checks actual contacts, loads and motion.

## Practice

The environment includes Python, NumPy, Pillow, MuJoCo, ONNX Runtime and the public SONIC models. Create a working submission and try either starting distance.

```bash
mkdir -p "$HOME/submission"
cp "$HOME/starter.py" "$HOME/submission/controller.py"
printf '%s\n' '#!/bin/bash' 'exec /opt/venv/bin/python3 -u controller.py' > "$HOME/submission/run.sh"
chmod +x "$HOME/submission/run.sh"
/opt/venv/bin/python3 "$HOME/practice.py" inspect --seed 19 --start far --out "$HOME/delivery-views"
/opt/venv/bin/python3 "$HOME/practice.py" run --seed 19 --start near --shape cylinder --target right --submission "$HOME/submission"
```

Practice accepts any unsigned 64-bit seed and optional `--start near|far`, `--shape box|cylinder` and `--target left|right`. For a matched experiment, keep the seed, start and shape fixed and switch the target. Practice prints endpoint and motion measurements after the run. During an episode the controller receives the sensor packet above.

# RGB-D visual delivery reference

The reference emits native SONIC body codes and two finger closures. A far start uses a fixed walking reference selected from the measured distance between the robot and the table. At a near start the robot stands until the station lights up. Both paths continue in the same physical episode.

The participant calibrates the three fixed RGB-D views against the visible tabletop on the first manipulation frame. At each subsequent 10 Hz camera update it registers a kinematic copy of the public G1 mesh against measured depth. Joint angles and the IMU constrain that fit. The right grasp pocket is 115 mm forward and 48 mm lateral to the right wrist yaw link.

The target comes from the illuminated pad. The controller estimates its center from the visible boundary and retains the clearest measurement for each observed surface. Orange depth points give the object's center and top-face normal. The feedback controller approaches, closes, lifts, moves the object's measured center toward the lit pad, levels it, lowers it, and withdraws. It can retry an empty grasp. An 18 mm forward offset compensates for measured landing shortfall in development. Each image supplies a five-row chunk. The 50 Hz decoder applies each row to the currently measured robot.

The participant process reads only public JSONL observations. Its internal stages use camera measurements and its own command history. Repeated or counterfactual requests at one tick each start from the last committed controller state.

## Run

The benchmark invokes `bash solution/solve.sh` to install the reference into `$ALE_HOME/submission`. The generated `run.sh` implements the public READY, reset and act protocol. Practice can invoke that same program. It runs on CPU with NumPy, SciPy, MuJoCo CPU kinematics, Pillow and the public robot assets.

## Rebuild the native actions

The reference includes two small, generated assets. The 128 walking rows come from a tapered sinusoidal joint reference and the released GEAR-SONIC V1 motion encoder. The 2,925-point Cartesian table contains FSQ outputs and local rotational derivatives compiled from public robot inverse kinematics and the same encoder. During a grasp the live wrist error, torso transform and pelvis orientation determine the next integer action. The body has 64 five-bit coordinates, each in [0, 31], and the fingers remain separate.

From this task's official image, with the solution directory mounted at `/solution`, the standalone commands below regenerate the assets. Shape, distance and target-side labels, physical scores and live simulator state are outside this interface.

```sh
cd /solution
python make_gait.py --out /tmp/approach_codes.npy
python make_cartesian.py --workers 2 --out /tmp/cartesian_fsq.npz
sha256sum /tmp/approach_codes.npy /tmp/cartesian_fsq.npz
```

| File | SHA-256 |
|---|---|
| `approach_codes.npy` | `7e3d779fad10ac6f235e52ddba4f7b8ef08f434bf391ef0ecd6a8b8e61be3151` |
| `cartesian_fsq.npz` | `70752eff1b4c6286cb305de08cf4a61a06893aa3ee0cf1c4605b6544e4c84915` |

The public asset manifest under `environment/` pins the NVIDIA encoder and decoder and the MuJoCo Menagerie G1 robot. The low-level decoder runs in the benchmark environment.

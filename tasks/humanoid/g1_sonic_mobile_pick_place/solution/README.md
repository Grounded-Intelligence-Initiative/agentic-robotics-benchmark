# Mobile reference policy

The reference walks approximately one metre toward the table, waits for a quiet stance in the docking region, then runs the frozen V4 visual pick-and-place controller. The action interface carries 64 five-bit SONIC body codes and two Dex3 closure fractions per 20 ms. Every applied row advances the same free-base MuJoCo robot and released decoder history.

The walk starts with 30 standing rows. Its 128 body rows were generated from an analytic 2.25-cycle leg-and-arm reference with a 48-tick period and 15-tick onset and braking ramps. The public NVIDIA SONIC V1 encoder maps ten reference frames at five-tick spacing to each native code row. Once the gait returns to its standing code, the policy estimates planar speed from consecutive public pelvis positions. It hands control to V4 from tick 250 onward after five camera intervals inside the quiet docking region. V4 then consumes calibrated three-view 128×96 RGB, proprioception and its own recurrent memory every five ticks.

Run `solve.sh` to install the reference JSONL process in the ALE submission directory. Inside the task image, reproduce the stored approach sequence with the public encoder:

```sh
PYTHONPATH=/opt/sonic /opt/venv/bin/python3 make_approach.py --output /tmp/approach_codes.npy
cmp /tmp/approach_codes.npy approach_codes.npy
```

| Artifact | SHA-256 |
| --- | --- |
| Procedural approach rows, NPY | `fbd5985072b2a64e6ceee8c55d689b8a70a253297c3a130054b7c56502649d8b` |
| Frozen V4 high-level actor, ONNX | `2f4ada4c14f540ee1f99b0e875aa63ade0adbb87d63d710d5b15e4e8300d1c55` |
| NVIDIA GEAR-SONIC V1 encoder | `013ab0287236aa2721e13f1e936d699db982302d0de0bfcdae76d5c3245362d3` |
| NVIDIA GEAR-SONIC V1 decoder | `c7241a123eaa36b5d64bad19540efde93cac1ad443bd4572fd12ca99898118ed` |

## V4 checkpoint provenance

The SONIC token-control project trained the tabletop actor by behavior cloning. Its direct and recovery teachers generated body codes and hand closures in MuJoCo. Mixed student/teacher rollouts added corrections on states visited by the student (DAgger). The final fit used 495,343 motor ticks from 878 trials for training and 54,962 ticks from 96 whole-seed validation trials. The model also received auxiliary geometry, contact and teacher-phase supervision during training. At runtime V4 receives the RGB, proprioception and recurrent inputs described above.

| Source record in the SONIC token-control project | Identity |
| --- | --- |
| Demonstration collection | `899880dcb8564376e90ae33f5b4e2b0d8f81c69e` |
| Model and training implementation | `2a50e7c163dbf31ae3b3f96355c779a5b1e5d9cc` |
| Integer ONNX export implementation | `5404dbfa7fe6bfcaa2d8b03016733d9598a8cd85` |
| Frozen rich1, epoch 38, manifest | `8d5200e2b01ae87d05447332e700012842fda21b`, `pixel_policy_v4/weights/freeze.json` |
| Training checkpoint SHA-256 | `d9df4905599acb3cf058eea4145e98679a99ca2e0a5dbc6deab2f722791e4af4` |

The frozen manifest records the distributed integer ONNX hash in the artifact table above. Git attributes the collection, training, export and freeze commits to Zi-ang Cao. The research snapshot has no separate V4 model-license file. The NVIDIA Open Model License accompanies the original GEAR-SONIC V1 encoder and decoder fetched by the task image. [Environment notices](../environment/NOTICE)

## Five-metre public practice

The optional controller uses public pelvis odometry to steer and slow an analytic walking reference. It progressively retracts the arms near the table. After a quiet stance it returns the arms to the standing posture and runs the same frozen V4 visual controller. Each control row passes through the original SONIC decoder. The full course shares one physics episode and one decoder history.

From this solution directory inside the task image, install the optional reference into its own submission directory and run a public practice episode:

```sh
ALE_HOME=/tmp/g1-long bash five_metre/install.sh
/opt/venv/bin/python3 /home/user/practice.py run --course five-metre --task cylinder_to_pad --seed 19 --submission /tmp/g1-long/submission
```

Use `--task box_to_pad --seed 3` for a box example. The host records a settled five-metre dock, the two ankle displacements, load-supported lift and stable pad release. The registered one-metre reference is installed with `solve.sh`.

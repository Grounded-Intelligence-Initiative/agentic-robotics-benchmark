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

The V4 checkpoint and its preprocessing come from the public G1 SONIC tabletop task. Walking uses the synthetic motion defined in `make_approach.py`. The native simulator grades the approach, dock, supported lift and stable release independently of the reference policy.

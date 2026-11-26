# Reference submissions

During ALE validation, `bash solve.sh` installs the frozen V4 visual actor into `$ALE_HOME/submission`. Set `SONIC_REFERENCE_POLICY=p1` when invoking the script to install the alternative planning controller. The installed entry point implements the same READY and JSONL protocol as a contestant submission.

**V4** evaluates an 825,756-parameter recurrent network on the three native 128×96 RGB images, 86 robot-state and previous-finger values, and its own recurrent memory. Each camera update produces five successive body commands, each with 64 coordinates and 32 bins per coordinate, and five left/right hand pairs. `v4_actor.onnx` is the frozen integer-output graph from the public SONIC token-control workflow, SHA-256 `2f4ada4c14f540ee1f99b0e875aa63ade0adbb87d63d710d5b15e4e8300d1c55`. Its supervised training used 878 demonstration episodes and 96 validation episodes. The image runs it on one CPU thread.

**P1** fits the orange object and green pad in the three calibrated 480×360 images. A waypoint controller uses these optical estimates and observed hand motion to time approach, grip, carry and release. Its IK uses link inertias and joint ranges from the public G1 XML. Each 20 ms tick, it evaluates candidate body codes locally on the published 930-value history with the original NVIDIA decoder. It ranks legal integer proposals against desired body targets and returns one body row with both hand closures. The physics engine decodes and applies the chosen row separately. The planner uses the four-to-32-coordinate visual switching rule of the released P1 controller.

The runner stages policy state for each observation ID. On the next tick it recovers precisely the snapshot associated with `last_observation_id`. A second request for the same physical tick can therefore use a different observation without advancing the recurrent network or Cartesian planner twice.

The image supplies `/opt/sonic/assets` and `/opt/venv/bin/python3`. For a local checkout with equivalent packages, set `SONIC_ASSETS` to the directory containing `policy/model_decoder.onnx` and `menagerie_g1/g1_with_hands.xml`, `SONIC_PYTHON` to the interpreter, and put the task's `environment/` on `PYTHONPATH`.

## V4 checkpoint provenance

The SONIC token-control project collected MuJoCo demonstrations with its direct and recovery teachers, plus mixed student/teacher episodes with teacher corrections on the visited states (DAgger). The learned high-level actor used behavior cloning on the corresponding three-camera RGB and robot observations. Training labels included the 50 Hz teacher body codes and hand actions, with auxiliary geometry, contact and teacher-phase supervision. The final rich1 fit initialized from the project's earlier tranche-1 checkpoint. It considered 1,070 native trials, trained on 495,343 motor ticks from 878 trials, validated on 54,962 ticks from 96 whole-seed held-out trials, and archived 96 failed pure teacher trials. The exported recurrent actor uses only the observation inputs described above.

| Source record in the SONIC token-control project | Identity |
|---|---|
| Demonstration collection implementation | `899880dcb8564376e90ae33f5b4e2b0d8f81c69e` |
| Model and training source | `2a50e7c163dbf31ae3b3f96355c779a5b1e5d9cc` |
| Native integer ONNX export implementation | `5404dbfa7fe6bfcaa2d8b03016733d9598a8cd85` |
| Frozen rich1, epoch 38, source and manifest | `8d5200e2b01ae87d05447332e700012842fda21b`, `pixel_policy_v4/weights/freeze.json` |
| Whole-seed split SHA-256 | `2e2562bbcbdc2290c6adc6874579c6925785b723781830c84751545602d32a6b` |
| Training checkpoint SHA-256 | `d9df4905599acb3cf058eea4145e98679a99ca2e0a5dbc6deab2f722791e4af4` |
| Floating-point export SHA-256 | `9d0c33ac4346d926f49fcb4bae6235d31c22428e17246592578662be1b7a0b8e` |
| Distributed integer ONNX SHA-256 | `2f4ada4c14f540ee1f99b0e875aa63ade0adbb87d63d710d5b15e4e8300d1c55` |

Git records Zi-ang Cao as the author of the data-collection, training, export and model-freeze commits listed above. The source snapshot has no project-wide license file or explicit license grant for this actor. Before submitting the PR, the contributor should confirm authority to redistribute the project-authored actor and reference code under the benchmark's Apache-2.0 terms. The separate NVIDIA Open Model License applies to the original low-level GEAR-SONIC decoder obtained by the environment. [Decoder and robot notices](../environment/NOTICE)

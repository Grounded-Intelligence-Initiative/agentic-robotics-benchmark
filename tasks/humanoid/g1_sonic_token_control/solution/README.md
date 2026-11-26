# Reference submissions

During ALE validation, `bash solve.sh` installs the frozen V4 visual actor into `$ALE_HOME/submission`. Set `SONIC_REFERENCE_POLICY=p1` when invoking the script to install the alternative planning controller. The installed entry point implements the same READY and JSONL protocol as a contestant submission.

**V4** evaluates an 825,756-parameter recurrent network on the three native 128×96 RGB images, 86 robot-state and previous-finger values, and its own recurrent memory. Each camera update produces five successive 64-bin body commands and five left/right hand pairs. `v4_actor.onnx` is the frozen integer-output graph from the public SONIC token-control workflow, SHA-256 `2f4ada4c14f540ee1f99b0e875aa63ade0adbb87d63d710d5b15e4e8300d1c55`. Its supervised training used 878 demonstration episodes and 96 validation episodes. The image runs it on one CPU thread.

**P1** fits the orange object and green pad in the three calibrated 480×360 images. A waypoint controller uses these optical estimates and observed hand motion to time approach, grip, carry and release. Its IK uses link inertias and joint ranges from the public G1 XML. Each 20 ms tick, it evaluates candidate body codes locally on the published 930-value history with the original NVIDIA decoder. It ranks legal integer proposals against desired body targets and returns one body row with both hand closures. The physics engine decodes and applies the chosen row separately. The planner uses the four-to-32-coordinate visual switching rule of the released P1 controller.

The runner stages policy state for each observation ID. On the next tick it recovers precisely the snapshot associated with `last_observation_id`. A second request for the same physical tick can therefore use a different observation without advancing the recurrent network or Cartesian planner twice.

The image supplies `/opt/sonic/assets` and `/opt/venv/bin/python3`. For a local checkout with equivalent packages, set `SONIC_ASSETS` to the directory containing `policy/model_decoder.onnx` and `menagerie_g1/g1_with_hands.xml`, `SONIC_PYTHON` to the interpreter, and put the task's `environment/` on `PYTHONPATH`.

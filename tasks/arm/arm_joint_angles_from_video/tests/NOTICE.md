# NOTICE - sources of this task's data (verify / maintainers only; never shown to the agent)

- **Recordings.** The videos and the logs are recordings of the NeuralActuator dataset (NAD) of the
  OpenManipulator-X: Z. Dou et al., "NeuralActuator: Neural Actuation Modeling for Robot Dynamics and External Force
  Perception", RSS 2026. Logs: GitHub `Frank-ZY-Dou/Dynamics-Modeling`, folders `NeuralActuator/data/force_unlabeled/`
  and `NeuralActuator/data/motor_condition/` (commit 870cba2, MIT licence). Videos: Hugging Face dataset
  `frankzydou/NAD`, `videos/omx/force_unlabeled/` and `videos/omx/motor_condition/` (MIT licence). Files are copied
  byte for byte; the logs of the test recordings are reduced to the columns `timestamp,pos1,pos2,pos3,pos4,aperture`.
- **Labelled recordings** (agent-visible): `force_unlabeled` `train/001` to `train/008` of `circular_cw`,
  `circular_ccw`, `backward_forward` and `go_up_and_stay_still`, renamed `<motion>_01` to `<motion>_08`.
- **Test recordings**: four recordings of each of `motor_condition/pick_place_object_200g`,
  `motor_condition/pick_place_object_200g_degrade`, `motor_condition/pick_place_empty_degrade`,
  `force_unlabeled/joint_sweep_motor12`, `force_unlabeled/joint_sweep_motor14` and
  `force_unlabeled/joint_sweep_motor15`, drawn and shuffled with `random.Random(20261118)` and renamed `T01` to `T24`;
  `ref/source.json` names the source of each. For every recording of these six motions the time offset between video
  and log was measured from the image motion and the logged joint motion (0.98 to 1.02 s), inside the 0.5 to 1.5 s
  that the grader searches.
- **The robot model** `robot/` (MJCF and meshes of the ROBOTIS OpenManipulator-X) is the copy shipped with
  NeuralActuator (`NeuralActuator/robot/`), Apache-2.0; its `LICENSE` file travels with it. The logged `aperture`
  (mm) is the position of the model's finger joints times 1000, the conversion NeuralActuator's own renderer uses.
- **Where they live:** the robot model, the 32 labelled recordings and the 24 test videos are in the benchmark's
  Hugging Face dataset under `tasks/arm/arm_joint_angles_from_video/payload/`, fetched at the commit pinned in
  `environment/assets.sha256` (docs/task-assets.md). The logs of the test recordings are only in `tests/ref/`.

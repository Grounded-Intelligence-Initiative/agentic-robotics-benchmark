# NOTICE - sources of this task's data (verify / maintainers only; never shown to the agent)

- **The robot model** `robot/` (agent-visible: `g1.xml`, `assets/`, `LICENSE`, `README.md`) is MuJoCo Menagerie's
  `unitree_g1` at commit `370c43965323c0944b433b202aac5058b5f907a9` (google-deepmind/mujoco_menagerie),
  BSD-3-Clause, copyright Unitree Robotics; its `LICENSE` file travels with it. `g1_keypoints.xml` (agent-visible in
  `tools/`, and here) is built from that `g1.xml` by the author's build script: the free joint, keyframe, sensors,
  actuators, geometry, meshes and materials removed, the pelvis fixed at (0, 0, 1.0) m, and the 21 scored points
  added as sites.
- **The recordings** are simulated and rendered by the author with MuJoCo 4.0.1 on the CPU (Mesa llvmpipe): the G1
  with its pelvis fixed on a stand, its own position servos tracking random smooth targets per limb (new targets
  every 0.7 to 1.9 s inside per-joint ranges, minimum-jerk paths, held at times; self-collisions on; time step
  1/480 s); the generator films each recording with two cameras on slow hand-held paths (front or sides, 2.1 to
  2.8 m away, vertical field of view 42 to 55 degrees, small shake), and the task gives the first camera's video
  alone (`video.mp4`); random floor, sky, lights and paint tone per recording; each 640 x 480 frame the mean of
  two renders 1/60 s apart plus Gaussian noise (sigma 2), written by OpenCV as MPEG-4 Part 2 at 30 frames per
  second; 300 frames per recording. The 30 recordings (6 labelled, 24 test) have seeds drawn from the master seed
  20261117; `ref/source.json` lists them with the camera and the sha256 of every video. The generator is not part of the task.
- **Where they live:** the robot model and the video of each of the 6 labelled and 24 test recordings, with the labelled
  recordings' joint angles, are in the benchmark's Hugging Face dataset under
  `tasks/humanoid/humanoid_joint_angles_from_video/payload/`, fetched at the commit pinned in
  `environment/assets.sha256` (docs/task-assets.md). The joint angles of the test recordings are only in `tests/ref/`
  and `solution/ref/`.

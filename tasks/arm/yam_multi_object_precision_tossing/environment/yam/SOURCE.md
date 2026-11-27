# YAM model

The YAM v1 MJCF with the linear 4310 gripper and D405 camera comes from the I2RT Robotics v1.3.1 release. The upstream file is `yam_linear_4310_d405.xml`, stored here as `yam.xml`. I2RT distributes the model under the included MIT license. [Upstream model and meshes](https://github.com/i2rt-robotics/i2rt/tree/1276f63d640eb45c226efd3dc08430b810372e94/i2rt/robot_models/arm/yam/v1)

The Docker build downloads the 13 referenced STL files at that commit and verifies each against `MESHES.sha256`. For local simulator development, run `sh yam/install_meshes.sh .` from the task's `environment/` directory. The generated `environment/assets/` is outside the task source digest.

The task adds workcell geometry and contact pads at runtime. The collision hulls in `finger-hulls.json` were derived from the upstream tip meshes with CoACD 1.0.14 (threshold 0.025, automatic preprocessing, 60 MCTS iterations).

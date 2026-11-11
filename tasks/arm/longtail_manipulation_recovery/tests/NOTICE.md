# NOTICE - sources of this task's assets (verify / maintainers only; never shown to the agent)

The kitchens, fixtures and objects the task's simulator loads come from the following sources. This notice is the
attribution their licences ask for.

- **RoboCasa code** (`robocasa/robocasa`, commit `456174f62b89b8fca99eaaf33949c29fec9cfc2a`): MIT. RoboCasa assets: CC BY 4.0
  (RoboCasa README). S. Nasiriany et al., RoboCasa, https://robocasa.ai.
- **Lightwheel fixtures and objects** (`fixtures/**` and `objects/lightwheel/**` in the asset tree): the Hugging Face dataset
  `nvidia/PhysicalAI-Robotics-Manipulation-Objects-Kitchen-MJCF` (revision `420a04af`), licence **CC-BY-4.0** as declared on its dataset card.
- **Objaverse objects** (`objects/objaverse/**`) and textures (`textures/**`, `generative_textures/**`): distributed with the RoboCasa asset set
  (Hugging Face dataset `robocasa/robocasa-assets`), licence **CC-BY-4.0** as stated by RoboCasa. The asset tree has no per-object
  licence files; the original per-object Objaverse terms are UNVERIFIED and this task relies on the distributors' statement, as the
  merged `robocasa_object_dynamics_from_video` task does.
- **aigen objects** (`objects/aigen_objs/**`): part of the same RoboCasa asset tree; only their folder names appear in this task as
  empty catalogue stubs (`object_instances.txt`), no model data unless a state uses one.
- **The robot**: Franka Panda arm, Omron mobile base and Panda gripper of robosuite (`ARISE-Initiative/robosuite`, commit
  `5ce6643f3092639d08f7b0f90ed1c6a84f50552c`), MIT, installed unmodified in the task image.
- **MuJoCo** 3.3.1: Apache-2.0.

Modifications: the agent's image and the verifier image carry only the files the scenes of this task load; every other object
instance folder is present as an empty `model.xml` (`make_stubs.py`), so that RoboCasa's seeded split of its catalogue selects the
same objects as with the full asset tree. No asset file is altered.

**The failure recordings** (actions in `tests/hidden/` and `solution/rescues/`, and `environment/setup/payload/practice_states/`)
are this task's own: rollouts of the frozen Xiaomi-Robotics-1 (XR-1, Apache-2.0) policy on RoboCasa, recorded on the
author's machine. XR-1 itself is not part of this task, is in neither image, and was only used offline to generate failures and
rescue action sequences.

Where the large files live: `grounded-intelligence-initiative/ale-robotics-assets`, under
`tasks/arm/longtail_manipulation_recovery/payload/` (`robocasa_assets/` for the agent image, `verifier_assets/` for the verifier
image), pinned by `environment/assets.sha256` and `tests/assets.sha256` (docs/task-assets.md).

# NOTICE - sources of this task's code and units (verify / maintainers only; never shown to the agent)

- **The robots**: MuJoCo Menagerie's Franka Emika Panda (folder `franka_emika_panda`, Apache-2.0), fetched at the
  commit pinned in `environment/panda_assets.sha256` and checked file by file; its visual meshes are converted to
  binary STL (`environment/obj2stl.py`, checked against `environment/panda_converted.sha256`). `srv_sim.py` replaces
  its position actuators by joint torques (the operational-space controller) and its gripper actuator by a force-capped
  servo on the fingers' tendon, and adds the tool site, the wrist sensor and the wrist camera.
- **The cell** (the server, its caddies, levers, buttons, fans, the tables, plates and bin) is built from MuJoCo
  primitives in `srv_sim.py`; the cameras are drawn by MuJoCo's own renderer (OSMesa on the CPU).
- **The units**: `units.json` holds the ten hidden units, each `srv_sim.draw_unit(seed)` for a 64-bit seed from
  `review/make_sets.py` (HIDDEN_SALT); `environment/commission_service/dev_units.json` holds the four development
  units (DEV_SALT). Development and hidden units come from the same ranges (`srv_sim.UNIT_RANGES`).
- **The hidden episodes** (`hidden/episodes.json`) were chosen from the candidates of `review/make_sets.py` by the
  rule in `hidden/episode_selection.json` (the record of every candidate run).
- **The code**: `srv_sim.py`, `srv_env.py` and `episode.py` are byte-identical in tests/ and in
  `environment/commission_service/` (the grader and the commissioning cell run the same simulator); `srv_grade.py`
  is the grader's episode loop; `robotics_grader/` is the repository's kit, byte-identical to
  `shared/robotics_grader/`.
- **The numerics**: `numerics_check.py` and `numerics.json` are the fingerprint the grader checks before grading.

# NOTICE - sources of this task's code and assets (verify / maintainers only; never shown to the agent)

- **The robot** is the Universal Robots UR20 of NVIDIA's public Isaac Sim 6.2 asset library
  (`Isaac/Robots/UniversalRobots/ur20/`, seven files pinned by sha256 in `environment/isaac_assets.sha256`; the
  folder carries its own licence file), referenced unmodified by the stage `packcell/scene.py` builds. The tool
  carrier, the nutrunner, the gripper and the cameras are this task's own geometry (primitives), added as two extra
  links of the robot's articulation.
- **The pack, the cell's furniture and the fasteners** are built from primitives and small generated meshes
  (`packcell/scene.py`); no third-party assets. The HVB-P series is fictional: its layout follows common plug-in
  hybrid packs (modules in a row, a service bay with a manual service disconnect and a battery management unit,
  busbars between terminal posts, hold-down brackets), its numbers are this task's.
- **`environment/fetch_isaac_assets.py`** is a byte copy of `tasks/swarm/hazard_bay_build/environment/fetch_isaac_assets.py`;
  `environment/isaac_assets.sha256` is that task's list cut down to the UR20's folder. The image's Python pins are
  that task's as well (`environment/requirements.txt`).
- **`nonet_exec.py`** is a byte copy of `tasks/arm/pantry_jar_refilling/tests/nonet_exec.py` (the seccomp launcher the
  controller is started through).
- **`robotics_grader/`** is the repository's kit, byte-identical to `shared/robotics_grader/`.
- **The cell's code** (`packcell/`) is byte-identical in `tests/packcell/` (the grader's, the authority) and
  `environment/commission_service/packcell/` (the commissioning service's); `review/check_copies.py` checks it and
  `--sync` restores it.
- **The documents** under `environment/setup/payload/given/` (the workshop manual with its drawings, the tool
  catalog, the cell's interface document, `cell.json`, `ur20.json`) are written by `review/make_documents.py` from the
  cell's own numbers; the manual's photographs by `review/make_photos.py` with the cell's own cameras on an example
  pack that is neither a graded nor a development pack.
- **The hidden set** (`hidden/packs.json`) and the development packs (`environment/commission_service/dev_packs.json`)
  were drawn by `review/select_set.py` under the rule in its docstring; `hidden/selection.json` is the record (the
  key, each pack's summary).

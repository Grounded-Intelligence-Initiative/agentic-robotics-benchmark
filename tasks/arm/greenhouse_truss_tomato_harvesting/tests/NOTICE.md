# NOTICE - sources of this task's code and assets (verify / maintainers only; never shown to the agent)

- **The robot** is the Universal Robots UR20 of NVIDIA's public Isaac Sim 6.2 asset library
  (`Isaac/Robots/UniversalRobots/ur20/`, seven files pinned by sha256 in `environment/isaac_assets.sha256`; the
  folder carries its own licence file), referenced unmodified by the stage `rowcell/scene.py` builds. The rail, the
  cart, the tool carrier with the cutter-gripper and the cameras are this task's own geometry (primitives), added as
  two extra links of the robot's articulation and one body of its own.
- **The plants, the greenhouse, the crates and the boards** are built from primitives and small generated meshes
  (`rowcell/scene.py`); no third-party assets. The cultivars "Rossa" and "Ambra" are fictional; their colour charts
  follow the usual six ripening stages of tomato (green, breaker, turning, pink, light red, red), their numbers are
  this task's.
- **The row markers** are the first six markers of OpenCV's `DICT_4X4_50` ArUco dictionary, drawn as squares
  (`rowcell/farm.py` `MARKER_BITS`).
- **`environment/fetch_isaac_assets.py`**, `environment/isaac_assets.sha256` and `environment/requirements.txt` are
  byte copies of `tasks/arm/battery_pack_disassembly_to_modules`' (the same Isaac Sim 6.2 / Isaac Lab 3.0 pins as
  `tasks/swarm/hazard_bay_build`), and `environment/Dockerfile` is that task's up to the cell's own account.
- **`nonet_exec.py`** is a byte copy of `tasks/arm/pantry_jar_refilling/tests/nonet_exec.py` (the seccomp launcher the
  controller is started through).
- **`robotics_grader/`** is the repository's kit, byte-identical to `shared/robotics_grader/`.
- **The cell's generic modules** (`rowcell/cellproc.py`, `rowcell/kin.py`, `rowcell/sensors.py`, `rowcell/server.py`,
  the structure of `rowcell/wire.py`, `verify.py`, `row_grade.py`, the commissioning service and its client) are taken
  from `tasks/arm/battery_pack_disassembly_to_modules` by the same author and adapted.
- **The cell's code** (`rowcell/`) is byte-identical in `tests/rowcell/` (the grader's, the authority) and
  `environment/commission_service/rowcell/` (the commissioning service's); `review/check_copies.py` checks it and
  `--sync` restores it.
- **The documents** under `environment/setup/payload/given/` (the growing protocol, the tool's data sheet, the cart's
  interface document, `site.json`, `ur20.json`) are written by `review/make_documents.py` from the cell's own numbers.
- **The hidden set** (`hidden/rows.json`) and the practice rows (`environment/commission_service/dev_rows.json`) were
  drawn by `review/select_set.py` under the rules in its docstring; `hidden/selection.json` is the record (the keys,
  each row's summary).

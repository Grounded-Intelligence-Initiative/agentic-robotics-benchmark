# Third-party material

* Franka Emika Panda model: MuJoCo Menagerie, folder `franka_emika_panda`, Apache License 2.0 (its LICENSE travels
  with the model into `~/given/robot/panda/` and `/opt/arb/given/robot/panda/`). Fetched at the commit pinned in
  `environment/panda_assets.sha256` and converted by `environment/obj2stl.py` (both byte copies of
  `tasks/arm/pump_skid_startup_and_operation/environment/`).
* `cellkit/` (lockdown, netaudit, nonet, runner, seeds, verify_freeze, wire): adapted from this repository's
  `tasks/arm/dual_arm_parcel_sorting_on_conveyor/tests/cellkit/`; `cellkit/pauser.py` follows
  `tasks/arm/kitchen_appliance_tending/tests/verify.py`'s Pauser. Same licence as the repository.
* `cellkit/`, `verify.py`'s structure, the commissioning service and the run script follow this author's
  `cnc_machine_tending_full_shift` task (same repository, same licence).
* Everything else in this task was written for it.

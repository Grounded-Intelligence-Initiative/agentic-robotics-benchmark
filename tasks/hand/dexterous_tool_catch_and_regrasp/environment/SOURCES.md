# Third-party material in this task

| what | where | source | licence |
|---|---|---|---|
| Wonik Allegro right hand (MJCF `right_hand.xml` and the 11 STL meshes it uses) | `environment/cell/wonik_allegro/`, byte copy in `tests/wonik_allegro/` | MuJoCo Menagerie, https://github.com/google-deepmind/mujoco_menagerie, folder `wonik_allegro`, commit eea4c7431b63da8278b0ff31638e0b0e941cec0e | BSD-2-Clause (Copyright (c) 2016, SimLab), `LICENSE` alongside |

Changes made by the cell at load time (the files are unmodified): the palm is attached to a servoed wrist, every hand
body is gravity-compensated, the 16 position actuators get stiffness 2 N m/rad and a 0.8 N m torque limit, and the
finger joints' damping is lowered from 0.1 to 0.03 N m s/rad (a fast catching hand).

The tools, the scene and the cell are this task's own.

# NOTICE (grader-side; never staged into the agent's home)

The robot is the MuJoCo Menagerie `unitree_g1` model (`g1_with_hands.xml`; BSD-3-Clause, its LICENSE ships inside the
folder), fetched by `environment/Dockerfile` at commit f3b6013629c7b94f8ee93fd977b161fa8350f9bc and pinned file by file
in `environment/g1_assets.sha256`. The cell replaces the model's floating base and actuators (base joints, servos,
gravity compensation) and uses the primitive colliders of Menagerie's `g1_mjlab.xml`; `tests/house_cell/robot.py`
builds it with MjSpec at cell start, as furniture_assembly_from_drawings's cell does (the same modifications; only the
collision bits differ, one robot here). The agent gets the unmodified folder in `~/given/robot/unitree_g1/`.

The room is furniture_assembly_from_drawings's room (the same walls, door, window, couch, plant and rug), and three
parts of that task's cell, made for that room by Xiatao Sun, are reused here:
  * the room's decor, what the cameras see of the room's shell and of the customer's own furniture (the oak floor, the
    plaster walls and ceiling, the skirting, the window's frame, glass and board, the door leaf, the pictures, the
    clock, the ceiling panels, the couch, the plants, the rug and the buildings across the street). As that task's
    NOTICE records, it is drawn from meshes and textures of two Unity Asset Store packs that the benchmark's owner
    licensed for redistribution, "Office and Police Station Pack" and "Office With Conference Room" (base maps
    downsized to 512 px), and its image light (`ibl.ktx`) is distilled from light probes of the same room path-traced
    with NVIDIA OVRTX and prefiltered with Filament's cmgen. The decor is not in git: its files are in the benchmark's
    Hugging Face dataset `grounded-intelligence-initiative/ale-robotics-assets` under that task's folder,
    `tasks/humanoid/furniture_assembly_from_drawings/payload/render/room_decor/`, fetched by this task's image build at
    the commit pinned in `environment/decor_assets.sha256` (a byte copy of that task's `environment/assets.sha256`) and
    checked file by file;
  * the cameras' renderer setup: `house_cell/render.py` (colour through MuJoCo's Filament module, Apache-2.0, part of
    the mujoco wheel, on Mesa's lavapipe; depth through OSMesa; the RealSense-like sensor model; that task's file with
    only its renderer thread renamed) and `house_cell/look.json` (the lights, the image light, the materials' terms and
    the colour gains; a byte copy);
  * the robot's legs as the cameras draw them while it moves (`house_cell/walk_core.py`, a byte copy: footsteps planned
    from the base motion, leg inverse kinematics on the G1 model's own leg chain; no motion-capture or third-party
    animation data).

The cleaning cell's own furniture and objects are procedural and original to this task: the kitchenette (cabinets,
sink, faucet, soap pump, sponge tray, hob, wall cabinets, splashback), the two flip-lid bins, the tool rail and its
clips, the dining table and chairs, the coffee table, the bookshelf, the trash, the dishes, the books, the sponge, the
broom and the lobby dustpan, their meshes and textures made in code (`house_cell/props.py`, `house_cell/objects.py`,
`house_cell/scene.py`) and their colliders primitive shapes; so are the cleaning processes (`house_cell/cleaning.py`:
the grease film, the sponge's soap, water and grease load, the crumbs), the layout draw and the scoring. No
third-party model, texture or dataset is part of the physics, the cleaning objects or the score.

# NOTICE (grader-side; never staged into the agent's home)

The two robots are the MuJoCo Menagerie `unitree_g1` model (`g1_with_hands.xml`; BSD-3-Clause, its LICENSE ships inside
the folder), fetched by `image/Dockerfile` at commit f3b6013629c7b94f8ee93fd977b161fa8350f9bc and pinned file by file in
`image/g1_assets.sha256`. The cell replaces the model's floating base and actuators (base joints, servos, gravity
compensation) and uses the primitive colliders of Menagerie's `g1_mjlab.xml`; `verify/furniture_cell/robot.py` builds
it with MjSpec at cell start. The agent gets the unmodified folder in `~/given/robot/unitree_g1/`.

The B930 bookcase, its drawing set (`verify/furniture_cell/drawings.py`, rendered with matplotlib), the room, the panel
cart and the kitting table are original to this task. No third-party furniture model, drawing or dataset is used.
Prior art the task was designed against (none of it reused): the Humanoid IKEA Assembly Challenge (IROS 2026, one G1,
IKEA UTTER table, learned policies), EmbodiedSWE-Bench (coding agents with privileged simulator state), Manual2Skill /
Manual2Skill++ (VLM-read IKEA manuals, fixed-base arms) and IKEA-Manual / IKEA Furniture Assembly environments.

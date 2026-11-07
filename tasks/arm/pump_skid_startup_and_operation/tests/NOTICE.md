# NOTICE (grader-side; never staged into the agent's home)

The robot is the MuJoCo Menagerie `franka_emika_panda` model (Apache-2.0, its LICENSE ships inside the folder), fetched
by `environment/Dockerfile` at commit f3b6013629c7b94f8ee93fd977b161fa8350f9bc and pinned file by file in
`environment/panda_assets.sha256`. At build time `environment/obj2stl.py` turns the model's visual OBJ meshes into
binary STL (same geometry) and rewrites the mesh references; the result is checked file by file against
`environment/panda_converted.sha256`. The cell builds the arm from that folder with MjSpec at cell start
(`tests/skidcell/physics.py`: the rail carriage and its lift, the grip weld). The agent gets the converted folder in
`~/given/panda/`.

Everything else is original to this task (`tests/skidcell/`, byte-identical to the agent's `~/given/lib/skidcell/`):
the skid and its devices (tanks, the pump, the duplex strainer, the handwheel / lever / needle valves, the control
station, the dial gauges, the flow meter, the level gauges, the sight glasses, the analyser), their textures (drawn
with Pillow's built-in font at run time), the lumped process model, the P&ID (`environment/setup/payload/given/pid.png`,
drawn by the task's own generator) and the operating procedure. The arm controller (`control.py`) and the camera rig
(`cameras.py`) come from this benchmark's shoe_lacing_and_boxing cell (itself ported from
robotic_arm_full_assembly_so101_panda), the verify kit (`cellkit/`, `robotics_grader/`) from the benchmark's shared
code. No third-party plant model, P&ID or dataset is used.

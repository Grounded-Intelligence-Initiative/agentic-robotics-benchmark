# Survey photos

Camera frames the robot took while it worked night shifts in two other pods of the same design (`pod_a`, `pod_b`):
other nights, other tickets and faults, each pod with its own installation tolerances (the racks within the drawing's
stated tolerances, the spares on the cart within 12 mm of their places) and each shift with its own camera white
balance and exposure. Neither pod is among the graded shifts; the faults these photos show are those pods' own.

The frames were taken in pairs, the head camera's and the working hand's wrist camera's, just before that hand
closed on a part, a lever, a handle or a tool (the job step is in the frame's `what`). This is a selection of 16
pairs: at the parts cart, a node's front (the fan wall, the tray's levers, the tray out and its cover open, a GPU
module's lift bar and screws), a rack's rear (its door, a supply and its cord, transceivers and MPO connectors, the
probe and the cleaner at work), and a leaf switch.

Each frame is three files: `<NN>_<what>_<camera>.json` (what `get_camera` returned for it, less the two images:
`camera`, `width`, `height`, `intrinsics`, `pose_in_room`, `sim_time`, `depth_unit_m`, `depth_invalid_value`; plus
`what` and `files`), `..._rgb.png` (the colour image) and `..._depth_mm.png` (16-bit depth in millimetres, 0 = no
reading). `dc_client.load_frame("<...>.json")` returns the reply `get_camera` gave for the frame, so
`dc_client.decode_frame()` reads it like a live one.

A frame's `pose_in_room` is the camera's pose as the robot knew it then (its localised base and its kinematics), in
that pod's hall frame: the same hall frame as the DCIM drawings, with that pod's own rack positions.

# Survey photos

Camera frames of two scenes of the customer's apartment (`scene_a`, `scene_b`), drawn by the rules of the cell
manual's section 2 like the graded scenes but not among them, taken by the cell's own cameras at the start of the
scene (nothing moved yet). Each frame is what `get_camera` returns, split into three files:

* `<frame>.json`: the reply's metadata (`camera`, `width`, `height`, `intrinsics`, `pose_in_room`, `depth_unit_m`,
  `depth_invalid_value`, `sim_time`) and the names of its two images;
* `<frame>_rgb.png`: the colour image (the reply's `rgb_png_b64`, decoded);
* `<frame>_depth_mm.png`: the 16-bit depth image in millimetres, 0 = no reading (the reply's
  `sensor_depth_u16_mm_b64`, decoded).

`house_client.client.load_frame("<frame>.json")` rebuilds the reply; `decode_frame` turns it into `(rgb, depth_m)`.

| frame | camera | the robot (base x, y, yaw; stand height; waist pitch) |
|---|---|---|
| `ceiling_kitchen`, `ceiling_dining`, `ceiling_living` | the fixed ceiling cameras | at its start pose |
| `head_start` | head | at its start pose, just inside the door; 0.793 m; 0 |
| `head_dining_south` | head | (2.85, 1.70, +pi/2) south of the dining table facing north; 0.793 m; 0.35 rad |
| `head_floor_south` | head | (2.85, 1.20, +pi/2) further south, facing north; 0.793 m; 0.10 rad |
| `head_coffee_west` | head | (3.95, 4.15, 0) west of the coffee table facing east; 0.793 m; 0.35 rad |
| `head_bins` | head | (1.00, 0.85, pi) facing the bins (west); 0.793 m; 0.20 rad |
| `head_sink` | head | (0.95, 2.65, pi) facing the sink (west); 0.793 m; 0.35 rad |

The head frames' `pose_in_room` is composed, as in every head frame, from the localised (noisy) base pose and the
exact kinematics.

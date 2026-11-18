# UAV Search and Rescue: Mezzanine Wing

Recover an orange **RESCUE MANNEQUIN** and sled from an indoor training
wing and deliver it to the blue staging pad. This is a simulated
civilian exercise. The doors, turns, and rescue bay vary. There is no
floor plan: explore with the UAV's two local RGB cameras and keep track
of the passages and landmarks you actually see.

- Find the **INSPECTION** slot off the staging strip. Align on the
  violet floor apron, settle at about **0.71 m** altitude, and fly
  through empty. Once clear, climb before leaving that pocket.
- Reserve time to search the mezzanine. Look into unfamiliar openings
  before committing. White aprons are ordinary doorways; amber aprons
  mark a change of floor with a raised sill. The lower floor is the
  launch datum; the mezzanine floor is **0.60 m higher**. A useful cruise
  altitude is **1.30 m above the local floor**. For an amber connection,
  climb to **1.90 m above launch** before entering, hold until fully
  clear on the other side, then adjust to the local floor. The walls
  are higher than the flight envelope.
- Search the marked **RESCUE BAY** rooms for the orange sled and its
  white grapple bar. Align in the down camera before descending. Settle
  about **0.43 m above the local floor**, close the gripper, then lift.
  If unsure of the grasp, look back at the pickup spot from a clear
  nearby hover: an ungrasped sled stays on the floor.
- Lifting the load opens the blue **CARGO** shutter and closes an
  interior safety shutter. A closed safety shutter has a red apron
  and a visible red panel. Recheck doors on the return and route around
  the obstruction. The cargo run connects to the search rooms at the
  far end and leads back down to staging. The sled cannot fit back
  through the inspection slot. Descend over the pad, settle at about
  0.43 m, open the gripper, and let the sled come to rest.

## Controls

`robot_act({"action":[right, forward, up, yaw_left, grip]})` takes five
numbers in `[-1,1]`. It increments the stabilized setpoint by up to
**0.32 m** on each horizontal axis, **0.19 m** vertically, and **30°**
in yaw per action. Positive yaw turns left; a zero holds that axis.
Horizontal increments are relative to the **heading setpoint at the
start of the action**. Body and camera motion have a short lag. Hold
still for two or three actions before precise alignment, grasp or
release. The UAV starts at 1.30 m with its heading facing the wing.
Altitude setpoints are limited to 0.39–2.09 m above launch.

`grip=+1` attempts a close, `-1` releases, and `0` leaves it alone.
It applies before movement. A close needs the grapple bar within about
13 cm of horizontal alignment and low translational and turn speed.
High or fast release, or striking a wall, sill, or shutter with the
airframe **or load**, ends the sortie. Each action advances 0.2 simulated
seconds; the allowance is **780 actions** (156 simulated seconds).
There is no reset. Observations do not advance time, but the live service
accepts at most 2,048 calls in total, including actions and observations.

## Cameras

`robot_observe()` and each action return two 256 × 256 RGB images:
`images[0]` looks straight down; `images[1]` looks forward and slightly
down by about 7°. Both rotate with the UAV. In the down view, right and up are body
right and forward. The down camera is 0.46 m forward and 0.145 m below
the UAV center, with a 94° perspective field of view. On a horizontal
surface `d` metres below that camera, a metre projects to
`(128 / tan(47°)) / d` pixels. The point under the gripper is centered
horizontally and 0.46 m down-image of the camera's ground center. The
floor modules are about 1.72 m across; apron paint is roughly 0.63 m
from a room center. These are measurement scales, not a map. Use the
forward view to inspect height, clearance and obstructions.

Only the images, `frame_id`, `done`, and `steps_remaining` are exposed.
There is no pose, depth, object list, map, or hidden identifier. Use the
tools or the public client at `/home/user/robot/client.py` with
`/opt/venv/bin/python3`. Scoring uses the protected live interaction.
Full credit is an intact, settled delivery after a physical lift; no
particular room, route, or map format is required.

# UAV Search and Rescue: Training Wing

Evacuate the orange **RESCUE MANNEQUIN** and its sled from a randomized
indoor training wing to the blue landing pad. This is a simulated civilian
exercise. The UAV starts over the pad with a fixed heading into the wing.

1. In the staging area, fly left to the white-framed floor plan. It is a
   scale model of **this** wing. North on the plan is forward in flight.
   White lines are walls, gaps are doorways, and green squares mark
   potential rescue bays. The blue square is the launch pad. Along the
   wall between the staging area and the rooms, the narrow amber gap is
   the inspection slot and the blue gap is the separate cargo shutter.
   Amber bars farther inside show raised doorway sills. The plan does not
   show which room has the sled.
2. Enter through the inspection slot **without a load**, centered and
   settled near **0.71 m** altitude. After clearing it, rise in the first
   room before moving through the raised doorways. About **1.34 m** clears
   those sills, including with the sled attached. Follow the corridor
   openings and inspect the bays. Walls are taller than the flight ceiling.
3. Center over the orange sled's white grapple bar, descend to about
   **0.43 m**, settle and close the gripper. Raise the sled. The lift
   interlock retracts the blue cargo shutter. **Do not retrace through
   the inspection slot:** the sled hangs too low to clear its sill
   while the UAV fits below its lintel. The cargo corridor runs along
   one side of the wing and connects to the rooms only at the far end.
   Use the plan to locate that connection. Back over the blue pad, descend to about 0.43 m,
   settle, open the gripper, and let the sled come to rest.

## Flight and cameras

- `robot_observe()` returns both onboard RGB frames without moving.
- `robot_act({"action":[dx,dy,dz,grip]})` takes four numbers in `[-1,1]`.
  The first three increment the position setpoint by
  `[0.32*dx, 0.32*dy, 0.19*dz]` metres. Positive X is right, positive Y is
  forward, and positive Z is up. The UAV follows the setpoint with a short
  lag. A zero holds the setpoint on that axis. Give the UAV two or three
  hold actions to settle before gripping or releasing.
- `grip=+1` tries to close, `-1` opens, and `0` leaves it alone. Grip
  applies before movement. A close needs the bar aligned within about
  13 cm and a slow hover. Never release while moving fast or from height.
- Each action consumes 0.2 simulated seconds. There are **300 actions**
  (60 simulated seconds), with no reset. A wall, shutter or sill strike
  by the UAV **or sled** ends the sortie.
- The pad is the origin; initial altitude is 1.24 m and the altitude
  setpoint range is 0.39–1.65 m. The plan is 1.72 m directly left of the
  pad. The model is five modules wide and five deep, including the
  staging strip. Centers of room modules are spaced 1.72 m in the
  wing and 0.29 m on the plan. Go door-center to door-center rather than
  cutting diagonally across room corners.
- Both images are 256 × 256. `images[0]` looks straight down with a 94°
  field of view; right is +X and up is +Y. It is **perspective**, so the
  footprint shrinks when descending. The camera is 0.20 m forward and
  0.145 m below the UAV center. On a surface `d` metres below the camera,
  1 metre spans `(128 / tan(47°)) / d` pixels. The point directly under
  the gripper is centered horizontally and 0.20 m down-image of the
  camera center. Align in the higher view before descending.
  `images[1]` is the fixed-heading forward view. Besides the frames,
  the interface returns only `frame_id`, `done`, and `steps_remaining`.

Use the tools directly or use `/home/user/robot/client.py` with
`/opt/venv/bin/python3`. There is no pose telemetry, map file, seed,
or target answer. The protected live interaction determines the score.
An intact, stationary sled on the pad after a physical lift earns full
credit; finding it or returning with it short of delivery earns partial
credit.

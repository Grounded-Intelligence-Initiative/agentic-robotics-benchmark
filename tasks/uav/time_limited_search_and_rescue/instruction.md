# UAV Search and Rescue

Evacuate the orange **RESCUE MANNEQUIN** and its sled from Sector B to the
blue evacuation pad in Sector A. This is a simulated civilian training
exercise. The drone starts hovering over the pad in A, facing B.

1. Fly forward to survey the four-tile heat forecast board in A. The only
   **teal-green tile** marks the cool opening; count its dark dots (one to four).
   The board is behind you by the time you reach the crossing. Remember it.
2. Cross into B through the opening with that many **white dots** on its
   threshold plate. Red-and-yellow baffles and all other openings are
   no-fly. The openings do not show their live heat locally. Use the center
   of the cool opening, below the overhead beam. Use the same opening on
   the way back. A heat, baffle, or cabinet strike ends the sortie.
3. Search B with the onboard cameras. The target is an orange rectangular
   sled with a yellow dummy and a white grapple bar. Hover over the bar,
   descend, settle, then close the gripper. Raise the sled clear of the
   ground and bring it back above the blue pad. Descend and settle again,
   open the gripper just above the pad, and let the sled come to rest.
   Releasing from height or while moving fast is unsafe.

## Flight and gripper

- `robot_observe()` returns the current two RGB images without advancing time.
- `robot_act({"action":[dx,dy,dz,grip]})` takes four numbers in `[-1,1]`.
  The first three increment the stabilized **position setpoint** by
  `[0.32*dx, 0.32*dy, 0.19*dz]` metres. Real motion follows with a short
  lag. Positive X is right, positive Y is forward, positive Z is up; the
  heading is fixed. Zero on an axis holds that setpoint. Allow a couple of
  hold actions for settling before grasp or release.
- Gripper: `+1` attempts to close on the bar, `-1` opens/releases,
  `0` leaves it alone. The command applies **before** the action's motion.
  Pickup needs alignment within about 13 cm and a slow hover; a UAV center
  altitude near 0.43 m works for both pickup and set-down. The drone starts
  at 1.12 m. For transit, around 1.15 m provides ground clearance. Over
  the crossing the drone center must stay below 1.43 m.
- Each action is 0.2 simulated seconds; there are 132 actions (26.4 seconds).
  There is no reset, phase-advance command, or separate coordinates report.

## Cameras and reference points

Both cameras move with the UAV and return 256 × 256 RGB. `images[0]` is
straight down, with right = +X, up = +Y, and a fixed 3.5 m square footprint.
The point under the gripper is approximately pixel **(128, 142)**; align the
grapple bar and the pad center there. `images[1]` faces forward and tilts
toward the ground. No world map, position vector, target identifier, heat
state, or grip state is returned. The other fields are `frame_id`, `done`,
and `steps_remaining`.

The following fixed calibration uses the pad center as the horizontal origin:
the board is about 2.0 m ahead of the pad; the crossing is 5.25 m ahead.
Opening centers, left to right, are at X = -2.7, -0.9, +0.9, +2.7 m.
Align before crossing: there is about ±0.44 m of lateral center clearance.
Sector B continues beyond the crossing; target placement varies. The
setpoint is bounded to X = ±3.39 m, Y = -0.45 to +10.75 m relative to
the pad, and Z = 0.39 to 1.88 m above ground.

Use the tools directly or the Python client at `/home/user/robot/client.py`
with `/opt/venv/bin/python3`. The protected live interaction is what counts.
Partial progress is credited for surveying, entering B safely, getting a
view of the target, lifting it, and bringing it back into A. Full credit
requires an intact, stationary delivery on the pad.

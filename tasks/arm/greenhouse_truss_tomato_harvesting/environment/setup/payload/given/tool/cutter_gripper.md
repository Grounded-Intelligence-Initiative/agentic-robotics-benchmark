# Cutter-gripper: data sheet

The harvesting tool on the robot's flange. Two jaws close a mouth that is open along the tool axis. A blade at one
side of the mouth cuts the stalk that lies across the mouth; a clamp pad beside the blade holds the stalk on the pad's
side of the cut. With it the robot cuts a truss off its plant and carries it by what is left of its peduncle.

## 1. Geometry

In the flange frame (origin on the flange face, z out of the flange: the tool axis), in millimetres. The tool frame
has the same axes and its origin at the tool centre point. Body and neck are boxes centred on the tool axis.

|  |  |
|---|---|
| mass | 1.9 kg |
| adapter | a disc of 90 diameter, 12 thick, on the flange |
| body | 60 x 60 (x by y), from z = 12 to 100 |
| neck | 26 x 44 (x by y), from z = 100 to 340 |
| jaws | two bars, each from x = -13 to 9, 8 thick (along y), from z = 340 to 370; they close along y, symmetrically about the tool axis |
| mouth | the gap between the jaws: 0 to 16 wide (along y), from z = 353 (the throat) to 367 (the tips) |
| tool centre point | the mouth's centre: on the tool axis at z = 360, the throat 7 behind it and the tips 7 ahead of it |
| blade | in the plane x = 5 |
| clamp pad | in the plane x = -8: on the -x side of the blade, 13 from it |
| wrist camera | its housing a box of 36 x 44 x 40 (x, y, z) centred at [-79, 0, 186]: on the -x side of the tool |
| jaw speed | 40 mm/s of opening (2 mm per control tick; 0.4 s from full opening to closed) |
| closing force | settable from 0 to 600 N |
| readings | opening (m), force (N): section 4 |

Body, neck, jaws, adapter and camera housing are what collides with plants, trusses, crates and cart. The jaws' bodies
stand at the full opening of 16 mm at all times; the jaws a camera frame shows follow the commanded opening. Through
the wrist camera's nominal mounting the tool centre point lies 0.167 m in front of the camera, nearer than the 0.2 m
from which its depth sensor reads (`../site/site_interface.md`, section 6).

## 2. Commanding it

An action's `gripper` sets the jaws' target opening and their closing force (`../site/site_interface.md`, section 3).
The jaws move toward the target at the jaw speed, tick by tick, and both values stay in force until another `gripper`
replaces them.

## 3. How the jaws meet a stalk

A stalk, here, is what a plant has that is long, straight and round: a piece of a main stem, a peduncle, a piece of a
rachis, a leaf's petiole (growing protocol, section 1). Fruits and leaf blades are not stalks: blade and pad do
nothing to them, and the tool's bodies push and press them like any body.

- **In the mouth.** A stalk is in the mouth at a plane x = X when its axis crosses that plane between the jaws'
  colliders (within half the open mouth's width of the tool axis, along y) and between the throat and the tips (from 7
  mm behind to 7 mm ahead of the tool centre point, along z). The stalk's axis decides, not its thickness.
- **Closing.** On their way to the target opening the jaws meet, at the blade's plane, the thickest uncut stalk that
  is in the mouth there, when the opening has come down to its diameter. With a closing force of that stalk's cutting
  force or more, the blade cuts it through at once and the jaws go on, to the next stalk or to the target. With less,
  the jaws stall on it: the opening stays at, and reads, the stalk's diameter, and the force reads the commanded
  force; a closing force raised to the stalk's cutting force later cuts it then. Jaws that open do nothing to a stalk,
  and a stalk that comes into a mouth that is already closed is neither cut nor held.
- **Closing jaws push a stalk to the middle.** A stalk that is in the mouth, between the jaws at the opening they have
  at that moment, at any tick of a closing stays caught for the rest of that closing, for as long as it is in the
  mouth: the jaws meet it wherever between them it was. (The jaws' bodies stand at the open mouth, section 1: the
  stalk itself is not moved, and what the pad holds is held where it is.) Jaws that open let everything go.
- **The cutting force** of a stalk is the row's toughness times the stalk's cross-section (pi d^2 / 4 with its
  diameter d in mm). The toughness is the row's own (the season, the irrigation): between 2.4 and 5.2 N/mm^2, the same
  for every stalk of a row section, and not on the wire.
- **The pad** acts at its own plane, 13 mm to the -x side of the blade. It takes hold of every stalk other than a main
  stem that is in the mouth at the pad's plane once the jaws have come down to that stalk's diameter (to within 0.5
  mm), and keeps hold of it for as long as the opening is not more than 0.5 mm above that diameter and the closing
  force is 3 N or more. A stalk the pad holds is fixed to the tool and moves with it; one that is still on its plant
  ties the tool to the plant. Opening the jaws, or a closing force below 3 N, lets go.
- **A cut peduncle frees its truss.** The truss stays in the tool when the pad holds the peduncle on the truss's side
  of the cut: the peduncle crosses the pad's plane inside the mouth, farther from the main stem than it crosses the
  blade's plane. When it crosses the pad's plane nearer to the main stem than the cut, or not inside the mouth (beyond
  the throat or the tips), nothing holds the truss and it falls.
- **Crushing.** The pad presses what it holds with the closing force. A peduncle the pad holds while the closing force
  is above 1.4 times that peduncle's own cutting force is crushed, at the cut or at any time after it (growing
  protocol, section 5).

What the blade can meet, over both cultivars and all rows, and what a cut does:

| stalk | diameter (mm) | cutting force (N) | when it is cut |
|---|---|---|---|
| peduncle, Rossa | 4.4 to 6.4 | 36 to 167 | frees its truss: held by the pad, or fallen (above) |
| peduncle, Ambra | 3.6 to 5.2 | 24 to 110 | frees its truss: held by the pad, or fallen (above) |
| rachis | 0.9 times its truss's peduncle (3.24 to 5.76) | 20 to 135 | parts the truss: it is waste |
| petiole of a leaf | 4.4 to 6.4 | 36 to 167 | drops the leaf; nothing else follows |
| main stem | 11 to 13.6 | 228 to 755 | the plant is lost: none of its trusses counts, those already in a crate included |

A stalk that has been cut once is not cut again. Nothing on the wire says which stalk the jaws met or what was cut.

## 4. Readings

- **opening** (`gripper.opening`, m): the gap between the jaws, with a noise of 0.08 mm (one standard deviation) on
  every reading. `trace.opening_mm` is the same reading once per control tick, in millimetres.
- **force** (`gripper.force`, N): the force the jaws press with. It is the commanded closing force while the jaws are
  stalled on a stalk, or closed completely at a target opening of 0, and 0 at every other time (jaws that move freely
  or stand open); a reading that is not 0 carries a noise of 0.5 N (one standard deviation). `trace.grip_force` is the
  same reading once per control tick.

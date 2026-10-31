# Site brief: the aerial workcell

A small workcell stands on the floor of a warehouse hall. Two flying robots with arms work there: they pick objects
off two tables and put them into a bin with a hinged lid. This brief, the photographs in `~/given/photos/`
(captions in `photos.json`) and what you can find in the offline library `/opt/assets/isaac/` are all there is to
know about it. Nobody can tell you more.

## Where things are

The world frame is fixed by the calibration mat on the floor: a flat board of 7 × 5 black and white squares of
100 mm (700 × 500 mm), long side along the world x axis, its centre at x = 0.00 m, y = -0.60 m, on the floor (z = 0).
+z points up, +y points from the robots' stands toward the bin. Everything else you take from the photographs.

The cell holds:

- **Table A** and **Table B**: two plain tables, A the taller one (on the left in `P01_overview_front.jpg`).
- **The bin**: an open box with a hinged lid, the hinge along the lid's back edge, the handle at its front edge.
  Beside the bin stands **the button post**: a square post with a push button on top.
- **Six objects** on the tables. A kitchen scale gave: cracker box 411 g, mustard bottle 603 g, sugar box 514 g,
  tomato soup can 349 g, power drill 895 g, mug 118 g.
- **Two robots**, each on its launch stand, each with an arm and one camera. Their nameplates give take-off masses
  (everything they carry included) of 3.83 kg for the larger and 1.87 kg for the smaller one.
  `P19_camera_large.jpg` and `P20_camera_small.jpg` were taken by the robots' own cameras where they stand now.
- The warehouse around the cell, which you may model or leave out.

## How the bin works

The lid is held shut by its own weight; a light spring in its gas strut takes part of it, so that lifting the lid
at its handle (pulling at right angles to the lid) takes about 3.2 N. Released before it has opened about 28°, the
lid falls back shut. Lifted past about 28°, the strut takes over and carries it on, by itself, to its stop at about
105° (past vertical), where it stays.

From there, pulled back at the handle by about 10° (toward closed), the strut's stay lets go and the lid closes by
itself, slowly: it takes several seconds and never moves faster than about 30° per second.

The button on the post travels 10 mm and takes a firm press (about 5 N). While the lid is shut, a press makes the
bin's motor open the lid all the way within a few seconds; the motor keeps it open for 20 s, counted from the
press, then closes it. Every press while the lid is shut does this again.

Objects dropped into the open bin stay in it.

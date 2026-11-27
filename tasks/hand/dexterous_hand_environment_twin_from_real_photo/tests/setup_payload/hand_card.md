# Hand card

The team's hand is a four-digit robot hand: three fingers (the index, middle and ring finger) and a thumb, carried by
the team's arm. In a pinch the index finger and the thumb touch the thing; in a hook the index finger alone does. This
card gives the sizes that the pick planner and the executor use. Lengths are in millimetres, times in seconds, forces
in newtons.

## The tips (measured on the hand's own parts)

- Finger tip, the last segment of a finger with its tactile pad: 24.7 mm along the closing direction, 19.8 mm wide and 49.4 mm long; the pad's touch field 15 mm wide and 21 mm tall.
- Thumb tip, with its tactile pad: 22.9 mm along the closing direction, 35.7 mm wide and 54.8 mm long; the pad's touch field 26 mm wide and 28 mm tall.
- These tip boxes are used by the lift trial only (below). The planner and the executor use the pad boxes of the next section instead, and the finger's tip box is not the finger pad's box (20 x 25 x 21 mm).

## The pick instrument

The planner and the executor do not move a model of the hand. They move these volumes, and only these; the twin
contract gives the rules in full.

- Finger pad: a box 20 mm deep along the closing direction, 25 mm wide and 21 mm tall, with a column of the same section 60 mm tall on top of it (the rest of the finger).
- Thumb pad: a box 23 mm deep along the closing direction, 36 mm wide and 28 mm tall, with its own column 60 mm tall on top of it.
- Hook: the index finger held straight, as a bar 20 mm wide and 25 mm tall that reaches 60 mm behind the rim's material around the opening (the hand's side) and 15 mm beyond it.
- Start gap between a pad and the thing: 10 mm (6 mm for a pad inside a vessel). Over-travel past the surface: 15 mm. Widest grip: 80 mm.
- Clearance above the board: a `body` or `top` pinch pad's lowest face is planned at least 2.5 mm above the board's top, the hook bar's at least 1 mm; a `rim` pad is placed from the rim's top with no such clearance (the twin contract, section 7).
- Margins: 4 mm to every other thing, 1 mm to the thing picked. Touch tolerance: 0.5 mm.
- Placement error of the hand on its arm: up to 1.5 mm along each axis. Every pick is executed at the nominal place and at the eight corners of that cube.
- Only these volumes are judged: the assessment sweeps no approach path and adds no other part of the hand, and it does not use the optional file of further hand volumes that `check_twin.py` reads when one is given (none is given).

## Lifting

The lift trial (`hand_instrument.lift`) is offered for your own rehearsal on your own twin (`check_twin.py --rehearse --lift`, the twin contract's section 9); in the assessment, every scored request, body pinches included, is judged geometrically (no lift trial is run).

The trial adds to the twin's model a finger tip and a thumb tip of the sizes above, each centred on its planned pad's centre and turned as the pad, so a tip's closing face is off the planned pad's face by half the difference of the two thicknesses (nearer the thing when the tip is the thicker). A tip is its touch field (1 mm thick, at the front of the tip along the closing direction), a link behind it of the rest of the tip's thickness, running up from the field's bottom for the tip's length, and above the link a column of the tip's section up to 60 mm above the field's top (masses 0.001, 0.02 and 0.001 kg); it slides along its closing direction over its plan's travel (the pad's start gap, or the inside start gap, plus the over-travel), with damping 30 and armature 0.001, driven by a motor of 0 to 20 N, and its contacts take the trial's sliding friction (in `check_twin.py`, 0.3), torsional 0.005 and rolling 0.0001 at contact priority 1. Both tips hang from a grip body, a sphere of 3 mm and 0.1 kg that touches nothing, on a vertical slide joint (armature 1000000) at the placement's offset, still until the pads have closed, then rising and holding; the pick sheet's held things are welded to the world.

The pads rest open for 0.5 s, close with 5 N over 0.7 s, then the hand rises 30 mm in 1 s and holds for 1 s. The thing counts as lifted when the state stays finite, neither tip touches anything at the start or at the end of the rest (contacts are counted at those two instants only), neither pad has reached the end of its travel, and the thing's centre of mass has risen at least 27 mm from the end of the rest to the end of the hold.

No given program reads these entries of `instrument_config.json`: `format`, `units`, the `approach` block, and `pad_travel_mm` and `held_things` of the `lift` block (the trial uses each plan's own travel and always welds the held things).

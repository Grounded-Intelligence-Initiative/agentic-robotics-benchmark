# Stacked Training Ward: Author Notes

Parent: `6787b2c0e274724f9250b806b642288991f6cd15`. The former
y-divided mezzanine and internal safety shutter are replaced, not extended.
The natural RGB navigation, side fitting survey, geometry-matched tool depot,
physical lift, cargo shutter, oriented 1.80 m litter transfer and whole-board
settled delivery remain.

## Construction

Each storey has fifteen rooms on the same five-by-three footprint. A
floor-to-ceiling partition divides the lower ward. Each side has an enclosed
hoist alcove off its rear landing. Only the upper ward connects the two sides,
at a variable crossing away from the wells. Each component is built connected,
with extra doors for alternatives. The target is chosen from two treatment
bay coordinates, each with a distinct room on **both** storeys. All are on
the remote side. Even a downstairs target requires an ascent, an upper
crossing and a descent before the fitting can be surveyed. The cargo leaf
and its fixed header seal the otherwise shorter approach until lift.

The upper floor is 2.60 m above launch. The same opaque 0.22 m slab boxes
support the free board, occlude RGB and participate in 5 ms contact checks.
The two 2.34 m square apertures are actual cutouts. Staging and loading have
a low roof, the ward has an upper roof, and the well surrounds are solid.
Normal top support at pickup or rest is not a slab strike. The long board
must fit inside a hole for the whole crossing; there is no required storey
sequence in the delivery grade.

## Reference and ablation

The pilot discovers the two staging doors from RGB. It registers successive
floor patches, looks at doorway wear plates, and can re-observe local wall
geometry or a stored camera view. An amber threshold leads physically into
a well. The pilot centers on the pair of visible back-wall guide channels,
flies the storey interval vertically, then exits horizontally into the
other landing. It tracks the level from its own flights, not from the server.
Its room search remembers each doorway actually crossed and a reversible
trail. That trail takes it to the depot and back to the surveyed bay.
Loaded egress reuses doors actually crossed on search: take a known route
to the observed cargo door, or approach a previously observed untried door.

The XY control uses the same pilot and safe breadcrumb trail. The single
ablation is the address of the high-level crossed-door memory in
`RoomMemory.key`: XY instead of XYZ. It still views the current room for
the litter on every arrival, reads the real doors, flies vertical actions in
both directions and uses identical geometric flight, tools and landing
control. If a downstairs door has already been crossed, the control can
mistake its upstairs counterpart for an exhausted branch. Exact action
prefixes and evidence of real same-XY, opposite-storey skipped doors are
checked outside the policy. This is an ablation of search memory, not of
altitude control or local visual registration.

## Reproduction

`tests/review/check_stack.py SEED OUTPUT --paired` records both outcomes,
raw action tapes, RGB images, stages and memory-alias evidence. Default
author runs render needed camera pairs at their original action timestep;
`--full` renders both cameras on every action. `check_stack_geometry.py`
also checks generator connectivity, casts rays from matched physical
poses, measures board clearance at different yaw angles and lateral
offsets, and flies into a closed ceiling. Its staged camera pairs are
explicitly separate from live search imagery. Scalar grader metrics and
private structured diagnostics are separate; the public observation
contains only RGB, frame number, remaining actions and done.

The solver wall remains 30 minutes, with 4,400 actions and 9,000 total
service calls for the physical vertical flights and return journeys.

## Measured controls

The first frozen search found and lifted the litter on all eight original
sites, but twice exhausted a 3,600-action allowance after discarding its
survey map for egress. Those are failed controls, not discarded seeds.
After the known-route repair, the RGB reference delivered on the original
8/8 and another 8/8 independently precommitted sites. Each panel includes
all target-storey / flange / trim combinations and both mirrored layouts.
The frozen reference used 1,998-3,296 actions. Actual launch-to-first-target
approach distance varied from 61 to 148 m. The recorded loaded envelope
stayed at least 0.195 m inside the rim across 14 loaded shaft transits.

The matched XY pilot delivered 0/8 on each panel. All 16 actually ascended,
descended and exhausted their projected search without finding the sled;
none crashed or stopped on a missing storey label. The independent referee
recorded 51 observations of a physically open doorway being skipped using a
crossing remembered at the same XY on the other storey.

The stacked geometry checks cover 512 procedural layouts (shortest inlet
to target: 6-18 abstract edges; a well edge requires three flight legs).
The 25-yaw sweep of the actual board left at least 0.243 m in a centered
well, or 0.123 m with simultaneous 0.12 m X and Y offsets. These are
measurements, not success thresholds. The memory control tests this
pilot's choice of a search branch, not the impossibility of solving the
ward with a different representation.

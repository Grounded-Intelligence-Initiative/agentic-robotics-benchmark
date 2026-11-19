# Warehouse handoff: author notes

Direct parent: `2b2f4b15478d7f7fd40d53902dc2fd140012a69d`. One change
of hypothesis: the accepting receiver is a condition of this incident,
not a destination specified by the mission. The loaded return, search,
fitting, cargo interlock and original budgets remain. There is no time
penalty, scan requirement, early-memory check or required order of doors.

## Public scope

The mission specifies the object and the physical handoff; the equipment
card describes both onboard cameras, action frame and clipping, gripper
fits and capture bands, the structural clearances, cargo interlock,
release limits and settled footprint/contact requirements. It explains
the dispatch arrow and crossed shutter. It does not give the target
storey, rack position, site map, receiving side, stage sequence or route.
The building and its visible equipment constrain what is possible.

Protected simulation: each site has two receiving rooms with separate,
oppositely staggered approaches from the turning pocket. Each has an
actual deck and roller housing. The receiving leaf closes the entire
clear aperture at the inactive station, and its deck is stowed upright.
The active station has an open throat and a horizontal supporting mat.
Changing the incident bit reverses the staging arrow, changes which
shutter is deployed and which deck is unfolded, and nothing about the
casualty, tool seats, flight trim or layout. The sign is ordinary
large-scale wayfinding; the door cross is visible without reading text
or distinguishing colors. Both jambs retain identical toe markings.

The original score weights and grader are unchanged. To score a handoff,
the released board must have a real MuJoCo contact with one of the two
mat geoms, pass the center and projected footprint tests on that same
mat, and settle. The check does not read the incident bit or consult a
table of permitted destinations. The folded deck cannot support the
board horizontally or contain its footprint. The shutter contact uses
the rendered solid, not a wrong-station penalty.
Inherited partial weights are .12 for entry, .08 upstairs, .10 location,
.12 grasp, .15 lift, .20 cargo return and up to .06 for room coverage.
Physical delivery is 1.0 regardless of the chosen route.

## Reference assumptions

The reference uses only the public observation and its own commanded
motion. It recognizes the dispatch arrow from the shaft and two vanes.
In the pocket it inspects a receiving throat from a safe distance: four
diagonal arms identify the crossed leaf, or the blue target and green
deck must be visible through the opening. A missing cross alone is
insufficient. If the first approach is closed or uncertain it can
return to the pocket center and inspect the other. No particular hint
or memory strategy is scored.

This is a warehouse-specific pilot, not a general building navigator:

- It assumes an axis-aligned five-column staging strip with launch near
  its middle, initial heading toward the wing, adjacent rooms on a
  roughly 2.70 m grid, and one-storey hoist transits. It scans staging for
  the actual rack heads and inspection/cargo thresholds. It maps actual
  doorway crossings in XYZ and registers texture and walls on revisits.
- It recognizes the documented fixture types from their geometry/materials;
  uses a nominal 0.71 m slot transit, 1.30 m cruise, calibrated flange
  profile at about 0.55 m standoff, and a cross-boom pickup. The latter
  keeps the board from covering its down camera.
- It expects the transfer entry across the cargo staging cell and the
  vestibule dispatch display to be visible facing the wing at launch.
  The arrow is optional. It estimates lane centers and entry plane from
  both real jambs, uses the nominal 2.30 m pocket half-depth, searches the
  two lateral approaches, and expects the deployed deck within roughly
  1.70 m beyond its doorway. The blue target supplies the final position.
  Neither the incident side nor the map of a particular seed is supplied.

## Author screen

`tests/review/check_receiving_geometry.py OUTPUT` freezes two new base
sites before any mission, one per mirror with opposite trims and fittings.
Each is run with both incident states. `--geometry` takes independent
onboard camera pairs, measures actual solid distances/rays and tests mat
support at both footprints. The supported and displaced board probes
are geometric fixtures, not scored flights.

`tests/review/check_receiving.py OUTPUT INDEX` flies the image-only
mission. At loaded cargo egress the referee snapshots the actual physics,
RGB frame and policy memory for matched continuations. Fixed-side commits
to the positive-X throat with all vision, load alignment, pickup, release
and flight feedback intact. Local-only deliberately discards the early
hint and on negative-X incidents must safely inspect the closed station
before finding the open one. Action tapes and both digests are checked.
RGB pairs are rendered at the exact physics steps where used. Formal
native controls and blind runs are separate.

Frozen screen: four clean status-aware deliveries in 1,833-1,869 actions;
two of four for fixed +X (both failures were board-on-shutter contact);
two of two safe closed-then-open replans without the early hint. The
minimum final footprint margin was .1268 m. At each portal the aligned
board had .33 m clearance when open and intersected the shutter when
closed. The shape of the dispatch arrow and the shutter cross survived
grayscale conversion on all four worlds. This tests current status, not
the necessity of early memory or increased difficulty for a blind solver.

Use `check_receiving_geometry.py OUTPUT --layout-audit` to verify paired
site descriptors and `check_receiving.py OUTPUT verify` to recheck the
saved stage and action receipts without rerunning any simulation.

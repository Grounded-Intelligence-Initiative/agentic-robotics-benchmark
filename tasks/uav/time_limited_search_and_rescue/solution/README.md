# Warehouse Litter Return: author notes

Parent: `c3eec60c8cde19bed87f6b69dd21a2dac4cddf1e`. This version
tests whether a return route proven with the aircraft is feasible for the
attached load. The casualty is always in one of the upper treatment bays.
Both complete floors connect their respective landings. The service well
is on a direct approach from inspection; a separate freight well connects
elsewhere. The ground cross-passage makes freight-only round trips legal.
There is no check for which shaft was used, when it was seen, or the
number of storey changes. The existing collet fit, pickup, cargo shutter,
two perpendicular loading throats and settled whole-board delivery remain.

## Geometry

Both deck openings have four cast-iron members. Their visible inner faces
provide contact geometry for the UAV, head and board at 5 ms intervals.
The service bore is 1.08 m square: even its 1.527 m diagonal is shorter
than the rigid 1.80 m board. Translating or yawing cannot fix that.
The freight bore is 2.34 m square: the greatest half-extent of the
0.44 by 1.80 m board at any yaw is `hypot(.22,.90) = .927 m`, leaving
.243 m per side when centered. No hidden size threshold grades the flight.
The back-wall guides are the same at both. The iron frame itself changes
shape; no color or sign codes the well class.

## Reference

The pilot receives just the public RGB envelope. It maps traversed
doorways in XYZ, estimates translation from local floor texture and
re-registers rooms. During empty ascent it centers using the guide rails
and measures the actual inner edges from the upper down view. Pixel span
at known hover height gives metres. It remembers the measurement by
landing. Above an unfamiliar well it can inspect without descending.
The separate mounting survey and trip to the depot still happen in flight.

With the load the pilot follows its proven approach back to its transfer
landing. It checks the remembered aperture against the board length before
descending. Infeasible vertical edges are excluded both from explored
paths and from unexplored-door choices. It explores for another route on
the same storey and continues through cargo to receiving. This reference
is deliberately not a shortest-path planner. With the board mounted
lengthwise the down view can be completely occluded; this pilot surveys
the transfer during the empty trip and picks up across the camera boom.

## Reproduction

In the cached rescue CPU image, with the task at `/task` and a fresh
private output directory at `/out`:

```sh
cd /task
python tests/review/check_freight.py /out
python tests/review/check_freight.py /out --index 0   # repeat for indices 1..7
python tests/review/check_freight_geometry.py /out/geometry
python tests/review/check_freight_geometry.py /out/air-headings --air-headings
python tests/review/check_stack_geometry.py 43862997158914082 68955551560688757
python tests/review/check_freight_alternate.py 43862997158914082 /out/freight-first
```

The first command freezes eight deterministic sites, balanced across
trim, mirror and fitting. No controller is run during selection.
`check_freight.py` flies once through the real lift, then runs two live
continuations from the identical integrator/weld state, camera frame and
RGB-derived memory. Replay changes only the loaded planner's clearance
test: it retains the XYZ route, fitting choice, heading, visual centering,
servo, and delivery controllers. It does not intentionally collide or
stop on an artificial check. The referee saves action tapes, measured
apertures, stage frames, both cameras, physical crossings and named
contacts. On-demand rendering generates every consumed image pair at its
original physics step. These author checks are not native ALE results.

The dynamic geometry control places the aircraft and correctly mounted
head above each bore, optionally grips the real board, and descends using
normal physics. Photo fixtures are distinct from live mission frames.
The topology/ray control also checks 512 layouts with each vertical link
removed in turn, an opaque slab between floors and a real ceiling impact.
The budgets remain 4,400 physical actions, 9,000 requests and 30 minutes.

## Author controls

The load-aware controller delivered cleanly on 8/8 precommitted sites in
1,541-3,329 actions. The smallest measured in-flight board-to-freight-rim
clearance was .213 m, and the final settled board had at least .119 m to
the mat edge. All eight matched replays lifted the same litter before
the board struck the actual service frame. All six equipped-empty service
descents and all six loaded freight descents succeeded in the dynamic
fixtures; all six loaded service descents contacted the frame. Twelve more
paired-condition descents with airframe headings of -45, 30 and 90 degrees,
an off-axis attachment and 8 cm offsets on both axes gave the same split.

From inspection it took three graph edges to reach the upper service landing
versus four to nine to reach the upper freight landing on these sites.
The shortest empty route all the way to the sled was sometimes tied. This
is evidence against simply reversing the proven empty path, not a claim
that size inference is the only way to solve the mission.

On the first frozen site, a separate camera-only control bypassed the first
observed hoist on empty search. It used the other (freight) well for all
four crossings and replayed its route successfully. Both cameras rendered
on every action: 3,210 actions, 330 s on one CPU, excluding native service
overhead. The normal shortcut replay on this same site hit the service rim.

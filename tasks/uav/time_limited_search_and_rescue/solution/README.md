# Medical Litter Transfer: Author Notes

Parent: `f1662ffce9c4d3d2e9513eaf91566e996f227a4c`. The fitting survey,
separate depot, physical pickup, mezzanine, shutter change and cargo
extraction precede the new transfer. The cargo shutter returns the
aircraft to staging, not directly to the final mat. The receiving room
is enclosed; its only opening is into the turning pocket.

## Geometry and Sensing

The registered structural solids are rendered MuJoCo boxes. At each
5 ms physics step, AABBs select potential pairs; signed MuJoCo geom
distance tests the actual aircraft meshes, mounted jaws and litter parts
against those same solids. A broad-phase overlap alone cannot fail a
flight. The litter is a rigid 1.80 by 0.44 m board; the weld preserves
its actual pickup transform. The floor/deck supports the free litter.
Paint is not a wall. No waypoint, yaw, portal-read event or clearance
margin is required by scoring: intact, settled delivery is still the
only full-credit outcome. At rest, the board's projected footprint must
also lie entirely over the actual receiving mat.

Both loading throats have a 1.10 m clear gap and 0.56 m depth. Offset
and handedness vary. The pocket has space to turn before translating
to the next centerline. The straight board has 0.33 m per side when
centered end-on; it is 0.70 m too long to pass broadside. Wing stations
are farther apart to allow yaw maneuvers away from the door frames.
The solver wall limit remains 30 minutes; the longer flight has 1,600
actions and 3,300 total requests.

The RGB reference identifies the flange from the side, revisits with
the right head, then positions the camera over both violet end grips.
Their projection estimates the board axis and center before grasp.
It chooses a roughly transverse attachment to keep the nose-mounted
down camera clear, retains the measured relative axis, and can fly a
straight segment independently of aircraft yaw. It recenters from the
station dots, reads the two real jamb lines at each throat, and looks
to both sides of the pocket as needed. No simulator object, pose,
layout variant or seed enters the policy.

## Controls

`tests/review/check_litter.py SEED OUTPUT` runs the full RGB reference
and a paired controller in fresh worlds. They are identical through
both the actual lift and the second jamb observation. The latter still
reads the aperture and translates to its measured lane; it keeps the
entry load axis instead of turning the board across the pocket. The
referee verifies identical actions through that observation, records
the actual impact geoms and aircraft clearance, and rotates **only the
board** in a separate copy of the terminal state to measure the
same-XYZ counterfactual. The regular mode renders both PNGs on every
action. Optional `--quick` author smokes defer rendering unread frames;
they do not skip physics steps.

`tests/review/check_loading.py SEED ...` separately tests centered,
offset, perturbed and broadside poses on both sets of actual jambs.
It then flies an empty UAV through both apertures with the fixed-heading
policy. These fixtures start at loading and are not counted as rescues.
Run either script inside the rescue image with the task mounted at
`/task:ro`, no network or GPU, and a writable evidence mount for the
paired controller. The author scripts do not perform native ALE
validation or call a participant model.

## Measured Outcomes

Eight paired author sites (four of each fitting; both exit sides and
both signs of each offset) delivered with RGB and hit the exit jamb with
the board in all eight fixed-heading flights. All 16 fully rendered
action tapes matched their on-demand repeats with the final mat-support
check. Four additional unfiltered sites delivered. Across the 12 rescues,
the reference used 577-1,336 actions of 1,600, left at least 0.157 m
between the loaded litter and the jambs, and settled with at least
0.063 m of board-to-mat edge margin. The RGB axis estimate was within
0.07 degrees of the measured pickup transform; the jamb offsets were
within 0.039 m of the geometry on these sites.

At the eight failed impacts, the aircraft and mounted head were at
least 0.157 m clear. A yaw-only correction of the litter at that same
XYZ left at least 0.147 m of board clearance. The empty fixed-heading
aircraft passed both throats on all eight sites. In the isolated
geometry check, even simultaneous 10-degree heading and 0.07 m lateral
errors left 0.204 m clearance. These are measurements, not scoring
thresholds or claims about untested worlds.

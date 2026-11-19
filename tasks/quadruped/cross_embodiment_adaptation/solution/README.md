# Author solution (not part of the participant sandbox)

ALE's `solve.sh` installs a public-observation reference into the normal
submission path. It surveys with the UAV, projects the captured RGB onto
floor and rack-top grids, and measures the gaps. It searches the passage
sequences that leave enough lateral clearance for the Go1. At the same
time it charts identical safety caps from overhead RGB. After deployment
it pans the mast through eight views while stationary, matches unlabeled
cap constellations and fits a planar transform using the measured rays.
It also tracks the two identical service covers across UAV frames.
Parallax yields each surface height and thus the position of the low duct
and raised deck. It applies the registration estimate to local odometry,
uses the open bays to align the loaded body, and changes the leg extension
near the measured duct. All motion is via native twelve-servo commands.

The controls in `tests/review/v8_controls.py` use the same survey, route,
registration, startup and gait. They fix only the leg-extension set point
to the ordinary, crouched or intermediate value. The flat counterfactual removes the two
fittings, not the load or the safety rules. No policy gets a seed, true
fixture class or pose. Only the author harness reads the private ledger.
See `tests/review/README.md` for the comparison and its limits.

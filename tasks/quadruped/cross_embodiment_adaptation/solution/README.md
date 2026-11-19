# Author solution (not part of the participant sandbox)

ALE's `solve.sh` installs a public-observation reference into the normal
submission path. It surveys with the UAV, projects the captured RGB onto
floor and rack-top grids, and measures the gaps. It searches the passage
sequences that leave enough lateral clearance for the Go1. At the same
time it charts identical safety caps from overhead RGB. After deployment
it pans the mast through eight views while stationary, matches unlabeled
cap constellations and fits a planar transform using the measured rays.
It applies this estimate to launch-frame odometry and walks the route
using the unchanged analytical gait and native twelve-servo commands.

The matched identity-transform control in `tests/review/v7_controls.py`
gets the same survey, route, pan images, odometry and joint controller but
uses local coordinates as if they were the map. Neither controller receives
a seed, candidate pose list or privileged inputs. The control harness reads
the private ledger only after deployment to measure pose error and outcome.
See `tests/review/README.md` for the comparison and its limits.

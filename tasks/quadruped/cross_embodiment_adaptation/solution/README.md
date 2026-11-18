# Author solution (not part of the participant sandbox)

ALE's `solve.sh` installs a public-observation reference into the normal
submission path. It surveys with the UAV, projects the captured RGB onto
floor and barrier-height grids, picks a physically open aisle and finds
the dock. It then walks using an analytical diagonal gait, inverse
kinematics and the public tracking pose. Its only target command is the
documented list of twelve servo angles and a hold time.

The privileged feasibility check in `tests/review/v5_controls.py` instead
supplies the true open aisle, dock and exact pose to the same low-level
controller. No privileged socket method, credentials or model policy are
shipped to the participant. See `tests/review/README.md` for evidence and
limitations.

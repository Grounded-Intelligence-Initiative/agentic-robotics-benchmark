# Author solution (not part of the participant sandbox)

ALE's `solve.sh` installs a public-observation reference into the normal
submission path. It surveys with the UAV, projects the captured RGB onto
floor and rack-top grids, and measures the gaps. It searches the passage
sequences that leave enough lateral clearance for the Go1, then walks
the shortest feasible sequence using an analytical diagonal gait,
inverse kinematics and public tracking. Its only target command is the
documented list of twelve servo angles and a hold time.

The matched controls in `tests/review/v6_controls.py` remove the minimum
width check at either the first or last rack, keeping the same RGB map,
route objective and joint controller. No privileged socket methods or
site data are used by any arm. See `tests/review/README.md` for evidence.

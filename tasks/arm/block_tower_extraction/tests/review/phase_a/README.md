# Phase A prototype (block_tower_extraction)

Physics/feasibility prototype, not the task package yet. Run with the Menagerie Panda directory in
`BT_PANDA_DIR` (MuJoCo 3.16, numpy, Pillow for snap.py).

- `tower.py`: tower generator (hidden per-block heights/friction/mass/offsets), MuJoCo tower + turntable model.
- `cell.py`: Panda + deployable pusher rod + wrist F/T site + staging post; Cartesian moves (IK), gripper, turntable.
- `oracle.py`: privileged reference (live contact loads + true poses); `python oracle.py <seed>`.
- `baseline.py`: non-privileged sanity baselines (`blind`, `probe` with F_ABORT); `BASE_TRUE_POSE=1` gives the
  probe baseline perfect block poses (isolates the tightness question).
- `t_drift.py`: 60 s standing test; `t_push.py`: push force tests on loose/tight blocks.
- Baseline env switches: `F_ABORT` (probe abort force, N), `BASE_TRUE_POSE=1` (perfect block poses),
  `BASE_FT_RELAX=1` (non-privileged pull: nudge the grip height from the wrist F/T vertical force only).
- `run_all.sh`: oracle + baselines on seeds 1-4 (logs/v5).

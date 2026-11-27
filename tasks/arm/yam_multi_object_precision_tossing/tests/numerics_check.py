"""Compare a deterministic two-arm simulation and controller math with the recorded build."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys


def fingerprint():
    import mujoco
    import numpy as np
    from workcell import ArenaRound, check_physics

    arena = ArenaRound(31)
    try:
        h = hashlib.sha256()
        for _ in range(160):
            for side, arm in arena.arms.items():
                desired = np.array([.075 if side == "left" else -.075, 1.21, 1.24, .04, 0., 0.])
                arena.drive(arm, desired, .021)
            previous_time = float(arena.data.time)
            mujoco.mj_step(arena.model, arena.data)
            check_physics(arena.data, previous_time)
            h.update(arena.data.qpos.astype("<f8").tobytes())
    finally:
        arena.close()
    data = np.arange(117, dtype=float).reshape(9, 13) / 57
    values = np.sin(data) + np.cos(data**1.25)
    solved = np.linalg.solve(np.eye(9) + values @ values.T, np.arctan2(data, 1.5))
    return {"physics": h.hexdigest(),
            "controller": hashlib.sha256(solved.astype("<f8").tobytes()).hexdigest()}


def verify_numerics():
    here = Path(__file__).resolve()
    env = {"PATH": "/opt/venv/bin:/usr/local/bin:/usr/bin:/bin", "HOME": "/tmp", "LANG": "C.UTF-8",
           "PYTHONPATH": "/opt/station:/opt/physics/lib/python3.12/site-packages"}
    actual = json.loads(subprocess.check_output([sys.executable, str(here)], env=env, timeout=120))
    expected = json.loads(here.with_name("numerics.json").read_text())
    if actual != expected:
        raise RuntimeError(f"YAM reproducibility check failed: {actual}, expected {expected}")


if __name__ == "__main__":
    print(json.dumps(fingerprint(), indent=2))

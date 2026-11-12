"""Trusted setup: start the commissioning service; only the client, not the world, is made public."""
import os
from pathlib import Path
import shutil
import subprocess
import time


def main():
    stage = Path(__file__).resolve().parent
    private = Path("/var/lib/visual_robotics")
    if private.exists():
        raise RuntimeError("refusing to reset an existing episode")
    private.mkdir(mode=0o700)
    shutil.copytree(stage/"runtime", private/"runtime")
    public = Path("/opt/visual_robotics/client")
    public.parent.mkdir(parents=True, exist_ok=True)
    shutil.copytree(stage/"payload", public, dirs_exist_ok=True)
    # Public clients are immutable; the agent can copy them but cannot change MCP execution.
    for path in public.rglob("*"):
        path.chmod(0o755 if path.is_dir() else 0o644)
    public.chmod(0o755)
    home = Path("/home/user/robot")
    shutil.copytree(public, home, dirs_exist_ok=True)
    for path in [home, *home.rglob("*")]:
        os.chown(path, 1000, 1000)
    with (private/"service.log").open("wb") as log:
        process = subprocess.Popen(
            ["/opt/venv/bin/python3", "-E", "-s", str(private/"runtime"/"service.py"), "--mode", "commissioning",
             "--directory", str(private/"commissioning")],
            stdout=log, stderr=log, start_new_session=True, cwd=private/"runtime",
            env={"PATH": "/opt/venv/bin:/usr/bin:/bin", "HOME": "/root", "LANG": "C.UTF-8",
                 "MUJOCO_GL": "osmesa", "LIBGL_ALWAYS_SOFTWARE": "1", "LP_NUM_THREADS": "2",
                 "OPENBLAS_NUM_THREADS": "1", "OMP_NUM_THREADS": "1", "PYTHONNOUSERSITE": "1"},
        )
    for _ in range(600):
        if process.poll() is not None:
            raise RuntimeError((private/"service.log").read_text())
        if Path("/run/visual_robotics/world.sock").exists():
            return
        time.sleep(.1)
    process.terminate()
    raise RuntimeError("robot service did not become ready")


if __name__ == "__main__":
    main()

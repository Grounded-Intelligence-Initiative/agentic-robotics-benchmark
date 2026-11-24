"""Trusted setup: stage the agent's files, write the session's time stamp and start the practice cell.

The cell's code (runtime/) is copied to a folder only root can read and run from there as a service; the agent gets
its client (payload/) and the service's socket. The graded episodes are not in this sandbox at all (they live in the
verifier's tests/ stage).
"""
import os
from pathlib import Path
import shutil
import subprocess
import time

AGENT_UID = 1000


def main():
    stage = Path(__file__).resolve().parent
    stage.chmod(0o700)
    private = Path("/var/lib/cable_cell")
    if private.exists():
        raise RuntimeError("refusing to reset an existing session")
    private.mkdir(mode=0o700)
    shutil.copytree(stage / "runtime", private / "runtime")
    home = Path("/home/user")
    for entry in (stage / "payload").iterdir():
        dest = home / entry.name
        if entry.is_dir():
            shutil.copytree(entry, dest, dirs_exist_ok=True)
        else:
            shutil.copyfile(entry, dest)
        for path in [dest, *(dest.rglob("*") if dest.is_dir() else [])]:
            os.chown(path, AGENT_UID, AGENT_UID)
    # session_time.sh stays root's, so the agent cannot change the limit it reports
    script = home / "session_time.sh"
    os.chown(script, 0, 0)
    script.chmod(0o755)
    submission = home / "submission"
    submission.mkdir(exist_ok=True)
    os.chown(submission, AGENT_UID, AGENT_UID)
    # the session's time stamp, in a folder that is root's: the agent's account can read it and cannot replace it
    session = Path("/run/session")
    session.mkdir(mode=0o755, exist_ok=True)
    os.chown(session, 0, 0)
    session.chmod(0o755)
    stamp = session / "start"
    stamp.write_text(str(int(time.time())) + "\n")
    os.chown(stamp, 0, 0)
    stamp.chmod(0o444)
    with (private / "service.log").open("wb") as log:
        process = subprocess.Popen(
            ["/opt/venv/bin/python3", "-E", "-s", str(private / "runtime" / "service.py")],
            stdout=log, stderr=log, start_new_session=True, cwd=private / "runtime",
            env={"PATH": "/opt/venv/bin:/usr/bin:/bin", "HOME": "/root", "LANG": "C.UTF-8", "MUJOCO_GL": "osmesa",
                 "LIBGL_ALWAYS_SOFTWARE": "1", "OMP_NUM_THREADS": "1", "OPENBLAS_NUM_THREADS": "1",
                 "PYTHONNOUSERSITE": "1"},
        )
    for _ in range(600):
        if process.poll() is not None:
            raise RuntimeError((private / "service.log").read_text())
        if Path("/run/cable_cell/cell.sock").exists():
            return
        time.sleep(0.1)
    process.terminate()
    raise RuntimeError("the practice cell did not become ready")


if __name__ == "__main__":
    main()

"""Trusted setup: draw one scene of the fixed hidden set, start the root-owned world service, publish only the client.

The hidden set (scenes.json, the same file as tests/hidden_scenes.json) stays root-only: it is read here, inside the
root-only setup stage, and the drawn seed reaches the service through a root-only file, never a command line (other
accounts can read a process's command line). The world implementation is copied into the root-only ledger directory.
"""
import json
import os
from pathlib import Path
import secrets
import shutil
import subprocess
import time


def main():
    stage = Path(__file__).resolve().parent
    private = Path("/var/lib/visual_robotics")
    if private.exists():
        raise RuntimeError("refusing to reset an existing episode")
    private.mkdir(mode=0o700)
    os.chmod(private, 0o700)
    shutil.copytree(stage/"runtime", private/"runtime")
    hidden = json.loads((stage/"scenes.json").read_text())
    seeds = hidden["seeds"]
    index = secrets.randbelow(len(seeds))
    scene = private/"scene.json"
    scene.write_text(json.dumps({"seed": int(seeds[index]), "scene_index": index}))
    os.chmod(scene, 0o600)
    public = Path("/opt/visual_robotics/client")
    public.parent.mkdir(parents=True, exist_ok=True)
    shutil.copytree(stage/"payload", public, dirs_exist_ok=True)
    # Public clients are immutable; the agent can copy them but cannot change how the MCP server runs.
    for path in public.rglob("*"):
        path.chmod(0o755 if path.is_dir() else 0o644)
    public.chmod(0o755)
    shutil.copytree(public, Path("/home/user/robot"), dirs_exist_ok=True)
    with (private/"service.log").open("wb") as log:
        process = subprocess.Popen(
            ["/opt/venv/bin/python3", str(private/"runtime"/"service.py"), "--scene-file", str(scene)],
            stdout=log, stderr=log, start_new_session=True, cwd=private/"runtime",
            env={**os.environ, "PYTHONPATH": "", "PYTHONNOUSERSITE": "1"},
        )
    for _ in range(600):   # the world, its meshes and the CPU renderer start in a few seconds; allow a busy host
        if process.poll() is not None:
            raise RuntimeError((private/"service.log").read_text())
        if Path("/run/visual_robotics/world.sock").exists():
            return
        time.sleep(.1)
    process.terminate()
    raise RuntimeError("world service did not become ready")


if __name__ == "__main__":
    main()

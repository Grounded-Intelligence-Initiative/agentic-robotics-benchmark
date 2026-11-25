"""Maintainers' tool (nothing runs it at verify): regenerate the givens that derive from the cell's code and pin them.

  1. environment/setup/payload/given/cell/cell_config.json  <- towercell.config.public_config()
  2. environment/setup/payload/given/lib/towercell/{__init__,client}.py <- tests/towercell/ (byte copies)
  2b. environment/commission_service/towercell/ and cellkit/{__init__,wire}.py <- tests/ (byte copies)
  3. tests/setup_payload/                                    <- environment/setup/payload/ (byte copy, the lint checks)
  4. tests/given_hashes.json: sha256 of every file of the payload and of the robot model (given/robot/panda, from the
     image's /opt/task-assets/panda; pass its folder with --panda), at their paths under the agent home / /opt/arb.
Usage: python3 tests/review/make_given.py --panda <converted panda folder>
"""
import argparse
import hashlib
import json
import os
import shutil
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
TESTS = os.path.dirname(HERE)
TASK = os.path.dirname(TESTS)
PAYLOAD = os.path.join(TASK, "environment", "setup", "payload")
sys.path.insert(0, TESTS)

from towercell.config import public_config  # noqa: E402


def sha(p):
    return hashlib.sha256(open(p, "rb").read()).hexdigest()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--panda", required=True)
    a = ap.parse_args()
    with open(os.path.join(PAYLOAD, "given", "cell", "cell_config.json"), "w") as f:
        json.dump(public_config(), f, indent=1, sort_keys=True)
        f.write("\n")
    os.makedirs(os.path.join(PAYLOAD, "given", "lib", "towercell"), exist_ok=True)
    os.makedirs(os.path.join(PAYLOAD, "given", "cell"), exist_ok=True)
    for fn in ("__init__.py", "client.py"):
        shutil.copyfile(os.path.join(TESTS, "towercell", fn), os.path.join(PAYLOAD, "given", "lib", "towercell", fn))
    svc = os.path.join(TASK, "environment", "commission_service")
    for sub in ("towercell",):
        shutil.rmtree(os.path.join(svc, sub), ignore_errors=True)
        shutil.copytree(os.path.join(TESTS, sub), os.path.join(svc, sub), ignore=shutil.ignore_patterns("__pycache__"))
    os.makedirs(os.path.join(svc, "cellkit"), exist_ok=True)
    for fn in ("__init__.py", "wire.py"):
        shutil.copyfile(os.path.join(TESTS, "cellkit", fn), os.path.join(svc, "cellkit", fn))
    dst = os.path.join(TESTS, "setup_payload")
    shutil.rmtree(dst, ignore_errors=True)
    shutil.copytree(PAYLOAD, dst, ignore=shutil.ignore_patterns("__pycache__"))
    files = {}
    for root, dirs, fns in os.walk(PAYLOAD):
        dirs[:] = sorted(d for d in dirs if d != "__pycache__")
        for fn in sorted(fns):
            p = os.path.join(root, fn)
            files[os.path.relpath(p, PAYLOAD)] = sha(p)
    for root, dirs, fns in os.walk(a.panda):
        dirs.sort()
        for fn in sorted(fns):
            p = os.path.join(root, fn)
            files[os.path.join("given", "robot", "panda", os.path.relpath(p, a.panda))] = sha(p)
    with open(os.path.join(TESTS, "given_hashes.json"), "w") as f:
        json.dump({"note": "sha256 of the givens at their paths under the agent home and /opt/arb (tests/review/make_given.py)",
                   "files": files}, f, indent=1, sort_keys=True)
        f.write("\n")
    print("pinned %d files" % len(files))


if __name__ == "__main__":
    main()

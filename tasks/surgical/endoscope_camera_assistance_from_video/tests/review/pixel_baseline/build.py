"""Maintainers' tool: put the pixel baseline together as a submission folder.

    python3 tests/review/pixel_baseline/build.py --model model.pt --out DIR

DIR then holds what an agent's /home/user/submission would: run.sh, the controller, the net and its weights, the
public holder model, and a copy of the reference's control law (which here stands for a control law the agent would
write itself; it gets no token and no privileged observation).
"""

import argparse
import os
import shutil

HERE = os.path.dirname(os.path.abspath(__file__))
TASK = os.path.dirname(os.path.dirname(os.path.dirname(HERE)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", required=True)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    for f in ("controller.py", "net.py"):
        shutil.copy(os.path.join(HERE, f), a.out)
    shutil.copy(a.model, os.path.join(a.out, "model.pt"))
    shutil.copy(os.path.join(TASK, "environment", "setup", "payload", "given", "holder_model.py"), a.out)
    shutil.copy(os.path.join(TASK, "solution", "reference", "controller.py"), os.path.join(a.out, "reference_controller.py"))
    with open(os.path.join(a.out, "run.sh"), "w") as f:
        f.write("#!/usr/bin/env bash\nexec /opt/venv/bin/python3 -B -s controller.py\n")
    print("pixel baseline submission in", a.out)


if __name__ == "__main__":
    main()

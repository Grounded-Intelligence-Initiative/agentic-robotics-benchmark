#!/usr/bin/env bash
# Run as root in a fresh task image each time, with the repo mounted at /root/source.
set -euo pipefail
mode=${1:?pass oracle or untouched}
case "$mode" in oracle|untouched) ;; *) exit 2 ;; esac
install -d -m 700 /opt/ale
cp -a /root/source/environment/setup /opt/ale/
bash /opt/ale/setup/setup.sh

runuser -u user -- sh -ec '
    test ! -r /var/lib/visual_robotics/state.json
    test ! -r /var/lib/visual_robotics/runtime/rescue.py
    test ! -r /opt/ale/setup/runtime/rescue_scene.py
    test ! -e /opt/ale/tests
    test ! -e /home/user/.ale-solution
'
if [[ "$mode" == oracle ]]; then
    cp -a /root/source/solution /home/user/.ale-solution
    chown -R user:user /home/user/.ale-solution
    runuser -u user -- bash /home/user/.ale-solution/solve.sh
    if [[ -d /evidence ]]; then
        cp /home/user/submission/final-camera-*.png /home/user/submission/reference-route.json /evidence/
    fi
fi

cp -a /root/source/tests /opt/ale/
export ROBOTICS_GRADER_VERDICT_PATH=/tmp/rescue-verdict.json
bash /opt/ale/tests/test.sh
python3 - "$mode" <<'PY'
import json
import os
from pathlib import Path
import shutil
import sys

with open(os.environ["ROBOTICS_GRADER_VERDICT_PATH"]) as stream:
    verdict = json.load(stream)
expected = int(sys.argv[1] == "oracle")
assert verdict["rewards"]["reward"] == expected, verdict
assert verdict["metrics"]["success"] == expected, verdict
assert (verdict["metrics"]["trusted_requests"] > 0) == bool(expected), verdict
print(json.dumps({"staged_mode": sys.argv[1], **verdict}))
if expected and Path("/evidence").is_dir():
    private = Path("/var/lib/visual_robotics")
    ledger = json.loads((private/"state.json").read_text())
    grasp = next(e["frame_id"] for e in ledger["events"]
                 if e["method"] == "act" and e["params"]["action"][4] == 1.)
    for label, step in (("pre-grasp", grasp-1), ("loaded-flight", grasp+5)):
        for camera in range(2):
            shutil.copy2(private/"frames"/f"{step:04d}-{camera}.png",
                         f"/evidence/{label}-{camera}.png")
PY

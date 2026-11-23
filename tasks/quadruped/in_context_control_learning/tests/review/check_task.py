"""Switchback author checks; run in the pinned task image on CPU."""
import json
from pathlib import Path
import subprocess
import sys

TASK = Path(__file__).resolve().parents[2]


def main():
    hidden = json.loads((TASK/"tests/hidden_scenes.json").read_text())["seeds"]
    candidates = json.loads((TASK/"tests/review/hidden_set/candidates.json").read_text())["seeds"]
    assert len(hidden) == len(set(hidden)) == 24
    assert hidden == candidates[:24], "Do not reselect seeds on candidate outcomes"
    legacy = TASK/"tests/review/legacy/round2-8b7195d/tests"
    for path in ("hidden_scenes.json", "review/hidden_set/candidates.json", "review/hidden_set/selection.jsonl"):
        assert (TASK/"tests"/path).read_bytes() == (legacy/path).read_bytes(), path
    assert (TASK/"environment/setup/scenes.json").read_bytes() == (TASK/"tests/hidden_scenes.json").read_bytes()
    for name in ("world.py", "lane.py", "service.py"):
        assert (TASK/"environment/setup/runtime"/name).read_bytes() == (TASK/"tests"/name).read_bytes()
    assert (TASK/"solution/client.py").read_bytes() == (TASK/"environment/setup/payload/client.py").read_bytes()
    for script in ("check_support.py","check_service_aisle.py","check_commissioning.py","check_route.py","check_joints.py","check_security.py"):
        subprocess.run([sys.executable, str(Path(__file__).with_name(script))], check=True)
    print("all split deck, service aisle, commissioning and switchback checks passed", flush=True)


if __name__ == "__main__":
    main()

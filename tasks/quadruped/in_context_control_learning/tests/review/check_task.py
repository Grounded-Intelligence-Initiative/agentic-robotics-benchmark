"""Current v3 checks; run in the task image on the remote CPU pool."""
import json
from pathlib import Path
import subprocess
import sys

TASK = Path(__file__).resolve().parents[2]


def main():
    hidden = json.loads((TASK/"tests/hidden_scenes.json").read_text())["seeds"]
    candidates = json.loads((TASK/"tests/review/hidden_set/candidates.json").read_text())["seeds"]
    assert len(hidden) == len(set(hidden)) == 24
    assert hidden == candidates[:24], "Do not reselect seeds on v3 outcomes"
    for script in ("check_v3.py", "check_security.py", "check_reference.py"):
        subprocess.run([sys.executable, str(Path(__file__).with_name(script))], check=True)
    print("all v3 checks passed", flush=True)


if __name__ == "__main__":
    main()

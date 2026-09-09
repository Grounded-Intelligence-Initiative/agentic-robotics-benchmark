"""A few hand-worked examples, not a dynamics implementation or hidden test suite."""
import json
import math
import os
from pathlib import Path
import queue
import subprocess
import threading


def main():
    root = Path(os.environ.get("SUBMISSION_DIR", "/home/user/submission"))
    proc = subprocess.Popen(["bash", "run.sh"], cwd=root, stdin=subprocess.PIPE,
                            stdout=subprocess.PIPE, text=True, bufsize=1)
    lines = queue.Queue()

    def pump():
        for line in proc.stdout:
            lines.put(line)
        lines.put(None)

    threading.Thread(target=pump, daemon=True).start()

    def read(seconds=1):
        value = lines.get(timeout=seconds)
        if value is None:
            raise RuntimeError("unexpected EOF")
        return value

    def request(req, expected):
        proc.stdin.write(json.dumps(req)+"\n")
        proc.stdin.flush()
        state = json.loads(read())["state"]
        assert isinstance(state["position"], list) and len(state["position"]) == 2
        assert isinstance(state["velocity"], list) and len(state["velocity"]) == 2
        vals = state["position"] + state["velocity"] + [state["time"]]
        return all(isinstance(a, (int, float)) and not isinstance(a, bool)
                   and math.isfinite(a) and abs(a-b) <= 2e-6+2e-7*abs(b)
                   for a, b in zip(vals, expected))

    def reset(g, mass, e, plate, position, velocity):
        return {"type": "reset", "config": {"gravity": g, "mass": mass,
                "radius": 0.125, "restitution": e, "plate_y": plate},
                "state": {"position": position, "velocity": velocity}}

    tests = [
        (reset(10, 2, 0.8, None, [0, 5], [1, 0]), [0, 5, 1, 0, 0]),
        ({"type": "step", "dt": 0.2, "impulse": [2, 4]}, [0.4, 5.2, 2, 0, 0.2]),
        ({"type": "step", "dt": 0.3, "impulse": [0, 0]}, [1, 4.75, 2, -3, 0.5]),
        (reset(2, 1, 1, -0.125, [0, 1], [0, 0]), [0, 1, 0, 0, 0]),
        ({"type": "step", "dt": 1, "impulse": [0, 0]}, [0, 0, 0, 2, 1]),
        ({"type": "step", "dt": 1, "impulse": [0, 0]}, [0, 1, 0, 0, 2]),
        (reset(2, 1, 0, -0.125, [0, 1], [1, 0]), [0, 1, 1, 0, 0]),
        ({"type": "step", "dt": 2, "impulse": [0, 0]}, [2, 0, 1, 0, 2]),
        ({"type": "step", "dt": 0, "impulse": [0, 2]}, [2, 0, 1, 2, 2]),
        ({"type": "step", "dt": 0.5, "impulse": [0, 0]}, [2.5, 0.75, 1, 1, 2.5]),
    ]
    try:
        assert read(4).strip() == "READY", "expected READY"
        passed = sum(request(req, answer) for req, answer in tests)
        print(json.dumps({"checks_passed": passed, "checks_total": len(tests)}))
        proc.stdin.write('{"type":"close"}\n')
        proc.stdin.flush()
        assert json.loads(read()) == {"ok": True}
        proc.wait(timeout=2)
    finally:
        if proc.poll() is None:
            proc.kill()
        proc.wait()


if __name__ == "__main__":
    main()

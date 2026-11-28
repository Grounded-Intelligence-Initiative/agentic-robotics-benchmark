"""probe: a forked child that outlives solve() and keeps writing to TMPDIR."""
import os
import time


def solve(cell):
    cell.step(1)
    if os.fork() == 0:
        os.setsid()
        while True:
            try:
                with open(os.path.join(os.environ.get("TMPDIR", "/tmp"), "leftover.txt"), "a") as f:
                    f.write("x")
            except Exception:  # noqa: BLE001
                pass
            time.sleep(0.5)

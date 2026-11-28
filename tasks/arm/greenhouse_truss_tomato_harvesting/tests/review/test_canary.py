#!/usr/bin/env python3
"""The canary's one fresh start, without the simulator (maintainers' check; nothing runs it at grading).

    python3 tests/review/test_canary.py

`verify.canary` against stand-in cells: a cell that does not start, or whose process fails during the check, is
started once more and counted; a second such failure, and a cell that runs but does not render, is a task error at
once (`robotics_grader.stage.fail` ends the run), never a retry without end.
"""

import base64
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import verify as V  # noqa: E402
from rowcell import cellproc  # noqa: E402
from rowcell import farm as F  # noqa: E402


def png(arr):
    return base64.b64encode(cv2.imencode(".png", arr)[1].tobytes()).decode("ascii")


def frames(flat=False):
    rng = np.random.default_rng(1)
    out = {}
    for name, c in F.CAMERAS.items():
        shape = (c["height"], c["width"], 3)
        img = np.full(shape, 0, np.uint8) if flat else rng.integers(40, 200, shape, dtype=np.uint8)
        out[name] = {"rgb": png(img)}
        if c["depth"]:
            out[name]["depth"] = png(np.full(shape[:2], 5000, np.uint16))
    return out


class Cell(object):
    def __init__(self, how):
        self.how, self.closed = how, False

    def request(self, msg, timeout=0.0):
        if self.how == "dies":
            raise cellproc.CellError("the cell's process ended (exit 1)")
        if msg["cmd"] == "reset":
            return {"obs": {"images": frames(flat=self.how == "flat"), "tcp": [0.8, 0.0, 0.5, 0.0, 0.0, 0.0]}}
        return {"obs": {"tcp": [0.8, 0.0, 0.48, 0.0, 0.0, 0.0]}}

    def close(self):
        self.closed = True


class Host(object):
    def __init__(self, plan):
        self.plan, self.count, self.cells = list(plan), 0, []

    def start(self, timeout):
        how = self.plan[self.count]
        self.count += 1
        if how == "no start":
            raise cellproc.CellError("the cell did not start: {'fatal': 'the simulator did not start'}")
        self.cells.append(Cell(how))
        return self.cells[-1]

    def log_tail(self, n=1500):
        return ""


def run(plan):
    host, said = Host(plan), []
    try:
        result = V.canary(host, {"cell_start_timeout_s": 1}, lambda *a: said.append(" ".join(str(x) for x in a)))
    except SystemExit as exc:
        result = "task error %s" % exc.code
    assert all(c.closed for c in host.cells), "a canary cell was left running"
    return result, host.count, said


def main():
    r, n, said = run(["ok"])
    assert r[2] == 0 and n == 1 and not said, (r, n, said)
    r, n, said = run(["no start", "ok"])
    assert r[2] == 1 and n == 2 and len(said) == 1, (r, n, said)
    r, n, said = run(["dies", "ok"])
    assert r[2] == 1 and n == 2 and len(said) == 1, (r, n, said)
    r, n, said = run(["no start", "no start", "ok"])
    assert r == "task error 1" and n == 2, (r, n)
    r, n, said = run(["no start", "dies", "ok"])
    assert r == "task error 1" and n == 2, (r, n)
    r, n, said = run(["flat", "ok"])                       # it runs and does not render: no second start
    assert r == "task error 1" and n == 1, (r, n)
    print("canary: a cell that does not start or dies is started once more and counted; twice, or one that does not "
          "render, is a task error")
    print("all checks passed")


if __name__ == "__main__":
    main()

"""Calibrate every graded unit through the lab and write its predictions and its three job programs.

The reference has what the agent has: the documents (cell_data.py, arm_model.py, defects.md and
camera.md, whose content reference.py and plate_job.py encode), the lab client, each unit's
conditions file, one measuring session per graded unit and, after the unit's service, its one
re-qualification session (requal.py: the model of the measuring session carried over in memory, in
the same process). It does not use the practice unit.
"""
import json
import os
import subprocess
import sys

HOME = os.environ.get("ALE_HOME", "/home/user")
sys.path.insert(0, HOME)
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import reference  # noqa: E402
import requal  # noqa: E402
import submission_format as sf  # noqa: E402
from arm_model import Q_MIN, Q_MAX  # noqa: E402
from cell_data import MOVE_SPEED_MAX, Q_HOME  # noqa: E402
from labclient import LabClient, decode_image  # noqa: E402


class LabCell(object):
    """The five commands of a session, over the lab's socket."""

    def __init__(self, lab, unit):
        self.lab = lab
        self.first = lab.reset(unit)

    def _do(self, command):
        obs, _ = self.lab.step(command)
        if obs["refused"] != "no":
            raise RuntimeError("the cell refused: " + obs["refused"])
        return obs

    def move(self, q, speed=MOVE_SPEED_MAX):
        return self._do({"do": "move", "q": [float(v) for v in q], "speed": float(speed)})

    def probe(self, q):
        return self._do({"do": "probe", "q": [float(v) for v in q]})

    def change_tool(self, name):
        return self._do({"do": "tool", "name": name})

    def image(self):
        obs = self._do({"do": "image"})
        obs["image"] = decode_image(obs["image"])
        return obs

    def wait(self, seconds):
        return self._do({"do": "wait", "seconds": float(seconds)})


def one(unit):
    out = os.path.join(HOME, "submission")
    with open(os.path.join(HOME, "conditions", "{}.json".format(unit))) as handle:
        conditions = json.load(handle)
    with LabClient() as lab:
        tip, cam, info = reference.calibrate(LabCell(lab, unit), conditions, return_info=True)
    # the measuring session is over (its connection closed); the unit has been serviced: its re-qualification
    try:
        with LabClient() as lab:
            info["job_serviced"], est, log = requal.requalify(LabCell(lab, unit + "-serviced"), info)
        print("[oracle] {}: serviced joint {} ({}), {}".format(unit, est and est["joint"], log[-1:] or "-",
                                                            "planned" if info["job_serviced"] is not None else "no job"),
              file=sys.stderr)
    except Exception as exc:  # noqa: BLE001 - the unit's other programs are still written
        info["job_serviced"] = None
        print("[oracle] {}: re-qualification failed ({})".format(unit, type(exc).__name__), file=sys.stderr)
    record = {"tip": [[float(v) for v in row] for row in tip],
              "camera": [[[float(v) for v in row] for row in rows] for rows in cam]}
    for key in ("job", "job_warm", "job_serviced"):
        job = info.get(key)
        if job is None:         # no program planned: one harmless row (the unit's job is not done)
            print("[oracle] {}: no {} program ({})".format(unit, key, "; ".join(info.get("job_log", [])[-2:])),
                  file=sys.stderr)
            job = [Q_HOME]
        record[key] = [[float(v) for v in row] for row in job]
    sf.check(record, len(record["tip"]), len(record["camera"][0]), (Q_MIN, Q_MAX))
    with open(os.path.join(out, "{}.json".format(unit)), "w") as handle:
        json.dump(record, handle)


def main():
    if len(sys.argv) == 2:
        one(sys.argv[1])
        return
    os.makedirs(os.path.join(HOME, "submission"), exist_ok=True)
    with LabClient() as lab:
        units = [u for u in lab.hello()["instances"] if u.startswith("u")]
    for unit in sorted(units):
        # One process per unit: whatever goes wrong with a unit, a crash included, costs that unit only.
        done = subprocess.run([sys.executable, os.path.abspath(__file__), unit])
        print("[oracle] {} {}".format(unit, "calibrated" if done.returncode == 0 else
                                      "failed (exit {})".format(done.returncode)), file=sys.stderr, flush=True)


if __name__ == "__main__":
    main()

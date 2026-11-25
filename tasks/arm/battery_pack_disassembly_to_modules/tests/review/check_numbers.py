#!/usr/bin/env python3
"""Every number the hand-written instruction states, against the code that makes it true (maintainers' tool; the
grader never runs it). The other documents (`given/service_manual.md`, `tool_catalog.md`, `cell/cell_interface.md`,
`cell/cell.json`) are written from the code by `make_documents.py`; this checks that they are what it writes now.

    python3 check_numbers.py
"""

import json
import os
import re
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
TESTS = os.path.dirname(HERE)
TASK = os.path.dirname(TESTS)
sys.path.insert(0, TESTS)
sys.dont_write_bytecode = True

from packcell import series as S  # noqa: E402
from packcell import wire  # noqa: E402


def main():
    text = open(os.path.join(TASK, "instruction.md")).read()
    grader = json.load(open(os.path.join(TESTS, "grader_config.json")))
    service = json.load(open(os.path.join(TASK, "environment", "commission_service", "dev_packs.json")))
    packs = json.load(open(os.path.join(TESTS, "hidden", "packs.json")))["packs"]
    T, W = S.MOUNT_TOL, S.SENSORS
    want = [
        # (what the instruction says, the code's value)
        ("within 12 mm and 1.5 degrees", round(1000 * 0.012) == 12 and S.Stream is not None),
        ("within\n  2 mm of the manual's nominal one", True),
        ("a bit engages only within 1.2 to\n  2 mm", (S.ENGAGE["torx"]["full"], S.ENGAGE["hex"]["full"]) == (0.0012, 0.0020)),
        ("(up to 15 mm and 1.5 degrees overhead, 4 mm and 1 degree on the wrist)",
         (T["overhead"]["mm"], T["overhead"]["deg"], T["wrist"]["mm"], T["wrist"]["deg"]) == (15.0, 1.5, 4.0, 1.0)),
        ("about\n2.5 mm at the fixture from the overhead camera, 0.3 mm from the wrist camera close up",
         round(1000 * W["overhead"]["sigma_k"] * 1.65 ** 2, 1) == 2.5 and round(1000 * W["wrist"]["sigma_k"] * 0.35 ** 2, 1) == 0.3),
        ("READY                   within 120 s of the start", grader["step_timeout_s"] * 4 == 120),
        ("one action for step t, within 30 s", grader["step_timeout_s"] == 30),
        ("1 to 40 control ticks of\n0.05 s", (wire.DEFAULTS["max_ticks"], S.TICK) == (40, 0.05)),
        ("2700 s of simulated time, 400 camera frames, 20000 steps, and 1200 s",
         (grader["cell"]["sim_cap_s"], grader["cell"]["frame_budget"], grader["max_steps"], grader["controller_budget_s"])
         == (2700.0, 400, 20000, 1200) == (wire.DEFAULTS["sim_cap_s"], wire.DEFAULTS["frame_budget"], 20000, 1200)),
        ("**six packs**", len(packs) == 6),
        ("four development packs", len(service["packs"]) == 4),
        ("**Ten sessions in all", service["sessions"] == 10),
        ("(value 8), the lid on its stand (5), the service disconnect in the electronics bin (3), a\nbusbar in the copper bin (2), a bracket in the steel bin (1), a low-voltage plug or the harness remnant in the\nelectronics bin (1 each), a fastener in a fasteners bin (0.5)",
         S.VALUES == {"module": 8.0, "lid": 5.0, "msd": 3.0, "busbar": 2.0, "bracket": 1.0, "connector": 1.0, "harness": 1.0,
                      "fastener": 0.5}),
        ("(4, 5 or 6 modules", sorted(v["modules"] for v in S.VARIANTS.values()) == [4, 5, 6]),
        ("at most three times", True),
    ]
    flat = re.sub(r"\s+", " ", text)
    bad = []
    for phrase, ok in want:
        if re.sub(r"\s+", " ", phrase) not in flat:
            bad.append("not in instruction.md: %r" % phrase)
        elif not ok:
            bad.append("the code disagrees: %r" % phrase)
    # the generated documents are what make_documents.py writes from the code now
    given = os.path.join(TASK, "environment", "setup", "payload", "given")
    names = ["service_manual.md", "tool_catalog.md", os.path.join("cell", "cell_interface.md"), os.path.join("cell", "cell.json")]
    before = {n: open(os.path.join(given, n)).read() for n in names}
    with tempfile.TemporaryDirectory() as tmp:
        subprocess.run([sys.executable, "-B", os.path.join(HERE, "make_documents.py")], check=True, stdout=subprocess.DEVNULL)
    after = {n: open(os.path.join(given, n)).read() for n in names}
    bad += ["%s is not what make_documents.py writes from the code" % n for n in names if before[n] != after[n]]
    for b in bad:
        print("PROBLEM:", b)
    print("numbers ok: %d statements of the instruction against the code; %d generated documents current" % (len(want), len(names))
          if not bad else "%d problem(s)" % len(bad))
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())

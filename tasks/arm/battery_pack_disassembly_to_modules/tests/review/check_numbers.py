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
    verify = open(os.path.join(TESTS, "verify.py")).read()
    toml = open(os.path.join(TASK, "task.toml")).read()
    T, W = S.MOUNT_TOL, S.SENSORS
    want = [
        # (what the instruction says, the code's value)
        ("within 12 mm and 1.5 degrees", round(1000 * 0.012) == 12 and S.Stream is not None),
        ("within\n  2 mm of the manual's nominal one", True),
        ("a bit engages only within 0.5 to 0.8 mm", (S.ENGAGE["torx"]["full"], S.ENGAGE["hex"]["full"]) == (0.0005, 0.0008)),
        ("(up to 15 mm and 1.5 degrees overhead, 4 mm and 1 degree on the wrist)",
         (T["overhead"]["mm"], T["overhead"]["deg"], T["wrist"]["mm"], T["wrist"]["deg"]) == (15.0, 1.5, 4.0, 1.0)),
        ("about\n2.5 mm at the fixture from the overhead camera, 0.3 mm from the wrist camera close up",
         round(1000 * W["overhead"]["sigma_k"] * 1.65 ** 2, 1) == 2.5 and round(1000 * W["wrist"]["sigma_k"] * 0.35 ** 2, 1) == 0.3),
        ("READY                   within 120 s of the start", grader["step_timeout_s"] * 4 == 120),
        ("one action for step t, within 30 s", grader["step_timeout_s"] == 30),
        ("1 to 40 control ticks of\n0.05 s", (wire.DEFAULTS["max_ticks"], S.TICK) == (40, 0.05)),
        ("1500 s of simulated time, 400 camera frames, 20000 steps, and 1200 s",
         (grader["cell"]["sim_cap_s"], grader["cell"]["frame_budget"], grader["max_steps"], grader["controller_budget_s"])
         == (1500.0, 400, 20000, 1200) == (wire.DEFAULTS["sim_cap_s"], wire.DEFAULTS["frame_budget"], 20000, 1200)),
        ("**six packs**", len(packs) == 6),
        ("four development packs", len(service["packs"]) == 4),
        ("**Three sessions in all, one at a time, never refunded**", service["sessions"] == 3),
        ("this session is stopped 2 hours after it starts", int(re.search(r"\[agent\]\ntimeout_sec = (\d+)", toml).group(1)) == 7200),
        ("a bit that is on a head\n  without driving it strips the head the moment it is loaded", S.DAMAGE["torque"] <= 1.0),
        ("(its shear torque 70 to 100 % of the thread's)", S.SHANK["strength"] == (0.70, 1.00)),
        ("(a bulge of 0.45 to 0.8 mm on its top, where a healthy module's top is crowned by up to\n  0.33 mm and the criterion is 0.4 mm)",
         S.SWOLLEN3 == {"bulge": (0.00045, 0.00080), "crown": (0.00010, 0.00033)} and S.SWOLLEN["criterion"] == 0.0004),
        ("a torque limit of 1 N m or more", S.DAMAGE["torque"] == 1.0),
        ("at the third protective stop in a pack", S.CONSEQUENCE["latch"] == 3),
        ("counts 0 (`service_manual.md`, section 9)", set(S.CONSEQUENCE["condemn"]) == {"live_contact", "fastener_short", "module_vented", "module_dropped"}),
        ("(value 8), the lid on its stand (5), the service disconnect in the electronics bin (3), a\nbusbar in the copper bin (2), a bracket in the steel bin (1), a low-voltage plug or the harness remnant in the\nelectronics bin (1 each), a fastener in a fasteners bin (0.5)",
         S.VALUES == {"module": 8.0, "lid": 5.0, "msd": 3.0, "busbar": 2.0, "bracket": 1.0, "connector": 1.0, "harness": 1.0,
                      "fastener": 0.5}),
        ("(4, 5 or 6 modules", sorted(v["modules"] for v in S.VARIANTS.values()) == [4, 5, 6]),
        ("at most three times", True),
        ("at most 5000 files and 200 MB", (int(re.search(r"MAX_SUBMISSION_FILES = (\d+)", verify).group(1)),
                                           grader["max_submission_mb"]) == (5000, 200)),
        ("more than 64 processes of your account", grader["max_processes"] == 64),
        ("The grader stops 20,400 s after it starts", grader["verifier_deadline_s"] == 20400
         and grader["verifier_deadline_s"] + 1200 == int(re.search(r"\[verifier\]\ntimeout_sec = (\d+)", toml).group(1))),
        ("about 12,500 s", "12,500 s with the canary" in toml),
        ("**seized** fasteners (up to four a pack", S.SEIZED["max"] == 4 and S.SEIZED["n_hidden"][1] <= 4
         and S.SEIZED["breakaway"][0] > 1.0),

        ("lifted or pushed 5 mm it vents", S.SWOLLEN["vent_m"] == 0.005),
        ("counted before every request", "def request(self, message):\n        if self.max_processes > 0" in verify),
        ("every file of your account outside the submission folder and every SysV IPC object of your account are\nremoved",
         "scrub_account(uid, gid, workdir, snap)" in verify and "charge_files" not in verify),
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

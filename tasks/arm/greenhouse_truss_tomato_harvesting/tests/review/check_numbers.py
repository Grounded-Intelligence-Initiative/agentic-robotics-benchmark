#!/usr/bin/env python3
"""Every number the hand-written instruction states, against the code that makes it true (maintainers' tool; the
grader never runs it). The other documents (`given/site/site_interface.md`, `site.json`, `tool/cutter_gripper.md`,
`crop/growing_protocol.md`, `robot/ur20.json`, and the reference's copy of `site.json`) are written from the code by
`make_documents.py`; this checks that they are what it writes now, without writing anything.

    python3 check_numbers.py

A statement is a phrase of `instruction.md` with `#` where a number stands (digits, or a word from one to twelve or
first to fifth) and the values the code has for them. A phrase that is not in the instruction, or that carries
another number than the code, is a problem, and so is a number in the instruction that no statement covers: a new
number needs a statement here. The conditions the grader and the commissioning service will hold (deadlines, budgets,
how many rows and sessions) come from `make_documents.conditions()`: from their files once those exist, from the
values the task was specified with until then (it says which).
"""

import json
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
TESTS = os.path.dirname(HERE)
TASK = os.path.dirname(TESTS)
sys.dont_write_bytecode = True
sys.path.insert(0, HERE)
sys.path.insert(0, TESTS)

import make_documents as M  # noqa: E402
from rowcell import farm as F  # noqa: E402
from rowcell import wire  # noqa: E402

WORDS = {w: i + 1 for i, w in enumerate("one two three four five six seven eight nine ten eleven twelve".split())}
WORDS.update({w: i + 1 for i, w in enumerate("first second third fourth fifth".split())})
NUMBER = r"(\d+(?:\.\d+)?|%s)" % "|".join(sorted(WORDS, key=len, reverse=True))
# Numbers of the instruction that state nothing about the cell: names, versions, references to a document's section,
# the wire's sample step, a score of nothing, the harness note.
NOT_THE_CELLS = [r"UR20", r"ur20", r"Python 3\.12", r"Isaac Sim 6\.2", r"Isaac Lab 3\.0", r"section \d+", r'"t": 0', r"scores 0",
                 r"counts 0", r"roughly 25-30k tokens", r"at least 15 minutes", r"mm\^2", r"base64", r"python3", r"cv2"]
# What no document for the tested agent may say (the task's rules for its texts).
BANNED = [r"\bpolicy\b", r"ground truth", r"\btruth\b", r"\bseeds?\b", r"\btests/", r"\bsolution/", r"\boracle\b",
          r"reference (program|solution|implementation)", r"\btoken run\b", r"\bbrief\b"]


def value(text):
    return float(WORDS.get(text.lower(), text))


def pattern(phrase):
    return "".join(NUMBER if tok == "#" else r"\s+" if tok.isspace() else re.escape(tok)
                   for tok in re.split(r"(#|\s+)", phrase) if tok)


def statements(C):
    """[(phrase, the code's values for its # marks: a tuple, or a function of the stated numbers that says whether
    the code agrees)]."""
    R, T, D, S = F.ROW, F.TOOL, wire.DEFAULTS, F.SENSORS["wrist"]
    cv = F.CULTIVARS[C["graded_cultivar"]]
    f_wrist = F.camera_K("wrist")[0][0]
    stalk = 2 * max(max(c["peduncle_r"][1] for c in F.CULTIVARS.values()), M.lit("farm", "^rp = st.uniform(#, #)")[1])
    return [
        ("grader stops # s after it starts", (C["verifier_deadline_s"],)),
        ("# plants hang on their strings, with # trusses each", (R["plants"], R["trusses"])),
        ("a peduncle is # to # mm long and # to # mm thick. The mouth that takes it is # mm wide and # mm deep",
         (1e3 * cv["peduncle_len"][0], 1e3 * cv["peduncle_len"][1], 2e3 * cv["peduncle_r"][0], 2e3 * cv["peduncle_r"][1],
          1e3 * T["open_max"], 1e3 * (T["mouth_z"][1] - T["mouth_z"][0]))),
        ("# to # of the #, by the cultivar's rule", (R["ready"][0], R["ready"][1], R["plants"] * R["trusses"])),
        ("(for %s, the cultivar you are graded on: at least # %% of a truss's fruits at ripening stage # of # or beyond)" % C["graded_cultivar"],
         (100 * cv["fraction"], cv["stage_min"], len(cv["chart"]))),
        ("brightness (# to # times the nominal) and colour (its green share down to #, its blue share down to # of the nominal)",
         (R["light_gain"][0], R["light_gain"][1], R["light_g"][0], R["light_b"][0])),
        ("**toughness** (# to # N per mm^2 of cross-section)", tuple(F.CUT["toughness"])),
        ("with more than # times its own cutting force", (F.CUT["crush"],)),
        ("the cart's wheels slip by # to # %", (100 * R["slip"][0], 100 * R["slip"][1])),
        ("so its odometer over-reads", lambda: R["slip"][0] > 0),
        ("(up to # mm and # degrees for the overview camera, # mm and # degrees for the wrist camera)",
         (F.MOUNT_TOL["overview"]["mm"], F.MOUNT_TOL["overview"]["deg"], F.MOUNT_TOL["wrist"]["mm"], F.MOUNT_TOL["wrist"]["deg"])),
        ("noisy (about # mm at # m)", lambda mm_, z: round(1e3 * S["sigma_k"] * z * z, 1) == mm_),
        ("without a reading closer than # m", (S["range_m"][0],)),
        ("narrower than its matcher's window of # pixels", (S["window_px"],)),
        ("a peduncle or a leaf's stalk gives no depth at any distance", lambda: stalk < S["window_px"] * S["range_m"][0] / f_wrist),
        ("The overview camera has no depth.", lambda: not F.CAMERAS["overview"]["depth"] and F.CAMERAS["wrist"]["depth"]),
        ("the velocity of its fruits changing by more than # m/s within one control tick", (F.DAMAGE["impact_dv"],)),
        ("with more than # N, is waste", (F.DAMAGE["squeeze_force"],)),
        ("at the # protective stop", (F.CONSEQUENCE["latch"],)),
        ("<- READY within # s of the start", (C["ready_timeout_s"],)),
        ("one action for step t, within # s", (C["step_timeout_s"],)),
        ("# to # control ticks of # s", (1, D["max_ticks"], F.TICK)),
        ("at most # files and # MB in all", (C["max_submission_files"], C["max_submission_mb"])),
        ("more than # processes of your account", (C["max_processes"],)),
        ("started again for the next row section, at most # times", (C["restarts"],)),
        ("(that restart is not one of the #)", (C["restarts"],)),
        ("runs the cell on # practice row sections", (C["practice_rows"],)),
        ("the practice rows grow the other cultivar, **%s**" % C["practice_cultivar"], lambda: C["practice_cultivar"] != C["graded_cultivar"]),
        ("**# sessions in all, one at a time, never refunded** (fewer than there are practice rows",
         lambda n: n == C["sessions"] < C["practice_rows"]),
        ("a fixed hidden set of **# row sections** of the cultivar %s" % C["graded_cultivar"], (C["graded_rows"],)),
        ("# s of simulated time, # camera frames, # steps, and # s of your program's own time",
         (D["sim_cap_s"], D["frame_budget"], C["max_steps"], C["controller_budget_s"])),
        ("is the mean of the # yields", (C["graded_rows"],)),
        ("this session is stopped # hours after it starts", (C["agent_timeout_s"] / 3600.0,)),
    ]


# (a reference of the instruction, the generated document, the heading it has to point at)
SECTIONS = [
    ("`~/given/site/site_interface.md`, section 6", ("site", "site_interface.md"), "## 6. Cameras"),
    ("`~/given/site/site_interface.md`, section 2", ("site", "site_interface.md"), "## 2. The wire"),
    ("(`site_interface.md`, section 2)", ("site", "site_interface.md"), "## 2. The wire"),
    ("(`site_interface.md`, section 9)", ("site", "site_interface.md"), "## 9. Commissioning"),
    ("(`site_interface.md`, section 8)", ("site", "site_interface.md"), "## 8. How a row section is judged"),
    ("(growing protocol, section 2)", ("crop", "growing_protocol.md"), "## 2. The cultivars"),
]


def main():
    C = M.conditions()
    docs = M.documents()
    with open(os.path.join(TASK, "instruction.md")) as f:
        raw = f.read()
    flat = re.sub(r"\s+", " ", raw)
    bad, covered = [], []
    want = statements(C)
    for phrase, code in want:
        found = list(re.finditer(pattern(phrase), flat, re.I))
        if not found:
            bad.append("not in instruction.md: %r" % phrase)
        for m in found:
            covered.append(m.span())
            said = tuple(value(v) for v in m.groups())
            if callable(code):
                if not code(*said):
                    bad.append("the code disagrees with %r" % m.group(0))
            elif len(said) != len(code) or any(abs(a - float(b)) > 1e-9 * max(1.0, abs(float(b))) for a, b in zip(said, code)):
                bad.append("instruction.md says %r; the code has %s" % (m.group(0), ", ".join(M.g(b) for b in code)))
    # a number that no statement covers
    for skip in NOT_THE_CELLS:
        covered += [m.span() for m in re.finditer(skip, flat)]
    for m in re.finditer(r"\d+(?:\.\d+)?", flat):
        if not any(a <= m.start() and m.end() <= b for a, b in covered):
            bad.append("a number of instruction.md that no statement here holds to the code: %r" % flat[max(0, m.start() - 50):m.end() + 30])
    # the sections the instruction points at
    for ref, doc, heading in SECTIONS:
        if ref not in flat:
            bad.append("not in instruction.md: %r" % ref)
        elif heading not in docs[os.path.join(M.GIVEN, *doc)]:
            bad.append("%r points at %r, which %s no longer has" % (ref, heading, doc[-1]))
    # the metric's name, once the anchor exists
    anchor = os.path.join(TESTS, "anchor.json")
    if os.path.exists(anchor):
        with open(anchor) as f:
            metric = json.load(f)["metric"]
        if "`%s`" % metric not in raw:
            bad.append("tests/anchor.json names the metric %r; instruction.md does not" % metric)
    # the instruction's size: about 190 lines of 120 columns (a table's row is one line however long)
    lines = raw.split("\n")
    wide = [i + 1 for i, ln in enumerate(lines) if len(ln) > 120 and not ln.startswith("|")]
    if len(lines) > 190 or wide:
        bad.append("instruction.md has %d lines (190 at most) and lines over 120 columns at %s" % (len(lines), wide or "none"))
    # what no text for the tested agent says
    texts = {"instruction.md": raw}
    texts.update({os.path.relpath(p, TASK): t for p, t in docs.items() if p.startswith(M.GIVEN) and p.endswith(".md")})
    for name, text in texts.items():
        for word in BANNED:
            for m in re.finditer(word, text, re.I):
                bad.append("%s says %r: ...%s..." % (name, m.group(0), re.sub(r"\s+", " ", text[max(0, m.start() - 40):m.end() + 40])))
    # the generated documents are what make_documents.py writes from the code now
    old = M.stale(docs)
    bad += ["%s is not what make_documents.py writes from the code (run it)" % n for n in old]
    for b in bad:
        print("PROBLEM:", b)
    if C["pending"]:
        print("note: not there yet, so the task's specified values stand in for theirs: " + ", ".join(C["pending"]))
    print("numbers ok: %d statements of the instruction against the code; %d generated documents current" % (len(want), len(docs))
          if not bad else "%d problem(s)" % len(bad))
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())

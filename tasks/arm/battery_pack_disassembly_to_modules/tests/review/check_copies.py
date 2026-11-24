#!/usr/bin/env python3
"""Check (or, with --sync, restore) the byte copies this task keeps of its own files (maintainers' tool).

    python3 check_copies.py [--sync]

- environment/commission_service/packcell/ is a byte copy of tests/packcell/: the commissioning service runs the
  cell the grader runs. The grader's copy is the authority; after a change there, run this with --sync.
- tests/robotics_grader/ is a byte copy of the repository's shared/robotics_grader/ (never edited here; CI checks it).
- environment/setup/payload/given/cell/cell.json and solution/reference/cell.json are the same file
  (tests/review/make_documents.py writes both).
"""

import filecmp
import os
import shutil
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
TASK = os.path.dirname(os.path.dirname(HERE))
REPO = os.path.abspath(os.path.join(TASK, "..", "..", ".."))


def files(root):
    out = {}
    for dp, dns, fns in os.walk(root):
        dns[:] = [d for d in dns if d != "__pycache__"]
        for fn in fns:
            p = os.path.join(dp, fn)
            out[os.path.relpath(p, root)] = p
    return out


def same_tree(a, b):
    fa, fb = files(a), files(b)
    bad = sorted(set(fa) ^ set(fb)) + sorted(k for k in set(fa) & set(fb) if not filecmp.cmp(fa[k], fb[k], shallow=False))
    return bad


def main():
    sync = "--sync" in sys.argv[1:]
    src, dst = os.path.join(TASK, "tests", "packcell"), os.path.join(TASK, "environment", "commission_service", "packcell")
    if sync:
        shutil.rmtree(dst, ignore_errors=True)
        shutil.copytree(src, dst, ignore=shutil.ignore_patterns("__pycache__"))
    problems = ["commission_service/packcell differs from tests/packcell: " + k for k in same_tree(src, dst)]
    kit = os.path.join(REPO, "shared", "robotics_grader")
    if os.path.isdir(kit):
        problems += ["tests/robotics_grader differs from shared/robotics_grader: " + k
                     for k in same_tree(kit, os.path.join(TASK, "tests", "robotics_grader"))]
    a = os.path.join(TASK, "environment", "setup", "payload", "given", "cell", "cell.json")
    b = os.path.join(TASK, "solution", "reference", "cell.json")
    if not (os.path.isfile(a) and os.path.isfile(b) and filecmp.cmp(a, b, shallow=False)):
        problems.append("given/cell/cell.json and solution/reference/cell.json differ")
    for p in problems:
        print("PROBLEM:", p)
    print("copies ok" if not problems else "%d problem(s)" % len(problems))
    return 1 if problems else 0


if __name__ == "__main__":
    sys.exit(main())

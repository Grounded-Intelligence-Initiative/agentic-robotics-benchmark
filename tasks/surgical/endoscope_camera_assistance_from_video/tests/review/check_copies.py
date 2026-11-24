"""Maintainers' tool: the task's byte copies, checked or rewritten.

    python3 tests/review/check_copies.py            exit 1 and a list when a copy differs
    python3 tests/review/check_copies.py --write    rewrite every copy from its source

The task folder holds three things twice, on purpose, and each pair has to stay byte-identical:

* tests/endosim/  ->  environment/commission_service/endosim/   the simulator the agent commissions against is the
  simulator that grades (the image cannot copy from tests/, and tests/ is absent from the agent's sandbox);
* tests/endosim/model.py  ->  environment/setup/payload/given/holder_model.py and .../payload/example/holder_model.py
  the public holder model the manual points to (and the example controller imports) is the file the grader computes
  with;
* shared/robotics_grader/  ->  tests/robotics_grader/   the vendored kit (checked only: never rewritten from here; the
  repository's CI compares it too).
"""

import filecmp
import os
import shutil
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
TASK = os.path.dirname(os.path.dirname(HERE))
REPO = os.path.dirname(os.path.dirname(os.path.dirname(TASK)))


def files(root):
    out = {}
    for d, dirs, names in os.walk(root):
        dirs[:] = [x for x in dirs if x != "__pycache__"]
        for n in names:
            if not n.endswith(".pyc"):
                p = os.path.join(d, n)
                out[os.path.relpath(p, root)] = p
    return out


def compare_tree(src, dst, write):
    a, b = files(src), files(dst) if os.path.isdir(dst) else {}
    bad = []
    for rel in sorted(set(a) | set(b)):
        if rel not in a:
            bad.append("{}: not in {}".format(os.path.join(dst, rel), src))
            if write:
                os.remove(b[rel])
        elif rel not in b or not filecmp.cmp(a[rel], b[rel], shallow=False):
            bad.append("{} differs from {}".format(os.path.join(dst, rel), a[rel]))
            if write:
                os.makedirs(os.path.dirname(os.path.join(dst, rel)), exist_ok=True)
                shutil.copyfile(a[rel], os.path.join(dst, rel))
    return bad


def main():
    write = "--write" in sys.argv
    bad = compare_tree(os.path.join(TASK, "tests", "endosim"), os.path.join(TASK, "environment", "commission_service", "endosim"), write)
    src = os.path.join(TASK, "tests", "endosim", "model.py")
    for where in ("given", "example"):
        dst = os.path.join(TASK, "environment", "setup", "payload", where, "holder_model.py")
        if not os.path.isfile(dst) or not filecmp.cmp(src, dst, shallow=False):
            bad.append("{} differs from {}".format(dst, src))
            if write:
                shutil.copyfile(src, dst)
    kit = compare_tree(os.path.join(REPO, "shared", "robotics_grader"), os.path.join(TASK, "tests", "robotics_grader"), False)
    for line in bad:
        print(("rewritten: " if write else "DIFFERS: ") + line)
    for line in kit:
        print("DIFFERS (the vendored kit; copy shared/robotics_grader over it): " + line)
    if not bad and not kit:
        print("copies identical: endosim (commissioning service), holder_model.py, robotics_grader")
    sys.exit(1 if (kit or (bad and not write)) else 0)


if __name__ == "__main__":
    main()

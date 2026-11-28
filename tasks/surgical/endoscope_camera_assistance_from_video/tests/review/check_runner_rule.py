#!/usr/bin/env python3
"""Check that the agent's session launcher counts a repeated step as the grader's lock does (maintainers' tool).

    python3 check_runner_rule.py

`environment/setup/payload/commission/run_session.py` prints how many repeated requests changed the action. Its
`differ(first, second)` has to be the lock's rule (`robotics_grader.replies_differ`) on the grader's canonical form of
an action (`endosim.model.clip_command`), or a count that looks fine in commissioning could belong to a controller
the lock fails. No simulator and no service are needed: the function is read out of the launcher's source.
"""

import ast
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
TESTS = os.path.dirname(HERE)
sys.path.insert(0, TESTS)

from endosim import model  # noqa: E402
from robotics_grader import replies_differ  # noqa: E402

RUNNER = os.path.join(os.path.dirname(TESTS), "environment", "setup", "payload", "commission", "run_session.py")
CASES = [
    ([0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0]),
    ([0.1, 0.0, 0.0, 0.0], [0.1, 0.0, 0.0, 0.00005]),          # inside the tolerance
    ([0.1, 0.0, 0.0, 0.0], [0.1, 0.0, 0.0, 0.0002]),           # outside it
    ([0.1, -0.2, 0.01, 0.3], [0.12, -0.2, 0.01, 0.3]),
    ([9.0, 0.0, 0.0, 0.0], [7.0, 0.0, 0.0, 0.0]),              # both beyond the pan limit: the same command
    ([0.0, 0.0, 5.0, 0.0], [0.0, 0.0, 0.5, 0.0]),              # both beyond the insertion limit
    ([0.0, 0.0, 0.0, -3.0], [0.0, 0.0, 0.0, 3.0]),             # beyond the limit on either side: they differ
    ([model.RATE_MAX[0], 0.0, 0.0, 0.0], [model.RATE_MAX[0] + 1.0, 0.0, 0.0, 0.0]),
]


def main():
    with open(RUNNER) as f:
        tree = ast.parse(f.read())
    fn = [n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == "differ"]
    assert fn, "run_session.py has no differ(first, second)"
    scope = {"clip_command": model.clip_command}
    exec(compile(ast.Module(body=fn, type_ignores=[]), RUNNER, "exec"), scope)
    canonical = lambda a: [float(x) for x in model.clip_command(a)]  # noqa: E731 - verify.canonical_action's form
    for first, second in CASES:
        want = replies_differ(first, second, 1e-4, canonical)
        got = bool(scope["differ"](first, second))
        assert got is want, (first, second, got, want)
    print("ok: {} pairs counted as the lock counts them".format(len(CASES)))


if __name__ == "__main__":
    main()

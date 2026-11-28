"""dexterous_hand_grip_controller_from_docs: the grader's and the controller's numerics do not depend on the CPU.

numpy's and scipy's bundled OpenBLAS picks its kernel per CPU when it is loaded; on an AVX-512 host (Zen 4 and later)
its blocking can crash a product or return wrong numbers, and a simulation amplifies a last-bit difference. The pair
OPENBLAS_CORETYPE=Haswell with OPENBLAS_L2_SIZE=256 (never one without the other) and one BLAS thread fix the kernels;
NPY_DISABLE_CPU_FEATURES="X86_V4 AVX512_ICL AVX512_SPR" keeps numpy's own loops on one code path (the AVX-512 loops of
exp, log, arctan2 and the like round differently from the AVX2 ones). All are read when the library loads, so each
process must start with them:

  * the image (environment/Dockerfile): each as an ENV line, side by side, so the agent's session and its self-test
    have them;
  * the verifier's own process: tests/test.sh exports them before it starts the interpreter;
  * the unit workers and the controller's process: the acceptance gives both an explicit environment holding them
    (grader/acceptance.py BLAS_ENV: worker_env and the runner's extra_env in _endpoint_for), and the given self-test
    starts the controller with the same values.

Standard library only; nothing is downloaded or run beyond reading the task's files.
"""
import ast
import os
import re
import sys
import unittest
from pathlib import Path

sys.dont_write_bytecode = True

SLUG = "dexterous_hand_grip_controller_from_docs"
PAIR = {"OPENBLAS_CORETYPE": "Haswell", "OPENBLAS_L2_SIZE": "256"}
THREADS = {"OPENBLAS_NUM_THREADS": "1"}
NUMPY = {"NPY_DISABLE_CPU_FEATURES": "X86_V4 AVX512_ICL AVX512_SPR"}


# a fixed path (the benchmark's tests/ beside tasks/), or ARB_TASK_DIR: a missing folder fails, it never skips
TASK = Path(os.environ.get("ARB_TASK_DIR") or Path(__file__).resolve().parents[1] / "tasks" / "hand" / SLUG)


def _dict_literal(path, name):
    tree = ast.parse(path.read_text(encoding="utf-8"))
    for node in ast.walk(tree):
        if isinstance(node, ast.Assign) and any(isinstance(t, ast.Name) and t.id == name for t in node.targets):
            return ast.literal_eval(node.value)
    raise AssertionError("%s defines no %s" % (path, name))


class BlasEnvironment(unittest.TestCase):
    def test_the_image_sets_the_pair(self):
        text = (TASK / "environment" / "Dockerfile").read_text(encoding="utf-8")
        envs = dict(re.findall(r"^ENV (OPENBLAS_[A-Z0-9_]+)=(\S+)\s*$", text, re.M))
        self.assertEqual({k: envs.get(k) for k in PAIR}, PAIR)

    def test_the_image_sets_numpys_code_paths_beside_the_pair(self):
        lines = (TASK / "environment" / "Dockerfile").read_text(encoding="utf-8").splitlines()
        rows = [i for i, ln in enumerate(lines) if re.match(r'^ENV NPY_DISABLE_CPU_FEATURES="([^"]*)"\s*$', ln)]
        self.assertEqual(len(rows), 1, "one ENV line of NPY_DISABLE_CPU_FEATURES")
        got = re.match(r'^ENV NPY_DISABLE_CPU_FEATURES="([^"]*)"', lines[rows[0]]).group(1)
        self.assertEqual(got, NUMPY["NPY_DISABLE_CPU_FEATURES"])
        pair = [i for i, ln in enumerate(lines) if re.match(r"^ENV OPENBLAS_(CORETYPE|L2_SIZE)=", ln)]
        self.assertEqual(sorted(pair + rows), list(range(min(pair + rows), min(pair + rows) + 3)),
                         "the three ENV lines stand side by side")

    def test_the_verify_stage_exports_the_pair_before_python(self):
        text = (TASK / "tests" / "test.sh").read_text(encoding="utf-8")
        exp = [ln for ln in text.splitlines() if ln.strip().startswith("export ")]
        joined = " ".join(exp)
        for k, v in dict(PAIR, **THREADS).items():
            self.assertIn("%s=%s" % (k, v), joined)
        for k, v in NUMPY.items():
            self.assertIn('%s="%s"' % (k, v), joined)
        first_python = next(i for i, ln in enumerate(text.splitlines()) if "verify.py" in ln and "exec" in ln)
        last_export = max(i for i, ln in enumerate(text.splitlines())
                          if ("OPENBLAS_" in ln or "NPY_" in ln) and "export" in ln)
        self.assertLess(last_export, first_python)

    def test_the_workers_and_the_controller_get_the_pair(self):
        acc = _dict_literal(TASK / "tests" / "grip" / "grader" / "acceptance.py", "BLAS_ENV")
        self.assertEqual({k: acc.get(k) for k in dict(PAIR, **THREADS, **NUMPY)}, dict(PAIR, **THREADS, **NUMPY))

    def test_the_acceptance_hands_the_values_to_the_worker_and_the_controller(self):
        tree = ast.parse((TASK / "tests" / "grip" / "grader" / "acceptance.py").read_text(encoding="utf-8"))
        fns = {n.name: n for n in tree.body if isinstance(n, ast.FunctionDef)}
        worker = ast.unparse(fns["worker_env"])
        self.assertIn("env.update(BLAS_ENV)", worker, "the unit worker's fixed environment lacks BLAS_ENV")
        ep = fns["_endpoint_for"]
        kws = [k for c in ast.walk(ep) if isinstance(c, ast.Call) and ast.unparse(c.func).endswith("Runner")
               for k in c.keywords if k.arg == "extra_env"]
        self.assertEqual([ast.unparse(k.value) for k in kws], ["dict(BLAS_ENV)"], "the controller's environment")

    def test_the_self_test_starts_the_controller_with_the_same_values(self):
        cc = _dict_literal(TASK / "environment" / "setup" / "payload" / "given" / "controller" / "check_controller.py",
                           "CONTROLLER_ENV")
        acc = _dict_literal(TASK / "tests" / "grip" / "grader" / "acceptance.py", "BLAS_ENV")
        for k in dict(PAIR, **THREADS, **NUMPY):
            self.assertEqual(cc.get(k), acc.get(k), k)


if __name__ == "__main__":
    unittest.main()

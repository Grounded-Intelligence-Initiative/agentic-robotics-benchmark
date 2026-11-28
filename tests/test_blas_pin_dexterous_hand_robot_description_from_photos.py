"""dexterous_hand_robot_description_from_photos: the grader's numerics do not depend on the CPU it runs on.

numpy's bundled OpenBLAS picks its kernel per CPU when it is loaded, and numpy picks its SIMD paths per CPU; the
grader's IK (damped least squares), the loop closure and the band arithmetic go through both, so a last-bit
difference can move a request across a band edge. The pin is OPENBLAS_CORETYPE=Haswell, OPENBLAS_L2_SIZE=256 (the
Haswell kernel's own blocking) and NPY_DISABLE_CPU_FEATURES="X86_V4 AVX512_ICL AVX512_SPR" (numpy 2.5's names), read
when numpy is first imported, so it must be in the environment the verify interpreter starts with:

  * tests/test.sh exports all three, and one thread each, unconditionally, before it execs the interpreter;
  * tests/verify.py and the two modules it imports at module level (lockdown, verify_freeze) are stdlib only, so no
    numpy is loaded before main();
  * nothing in tests/ (the kit aside) sets, removes or rebuilds the process environment, and the grading child of
    rule 11 is an os.fork of the verify process (no exec, no subprocess), so it runs on the same kernels.

Stdlib only; nothing is downloaded and nothing of the grading stack is imported.
"""
import ast
import re
import unittest
from pathlib import Path

SLUG = "dexterous_hand_robot_description_from_photos"
TASK = Path(__file__).resolve().parents[1] / "tasks" / "hand" / SLUG
TESTS = TASK / "tests"
PIN = {"OPENBLAS_CORETYPE": "Haswell", "OPENBLAS_L2_SIZE": "256",
       "NPY_DISABLE_CPU_FEATURES": "X86_V4 AVX512_ICL AVX512_SPR",
       "OMP_NUM_THREADS": "1", "OPENBLAS_NUM_THREADS": "1", "MKL_NUM_THREADS": "1"}
STDLIB_OK = {"ast", "hashlib", "json", "math", "os", "pwd", "shutil", "signal", "stat", "sys", "time", "threading"}


def _exports(text):
    """{name: value} of the `export` lines of a shell script (values unquoted), in order."""
    out = {}
    for line in text.splitlines():
        s = line.strip()
        if not s.startswith("export "):
            continue
        for name, value in re.findall(r'([A-Z_][A-Z0-9_]*)=("[^"]*"|\S+)', s[len("export "):]):
            out[name] = value.strip('"')
    return out


class TestShPin(unittest.TestCase):
    def test_test_sh_exports_the_pin_before_the_interpreter(self):
        text = (TESTS / "test.sh").read_text()
        exec_at = next(i for i, line in enumerate(text.splitlines()) if line.lstrip().startswith("exec "))
        before = "\n".join(text.splitlines()[:exec_at])
        exported = _exports(before)
        for key, value in PIN.items():
            self.assertEqual(exported.get(key), value, key)
            self.assertNotIn("${%s:-" % key, before, "%s must not keep the calling shell's value" % key)
        self.assertIn('exec "${ALE_VERIFY_PYTHON:-python3}" -B verify.py', text)


class NoNumpyBeforeMain(unittest.TestCase):
    def _module_imports(self, path):
        tree = ast.parse(path.read_text())
        names = set()
        for node in tree.body:
            if isinstance(node, ast.Import):
                names.update(a.name.split(".")[0] for a in node.names)
            elif isinstance(node, ast.ImportFrom):
                names.add((node.module or "").split(".")[0])
        return names

    def test_verify_and_its_module_level_imports_are_stdlib(self):
        self.assertEqual(self._module_imports(TESTS / "verify.py") - STDLIB_OK, {"lockdown", "verify_freeze"})
        for name in ("lockdown.py", "verify_freeze.py"):
            self.assertEqual(self._module_imports(TESTS / name) - STDLIB_OK, set(), name)


class TheEnvironmentIsInherited(unittest.TestCase):
    def test_nothing_in_tests_rewrites_the_environment(self):
        bad = []
        for path in sorted(TESTS.rglob("*.py")):
            if "robotics_grader" in path.parts:
                continue
            src = path.read_text()
            for pat in (r"os\.environ\[[^\]]+\]\s*=", r"os\.environ\.(pop|update|clear|setdefault)\(", r"os\.putenv",
                        r"os\.unsetenv", r"\bsubprocess\b", r"os\.exec[lv]", r"os\.spawn", r"os\.posix_spawn"):
                for m in re.finditer(pat, src):
                    bad.append("%s: %s" % (path.relative_to(TASK), m.group(0)))
        self.assertEqual(bad, [])

    def test_the_grading_child_is_a_fork(self):
        tree = ast.parse((TESTS / "verify.py").read_text())
        fn = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == "grade_in_budget")
        calls = {ast.unparse(c.func) for c in ast.walk(fn) if isinstance(c, ast.Call)}
        self.assertIn("os.fork", calls)
        self.assertFalse({c for c in calls if c.startswith(("os.exec", "subprocess"))}, calls)


if __name__ == "__main__":
    unittest.main()

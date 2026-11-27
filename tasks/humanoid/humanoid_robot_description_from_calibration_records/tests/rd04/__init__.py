"""rd04: the static grader of the humanoid task "a robot description from calibration records".

    from rd04 import load, score, Hidden, TrueUnit
    F = load(sub_dir)                 # Files, or GateFail(reasons) (returned)
    out = score(F, hidden)            # {score, M, D, families, terms, postures, counts, gate, metrics}

Nothing submitted is executed: unit.xml is parsed and compiled by MuJoCo from a string (with the grader's own copy
of the given meshes), calibration.yaml is token-scanned and read with yaml.safe_load. The grader is deterministic
(no clock, no draw; fixed iteration counts) and runs on one CPU thread under the image's BLAS pin."""
from .files import Files, GateFail, GateMissing, load, load_truth  # noqa: F401
from .grade import combine, grade_submission, load_config, load_hidden, raw_errors, score  # noqa: F401
from .truth import Hidden, TrueSideFail, TrueUnit, make_bus_arrays, make_plan_set  # noqa: F401

__all__ = ["Files", "GateFail", "GateMissing", "load", "load_truth", "score", "combine", "raw_errors", "load_config", "Hidden", "TrueUnit",
           "TrueSideFail", "make_bus_arrays", "make_plan_set", "grade_submission", "load_hidden"]

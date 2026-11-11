"""The wire rules of longtail_manipulation_recovery, shared byte for byte by the grader (tests/) and the practice grader
(the agent's home), so the practice run enforces exactly what grading enforces.

Everything here is pure checking: limits, the action check and the job-balanced score. No simulator, no I/O.
"""
import math

STEP_TIMEOUT_S = 10.0          # per request (reset or act)
READY_TIMEOUT_S = 40.0         # READY after each (re)start
CONTROLLER_CAP_S = 400.0       # per episode, summed over act replies, from the request being written to the reply
MAX_RESTARTS = 3
MAX_LINE = 65536               # characters per reply line, newline excluded
MAX_QUEUED_LINES = 64          # lines buffered from the controller; beyond that its writes block
MAX_PROCESSES = 512            # processes of the controller's account at any pause
MAX_FRAMES = 3000              # rendered camera images per episode
SUCCESS_RUN = 10               # consecutive steps the job's success check must hold


class Malformed(ValueError):
    """An action that ends the episode (scored 0)."""


def _number(x):
    # JSON gives int, float, bool, str, list, dict or None; only real numbers count (bool is not a number here).
    if type(x) not in (int, float):
        raise Malformed("action entries must be numbers")
    try:
        v = float(x)
    except OverflowError:
        raise Malformed("action entry out of range")
    if not math.isfinite(v):
        raise Malformed("non-finite action")
    return v


def check_action(action):
    """(12 controls clipped to [-1, 1], camera mask) for a valid action, else Malformed."""
    if not isinstance(action, list) or len(action) not in (12, 13):
        raise Malformed("malformed action (need a list of 12 numbers, or 13 with the camera mask)")
    values = [_number(x) for x in action]
    controls = [max(-1.0, min(1.0, v)) for v in values[:12]]
    cams = 0
    if len(values) == 13:
        m = values[12]
        if m != int(m) or not 0 <= m <= 15:
            raise Malformed("camera mask must be an integer from 0 to 15")
        cams = int(m)
    return controls, cams


def line_too_long(line):
    return len(line.rstrip("\n")) > MAX_LINE


def job_balanced(results):
    """results: list of (job, recovered 0/1). Mean over jobs of each job's recovered fraction, and the per-job table."""
    per = {}
    for job, ok in results:
        n, k = per.get(job, (0, 0))
        per[job] = (n + 1, k + (1 if ok else 0))
    fractions = {job: k / n for job, (n, k) in per.items()}
    score = sum(fractions.values()) / len(fractions) if fractions else 0.0
    return score, fractions

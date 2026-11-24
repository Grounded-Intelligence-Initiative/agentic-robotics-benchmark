"""submission_format.py: a unit's submission, <unit id>.json, and how it is read. Public.

A submission for a unit is a JSON object with exactly five keys:

    "tip":      N rows of 3 numbers: the predicted tool-tip position (m, datum frame) for each of
                the unit's N acceptance conditions, in the order of the conditions file;
    "camera":   N rows of K rows of 3 numbers: for each condition, the predicted datum-frame
                position (m) of each of the K camera-frame points of the conditions file;
    "job":      the cold job program: 1 to JOB_MAX_ROWS rows of 6 numbers, joint targets (rad) in the
                order they are run, each inside its joint's limits (the limits included);
    "job_warm": the warm job program (run from the start temperatures `job_warm_temp` of the
                conditions file): the same rules as "job";
    "job_serviced": the serviced job program (run cold on the unit after its service): the same
                rules as "job".

Numbers are JSON numbers that are finite as 64-bit floats and no larger than LIMIT in magnitude:
no booleans, no strings, no null, no NaN or Infinity. No key may appear twice. The file is UTF-8
and at most MAX_BYTES long. `parse(data, n, k, q_limits)` is the grader's reader (q_limits: the
joint limits, arm_model.Q_MIN and Q_MAX).
"""
import json

MAX_BYTES = 1 << 20
LIMIT = 100.0           # m: nothing in the cell is farther from the datum than this
JOB_MAX_ROWS = 200      # rows of a job program (cell_data.JOB_MAX_ROWS)
KEYS = ("tip", "camera", "job", "job_warm", "job_serviced")
JOBS = ("job", "job_warm", "job_serviced")


def _no_constant(name):
    raise ValueError("not a number: " + name)


def _no_duplicates(pairs):
    keys = [key for key, _ in pairs]
    if len(set(keys)) != len(keys):
        raise ValueError("a key appears twice")
    return dict(pairs)


def _number(value):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError("not a number")
    try:
        value = float(value)
    except OverflowError:
        raise ValueError("not finite")
    if value != value or abs(value) > LIMIT:
        raise ValueError("not finite, or larger than {} in magnitude".format(LIMIT))
    return value


def _rows(value, shape):
    if not isinstance(value, list) or len(value) != shape[0]:
        raise ValueError("wrong shape")
    if len(shape) == 1:
        return [_number(v) for v in value]
    return [_rows(v, shape[1:]) for v in value]


def check_job(value, q_limits):
    """A job program as a list of rows of 6 floats; ValueError otherwise."""
    if not isinstance(value, list) or not 1 <= len(value) <= JOB_MAX_ROWS:
        raise ValueError("1 to {} rows".format(JOB_MAX_ROWS))
    rows = [_rows(row, (6,)) for row in value]
    lo, hi = [float(v) for v in q_limits[0]], [float(v) for v in q_limits[1]]
    for row in rows:
        if any(not lo[j] <= row[j] <= hi[j] for j in range(6)):
            raise ValueError("a target outside the joint limits")
    return rows


def check(obj, n, k, q_limits):
    """(tip, camera, job, job_warm, job_serviced) as nested lists of floats from a decoded submission; ValueError
    otherwise."""
    if not isinstance(obj, dict) or set(obj) != set(KEYS):
        raise ValueError("a submission has exactly the keys tip, camera, job, job_warm and job_serviced")
    try:
        tip = _rows(obj["tip"], (n, 3))
    except ValueError as exc:
        raise ValueError("tip: {}".format(exc))
    try:
        camera = _rows(obj["camera"], (n, k, 3))
    except ValueError as exc:
        raise ValueError("camera: {}".format(exc))
    jobs = []
    for key in JOBS:
        try:
            jobs.append(check_job(obj[key], q_limits))
        except ValueError as exc:
            raise ValueError("{}: {}".format(key, exc))
    return (tip, camera) + tuple(jobs)


def parse(data, n, k, q_limits):
    """(tip, camera, job, job_warm, job_serviced) from a submission file's bytes; ValueError on anything else."""
    if len(data) > MAX_BYTES:
        raise ValueError("larger than {} bytes".format(MAX_BYTES))
    try:
        obj = json.loads(data.decode("utf-8"), parse_constant=_no_constant, object_pairs_hook=_no_duplicates)
    except (UnicodeDecodeError, ValueError, RecursionError) as exc:
        raise ValueError("not JSON, or a key twice ({})".format(type(exc).__name__))
    return check(obj, n, k, q_limits)


def flatten(tip, camera, job=(), job_warm=(), job_serviced=()):
    """The submission as one flat list (a convenience; neither the lab nor the grader takes this form): the tip row
    by row, then the camera points, then the numbers of the cold and the warm job's rows (each 0 to JOB_MAX_ROWS),
    then the cold job's rows, the warm job's rows and the serviced job's rows (0 to JOB_MAX_ROWS)."""
    out = []
    for row in tip:
        out += [float(v) for v in row]
    for rows in camera:
        for row in rows:
            out += [float(v) for v in row]
    out += [float(len(job)), float(len(job_warm))]
    for row in list(job) + list(job_warm) + list(job_serviced):
        out += [float(v) for v in row]
    return out


def unflatten(flat, n, k):
    """(tip, camera, job, job_warm, job_serviced) from a flat list: 3 n + 3 n k numbers, the cold and the warm
    job's numbers of rows m and w, the m and w rows, then the serviced job's rows (0 or more)."""
    tip = [flat[3 * i:3 * i + 3] for i in range(n)]
    base = 3 * n
    camera = [[flat[base + 3 * (k * i + j):base + 3 * (k * i + j) + 3] for j in range(k)] for i in range(n)]
    m, w = int(flat[3 * n * (1 + k)]), int(flat[3 * n * (1 + k) + 1])
    rest = flat[3 * n * (1 + k) + 2:]
    rows = [rest[6 * r:6 * r + 6] for r in range(len(rest) // 6)]
    return tip, camera, rows[:m], rows[m:m + w], rows[m + w:]

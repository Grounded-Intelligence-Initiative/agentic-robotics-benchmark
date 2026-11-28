"""calib: the host software's calibration file config.yaml and the host's own count formula (payload-safe).

The same file ships with the format checker and with the grader. It holds no number of the photographed unit.

The file (the host software's own format, as its save() writes it):
    motor_ids: [0, 1, ..., 12]            the list of the 13 servo ids, in this order
    home_offsets: {0: <rad>, ..., 12: <rad>}   per servo id the raw position (radians) at which the joint angle is 0
    joint_signs:  {0: 1, ..., 12: -1}          per servo id +1 or -1
    joint_lower_limits / joint_upper_limits    optional; accepted and ignored (the travel is read from robot.xml)
  home_offsets and joint_signs may be a mapping whose keys are the ints 0 .. 12 or their decimal strings "0" .. "12"
  (each id exactly once) or a list of exactly 13 numbers in motor_ids order. Values are finite numbers (a YAML
  boolean is not a number); a sign is numerically +1 or -1; |home offset| <= MAX_ABS_HOME_RAD. Any other top-level
  key is ignored, as the host software ignores it.
  Refused before the file is parsed: a file above CONFIG_MAX_BYTES, a YAML node marker ("&", "*") outside
  quoted strings and comments, "[" / "{" nesting deeper than MAX_FLOW_DEPTH outside quoted strings, a line indented
  by more than MAX_INDENT characters. Then the file is read with yaml.safe_load (a duplicated key is refused).

The host's formula (D = 2 pi / 4096 radians per count):
    reading:  raw_i = k_i x D                  (k_i the signed Present Position count)
              q_i   = (raw_i - home_i) x sign_i
    command:  raw_i = sign_i x q_i + home_i     (radians)
              goal_i = int(round(raw_i / D))    (Python's round: halves to even)

    calib = load_calibration("submission/config.yaml")      # raises CalibError
    q = reading_to_model(calib, counts)                      # 13 angles (rad), motor order
    goal = model_to_goal(calib, {5: -0.4, 6: 0.1})           # servo id -> goal count
"""
import math
import os

import numpy as np

try:
    from . import contract as C
except ImportError:  # loaded as a plain module beside contract.py
    import contract as C

D = 2.0 * math.pi / C.COUNTS_PER_TURN          # radians per count (the host's position unit)
N_SERVOS = len(C.MOTOR_IDS)
CONFIG_MAX_BYTES = 64 * 1024
MAX_FLOW_DEPTH = 4
MAX_INDENT = 16
MAX_ABS_HOME_RAD = 1000.0
REQUIRED_KEYS = ("motor_ids", "home_offsets", "joint_signs")
IGNORED_KEYS = ("joint_lower_limits", "joint_upper_limits")
RULE = 9                                        # the gate rule these checks belong to


class CalibError(Exception):
    """config.yaml breaks a rule of the format (rule 9 of the gate)."""

    def __init__(self, message, rule=RULE):
        super().__init__("rule %d: config.yaml: %s" % (rule, message))
        self.rule = rule
        self.reason = message


class Calib(object):
    """home (rad) and sign (+1.0 / -1.0) per servo, in motor order 0 .. 12."""

    __slots__ = ("home", "sign")

    def __init__(self, home, sign):
        home = np.array(home, dtype=np.float64).reshape(N_SERVOS)
        sign = np.array(sign, dtype=np.float64).reshape(N_SERVOS)
        home.setflags(write=False)
        sign.setflags(write=False)
        self.home = home
        self.sign = sign


# ------------------------------------------------------------------------------------------------ before parsing
_TOKEN_START = set(" \t[{,:-?")


def precheck_text(text):
    """The rules applied before the file is parsed: a list of reasons (empty = passes)."""
    out = []
    depth, worst = 0, 0
    quote = None                     # None, "'" or '"'
    marker = None
    for lineno, line in enumerate(text.split("\n"), 1):
        if quote is None:
            stripped = line.lstrip(" \t")
            indent = len(line) - len(stripped)
            if stripped and not stripped.startswith("#") and indent > MAX_INDENT:
                out.append("line %d is indented by %d characters (at most %d)" % (lineno, indent, MAX_INDENT))
        i, n = 0, len(line)
        prev = " "
        while i < n:
            ch = line[i]
            if quote == '"':
                if ch == "\\":
                    i += 2
                    continue
                if ch == '"':
                    quote = None
            elif quote == "'":
                if ch == "'":
                    if i + 1 < n and line[i + 1] == "'":
                        i += 2
                        continue
                    quote = None
            else:
                if ch == "#" and (i == 0 or prev in " \t"):
                    break                                   # a comment runs to the end of the line
                if ch in "'\"" and (i == 0 or prev in _TOKEN_START):
                    quote = ch
                elif ch in "&*" and marker is None:
                    marker = (lineno, ch)
                elif ch in "[{":
                    depth += 1
                    worst = max(worst, depth)
                elif ch in "]}":
                    depth = max(0, depth - 1)
            prev = ch
            i += 1
    if marker is not None:
        out.append('a YAML node marker "%s" on line %d (node markers are not allowed)'
                   % (marker[1], marker[0]))
    if worst > MAX_FLOW_DEPTH:
        out.append('"[" / "{" nested %d deep (at most %d)' % (worst, MAX_FLOW_DEPTH))
    return out


# ------------------------------------------------------------------------------------------------ parsing
def _loader():
    import yaml

    class _NoDuplicates(yaml.SafeLoader):
        pass

    def construct_mapping(loader, node, deep=False):
        loader.flatten_mapping(node)
        seen = []
        for key_node, _ in node.value:
            key = loader.construct_object(key_node, deep=True)
            try:
                hash(key)
            except TypeError:
                raise yaml.constructor.ConstructorError(None, None, "a key that is not a scalar",
                                                        key_node.start_mark)
            if any(type(key) is type(k) and key == k for k in seen):
                raise yaml.constructor.ConstructorError(None, None, "the key %r appears twice" % (key,),
                                                        key_node.start_mark)
            seen.append(key)
        return yaml.SafeLoader.construct_mapping(loader, node, deep=deep)

    _NoDuplicates.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, construct_mapping)
    return yaml, _NoDuplicates


def _has_node_marker(yaml, text):
    for ev in yaml.parse(text, Loader=yaml.SafeLoader):
        if isinstance(ev, yaml.AliasEvent) or getattr(ev, "anc" "hor", None) is not None:
            return True
    return False


def _number(v, what):
    if isinstance(v, bool) or not isinstance(v, (int, float)):
        raise CalibError("%s is %r, not a number" % (what, v))
    try:
        x = float(v)
    except (OverflowError, ValueError):
        raise CalibError("%s is not a finite number" % what)
    if not math.isfinite(x):
        raise CalibError("%s is not a finite number" % what)
    return x


def _motor_values(val, name):
    """The 13 values of a per-servo field (a mapping by id or a list in motor_ids order), motor order."""
    if isinstance(val, dict):
        by_id = {}
        for key, v in val.items():
            if isinstance(key, bool):
                raise CalibError("%s: the key %r is not a servo id" % (name, key))
            if isinstance(key, int) and key in C.MOTOR_IDS:
                mid = key
            elif isinstance(key, str) and key in tuple(str(i) for i in C.MOTOR_IDS):
                mid = int(key)
            else:
                raise CalibError("%s: the key %r is not a servo id 0 .. %d (an int or its decimal string)"
                                 % (name, key, N_SERVOS - 1))
            if mid in by_id:
                raise CalibError("%s: servo id %d is given twice (as an int and as a string)" % (name, mid))
            by_id[mid] = v
        missing = [i for i in C.MOTOR_IDS if i not in by_id]
        if missing:
            raise CalibError("%s: no value for servo id(s) %s" % (name, missing))
        return [_number(by_id[i], "%s[%d]" % (name, i)) for i in C.MOTOR_IDS]
    if isinstance(val, list):
        if len(val) != N_SERVOS:
            raise CalibError("%s: a list needs exactly %d values, got %d" % (name, N_SERVOS, len(val)))
        return [_number(v, "%s[%d]" % (name, i)) for i, v in enumerate(val)]
    raise CalibError("%s must be a mapping by servo id or a list of %d numbers" % (name, N_SERVOS))


def parse_calibration(data):
    """Calib from the bytes of a config.yaml; raises CalibError."""
    if len(data) > CONFIG_MAX_BYTES:
        raise CalibError("%d bytes (at most %d)" % (len(data), CONFIG_MAX_BYTES))
    try:
        text = data.decode("utf-8")
    except UnicodeDecodeError:
        raise CalibError("not UTF-8 text")
    pre = precheck_text(text)
    if pre:
        raise CalibError("; ".join(pre))
    yaml, loader = _loader()
    try:
        if _has_node_marker(yaml, text):
            raise CalibError("a YAML node marker (\"&\" or \"*\") is not allowed")
        doc = yaml.load(text, Loader=loader)
    except CalibError:
        raise
    except (yaml.YAMLError, RecursionError, MemoryError, ValueError, TypeError, OverflowError) as exc:
        raise CalibError("does not parse as YAML (%s)" % (str(exc).splitlines()[0][:160] if str(exc) else
                                                          type(exc).__name__))
    if not isinstance(doc, dict):
        raise CalibError("the top level must be a mapping with the keys %s" % ", ".join(REQUIRED_KEYS))
    for key in REQUIRED_KEYS:
        if key not in doc:
            raise CalibError("the key %s is missing" % key)
    ids = doc["motor_ids"]
    if (not isinstance(ids, list) or len(ids) != N_SERVOS
            or any(isinstance(x, bool) or not isinstance(x, int) for x in ids) or list(ids) != list(C.MOTOR_IDS)):
        raise CalibError("motor_ids must be exactly the list %s" % list(C.MOTOR_IDS))
    home = _motor_values(doc["home_offsets"], "home_offsets")
    for i, h in enumerate(home):
        if abs(h) > MAX_ABS_HOME_RAD:
            raise CalibError("home_offsets[%d] = %r rad: |home offset| must be at most %g rad"
                             % (i, h, MAX_ABS_HOME_RAD))
    sign = _motor_values(doc["joint_signs"], "joint_signs")
    for i, s in enumerate(sign):
        if s not in (1.0, -1.0):
            raise CalibError("joint_signs[%d] = %r: a sign is +1 or -1" % (i, s))
    return Calib(home, sign)


def load_calibration(path):
    """Calib from a config.yaml file; raises CalibError (a missing, irregular or oversized file included)."""
    path = str(path)
    if not os.path.isfile(path) or os.path.islink(path):
        raise CalibError("missing (a regular file in the submission root)")
    size = os.path.getsize(path)
    if size > CONFIG_MAX_BYTES:
        raise CalibError("%d bytes (at most %d)" % (size, CONFIG_MAX_BYTES))
    with open(path, "rb") as fh:
        data = fh.read(CONFIG_MAX_BYTES + 1)
    return parse_calibration(data)


# ------------------------------------------------------------------------------------------------ the formula
def reading_to_model(calib, k):
    """The model's 13 angles (rad, motor order) at a read-out of 13 Present Position counts."""
    k = np.asarray(k)
    if k.shape != (N_SERVOS,):
        raise ValueError("a read-out has %d counts" % N_SERVOS)
    raw = k.astype(np.float64) * D
    return (raw - calib.home) * calib.sign


def model_to_goal(calib, q_m):
    """Goal counts {servo id: int} for model angles {servo id: rad} (the servos that are commanded)."""
    out = {}
    for mid in sorted(q_m):
        i = int(mid)
        if i not in C.MOTOR_IDS:
            raise ValueError("not a servo id: %r" % (mid,))
        raw = float(q_m[mid]) * float(calib.sign[i]) + float(calib.home[i])
        out[i] = int(round(raw / D))
    return out

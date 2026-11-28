"""The world's hidden constants, read from the plant's truth_constants.json (names and values only).

The draw ranges that are not printed (the start temperature's draw range and the touch gains: block "unit_draw";
the fixed spring constants of the shells and blocks: "specimens"; the carry segments per code, the decline
segment, the job mix, the zero residuals' probability, the true home offsets' band and the ceilings' rule: "world")
live in that one file; no shipped module of the world holds them as literals. load() returns them in the world's
own form:
    t_start_c, touch_gain, residual_p (or None), spring_k_n_per_mm {brittle, full_load}, carry_kg {code: [lo, hi]},
    decline_kg, mix {jobs_per_shift, decline_per_shift, class_p}, home_true_counts (or None),
    ceiling {factor, floor_n, soft_travel_factor, hold_slip_mm}
"""
import importlib.util
import json
import os

KEYS = ("t_start_c", "touch_gain", "residual_p", "spring_k_n_per_mm", "carry_kg", "decline_kg", "mix",
        "home_true_counts", "ceiling")

_CACHE = {}


def default_path():
    """plant/truth_constants.json, found from the plant package on sys.path."""
    spec = importlib.util.find_spec("plant")
    if spec is None or not spec.submodule_search_locations:
        raise FileNotFoundError("the plant package is not on sys.path")
    return os.path.join(list(spec.submodule_search_locations)[0], "truth_constants.json")


def adapt(data):
    """The world's form of a truth-constants dict (raises KeyError naming what is missing)."""
    w = data.get("world")
    if not isinstance(w, dict):
        raise KeyError("no 'world' block in the truth constants")
    try:
        ud, sp = data["unit_draw"], data["specimens"]
        cd = w["jobs_carry_decline_per_shift"]
        out = {"t_start_c": list(ud["start_temperature_c"]), "touch_gain": list(ud["touch_gain"]),
               "spring_k_n_per_mm": {k: float(v) / 1000.0 for k, v in sp["spring_n_per_m"].items()},
               "carry_kg": {k: list(v) for k, v in w["carry_segments_kg"].items()},
               "decline_kg": list(w["decline_segment_kg"]),
               "mix": {"jobs_per_shift": int(cd[0]) + int(cd[1]), "decline_per_shift": int(cd[1]),
                       "class_p": dict(w["class_probability"])},
               "ceiling": {"factor": float(w["ceiling_factor"]), "floor_n": float(w["ceiling_floor_n"]),
                           "soft_travel_factor": float(w["soft_limit_factor"]),
                           "hold_slip_mm": 1000.0 * float(w["least_squeeze_slip_m"])}}
    except KeyError as e:
        raise KeyError("the truth constants lack %s (the world's draw needs it)" % e) from None
    # the zero residuals' probability: only the unit draw needs it (draw.unit refuses a block without it)
    out["residual_p"] = float(w["residual_p"]) if "residual_p" in w else None
    out["home_true_counts"] = list(w["home_true_counts"]) if "home_true_counts" in w else None
    return out


def load(path=None):
    """The world's constants as a dict (a fresh copy)."""
    p = os.path.abspath(path or default_path())
    if p not in _CACHE:
        with open(p, "r", encoding="utf-8") as fh:
            _CACHE[p] = json.dumps(adapt(json.load(fh)), sort_keys=True)
    return json.loads(_CACHE[p])

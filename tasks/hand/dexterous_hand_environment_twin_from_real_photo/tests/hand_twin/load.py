"""The public payload modules and the one pipeline from a twin folder to its settled pieces.

    P = payload()                           # {"twin_gate", "twin_sim", "hand_instrument"} modules
    cfg_i, env = instrument_config(), envelope(gcfg)   # env None when grader_config "envelope" is false
    tw = gate_and_settle(folder, held)      # TwinRun: gate result, settle output, model and data at the settle's end
    pieces, top_T = settled_pieces(tw)      # {key: [V (n, 3) mm]} in the twin's own support-top frame; 4x4 (m)

The payload modules come from $HAND_TWIN_PAYLOAD_DIR or tests/setup_payload (byte copies of the agent's payload), and
nothing else: the grader and the agent's check_twin.py run the same gate, the same settle and the same pieces:
    gate -> physics_model -> settle(held) -> data at qpos_end -> pieces_from_model -> support_frame
The grader then registers the pieces onto the true support frame (register.py) and plans on them; it never renders.
"""
import importlib
import json
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_MODS = {}
MODULES = ("twin_sim", "twin_gate", "hand_instrument")


def payload_dir():
    env = os.environ.get("HAND_TWIN_PAYLOAD_DIR", "")
    cand = env or os.path.join(os.path.dirname(_HERE), "setup_payload")
    if not os.path.isfile(os.path.join(cand, "twin_gate.py")):
        raise RuntimeError("the payload modules are missing: no twin_gate.py in %s" % cand)
    return os.path.abspath(cand)


def payload():
    """The payload modules, imported once from payload_dir()."""
    if not _MODS:
        d = payload_dir()
        if d not in sys.path:
            sys.path.insert(0, d)
        for nm in MODULES:
            _MODS[nm] = importlib.import_module(nm)
    return _MODS


def payload_json(name):
    with open(os.path.join(payload_dir(), name), encoding="utf-8") as fh:
        return json.load(fh)


def instrument_config():
    return payload_json("instrument_config.json")


ENVELOPE_FILE = "hand_envelope.json"


def envelope(gcfg):
    """The hand envelope the instrument plans and executes with, by the grader configuration's "envelope" switch:
    true = the payload's hand_envelope.json (it must be there); false = None (no envelope), and then the payload must
    not hold that file either (the agent's check_twin.py uses the file when it is there, so the grader and the agent's
    rehearsal would differ). Anything else is refused."""
    on = gcfg.get("envelope")
    has = os.path.isfile(os.path.join(payload_dir(), ENVELOPE_FILE))
    if on is True:
        if not has:
            raise RuntimeError("the grader configuration says envelope true but the payload has no %s" % ENVELOPE_FILE)
        return payload_json(ENVELOPE_FILE)
    if on is False:
        if has:
            raise RuntimeError("the grader configuration says envelope false but the payload holds %s" % ENVELOPE_FILE)
        return None
    raise RuntimeError("the grader configuration's envelope must be true or false, not %r" % (on,))


class TwinRun:
    """A twin through the gate and the settle."""

    def __init__(self):
        self.gate = None                # the gate's GateResult
        self.report = {}
        self.ok = False                 # the gate passed
        self.settle = None              # twin_sim.settle's output
        self.blowup = False
        self.model = None
        self.data = None
        self.roles = None
        self.reasons = []


def gate(folder):
    """The gate on a submission folder: (GateResult, report dict)."""
    TG = payload()["twin_gate"]
    res = TG.gate(folder)
    return res, dict(res.report or {})


def gate_twin(folder):
    """The gate on a twin folder -> TwinRun (ok, report, reasons, roles)."""
    run = TwinRun()
    res, rep = gate(folder)
    run.gate, run.report, run.ok = res, rep, bool(res.ok)
    run.reasons = [str(r) for r in (rep.get("reasons") or [])]
    if run.ok:
        run.roles = res.roles
    return run


def settle_twin(run, held=()):
    """The physics model (the gate's grader copy: excluded bodies deleted) and the settle; the settle's held list is
    the contract's, restricted to the things this twin has. Fills run.settle, run.blowup, run.model, run.data."""
    import mujoco
    TS = payload()["twin_sim"]
    model = run.gate.physics_model()
    keys = set(_role_keys(run.roles))
    held_here = tuple(k for k in held if k in keys)
    out = TS.settle(model, run.roles, held=held_here, config=None)
    run.settle = out
    run.blowup = bool(out.get("blowup"))
    if run.blowup:
        run.reasons.append("the settle blew up")
        return run
    data = mujoco.MjData(model)
    q = np.asarray(out["qpos_end"], dtype=np.float64)
    if q.shape != (model.nq,) or not np.all(np.isfinite(q)):
        run.blowup = True
        run.reasons.append("the settle ended in a state that is not finite")
        return run
    data.qpos[:] = q
    mujoco.mj_forward(model, data)
    run.model, run.data = model, data
    return run


def gate_and_settle(folder, held=()):
    """gate_twin, then settle_twin when the gate passed."""
    run = gate_twin(folder)
    return settle_twin(run, held) if run.ok else run


def _role_keys(roles):
    for attr in ("key_to_body", "keys"):
        v = getattr(roles, attr, None)
        if isinstance(v, dict):
            return list(v)
        if isinstance(v, (list, tuple)):
            return list(v)
    raise TypeError("the payload's Roles exposes no key_to_body")


def inv_T(T):
    T = np.asarray(T, dtype=np.float64)
    R, p = T[:3, :3], T[:3, 3]
    out = np.eye(4)
    out[:3, :3] = R.T
    out[:3, 3] = -R.T @ p
    return out


def settled_pieces(run):
    """({key: [V (n, 3) mm]} in the twin's support-top frame, world_from_top 4x4 in metres)."""
    TS = payload()["twin_sim"]
    pieces_w = TS.pieces_from_model(run.model, run.data, run.roles)
    top_T = np.asarray(TS.support_frame(run.model, run.data, run.roles), dtype=np.float64)
    if top_T.shape != (4, 4) or not np.all(np.isfinite(top_T)):
        raise ValueError("the support frame is not a finite 4x4 transform")
    top_from_world = inv_T(top_T)
    R, p_mm = top_from_world[:3, :3], top_from_world[:3, 3] * 1000.0
    out = {}
    for key, pieces in pieces_w.items():
        out[key] = [np.asarray(V, dtype=np.float64).reshape(-1, 3) @ R.T + p_mm for V in pieces]
    return out, top_T


def settle_summary(run):
    """{key: {"disp_mm", "rot_deg"}} of the settle (the grade dump's block)."""
    out = {}
    s = run.settle or {}
    for k in sorted((s.get("disp_mm") or {})):
        out[k] = {"disp_mm": float(s["disp_mm"][k]), "rot_deg": float((s.get("rot_deg") or {}).get(k, 0.0))}
    return out

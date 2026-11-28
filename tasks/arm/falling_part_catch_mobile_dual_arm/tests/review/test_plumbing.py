"""CPU test of the wire between the grader and the cell server, with a fake cell (no simulator)."""
import hashlib, json, os, subprocess, sys, textwrap
HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__))); sys.path.insert(0, HERE)
import numpy as np
from sim_client import SimClient, SimError

FAKE = textwrap.dedent('''
    import sys, os, numpy as np
    sys.path.insert(0, %r)
    import cell_server as S
    class Fake:
        def __init__(s, station, seed, parts): s.t = 0; s.parts = parts; s.frames = 0
        def observe(s, cams=()):
            o = {"t": s.t, "done": s.t >= 5, "proprio": np.arange(61) * 0.01, "odometry": [0, 0, 0]}
            for c in cams: o[c] = {"rgb": np.full((4, 6, 3), 7, np.uint8), "depth": np.full((4, 6), 1.25, np.float32), "K": np.eye(3), "pos_base": np.zeros(3), "rot_base": np.eye(3)}
            return o
        def step(s, a, cams=(), late=0): s.t += late + 1 + 3 * len(cams); s.frames += len(cams); return s.observe(cams)
        def truth(s): return {"phase": "live"}
        def summary(s): return {"score": 0.25, "steps": s.t, "frames_charged": s.frames}
    S.serve(lambda st, seed, parts: Fake(st, seed, parts))
''') % HERE


def test_wire(tmp_path):
    f = tmp_path / "fake.py"; f.write_text(FAKE)
    env = dict(os.environ, CELL_TOKEN_SHA256=hashlib.sha256(b"secret").hexdigest())
    sim = SimClient(str(f), python=sys.executable, env=env)
    obs = sim.reset("drop", 3, 20, cameras=("head",))
    assert obs["t"] == 0 and len(obs["proprio"]) == 61 and obs["cameras"]["head"]["shape"] == [4, 6]
    import base64
    d = np.frombuffer(base64.b64decode(obs["cameras"]["head"]["depth_mm_u16"]), "<u2"); assert d[0] == 1250
    obs = sim.step([0.0] * 23, ["head", "right_wrist"]); assert obs["t"] == 7 and set(obs["cameras"]) == {"head", "right_wrist"}
    late = sim.step([0.0] * 23, [], late=4); assert late["t"] == 12      # four steps went by before the action, then its own
    for bad in ([0.0] * 22, [float("nan")] * 23, "x"):
        try: sim.call(op="step", action=bad, cameras=[]); raise AssertionError("accepted a bad action")
        except SimError: pass
    assert sim.truth("secret")["phase"] == "live"
    try: sim.truth("wrong"); raise AssertionError("truth without the token")
    except SimError: pass
    assert sim.summary()["frames_charged"] == 2
    sim.close()


def test_parse_reply():
    import verify as V
    from robotics_grader import SolverAborted
    a, c = V.parse_reply([0] * 23 + [1, 0, 0.9]); assert len(a) == 23 and c == ["head", "right_wrist"]
    for bad in ([0] * 25, [float("inf")] * 26, {"action": [0] * 26}, "abc", None):
        try: V.parse_reply(bad); raise AssertionError(bad)
        except SolverAborted: pass

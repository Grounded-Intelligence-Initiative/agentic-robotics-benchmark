"""The cell as a process: JSON lines on stdin/stdout, run by the simulator interpreter.

Requests: {"op": "reset", "station", "seed", "parts"} | {"op": "step", "action": [23], "cameras": [...]} |
{"op": "truth", "token"} | {"op": "summary"} | {"op": "close"}. Replies carry "ok". Images travel as base64 PNG (RGB) and
base64 little-endian uint16 millimetres (depth). Nothing but the reply line is written to stdout.
The same file serves the grader, the reference and the agent's lab; `truth` answers only the holder of the token."""
import base64, hashlib, io, json, os, sys

# The simulator's own chatter must not reach the wire. Kit also writes from native code, straight to file descriptor 1,
# so the wire gets a private copy of that descriptor and descriptor 1 itself is pointed at stderr.
REAL_OUT = os.fdopen(os.dup(1), "w", buffering=1); os.dup2(2, 1); sys.stdout = sys.stderr
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
TOKEN_SHA = os.environ.get("CELL_TOKEN_SHA256", "")


def _png(rgb):
    from PIL import Image
    buf = io.BytesIO(); Image.fromarray(rgb).save(buf, format="PNG", compress_level=1); return base64.b64encode(buf.getvalue()).decode()


def pack(obs):
    import numpy as np
    out = {"t": int(obs["t"]), "done": bool(obs["done"]), "proprio": [float(v) for v in obs["proprio"]], "odometry": [float(v) for v in obs["odometry"]], "cameras": {}}
    for cam in ("head", "left_wrist", "right_wrist"):
        if cam in obs:
            c = obs[cam]; d = np.nan_to_num(np.asarray(c["depth"], np.float32), nan=0.0, posinf=0.0, neginf=0.0)
            mm = np.clip(d * 1000.0, 0, 65535).astype("<u2")
            out["cameras"][cam] = {"rgb_png": _png(c["rgb"]), "depth_mm_u16": base64.b64encode(mm.tobytes()).decode(), "shape": list(mm.shape),
                                   "K": np.asarray(c["K"]).tolist(), "pos_base": np.asarray(c["pos_base"]).tolist(), "rot_base": np.asarray(c["rot_base"]).tolist()}
    return out


def serve(make_cell):
    cell = None
    for line in sys.stdin:
        line = line.strip()
        if not line: continue
        try:
            req = json.loads(line); op = req.get("op")
            if op == "reset":
                if cell is not None: raise RuntimeError("one shift per process")
                privileged = bool(TOKEN_SHA) and hashlib.sha256(str(req.get("token", "")).encode()).hexdigest() == TOKEN_SHA
                cell = make_cell(req["station"], int(req["seed"]), int(req.get("parts", 20)), privileged)   # assisted grasp: the reference only
                rep = {"ok": True, "obs": pack(cell.observe(tuple(req.get("cameras", ()))))}
            elif op == "step":
                a = req["action"]
                if not (isinstance(a, list) and len(a) == 23 and all(isinstance(v, (int, float)) and v == v and abs(v) < 1e6 for v in a)): raise ValueError("action must be 23 finite numbers")
                rep = {"ok": True, "obs": pack(cell.step(a, tuple(c for c in req.get("cameras", ()) if c in ("head", "left_wrist", "right_wrist")), late=max(0, min(900, int(req.get("late", 0))))))}
            elif op == "truth":
                if not TOKEN_SHA or hashlib.sha256(str(req.get("token", "")).encode()).hexdigest() != TOKEN_SHA: raise PermissionError("truth is not available")
                rep = {"ok": True, "truth": cell.truth()}
            elif op == "summary":
                rep = {"ok": True, "summary": cell.summary()}
            elif op == "close":
                REAL_OUT.write(json.dumps({"ok": True}) + "\n"); REAL_OUT.flush(); break
            else: raise ValueError("unknown op")
        except Exception as exc:  # noqa: BLE001 - a bad request is answered, never fatal
            rep = {"ok": False, "error": "{}: {}".format(type(exc).__name__, exc)[:300]}
        REAL_OUT.write(json.dumps(rep, default=float) + "\n"); REAL_OUT.flush()


if __name__ == "__main__":
    def make_cell(station, seed, parts, assisted=False):
        import cell_env
        return cell_env.Cell(station, seed, n_parts=parts, assisted=assisted)
    serve(make_cell)
    try:
        import omnigibson as og; og.shutdown()
    except Exception: pass

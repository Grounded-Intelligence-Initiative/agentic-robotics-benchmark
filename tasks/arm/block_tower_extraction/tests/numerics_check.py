"""The numerics fingerprint of this machine (run by verify.py in a fresh interpreter, `python3 -I -B`): a short
scripted run of the cell (Cartesian moves, a turntable turn, a push of a block with the pusher rod) and a set of NumPy, SciPy and OpenCV results.
Prints one JSON line {"fingerprint", "parts"}; tests/numerics.json records the value the reference was measured with.
The cell's own numerics run under the image's pin (OPENBLAS_CORETYPE=Haswell, OPENBLAS_L2_SIZE=256, NumPy's AVX-512
loops off, one BLAS thread; tests/test.sh and the venv's .pth)."""
import hashlib
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)


def h(b):
    return hashlib.sha256(b).hexdigest()


def main():
    import numpy as np
    parts = {}
    rng = np.random.default_rng(12345)
    a = rng.standard_normal((400, 400))
    b = rng.standard_normal((400, 7))
    parts["blas"] = h((a @ a.T).tobytes() + np.linalg.solve(a + 400 * np.eye(400), b).tobytes())
    x = np.linspace(-3, 3, 20001)
    parts["ufunc"] = h(np.arctan2(x, 1.3 - x).tobytes() + np.exp(x).tobytes() + np.log1p(x * x).tobytes()
                       + np.power(np.abs(x) + 1e-3, 1.7).tobytes() + np.arcsin(np.clip(x / 3, -1, 1)).tobytes())
    import scipy.linalg
    parts["scipy"] = h(scipy.linalg.expm(a[:50, :50] * 0.01).tobytes())
    import cv2
    img = (rng.random((240, 320)) * 255).astype(np.uint8)
    parts["cv2"] = h(cv2.GaussianBlur(img, (7, 7), 1.5).tobytes())
    from towercell.server import CellServer, default_panda_dir
    from towercell.tower import draw_tower
    panda = os.environ.get("BT_PANDA_DIR") or os.path.join(os.environ.get("ARB_ROOT_PAYLOAD", "/opt/arb"), "given",
                                                             "robot", "panda")
    srv = CellServer(draw_tower(7), duration_s=120.0, panda_dir=panda if os.path.isdir(panda) else default_panda_dir())
    c = srv.cell
    q = [0.0, 1.0, 0.0, 0.0]
    srv.m_step(n=150, target={"pos": [0.40, 0.0, 0.42], "quat": q}, speed=0.3, turntable=0.4)
    srv.m_step(n=100, target={"pos": [0.53, -0.0265, 0.3266], "quat": q}, frame="rod_tip", speed=0.3, pusher=True)
    srv.m_step(n=300, target={"pos": [0.575, -0.0265, 0.3266], "quat": q}, frame="rod_tip", speed=0.01)
    parts["cell"] = h(c.d.qpos.tobytes() + c.d.qvel.tobytes())
    parts["cell_ik"] = h(json.dumps(srv.m_ik([0.5, 0.1, 0.3], q), sort_keys=True).encode())
    fp = h(json.dumps(parts, sort_keys=True).encode())
    print(json.dumps({"fingerprint": fp, "parts": parts}))


if __name__ == "__main__":
    main()

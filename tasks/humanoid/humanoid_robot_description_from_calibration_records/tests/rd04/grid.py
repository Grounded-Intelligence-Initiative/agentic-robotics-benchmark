"""The ankle map of calibration.yaml: per foot 19 x 19 nodes over the two ankle motors' q,
(A, B) in [-0.787, 0.787] rad, to the model's foot hinge angles (pitch, roll); bilinear between nodes; outside the
extent the bilinear formula of the nearest edge cell (cell index clamped to 0..17) is extended. The inverse:
Newton from the firmware's linear map's inverse, central finite-difference Jacobian with step 1e-6, at most 40
iterations to max |residual| < 1e-12; it fails (the posture scores 0) when |det J| < 1e-12, a value is not finite, or
the final residual exceeds 1e-9 (the acceptance sheet prints these rules)."""
from __future__ import annotations

import math

import numpy as np

from .vendor_chain import K_PITCH, K_ROLL

NODES = 19
EXTENT = 0.787                      # rad, fixed (the literal, not 2.02 x 0.35 + 0.8 x 0.10 in floating point)
STEP = 1.574 / 18.0                 # rad between nodes (node i at -0.787 + i x 1.574 / 18)
NODE_V = np.array([-EXTENT + i * 1.574 / 18.0 for i in range(NODES)])
# inverse numerics (printed in the acceptance sheet)
INV_MAXIT = 40
INV_TOL = 1e-12
INV_FD = 1e-6
INV_DET_MIN = 1e-12
INV_RESID_MAX = 1e-9


class InverseFail(Exception):
    """the inverse of a foot's map failed by one of its rules (code: 'det', 'nonfinite' or 'residual')"""

    def __init__(self, code: str, detail: str = ""):
        super().__init__(f"{code}: {detail}")
        self.code = code


class AnkleMap:
    """one foot's map: nodes P[i][j], R[i][j] = (pitch, roll) at (A_i, B_j), rad"""

    __slots__ = ("P", "R")

    def __init__(self, P, R):
        P = np.array(P, float); R = np.array(R, float)
        if P.shape != (NODES, NODES) or R.shape != (NODES, NODES):
            raise ValueError(f"ankle map nodes {P.shape} / {R.shape}, expected ({NODES}, {NODES})")
        self.P, self.R = P, R

    @classmethod
    def from_function(cls, fn):
        """nodes from fn(A, B) -> (pitch, roll) at every node (row index A, column index B)"""
        P = np.zeros((NODES, NODES)); R = np.zeros((NODES, NODES))
        for i, a in enumerate(NODE_V):
            for j, b in enumerate(NODE_V):
                P[i, j], R[i, j] = fn(float(a), float(b))
        return cls(P, R)

    @classmethod
    def linear(cls):
        """the firmware's linear map as nodes: pitch = (A - B) / (2 k_p), roll = -(A + B) / (2 k_r)"""
        return cls.from_function(lambda a, b: ((a - b) / (2.0 * K_PITCH), -(a + b) / (2.0 * K_ROLL)))

    def __call__(self, A, B):
        """(pitch, roll) at motor coordinates (A, B) (scalars); NaN when A or B is not finite"""
        A = float(A); B = float(B)
        if not (math.isfinite(A) and math.isfinite(B)):
            return float("nan"), float("nan")
        fa = (A - NODE_V[0]) / STEP
        fb = (B - NODE_V[0]) / STEP
        i = min(max(int(math.floor(fa)), 0), NODES - 2)
        j = min(max(int(math.floor(fb)), 0), NODES - 2)
        ta = fa - i; tb = fb - j
        out = []
        for G in (self.P, self.R):
            g00 = G[i, j]; g10 = G[i + 1, j]; g01 = G[i, j + 1]; g11 = G[i + 1, j + 1]
            out.append(float((1 - ta) * (1 - tb) * g00 + ta * (1 - tb) * g10 + (1 - ta) * tb * g01 + ta * tb * g11))
        return out[0], out[1]

    def inverse(self, p, r):
        """(A, B) with self(A, B) = (p, r); InverseFail by the inverse's rules (module docstring)"""
        p = float(p); r = float(r)
        if not (math.isfinite(p) and math.isfinite(r)):
            raise InverseFail("nonfinite", "target")
        x = np.array((K_PITCH * p - K_ROLL * r, -K_PITCH * p - K_ROLL * r), float)
        tgt = np.array((p, r), float)
        for _ in range(INV_MAXIT):
            f = np.array(self(x[0], x[1]), float) - tgt
            if not np.all(np.isfinite(f)):
                raise InverseFail("nonfinite", "residual")
            if np.max(np.abs(f)) < INV_TOL:
                break
            e = INV_FD
            J = np.zeros((2, 2))
            for k in range(2):
                dx = np.zeros(2); dx[k] = e
                J[:, k] = (np.array(self(*(x + dx)), float) - np.array(self(*(x - dx)), float)) / (2 * e)
            det = J[0, 0] * J[1, 1] - J[0, 1] * J[1, 0]
            if not np.all(np.isfinite(J)) or not math.isfinite(det):
                raise InverseFail("nonfinite", "jacobian")
            if abs(det) < INV_DET_MIN:
                raise InverseFail("det", f"|det J| = {abs(det):.3e}")
            x = x - np.linalg.solve(J, f)
            if not np.all(np.isfinite(x)):
                raise InverseFail("nonfinite", "step")
        f = np.array(self(x[0], x[1]), float) - tgt
        if not np.all(np.isfinite(f)):
            raise InverseFail("nonfinite", "final")
        res = float(np.max(np.abs(f)))
        if res > INV_RESID_MAX:
            raise InverseFail("residual", f"{res:.3e}")
        return float(x[0]), float(x[1])

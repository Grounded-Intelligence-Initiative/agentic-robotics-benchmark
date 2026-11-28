"""Generate the fixed 128-row station approach with the released G1 reference encoder."""

import argparse
from pathlib import Path

import numpy as np
from sonic_sim.decoder import DEFAULT_Q
from sonic_sim.encoder import ReferenceEncoder


def make_codes(encoder):
    duration, period, ramp = 108, 48, 15
    ticks = np.arange(duration + 65, dtype=float)
    phase = 2 * np.pi * (ticks / period + 0.875)
    weight = np.clip(ticks / ramp, 0, 1) * np.clip((duration - ticks) / ramp, 0, 1)
    q = np.broadcast_to(DEFAULT_Q, (len(ticks), 29)).copy()
    for sign, hip, shoulder in ((1, 0, 15), (-1, 6, 22)):
        foot_x = -sign * 0.229 * np.cos(phase)
        knee = 0.60 * np.maximum(sign * np.sin(phase), 0)
        q[:, hip] += -foot_x - knee / 2
        q[:, hip + 3] += knee
        q[:, hip + 4] += foot_x - knee / 2
        q[:, hip + 1] += 0.065 * np.sin(phase)
        q[:, hip + 5] -= 0.065 * np.sin(phase)
        q[:, shoulder] += 0.55 * foot_x
    q = DEFAULT_Q + weight[:, None] * (q - DEFAULT_Q)
    dq = 50 * np.gradient(q, axis=0)
    change = np.clip((ticks - 68) / 8, 0, 1)
    yaw = ((1 - change) * (-0.018) + change * 0.085) * weight
    zero = np.zeros_like(yaw)
    rotation = np.column_stack(
        (np.cos(yaw), -np.sin(yaw), np.sin(yaw), np.cos(yaw), zero, zero)
    )
    offsets = 5 * np.arange(10)
    return np.asarray(
        [
            encoder.codes_g1(q[t + offsets], dq[t + offsets], rotation[t + offsets])
            for t in range(len(q) - 45)
        ],
        dtype=np.uint8,
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--assets", type=Path, default=Path("/opt/sonic/assets"))
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    np.save(
        args.out,
        make_codes(ReferenceEncoder(args.assets / "policy/model_encoder.onnx")),
        allow_pickle=False,
    )

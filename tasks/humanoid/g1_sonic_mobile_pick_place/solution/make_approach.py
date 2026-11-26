"""Regenerate the approach codes from a procedural joint-space gait and NVIDIA's public encoder."""
import argparse
from pathlib import Path

import numpy as np

from sonic_sim.decoder import DEFAULT_Q
from sonic_sim.encoder import ReferenceEncoder


def make_codes(encoder):
    period, duration, ramp = 48, 108, 15
    ticks = np.arange(duration+65, dtype=float)
    phase = 2*np.pi*(ticks/period+.875)
    q = np.broadcast_to(DEFAULT_Q, (len(ticks), 29)).copy()
    for sign, hip, shoulder in ((1, 0, 15), (-1, 6, 22)):
        swing = np.maximum(sign*np.sin(phase), 0.)
        foot_x = -sign*.25*np.cos(phase)
        knee = .60*swing
        q[:, hip] += -foot_x-knee/2
        q[:, hip+3] += knee
        q[:, hip+4] += foot_x-knee/2
        q[:, hip+1] += .065*np.sin(phase)
        q[:, hip+5] -= .065*np.sin(phase)
        q[:, shoulder] += foot_x*.55
    weight = np.clip(ticks/ramp, 0., 1.)*np.clip((duration-ticks)/ramp, 0., 1.)
    q = DEFAULT_Q+weight[:, None]*(q-DEFAULT_Q)
    dq = np.gradient(q, axis=0)*50.
    anchor = np.broadcast_to([1, 0, 0, 1, 0, 0], (10, 6))
    rows = []
    for t in range(len(q)-45):
        future = t+np.arange(10)*5
        rows.append(encoder.codes_g1(q[future], dq[future], anchor))
    return np.array(rows, dtype=np.uint8)


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--encoder", type=Path, default=Path("/opt/sonic/assets/policy/model_encoder.onnx"))
    p.add_argument("--output", type=Path, required=True)
    args = p.parse_args()
    np.save(args.output, make_codes(ReferenceEncoder(args.encoder)), allow_pickle=False)

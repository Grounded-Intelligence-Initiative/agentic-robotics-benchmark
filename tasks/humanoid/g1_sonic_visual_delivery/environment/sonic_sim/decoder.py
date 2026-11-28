"""Pinned public GEAR-SONIC decoder and 50 Hz Unitree hardware-order mapping.

The deployment constants follow NVIDIA GR00T-WholeBodyControl commit
1e1ec4f5f27e6c3e03048f693bd7c60cce04afda (policy_parameters.hpp).
"""
import hashlib
from pathlib import Path

import numpy as np
import onnxruntime as ort


DECODER_REVISION = "6733128a3d8a523b1418b06bca3cdf61c8b0987f"
DECODER_SHA256 = "c7241a123eaa36b5d64bad19540efde93cac1ad443bd4572fd12ca99898118ed"
# Apply as hardware_array[..., MJ_TO_POLICY] and policy_array[..., POLICY_TO_MJ].
MJ_TO_POLICY = np.array(
    [
        0,
        6,
        12,
        1,
        7,
        13,
        2,
        8,
        14,
        3,
        9,
        15,
        22,
        4,
        10,
        16,
        23,
        5,
        11,
        17,
        24,
        18,
        25,
        19,
        26,
        20,
        27,
        21,
        28,
    ]
)
POLICY_TO_MJ = np.argsort(MJ_TO_POLICY)
DEFAULT_Q = np.array(
    [-0.312, 0, 0, 0.669, -0.363, 0] * 2
    + [0, 0, 0]
    + [0.2, 0.2, 0, 0.6, 0, 0, 0]
    + [0.2, -0.2, 0, 0.6, 0, 0, 0],
    dtype=np.float64,
)
# Upstream motor types ordered 7520_22, 7520_14, 5020, 4010.
_MOTOR_TYPE = np.array([0, 0, 1, 0, 2, 2] * 2 + [1, 2, 2] + [2, 2, 2, 2, 2, 3, 3] * 2)
ARMATURE = np.array([0.025101925, 0.010177520, 0.003609725, 0.00425])[_MOTOR_TYPE]
EFFORT_LIMIT = np.array([139.0, 88.0, 25.0, 5.0])[_MOTOR_TYPE]
_NATURAL_FREQ = 10 * 2 * 3.1415926535
_GAIN_MULTIPLIER = np.ones(29)
_GAIN_MULTIPLIER[[4, 5, 10, 11, 13, 14]] = 2
KP = ARMATURE * _NATURAL_FREQ**2 * _GAIN_MULTIPLIER
KD = 4 * ARMATURE * _NATURAL_FREQ * _GAIN_MULTIPLIER
# NVIDIA doubles ankle/waist PD gains after calculating the action scale.
ACTION_SCALE = 0.25 * EFFORT_LIMIT / (ARMATURE * _NATURAL_FREQ**2)

def _array(value, shape, name, dtype=np.float32):
    array = np.asarray(value, dtype=dtype)
    if array.shape != shape or not np.isfinite(array).all():
        raise ValueError(f"{name} requires finite values with shape {shape}, got {array.shape}")
    return array


class DecoderHistory:
    """One independently resettable 50-Hz episode history, oldest sample first."""

    def __init__(self):
        self.last_action = np.zeros(29, dtype=np.float32)
        self.frames = None

    def observation(self, token, q, dq, base_gyro, gravity_body):
        """Construct a prospective decoder input and history; commit only after inference."""
        token = _array(token, (64,), "token")
        q = _array(q, (29,), "q", dtype=np.float64)
        dq = _array(dq, (29,), "dq")
        gyro = _array(base_gyro, (3,), "base_gyro")
        gravity = _array(gravity_body, (3,), "gravity_body")
        frame = np.concatenate(
            [gyro, (q - DEFAULT_Q)[MJ_TO_POLICY], dq[MJ_TO_POLICY], self.last_action, gravity]
        ).astype(np.float32)
        if self.frames is None:
            # StateLogger::makeZeroEntry_ pads missing joints and gyros with zeros.
            # Its zero quaternion rotates gravity to +Z under quat_rotate_d.
            frames = np.zeros((10, 93), dtype=np.float32)
            frames[:-1, -1] = 1.0
            frames[-1] = frame
        else:
            frames = np.concatenate((self.frames[1:], frame[None, :]), axis=0)
        # Published YAML orders whole history blocks by modality.
        groups = np.split(frames, [3, 32, 61, 90], axis=1)
        packed = np.concatenate([token, *(group.ravel() for group in groups)])
        return packed, frames


class Decoder:
    """Load the original decoder and enforce its artifact identity and tensor contract."""

    def __init__(self, model_path):
        path = Path(model_path)
        digest = hashlib.sha256()
        with path.open("rb") as model_file:
            for block in iter(lambda: model_file.read(1024 * 1024), b""):
                digest.update(block)
        if digest.hexdigest() != DECODER_SHA256:
            raise ValueError(f"SONIC decoder SHA-256 mismatch for {path}")
        opts = ort.SessionOptions()
        opts.intra_op_num_threads = 1
        opts.inter_op_num_threads = 1
        self.decoder = ort.InferenceSession(str(path), opts, providers=["CPUExecutionProvider"])
        inputs, outputs = self.decoder.get_inputs(), self.decoder.get_outputs()
        if (len(inputs) != 1 or len(outputs) != 1
                or inputs[0].name != "obs_dict" or inputs[0].shape != [1, 994]
                or outputs[0].name != "action" or outputs[0].shape != [1, 29]):
            raise ValueError("Unexpected signature for the SONIC decoder")

    def decode_observation(self, packed):
        """Return raw, unscaled policy-order actions for one complete ONNX input."""
        packed = _array(packed, (994,), "decoder observation")
        action = self.decoder.run(["action"], {"obs_dict": packed[None, :]})[0][0]
        return _array(action, (29,), "decoder output")

    @staticmethod
    def joint_targets(raw_action):
        """Map a released decoder output to absolute joint angles in simulator order."""
        return DEFAULT_Q + ACTION_SCALE * raw_action[POLICY_TO_MJ]

    def decode_step(self, token, history, q, dq, base_gyro, gravity_body):
        """Consume one 50-Hz sample and return 29 absolute hardware-order targets."""
        packed, frames = history.observation(token, q, dq, base_gyro, gravity_body)
        action = self.decode_observation(packed)
        history.frames = frames
        history.last_action = action
        return self.joint_targets(action)

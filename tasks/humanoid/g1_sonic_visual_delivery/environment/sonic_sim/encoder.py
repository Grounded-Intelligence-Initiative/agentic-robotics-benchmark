"""The G1 reference-motion input of the released GEAR-SONIC V1 encoder."""
import hashlib
from pathlib import Path

import numpy as np
import onnxruntime as ort

from .decoder import MJ_TO_POLICY, _array

ENCODER_SHA256 = "013ab0287236aa2721e13f1e936d699db982302d0de0bfcdae76d5c3245362d3"

# Slices in the published observation_config.yaml. In G1 mode all other
# modalities are zero. Joint arrays on the caller's side use Unitree order.
Q_REFERENCE = slice(4, 294)
DQ_REFERENCE = slice(294, 584)
ANCHOR_ROTATIONS = slice(601, 661)


class ReferenceEncoder:
    """Encode ten G1 reference poses at five-control-tick intervals."""

    def __init__(self, path="/opt/sonic/assets/policy/model_encoder.onnx"):
        model = Path(path)
        with model.open("rb") as source:
            digest = hashlib.sha256()
            for block in iter(lambda: source.read(1024 * 1024), b""):
                digest.update(block)
        if digest.hexdigest() != ENCODER_SHA256:
            raise ValueError(f"SONIC encoder SHA-256 mismatch for {model}")
        options = ort.SessionOptions()
        options.intra_op_num_threads = 1
        options.inter_op_num_threads = 1
        self.session = ort.InferenceSession(str(model), options, providers=["CPUExecutionProvider"])
        inputs, outputs = self.session.get_inputs(), self.session.get_outputs()
        if (len(inputs) != 1 or len(outputs) != 1 or inputs[0].name != "obs_dict"
                or inputs[0].shape != [1, 1762] or outputs[0].name != "encoded_tokens"
                or outputs[0].shape != [1, 64]):
            raise ValueError("Unexpected GEAR-SONIC V1 encoder interface")

    def encode_g1(self, joint_pos, joint_vel, anchor_rot6d):
        """Return 64 FSQ values. Each anchor contains the first two columns of
        R_current_pelvis.T @ R_reference_pelvis, flattened in row-major order.
        """
        obs = np.zeros(1762, dtype=np.float32)
        obs[Q_REFERENCE] = _array(joint_pos, (10, 29), "joint_pos")[:, MJ_TO_POLICY].ravel()
        obs[DQ_REFERENCE] = _array(joint_vel, (10, 29), "joint_vel")[:, MJ_TO_POLICY].ravel()
        obs[ANCHOR_ROTATIONS] = _array(anchor_rot6d, (10, 6), "anchor_rot6d").ravel()
        prediction = self.session.run(["encoded_tokens"], {"obs_dict": obs[None, :]})[0][0]
        return _array(prediction, (64,), "encoded tokens")

    def codes_g1(self, joint_pos, joint_vel, anchor_rot6d):
        """Return the 64 native integer codes accepted by the task."""
        token = self.encode_g1(joint_pos, joint_vel, anchor_rot6d)
        codes = np.rint(token * 16 + 16).astype(np.int32)
        if np.any((codes < 0) | (codes > 31)) or not np.allclose((codes - 16) / 16, token, atol=1e-6, rtol=0):
            raise ValueError("Encoder output is outside the original 32-level FSQ grid")
        return codes

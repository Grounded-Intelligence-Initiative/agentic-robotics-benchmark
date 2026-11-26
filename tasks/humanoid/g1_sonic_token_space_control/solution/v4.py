"""Three-camera V4 streaming policy distributed as a frozen CPU ONNX graph."""
import hashlib
from pathlib import Path

import numpy as np
import onnxruntime as ort
from PIL import Image


ACTOR_SHA256 = "2f4ada4c14f540ee1f99b0e875aa63ade0adbb87d63d710d5b15e4e8300d1c55"


class V4:
    def __init__(self):
        path = Path(__file__).with_name("v4_actor.onnx")
        if hashlib.sha256(path.read_bytes()).hexdigest() != ACTOR_SHA256:
            raise ValueError("V4 high-level checkpoint checksum mismatch")
        options = ort.SessionOptions()
        options.intra_op_num_threads = options.inter_op_num_threads = 1
        self.model = ort.InferenceSession(str(path), options, providers=["CPUExecutionProvider"])
        self.memory_shape = self.model.get_inputs()[2].shape
        if self.memory_shape != [1, 1, 224]:
            raise ValueError("unexpected V4 recurrent state")
        self.reset()

    def reset(self):
        self.memory = np.zeros(self.memory_shape, np.float32)

    def snapshot(self):
        # ONNX Runtime returns a fresh array on inference.
        return self.memory

    def restore(self, memory):
        self.memory = memory

    def act(self, public):
        images = []
        for view in ("hand", "overview", "side"):
            im = public.images[view]
            if im.size != (128, 96):
                im = im.resize((128, 96), Image.Resampling.BILINEAR)
            images.append(np.asarray(im, dtype=np.uint8))
        rgb = np.concatenate(images, axis=-1)[None, None]
        obs = public.proprio
        body = [obs[k] for k in ("q", "dq", "base_quat", "base_gyro", "gravity_body")]
        extra = [obs[k] for k in ("base_pos", "left_hand_pos", "right_hand_pos",
                                   "torso_pos", "torso_quat")]
        proprio = np.concatenate([*body, public.last_hand, *extra]).astype(np.float32)
        if proprio.shape != (86,) or not np.isfinite(proprio).all():
            raise ValueError("expected 86 finite V4 proprioceptive inputs")
        codes, fingers, self.memory = self.model.run(
            ["codes", "hand", "next_memory"],
            {"rgb": rgb, "proprio": proprio[None, None], "memory": self.memory},
        )
        return [{"codes": c.tolist(), "hand": h.tolist()}
                for c, h in zip(codes[0, 0], np.clip(fingers[0, 0], 0., 1.))]

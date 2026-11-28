"""Privileged ground-truth detector; unavailable to evaluated agents."""
import hashlib
import numpy as np
import torch


class Detector:
    def __init__(self, checkpoint_path, device="cuda"):
        state = torch.load(checkpoint_path, map_location="cpu", weights_only=True)
        keys = state["oracle.keys"].numpy()
        offsets = state["oracle.offsets"].tolist()
        self.boxes = state["oracle.boxes"].numpy()
        if keys.ndim != 2 or keys.shape[1] != 32 or len(offsets) != len(keys) + 1:
            raise ValueError("Invalid GT oracle state")
        self.index = {bytes(key): (offsets[i], offsets[i+1]) for i, key in enumerate(keys)}
        if len(self.index) != len(keys):
            raise ValueError("Duplicate oracle key")

    def reset(self):
        pass

    def predict(self, frame):
        height, width = frame.shape[:2]
        key = hashlib.sha256(f"{height}x{width}:".encode() + frame.tobytes()).digest()
        if key not in self.index:
            raise ValueError("RGB frame is outside the pinned oracle fixture")
        first, last = self.index[key]
        return self.boxes[first:last].copy().reshape(-1, 5)

"""Interface example only; it performs no detection or training."""
import numpy as np

class Detector:
    def __init__(self, checkpoint_path, device):
        raise NotImplementedError("Construct your architecture and load model.pt")

    def reset(self):
        """Discard all sequence-specific state."""
        pass

    def predict(self, frame):
        return np.empty((0, 5), dtype=np.float32)

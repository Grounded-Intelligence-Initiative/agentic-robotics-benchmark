"""Trusted adapter, executed at the submitting user's UID, never at root."""
import contextlib
import importlib.util
import json
import math
import struct
import sys
from pathlib import Path
import numpy as np

def validate_artifacts(submission):
    """Check the declared artifact format at the agent UID, before loading its code.

    These are consistency checks, not evidence that the submitting agent trained.
    Training provenance still requires its archived trajectory and reproduction.
    """
    import torch
    config_path = submission / "config.json"
    if config_path.stat().st_size > 1024**2:
        raise ValueError("config.json exceeds 1 MiB")
    config = json.loads(config_path.read_text())
    if not isinstance(config, dict) or not config:
        raise ValueError("config.json must be a nonempty JSON object")
    log_path = submission / "training_log.jsonl"
    if log_path.stat().st_size > 128 * 1024**2:
        raise ValueError("training_log.jsonl exceeds 128 MiB")
    previous = 0
    with log_path.open() as log:
        while True:
            line = log.readline(65537)
            if not line:
                break
            if len(line) > 65536:
                raise ValueError("training log record exceeds 64 KiB")
            record = json.loads(line)
            if not isinstance(record, dict):
                raise ValueError("training records must be JSON objects")
            step, loss = record.get("step"), record.get("loss")
            if type(step) is not int or step <= previous:
                raise ValueError("training steps must be positive and strictly increasing")
            if type(loss) not in (int, float) or not math.isfinite(loss):
                raise ValueError("training losses must be finite numbers")
            previous = step
    if previous == 0:
        raise ValueError("training_log.jsonl must contain actual step/loss records")
    # Never deserialize a submitted checkpoint in the root verifier process.
    state = torch.load(submission / "model.pt", map_location="cpu", weights_only=True)
    if not isinstance(state, dict) or not state:
        raise ValueError("model.pt must contain a nonempty PyTorch state dictionary")
    if any(not isinstance(k, str) or not isinstance(v, torch.Tensor) for k, v in state.items()):
        raise ValueError("model.pt must map parameter/buffer names to tensors")
    if not any(v.numel() for v in state.values()):
        raise ValueError("model.pt contains no tensor values")

def exact(stream, n):
    result = bytearray()
    while len(result) < n:
        block = stream.read(n-len(result))
        if not block:
            raise EOFError
        result.extend(block)
    return bytes(result)

def main():
    submission = Path(sys.argv[1])
    # Capture the protocol stream before redirecting Python model prints.
    wire = sys.stdout
    # Validate before submission modules can shadow or modify the checks.
    with contextlib.redirect_stdout(sys.stderr):
        validate_artifacts(submission)
    sys.path.insert(0, str(submission))
    with contextlib.redirect_stdout(sys.stderr):
        spec = importlib.util.spec_from_file_location("submitted_model", submission/"model.py")
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        model = module.Detector(str(submission/"model.pt"), sys.argv[2])
        model.reset()
    wire.write('{"ready":true}\n'); wire.flush()
    while True:
        try:
            length = struct.unpack(">I", exact(sys.stdin.buffer,4))[0]
        except EOFError:
            return
        header = json.loads(exact(sys.stdin.buffer,length))
        h,w = header["height"],header["width"]
        image = np.frombuffer(exact(sys.stdin.buffer,h*w*3),dtype=np.uint8).reshape(h,w,3).copy()
        with contextlib.redirect_stdout(sys.stderr):
            prediction = np.asarray(model.predict(image))
        # Shape limit before serialization; parent validates independently.
        if prediction.ndim != 2 or prediction.shape[1] != 5 or len(prediction)>300:
            raise ValueError("predict must return N x 5, N <= 300")
        message = json.dumps({"boxes":prediction.tolist()},allow_nan=False,separators=(",",":"))
        wire.write(message+"\n"); wire.flush()

if __name__ == "__main__":
    main()

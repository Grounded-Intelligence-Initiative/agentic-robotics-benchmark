"""The observation on the wire: what the grader and the practice cell send to a controller, one JSON object per step.

Images travel as {"dtype": "uint8", "shape": [H, W, 3], "b64": <base64 of the raw RGB bytes, row-major, top row first>};
every other value is a number, a list of numbers, a string or null. A controller decodes an image with

    np.frombuffer(base64.b64decode(d["b64"]), np.uint8).reshape(d["shape"])
"""

import base64

import numpy as np


def encode_image(img):
    img = np.ascontiguousarray(img, dtype=np.uint8)
    return {"dtype": "uint8", "shape": list(img.shape), "b64": base64.b64encode(img.tobytes()).decode("ascii")}


def decode_image(d):
    return np.frombuffer(base64.b64decode(d["b64"]), np.uint8).reshape(d["shape"])


def to_wire(obs):
    out = {}
    for k, v in obs.items():
        if k.startswith("img_"):
            out[k] = encode_image(v)
        elif isinstance(v, np.ndarray):
            out[k] = v.tolist()
        elif isinstance(v, (np.floating, np.integer)):
            out[k] = v.item()
        else:
            out[k] = v
    return out

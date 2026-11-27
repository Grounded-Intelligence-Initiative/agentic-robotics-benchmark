"""A complete controller with no policy: it holds the robot still and scores 0. Copy it to ~/submission and replace act().

Wire (one JSON object per line on stdin/stdout): print READY once, then answer every request.
  {"type": "reset"}                      -> {"ok": true}
  {"type": "act", "t": int, "obs": obs}  -> {"action": [26 numbers]}: the robot's 23, then 3 camera flags
  {"type": "close"}                      -> {"ok": true}, then exit
A step may be asked more than once with the same t: answer each request from the observation it carries.
"""
import base64, json, sys
import numpy as np

HOLD = None


def decode(cam):
    """-> rgb (H, W, 3) uint8, depth in metres (H, W) float32 (0 = no reading)."""
    import cv2
    rgb = cv2.cvtColor(cv2.imdecode(np.frombuffer(base64.b64decode(cam["rgb_png"]), np.uint8), cv2.IMREAD_COLOR), cv2.COLOR_BGR2RGB)
    depth = np.frombuffer(base64.b64decode(cam["depth_mm_u16"]), "<u2").reshape(cam["shape"]).astype(np.float32) / 1000.0
    return rgb, depth


def act(t, obs):
    global HOLD
    p = obs["proprio"]
    if HOLD is None:      # base 3, torso 4, left arm 7, left gripper 1, right arm 7, right gripper 1
        HOLD = [0.0, 0.0, 0.0] + p[53:57] + p[3:10] + [1.0] + p[28:35] + [1.0]
    return {"action": HOLD + [0.0, 0.0, 0.0]}      # last three: 1.0 to get the head, left-wrist, right-wrist frame next step (0.1 s each)


def main():
    print("READY", flush=True)
    for line in sys.stdin:
        msg = json.loads(line)
        if msg["type"] == "reset":
            global HOLD; HOLD = None; out = {"ok": True}
        elif msg["type"] == "act": out = act(msg["t"], msg["obs"])
        else:
            print(json.dumps({"ok": True}), flush=True); break
        print(json.dumps(out), flush=True)


if __name__ == "__main__":
    main()

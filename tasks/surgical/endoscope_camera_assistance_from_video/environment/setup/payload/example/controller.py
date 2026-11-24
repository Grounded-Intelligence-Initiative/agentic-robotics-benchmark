"""A minimal controller: it speaks the wire correctly and holds the camera still (which scores about 0).

Copy this folder to /home/user/submission and replace `decide`. What it shows:

* the handshake (READY), reset / act / close, one JSON object per line on stdout, diagnostics on stderr;
* an observation: obs["image"] is a base64 JPEG (640 x 480, RGB, one control step old), obs["joints"] the holder's
  readings [pan, tilt, insertion, roll] now;
* memory keyed by the step index t. A step may be asked twice with the same t: once with the true observation and
  once with a perturbed copy (the picture shifted, the pan / tilt / roll readings offset by 0.2-0.4 rad), in either
  order. Answer each request from the observation it carries, and let only a physically possible observation become
  your memory: the holder cannot move 0.2 rad in one step.
"""

import base64
import json
import sys

import cv2
import numpy as np

import holder_model as model


def decode(obs):
    """The frame as an RGB uint8 array (480, 640, 3)."""
    data = np.frombuffer(base64.b64decode(obs["image"]), dtype=np.uint8)
    return cv2.cvtColor(cv2.imdecode(data, cv2.IMREAD_COLOR), cv2.COLOR_BGR2RGB)


class Controller(object):
    def reset(self):
        self.memory = {"joints": np.array(model.Q_HOME)}     # every procedure starts at the home pose
        self.before = {}                                       # step index -> the memory before that step

    def act(self, t, obs):
        if t not in self.before:
            self.before = {t: self.memory}
        memory = self.before[t]
        joints = np.asarray(obs["joints"], dtype=float)
        frame = decode(obs)
        action, new_memory = self.decide(memory, frame, joints)
        if np.all(np.abs(joints - memory["joints"]) <= model.RATE_MAX * model.DT * 1.5 + 1e-3):
            self.memory = new_memory                           # a perturbed copy never passes this test
        return action

    def decide(self, memory, frame, joints):
        """Your controller: rates [pan, tilt, insertion, roll] (rad/s, rad/s, m/s, rad/s) and the next memory."""
        return [0.0, 0.0, 0.0, 0.0], {"joints": joints}


def main():
    controller = Controller()
    controller.reset()
    print("READY", flush=True)
    for line in sys.stdin:
        if not line.strip():
            continue
        msg = json.loads(line)
        if msg["type"] == "reset":
            controller.reset()
            print(json.dumps({"ok": True}), flush=True)
        elif msg["type"] == "act":
            print(json.dumps({"action": controller.act(int(msg["t"]), msg["obs"])}), flush=True)
        elif msg["type"] == "close":
            print(json.dumps({"ok": True}), flush=True)
            return


if __name__ == "__main__":
    main()

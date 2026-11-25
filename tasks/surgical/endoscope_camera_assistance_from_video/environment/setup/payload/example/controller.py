"""A minimal controller: it speaks the wire correctly and holds the camera still (which scores about 0).

Copy this folder to /home/user/submission and replace `decide`. What it shows:

* the handshake (READY), reset / act / close, one JSON object per line on stdout, diagnostics on stderr;
* an observation: obs["image"] is a base64 JPEG (640 x 480, RGB, one control step old), obs["joints"] the holder's
  readings [pan, tilt, insertion, roll] now;
* memory keyed by the step index t. A step may be asked twice with the same t: once with the true observation and
  once with a copy (the same frame rendered again with the instruments moved differently over its last step, the
  joint readings the same), in either order. Answer each request from the observation it carries and keep the memory
  each request leaves; when the next step arrives, `choose` picks the one that becomes your memory (this example takes
  the latest; a controller that tracks the instruments can take the one whose instruments the new frame continues).
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
        self.after = []                                        # the memories the requests of the latest step left

    def act(self, t, obs):
        joints = np.asarray(obs["joints"], dtype=float)
        frame = decode(obs)
        if t not in self.before:
            if self.after:
                self.memory = self.choose(self.after, frame, joints)
            self.before, self.after = {t: self.memory}, []
        action, new_memory = self.decide(self.before[t], frame, joints)
        self.after.append(new_memory)
        return action

    def decide(self, memory, frame, joints):
        """Your controller: rates [pan, tilt, insertion, roll] (rad/s, rad/s, m/s, rad/s) and the next memory."""
        return [0.0, 0.0, 0.0, 0.0], {"joints": joints}

    def choose(self, memories, frame, joints):
        """Which of the memories the requests of the last step left becomes the memory (one, or two for a repeated
        step); `frame` and `joints` are the new step's."""
        return memories[-1]


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

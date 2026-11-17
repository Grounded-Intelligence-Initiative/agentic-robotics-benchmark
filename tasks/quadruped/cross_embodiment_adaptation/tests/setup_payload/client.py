"""Public client of the robot service. The socket accepts only the documented calls.

    from client import Client, camera_image
    robot = Client()                 # ROBOT_SOCKET, default /run/visual_robotics/world.sock
    obs = robot.observe()            # does not move the robot
    obs = robot.act([1/3])           # one button press (0, 1/3, 2/3 or 1): 0.2 s of motion
    obs = robot.advance()            # the one switch from the scout to the carrier
    obs = robot.new_site()           # commissioning only: a fresh development site
    image = camera_image(obs, 0)     # front; index 1 is rear, both body-relative perspective
    obs = robot.act([0., 0.])        # carrier only: brake; signed channels otherwise
"""
import base64
import io
import json
import os
import socket

SOCKET = os.environ.get("ROBOT_SOCKET", "/run/visual_robotics/world.sock")


class Client:
    def __init__(self, path=None, timeout=60):
        self.path = path or SOCKET
        self.timeout = timeout

    def request(self, method, **params):
        with socket.socket(socket.AF_UNIX) as sock:
            sock.settimeout(self.timeout)
            sock.connect(self.path)
            sock.sendall((json.dumps({"method": method, "params": params})+"\n").encode())
            with sock.makefile("rb") as stream:
                result = json.loads(stream.readline(4_000_000))
        if "error" in result:
            raise ValueError(result["error"])
        return result

    def observe(self):
        return self.request("observe")

    def act(self, action):
        return self.request("act", action=action)

    def advance(self):
        return self.request("advance")

    def new_site(self):
        return self.request("new_site")


def camera_image(obs, index=0):
    """Body-relative front/rear perspective image as a (256, 256, 3) array."""
    import numpy as np
    from PIL import Image
    return np.asarray(Image.open(io.BytesIO(base64.b64decode(obs["images"][index]))).convert("RGB"))


def top_image(obs):
    """Legacy decoding alias only: this is NOT a calibrated world top view."""
    return camera_image(obs, 0)

"""Public client of the robot service. The socket accepts only the documented calls.

    from client import Client, top_image, pixel_to_world, world_to_pixel
    robot = Client()                 # ROBOT_SOCKET, default /run/visual_robotics/world.sock
    obs = robot.observe()            # does not move the robot
    obs = robot.act([1/3])           # one button press (0, 1/3, 2/3 or 1): 0.2 s of motion
    obs = robot.advance()            # the one switch from the scout to the carrier
    obs = robot.new_site()           # commissioning only: a fresh development site
    image = top_image(obs)           # 256 x 256 x 3 uint8 (needs numpy and Pillow)
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


def top_image(obs):
    """The observation's top view as a (256, 256, 3) uint8 array."""
    import numpy as np
    from PIL import Image
    return np.asarray(Image.open(io.BytesIO(base64.b64decode(obs["images"][0]))).convert("RGB"))


def pixel_to_world(u, v):
    """Pixel centre (column u, row v) of the top view to hall coordinates (x, y) in metres."""
    return (u+.5)/32-4, 4-(v+.5)/32


def world_to_pixel(x, y):
    """Hall coordinates in metres to fractional pixel coordinates (u, v) of the top view."""
    return (x+4)*32-.5, (4-y)*32-.5

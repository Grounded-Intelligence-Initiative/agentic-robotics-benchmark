"""Public client of the robot service. The socket accepts only the documented calls.

    from client import Client, camera_image, pixel_to_body
    robot = Client()                 # ROBOT_SOCKET, default /run/visual_robotics/world.sock
    obs = robot.observe()            # does not move the robot
    obs = robot.act([1/3])           # one button press (0, 1/3, 2/3 or 1): 0.2 s of motion
    obs = robot.advance()            # the one switch from the scout to the carrier
    obs = robot.new_site()           # commissioning only: a fresh development site
    image = camera_image(obs)        # 256 x 256 x 3 uint8, the current body's camera (needs numpy and Pillow)
    obs["odometer"]                  # [x, y] metres: the leg odometer's running total, in the body's frame
"""
import base64
import io
import json
import os
import socket

SOCKET = os.environ.get("ROBOT_SOCKET", "/run/visual_robotics/world.sock")
SCOUT_FLOOR_SPAN = 2.8           # metres of floor across the scout's image (calibrated)


class Client:
    def __init__(self, path=None, timeout=180):
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


def camera_image(obs):
    """The observation's camera image as a (256, 256, 3) uint8 array."""
    import numpy as np
    from PIL import Image
    return np.asarray(Image.open(io.BytesIO(base64.b64decode(obs["images"][0]))).convert("RGB"))


def pixel_to_body(u, v, span=SCOUT_FLOOR_SPAN):
    """Pixel centre (column u, row v) to the floor point it shows, in the body's own frame (metres; x to the
    image's right, y up the image, origin under the body's centre), for an image `span` metres of floor across."""
    return (u-127.5)*span/256, (127.5-v)*span/256


def body_to_pixel(x, y, span=SCOUT_FLOOR_SPAN):
    """A floor point in the body's frame to fractional pixel coordinates (u, v)."""
    return 127.5+x*256/span, 127.5-y*256/span

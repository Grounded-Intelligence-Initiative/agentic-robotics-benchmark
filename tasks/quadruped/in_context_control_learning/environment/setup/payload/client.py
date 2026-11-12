"""The robot service's socket client, used by the MCP bridge. The socket accepts only observe, act and advance."""
import json
import socket

SOCKET = "/run/visual_robotics/world.sock"


class Client:
    def __init__(self, path=SOCKET):
        self.path = path

    def request(self, method, **params):
        with socket.socket(socket.AF_UNIX) as sock:
            sock.settimeout(60)
            sock.connect(self.path)
            sock.sendall((json.dumps({"method": method, "params": params})+"\n").encode())
            with sock.makefile("rb") as stream:
                result = json.loads(stream.readline(2_000_000))
        if "error" in result:
            raise ValueError(result["error"])
        return result

    def observe(self):
        return self.request("observe")

    def act(self, action):
        return self.request("act", action=action)

    def advance(self):
        return self.request("advance")

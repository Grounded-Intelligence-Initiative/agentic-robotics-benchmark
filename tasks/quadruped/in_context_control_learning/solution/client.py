"""Transport for the native MCP bridge and the engine-staged author oracle."""
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

    def act(self, target_rad, ticks):
        return self.request("act", target_rad=target_rad, ticks=ticks)

    def recover(self):
        return self.request("recover")

    def advance(self):
        return self.request("advance")

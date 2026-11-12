"""Minimal MCP stdio bridge to the robot service: observe, press a button, advance. No shell, files or reset."""
import json
import sys
from client import Client

TOOLS = [
    {"name": "robot_observe",
     "description": "Look through the ceiling camera without moving the robot: one 256x256 top-view image and the "
                    "delivery status. Does not use a button press.",
     "inputSchema": {"type": "object", "properties": {}, "additionalProperties": False}},
    {"name": "robot_act",
     "description": "Press one remote button for one 0.2 s step of walking, then return the new camera image and "
                    "status. action is a list holding one value: 0, 0.3333333333333333, 0.6666666666666666 or 1.",
     "inputSchema": {"type": "object",
                     "properties": {"action": {"type": "array", "items": {"type": "number"},
                                               "minItems": 1, "maxItems": 1}},
                     "required": ["action"], "additionalProperties": False}},
    {"name": "robot_advance",
     "description": "After the active delivery point is reached (goal_reached true), light the next one. "
                    "The robot stays where it is; no budget is restored.",
     "inputSchema": {"type": "object", "properties": {}, "additionalProperties": False}},
]


def main():
    client = Client()
    for line in sys.stdin:
        req = json.loads(line)
        if "id" not in req:
            continue
        try:
            method = req["method"]
            if method == "initialize":
                result = {"protocolVersion": req.get("params", {}).get("protocolVersion", "2024-11-05"),
                          "capabilities": {"tools": {}},
                          "serverInfo": {"name": "delivery_robot", "version": "2.0"}}
            elif method == "tools/list":
                result = {"tools": TOOLS}
            elif method == "tools/call":
                name = req["params"]["name"]
                if name not in {t["name"] for t in TOOLS}:
                    raise ValueError("unknown tool")
                obs = client.request(name.removeprefix("robot_"), **req["params"].get("arguments", {}))
                images = obs.pop("images", [])
                result = {"content": [{"type": "text", "text": json.dumps(obs)}] +
                          [{"type": "image", "data": x, "mimeType": "image/png"} for x in images]}
            elif method == "ping":
                result = {}
            else:
                raise ValueError("unsupported method")
            reply = {"jsonrpc": "2.0", "id": req["id"], "result": result}
        except (ValueError, KeyError, OSError) as exc:
            reply = {"jsonrpc": "2.0", "id": req["id"], "error": {"code": -32602, "message": str(exc)}}
        print(json.dumps(reply), flush=True)


if __name__ == "__main__":
    main()

"""Minimal MCP stdio bridge to the robot service: observe, press a button, advance. No shell, files or reset."""
import json
import sys
from client import Client

TOOLS = [
    {"name": "robot_observe",
     "description": "Look through the onboard perspective camera without advancing time: one 384x384 image and the "
                    "delivery status. Uses one of 120 camera frames, not a button press.",
     "inputSchema": {"type": "object", "properties": {}, "additionalProperties": False}},
    {"name": "robot_act",
     "description": "Hold a button for frames physics ticks (1-120, default 30; each tick is 0.01 s), then return "
                    "status only (no image). action contains one of 0, 0.3333333333333333, "
                    "0.6666666666666666 or 1. The unknown, episode-fixed assignment is forward, "
                    "turn left, turn right and brake. Motion has lag and momentum.",
     "inputSchema": {"type": "object",
                     "properties": {"action": {"type": "array", "items": {"type": "number"},
                                               "minItems": 1, "maxItems": 1},
                                    "frames": {"type": "integer", "minimum": 1, "maximum": 120, "default": 30}},
                     "required": ["action"], "additionalProperties": False}},
    {"name": "robot_advance",
     "description": "After the active delivery point is reached (goal_reached true), light the next one. "
                    "Returns status only. The robot stays where it is; no budget is restored.",
     "inputSchema": {"type": "object", "properties": {}, "additionalProperties": False}},
]
METHODS = {"robot_observe": "observe", "robot_act": "act", "robot_advance": "advance"}


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
                          "serverInfo": {"name": "delivery_robot", "version": "4.0"}}
            elif method == "tools/list":
                result = {"tools": TOOLS}
            elif method == "tools/call":
                name = req["params"]["name"]
                if name not in METHODS:
                    raise ValueError("unknown tool")
                arguments = req["params"].get("arguments", {})
                allowed = {"action", "frames"} if name == "robot_act" else set()
                if not isinstance(arguments, dict) or not set(arguments) <= allowed:
                    raise ValueError("unsupported arguments")
                obs = client.request(METHODS[name], **arguments)
                images = obs.pop("images", [])
                result = {"content": [{"type": "text", "text": json.dumps(obs)}] +
                          [{"type": "image", "data": x, "mimeType": "image/png"} for x in images]}
            elif method == "ping":
                result = {}
            else:
                raise ValueError("unsupported method")
            reply = {"jsonrpc": "2.0", "id": req["id"], "result": result}
        except (ValueError, TypeError, KeyError, OSError) as exc:
            reply = {"jsonrpc": "2.0", "id": req["id"], "error": {"code": -32602, "message": str(exc)}}
        print(json.dumps(reply), flush=True)


if __name__ == "__main__":
    main()

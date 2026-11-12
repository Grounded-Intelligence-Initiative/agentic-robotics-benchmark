"""Minimal MCP stdio bridge to the robot service; no shell, paths, code execution or reset of budgets."""
import json
import sys

from client import Client

TOOLS = [
    {"name": "robot_observe", "description": "Look through the robot's top-view camera; the robot does not move.",
     "inputSchema": {"type": "object", "properties": {}, "additionalProperties": False}},
    {"name": "robot_act", "description": "Press one button: action [0], [1/3], [2/3] or [1]; 0.2 s of motion.",
     "inputSchema": {"type": "object", "properties": {"action": {"type": "array", "items": {"type": "number"}}},
                     "required": ["action"], "additionalProperties": False}},
    {"name": "robot_advance", "description": "Switch once from the scout to the carrier (after 12 or more scout actions).",
     "inputSchema": {"type": "object", "properties": {}, "additionalProperties": False}},
    {"name": "robot_new_site", "description": "Commissioning: start a fresh mission on a new development site.",
     "inputSchema": {"type": "object", "properties": {}, "additionalProperties": False}},
]


def main():
    client = Client()
    for line in sys.stdin:
        try:
            req = json.loads(line)
        except ValueError:
            continue
        if not isinstance(req, dict) or "id" not in req:
            continue
        try:
            method = req["method"]
            if method == "initialize":
                result = {"protocolVersion": req.get("params", {}).get("protocolVersion", "2024-11-05"),
                          "capabilities": {"tools": {}},
                          "serverInfo": {"name": "cross_embodiment_robot", "version": "2.0"}}
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
        except (ValueError, KeyError, TypeError, OSError) as exc:
            reply = {"jsonrpc": "2.0", "id": req["id"], "error": {"code": -32602, "message": str(exc)}}
        print(json.dumps(reply), flush=True)


if __name__ == "__main__":
    main()

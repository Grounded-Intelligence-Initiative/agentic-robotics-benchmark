"""Minimal MCP stdio bridge to the robot service; no shell, paths, code execution or reset of budgets."""
import json
import sys

from client import Client

TOOLS = [
    {"name": "robot_observe", "description": "Current UAV down-camera and local tracking; no motion.",
     "inputSchema": {"type": "object", "properties": {}, "additionalProperties": False}},
    {"name": "robot_act", "description": "Translate the UAV by [dx, dy] metres (norm <= 0.60).",
     "inputSchema": {"type": "object", "properties": {"action": {"type": "array", "items": {"type": "number"},
                                                               "minItems": 2, "maxItems": 2}},
                     "required": ["action"], "additionalProperties": False}},
    {"name": "robot_new_site", "description": "Commission another scout-only site; no access to the Go1.",
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
                          "serverInfo": {"name": "cross_embodiment_robot", "version": "5.0"}}
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

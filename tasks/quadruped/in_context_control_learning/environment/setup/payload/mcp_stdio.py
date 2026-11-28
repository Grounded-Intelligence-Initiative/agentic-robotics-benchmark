"""Native MCP image and direct joint controls. No operator program or locomotion primitives."""
import json
import sys
from client import Client

TOOLS = [
    {"name": "robot_observe",
     "description": "Return one onboard 384x288 RGB image and milestone status. No time passes.",
     "inputSchema": {"type": "object", "properties": {}, "additionalProperties": False}},
    {"name": "robot_act",
     "description": "Hold twelve absolute servo targets (radians) for 1-500 ticks of 2 ms each. "
                    "Four leg triples: abduction, thigh, knee. Returns milestone status, no image.",
     "inputSchema": {"type": "object",
                     "properties": {"target_rad": {"type": "array", "items": {"type": "number"},
                                                   "minItems": 12, "maxItems": 12},
                                    "ticks": {"type": "integer", "minimum": 1, "maximum": 500}},
                     "required": ["target_rad", "ticks"], "additionalProperties": False}},
    {"name": "robot_advance",
     "description": "Leave calibration for the launch line, or activate the next fixture after a completed dock.",
     "inputSchema": {"type": "object", "properties": {}, "additionalProperties": False}},
    {"name": "robot_recover",
     "description": "Re-rack at the start of the current unfinished leg or calibration. No new credit.",
     "inputSchema": {"type": "object", "properties": {}, "additionalProperties": False}},
]
METHODS = {t["name"]: t["name"].removeprefix("robot_") for t in TOOLS}


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
                          "serverInfo": {"name": "delivery_robot", "version": "9.0"}}
            elif method == "tools/list":
                result = {"tools": TOOLS}
            elif method == "tools/call":
                name = req["params"]["name"]
                if name not in METHODS:
                    raise ValueError("unknown tool")
                arguments = req["params"].get("arguments", {})
                allowed = {"target_rad", "ticks"} if name == "robot_act" else set()
                if not isinstance(arguments, dict) or set(arguments) != allowed:
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

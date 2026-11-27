"""Checks the engine's trusted trajectory, not an agent-supplied activity report."""
import json
from pathlib import Path

METHODS = {"robot_observe": "observe", "robot_act": "act",
           "robot_advance": "advance", "robot_report": "report"}


def audit_trajectory(path, events):
    payload = json.loads(Path(path).read_text())
    if payload.get("schema_version") != "ATIF-v1.8":
        return False
    model = payload.get("agent", {}).get("model_name")
    if not model or payload.get("subagent_trajectories"):
        return False
    calls = []
    for step in payload.get("steps", []):
        if step.get("source") != "agent":
            continue
        if step.get("model_name") not in (None, model):
            return False
        for call in step.get("tool_calls", []):
            name = call.get("function_name", "").split("__")[-1].split(".")[-1]
            if name not in METHODS:
                return False
            arguments = call.get("arguments", {})
            if isinstance(arguments, str):
                arguments = json.loads(arguments)
            calls.append({"method": METHODS[name], "params": arguments})
    actual = [{"method": e["method"], "params": e["params"]} for e in events]
    return bool(calls) and calls == actual

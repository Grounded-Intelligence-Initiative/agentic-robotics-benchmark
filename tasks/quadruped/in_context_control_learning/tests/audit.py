"""Direct-control audit of the engine's trajectory against the robot service's ledger.

The rule the instruction states: every request the robot service received must have been made by one of the agent's
own robot_* tool calls, and the agent must not call a tool that runs commands or code, writes or edits files, or
starts another agent. Tools that only keep a to-do list or read files are allowed, as are tool calls the harness
rejected before they reached the service (they appear in the trajectory but not in the ledger), so the ledger has to
be an ordered subsequence of the agent's robot_* calls, not equal to them. Unknown tools fail closed.
"""
import json
import re

ROBOT = {"robot_" + op: op for op in ("observe", "act", "advance", "recover")}
# Note-keeping and read-only tools of the harnesses we run: none of them can reach the robot service or run code.
HARMLESS = {"TodoWrite", "TodoRead", "TaskCreate", "TaskUpdate", "TaskList", "TaskGet", "TaskOutput", "TaskStop",
            "Read", "Glob", "Grep", "LS", "ToolSearch", "EnterPlanMode", "ExitPlanMode",
            "AskUserQuestion", "ListMcpResourcesTool", "ReadMcpResourceTool"}
# Codex preserves the functions namespace in ATIF; only these exact built-ins are exempt.
CODEX_HARMLESS = {"update_plan", "view_image", "request_user_input", "tool_search_tool",
                  "list_mcp_resources", "list_mcp_resource_templates", "read_mcp_resource"}
HARMLESS |= CODEX_HARMLESS | {"functions." + name for name in CODEX_HARMLESS}
FORBIDDEN = {"Bash", "BashOutput", "KillShell", "KillBash", "Write", "Edit", "MultiEdit", "NotebookEdit", "Agent",
             "Task", "Skill", "SlashCommand", "WebFetch", "WebSearch"}
# Other harnesses' names for the same capabilities (shell, code, file writing, sub-agents, web, desktop control).
FORBIDDEN_PATTERN = re.compile(r"bash|shell|terminal|exec|command|python|code|script|notebook|write|edit|patch|"
                               r"apply|agent|spawn|web|browser|computer|kill", re.IGNORECASE)


def _tool(call):
    """(kind, name): kind is "robot", "harmless", "forbidden" or "unclassified"."""
    name = str(call.get("function_name", ""))
    mcp = ((call.get("extra") or {}).get("ale") or {}).get("mcp") or {}
    server, tool = mcp.get("server"), mcp.get("tool")
    if server is None and name.startswith("mcp__"):
        parts = name.split("__")
        server, tool = (parts[1], "__".join(parts[2:])) if len(parts) >= 3 else (None, name)
    if server == "robot" and tool in ROBOT:
        return "robot", tool
    short = tool if server is not None else name
    if server is None and short in HARMLESS:
        return "harmless", short
    if (server is None and short in FORBIDDEN) or FORBIDDEN_PATTERN.search(short or ""):
        return "forbidden", name
    return "unclassified", name


def _arguments(call):
    arguments = call.get("arguments", {})
    if isinstance(arguments, str):
        try:
            arguments = json.loads(arguments) if arguments.strip() else {}
        except ValueError:
            return {"_unparsed": arguments}  # an argument the harness rejected before dispatch
    return arguments if isinstance(arguments, dict) else {"_raw": arguments}


def _same(call_params, event_params):
    # Numeric JSON 1 and 1.0 compare equal; strings and booleans are not joint targets.
    if isinstance(call_params, dict) and isinstance(event_params, dict):
        return call_params.keys() == event_params.keys() and all(
            _same(call_params[k], event_params[k]) for k in call_params)
    if isinstance(call_params, list) and isinstance(event_params, list):
        return len(call_params) == len(event_params) and all(map(_same, call_params, event_params))
    return (type(call_params) is bool) == (type(event_params) is bool) and call_params == event_params


def audit_trajectory(payload, events):
    """(passed, reason, stats) for an ATIF trajectory dict and the ledger's event list."""
    stats = {"robot_calls": 0, "harmless_calls": 0, "unclassified_calls": 0, "forbidden_calls": 0,
             "ledger_events": len(events), "ledger_events_matched": 0}
    calls, forbidden, unclassified = [], [], []
    for step in payload.get("steps", []):
        if step.get("source") != "agent":
            continue
        for call in step.get("tool_calls") or []:
            kind, name = _tool(call)
            stats[f"{kind}_calls"] += 1
            if kind == "robot":
                calls.append((ROBOT[name], _arguments(call)))
            elif kind == "forbidden":
                forbidden.append(name)
            elif kind == "unclassified":
                unclassified.append(name or "<unnamed>")
    if forbidden:
        return False, "the agent called tools that run code, write files or start agents: " + \
            ", ".join(sorted(set(forbidden))), stats
    if unclassified:
        return False, "the agent called tools not approved for direct control: " + \
            ", ".join(sorted(set(unclassified))), stats
    i = 0
    for n, event in enumerate(events):
        while i < len(calls) and not (calls[i][0] == event.get("method") and _same(calls[i][1], event.get("params"))):
            i += 1
        if i == len(calls):
            return False, (f"service request {n+1} of {len(events)} ({event.get('method')}) was not made by a "
                           "robot_* tool call of the agent"), stats
        i += 1
        stats["ledger_events_matched"] += 1
    return True, "", stats

# Robot service

The robot is driven through the MCP tools `robot_observe`, `robot_act` and `robot_advance` (see the task
instruction). `client.py` and `mcp_stdio.py` are the bridge those tools run on; they are here for reference only.
In this task no program may talk to the robot service: every request has to come from your own `robot_*` tool calls.

# Robot service

Use the declared MCP tools for direct image interaction.
Import `Client` from `client.py` and call `observe()` or
`act([right, forward, up, yaw_left, grip])`. See the instructions for camera and control
calibration.
Run code with `/opt/venv/bin/python3`; do not rely on the harness's PATH.
The endpoint is `/run/visual_robotics/world.sock`.
Both methods return two onboard base64 PNG images and remaining actions.
There is no reset, state inspection, file-read or code-execution operation.
Keep deliverables only under `/home/user/submission`.

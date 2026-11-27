# Robot service

Use `robot_observe` and `robot_act` for direct image interaction. In Python,
import `Client` from `client.py` and call `observe()` or
`act([dA, dB, dz, vacuum])`. A and B are motor increments in the enclosed
crossed-belt cassette, not camera X and Y. The generic client's `advance()`
and `report(frame_id, point)` operations do not apply to this workcell.
Run code with `/opt/venv/bin/python3`; do not rely on the harness's PATH.
The endpoint is `/run/visual_robotics/world.sock`.
Both arm methods return base64 PNG images and remaining budgets.
There is no reset, state inspection, file-read or code-execution operation.
Keep deliverables only under `/home/user/submission`.

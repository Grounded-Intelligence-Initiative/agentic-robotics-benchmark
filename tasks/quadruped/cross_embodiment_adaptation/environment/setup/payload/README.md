# Robot Service v6

The public client and MCP tools reach the service on
`/run/visual_robotics/world.sock` (`ROBOT_SOCKET` overrides it).
Run the submission with `/opt/venv/bin/python3`. See the task instruction
for camera calibration, joint semantics, observation fields and budgets.

- `observe()` does not move either vehicle.
- UAV: `act([dx, dy])` translates and returns a down-camera image.
  `camera_image(obs)` returns a NumPy RGB array.
- Grading only: `advance()` retires the scout and deploys a native Go1 once.
  `act({"target_rad": [...], "ticks": n})` holds one explicit pose.
- Development only: `new_site()` starts another scout-only survey.

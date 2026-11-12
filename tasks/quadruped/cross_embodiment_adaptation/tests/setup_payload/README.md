# Robot service

The MCP tools (`robot_observe`, `robot_act`, `robot_advance`, `robot_new_site`) and `client.py`
reach the same service at `/run/visual_robotics/world.sock` (`ROBOT_SOCKET` overrides the path).
Run code with `/opt/venv/bin/python3`; do not rely on the harness's PATH.

`observe()`, `act(action)`, `advance()` and `new_site()` return an observation: one base64 PNG
in `images`, `frame_id`, `phase`, `done`, `steps_remaining`, `requests_remaining`, `bumper`
and `contact_stop`. `top_image(obs)` decodes the image; `pixel_to_world` / `world_to_pixel`
convert between pixels and metres. There is no state, map, seed or file-reading call.

Your mission program goes in `/home/user/submission/run.sh` (the instruction says how it is run).

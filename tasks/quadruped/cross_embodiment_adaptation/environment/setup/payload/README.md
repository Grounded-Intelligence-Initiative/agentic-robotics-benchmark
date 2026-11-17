# Robot Service v3

The MCP tools and client.py reach the same service at
/run/visual_robotics/world.sock (ROBOT_SOCKET overrides it).
Run code with /opt/venv/bin/python3.

observe(), act(action), advance() and commissioning-only new_site() return
front/rear body-mounted perspective images (256 x 256, 85-degree vertical FOV,
25 degrees downward) and frame_id, phase, done, steps_remaining,
requests_remaining, bumper and contact_stop. camera_image(obs, 0/1) decodes
front/rear. top_image is a historical decoding alias; there is no global
pixel-to-world mapping. No public state, seed, map or score call exists.

Scout actions: [0], [1/3], [2/3], [1], unknown assignment to planar directions.
Carrier actions: two signed channels in [-1,1], unknown assignment/sign to
longitudinal speed and steering. [0,0] brakes with residual inertia. Each
action lasts 0.2 seconds. The long rack rotates, cannot translate sideways
or spin in place, and must settle with the correct heading inside the dock.
The instruction gives footprint, dynamics, camera and budget ranges.

Leave the unattended mission program at /home/user/submission/run.sh.

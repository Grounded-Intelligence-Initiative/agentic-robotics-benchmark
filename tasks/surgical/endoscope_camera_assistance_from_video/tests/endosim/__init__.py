"""endosim: the endoscope camera-holding simulator.

`model` is the public holder and camera model (the agent has a byte copy as ~/given/holder_model.py). Everything else
is hidden from the agent's account: `tissue` and `procedure` generate a procedure, `episode` runs and judges it,
`session` joins it with the renderer (`render_client` -> `render_server`, the Isaac Sim process) into observations.
"""

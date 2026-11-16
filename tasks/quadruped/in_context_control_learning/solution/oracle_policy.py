"""The privileged oracle of the delivery bay. Not an LLM result, and not what an agent can do.

It presents the oracle token to the robot service (`unlock`), which answers with the scene's ground truth (walls,
button assignment, gait speeds, the load factor, all six delivery points) and from then on adds the robot's true pose
to every reply. With those it plans on the true map and drives the true pose to each delivery point, never looking
through the camera. It fixes what a full score is (the anchor); `reference_policy.py` is the evidence that the
agent's own observations suffice.
"""
import math

import nav

MARGIN = .035             # planning clearance beyond the footprint (m)
SETTLE = .875             # the base settles at this fraction of a commanded speed (velocity gain 35, damping 5)
PRESS = .2                # seconds per press
LOADING_STOP = 3          # from the instruction: the load goes on when advancing from the third delivery point


def solve(client, token):
    obs = client.request("unlock", token=token)
    scene = obs["scene"]
    button = {gait: index for index, gait in enumerate(scene["mapping"])}
    blocked = nav.rasterize(scene["walls"], nav.ROBOT_RADIUS+MARGIN)
    fields = {}
    while not obs["done"]:
        if obs["goal_reached"]:
            obs = client.advance()
            continue
        index = obs["delivery"]-1
        goal = scene["goals"][index]
        if index not in fields:
            fields[index] = nav.distance_field(blocked, goal)
        load = scene["load"] if obs["delivery"] > LOADING_STOP else 1.
        per_press = PRESS*SETTLE*load
        steps = {"F": scene["speeds"]["forward"]*per_press, "L": scene["speeds"]["left"]*per_press,
                 "R": scene["speeds"]["right"]*per_press}
        truth = obs["truth"]
        press = nav.next_press((truth["x"], truth["y"], truth["yaw"]), fields[index], blocked, steps, goal)
        gait = {"F": "forward", "L": "left", "R": "right"}[press]
        obs = client.act([button[gait]/3])
    return obs

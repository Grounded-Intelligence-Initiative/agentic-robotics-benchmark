"""ALE author solution: no privileged service calls or hidden inputs."""
import numpy as np

from client import Client
from survey import scan, waypoints
from walk import Walk


def main():
    robot = Client()
    side, goal = scan(robot).route()
    obs = robot.advance()
    walk = Walk(waypoints(side, goal))
    while not obs["done"]:
        obs = robot.act(walk.action(np.asarray(obs["tracking_xy_m"]), obs["heading_rad"],
                                   obs["ticks_remaining"]))


if __name__ == "__main__":
    main()

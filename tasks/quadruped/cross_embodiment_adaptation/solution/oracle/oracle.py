"""ALE author solution: no privileged service calls or hidden inputs."""
import numpy as np

from client import Client
from survey import scan
from walk import Walk


def deploy(robot, path):
    obs = robot.advance()
    walk = Walk(path)
    while not obs["done"]:
        obs = robot.act(walk.action(np.asarray(obs["tracking_xy_m"]), obs["heading_rad"],
                                   obs["ticks_remaining"]))
    return obs


def main():
    robot = Client()
    deploy(robot, scan(robot).route())


if __name__ == "__main__":
    main()

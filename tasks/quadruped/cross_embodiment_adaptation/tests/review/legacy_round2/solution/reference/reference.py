"""Reference mission program from the robots' own observations: evidence that they suffice.

No token and no truth: it uses only what the instruction gives an agent (the two cameras' images with their
documented geometry, the odometer, the observation's fields, the hall's documented dimensions).

  Scout    Every frame is registered on a mosaic of the floor (mapping.py), so the scout knows its pose in its
           own map through slips and turns. Each frame says where floor is and where a wall's foot or a crate's
           footprint is (perception.py: the mast camera is a pinhole, obstacles lean outward and hide floor).
           The scout looks for the pad, then for the missing pieces of a route the carrier can take, until
           every cell the carrier would cover was seen as floor (explore.py).
  Carrier  It finds its pose in the scout's map by matching its floor camera's first image with the mosaic at
           every heading and span, keeps track by visual odometry corrected on the mosaic (floorloc.py), fits
           its lagged response as it drives and steers by model-predictive control along the certain route
           (driving.py).
"""
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from client import Client, camera_image  # noqa: E402
import driving  # noqa: E402
import explore  # noqa: E402
import floorloc  # noqa: E402
import mapping  # noqa: E402

VERBOSE = bool(os.environ.get("REFERENCE_VERBOSE"))
START = time.time()


def log(*args):
    if VERBOSE:
        print("[%6.1f]" % (time.time()-START), *args, file=sys.stderr, flush=True)


def mission(robot):
    obs = robot.observe()
    mosaic = mapping.Mosaic()
    tracker = mapping.Tracker(mosaic, camera_image(obs))

    def sense(obs, stride):
        pose, score = tracker.update(camera_image(obs), stride)
        return pose
    pad, sure = explore.Scout(robot, obs, sense, mosaic, log).run()
    obs = robot.advance()
    if pad is None:
        log("the pad was never seen: nowhere to deliver")
        return obs
    localizer = floorloc.FloorLocalizer(mosaic, log)

    def locate(obs):
        if localizer.pose is None:
            return localizer.relocalize(camera_image(obs))[0]
        return localizer.track(camera_image(obs))[0]
    return driving.Driver(robot, obs, locate, mosaic, pad, log).run()


def main():
    obs = mission(Client())
    log("done:", {k: v for k, v in obs.items() if k != "images"})
    return 0


if __name__ == "__main__":
    sys.exit(main())

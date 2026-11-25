"""The oracle's graded episode (dual mode): the privileged reference program on THE graded mission, through the
agent-side client.

    python3 -I -B online_oracle.py <the given library folder> <this folder>

It connects to the graded episode's service (plantcell.graded.connect_graded), starts the graded mission with the
oracle token (start's only parameter the service accepts, and only with the token whose sha256 it holds; the agent's
client has no such option), runs this folder's patrol.py solve(robot) on it (one frame of every sensor, then the
privileged reference_run: the reference program inside the cell, the same call sequence as the offline oracle's
missions, so the mission replays the offline reference's mission on this site) and finishes the mission. The
connection stays open through reference_run, which takes several minutes of wall clock."""

import importlib.util
import os
import sys


def main(argv):
    lib, here = argv[1], argv[2]
    sys.path.insert(0, lib)
    from plantcell.graded import connect_graded
    token = open(os.path.join(here, "oracle_token")).read().strip()
    spec = importlib.util.spec_from_file_location("oracle_patrol", os.path.join(here, "patrol.py"))
    patrol = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(patrol)
    robot = connect_graded()
    print("[oracle] graded episode's service: %s; status %s" % (robot.service, robot.status()), file=sys.stderr)
    res = robot._call("start", oracle_token=token)
    print("[oracle] graded mission started: %s" % (res,), file=sys.stderr)
    patrol.solve(robot)
    print("[oracle] finish: %s" % (robot.finish(),), file=sys.stderr)
    print("[oracle] status %s" % (robot.status(),), file=sys.stderr)
    robot.close()
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))

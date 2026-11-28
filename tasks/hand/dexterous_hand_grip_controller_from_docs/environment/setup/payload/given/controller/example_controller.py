"""An example controller: it shows the interface and nothing more. It switches every servo on in the cell's open
pre-shape and holds it there; it never closes on a specimen, never reports a grasp and never declines. A controller
that does only this completes no job.

Copy it to submission/controller/controller.py and run the self-test (check_controller.py) to see the interface work.
Standard library only.
"""

DEG_PER_COUNT = 360.0 / 4096


class Controller(object):
    def __init__(self, workdir):
        self.workdir = workdir          # an empty folder this controller may write to
        self.goal = None
        self.started = False

    def setup(self, record, cell):
        """record: this hand unit's own record; cell: the cell's data. Returns the cycle period and the read blocks."""
        joints = {s["id"]: s for s in record["servos"]}
        goal = []
        for sid in range(13):
            s = joints[sid]
            deg = float(cell["taught"]["open"][s["joint"]])
            goal.append(int(round(s["home_offset"] + s["direction"] * deg / DEG_PER_COUNT)))
        self.goal = goal
        # one block: Present Position (address 132, 4 bytes), a 10 ms cycle
        return {"period_ms": 10, "reads": [[132, 4]]}

    def step(self, obs):
        """obs: the registers read this cycle, the newest tactile frame, the cell's phase and ticket."""
        if not self.started:
            self.started = True
            return {"write": {"goal_position": list(self.goal), "torque_enable": [1] * 13}}
        return {"write": {"goal_position": list(self.goal)}, "grasped": False, "decline": False}

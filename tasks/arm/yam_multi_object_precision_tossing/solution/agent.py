"""Observation-driven reference controller using nominal motion templates."""
import json
from pathlib import Path
import sys

import mujoco as mj
import numpy as np

from manipulation import carry, pickup, tool_position
from workcell import ArenaRound, TABLE_HEIGHT
from arcade_moves import configuration
from grips import make_grip
from wood_reference import WOOD, aim_wood_at_receiver

SOURCE = Path(__file__).parent
YELLOW_SPHERE = "left_item5"
SORTING_HEIGHTS = {1.45: .12, 1.86: .008, 2.02: .005, 2.24: .008, 2.68: .145}
RIM_HEIGHTS = {.68: .15, 1.05: .075, 1.56: .0, 1.72: .0, 1.98: .002, 2.60: .15}


class Reference:
    def __init__(self, clips):
        self.clips = clips
        self.games = json.loads((SOURCE / "learned-games.json").read_text())
        self.grips = json.loads((SOURCE / "learned-grips.json").read_text())
        self.kinematics = ArenaRound(17)
        self.forward = mj.MjData(self.kinematics.model)

    def effort(self, obj, observation):
        arm = observation["arms"][obj["arm"]]
        distance = np.linalg.norm(np.asarray(obj["position"]) - arm["hand"])
        last = observation["previous_item"]
        repeat_arm = .16 if last and self.kinematics.items[last].arm == obj["arm"] else 0.
        sphere = .24 if obj["label"] == "yellow round" else 0.
        return distance + repeat_arm - sphere

    def template(self, obj):
        if obj["arm"] == "right" or obj["label"] == "yellow round":
            return YELLOW_SPHERE if obj["label"] == "yellow round" else obj["id"]
        options = [item for item in self.kinematics.items.values()
                   if item.arm == "left" and item.variant == obj["variant"] and item.id != YELLOW_SPHERE]
        return min(options, key=lambda item: (
            item.label != obj["label"],
            np.linalg.norm(self.kinematics.data.xpos[item.body] - np.asarray(obj["position"])),
        )).id

    def first_waypoint(self, current, rows):
        rows[0][1:7] = current["joints"]
        rows[0][-1] = min(.04695, max(0., current["jaw"]))

    def align_pickup(self, obj, rows):
        arena = self.kinematics
        arm = arena.arms[obj["arm"]]
        site = arena.model.site(f"{obj['arm']}_grasp_site").id
        grip = np.array(obj["position"])
        heights = SORTING_HEIGHTS
        if obj["mode"] == "ring":
            heights = RIM_HEIGHTS
            piece = arena.items[obj["id"]]
            rim_points = np.array([arena.data.geom(i).xpos for i in piece.geoms])
            home_side = np.argmin(rim_points[:, 0])
            grip = grip + rim_points[home_side] - arena.data.xpos[piece.body]
        for row in rows:
            height = heights.get(row[0])
            if height is None:
                continue
            target = grip + np.array([0., 0., height])
            q, _ = tool_position(arena.model, self.forward, site, arm["qpos"][:6],
                                 np.array(row[1:7]), arm["dof"], target)
            row[1:7] = q.tolist()

    def act(self, _turn, observation):
        objects = observation["items"]
        last = observation["previous_item"]
        sphere = next((obj for obj in objects if obj["id"] == last and obj["label"] == "yellow round"
                       and obj["attempts"] == 1 and obj["position"][2] > TABLE_HEIGHT + .06), None)
        if sphere is not None:
            self.kinematics.arms["left"]["ref"] = np.array(observation["arms"]["left"]["reference"])
            rows = carry(self.kinematics, sphere)
            if rows:
                return {"arm": "left", "item": sphere["id"], "waypoints": rows}
        candidates = [obj for obj in objects if not obj["attempts"]]
        if not candidates:
            return {"finish": True}
        obj = min(candidates, key=lambda o: self.effort(o, observation))
        side, item_id = obj["arm"], obj["id"]
        template = self.template(obj)
        current = observation["arms"][side]
        rows = [list(row) for row in self.clips[template]]
        self.first_waypoint(current, rows)
        if template == YELLOW_SPHERE:
            qpos = self.kinematics.arms["left"]["qpos"][:6]
            self.kinematics.data.qpos[qpos] = current["joints"]
            rows = pickup(self.kinematics, obj, rows)
            self.first_waypoint(current, rows)
        else:
            self.align_pickup(obj, rows)
        if item_id == WOOD:
            limits = self.kinematics.arms[side]["range"]
            aim_wood_at_receiver(rows, current["base"], obj["target"], limits)
        if item_id in self.grips:
            rows = make_grip(self.kinematics, obj, rows, self.grips[item_id])
        if template in self.games:
            return configuration(self.kinematics, self.kinematics.items[item_id], rows, self.clips, self.games[template], obj["target"])
        return {"arm": side, "item": item_id, "waypoints": rows}


def main():
    controller = Reference(json.loads((SOURCE / "fixed_motions.json").read_text()))
    print("READY", flush=True)
    for line in sys.stdin:
        request = json.loads(line)
        request_type = request["type"]
        if request_type == "act":
            action = controller.act(request["t"], request["obs"])
            print(json.dumps({"action": action}), flush=True)
        else:
            print('{"ok":true}', flush=True)
            if request_type == "close":
                break


if __name__ == "__main__":
    main()

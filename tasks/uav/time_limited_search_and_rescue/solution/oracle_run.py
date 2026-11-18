"""Read the in-world floor plan and find the sled through the public RGB client."""
import base64
from collections import deque
from dataclasses import dataclass
import io
import json
import math
from pathlib import Path
import sys

import numpy as np
from PIL import Image

sys.path.insert(0, "/opt/visual_robotics/client")

PITCH, MODULE = 1.72, .29
PACE = np.array([.32, .32, .19])
START = (0., 0., 1.24)
PLAN = (-PITCH, 0.)
FOCAL = 128/math.tan(math.radians(47))
TRAVEL = 1.34
SLOT = .71


def decode(observation):
    pixels = base64.b64decode(observation["images"][0])
    return np.asarray(Image.open(io.BytesIO(pixels)), dtype=np.int16)


def xy(cell, pitch=PITCH):
    return np.array([(cell[0]-2)*pitch, cell[1]*pitch])


@dataclass(frozen=True)
class Plan:
    halls: tuple
    inlet: int
    outlet: int
    rooms: tuple


def read_plan(observation):
    rgb = decode(observation)
    scale = FOCAL/(START[2]-.145-.051)

    def color(local):
        u, v = np.rint(np.array([127.5, 127.5])+(local-[0., .20])*[scale, -scale]).astype(int)
        return np.median(rgb[v-1:v+2, u-1:u+2], axis=(0, 1))

    def ink(cell):
        return xy(cell, MODULE)-[0., 2*MODULE]

    halls, rooms = [], []
    for y in range(1, 5):
        for x in range(5):
            a = (x, y)
            r, g, b = color(ink(a))
            if g > 110 and g > r+55:
                rooms.append(a)
            for dx, dy in ((1, 0), (0, 1)):
                b_cell = (x+dx, y+dy)
                if max(b_cell) >= 5:
                    continue
                r, g, b = color((ink(a)+ink(b_cell))/2)
                if min(r, g, b) < 145:  # dark floor or amber raised sill, not white wall
                    halls.append(tuple(sorted((a, b_cell))))
    inlet, outlet = None, None
    for x in range(5):
        r, g, b = color((ink((x, 0))+ink((x, 1)))/2)
        if b > r+65 and b > 120:
            outlet = x
        elif g > b+55 and r > 130:
            inlet = x
    if inlet is None or outlet is None or len(rooms) != 3:
        raise RuntimeError(f"unreadable plan: inlet {inlet}, outlet {outlet}, bays {rooms}")
    return Plan(tuple(halls), inlet, outlet, tuple(rooms))


def path(start, goal, halls):
    reached = {start: None}
    visit = deque([start])
    while visit:
        a = visit.popleft()
        if a == goal:
            route = []
            while a is not None:
                route.append(a)
                a = reached[a]
            return list(reversed(route))
        for u, v in halls:
            b = v if u == a else u if v == a else None
            if b is not None and b not in reached:
                reached[b] = a
                visit.append(b)
    raise RuntimeError(f"no hallway connection from {start} to {goal}")


class Pilot:
    def __init__(self, client):
        self.client = client
        self.command = np.array(START)
        self.obs = client.observe()
        self.flown = {"search": [], "egress": []}

    def act(self, action):
        if not self.obs["done"]:
            self.obs = self.client.act(action)
            self.command += PACE*action[:3]

    def go(self, x, y, z=TRAVEL, settle=0):
        target = np.array([x, y, z])
        while not self.obs["done"] and np.max(np.abs(target-self.command)) > 1e-7:
            self.act([*np.clip((target-self.command)/PACE, -1., 1.).tolist(), 0.])
        for _ in range(settle):
            self.grip(0.)

    def cell(self, cell, z=TRAVEL, settle=0):
        self.go(*xy(cell), z, settle)

    def grip(self, signal):
        self.act([0., 0., 0., signal])

    def follow(self, route, stage):
        for cell in route:
            if self.obs["done"]:
                break
            self.cell(cell)
            self.flown[stage].append(cell)


def find_pod(observation, command):
    r, g, b = decode(observation).transpose(2, 0, 1)
    rows, cols = np.where((r > 150) & (g > 48) & (r > 1.7*g) & (b < 55))
    if rows.size < 230 or np.ptp(cols) > 68 or np.ptp(rows) > 80:
        return None
    u, v = (cols.min()+cols.max())/2, (rows.min()+rows.max())/2
    scale = (command[2]-.145-.177)/FOCAL
    return command[:2]+[(u-127.5)*scale, .20-(v-127.5)*scale]


def run(client, plan=None, target=None, retrace=False, locate_only=False, visit_plan=True):
    pilot = Pilot(client)
    if visit_plan or plan is None:
        pilot.go(*PLAN, START[2], settle=3)
    if plan is None:
        plan = read_plan(pilot.obs)
    inlet = (plan.inlet, 1)
    pilot.cell((plan.inlet, 0), SLOT, settle=1)
    pilot.cell(inlet, SLOT)
    pilot.cell(inlet)
    if pilot.obs["done"]:
        return pilot
    current = inlet
    if target is not None:
        goal = (int(round(target[0]/PITCH))+2, int(round(target[1]/PITCH)))
        pilot.follow(path(current, goal, plan.halls), "search")
        current = goal
    else:
        goals = set(plan.rooms)
        while goals and not pilot.obs["done"]:
            route = min((path(current, goal, plan.halls) for goal in sorted(goals)),
                        key=len)
            pilot.follow(route, "search")
            current = route[-1]
            goals.remove(current)
            pilot.cell(current, settle=2)
            if not pilot.obs["done"]:
                target = find_pod(pilot.obs, pilot.command)
                if target is not None:
                    break
    if pilot.obs["done"] or locate_only:
        return pilot
    if target is None:
        raise RuntimeError("no rescue sled in the marked rooms")
    px, py = map(float, target)
    pilot.go(px, py, .43, settle=3)
    pilot.grip(1.)
    pilot.go(px, py)
    pilot.cell(current)
    if retrace:
        pilot.follow(reversed(pilot.flown["search"]), "egress")
        pilot.cell(inlet, SLOT, settle=2)
        pilot.cell((plan.inlet, 0), SLOT)
        return pilot
    outlet = (plan.outlet, 1)
    pilot.follow(path(current, outlet, plan.halls), "egress")
    pilot.cell((plan.outlet, 0))
    pilot.cell((2, 0))
    pilot.cell((2, 0), .43, settle=3)
    pilot.grip(-1.)
    for _ in range(3):
        pilot.grip(0.)
    return pilot


if __name__ == "__main__":
    from client import Client

    pilot = run(Client())
    out = Path("/home/user/submission")
    for i, frame in enumerate(pilot.obs["images"]):
        (out/f"final-camera-{i}.png").write_bytes(base64.b64decode(frame))
    (out/"reference-route.json").write_text(json.dumps(pilot.flown, indent=2))
    print(json.dumps({k: v for k, v in pilot.obs.items() if k != "images"}), flush=True)

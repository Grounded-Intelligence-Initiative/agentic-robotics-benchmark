"""RGB-only local exploration and evacuation; no layout or simulator imports."""
import base64
from collections import deque
import io
import json
import math
from pathlib import Path
import sys

import numpy as np
from PIL import Image

sys.path.insert(0, "/opt/visual_robotics/client")
PITCH, APRON, DECK = 1.72, .63, .60
DOWN_OFFSET = .46
PACE = np.array([.32, .32, .19])
TRAVEL, SLOT = 1.30, .71
TURN = math.pi/6
FOCAL = 128/math.tan(math.radians(47))
DIRECTIONS = ((0, 1), (1, 0), (-1, 0), (0, -1))


def basis(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s], [s, c]])


def decode(observation, camera=0):
    return np.asarray(Image.open(io.BytesIO(base64.b64decode(observation["images"][camera]))),
                      dtype=np.int16)


def apron_color(rgb, local):
    scale = FOCAL/(TRAVEL-.145-.011)
    u, v = np.rint(np.array([127.5, 127.5])+(local-[0., DOWN_OFFSET])*[scale, -scale]).astype(int)
    r, g, b = np.median(rgb[v-3:v+4, u-3:u+4], axis=(0, 1))
    if r > 150 and g > 120 and b < 108:
        return "landing"
    if r > 155 and g < 100 and b < 100:
        return "closed"
    if b > 145 and r > 145 and g < min(r, b)-45:
        return "inspection"
    if b > 140 and b > r+60 and b > g+40:
        return "cargo"
    if min(r, g, b) > 145 and max(r, g, b)-min(r, g, b) < 45:
        return "open"
    return "wall"


def find_pod(rgb, command, floor):
    r, g, b = rgb.transpose(2, 0, 1)
    rows, cols = np.where((r > 160) & (g > 48) & (r > 1.7*g) & (b < 55))
    if rows.size < 250 or np.ptp(cols) > 77 or np.ptp(rows) > 84:
        return None
    u, v = (cols.min()+cols.max())/2, (rows.min()+rows.max())/2
    scale = (command[2]-.145-floor-.177)/FOCAL
    local = np.array([(u-127.5)*scale, DOWN_OFFSET-(v-127.5)*scale])
    return command[:2]+basis(command[3]) @ local


class Pilot:
    def __init__(self, client):
        self.client = client
        self.command = np.array([0., 0., TRAVEL, 0.])
        self.obs = client.observe()
        self.trace = []
        self.path = []

    def act(self, action):
        if not self.obs["done"]:
            self.obs = self.client.act(action)
            delta = PACE*action[:3]
            delta[:2] = basis(self.command[3]) @ delta[:2]
            self.command[:3] += delta
            self.command[3] += TURN*action[3]

    def grip(self, value=0.):
        self.act([0., 0., 0., 0., value])

    def face(self, dx, dy):
        heading = math.atan2(-dx, dy)
        error = math.remainder(heading-self.command[3], 2*math.pi)
        while not self.obs["done"] and abs(error) > 1e-7:
            move = float(np.clip(error/TURN, -1, 1))
            self.act([0., 0., 0., move, 0.])
            error -= move*TURN

    def go(self, x, y, z, settle=0):
        target = np.array([x, y, z])
        while not self.obs["done"] and np.max(np.abs(target-self.command[:3])) > 1e-7:
            error = target-self.command[:3]
            error[:2] = basis(self.command[3]).T @ error[:2]
            self.act([*np.clip(error/PACE, -1., 1.).tolist(), 0., 0.])
        for _ in range(settle):
            self.grip()

    def cell(self, cell, z, settle=0):
        x, y = PITCH*np.asarray(cell)
        self.go(x, y, z, settle)

    def scan(self):
        rgb = decode(self.obs)
        transform = basis(self.command[3]).T
        return rgb, {d: apron_color(rgb, transform @ np.asarray(d)*APRON) for d in DIRECTIONS}


def add(a, b):
    return tuple(x+y for x, y in zip(a, b))


def delta(a, b):
    return tuple(y-x for x, y in zip(a, b))


class Discovery:
    """Topological map built only from observed local doorway aprons."""
    def __init__(self, pilot, inlet):
        self.pilot, self.current = pilot, inlet
        self.floors = {inlet: 0.}
        self.adjacency = {}
        self.seen = set()
        self.closures, self.invalidated = set(), set()
        self.confirmed_shutters = set()
        self.signatures = {}
        self.cargo = None
        self.previous = None

    def scan(self, loaded):
        p, at = self.pilot, self.current
        rgb, view = p.scan()
        doors = self.adjacency.setdefault(at, {})
        expected = self.signatures.get(at)
        if loaded:
            # A suspended load can occlude the aft apron. Resolve contradictions
            # with a second camera heading before overwriting the remembered map.
            known = expected if expected is not None else (
                (delta(at, self.previous),) if self.previous is not None else ())
            for d in DIRECTIONS:
                kind = view[d]
                if kind != "closed" and not (d in known and kind == "wall"):
                    continue
                there = add(at, d)
                edge = tuple(sorted((at, there)))
                if edge in self.confirmed_shutters:
                    view[d] = "closed"
                    continue
                p.face(*d)
                p.grip()
                _, second = p.scan()
                view[d] = second[d]
                if "closed" in (kind, second[d]):
                    front = decode(p.obs, camera=1)[68:188, 66:190]
                    r, g, b = front.transpose(2, 0, 1)
                    red = float(np.mean((r > 155) & (g < 105) & (b < 105)))
                    if red > .28:
                        self.confirmed_shutters.add(edge)
                        view[d] = "closed"
                    elif second[d] == "closed":
                        view[d] = doors.get(there, self.adjacency.get(there, {}).get(at, "wall"))
                    p.trace.append({"event": "shutter_probe", "cell": at, "direction": d,
                                    "red_face_fraction": red, "confirmed": red > .28,
                                    "frame": p.obs["frame_id"]})
                else:
                    p.trace.append({"event": "parallax_probe", "cell": at, "direction": d,
                                    "result": second[d], "frame": p.obs["frame_id"]})
        signature = tuple(d for d, kind in view.items() if kind != "wall")
        if at in self.signatures:
            if signature != self.signatures[at]:
                raise RuntimeError(f"doorway landmarks disagree at revisited cell {at}: {view}")
            p.trace.append({"event": "revisit_confirmed", "cell": at, "frame": p.obs["frame_id"]})
        self.signatures[at] = signature
        if "cargo" in view.values():
            self.cargo = at
        for d, kind in view.items():
            there = add(at, d)
            if kind == "closed":
                edge = tuple(sorted((at, there)))
                if there in doors or at in self.adjacency.get(there, {}):
                    doors.pop(there, None)
                    self.adjacency.get(there, {}).pop(at, None)
                    self.invalidated.add(edge)
                continue
            if kind not in ("open", "landing"):
                continue
            floor = self.floors[at] if kind == "open" else round(DECK-self.floors[at], 2)
            if there in self.floors and floor != self.floors[there]:
                raise RuntimeError("inconsistent landing observation")
            if there in self.seen and there != self.previous and there not in doors:
                self.closures.add(tuple(sorted((at, there))))
            self.floors[there] = floor
            doors[there] = kind
        self.seen.add(at)
        p.trace.append({"event": "local_rgb", "cell": at, "floor": self.floors[at],
                        "view": list(view.values()), "frame": p.obs["frame_id"], "loaded": loaded})
        return rgb, view

    def route(self, goal=None):
        parent = {self.current: None}
        visit = deque([self.current])
        frontiers = []
        while visit:
            at = visit.popleft()
            if goal is not None and at == goal:
                frontiers = [at]
                break
            if at not in self.seen:
                frontiers.append(at)
                continue
            for there in self.adjacency.get(at, {}):
                if there not in parent:
                    parent[there] = at
                    visit.append(there)
        if not frontiers:
            raise RuntimeError(f"no accessible frontier at {self.current} after visiting {len(self.seen)} cells")

        def unwind(end):
            route = []
            while end != self.current:
                route.append(end)
                end = parent[end]
            return list(reversed(route))

        # Prefer nearby frontiers. On ties look into the mezzanine on search,
        # and walk the upper deck before exploring the lower floor on egress.
        def preference(at):
            heading = delta(self.current, unwind(at)[0])
            a = math.atan2(-heading[0], heading[1])
            return len(unwind(at)), -self.floors[at], abs(math.remainder(a-self.pilot.command[3], 2*math.pi))

        end = frontiers[0] if goal in frontiers else min(frontiers, key=preference)
        return unwind(end)[0]

    def move(self, there):
        p, here = self.pilot, self.current
        p.face(*delta(here, there))
        if self.floors[here] != self.floors[there]:
            p.trace.append({"event": "landing_probe", "cell": here, "floor": self.floors[here],
                            "frame": p.obs["frame_id"]})
            p.cell(here, TRAVEL+DECK)
            p.cell(there, TRAVEL+DECK)
        p.cell(there, TRAVEL+self.floors[there], settle=1)
        self.previous, self.current = here, there
        p.path.append(there)


def run(client, retrace=False, locate_only=False):
    p = Pilot(client)
    inlet = None
    for col in (0, 1, 2, -1, -2):
        p.cell((col, 0), TRAVEL, settle=1)
        _, view = p.scan()
        if view[(0, 1)] == "inspection":
            inlet = (col, 1)
            break
    if inlet is None:
        raise RuntimeError("no visible inspection slot")
    p.cell((inlet[0], 0), SLOT, settle=1)
    p.cell(inlet, SLOT)
    p.cell(inlet, TRAVEL, settle=1)
    nav = Discovery(p, inlet)
    p.path.append(inlet)
    target = None
    while not p.obs["done"]:
        rgb, _ = nav.scan(False)
        target = find_pod(rgb, p.command, nav.floors[nav.current])
        if (target is not None and nav.floors[nav.current] == DECK
                and np.max(np.abs(target-p.command[:2])) < .45):
            break
        nav.move(nav.route())
    if p.obs["done"] or locate_only:
        return p
    px, py = map(float, target)
    p.go(px, py, .43+DECK, settle=3)
    p.grip(1.)
    p.go(px, py, TRAVEL+DECK)
    p.cell(nav.current, TRAVEL+DECK, settle=1)
    p.trace.append({"event": "pickup_attempt", "cell": nav.current, "frame": p.obs["frame_id"]})
    while not p.obs["done"]:
        _, view = nav.scan(True)
        if retrace and nav.current == inlet:
            p.face(0, -1)
            p.cell(inlet, SLOT, settle=2)
            p.cell((inlet[0], 0), SLOT)
            return p
        if not retrace and "cargo" in view.values():
            d = next(d for d, k in view.items() if k == "cargo")
            p.face(*d)
            p.cell(add(nav.current, d), TRAVEL)
            break
        nav.move(nav.route(goal=inlet if retrace else nav.cargo))
    p.cell((0, 0), TRAVEL)
    p.face(0, 1)
    p.cell((0, 0), .43, settle=3)
    p.grip(-1.)
    for _ in range(3):
        p.grip()
    p.trace.append({"event": "topology_summary", "views": len(nav.seen),
                    "loop_edges": sorted(nav.closures), "invalidated_edges": sorted(nav.invalidated)})
    return p


if __name__ == "__main__":
    from client import Client

    pilot = run(Client())
    out = Path("/home/user/submission")
    for i, frame in enumerate(pilot.obs["images"]):
        (out/f"final-camera-{i}.png").write_bytes(base64.b64decode(frame))
    (out/"reference-route.json").write_text(json.dumps(pilot.trace, indent=2))
    print(json.dumps({k: v for k, v in pilot.obs.items() if k != "images"}), flush=True)

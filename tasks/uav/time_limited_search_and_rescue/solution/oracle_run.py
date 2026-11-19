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
from scipy import ndimage

sys.path.insert(0, "/opt/visual_robotics/client")
PITCH, APRON, DECK = 1.72, .63, .60
DOWN_OFFSET = .46
PACE = np.array([.32, .32, .19])
TRAVEL, SLOT = 1.30, .71
TURN = math.pi/6
FOCAL = 128/math.tan(math.radians(47))
DIRECTIONS = ((0, 1), (1, 0), (-1, 0), (0, -1))
MARKS = np.array([(x, y) for x in (-.29, .29) for y in (-.29, .29)])


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


def steel(rgb):
    r, g, b = rgb.transpose(2, 0, 1)
    return (r < 115) & (g > 145) & (b > 165) & (g > r+65)


def survey(p, target, cell):
    # Pick the side with room to offset from the sled. At cruise height the
    # common cover hides the flange; see under it in the forward camera.
    relative = target-PITCH*np.asarray(cell)
    side = -1 if relative[1] > 0. else 1
    offset = np.array([0., side*.55])
    diagonal = abs(relative[1]) < .09
    if diagonal:
        # Keep both horizontal components clear of the nearest walls.
        offset = np.where(relative > 0., -1., 1.)*(.55/math.sqrt(2))
    p.face(*(-offset))
    p.go(*(target+offset), .43+DECK, settle=3)
    rows, cols = np.where(steel(decode(p.obs, 1)[130:244, 75:181]))
    if rows.size < 40:
        raise RuntimeError("no flange visible under the fitting cover")
    width = int(np.ptp(cols)+1)
    wide = width >= 37
    p.trace.append({"event": "flange_survey", "cell": cell, "width_px": width,
                    "profile": "wide" if wide else "narrow", "frame": p.obs["frame_id"]})
    p.go(*p.command[:2], TRAVEL+DECK)
    if diagonal:
        p.face(0, -side)
    p.cell(cell, TRAVEL+DECK, settle=1)
    p.register(cell)
    return wide


def equip(p, wide):
    p.face(0, 1)
    p.grip()
    mask = steel(decode(p.obs))
    tools = []
    scale = (TRAVEL-.145-.237)/FOCAL
    for left, right in ((60, 127), (128, 196)):
        rows, cols = np.where(mask[145:206, left:right])
        if rows.size < 50:
            raise RuntimeError("depot collet not visible")
        u = left+(cols.min()+cols.max())/2
        v = 145+(rows.min()+rows.max())/2
        tools.append((int(np.ptp(cols)), (u-127.5)*scale, DOWN_OFFSET-(v-127.5)*scale))
    width, dx, dy = sorted(tools)[int(wide)]
    p.trace.append({"event": "depot_choice", "profile": "wide" if wide else "narrow",
                    "slot": int(dx > 0), "width_px": width, "frame": p.obs["frame_id"]})
    center = p.command[:2].copy()
    p.go(*(center+[dx, dy]), .43, settle=3)
    p.grip(1.)
    p.go(*(center+[dx, dy]), TRAVEL)
    p.trace.append({"event": "depot_departure", "frame": p.obs["frame_id"]})
    p.cell((2, 0), TRAVEL, settle=1)
    p.register((2, 0))


def station_offset(rgb):
    """Find a repeated local 58 cm square in the calibrated cruise view."""
    r, g, b = rgb.transpose(2, 0, 1)
    labels, count = ndimage.label((r > 185) & (g < 70) & (b > 92) & (r > b+28))
    if not count:
        return None
    sizes = np.bincount(labels.ravel())
    ids = [i for i in range(1, count+1) if 16 <= sizes[i] <= 150]
    if len(ids) < 2:
        return None
    scale = (TRAVEL-.145-.016)/FOCAL
    uv = np.asarray(ndimage.center_of_mass(np.ones_like(labels), labels, ids))[:, ::-1]
    spots = (uv-127.5)*[scale, -scale]+[0., DOWN_OFFSET]
    hypotheses = (spots[:, None, :]-MARKS[None, :, :]).reshape(-1, 2)
    fits = []
    for center in hypotheses:
        if np.max(np.abs(center)) > .43:
            continue
        error = np.linalg.norm(spots[:, None, :]-center-MARKS[None, :, :], axis=2)
        neighbors = error.argmin(axis=0)
        matched = error[neighbors, np.arange(len(MARKS))] < .058
        if sum(matched) >= 2:
            fit = np.mean(spots[neighbors[matched]]-MARKS[matched], axis=0)
            fits.append((-sum(matched), np.linalg.norm(fit), fit))
    return min(fits, key=lambda f: f[:2])[2] if fits else None


def pad_offset(rgb):
    r, g, b = rgb.transpose(2, 0, 1)
    labels, count = ndimage.label((b > 145) & (b > r+65) & (b > g+45))
    if not count:
        return None
    largest = 1+np.argmax(np.bincount(labels.ravel())[1:])
    rows, cols = np.where(labels == largest)
    if rows.size < 2200:
        return None
    scale = (TRAVEL-.145-.008)/FOCAL
    u, v = (cols.min()+cols.max())/2, (rows.min()+rows.max())/2
    return (np.array([u, v])-127.5)*[scale, -scale]+[0., DOWN_OFFSET]


class Pilot:
    def __init__(self, client, feedback="rgb"):
        self.client = client
        self.command = np.array([0., 0., TRAVEL, 0.])
        self.obs = client.observe()
        self.trace = []
        self.path = []
        self.feedback, self.loaded = feedback, False

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

    def register(self, cell):
        if self.feedback == "odometry" or (self.feedback == "prelift" and self.loaded):
            return
        if self.obs["done"]:
            return
        initial = None
        before_frame = None
        turned = False
        for _ in range(6):
            offset = station_offset(decode(self.obs))
            if offset is None:
                if turned:
                    break
                # Change the optical viewpoint around the suspended sled;
                # a hold stabilizes on the same patch without adding drift.
                self.face(*(-basis(self.command[3])[:, 0]))
                self.grip()
                turned = True
                continue
            error = float(np.linalg.norm(offset))
            if initial is None:
                initial = error
                before_frame = self.obs["frame_id"]
            if error <= .036:
                self.command[:2] = PITCH*np.asarray(cell)
                self.trace.append({"event": "station_fix", "cell": cell, "loaded": self.loaded,
                                   "before_m": round(initial, 4), "after_m": round(error, 4),
                                   "before_frame": before_frame,
                                   "frame": self.obs["frame_id"]})
                return
            self.act([*np.clip(.94*offset/PACE[:2], -1., 1.).tolist(), 0., 0., 0.])
            self.grip()
        if not self.obs["done"]:
            raise RuntimeError(f"cannot establish a local station fix at {cell}")

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
        self.crossed = set()
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
        if not loaded:
            # Before lift the shutters do not move. An occluded incoming apron
            # cannot undo the crossing we just made. Keep the survey camera pose.
            for there, kind in doors.items():
                d = delta(at, there)
                if view[d] == "wall" and frozenset((at, there)) in self.crossed:
                    view[d] = kind
        else:
            # After lift a known passage can really be closed. Face it to check.
            known = set(expected or ())
            known.update(d for d in DIRECTIONS if frozenset((at, add(at, d))) in self.crossed)
            for d in DIRECTIONS:
                kind = view[d]
                there = add(at, d)
                edge = tuple(sorted((at, there)))
                if edge in self.confirmed_shutters:
                    view[d] = "closed"
                    continue
                if kind != "closed" and not (d in known and kind == "wall"):
                    continue
                p.face(*d)
                p.grip()
                rgb, second = p.scan()
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
                    self.crossed.discard(frozenset((at, there)))
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

    def route(self, goal=None, *, observed_detour=False):
        """Path excluding the current cell; [] means arrived, None means unreachable.

        An explicit destination uses actual crossings, unless egress requests
        an observed detour. Exploration stops at an unscanned station.
        """
        visited = {self.current}
        visit = deque([(self.current, [])])
        frontiers = []
        while visit:
            at, path = visit.popleft()
            if at == goal:
                return path
            if goal is None and at not in self.seen:
                if not path:
                    return []
                frontiers.append(path)
                continue
            for there in self.adjacency.get(at, {}):
                if there in visited:
                    continue
                if goal is not None and not observed_detour and frozenset((at, there)) not in self.crossed:
                    continue
                visited.add(there)
                visit.append((there, path+[there]))
        if not frontiers:
            return None

        # Prefer nearby frontiers. On ties look into the mezzanine on search,
        # and walk the upper deck before exploring the lower floor on egress.
        def preference(path):
            heading = delta(self.current, path[0])
            a = math.atan2(-heading[0], heading[1])
            return len(path), -self.floors[path[-1]], abs(math.remainder(a-self.pilot.command[3], 2*math.pi))

        return min(frontiers, key=preference)

    def return_to(self, goal):
        path = self.route(goal)
        if path is None:
            raise RuntimeError(f"no crossed route from {self.current} to {goal}")
        self.pilot.trace.append({"event": "known_return", "cell": self.current, "goal": goal,
                                 "path": path, "frame": self.pilot.obs["frame_id"]})
        for there in path:
            if self.pilot.obs["done"]:
                break
            self.move(there)

    def explore(self):
        path = self.route()
        if path is None:
            raise RuntimeError(f"no accessible frontier at {self.current} after visiting {len(self.seen)} cells")
        if path:
            self.move(path[0])

    def move(self, there):
        p, here = self.pilot, self.current
        p.face(*delta(here, there))
        if self.floors[here] != self.floors[there]:
            p.trace.append({"event": "landing_probe", "cell": here, "floor": self.floors[here],
                            "frame": p.obs["frame_id"]})
            p.cell(here, TRAVEL+DECK)
            p.cell(there, TRAVEL+DECK)
        p.cell(there, TRAVEL+self.floors[there], settle=1)
        p.register(there)
        if p.obs["done"]:
            return
        self.crossed.add(frozenset((here, there)))
        self.adjacency.setdefault(there, {})[here] = self.adjacency[here][there]
        self.previous, self.current = here, there
        p.path.append(there)


def run(client, retrace=False, locate_only=False, feedback="rgb", fixed_tool=None):
    p = Pilot(client, feedback)

    def enter(inlet):
        p.face(0, 1)
        p.cell((inlet[0], 0), SLOT, settle=1)
        p.cell(inlet, SLOT)
        p.cell(inlet, TRAVEL, settle=1)
        p.register(inlet)

    def staging_to(start, end):
        for col in range(start, end, 1 if start < end else -1):
            cell = (col+(1 if start < end else -1), 0)
            p.cell(cell, TRAVEL, settle=1)
            p.register(cell)

    inlet = None
    tried = set()
    for col in (0, 1, 2, 1, 0, -1, -2):
        p.cell((col, 0), TRAVEL, settle=1)
        p.register((col, 0))
        if col in tried:
            continue
        tried.add(col)
        _, view = p.scan()
        if view[(0, 1)] == "inspection":
            inlet = (col, 1)
            break
    if inlet is None:
        raise RuntimeError("no visible inspection slot")
    enter(inlet)
    nav = Discovery(p, inlet)
    p.path.append(inlet)
    target = None
    while not p.obs["done"]:
        rgb, _ = nav.scan(False)
        target = find_pod(rgb, p.command, nav.floors[nav.current])
        if (target is not None and nav.floors[nav.current] == DECK
                and np.max(np.abs(target-p.command[:2])) < .45):
            break
        nav.explore()
    if p.obs["done"] or locate_only:
        return p
    bay = nav.current
    p.trace.append({"event": "sled_found", "cell": bay, "frame": p.obs["frame_id"]})
    wide = survey(p, target, bay) if fixed_tool is None else fixed_tool == "wide"
    nav.return_to(inlet)
    p.face(0, -1)
    p.cell(inlet, SLOT, settle=1)
    p.cell((inlet[0], 0), SLOT)
    p.cell((inlet[0], 0), TRAVEL, settle=1)
    p.register((inlet[0], 0))
    staging_to(inlet[0], 2)
    equip(p, wide)
    staging_to(2, inlet[0])
    enter(inlet)
    nav.return_to(bay)
    target = find_pod(decode(p.obs), p.command, DECK)
    if target is None:
        raise RuntimeError("sled not visible on return to remembered bay")
    p.trace.append({"event": "sled_reacquired", "cell": bay, "frame": p.obs["frame_id"]})
    px, py = map(float, target)
    p.go(px, py, .43+DECK, settle=3)
    p.grip(1.)
    p.loaded = True
    p.go(px, py, TRAVEL+DECK)
    p.cell(nav.current, TRAVEL+DECK, settle=1)
    p.register(nav.current)
    p.trace.append({"event": "pickup_attempt", "cell": nav.current, "frame": p.obs["frame_id"]})
    if find_pod(decode(p.obs), p.command, DECK) is not None:
        # On a miss the orange sled is still next to the original station.
        p.trace.append({"event": "sled_still_on_floor", "frame": p.obs["frame_id"]})
        return p
    staging = None
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
            staging = add(nav.current, d)
            p.cell(staging, TRAVEL, settle=1)
            p.register(staging)
            break
        goal = inlet if retrace else nav.cargo
        path = nav.route(goal) if goal is not None else None
        if goal is not None and path is None:
            path = nav.route(goal, observed_detour=True)
        if path is None:
            # A safety shutter may have cut the known path to the exit.
            nav.explore()
        elif path:
            nav.move(path[0])
        else:
            raise RuntimeError(f"no exit apron visible at remembered destination {goal}")
    if staging is not None:
        col = staging[0]
        while col and not p.obs["done"]:
            col -= int(math.copysign(1, col))
            p.cell((col, 0), TRAVEL, settle=1)
            p.register((col, 0))
    p.face(0, 1)
    p.grip()
    if not p.obs["done"]:
        offset = pad_offset(decode(p.obs))
        if offset is None:
            raise RuntimeError("blue delivery pad not in the down view")
        tx, ty = p.command[:2]+basis(p.command[3]) @ offset
        p.go(tx, ty, .43, settle=3)
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

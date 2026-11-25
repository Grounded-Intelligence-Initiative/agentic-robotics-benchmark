"""Observation-only reference and ablations. No imports from the environment."""

import base64
from collections import deque
from functools import lru_cache
from io import BytesIO
import json
import sys

from PIL import Image

V = {"N": (0, -1), "E": (1, 0), "S": (0, 1), "W": (-1, 0), "stay": (0, 0)}
DIR = ("N", "E", "S", "W")
PALETTE = {
    k: tuple(bytes.fromhex(v))
    for k, v in {
        "hall": "b6ae9e",
        "oak": "c99762",
        "sage": "87af90",
        "slate": "849db5",
        "rose": "c095aa",
        "wall": "656577",
        "furniture": "285750",
        "dust": "684127",
        "box": "e4ac29",
        "red": "cb4d5d",
        "blue": "397ecb",
        "dock": "24a99c",
    }.items()
}
ROOMS = {"oak", "sage", "slate", "rose"}


@lru_cache(maxsize=1024)
def paint(rgb):
    if max(rgb) < 35:
        return None
    return min(
        PALETTE, key=lambda k: sum((a - b) ** 2 for a, b in zip(rgb, PALETTE[k]))
    )


def add(p, d, k=1):
    return p[0] + k * V[d][0], p[1] + k * V[d][1]


def distance(p, q):
    return abs(p[0] - q[0]) + abs(p[1] - q[1])


def read_view(encoded, origin, r):
    im = Image.open(BytesIO(base64.b64decode(encoded))).convert("RGB")
    px = im.load()
    s = im.width // (2 * r + 1)
    for j in range(2 * r + 1):
        for i in range(2 * r + 1):
            mark = paint(px[i * s + s // 2, j * s + s // 2])
            if mark is not None:
                base = paint(px[i * s + s // 4, j * s + s // 4])
                yield (origin[0] + i - r, origin[1] + j - r), mark, base


def paths(start, allowed):
    todo = deque([start])
    out = {start: ""}
    while todo:
        p = todo.popleft()
        for d in DIR:
            q = add(p, d)
            if q not in out and q in allowed:
                out[q] = out[p] + d
                todo.append(q)
    return out


def deliver(start, box, pad, free):
    """Plan box pushes, keeping a walking path to the back of each push."""
    todo = deque([(start, box, "")])
    seen = set()
    while todo:
        g, b, path = todo.popleft()
        if b == pad:
            return path
        reach = paths(g, free - {b})
        key = (b, min(reach))
        if key in seen:
            continue
        seen.add(key)
        for d in sorted(DIR, key=lambda d: distance(add(b, d), pad)):
            back = add(b, d, -1)
            new = add(b, d)
            if back in reach and new in free:
                todo.append((b, new, path + reach[back] + d))
    return ""


class Team:
    def __init__(self, mode="adaptive"):
        self.mode = mode
        self.g = self.a = (0, 0)
        self.previous = ["stay", "stay", 0, True]
        self.cells = {}
        self.air_cells = {}
        self.rooms = {}
        self.pads = {}
        self.dirt = {}
        self.cleaned = set()
        self.box = self.initial_box = None
        self.route = ""
        self.last_t = -1
        self.scout_visits = {}

    def absorb(self, img, p, r, from_air):
        for q, mark, base in read_view(img, p, r):
            surface = mark if mark in {"wall", "furniture"} else "floor"
            self.cells[q] = surface
            if from_air:
                self.air_cells[q] = surface
            if base in ROOMS | {"hall"}:
                self.rooms[q] = base
            if mark == "dust":
                self.dirt[q] = self.rooms.get(q, base)
            if mark == "box":
                self.box = q
                if self.initial_box is None:
                    self.initial_box = q
            if mark in ("red", "blue"):
                self.pads[mark] = q

    def scout(self, obs, requested, target):
        if not obs["flight"] or self.mode == "solo":
            return "stay", 0
        cells = self.air_cells if self.mode == "fixed" else self.cells
        reach = paths(self.a, {p for p, v in cells.items() if v != "wall"})
        front = [
            p for p in reach if reach[p] and any(add(p, d) not in cells for d in DIR)
        ]
        if not front:
            return "stay", 0
        if self.mode == "fixed":
            # Preassigned sweep order; detours use only the aircraft's own observations.
            dest = min(front, key=lambda p: (p[1], p[0] if p[1] % 2 == 0 else -p[0]))
        else:

            def utility(p):
                gains = sum(
                    (p[0] + x, p[1] + y) not in cells
                    for x in range(-2, 3)
                    for y in range(-2, 3)
                )
                useful = (
                    self.rooms.get(p) in requested
                    and sum(self.dirt.get(q) == self.rooms.get(p) for q in self.dirt)
                    < 2
                )
                hall = self.rooms.get(p) == "hall" and (
                    target not in self.pads or self.box is None
                )
                return (
                    1.3 * min(gains, 12)
                    + 6 * useful
                    + 5 * hall
                    - 0.95 * len(reach[p])
                    + 0.15 * distance(p, self.g)
                    - 3 * self.scout_visits.get(p, 0)
                )

            dest = max(front, key=lambda p: (utility(p), p))
        route = reach[dest]
        return route[0], 2 if len(route) > 1 and route[1] == route[0] else 1

    def choose(self, t, obs):
        if t == self.last_t:
            return self.previous
        self.last_t = t
        oldg = self.g
        self.g = add(self.g, self.previous[0], obs["moved"][0])
        self.a = add(self.a, self.previous[1], obs["moved"][1])
        if self.g != oldg and self.g == self.box:
            self.box = add(self.g, self.previous[0])
        if self.g in self.dirt or self.g == self.initial_box:
            self.cleaned.add(self.g)
        if obs["air"] and self.mode != "solo":
            self.absorb(obs["air"], self.a, 3, True)
        self.absorb(obs["ground"], self.g, 1, False)
        self.scout_visits[self.a] = self.scout_visits.get(self.a, 0) + 1
        req = {k for k in ROOMS if k in obs["instruction"].split()}
        target = "blue" if "blue" in obs["instruction"].split() else "red"
        pad = self.pads.get(target)
        free = {p for p, v in self.cells.items() if v == "floor"}
        reach = paths(self.g, free - {self.box})
        todo = [
            p
            for p, r in self.dirt.items()
            if r in req | {"hall"} and p not in self.cleaned and p in reach
        ]
        finished = all(
            sum(self.dirt.get(p) == r for p in self.cleaned) >= 2 for r in req
        )
        if self.initial_box not in self.cleaned:
            finished = False
        if self.route and obs["moved"][0] == 0:
            self.route = ""
        if (
            not self.route
            and self.box is not None
            and pad is not None
            and self.box != pad
        ):
            self.route = deliver(self.g, self.box, pad, free)
        if self.route:
            g, self.route = self.route[0], self.route[1:]
        elif todo:
            q = min(todo, key=lambda p: (len(reach[p]), p))
            g = reach[q][:1] or "stay"
        elif finished and pad is not None and self.box == pad:
            g = reach.get((0, 0), "")[:1] or "stay"
        else:
            front = [
                p
                for p, path in reach.items()
                if path and any(add(p, d) not in self.cells for d in DIR)
            ]
            if front:

                def cost(p):
                    room = self.rooms.get(p)
                    incomplete = (
                        room in req
                        and sum(self.dirt.get(q) == room for q in self.cleaned) < 2
                    )
                    return len(reach[p]) - 5 * incomplete, p

                g = reach[min(front, key=cost)][0]
            else:
                g = reach.get((0, 0), "")[:1] or "stay"
        a, stride = self.scout(obs, req, target)
        self.previous = [g, a, stride, True]
        return self.previous


if __name__ == "__main__":
    print("READY", flush=True)
    team = Team()
    for line in sys.stdin:
        msg = json.loads(line)
        if msg["type"] == "reset":
            team = Team()
            out = {"ok": True}
        elif msg["type"] == "act":
            out = {"action": team.choose(msg["t"], msg["obs"])}
        else:
            print('{"ok":true}', flush=True)
            break
        print(json.dumps(out), flush=True)

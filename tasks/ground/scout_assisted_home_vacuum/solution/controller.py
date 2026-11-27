"""Observation-only reference and ablations. No imports from the environment."""

import base64
from collections import Counter, deque
from functools import lru_cache
from io import BytesIO
import json
import sys

from PIL import Image
import numpy as np

V = {"N": (0, -1), "E": (1, 0), "S": (0, 1), "W": (-1, 0), "stay": (0, 0)}
DIR = ("N", "E", "S", "W")
PALETTE = {
    k: tuple(bytes.fromhex(v))
    for k, v in {
        "hall": "c5bba5",
        "oak": "bc936b",
        "sage": "98ad95",
        "slate": "8b9eac",
        "rose": "c2a2a4",
        "wall": "656577",
        "furniture": "2f403f",
        "dust": "684127",
        "box": "e4ac29",
        "red": "cb4d5d",
        "blue": "397ecb",
        "jade": "36b44a",
        "violet": "9b54e0",
        "dock": "24a99c",
        "runner": "945078",
    }.items()
}
ROOMS = {"oak", "sage", "slate", "rose"}
INK = {
    "oak": "f68c18",
    "sage": "00d9ec",
    "slate": "7042fa",
    "rose": "f62ac4",
    "jade": "36b44a",
    "violet": "9b54e0",
    "red": "cb4d5d",
    "blue": "397ecb",
}


def as_image(frame):
    return (
        frame
        if isinstance(frame, Image.Image)
        else Image.open(BytesIO(base64.b64decode(frame))).convert("RGB")
    )


def infer_mount(im, r):
    size = im.width // (2 * r + 1)
    cell = np.asarray(
        im.crop((r * size, r * size, (r + 1) * size, (r + 1) * size)), dtype=float
    )
    red, green, blue = cell.transpose(2, 0, 1)
    yy, xx = np.where(
        (red > 140) & (green > 130) & (red > 2.3 * blue) & (green > 2.3 * blue)
    )
    if not len(xx):
        return 0
    dx, dy = xx.mean() - (size - 1) / 2, yy.mean() - (size - 1) / 2
    north = ("E" if dx > 0 else "W") if abs(dx) > abs(dy) else ("S" if dy > 0 else "N")
    return (-DIR.index(north)) % 4


def north_up(im, k):
    return (
        im
        if not k
        else im.transpose(
            (
                None,
                Image.Transpose.ROTATE_270,
                Image.Transpose.ROTATE_180,
                Image.Transpose.ROTATE_90,
            )[k]
        )
    )


def read_card(im, origin):
    a = np.asarray(im, dtype=float)
    red, green, blue = a.transpose(2, 0, 1)
    rim = (green > 125) & (green > 1.23 * red) & (red > 3 * blue) & (red > 70)
    candidates = np.argwhere(rim.reshape(7, 12, 7, 12).sum(axis=(1, 3)) >= 5)

    def swatch(rgb, names):
        best = None
        for name in names:
            ref = np.array(tuple(bytes.fromhex(INK[name])))
            scale = np.clip(rgb @ ref / (ref @ ref), 0.65, 1.24)
            dist = float(np.linalg.norm(rgb - scale * ref))
            if best is None or dist < best[0]:
                best = dist, name
        return best[1] if best[0] < 31 else None

    for j, i in candidates:
        cell = a[j * 12 : (j + 1) * 12, i * 12 : (i + 1) * 12]
        values = [
            swatch(cell[y : y + 2, x : x + 2].mean(axis=(0, 1)), names)
            for x, y, names in (
                (3, 3, ROOMS),
                (7, 3, ROOMS),
                (3, 7, ("jade", "violet")),
                (7, 7, ("red", "blue")),
            )
        ]
        if all(values) and values[0] != values[1]:
            return ({values[0], values[1]}, values[2], values[3]), (
                origin[0] + int(i) - 3,
                origin[1] + int(j) - 3,
            )
    return None


@lru_cache(maxsize=1024)
def paint(rgb):
    if max(rgb) < 35:
        return None

    def cost(k):
        ref = PALETTE[k]
        scale = sum(a * b for a, b in zip(rgb, ref)) / sum(v * v for v in ref)
        scale = max(0.76, min(1.26, scale))
        return (
            sum((a - scale * b) ** 2 for a, b in zip(rgb, ref)) + 20 * (scale - 1) ** 2
        )

    hit = min(PALETTE, key=cost)
    return hit if cost(hit) <= 13**2 else "furniture"


def add(p, d, k=1):
    return p[0] + k * V[d][0], p[1] + k * V[d][1]


def distance(p, q):
    return abs(p[0] - q[0]) + abs(p[1] - q[1])


def read_view(encoded, origin, r):
    im = as_image(encoded)
    px = im.load()
    s = im.width // (2 * r + 1)
    for j in range(2 * r + 1):
        for i in range(2 * r + 1):
            mark = paint(px[i * s + s // 2, j * s + s // 2])
            if mark is not None:
                base = paint(px[i * s + s // 4, j * s + s // 4])
                # A multicoloured table/cushion can share one swatch with a marker.
                # The footing and surrounding texture disambiguate its footprint.
                corners = [
                    paint(px[i * s + x, j * s + y])
                    for x in (1, s - 2)
                    for y in (1, s - 2)
                ]
                if mark in ROOMS | {"hall"} and corners.count("furniture") >= 2:
                    mark = "furniture"
                if mark == "dust":
                    ground_votes = [
                        v for v in corners + [base] if v in ROOMS | {"hall"}
                    ]
                    if ground_votes:
                        base = Counter(ground_votes).most_common(1)[0][0]
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
        self.mount = (0, 0)
        self.last_wire = ["stay", "stay", 0, False]
        self.order = ({"oak", "sage"}, "jade", "red") if mode == "solo" else None
        self.card_pos = None
        self.hall_direction = None
        self.previous = ["stay", "stay", 0, True]
        self.cells = {}
        self.air_cells = {}
        self.rooms = {}
        self.pads = {}
        self.dirt = {}
        self.cleaned = set()
        self.box = self.initial_box = None
        self.staged = False
        self.route = ""
        self.last_t = -1
        self.scout_visits = {}
        self.ground_observed = set()
        self.confirmed_free = {(0, 0)}
        self.bumped = set()
        self.front_visits = Counter()
        self.chosen_front = None

    def absorb(self, img, p, r, from_air):
        for q, mark, base in read_view(img, p, r):
            if q == self.card_pos:
                self.cells[q] = "furniture"
                continue
            surface = mark if mark in {"wall", "furniture"} else "floor"
            if from_air:
                self.air_cells[q] = surface
            if from_air and q in self.ground_observed:
                continue
            if not from_air:
                self.ground_observed.add(q)
            self.cells[q] = "floor" if q in self.confirmed_free else surface
            if base in ROOMS | {"hall"}:
                self.rooms[q] = base
            if mark == "dust":
                self.dirt[q] = self.rooms.get(q, base)
            if mark == "box":
                self.box = q
                if self.initial_box is None:
                    self.initial_box = q
            if mark in ("red", "blue", "jade", "violet"):
                self.pads[mark] = q

    def scout(self, obs, requested, target):
        if not obs["flight"] or self.mode == "solo":
            return "stay", 0
        cells = self.air_cells if self.mode == "fixed" else self.cells
        if self.order is None and self.hall_direction:
            d = self.hall_direction
            if cells.get(add(self.a, d)) != "wall":
                return d, 2
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
                hall = self.rooms.get(p) == "hall" and self.box is None
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
            return self.last_wire
        self.last_t = t
        oldg = self.g
        self.g = add(self.g, self.previous[0], obs["moved"][0])
        self.a = add(self.a, self.previous[1], obs["moved"][1])
        self.confirmed_free.add(self.g)
        if self.chosen_front == self.g:
            self.front_visits[self.g] += 1
        self.chosen_front = None
        if t and self.g == oldg and self.previous[0] != "stay":
            attempted = add(oldg, self.previous[0])
            if attempted != self.box:
                self.bumped.add(attempted)
        if self.g != oldg and self.g == self.box:
            self.box = add(self.g, self.previous[0])
        if self.previous[3] and (self.g in self.dirt or self.g == self.initial_box):
            self.cleaned.add(self.g)
        ground = as_image(obs["ground"])
        air = as_image(obs["air"]) if obs["air"] and self.mode != "solo" else None
        if t == 0:
            self.mount = infer_mount(ground, 1), infer_mount(air, 3) if air else 0
        if air:
            air = north_up(air, self.mount[1])
            if self.order is None:
                reading = read_card(air, self.a)
                if reading:
                    self.order, self.card_pos = reading
            self.absorb(air, self.a, 3, True)
        self.absorb(north_up(ground, self.mount[0]), self.g, 1, False)
        if t == 0 and air:

            def axis(d):
                return 12 * (self.cells.get(add((0, 0), d, -1)) == "wall") + sum(
                    (2 if self.rooms.get(add((0, 0), d, n)) == "hall" else 0)
                    - 5 * (self.cells.get(add((0, 0), d, n)) == "wall")
                    for n in (1, 2, 3)
                )

            self.hall_direction = max(DIR, key=axis)
        self.scout_visits[self.a] = self.scout_visits.get(self.a, 0) + 1
        req, first, final = self.order or (set(), None, None)
        if (
            not self.staged
            and self.pads.get(first) is not None
            and self.box == self.pads[first]
        ):
            self.staged = True
            self.route = ""
        target = final if self.staged else first
        pad = self.pads.get(target)
        free = (
            self.confirmed_free | {p for p, v in self.cells.items() if v == "floor"}
        ) - self.bumped
        reach = paths(self.g, free - {self.box})
        todo = [
            p
            for p, r in self.dirt.items()
            if r in req | {"hall"} and p not in self.cleaned and p in reach
        ]
        finished = bool(req) and all(
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
        elif finished and self.staged and pad is not None and self.box == pad:
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
                    return len(reach[p]) - 5 * incomplete + min(
                        70, 9 * self.front_visits[p]
                    ), p

                self.chosen_front = min(front, key=cost)
                g = reach[self.chosen_front][0]
            else:
                g = reach.get((0, 0), "")[:1] or "stay"
        a, stride = self.scout(obs, req, target)
        dest = add(self.g, g)
        brush = (
            (dest in self.dirt and self.dirt[dest] in req | {"hall"})
            or dest == self.initial_box
        ) and dest not in self.cleaned
        self.previous = [g, a, stride, brush]

        def local(d, k):
            return "stay" if d == "stay" else DIR[(DIR.index(d) - k) % 4]

        self.last_wire = [
            local(g, self.mount[0]),
            local(a, self.mount[1]),
            stride,
            brush,
        ]
        return self.last_wire


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

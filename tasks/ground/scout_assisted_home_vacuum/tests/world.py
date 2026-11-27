"""Private household state. One simultaneous tick, one grid-based motion primitive per robot."""

from collections import deque
import random

VEC = {"stay": (0, 0), "N": (0, -1), "E": (1, 0), "S": (0, 1), "W": (-1, 0)}
ROOMS = ("oak", "sage", "slate", "rose")
SUITES = (
    ("bedDouble", "cabinetBedDrawer", "lampRoundFloor", "plantSmall1"),
    ("loungeSofaLong", "tableCoffee", "loungeChair", "bookcaseOpen"),
    ("desk", "bookcaseOpen", "chairDesk", "plantSmall1"),
    ("kitchenSink", "kitchenStove", "kitchenFridge", "kitchenCabinetDrawer"),
)


def plus(p, d):
    return p[0] + d[0], p[1] + d[1]


def connected(cells):
    first = next(iter(cells))
    seen, todo = {first}, deque([first])
    while todo:
        p = todo.popleft()
        for d in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            q = plus(p, d)
            if q in cells and q not in seen:
                seen.add(q)
                todo.append(q)
    return len(seen) == len(cells)


def reversible_carton_cells(free, start, dock):
    """Flood only pushes that the vacuum could undo; enclosed corners cannot be stations."""
    walks = {}

    def reach(block, pos):
        seen = walks.setdefault(block, {})
        if pos not in seen:
            group, todo = {pos}, [pos]
            while todo:
                p = todo.pop()
                for v in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                    q = plus(p, v)
                    if q in free and q != block and q not in group:
                        group.add(q)
                        todo.append(q)
            marker = min(group)
            for p in group:
                seen[p] = marker
        return seen[pos]

    todo = deque([(start, dock, 0)])
    distances, seen = {}, set()
    while todo:
        b, g, n = todo.popleft()
        key = b, reach(b, g)
        if key in seen:
            continue
        seen.add(key)
        distances.setdefault(b, n)
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            behind, new, ahead = (
                (b[0] - dx, b[1] - dy),
                (b[0] + dx, b[1] + dy),
                (b[0] + 2 * dx, b[1] + 2 * dy),
            )
            if (
                behind in free
                and new in free
                and ahead in free
                and reach(b, behind) == key[1]
                and reach(new, ahead) == reach(new, b)
            ):
                todo.append((new, b, n + 1))
    return distances


class Home:
    def __init__(self, seed, ticks=200, flight=36, order_key=None, _attempt=0):
        rng = random.Random(seed)
        w, h = rng.choice((17, 19, 21)), rng.choice((15, 17))
        mid = h // 2 + rng.choice((-1, 0, 1))
        self.area, self.wall, self.furniture = {}, set(), {}
        for y in range(h):
            for x in range(w):
                if x in (0, w - 1) or y in (0, h - 1):
                    self.wall.add((x, y))
                else:
                    self.area[x, y] = "hall"
        names = list(ROOMS)
        rng.shuffle(names)
        rug_candidates, specs = [], []
        styles = list(SUITES)
        rng.shuffle(styles)
        self.pieces = []
        self.original = {}
        for side in (0, 1):
            partition = rng.randint(w // 2 - 2, w // 2 + 2)
            edge = mid - 2 if side == 0 else mid + 2
            rows = range(1, edge) if side == 0 else range(edge + 1, h - 1)
            self.wall.update((partition, y) for y in rows)
            inner_y = rng.choice(list(rows)[1:-1])
            self.wall.remove((partition, inner_y))
            self.wall.update((x, edge) for x in range(1, w - 1))
            # Gallery, end suite, or through suite: not every room opens onto the hall.
            openings = rng.choice(((0, 1), (0,), (1,)))
            for wing, xs in enumerate(
                (range(1, partition), range(partition + 1, w - 1))
            ):
                room = names.pop()
                door = (rng.choice(list(xs)[1:-1]), edge)
                entries = {(partition + (-1 if wing == 0 else 1), inner_y)}
                if wing in openings:
                    self.wall.remove(door)
                    entries.add((door[0], edge + (-1 if side == 0 else 1)))
                    rug_candidates.append(door)
                cells = [(x, y) for x in xs for y in rows]
                self.area.update(dict.fromkeys(cells, room))
                specs.append((room, cells, entries, styles.pop()))
        for room, cells, entries, suite in specs:
            interior, occupied = set(cells), set()
            # A utility core cuts a notch out of some rooms; its location changes which
            # sight lines and paths are available. Do not pinch either door landing.
            if len(cells) >= 24 and rng.random() < 0.8:
                xs, ys = zip(*cells)
                x = rng.choice((min(xs), max(xs)))
                y = rng.choice((min(ys), max(ys)))
                dx, dy = (1 if x == min(xs) else -1), (1 if y == min(ys) else -1)
                notch = {(x, y), (x + dx, y), (x, y + dy)}
                if not notch & entries and connected(interior - notch):
                    interior -= notch
                    self.wall.update(notch)
            anchor = None
            for k, model in enumerate(suite):
                # Anchor furniture occupies up to four actual cells. Reserve door landings
                # and check connectivity before accepting each piece, including side tables.
                sizes = (
                    [(2, 2), (2, 1)]
                    if model == "bedDouble"
                    else [(2, 1), (1, 2)]
                    if k == 0
                    else []
                )
                sizes = rng.sample(sizes, len(sizes)) + [(1, 1)]
                for sx, sy in sizes:
                    candidates = rng.sample(sorted(interior), len(interior))
                    if k == 0:
                        candidates.sort(
                            key=lambda p: sum(
                                plus(p, d) in interior for d in VEC.values()
                            )
                        )
                    elif k == 1 and anchor is not None:
                        candidates.sort(
                            key=lambda p: abs(p[0] - anchor[0]) + abs(p[1] - anchor[1])
                        )
                    for x, y in candidates:
                        foot = {(x + i, y + j) for i in range(sx) for j in range(sy)}
                        if not foot <= interior or foot & (entries | occupied):
                            continue
                        free = interior - (foot | occupied)
                        if len(free) < 8 or not connected(free):
                            continue
                        occupied.update(foot)
                        self.furniture.update(dict.fromkeys(foot, model))
                        self.pieces.append(
                            (
                                x + (sx - 1) / 2,
                                y + (sy - 1) / 2,
                                sx,
                                sy,
                                model,
                                rng.choice((0, 180)),
                            )
                        )
                        if k == 0:
                            anchor = (x, y)
                        break
                    else:
                        continue
                    break
            floor = sorted(interior - occupied)
            self.original[room] = set(rng.sample(floor, 2))
            rug_candidates.append(rng.choice(sorted(set(floor) - self.original[room])))
        pads = [(2, mid), (w - 3, mid)]
        rng.shuffle(pads)
        self.dock = (rng.choice((1, w - 2)), mid)
        self.card = (w - 2 if self.dock[0] == 1 else 1, mid + rng.choice((-1, 1)))
        self.furniture[self.card] = "dispatch cabinet"
        center_bays = [(x, mid) for x in range(3, w - 3)]
        self.box = rng.choice(center_bays)
        free = set(self.area) - self.wall - set(self.furniture)
        reversible = reversible_carton_cells(free, self.box, self.dock)
        already_dirty = set.union(*self.original.values())
        candidates = [
            p
            for p, n in reversible.items()
            if self.area[p] in ROOMS and p not in already_dirty and n >= 4
        ]
        if len({self.area[p] for p in candidates}) < 2:
            # Keep the challenge consistent; do not replace an unreachable indoor goal
            # with a trivial hall target or include such homes in the denominator.
            if _attempt >= 63:
                raise RuntimeError("could not generate a reversible indoor route")
            self.__init__(rng.getrandbits(128), ticks, flight, order_key, _attempt + 1)
            return
        candidates.sort(key=lambda p: (min(7, reversible[p]), abs(p[1] - mid), p))
        first = rng.choice(candidates[-min(8, len(candidates)) :])
        others = [p for p in candidates if self.area[p] != self.area[first]]
        second = rng.choice(others[-min(8, len(others)) :])
        self.mats = {"jade": first, "violet": second, "red": pads[0], "blue": pads[1]}
        self.staged = False
        self.original["hall"] = {self.box}
        self.dust = set.union(*self.original.values())
        # An entrance runner must be crossed to enter several rooms. A protected rug
        # is traversable with suction off, not a wall or a camera-only appearance.
        self.rugs = set(rug_candidates)
        self.rugs.add((self.dock[0] + (2 if self.dock[0] == 1 else -2), mid))
        self.rugs -= self.dust | set(self.mats.values()) | {self.dock}
        self.scuffs = set()
        turns = rng.randrange(4)
        self.mount = (rng.randrange(4), rng.randrange(4))
        order = random.Random(rng.getrandbits(128) if order_key is None else order_key)
        self.request = tuple(order.sample(ROOMS, 2))
        self.stage = order.choice(("jade", "violet"))
        self.target = order.choice(("red", "blue"))
        self.disturbed = set()

        def rotate(p):
            x, y = p
            return ((x, y), (h - 1 - y, x), (w - 1 - x, h - 1 - y), (y, w - 1 - x))[
                turns
            ]

        self.width, self.height = (h, w) if turns % 2 else (w, h)
        self.area = {rotate(p): r for p, r in self.area.items()}
        self.wall = set(map(rotate, self.wall))
        self.furniture = {rotate(p): v for p, v in self.furniture.items()}
        self.pieces = [
            (*rotate((x, y)), sx, sy, m, a - 90 * turns)
            for x, y, sx, sy, m, a in self.pieces
        ]
        self.original = {r: set(map(rotate, ps)) for r, ps in self.original.items()}
        self.rugs, self.dust = set(map(rotate, self.rugs)), set(map(rotate, self.dust))
        self.mats = {k: rotate(p) for k, p in self.mats.items()}
        self.dock, self.box = rotate(self.dock), rotate(self.box)
        self.card = rotate(self.card)
        self.ground = self.air = self.dock
        self.t, self.budget, self.fuel = 0, ticks, flight
        self.motion = [0, 0]
        self.distance = 0
        self.prompt = (
            "Read today's four swatches from the top of the dispatch cabinet at the "
            "far end of the entrance hall. Clean the two rooms on the card and the "
            "hall spill. Leave treatment powder in the other rooms untouched. Stage "
            "the carton on the selected room mat, then park it on the selected hall "
            "mat. Protect the runners and return the vacuum to its dock."
        )

    def blocked(self, p):
        return p not in self.area or p in self.wall

    def advance(self, action):
        if self.t >= self.budget:
            raise ValueError("episode ended")
        if not isinstance(action, list) or len(action) != 4:
            raise ValueError("expected four actions")
        g, a, stride, brush = action
        if (
            not isinstance(g, str)
            or g not in VEC
            or not isinstance(a, str)
            or a not in VEC
        ):
            raise ValueError("unknown direction")
        if (
            type(stride) is not int
            or stride not in (0, 1, 2)
            or type(brush) is not bool
        ):
            raise ValueError("bad stride or suction switch")
        self.motion = [0, 0]
        directions = ("N", "E", "S", "W")
        if g != "stay":
            g = directions[(directions.index(g) + self.mount[0]) % 4]
        if a != "stay":
            a = directions[(directions.index(a) + self.mount[1]) % 4]
        if g != "stay":
            dest = plus(self.ground, VEC[g])
            if not self.blocked(dest) and dest not in self.furniture:
                beyond = plus(dest, VEC[g])
                if dest != self.box or (
                    not self.blocked(beyond) and beyond not in self.furniture
                ):
                    if dest == self.box:
                        self.box = beyond
                        if self.box == self.mats[self.stage]:
                            self.staged = True
                    self.ground = dest
                    self.distance += 1
                    self.motion[0] = 1
        if brush:
            if self.ground in self.dust and self.area.get(self.ground) in set(
                ROOMS
            ) - set(self.request):
                self.disturbed.add(self.ground)
            self.dust.discard(self.ground)
            if self.ground in self.rugs:
                self.scuffs.add(self.ground)
        if self.fuel:
            for _ in range(stride if a != "stay" else 0):
                dest = plus(self.air, VEC[a])
                if self.blocked(dest):
                    break
                self.air = dest
                self.motion[1] += 1
            self.fuel -= 1  # hovering consumes the same flight tick
        self.t += 1

    def result(self):
        clean = {
            r: len(self.original[r] - self.dust) / len(self.original[r])
            for r in (*self.request, "hall")
        }
        parked = self.staged and self.box == self.mats[self.target]
        docked = self.ground == self.dock
        preserved = 1 - len(self.disturbed) / 4
        success = (
            min(clean.values()) == 1
            and parked
            and docked
            and not self.scuffs
            and not self.disturbed
        )
        score = (
            0.45 * sum(clean.values()) / 3 * preserved
            + 0.125 * (int(self.staged) + int(parked))
            + 0.30 * success
        )
        return dict(
            clean=clean,
            parked=parked,
            staged=self.staged,
            treatment_preserved=preserved,
            docked=docked,
            success=success,
            score=score,
            ticks=self.t,
            ground_moves=self.distance,
            scuffed_runners=len(self.scuffs),
        )

    def visible(self, origin, p, airborne):
        # Intermediate walls occlude both cameras; ground-level furniture also occludes.
        # Using a supercover at diagonal corners prevents looking through closed corners.
        dx, dy = p[0] - origin[0], p[1] - origin[1]
        n = max(abs(dx), abs(dy))

        def opaque(q):
            return self.blocked(q) or (not airborne and q in self.furniture)

        prev = origin
        for i in range(1, n + 1):
            q = (origin[0] + round(dx * i / n), origin[1] + round(dy * i / n))
            if (
                q[0] != prev[0]
                and q[1] != prev[1]
                and opaque((prev[0], q[1]))
                and opaque((q[0], prev[1]))
            ):
                return False
            if q != p and opaque(q):
                return False
            prev = q
        return True

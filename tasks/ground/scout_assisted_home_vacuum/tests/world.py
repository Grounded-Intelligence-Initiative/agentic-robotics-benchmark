"""Private household state. One simultaneous tick, one grid-based motion primitive per robot."""

import random

VEC = {"stay": (0, 0), "N": (0, -1), "E": (1, 0), "S": (0, 1), "W": (-1, 0)}
ROOMS = ("oak", "sage", "slate", "rose")


def plus(p, d):
    return p[0] + d[0], p[1] + d[1]


class Home:
    def __init__(self, seed, ticks=150, flight=36):
        rng = random.Random(seed)
        self.width = w = rng.choice((15, 17, 19))
        self.height = h = rng.choice((13, 15))
        mid = h // 2
        self.area, self.wall, self.furniture = {}, set(), {}
        for y in range(h):
            for x in range(w):
                if x in (0, w - 1) or y in (0, h - 1):
                    self.wall.add((x, y))
                else:
                    self.area[x, y] = "hall"
        names = list(ROOMS)
        rng.shuffle(names)
        doors = []
        self.original = {}
        for side in (0, 1):
            partition = rng.randint(w // 2 - 2, w // 2 + 2)
            edge = mid - 2 if side == 0 else mid + 2
            rows = range(1, edge) if side == 0 else range(edge + 1, h - 1)
            self.wall.update((partition, y) for y in rows)
            self.wall.remove((partition, rng.choice(list(rows)[1:-1])))
            self.wall.update((x, edge) for x in range(1, w - 1))
            for xs in (range(1, partition), range(partition + 1, w - 1)):
                room = names.pop()
                door = (rng.choice(list(xs)[1:-1]), edge)
                self.wall.remove(door)
                doors.append((door[0], edge + (1 if side == 0 else -1)))
                cells = [(x, y) for x in xs for y in rows]
                self.area.update(dict.fromkeys(cells, room))
                far_y = rows[0] if side == 0 else rows[-1]
                seat = rng.choice([(x, far_y) for x in xs])
                self.furniture[seat] = rng.choice(("chair", "table", "sofa"))
                self.original[room] = set(
                    rng.sample([p for p in cells if p != seat], 2)
                )
        self.request = tuple(rng.sample(ROOMS, 2))
        self.target = rng.choice(("red", "blue"))
        pads = [(2, mid), (w - 3, mid)]
        rng.shuffle(pads)
        self.mats = dict(zip(("red", "blue"), pads))
        self.box = rng.choice(doors)
        self.original["hall"] = {self.box}
        self.dust = set.union(*self.original.values())
        self.dock = (rng.choice((1, w - 2)), mid)
        self.ground = self.air = self.dock
        self.t, self.budget, self.fuel = 0, ticks, flight
        self.motion = [0, 0]
        self.distance = 0
        self.prompt = (
            f"Vacuum the visible dust in the {self.request[0]} and {self.request[1]} "
            f"rooms and the entrance hall. Park the carton on the {self.target} mat. Return to the dock."
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
        if g != "stay":
            dest = plus(self.ground, VEC[g])
            if not self.blocked(dest) and dest not in self.furniture:
                beyond = plus(dest, VEC[g])
                if dest != self.box or (
                    not self.blocked(beyond) and beyond not in self.furniture
                ):
                    if dest == self.box:
                        self.box = beyond
                    self.ground = dest
                    self.distance += 1
                    self.motion[0] = 1
        if brush:
            self.dust.discard(self.ground)
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
        parked, docked = self.box == self.mats[self.target], self.ground == self.dock
        success = min(clean.values()) == 1 and parked and docked
        score = 0.45 * sum(clean.values()) / 3 + 0.25 * parked + 0.30 * success
        return dict(
            clean=clean,
            parked=parked,
            docked=docked,
            success=success,
            score=score,
            ticks=self.t,
            ground_moves=self.distance,
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

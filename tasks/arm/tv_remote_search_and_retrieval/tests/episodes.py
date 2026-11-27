"""The graded episodes' rule: where the remote and the other small things are. Hidden.

This file is the grader's alone. The simulator the agent has (remote_env.py) puts the remote
wherever its caller says and holds no rule for it; this is the rule. An episode is a seed and one
of seven hiding spots:

* ``left_cushion`` / ``right_cushion``: on the seat deck, under one of the two loose seat pads
  (left is +y), under the pad's front half. A pad is soft: it lies flat on the deck over the
  remote (in the model a pad and the remote do not touch each other).
* ``under_sofa``: on the floor under the sofa, 15 to 19 cm behind its front face, deeper than the
  hand reaches. The wooden clothes hanger on the chest's outer side reaches it.
* ``under_chest``: on the floor under the chest, between its legs, pointing in, its near end 10
  to 11.5 cm behind the chest's front: out of sight from above. The carcass is 10.8 cm above the
  floor: the hand fits in lying flat, not from above.
* ``under_hat``: in the drawer the sun hat lies in (any of the three), under the hat: one end
  under its crown, the other under the high side of its brim.
  Opening the drawer shows a hat; the remote shows once the hat is out of the way.
* ``in_box``: on the floor of the storage basket on the chest, under the book that lies in it.
  The book is too wide for the hand to take from above: the basket can be taken by its rim and
  tipped out.

* ``in_drawer``: on the floor of a closed drawer the hat is not in, lying free (the phone or the glasses case may
  lie in the same drawer, at least CLUTTER_GAP away).

The other remote (the air conditioner's: the TV remote's size, light grey; `Scene.decoy`) lies in every episode in
a place the TV remote is not in: under a seat pad, in a drawer the hat is not in, or under the hat. Taking it out
scores nothing; a search that stops at the first remote-sized thing ends there.

The book is always in the box. The phone lies under a seat pad, in a drawer the hat is not in,
under the hat or in the box under the book (those two only when the remote is not there); the
glasses case in a drawer the hat is not in. Things in one drawer or
under one pad are at least CLUTTER_GAP apart. At most a few pixels of the remote show in either camera when an episode starts (the selection
of the graded episodes checks it).

`make_episode(E, seed, spot)` returns the simulator's `Scene`; `E` is the simulator module.
"""

import math

HIDING_SPOTS = ("left_cushion", "right_cushion", "under_sofa", "under_chest", "under_hat", "in_box", "in_drawer")
DRAWER_REACH = (0.11, 0.21)             # a remote loose in a drawer: its centre this far behind the front of the floor
DECOY_SPOTS = ("left_cushion", "right_cushion", "under_sofa", "under_chest", "under_hat", "in_box", "drawer")
CLUTTER_GAP = 0.04
PHONE_SPOTS = ("left_cushion", "right_cushion", "drawer", "drawer", "under_hat", "in_box")
CASE_SPOTS = ("drawer",)
BOX_BOOK_X = -0.015         # in the box frame: the middle of the book
HAT_REMOTE_Y = -0.026       # in the hat's frame: where the remote's middle is under the hat
BOOK_SHIFT = 0.045          # the book's middle in front of the middle of what lies under it


def _corners(xy, yaw, half):
    c, s = math.cos(yaw), math.sin(yaw)
    return [(xy[0] + c * a - s * b, xy[1] + s * a + c * b)
            for a, b in ((half[0], half[1]), (half[0], -half[1]), (-half[0], -half[1]), (-half[0], half[1]))]


def _apart(one, other, gap):
    """True when two rectangles (xy, yaw, half sizes) are at least `gap` apart (separating axes)."""
    ca, cb = _corners(*one), _corners(*other)
    for yaw in (one[1], one[1] + 0.5 * math.pi, other[1], other[1] + 0.5 * math.pi):
        ax, ay = math.cos(yaw), math.sin(yaw)
        pa = [x * ax + y * ay for x, y in ca]
        pb = [x * ax + y * ay for x, y in cb]
        if min(pa) - max(pb) >= gap or min(pb) - max(pa) >= gap:
            return True
    return False


def _spot_area(E, spot):
    """((x0, x1), (y0, y1)) a thing in `spot` has to lie within, in the nominal layout."""
    if spot.endswith("_cushion"):
        centre = E.PAD_Y["left" if spot == "left_cushion" else "right"]
        return (E.SOFA_X0 + 0.02, E.SOFA_X0 + 0.40), (centre - 0.21, centre + 0.21)
    x0 = E.CAB_X0 + E.DRAWER_FRONT_X[1]
    return ((x0 + 0.008, x0 + E.DRAWER_INNER[0] - 0.008),
            (E.CAB_CY - 0.5 * E.DRAWER_INNER[1] + 0.008, E.CAB_CY + 0.5 * E.DRAWER_INNER[1] - 0.008))


def _lay(E, draw, spot, half, others):
    """A pose in `spot` (a seat pad's recess or a drawer) for a thing of half sizes `half`,
    clear of `others` already there (None when 400 draws find none)."""
    (x0, x1), (y0, y1) = _spot_area(E, spot)
    for _ in range(400):
        xy = (draw.uniform(x0, x1), draw.uniform(y0, y1))
        yaw = draw.uniform(-math.pi, math.pi)
        inside = all(x0 <= x <= x1 and y0 <= y <= y1 for x, y in _corners(xy, yaw, half))
        if inside and all(_apart((xy, yaw, half), other, CLUTTER_GAP) for other in others):
            return xy, yaw
    return None


def _under_hat(scene, x, y, yaw):
    """Nominal-layout (xy, yaw) of a thing at (x, y), `yaw` of the hat's frame."""
    c, s = math.cos(scene.hat_yaw), math.sin(scene.hat_yaw)
    return (scene.hat_xy[0] + c * x - s * y, scene.hat_xy[1] + s * x + c * y), scene.hat_yaw + yaw


def _in_box(scene, local, yaw):
    """Nominal-layout (xy, yaw) of a thing at `local`, `yaw` of the box frame."""
    c, s = math.cos(scene.box_yaw), math.sin(scene.box_yaw)
    return ((scene.box_xy[0] + c * local[0] - s * local[1], scene.box_xy[1] + s * local[0] + c * local[1]),
            scene.box_yaw + yaw)


def _world(E, scene, piece, xy, yaw, z):
    """(x, y, z, yaw) in the scene of a thing at `xy`, `yaw` of the nominal layout of `piece`."""
    on_sofa = piece == "sofa"
    (x, y), turned = E.place(E.SOFA_PIVOT if on_sofa else E.CAB_PIVOT,
                             scene.sofa_pose if on_sofa else scene.cabinet_pose, xy, yaw)
    return (x, y, z + 0.0005, turned)


def make_episode(E, seed, hiding_spot):
    """The episode of `seed` with the remote in `hiding_spot`, as a `Scene` of the simulator `E`."""
    if hiding_spot not in HIDING_SPOTS:
        raise ValueError("unknown hiding spot: {!r}".format(hiding_spot))
    base = E.make_scene(seed, remote=(0.0, 0.0, 0.0, 0.0))      # the layout, the hat and the box of this seed
    pose = E._Draw(seed, "pose:" + hiding_spot)
    yaw = pose.uniform(-math.pi, math.pi)
    box_floor = E.CAB_H + E.BOX_FLOOR_Z
    hat_floor = E.DRAWER_FLOOR_Z[base.hat_drawer]
    half_z = E.REMOTE_HALF[2]
    piece, z = "cabinet", half_z
    if hiding_spot in ("left_cushion", "right_cushion"):
        centre = E.PAD_Y["left" if hiding_spot == "left_cushion" else "right"]
        # The front part of a pad's recess: what stays clear when the pad is stood up against
        # the backrest.
        xy = (pose.uniform(E.SOFA_X0 + 0.11, E.SOFA_X0 + 0.24), centre + pose.uniform(-0.12, 0.12))
        piece, z = "sofa", E.DECK_Z + half_z
    elif hiding_spot == "under_hat":
        # Along the hat's y axis, one end under the crown and the other under the high side of
        # the brim: the only way it fits under the hat.
        xy, yaw = _under_hat(base, pose.uniform(-0.003, 0.003), HAT_REMOTE_Y + pose.uniform(-0.002, 0.002),
                             0.5 * math.pi + pose.uniform(-0.04, 0.04) + (math.pi if pose.uniform() < 0.5 else 0.0))
        z = hat_floor + half_z
    elif hiding_spot == "in_box":
        # Along the box, behind its middle.
        xy, yaw = _in_box(base, (BOX_BOOK_X + BOOK_SHIFT + pose.uniform(-0.005, 0.005), pose.uniform(-0.02, 0.02)),
                          0.5 * math.pi + pose.uniform(-0.05, 0.05) + (math.pi if pose.uniform() < 0.5 else 0.0))
        z = box_floor + half_z
    elif hiding_spot == "under_chest":
        # Between the chest's legs, pointing in, its near end far enough behind the chest's
        # front that nothing of it shows from above.
        xy = (E.CAB_X0 + pose.uniform(0.190, 0.200), E.CAB_CY + pose.uniform(-0.11, 0.11))
        yaw = pose.uniform(-0.2, 0.2) + (math.pi if pose.uniform() < 0.5 else 0.0)
    elif hiding_spot == "under_sofa":
        xy = (E.SOFA_X0 + pose.uniform(0.15, 0.19), pose.uniform(E.SOFA_Y[0] + 0.34, E.SOFA_Y[1] - 0.36))
        piece = "sofa"
    drawers = [n for n in E.DRAWERS if n != base.hat_drawer]
    if hiding_spot == "in_drawer":
        # Loose in one of the two drawers the hat is not in.
        own = pose.choice(drawers) + "_drawer"
        # ... in the middle band of its depth, where a hand can come down on it at any angle: at the back of the
        # pulled-out drawer the hand meets the front of the drawer above, at the front the drawer's own front panel.
        front = _spot_area(E, own)[0][0]
        while True:
            xy, yaw = _lay(E, pose, own, E.REMOTE_HALF[:2], [])
            if DRAWER_REACH[0] <= xy[0] - front <= DRAWER_REACH[1]:
                break
        z = E.DRAWER_FLOOR_Z[own.split("_")[0]] + half_z
        things = {own: [(xy, yaw, E.REMOTE_HALF[:2])]}
    else:
        things = {hiding_spot: [(xy, yaw, E.REMOTE_HALF[:2])]}
    remote = _world(E, base, piece, xy, yaw, z)
    swing = pose.uniform(-0.06, 0.06)
    # The other remote: in a place the TV remote is not in. (Its draw does not change the TV remote's.)
    other = E._Draw(seed, "decoy")
    spot = other.choice([s for s in DECOY_SPOTS if s != hiding_spot])
    decoy_spot = spot
    under_hat_taken = hiding_spot == "under_hat" or spot == "under_hat"
    if spot == "under_hat":
        lay = _under_hat(base, other.uniform(-0.003, 0.003), HAT_REMOTE_Y + other.uniform(-0.002, 0.002),
                         0.5 * math.pi + other.uniform(-0.04, 0.04) + (math.pi if other.uniform() < 0.5 else 0.0))
        decoy = _world(E, base, "cabinet", lay[0], lay[1], hat_floor + half_z)
    elif spot == "in_box":
        lay = _in_box(base, (BOX_BOOK_X + BOOK_SHIFT + other.uniform(-0.005, 0.005), other.uniform(-0.02, 0.02)),
                      0.5 * math.pi + other.uniform(-0.05, 0.05) + (math.pi if other.uniform() < 0.5 else 0.0))
        decoy = _world(E, base, "cabinet", lay[0], lay[1], box_floor + half_z)
    elif spot == "under_chest":
        lay = ((E.CAB_X0 + other.uniform(0.190, 0.200), E.CAB_CY + other.uniform(-0.11, 0.11)),
               other.uniform(-0.2, 0.2) + (math.pi if other.uniform() < 0.5 else 0.0))
        decoy = _world(E, base, "cabinet", lay[0], lay[1], half_z)
    elif spot == "under_sofa":
        lay = ((E.SOFA_X0 + other.uniform(0.15, 0.19), other.uniform(E.SOFA_Y[0] + 0.34, E.SOFA_Y[1] - 0.36)),
               other.uniform(-math.pi, math.pi))
        decoy = _world(E, base, "sofa", lay[0], lay[1], half_z)
    else:
        if spot == "drawer":
            spot = other.choice(drawers) + "_drawer"
        if spot.endswith("_cushion"):
            centre = E.PAD_Y["left" if spot == "left_cushion" else "right"]
            lay = ((other.uniform(E.SOFA_X0 + 0.11, E.SOFA_X0 + 0.24), centre + other.uniform(-0.12, 0.12)), other.uniform(-math.pi, math.pi))
            surface, on = E.DECK_Z, "sofa"
        else:
            lay = _lay(E, other, spot, E.REMOTE_HALF[:2], things.get(spot, []))
            if lay is None:
                raise RuntimeError("no room for the other remote")
            surface, on = E.DRAWER_FLOOR_Z[spot.split("_")[0]], "cabinet"
        things.setdefault(spot, []).append((lay[0], lay[1], E.REMOTE_HALF[:2]))
        decoy = _world(E, base, on, lay[0], lay[1], surface + half_z)
    # The other small things. (The remote's own draw does not depend on them.)
    clutter = E._Draw(seed, "clutter")
    placed = {}
    under_book = half_z if "in_box" in (hiding_spot, decoy_spot) else 0.0     # half the height of what the book lies on
    book_at = (xy, yaw) if hiding_spot == "in_box" else (lay if decoy_spot == "in_box" else None)
    for name, spots, half in (("phone", PHONE_SPOTS, E.PHONE_HALF), ("case", CASE_SPOTS, E.CASE_HALF)):
        spots = [s for s in spots if s != hiding_spot and not (s == "under_hat" and under_hat_taken)
                 and not (s == "in_box" and decoy_spot == "in_box")]
        spot = clutter.choice(spots)
        if spot == "drawer":
            spot = clutter.choice(drawers) + "_drawer"
        if spot == "under_hat":
            lay = _under_hat(base, clutter.uniform(-0.003, 0.003), -0.012 + clutter.uniform(-0.004, 0.004),
                             0.5 * math.pi + clutter.uniform(-0.08, 0.08))
            placed[name] = _world(E, base, "cabinet", lay[0], lay[1], hat_floor + half[2])
        elif spot == "in_box" and name == "phone":
            lay = _in_box(base, (BOX_BOOK_X + BOOK_SHIFT + clutter.uniform(-0.005, 0.005), clutter.uniform(-0.02, 0.02)),
                          0.5 * math.pi + clutter.uniform(-0.08, 0.08))
            placed[name] = _world(E, base, "cabinet", lay[0], lay[1], box_floor + half[2])
            under_book, book_at = half[2], lay
        else:
            candidates = [spot] + ([d + "_drawer" for d in drawers if d + "_drawer" != spot] if spot.endswith("_drawer") else [])
            for spot in candidates:                     # the drawn place, or the next with room
                lay = _lay(E, clutter, spot, half[:2], things.get(spot, []))
                if lay is not None:
                    break
            else:
                raise RuntimeError("no room for the " + name)
            things.setdefault(spot, []).append((lay[0], lay[1], half[:2]))
            on_sofa = spot.endswith("_cushion")
            surface = E.DECK_Z if on_sofa else E.DRAWER_FLOOR_Z[spot.split("_")[0]]
            placed[name] = _world(E, base, "sofa" if on_sofa else "cabinet", lay[0], lay[1], surface + half[2])
    # The book: over the remote or the phone when one of them is in the box (which lies under
    # its back half, so that the book's front edge is down on the box's floor and nothing shows
    # from the front), else flat on the box's floor.
    if book_at is None:
        book_at = _in_box(base, (BOX_BOOK_X + clutter.uniform(-0.006, 0.006), clutter.uniform(-0.02, 0.02)), clutter.uniform(-0.04, 0.04))
    else:
        c, s = math.cos(base.box_yaw), math.sin(base.box_yaw)
        book_at = ((book_at[0][0] - c * BOOK_SHIFT, book_at[0][1] - s * BOOK_SHIFT), base.box_yaw + clutter.uniform(-0.03, 0.03))
    book = _world(E, base, "cabinet", book_at[0], book_at[1], box_floor + 2.0 * under_book + E.BOOK_HALF[2] + 0.001)
    return E.make_scene(seed, remote=remote, phone=placed["phone"], case=placed["case"], book=book, hanger_swing=swing, decoy=decoy)

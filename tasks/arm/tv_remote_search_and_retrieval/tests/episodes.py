"""The graded episodes' rule: where the remote, the phone and the glasses case are. Hidden.

This file is the grader's alone. The simulator the agent has (remote_env.py) puts the remote
wherever its caller says and holds no rule for it; this is the rule. An episode is a seed and one
of six hiding spots:

* ``left_cushion`` / ``right_cushion``: on the seat deck, under one of the two loose seat pads
  (left is +y), under the pad's front half. A pad is soft: it lies flat on the deck over the
  remote (in the model a pad and the remote do not touch each other).
* ``upper_drawer`` / ``lower_drawer``: on the floor of the top or the bottom one of the chest's
  three closed drawers, toward its front.
* ``under_sofa``: on the floor under the sofa, 15 to 19 cm behind its front face, deeper than the
  hand reaches. The wooden clothes hanger on the chest's outer side reaches it.
* ``under_hat``: on top of the chest, under the crown of the sun hat that lies there.

The phone lies under a seat pad, in one of the three drawers or (when the remote is not there)
under the hat; the glasses case in one of the three drawers. Either may share its place with the
remote: nothing lies on anything else, and things in one place are at least CLUTTER_GAP apart.
Nothing of the remote is visible in either camera when an episode starts.

`make_episode(E, seed, spot)` returns the simulator's `Scene`; `E` is the simulator module.
"""

import math

HIDING_SPOTS = ("left_cushion", "right_cushion", "upper_drawer", "lower_drawer", "under_sofa", "under_hat")
CLUTTER_GAP = 0.04
PHONE_SPOTS = ("left_cushion", "right_cushion", "upper_drawer", "mid_drawer", "lower_drawer", "under_hat")
CASE_SPOTS = ("upper_drawer", "mid_drawer", "lower_drawer")


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


def _spot_area(E, spot, hat_xy):
    """((x0, x1), (y0, y1)) a thing in `spot` has to lie within, in the nominal layout."""
    if spot.endswith("_cushion"):
        centre = E.PAD_Y["left" if spot == "left_cushion" else "right"]
        return (E.SOFA_X0 + 0.02, E.SOFA_X0 + 0.40), (centre - 0.21, centre + 0.21)
    if spot.endswith("_drawer"):
        x0 = E.CAB_X0 + E.DRAWER_FRONT_X[1]
        return ((x0 + 0.008, x0 + E.DRAWER_INNER[0] - 0.008),
                (E.CAB_CY - 0.5 * E.DRAWER_INNER[1] + 0.008, E.CAB_CY + 0.5 * E.DRAWER_INNER[1] - 0.008))
    return (hat_xy[0] - 0.09, hat_xy[0] + 0.09), (hat_xy[1] - 0.09, hat_xy[1] + 0.09)      # under the crown


def _lay(E, draw, spot, half, hat_xy, others):
    """A pose in `spot` for a thing of half sizes `half`, clear of `others` already there (None
    when 400 draws find none)."""
    (x0, x1), (y0, y1) = _spot_area(E, spot, hat_xy)
    for _ in range(400):
        if spot == "under_hat":
            xy = (hat_xy[0] + draw.uniform(-0.005, 0.005), hat_xy[1] + draw.uniform(-0.005, 0.005))
        else:
            xy = (draw.uniform(x0, x1), draw.uniform(y0, y1))
        yaw = draw.uniform(-math.pi, math.pi)
        inside = spot == "under_hat" or all(x0 <= x <= x1 and y0 <= y <= y1 for x, y in _corners(xy, yaw, half))
        if inside and all(_apart((xy, yaw, half), other, CLUTTER_GAP) for other in others):
            return xy, yaw
    return None


def _surface(E, spot):
    """The height of the surface a thing in `spot` lies on."""
    if spot.endswith("_cushion"):
        return E.DECK_Z
    if spot.endswith("_drawer"):
        return E.DRAWER_FLOOR_Z[spot.split("_")[0]]
    return E.CAB_H if spot == "under_hat" else 0.0


def _world(E, scene, spot, xy, yaw, half_z):
    """(x, y, z, yaw) in the scene of a thing at `xy`, `yaw` of the nominal layout in `spot`."""
    on_sofa = spot.endswith("_cushion") or spot == "under_sofa"
    (x, y), turned = E.place(E.SOFA_PIVOT if on_sofa else E.CAB_PIVOT,
                             scene.sofa_pose if on_sofa else scene.cabinet_pose, xy, yaw)
    return (x, y, _surface(E, spot) + half_z + 0.0005, turned)


def make_episode(E, seed, hiding_spot):
    """The episode of `seed` with the remote in `hiding_spot`, as a `Scene` of the simulator `E`."""
    if hiding_spot not in HIDING_SPOTS:
        raise ValueError("unknown hiding spot: {!r}".format(hiding_spot))
    base = E.make_scene(seed, remote=(0.0, 0.0, 0.0, 0.0))      # the layout and the hat of this seed
    hat_xy = base.hat_xy
    pose = E._Draw(seed, "pose:" + hiding_spot)
    yaw = pose.uniform(-math.pi, math.pi)
    if hiding_spot in ("left_cushion", "right_cushion"):
        centre = E.PAD_Y["left" if hiding_spot == "left_cushion" else "right"]
        # The front part of a pad's recess: what stays clear when the pad is stood up against
        # the backrest.
        xy = (pose.uniform(E.SOFA_X0 + 0.11, E.SOFA_X0 + 0.24), centre + pose.uniform(-0.12, 0.12))
    elif hiding_spot in ("upper_drawer", "lower_drawer"):
        # Toward the front of the drawer: where a hand reaches it once the drawer is out.
        xy = (E.CAB_X0 + E.DRAWER_FRONT_X[1] + pose.uniform(0.115, 0.18), E.CAB_CY + pose.uniform(-0.09, 0.09))
    elif hiding_spot == "under_hat":
        # Under the crown: anywhere it fits, in any direction.
        xy = (hat_xy[0] + pose.uniform(-0.005, 0.005), hat_xy[1] + pose.uniform(-0.005, 0.005))
    else:
        xy = (E.SOFA_X0 + pose.uniform(0.15, 0.19), pose.uniform(E.SOFA_Y[0] + 0.34, E.SOFA_Y[1] - 0.36))
    swing = pose.uniform(-0.06, 0.06)
    # The phone and the glasses case. (The remote's own draw does not depend on them.)
    clutter = E._Draw(seed, "clutter")
    things = {hiding_spot: [(xy, yaw, E.REMOTE_HALF[:2])]}
    placed = {}
    for name, spots, half in (("phone", PHONE_SPOTS, E.PHONE_HALF), ("case", CASE_SPOTS, E.CASE_HALF)):
        spots = [s for s in spots if not (s == "under_hat" and hiding_spot == "under_hat")]
        first = spots.index(clutter.choice(spots))
        for spot in spots[first:] + spots[:first]:      # the drawn place, or the next with room
            lay = _lay(E, clutter, spot, half[:2], hat_xy, things.get(spot, []))
            if lay is not None:
                break
        else:
            raise RuntimeError("no room for the " + name)
        things.setdefault(spot, []).append((lay[0], lay[1], half[:2]))
        placed[name] = _world(E, base, spot, lay[0], lay[1], half[2])
    return E.make_scene(seed, remote=_world(E, base, hiding_spot, xy, yaw, E.REMOTE_HALF[2]),
                        phone=placed["phone"], case=placed["case"], hanger_swing=swing)

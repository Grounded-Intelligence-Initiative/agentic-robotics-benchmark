"""Permanent warehouse passages on both sides of the turn station."""

from dataclasses import asdict, dataclass, replace
import hashlib
import math
import xml.etree.ElementTree as ET

import numpy as np

BRANCHES = ("left", "right")
VARIANTS = ("low-left", "low-right", "high-left", "high-right")
POST_RGBA = ".19 .46 .69 1"
DECK_RGBA = ".86 .61 .15 1"
SIDE_RGBA = ".10 .54 .64 1"
STAND_HEAD_M = 1.25


@dataclass(frozen=True)
class Rack:
    branch: str
    side: int
    height_family: str
    front_m: float
    length_m: float
    left_m: float
    width_m: float
    underside_m: float

    @property
    def back_m(self):
        return self.front_m + self.length_m

    @property
    def edges(self):
        return self.left_m - self.width_m / 2, self.left_m + self.width_m / 2

    def record(self):
        lo, hi = self.edges
        return dict(asdict(self), back_m=self.back_m, gap_min_m=lo, gap_max_m=hi)


@dataclass(frozen=True)
class RackPair:
    nominal_pivot_forward_m: float
    left: Rack
    right: Rack

    def branch(self, name):
        if name not in BRANCHES:
            raise ValueError("expected left or right outgoing aisle")
        return self.left if name == "left" else self.right

    def mirrored(self):
        def other(r, name):
            return replace(r, branch=name, side=-r.side, left_m=-r.left_m)

        return RackPair(
            self.nominal_pivot_forward_m, other(self.right, "left"), other(self.left, "right")
        )


def make_racks(seed, first_m, left_rack=None, right_rack=None):
    if type(seed) is not int or not 0 <= seed < 2**64:
        raise ValueError("rack seed must be an unsigned 64-bit integer")

    def make(branch, override):
        def draw(field, lo, hi):
            value = hashlib.blake2b(
                (branch + "-" + field).encode(),
                key=seed.to_bytes(8, "little"),
                person=b"g1rack.v1",
                digest_size=8,
            ).digest()
            return lo + (hi - lo) * ((int.from_bytes(value, "little") >> 11) * 2.0**-53)

        if override is not None and override not in VARIANTS:
            raise ValueError("rack variant must be low-left, low-right, high-left or high-right")
        if override is None:
            family = "low" if draw("tier", 0, 1) < 0.5 else "high"
            direction = "left" if draw("side", 0, 1) < 0.5 else "right"
        else:
            family, direction = override.split("-")
        sign = 1 if direction == "left" else -1
        h = draw("low-h", 1.205, 1.225) if family == "low" else draw("high-h", 1.36, 1.42)
        return Rack(
            branch,
            sign,
            family,
            draw("front", 1.72, 1.95),
            draw("length", 0.64, 0.80),
            sign * draw("offset", 0.58, 0.72),
            draw("width", 0.96, 1.06),
            h,
        )

    return RackPair(first_m, make("left", left_rack), make("right", right_rack))


def parts(rack, station_m=4.0):
    """Names, local box centers, fixed half-sizes, RGBA and physical contact switch."""
    f, b = rack.front_m, rack.back_m
    center = 0.5 * (f + b)
    lo, hi = rack.edges
    for sign, edge in [(-1, lo), (1, hi)]:
        side = "minus_y" if sign < 0 else "plus_y"
        yield "side_" + side, (center, edge + sign * 1.20, 0.36), (0.43, 1.20, 0.36), SIDE_RGBA, True
        for which, x in [("front", f + 0.05), ("back", b - 0.05)]:
            yield "post_" + which + "_" + side, (x, edge + sign * 0.045, 0.7425), (
                0.05,
                0.045,
                0.7425,
            ), POST_RGBA, True
        yield "stripe_" + side, (2.2, sign * 1.58, 0.002), (2.2, 0.015, 0.001), ".80 .86 .87 1", False
    # Neutral painted square at the standing inspection station on each outgoing aisle.
    for label, offset in [("minus", -0.245), ("plus", 0.245)]:
        yield "station_x_" + label, (station_m + offset, 0.0, 0.0025), (
            0.013,
            0.255,
            0.001,
        ), ".88 .86 .76 1", False
        yield "station_y_" + label, (station_m, offset, 0.0025), (
            0.255,
            0.013,
            0.001,
        ), ".88 .86 .76 1", False
    # Overlapping sections form one contiguous solid shelf across the whole aisle.
    for which, x in [("front", f + 0.23), ("back", b - 0.23)]:
        yield "deck_" + which, (x, rack.left_m, rack.underside_m + 0.055), (
            0.23,
            0.69,
            0.055,
        ), DECK_RGBA, True


def add_models(world):
    example = make_racks(0, 6.0)
    for arm in BRANCHES:
        for part, _, size, color, solid in parts(example.branch(arm)):
            name = "rack_" + arm + "_" + part
            body = ET.SubElement(world, "body", name=name + "_anchor", mocap="true", pos="0 0 -4")
            ET.SubElement(
                body,
                "geom",
                name=name,
                type="box",
                group="0",
                size=" ".join(map(str, size)),
                rgba=color,
                contype="2" if solid else "0",
                conaffinity="1" if solid else "0",
            )


def frame(pair, arm, origin, axes):
    sign = 1 if arm == "left" else -1
    return origin + axes[:, 0] * pair.nominal_pivot_forward_m, axes @ np.array(
        [[0.0, -sign], [sign, 0.0]]
    )


def place(data, pair, origin, axes, yaw, mocaps, station_m):
    for arm in BRANCHES:
        start, R = frame(pair, arm, origin, axes)
        angle = yaw + (math.pi / 2 if arm == "left" else -math.pi / 2)
        quat = (math.cos(angle / 2), 0.0, 0.0, math.sin(angle / 2))
        for part, xyz, _, _, _ in parts(pair.branch(arm), station_m):
            mid = mocaps["rack_" + arm + "_" + part]
            xy = start + R @ xyz[:2]
            data.mocap_pos[mid] = (*xy, xyz[2])
            data.mocap_quat[mid] = quat

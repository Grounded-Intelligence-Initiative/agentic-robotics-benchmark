"""Shared pieces of the reference localizer: reading a flight, the site geometry, the UWB
antenna-pattern and received-power bases and the integrated gyro."""

import json
import math
import os

import numpy as np
import pandas as pd


class Site(object):
    def __init__(self, d):
        self.racks = np.array([[r["x"][0], r["x"][1], r["y"][0], r["y"][1], r["z"][0], r["z"][1]]
                               for r in d["racks"]])
        self.anchors = {a["id"]: np.array(a["position"], dtype=float) for a in d["beacons"]}
        self.pads = np.array([dr["pad"] for dr in sorted(d["drones"], key=lambda x: x["id"])])
        self.n_drones = len(self.pads)

    def surface_below(self, x, y):
        """Vectorised: height of the rack top or floor below (x, y)."""
        x = np.asarray(x, dtype=float)
        y = np.asarray(y, dtype=float)
        h = np.zeros(np.broadcast(x, y).shape)
        for b in self.racks:
            inside = (x >= b[0]) & (x <= b[1]) & (y >= b[2]) & (y <= b[3])
            h = np.where(inside, np.maximum(h, b[5]), h)
        return h

    def blocked(self, p, q):
        return self.inside_length(p, q) > 0.0

    def inside_length(self, p, q):
        """Vectorised: length (m) of each segment p->q (n, 3) inside rack boxes."""
        return self.inside_lengths(p, q).sum(axis=1)

    def inside_lengths(self, p, q):
        """Vectorised: length (m) of each segment p->q (n, 3) inside each rack box (n, R)."""
        p = np.asarray(p, dtype=float)
        q = np.asarray(q, dtype=float)
        d = q - p
        L = np.linalg.norm(d, axis=1)
        out = np.zeros((len(p), len(self.racks)))
        with np.errstate(divide="ignore", invalid="ignore"):
            for r, b in enumerate(self.racks):
                t0 = np.zeros(len(p))
                t1 = np.ones(len(p))
                ok = np.ones(len(p), dtype=bool)
                for a in range(3):
                    lo, hi = b[2 * a], b[2 * a + 1]
                    par = np.abs(d[:, a]) < 1e-12
                    ok &= ~(par & ((p[:, a] < lo) | (p[:, a] > hi)))
                    ta = (lo - p[:, a]) / d[:, a]
                    tb = (hi - p[:, a]) / d[:, a]
                    t0 = np.maximum(t0, np.where(par, -np.inf, np.minimum(ta, tb)))
                    t1 = np.minimum(t1, np.where(par, np.inf, np.maximum(ta, tb)))
                out[:, r] = np.where(ok & (t0 < t1), (t1 - t0) * L, 0.0)
        return out

    def _blocked_unused(self, p, q):
        """Vectorised slab test: does segment p->q (n, 3) pass through any rack?"""
        p = np.asarray(p, dtype=float)
        q = np.asarray(q, dtype=float)
        d = q - p
        hit = np.zeros(len(p), dtype=bool)
        with np.errstate(divide="ignore", invalid="ignore"):
            for b in self.racks:
                t0 = np.zeros(len(p))
                t1 = np.ones(len(p))
                ok = np.ones(len(p), dtype=bool)
                for a in range(3):
                    lo, hi = b[2 * a], b[2 * a + 1]
                    par = np.abs(d[:, a]) < 1e-12
                    ok &= ~(par & ((p[:, a] < lo) | (p[:, a] > hi)))
                    ta = (lo - p[:, a]) / d[:, a]
                    tb = (hi - p[:, a]) / d[:, a]
                    tmin = np.where(par, -np.inf, np.minimum(ta, tb))
                    tmax = np.where(par, np.inf, np.maximum(ta, tb))
                    t0 = np.maximum(t0, tmin)
                    t1 = np.minimum(t1, tmax)
                hit |= ok & (t0 <= t1)
        return hit


def load_flight(folder):
    with open(os.path.join(folder, "site.json")) as f:
        site = Site(json.load(f))
    with open(os.path.join(folder, "flight.json")) as f:
        meta = json.load(f)
    uwb = pd.read_csv(os.path.join(folder, "uwb.csv")).to_numpy()
    drones = []
    for i in range(site.n_drones):
        d = pd.read_csv(os.path.join(folder, "drone_{}.csv".format(i))).to_numpy()
        drones.append(d)
    truth = None
    tp = os.path.join(folder, "truth.csv")
    if os.path.isfile(tp):
        truth = pd.read_csv(tp).to_numpy()
    return site, meta, uwb, drones, truth


N_PATTERN = 8
N_POWER = 2


def pattern_basis(vec, yaw):
    """Antenna-pattern basis (n, 8) of a radio towards the world-frame vectors vec (n, 3), its
    drone at heading yaw (n,), level: real spherical harmonics of degree 1 and 2 of the unit
    direction in the body frame (u_x, u_y, u_z), without the constant (the antenna delay)."""
    vec = np.asarray(vec, dtype=float)
    n = np.maximum(np.linalg.norm(vec, axis=-1), 1e-9)
    c, s = np.cos(yaw), np.sin(yaw)
    ux = (c * vec[..., 0] + s * vec[..., 1]) / n
    uy = (-s * vec[..., 0] + c * vec[..., 1]) / n
    uz = vec[..., 2] / n
    return np.stack([ux, uy, ux * ux - uy * uy, 2 * ux * uy, uz, 0.5 * (3 * uz * uz - 1),
                     2 * uz * ux, 2 * uz * uy], axis=-1)


def power_basis(dist):
    """Received-power (distance) bias basis of the chipset, shared by every radio."""
    lg = np.log(np.maximum(np.asarray(dist, dtype=float), 0.5) / 8.0)
    return np.stack([lg, lg * lg], axis=-1)


def integrated_gyro(drone_rows, times):
    """Integral of the gyro z rate from 0 to each of `times` (trapezoid on the 50 Hz log)."""
    t, w = drone_rows[:, 0], drone_rows[:, 4]
    cum = np.concatenate([[0.0], np.cumsum(0.5 * (w[1:] + w[:-1]) * np.diff(t))])
    return np.interp(times, t, cum)


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi

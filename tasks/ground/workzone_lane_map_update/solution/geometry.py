"""Ground-plane geometry, shared by the camera model and the verifier."""

from bisect import bisect_right
from dataclasses import dataclass
from math import ceil, cos, dist, radians, sin


class RoadSurface:
    """A surveyed longitudinal elevation profile and transverse drainage slope."""

    def __init__(self, profile):
        self.knots = [p[0] for p in profile["station_elevation_m"]]
        self.levels = [p[1] for p in profile["station_elevation_m"]]
        self.crossfall = profile["dz_dx"]

    def height(self,x,y):
        if getattr(y,"ndim",0)>0:
            import numpy as np
            return np.interp(y,self.knots,self.levels)+self.crossfall*x
        i = bisect_right(self.knots,y)
        if i == 0:
            z = self.levels[0]
        elif i == len(self.knots):
            z = self.levels[-1]
        else:
            f = (y-self.knots[i-1])/(self.knots[i]-self.knots[i-1])
            z = self.levels[i-1]+f*(self.levels[i]-self.levels[i-1])
        return z+self.crossfall*x

    def intersection(self,origin,ray):
        x,y,z=origin
        candidates=[]
        limits=[float("-inf"),*self.knots,float("inf")]
        for i,(a,b) in enumerate(zip(limits,limits[1:])):
            if i==0 or i==len(self.knots):
                slope, intercept = 0,self.levels[0 if i==0 else -1]
            else:
                slope=(self.levels[i]-self.levels[i-1])/(b-a)
                intercept=self.levels[i-1]-slope*a
            den=ray[2]-slope*ray[1]-self.crossfall*ray[0]
            if abs(den)<1e-10:
                continue
            t=(intercept+slope*y+self.crossfall*x-z)/den
            if t>0 and a-1e-7<=y+t*ray[1]<=b+1e-7:
                candidates.append(t)
        if candidates:
            distance=min(candidates)
            return x+distance*ray[0],y+distance*ray[1]
        return None


def length(points):
    return sum(dist(a, b) for a, b in zip(points, points[1:]))


def samples(points, spacing=0.25):
    yield points[0]
    for a, b in zip(points, points[1:]):
        count = max(1, ceil(dist(a, b) / spacing))
        for i in range(1, count + 1):
            t = i / count
            yield (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)


def cross_section(line, station):
    """Interpolate x on a boundary with strictly increasing y."""
    i = min(len(line) - 1, max(1, bisect_right(line, station, key=lambda point: point[1])))
    (x0, y0), (x1, y1) = line[i - 1], line[i]
    weight = min(1.0, max(0.0, (station - y0) / (y1 - y0)))
    return x0 + weight * (x1 - x0)


def nearest_station(point, line):
    """Return distance to a polyline and distance along it to the nearest point."""
    best = (float("inf"), 0.0)
    along = 0.0
    for a, b in zip(line, line[1:]):
        dx, dy = b[0] - a[0], b[1] - a[1]
        segment = dist(a, b)
        if segment == 0:
            continue
        t = ((point[0] - a[0]) * dx + (point[1] - a[1]) * dy) / segment**2
        t = min(1.0, max(0.0, t))
        best = min(best, (dist(point, (a[0] + t * dx, a[1] + t * dy)), along + t * segment))
        along += segment
    return best


def dot(a, b):
    return sum(x * y for x, y in zip(a, b))


@dataclass
class Camera:
    position: list
    yaw_deg: float
    pitch_down_deg: float
    focal: float
    cx: float
    cy: float

    def __post_init__(self):
        yaw, pitch = radians(self.yaw_deg), radians(self.pitch_down_deg)
        self.right = (cos(yaw), -sin(yaw), 0.0)
        self.down = (-sin(yaw) * sin(pitch), -cos(yaw) * sin(pitch), -cos(pitch))
        self.forward = (sin(yaw) * cos(pitch), cos(yaw) * cos(pitch), -sin(pitch))

    def camera_point(self, point):
        from_camera = [x - c for x, c in zip(point, self.position)]
        return tuple(dot(from_camera, axis) for axis in (self.right, self.down, self.forward))

    def pixel(self, camera_point):
        x, y, depth = camera_point
        return (self.cx + self.focal * x / depth, self.cy + self.focal * y / depth)

    def ground_point(self, u, v):
        x, y = (u - self.cx) / self.focal, (v - self.cy) / self.focal
        ray = [f + x * r + y * d for f, r, d in zip(self.forward, self.right, self.down)]
        if hasattr(self,"surface"):
            return self.surface.intersection(self.position,ray)
        if ray[2] >= 0:
            return None
        scale = -self.position[2] / ray[2]
        return tuple(self.position[i] + scale * ray[i] for i in (0, 1))

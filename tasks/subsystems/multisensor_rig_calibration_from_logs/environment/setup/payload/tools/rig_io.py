"""Small readers for the rig format (see the task instruction for the full schema).

    import sys; sys.path.insert(0, "/home/user/tools"); import rig_io
    rig = rig_io.Rig("/opt/rigcal/practice/rig_a")
    seq = rig.sequence("seq_00")
    xyz, refl = seq.scan_points(0)        # LiDAR scan 0: points (N,3) in the LiDAR frame, reflectivity 0..1
    img = seq.image(0)                    # camera frame 0: uint8 (240, 320, 3), RGB
"""

import csv
import json
import os

import numpy as np


class Rig(object):
    def __init__(self, path):
        self.path = path
        with open(os.path.join(path, "manifest.json")) as fh:
            self.manifest = json.load(fh)
        import yaml
        with open(os.path.join(path, "rough_prior.yaml")) as fh:
            self.prior = yaml.safe_load(fh)
        lid = self.manifest["lidar"]
        el = np.radians(np.array(lid["elevation_deg"], float))[:, None]
        az = np.radians(np.arange(lid["columns"]) * 360.0 / lid["columns"])[None, :]
        self.ray_dirs = np.stack([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az),
                                  np.sin(el) * np.ones_like(az)], axis=-1)   # (rings, columns, 3)
        self.K = np.array(self.manifest["camera"]["K"], float)
        self.T_body_lidar = np.array(self.manifest["T_body_lidar"], float)

    def sequence(self, name):
        return Sequence(self, os.path.join(self.path, name))

    @property
    def sequence_names(self):
        return [s["name"] for s in self.manifest["sequences"]]


def _stamps(path):
    with open(path) as fh:
        rows = list(csv.reader(fh))[1:]
    return [r[0] for r in rows], np.array([float(r[1]) for r in rows])


class Sequence(object):
    def __init__(self, rig, path):
        self.rig = rig
        self.path = path
        self.cam_files, self.cam_stamps = _stamps(os.path.join(path, "camera", "timestamps.csv"))
        self.scan_files, self.scan_stamps = _stamps(os.path.join(path, "lidar", "timestamps.csv"))
        imu = np.loadtxt(os.path.join(path, "imu.csv"), delimiter=",", skiprows=1)
        self.imu_stamps, self.gyro, self.accel = imu[:, 0], imu[:, 1:4], imu[:, 4:7]
        od = np.loadtxt(os.path.join(path, "wheel_odom.csv"), delimiter=",", skiprows=1)
        self.odom_stamps, self.odom_v, self.odom_yaw_rate = od[:, 0], od[:, 1], od[:, 2]

    def scan(self, i):
        """Range image (rings, columns) in metres (0 = no return) and reflectivity 0..1."""
        z = np.load(os.path.join(self.path, "lidar", "scans", self.scan_files[i]))
        return z["range_mm"].astype(np.float64) / 1000.0, z["reflectivity"].astype(np.float64) / 255.0

    def scan_points(self, i):
        r, refl = self.scan(i)
        m = r > 0
        return self.rig.ray_dirs[m] * r[m][:, None], refl[m]

    def image(self, i):
        import cv2
        img = cv2.imread(os.path.join(self.path, "camera", "frames", self.cam_files[i]), cv2.IMREAD_COLOR)
        return img[:, :, ::-1].copy()

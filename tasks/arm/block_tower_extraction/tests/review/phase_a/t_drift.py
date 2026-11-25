"""Step 1: tower stands still for T s; report drift and speed. usage: t_drift.py <variant> [T] [seed] [panda]"""
import json
import math
import sys
import time

import mujoco
import numpy as np

import tower as T

VAR = {
    "base": {},
    "dt05": dict(dt=0.0005, solref=(0.001, 1.0)),
    "dt2": dict(dt=0.002, solref=(0.004, 1.0)),
    "soft": dict(solref=(0.004, 1.0), solimp=(0.95, 0.99, 0.0005, 0.5, 2.0)),
    "noslip": dict(noslip=5),
    "lowit": dict(iterations=30, ls_iterations=10, tol=1e-8),
    "lowit_ns": dict(iterations=30, ls_iterations=10, tol=1e-8, noslip=3),
}
var = sys.argv[1]
dur = float(sys.argv[2]) if len(sys.argv) > 2 else 60.0
seed = int(sys.argv[3]) if len(sys.argv) > 3 else 1
panda = sys.argv[4] if len(sys.argv) > 4 else None
sleep = "sleep" in sys.argv
phys = dict(T.PHYS)
phys.update(VAR[var])
tw = T.gen_tower(seed)
spec = T.build_spec(tw, panda_dir=panda, phys=phys)
m = spec.compile()
if sleep:
    m.opt.enableflags |= mujoco.mjtEnableBit.mjENBL_SLEEP
d = mujoco.MjData(m)
if panda:
    q0 = [0.0, -0.6, 0.0, -2.4, 0.0, 1.8, 0.785]
    d.qpos[:7] = q0
    d.ctrl[:7] = q0
    d.ctrl[7] = 255
mujoco.mj_forward(m, d)
ids = T.block_ids(m, len(tw["blocks"]))
pg = d.xpos[ids].copy()
for _ in range(int(2.0 / m.opt.timestep)):      # settle (part of tower generation)
    mujoco.mj_step(m, d)
settle_mm = 1000 * np.linalg.norm(d.xpos[ids] - pg, axis=1).max()
p0 = d.xpos[ids].copy()
q0 = d.xquat[ids].copy()
n = int(round(dur / m.opt.timestep))
n1 = int(round(1.0 / m.opt.timestep))
t0 = time.time()
p1 = None
maxv = []
ncon = []
for i in range(n):
    mujoco.mj_step(m, d)
    if i == n1:
        p1 = d.xpos[ids].copy()
    if i % 1000 == 0:
        ncon.append(d.ncon)
wall = time.time() - t0
pe = d.xpos[ids].copy()
dz0 = np.linalg.norm(pe - p0, axis=1)
dz1 = np.linalg.norm(pe - p1, axis=1)
lat1 = np.linalg.norm((pe - p1)[:, :2], axis=1)
v = np.abs(d.qvel[-6 * len(ids):]).max()
ang = [2 * math.degrees(math.acos(min(1.0, abs(float(np.dot(a, b)))))) for a, b in zip(q0, d.xquat[ids])]
out = dict(var=var, seed=seed, panda=bool(panda), sleep=sleep, dur=dur, dt=m.opt.timestep, wall_s=round(wall, 2),
           rtf=round(dur / wall, 3), ms_per_step=round(1000 * wall / n, 3), ncon_mean=float(np.mean(ncon)),
           nefc=int(d.nefc), max_drift_from0_mm=round(1000 * dz0.max(), 4), max_drift_after1s_mm=round(1000 * dz1.max(), 4),
           max_lat_after1s_mm=round(1000 * lat1.max(), 4), sink_top_mm=round(1000 * (p0[-1, 2] - pe[-1, 2]), 4),
           max_qvel_end=float(v), settle_mm=round(settle_mm, 3), max_rot_deg=round(max(ang), 5))
print(json.dumps(out))

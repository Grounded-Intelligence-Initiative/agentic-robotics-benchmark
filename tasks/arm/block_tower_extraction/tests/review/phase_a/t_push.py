"""Step 2: push loose vs tight blocks with a stiff probe; record force-displacement and tower disturbance."""
import json, math, sys, time
import mujoco, numpy as np
import tower as T

def push_test(seed, bi, travel=0.035, speed=0.01, fmax=None, sleep=False):
    tw = T.gen_tower(seed)
    b = tw["blocks"][bi]
    yaw = b["yaw"]; ax = np.array([math.cos(yaw), math.sin(yaw), 0.0])
    p_face = np.array(b["pos"]) - ax * (T.L / 2 + b["dax"] * 0)   # approximate; refined after settle
    spec = T.build_spec(tw)
    start = np.array(b["pos"]) - ax * (T.L / 2 + 0.02)
    T.add_gantry_probe(spec, start)
    m = spec.compile(); d = mujoco.MjData(m)
    if sleep: m.opt.enableflags |= mujoco.mjtEnableBit.mjENBL_SLEEP
    jadr = [m.jnt_qposadr[m.joint("gp_" + n).id] for n in "xyz"]
    mujoco.mj_forward(m, d); T.settle(m, d, 2.0)
    ids = T.block_ids(m, 54); top = T.block_loads(m, d, ids)
    p0 = d.xpos[ids].copy(); q0 = d.xquat[ids].copy()
    # target block's actual end face after settle
    bp = d.xpos[ids[bi]].copy(); R = d.xmat[ids[bi]].reshape(3, 3); axr = R[:, 0]; axr[2] = 0; axr /= np.linalg.norm(axr)
    face = bp - axr * T.L / 2
    pos0 = face - axr * (0.003 + 0.004)           # probe centre 4 mm off the face
    cmd = pos0 - start
    aid = [m.actuator("gpa_" + c).id for c in "xyz"]
    d.qpos[jadr] = cmd; d.ctrl[aid] = cmd; mujoco.mj_forward(m, d)
    T.settle(m, d, 0.2)
    out = []; f_first = None; fpk = 0.0; collapsed = False
    n = int((travel + 0.004) / speed / m.opt.timestep)
    aid = [m.actuator("gpa_" + c).id for c in "xyz"]
    for i in range(n):
        s = (i + 1) * speed * m.opt.timestep
        d.ctrl[aid] = cmd + axr * s
        mujoco.mj_step(m, d)
        f = float(np.dot(d.actuator_force[aid], axr))       # force the probe exerts along the push axis
        disp = float(np.dot(d.xpos[ids[bi]] - bp, axr))
        fpk = max(fpk, f)
        if f_first is None and disp > 0.001:
            f_first = fpk
        if i % 100 == 0:
            others = np.linalg.norm(np.delete(d.xpos[ids] - p0, bi, axis=0), axis=1)
            out.append((round(s * 1000, 1), round(f, 3), round(disp * 1000, 2), round(float(others.max()) * 1000, 3)))
            if others.max() > 0.010:
                collapsed = True
        if fmax is not None and f > fmax:
            break
    T.settle(m, d, 1.0)
    others = np.linalg.norm(np.delete(d.xpos[ids] - p0, bi, axis=0), axis=1)
    return dict(seed=seed, block=bi, level=b["level"], slot=b["slot"], design_tight=b["tight"], top_load_N=round(float(top[bi]), 3),
                mu=round(b["mu"], 3), f_to_move_1mm_N=None if f_first is None else round(f_first, 3), f_peak_N=round(fpk, 3),
                block_disp_mm=round(out[-1][2], 2), others_max_disp_mm_end=round(float(others.max()) * 1000, 3),
                fell=bool(others.max() > 0.010), trace=out[::5])

if __name__ == "__main__":
    seed = int(sys.argv[1]); bi = int(sys.argv[2]); fmax = float(sys.argv[3]) if len(sys.argv) > 3 and sys.argv[3] != "none" else None
    t0 = time.time(); r = push_test(seed, bi, fmax=fmax); r["wall_s"] = round(time.time() - t0, 1)
    print(json.dumps(r))

"""Identify an instrument from its recordings; the parameters compensate.py needs.

  python3 fit.py <recordings_dir> <params.json> [<angles cache .npz>]

<recordings_dir> holds one folder per recording session (motors.csv, endoscope/,
endoscope_pose.json); run.py calls identify() on a mounted instrument's folder, whose one
session is its calibration recording.

1. Joint angles from the stereo endoscope video (vision.py: silhouette fits of the
   instrument's meshes, with the endoscope pose calibrated from the video itself).
2. Camera clock: per session, the offset that best explains pitch by the pitch encoder.
3. Servo lag: per motor, the first-order time constant that best maps commands to encoders.
4. Lost motion: per joint, a Prandtl-Ishlinskii model on the encoder signal (nonnegative
   weights over a grid of play widths); for the jaws also the pitch coupling and the growth
   of the play with |pitch|, found by a coarse grid then Nelder-Mead. Every fit is trimmed:
   frames whose angle the model misses by many times the typical residual are left out.
5. Consistency: the frames whose seen angles disagree with what the model predicts from the
   motor log (or that the first pass could not use) are fitted again in the video, starting
   from that prediction, and the models are identified again.
"""

import json
import math
import os
import sys
import time

import numpy as np
import pandas as pd
from scipy.optimize import lsq_linear, minimize

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import vision  # noqa: E402
import wristmodel as WM  # noqa: E402


TRIM = 6.0           # a frame further than this many robust sigmas from the model is left out
TRIM_FLOOR = 0.002   # rad: never trim closer than this
RECHECK_BY = float(os.environ.get("WRIST_RECHECK_BY", 510.0))   # s since the program started


def load(bench, cache=None):
    if cache and os.path.isfile(cache):
        z = np.load(cache)
        names = [str(n) for n in z["names"]]
        seen = {n: dict(stamp=z[n + "__stamp"], angles=z[n + "__angles"]) for n in names}
        l1, l2 = float(z["l1"]), float(z["l2"])
    else:
        seen, l1, l2 = vision.angles(bench)
        if cache:
            np.savez(cache, names=np.array(sorted(seen)), l1=l1, l2=l2,
                     **{"{}__{}".format(n, k): v for n, d in seen.items()
                        for k, v in d.items() if k in ("stamp", "angles")})
    sessions = []
    for name in sorted(seen):
        m = pd.read_csv(os.path.join(bench, name, "motors.csv")).to_numpy()
        a = seen[name]["angles"]
        sessions.append(dict(name=name, t=m[:, 0], cmd=m[:, 1:4], enc=m[:, 4:7],
                             stamp=seen[name]["stamp"], pitch=a[:, 0].copy(),
                             jaw1=a[:, 1].copy(), jaw2=a[:, 2].copy(),
                             video=seen[name].get("session")))
    return sessions, dict(l1=l1, l2=l2)


def clock_offset(s):
    """Camera clock offset: the shift that best explains measured pitch by an affine function of
    the pitch encoder (lost motion only adds a residual that does not move the minimum)."""
    ok = np.isfinite(s["pitch"])
    st, pit = s["stamp"][ok], s["pitch"][ok]

    def resid(o):
        e = np.interp(st - o, s["t"], s["enc"][:, 0])
        A = np.column_stack([e, np.ones_like(e)])
        c, *_ = np.linalg.lstsq(A, pit, rcond=None)
        r2 = np.sort((A @ c - pit) ** 2)
        return float(np.mean(r2[:int(0.9 * len(r2))]))     # a few wrong frames do not count

    grid = np.arange(-1.5, 1.5, 0.005)
    best = grid[int(np.argmin([resid(o) for o in grid]))]
    fine = np.arange(best - 0.006, best + 0.006, 0.0002)
    return float(fine[int(np.argmin([resid(o) for o in fine]))])


def refine_offset(s):
    """Re-estimate the camera clock offset against the full pitch model (lost motion included):
    the affine first guess is biased by the hysteresis's apparent phase lag."""
    ok = np.isfinite(s["pitch"])
    st, pit = s["stamp"][ok], s["pitch"][ok]

    def resid(o):
        r2 = np.sort((np.interp(st - o, s["t"], s["pitch_model"]) - pit) ** 2)
        return float(np.mean(r2[:int(0.95 * len(r2))]))

    grid = np.arange(s["offset"] - 0.03, s["offset"] + 0.03, 0.0001)
    return float(grid[int(np.argmin([resid(o) for o in grid]))])


def servo_lag(sessions):
    taus = []
    grid = np.arange(0.005, 0.045, 0.0002)
    for i in range(3):
        err = np.zeros(len(grid))
        for s in sessions:
            cmd, enc = s["cmd"][:, i], s["enc"][:, i]
            q = np.full(len(grid), enc[0])
            alpha = 1.0 - np.exp(-0.001 / grid)
            pred = np.empty((len(cmd), len(grid)))
            pred[0] = q
            for k in range(len(cmd) - 1):
                for _ in range(5):
                    q = q + np.clip((cmd[k] - q) * alpha, -0.03, 0.03)
                pred[k + 1] = q
            err += np.sum((pred - enc[:, None]) ** 2, axis=0)
        taus.append(float(grid[int(np.argmin(err))]))
    return taus


def sample(series, s):
    """Values of a 200 Hz drive-clock series at the camera frames' drive times."""
    td = s["stamp"] - s["offset"]
    if series.ndim == 1:
        return np.interp(td, s["t"], series)
    return np.column_stack([np.interp(td, s["t"], series[:, m]) for m in range(series.shape[1])])


def fit_linear(feats, target, keep=None, rounds=4):
    """theta = b + a0 v + sum w play: b free, the rest nonnegative. Trimmed: refitted without
    the samples far off the fit until the kept set settles (or on the given `keep` only)."""
    A = np.concatenate(feats)
    y = np.concatenate(target)
    n = A.shape[1]
    A1 = np.column_stack([np.ones(len(A)), A])
    lb = np.r_[-np.inf, np.zeros(n)]
    fixed = keep is not None
    keep = np.ones(len(y), bool) if keep is None else keep
    for _ in range(1 if fixed else rounds):
        x = lsq_linear(A1[keep], y[keep], bounds=(lb, np.full(n + 1, np.inf))).x
        r = A1 @ x - y
        if fixed:
            break
        sigma = 1.4826 * float(np.median(np.abs(r[keep])))
        new = np.abs(r) < max(TRIM * sigma, TRIM_FLOOR)
        if np.array_equal(new, keep):
            break
        keep = new
    return x, float(np.mean(r[keep] ** 2)), keep


def fit_pitch(sessions):
    feats, target = [], []
    for s in sessions:
        q = s["enc"][:, 0]
        F = np.column_stack([q, WM.play_series(q, WM.RHO)])
        Fs = sample(F, s)
        ok = np.isfinite(s["pitch"])
        feats.append(Fs[ok])
        target.append(s["pitch"][ok])
    x, mse, _ = fit_linear(feats, target)
    model = WM.JointModel(x[0], x[1], x[2:])
    for s in sessions:   # model pitch on the drive clock, used by the jaw fits
        q = s["enc"][:, 0]
        s["pitch_model"] = x[0] + np.column_stack([q, WM.play_series(q, WM.RHO)]) @ x[1:]
    return model, mse


def fit_jaw(sessions, j):
    key = "jaw{}".format(j)

    def series(kappa, mu, s):
        v = s["enc"][:, j] + kappa * s["pitch_model"]
        scale = np.exp(mu * np.abs(s["pitch_model"]))
        return np.column_stack([v, WM.play_series(v, WM.RHO, scale)])

    def solve(kappa, mu, keep=None):
        feats, target = [], []
        for s in sessions:
            Fs = sample(series(kappa, mu, s), s)
            ok = np.isfinite(s[key])
            feats.append(Fs[ok])
            target.append(s[key][ok])
        return fit_linear(feats, target, keep)

    # coarse grid (trimmed fits), then Nelder-Mead on the frames the best grid point keeps,
    # then once more on the frames the optimum keeps
    best = (None, np.inf, None)
    for kappa in np.linspace(-1.1, -0.2, 10):
        for mu in np.linspace(0.0, 1.5, 7):
            x, mse, keep = solve(kappa, mu)
            if mse < best[1]:
                best = ((kappa, mu), mse, keep)
    p0, keep = np.array(best[0]), best[2]
    steps = (0.08, 0.15)
    for _ in range(2):
        res = minimize(lambda p: solve(p[0], max(p[1], 0.0), keep)[1], x0=p0,
                       method="Nelder-Mead",
                       options=dict(xatol=1e-4, fatol=1e-12, maxiter=120,
                                    initial_simplex=[p0, p0 + [steps[0], 0.0],
                                                     p0 + [0.0, steps[1]]]))
        p0 = np.array([res.x[0], max(res.x[1], 0.0)])
        keep = solve(p0[0], p0[1])[2]
        steps = (0.02, 0.04)
    kappa, mu = float(p0[0]), float(p0[1])
    x, mse, keep = solve(kappa, mu)
    for s in sessions:      # model jaw angle on the drive clock
        s[key + "_model"] = x[0] + series(kappa, mu, s) @ x[1:]
    return WM.JointModel(x[0], x[1], x[2:], kappa=kappa, mu=mu), mse


def fit_models(sessions):
    pitch, mse_p = fit_pitch(sessions)
    for _ in range(2):
        for s in sessions:
            s["offset"] = refine_offset(s)
        pitch, mse_p = fit_pitch(sessions)
    jaw1, mse_1 = fit_jaw(sessions, 1)
    jaw2, mse_2 = fit_jaw(sessions, 2)
    return (pitch, jaw1, jaw2), (mse_p, mse_1, mse_2)


KEYS = ("pitch", "jaw1", "jaw2")


def recheck(sessions, mses, geo, log=True):
    """Fit again, from the model's prediction, every frame whose seen angles are off the
    prediction (or were not usable). A re-fit replaces the first one where it explains the
    image at least as well."""
    changed = 0
    for s in sessions:
        sess = s["video"]
        if sess is None:
            continue
        pred = np.column_stack([sample(s[k + "_model"], s) for k in KEYS])
        seen = np.column_stack([s[k] for k in KEYS])
        thr = np.maximum(TRIM * np.sqrt(np.array(mses)), TRIM_FLOOR)
        off = np.any(~np.isfinite(seen) | (np.abs(seen - pred) > thr), axis=1)
        frames = np.where(off)[0]
        if len(frames) == 0:
            continue
        Q, rms = vision.refit(sess, frames, pred[frames], geo["l1"], geo["l2"])
        better = np.isfinite(Q) & (np.abs(Q - pred[frames]) <= thr)
        for i, k in enumerate(KEYS):
            s[k][frames[better[:, i]]] = Q[better[:, i], i]
        changed += int(better.sum())
        if log:
            print("[fit] {}: {} frames re-fitted from the model, {} angles recovered".format(
                s["name"], len(frames), int(better.sum())), file=sys.stderr, flush=True)
    return changed


def end_state(s, models):
    """Each joint's play states (and input) when the recording `s` ends and the drive, holding
    its last command, has brought the motors to rest there: the episodes start from it."""
    out = {}
    pm = s["pitch_model"]
    for i, (key, m) in enumerate(zip(KEYS, models)):
        y = m.end_state(s["enc"][:, i] + m.kappa * pm, pm)
        v_rest = float(s["cmd"][-1, i] + m.kappa * pm[-1])
        out[key] = dict(y=m.settle(y, v_rest, pm[-1]).tolist(), v=v_rest)
    return out


def identify(bench, cache=None, log=True):
    """Every recording session under `bench` -> the parameters compensate.py needs."""
    sessions, geo = load(bench, cache)
    for s in sessions:
        s["offset"] = clock_offset(s)
    taus = servo_lag(sessions)
    (pitch, jaw1, jaw2), (mse_p, mse_1, mse_2) = fit_models(sessions)
    # the re-fits from the model need about half a minute: not when the program is late
    if time.time() - vision._T0 > RECHECK_BY:
        print("[fit] behind time: no re-fits from the model", file=sys.stderr, flush=True)
    elif recheck(sessions, (mse_p, mse_1, mse_2), geo, log):
        (pitch, jaw1, jaw2), (mse_p, mse_1, mse_2) = fit_models(sessions)
    params = dict(tau=taus, geometry=geo, offsets={s["name"]: s["offset"] for s in sessions},
                  pitch=pitch.as_dict(), jaw1=jaw1.as_dict(), jaw2=jaw2.as_dict(),
                  state=end_state(sessions[-1], (pitch, jaw1, jaw2)),
                  residual_deg=[math.degrees(math.sqrt(m)) for m in (mse_p, mse_1, mse_2)])
    if log:
        print("[fit] tau", np.round(taus, 4), "L1 %.3f L2 %.3f" % (geo["l1"], geo["l2"]),
              "offsets", [round(s["offset"], 4) for s in sessions],
              "kappa %.4f %.4f mu %.3f %.3f" % (jaw1.kappa, jaw2.kappa, jaw1.mu, jaw2.mu),
              "residual deg", np.round(params["residual_deg"], 4), file=sys.stderr, flush=True)
    return params


def main():
    bench, out = sys.argv[1], sys.argv[2]
    params = identify(bench, sys.argv[3] if len(sys.argv) > 3 else None)
    with open(out, "w") as f:
        json.dump(params, f, indent=1)


if __name__ == "__main__":
    main()

"""Baselines (maintainers' tool; never run by the grader): each is a variant of the honest program
(tests/review/honest/) that leaves out or botches one essential part of the job. Run one as a submission:
    import baselines; baselines.solve(cell, "<name>")       (a submission folder holds honest/*.py, this file and a
                                                             belt.py calling it with its name)
Names:
  routed_only        drape the loop over ALT, under I and over WP and seat the top spans, never open the tensioner
                     nor put the last span on
  forced_span        route as above, then push the last span in beside the closed tensioner pulley (no opening)
  chord_pull         take the lever (left hand) and pull it along a straight chord at full stiffness, then let go
  fast_release       the honest program, but the tensioner let back at 120 deg/s and the lever released loaded
  one_arm            the right (belt) arm only: drape the loop over ALT, push the top run and C by hand
  open_loop          the honest sequence on the nominal drawing alone: no camera frame (nominal engine pose, the belt
                     where the nominal geometry says), no re-aims, no checks
"""
import math
import os
import sys

import numpy as np

_here = os.path.dirname(os.path.abspath(__file__))
for _d in (os.path.join(_here, "honest"), os.path.join(os.path.dirname(_here), "honest")):
    if os.path.isdir(_d):
        sys.path.insert(0, _d)
        break
import hseq  # noqa: E402


def _routed(s):
    for st in (s.st_look, s.st_drape, s.st_seat_top):
        st()


def solve(cell, name):
    given = os.environ.get("ARB_GIVEN_DIR", os.path.expanduser("~/given"))
    s = hseq.Seq(cell, given)
    r = s.r
    try:
        if name == "routed_only":
            _routed(s)
        elif name == "forced_span":
            _routed(s)
            # the last span pushed in towards the face just beside T's pulley (inside the loop) with the closed
            # fingertips, the tensioner left on its closed stop
            dw = s.dw
            tc = dw.T_centre(dw.th_closed)
            for dz in (-0.02, 0.0, 0.02):
                r.push(s.B, tc[0] + dw.T_r + 0.012, tc[1] + dz, 0.075, 0.03, fmax=25.0)
        elif name == "chord_pull":
            s.st_look()
            r.pin_lever()
            g = r.goal[s.T][0].copy()
            u = r.W(*r.lever_E(s.dw.th_open)) - g
            r.move(s.T, g + u, speed=0.10, fmax=60.0)
            r.set_width(s.T, 0.05, wait=0.3)
        elif name == "fast_release":
            hseq.OPT["RT_LAST_OPEN"] = 85.0
            orig = s.guided_return

            def fast(B_holding, tag, rate_dps=10.0):
                return orig(B_holding, tag, rate_dps=120.0)
            s.guided_return = fast
            s.run()
        elif name == "one_arm":
            s.st_look()
            Q = s.peg_survey()
            c = Q[:, :2].mean(axis=0)
            ang = np.degrees(np.arctan2(Q[:, 1] - c[1], Q[:, 0] - c[0]))
            a = int(np.argmin(np.abs((ang - hseq.OPT["DS_ANG"] + 180) % 360 - 180)))
            s.drape_grab(s.B, [(Q[a], s.normal_at(Q, a))], push=True)
            pa = r.hand_pos(s.B)
            r.moveE(s.B, pa[0], pa[1] + 0.02, 0.17, speed=0.08)
            alt_top = s.ALT[1] + s.dw.pul["ALT"]["r"] + 0.5 * s.dw.belt_thick
            r.moveE(s.B, s.ALT[0], alt_top + 0.08, 0.17, speed=0.10)
            r.moveE(s.B, s.ALT[0], alt_top + 0.04, s.PLANE + 0.0083, speed=0.02, fmax=8.0)
            s.drape_release(s.B)
            r.park(s.B)
            s.push_seat("ALT", (40, 70, 100), side=s.B)
            s.push_seat("WP", (60, 100, 140), side=s.B)
            s.push_seat("C", (-90, -60, -120), side=s.B)
        elif name == "open_loop":
            f = r.info["frames"]["engine_nominal_pose"]
            nominal = (np.asarray(f["origin_world"], float), np.column_stack([f["x_E_world"], f["y_E_world"], f["z_E_world"]]).astype(float))
            s.st_look = lambda: r.set_engine(*nominal)
            s.y.frame = lambda name: (_ for _ in ()).throw(RuntimeError("open loop: no camera"))
            s.y.local = lambda *a, **k: None
            orig_aim = s.aim_fn

            def aim_fn(side, cands, twist_max=0.0):
                def aim(i, phase):
                    pE, nrm = cands[i]
                    y = r.R_E @ np.array([nrm[0], 0.0, nrm[1]])
                    return r.W(*pE), y
                return aim
            s.aim_fn = aim_fn
            s.y.lever_angle = lambda: None
            r.lever_eye = None

            def nominal_pts(tag=""):
                # the loop as the drawing would have it: the loop hanging from ALT, then the route over the
                # pulleys (C, ALT, I, WP; T open) sampled every 121 points at the groove plane
                if tag in ("hung_ALT",):
                    a = np.linspace(0, 2 * math.pi, 121, endpoint=False)
                    return np.column_stack([s.ALT[0] + 0.08 * np.cos(a), s.ALT[1] - 0.20 + 0.25 * np.sin(a), np.full_like(a, 0.06)])
                a = np.linspace(0, 2 * math.pi, 121, endpoint=False)
                return np.column_stack([0.02 + 0.14 * np.cos(a), 0.11 + 0.20 * np.sin(a), np.full_like(a, s.PLANE)])
            s.pts = nominal_pts
            s.peg_survey = lambda: np.column_stack([s.dw.peg[0] + 0.15 * np.cos(np.linspace(0, 6.28, 121)),
                                                    s.dw.peg[1] - 0.20 + 0.22 * np.sin(np.linspace(0, 6.28, 121)),
                                                    np.full(121, 0.06)])
            s.check_now = lambda tag: ([], [])
            s.run()
    except Exception as e:  # noqa: BLE001
        print("[baseline] %s ended: %s" % (name, e), file=sys.stderr)

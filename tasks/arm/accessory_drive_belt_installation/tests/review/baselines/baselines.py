"""Baselines (maintainers' tool; never run by the grader): each is a variant of the honest program
(tests/review/honest/) that leaves out or botches one essential part of the job. Run one as a submission:
    import baselines; baselines.solve(cell, "<name>")       (a submission folder holds honest/*.py, this file and a
                                                             belt.py calling it with its name)
Names:
  routed_only        drape the loop and seat the top spans, never open the tensioner nor put the last span on
  forced_span        route as above, then push the last span in under the closed tensioner pulley (no opening)
  chord_pull         take the lever and pull it along a straight chord at full stiffness, then let go
  fast_release       the honest program, but the tensioner let back at 120 deg/s and the lever released loaded
  one_arm            the left arm only: drape the loop over WP and the top, push the rest by hand
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
            # the last span pushed in towards the face just under T's pulley with the closed fingertips
            dw = s.dw
            tc = dw.T_centre(dw.th_closed)
            for dz in (-0.02, 0.0, 0.02):
                r.push("L", tc[0] - dw.T_r - 0.012, tc[1] + dz, 0.075, 0.03, fmax=25.0)
        elif name == "chord_pull":
            s.st_look()
            r.pin_lever()
            g = r.goal["R"][0].copy()
            u = r.W(*r.lever_E(s.dw.th_open)) - g
            r.move("R", g + u, speed=0.10, fmax=60.0)
            r.set_width("R", 0.05, wait=0.3)
        elif name == "fast_release":
            hseq.OPT["RT_LAST"] = -60.0
            orig = s.guided_return

            def fast(L_holding, tag, rate_dps=10.0):
                return orig(L_holding, tag, rate_dps=120.0)
            s.guided_return = fast
            s.run()
        elif name == "one_arm":
            s.st_look()
            s._stop_after_L = True
            s.st_drape()
            s.push_seat("WP", (100, 140), side="L")
            s.push_seat("C", (-90, -60, -120), side="L")
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
                # the loop as the drawing would have it: the peg loop hanging straight, then the route over the
                # pulleys (C, WP, ALT; T open) sampled every 10 mm at the groove plane
                if tag in ("hung_WP",):
                    a = np.linspace(0, 2 * math.pi, 105, endpoint=False)
                    return np.column_stack([s.WP[0] + 0.07 * np.cos(a), s.WP[1] - 0.20 + 0.25 * np.sin(a), np.full_like(a, 0.06)])
                a = np.linspace(0, 2 * math.pi, 105, endpoint=False)
                return np.column_stack([0.0 + 0.12 * np.cos(a), 0.10 + 0.20 * np.sin(a), np.full_like(a, s.PLANE)])
            s.pts = nominal_pts
            s.peg_survey = lambda: np.column_stack([s.dw.peg[0] + 0.06 * np.cos(np.linspace(0, 6.28, 105)),
                                                    s.dw.peg[1] - 0.19 + 0.19 * np.sin(np.linspace(0, 6.28, 105)),
                                                    np.full(105, 0.06)])
            s.check_now = lambda tag: ([], [])
            s.run()
    except Exception as e:  # noqa: BLE001
        print("[baseline] %s ended: %s" % (name, e), file=sys.stderr)

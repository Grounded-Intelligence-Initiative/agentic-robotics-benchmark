"""The acceptance after finish (cell internals): the robots are switched off and out of the drive, the drive settles for
1 s, a run-in motor turns the crank at 1.5 rev/s for 4 s (0.3 s ramp; clockwise as drawn for drive B) against the accessories' drag, then the drive is
inspected.

Per engine: score = g x q, where
  g = no damage (during the episode, the switch-off, the settle and the run-in) x no protective stop x the tensioner at
      rest inside its window (after the run-in, not held) x the backside idler I passing (its back wrap on the face);
  q = (pulleys passing winding and groove after the run-in (C, WP, ALT, T) + 1 if the run-in slip is within spec) / 5.
A physics fault ends the episode as a task error (the episode is re-run once by the grader, never credited).
"""

import math

import mujoco
import numpy as np

from . import drive as dr

__all__ = ["RUN_IN", "run_in", "score_engine"]

RUN_IN = {"settle_s": 1.0, "duration_s": 4.0, "speed_rev_s": 1.5, "ramp_s": 0.3, "kv": 20.0, "slip_max": 0.03}


def robots_off(scene, rules):
    """Open every grip and take the arms out of the drive (their contacts disabled)."""
    m = scene.model
    for s in list(rules.held):
        if rules.held[s] is not None:
            rules.release(s, "finish")
    for g in range(m.ngeom):
        if int(scene.geom_arm[g]):
            m.geom_contype[g] = 0
            m.geom_conaffinity[g] = 0
    for e in range(m.neq):
        nm = m.equality(e).name
        if nm.startswith(("grip/", "pin/")):
            scene.data.eq_active[e] = 0


def run_in(scene, rules, tick=None):
    """The settle and the run-in on the scene (arms off). tick(): one 10 ms tick of the drive (default: the scene's
    physics with the arms' torques zeroed). Returns {ok, slip {WP, ALT}, inspect, dphi_C}."""
    m, d = scene.model, scene.data
    dv = scene.drive
    robots_off(scene, rules)
    if tick is None:
        def tick():
            return scene.step_physics(arms=False)
    phys_tick = tick

    def tick():
        # the damage rules hold through the switch-off, the settle and the run-in as during the episode: a lever let go
        # by the switch-off that snaps back, or a belt that falls onto the plate, damages the engine (amendment 1)
        ok_ = phys_tick()
        rules._damage_tick()
        return ok_
    ok = True
    for _ in range(int(round(RUN_IN["settle_s"] / scene.tick_s))):
        ok = tick() and ok
    m.dof_frictionloss[dv.p["C"]["dof"]] = 0.0
    m.dof_frictionloss[dv.p["WP"]["dof"]] = float(scene.prm["drag_WP"])
    m.dof_frictionloss[dv.p["ALT"]["dof"]] = float(scene.prm["drag_ALT"])
    kv = RUN_IN["kv"]
    m.actuator_gainprm[scene.crank_act, 0] = kv
    m.actuator_biasprm[scene.crank_act, 2] = -kv
    q0 = {k: float(d.qpos[P["qadr"]]) for k, P in dv.p.items()}
    t0 = float(d.time)
    speed = RUN_IN["speed_rev_s"] * 2 * math.pi
    # the crank's rotation as drawn (+1 counter-clockwise); the spin hinges' axis is E -y, so +q is counter-clockwise
    sense = int(dr.VARIANTS[scene.variant].get("crank_sense", 1))
    nt = int(round((RUN_IN["duration_s"] + RUN_IN["ramp_s"]) / scene.tick_s))
    for _ in range(nt):
        t = float(d.time) - t0
        d.ctrl[scene.crank_act] = sense * speed * min(1.0, t / RUN_IN["ramp_s"])
        if not tick():
            ok = False
            break
    m.actuator_gainprm[scene.crank_act, 0] = 0.0
    m.actuator_biasprm[scene.crank_act, 2] = 0.0
    d.ctrl[scene.crank_act] = 0.0
    q1 = {k: float(d.qpos[P["qadr"]]) for k, P in dv.p.items()}
    dphi = {k: q1[k] - q0[k] for k in q0}
    t = dr.BELT["thick"]
    rhoC = dv.p["C"]["r"] + 0.5 * t
    slip = {}
    for k in ("WP", "ALT"):
        exp = dphi["C"] * rhoC / (dv.p[k]["r"] + 0.5 * t)
        slip[k] = 1.0 - dphi[k] / exp if abs(exp) > 1e-6 else 1.0
    ins = dr.inspect_drive(m, d, dv, scene.belt, scene.E_pos, scene.R_E, scene.variant)
    return {"ok": bool(ok and not scene.physics_fault), "slip": slip, "inspect": ins, "dphi_C": dphi["C"]}


def score_engine(result, stopped, damage, window):
    """(score, items) from a run-in result, the protective stop and damage flags, and the window (rad)."""
    ins = result["inspect"]
    th = math.radians(ins["tensioner"]["theta_deg"])
    in_window = bool(window[0] <= th <= window[1])
    pulleys = {k: bool(ins[k]["ok"]) for k in ("C", "WP", "ALT", "T")}
    slip_ok = bool(max(abs(result["slip"]["WP"]), abs(result["slip"]["ALT"])) <= RUN_IN["slip_max"])
    idlers = {k: bool(ins[k]["ok"]) for k in ins if k not in pulleys and isinstance(ins[k], dict) and "wind" in ins[k]}
    g = (not stopped) and (damage is None) and in_window and all(idlers.values())
    q = (sum(pulleys.values()) + (1 if slip_ok else 0)) / 5.0
    items = dict(pulleys, slip=slip_ok, in_window=in_window, no_stop=not stopped, no_damage=damage is None)
    items.update(idlers)
    return (float(q) if g else 0.0), items

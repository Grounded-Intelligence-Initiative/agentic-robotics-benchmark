"""alelab plugin: the hidden appliance units (the same robot environment as the agent's dev sim,
appliance_env.py, and the same episodes and requests, episode.py).

Instance file (environment/setup/ref/units.json): {unit id: {model, kind, dir}}; each unit's model files
(ref/units/<name>/) are read into memory here, before the worker drops privileges. Lab instances = the
scored units. Every trial starts from a fresh 64-bit seed from OS entropy (placement, initial control
states, request queue, sensor noise). Replies carry the observation only (images, proprioception, wrench,
the open request), never the units' state or any credit; an episode ends when its last request closes.

Images: appliance_env.py draws them with kitchen_render.py beside it (imported with appliance_env, at module load,
before the worker drops privileges; the room bundle in /opt/task-assets/payload/render is world-readable). The
worker's start-up validation resets every unit, so the renderer starts (its first image takes several seconds) and
is proven before the lab serves; a renderer that cannot start fails the lab's start. A renderer failure later, in a
reset or a step (appliance_env.RenderError), is the lab's own error: the server answers `internal` and logs the
traceback lab-side; it is never reported as anything the agent's action caused.

The graded episode (graded_reset; the lab chooses it at the start from the random order of the fixed hidden set in
environment/setup/online/order.json: the first entry on a unit no lab trial used) runs exactly as the offline grader
runs an episode of that set, and is scored as it scores one (the mean request credit, 0 after an unstable
simulation). With the oracle token (lab.json "graded" oracle_token_sha256) every graded observation also carries the
unit's truth, as the offline grader gives a tokened submission; a renderer failure in the graded episode restarts
the worker, which ends it.
"""

import os
import secrets

import appliance_env as E
import episode as EP

_U8 = "u8[*,*,3]"
TEXTS = EP.request_texts()            # every text an episode can open (episode.py), and ""
OBS_SCHEMA = {
    "t": "int",
    "unit": "enum:",
    "q": "f64[7]",
    "dq": "f64[7]",
    "ee_pos": "f64[3]",
    "ee_quat": "f64[4]",
    "gripper": "float",
    "wrench": "f64[6]",
    "T_world_wrist": "f64[4,4]",
    "T_world_left": "f64[4,4]",
    "T_world_right": "f64[4,4]",
    "K_left": "f64[3,3]",
    "K_right": "f64[3,3]",
    "K_wrist": "f64[3,3]",
    "left": _U8,
    "right": _U8,
    "wrist": _U8,
    "request_id": "int",
    "request_text": "enum:" + "|".join(TEXTS),
    "request_deadline_t": "int",
}
MESSAGES = {
    "action": "action must be a list of 8 finite numbers (6 motion values, grip, camera mask 0-7)",
}


def _schema(ids):
    s = dict(OBS_SCHEMA)
    s["unit"] = "enum:" + "|".join(ids)
    return {"info": {"control_dt": "float", "request_deadline_steps": "int", "compound_deadline_steps": "int"},
            "reset": s, "step": s}


class _Episode(object):
    def __init__(self, ep, uid):
        self.ep, self.uid = ep, uid

    def obs(self, obs):
        obs = dict(obs)
        obs["unit"] = self.uid
        for cam in ("left", "right", "wrist"):
            if cam not in obs:                      # an image the action did not ask for: empty
                obs[cam] = E.np.zeros((0, 0, 3), E.np.uint8)
        return obs

    def step(self, action):
        obs, done, _info = self.ep.step(action)
        if self.ep.env.unstable():
            done = True
        return self.obs(obs), bool(done)


class _GradedEpisode(_Episode):
    """The graded episode, run as the offline grader (tests/verify.py on main) runs an episode of its fixed hidden set:
    Episode(ApplianceEnv(spec, None, render=True), seed). It ends when its last request closes; an unstable simulation
    ends it and scores 0."""

    def __init__(self, ep, uid):
        _Episode.__init__(self, ep, uid)
        self.fault = False
        self.done = False

    def step(self, action):
        obs, done, _info = self.ep.step(action)
        if self.ep.env.unstable():
            self.fault = True
            done = True
        self.done = bool(done)
        return self.obs(obs), self.done

    def result(self):
        """The grader's score of one episode were it to end now: the mean credit of its requests (episode.py), 0 after
        an unstable simulation. Ended before its last request closed, the episode is scored as if it ended at that
        moment, with nothing simulated afterwards: the open request closes then (reached if its state has held for
        the last REACH_HOLD steps; completed only if it already was) and a request not yet opened earns 0, out of
        K_MAX requests. The sub-metrics are the grader's for one episode, and each request's outcome."""
        ep = self.ep
        reqs = []
        for r in ep.requests:
            closed = r["t_close"] is not None
            reached = bool(r["reached"]) if closed else r["run_reach"] >= EP.REACH_HOLD
            completed = bool(r["completed"])
            credit = 0.5 * reached + 0.5 * completed
            reqs.append({"id": r["id"], "what": r["what"], "target": r["target"], "t_open": r["t_open"],
                         "t_close": r["t_close"], "closed": closed, "reached": bool(reached),
                         "completed": completed, "credit": credit,
                         "outcome": "completed" if completed else ("reached" if reached else "missed")})
        n = len(reqs) if self.done else max(EP.K_MAX, len(reqs))
        score = 0.0 if (self.fault or not n) else float(sum(q["credit"] for q in reqs) / n)
        metrics = {"score_" + self.uid: score, "sim_faults": 1.0 if self.fault else 0.0,
                   "requests_opened": float(len(reqs)),
                   "requests_reached": float(sum(1 for q in reqs if q["reached"])),
                   "requests_completed": float(sum(1 for q in reqs if q["completed"]))}
        if not self.fault:
            per = {}
            for q in reqs:
                per.setdefault(q["what"], []).append(q["credit"])
            for what, vals in sorted(per.items()):
                metrics["request_" + what] = sum(vals) / len(vals)
        return {"score": score, "metrics": metrics,
                "detail": {"t": int(ep.env.t), "unstable": self.fault, "done": self.done, "requests": reqs}}

    def finish(self):
        return self.result()

    def truth(self):
        """The oracle token's channel (as tests/verify.py on main gives a tokened submission): the unit's placement
        jitter, its joint values and the microwave's flag."""
        return self.ep.env.truth()

    def close(self):
        self.ep.env.close()


class ApplianceLab(object):
    def __init__(self, instances, api):
        if E._KR is None:                           # the renderer must be importable before the privilege drop
            raise E.RenderError(E._KR_ERROR)
        self.api = api
        base = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "ref")
        self.specs = {}
        for uid, cfg in sorted(instances.items()):
            if uid.startswith("_"):
                continue
            files = E.read_unit_files(os.path.join(base, cfg["dir"]))
            self.specs[uid] = E.unit_spec(cfg["model"], files=files, quirks=cfg.get("quirks"))
        self.SCHEMA = _schema(sorted(self.specs))
        self.MESSAGES = MESSAGES

    def instance_ids(self):
        return sorted(self.specs)

    def info(self):
        return {"control_dt": E.CONTROL_DT, "request_deadline_steps": EP.DEADLINE,
                "compound_deadline_steps": EP.DEADLINE_COMPOUND}

    def validate_action(self, action):
        # depends only on the request and public constants (never on the instance or hidden state)
        if not isinstance(action, list) or len(action) != 8:
            raise self.api.BadRequest("action")
        out = []
        for v in action:
            if isinstance(v, bool) or not isinstance(v, (int, float)) or v != v or v in (float("inf"), float("-inf")):
                raise self.api.BadRequest("action")
            out.append(float(v))
        if not (0 <= out[7] <= 7 and out[7].is_integer()):
            raise self.api.BadRequest("action")
        return out

    def reset(self, instance):
        ep = EP.Episode(E.ApplianceEnv(self.specs[instance], None, render=True), secrets.randbits(64))
        obs = ep.reset()
        e = _Episode(ep, instance)
        return e, e.obs(obs)

    def graded_reset(self, instance, seed):
        """The graded episode: the hidden unit `instance` at its fixed 64-bit `seed` from the hidden set."""
        ep = EP.Episode(E.ApplianceEnv(self.specs[instance], None, render=True), seed)
        obs = ep.reset()
        g = _GradedEpisode(ep, instance)
        return g, g.obs(obs)


def make_plugin(instances, api):
    return ApplianceLab(instances, api)

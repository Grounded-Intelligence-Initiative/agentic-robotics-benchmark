"""Server hot-swap maintenance: an episode, its end and its credit.

An episode is one work order on one unit: replace the failed drive(s) and the failed fan. It ends at the first of:
- EPISODE_STEPS control steps;
- an action with "done": true (after that step);
- an irreversible incident (srv_sim.World.lost: a healthy drive unseated, a part dropped, a lever broken, an
  overheat, a robot striking the server);
- an unstable simulation.

The credit is all or nothing: 1 when, at the end, every failed drive's slot holds a spare caddy seated and latched
(lever closed on its latch), every other slot still holds its own caddy seated and latched, every fan bay holds a
seated fan (the failed fan's bay the spare fan), every removed part lies in the failed-parts bin and nothing
irreversible happened; else 0. An untouched run scores 0 (the failed parts stay in).
"""

import srv_sim as S

EPISODE_STEPS = 6000            # 10 simulated minutes


class Episode:
    def __init__(self, env, seed):
        self.env, self.seed = env, seed

    def reset(self):
        self.done = False
        self.end = None
        return self.env.reset(self.seed)

    def step(self, action):
        obs = self.env.step(action)
        w = self.env.w
        if self.env.unstable():
            self.done, self.end = True, "unstable"
        elif w.lost:
            self.done, self.end = True, "incident"
        elif self.env.done_flag:
            self.done, self.end = True, "done"
        elif self.env.t >= EPISODE_STEPS:
            self.done, self.end = True, "time"
        return obs, self.done, {}

    def score(self):
        env, w = self.env, self.env.w
        unstable = env.unstable()
        st = w.status()
        ok = (not unstable and st["lost"] is None and all(st["slots_ok"]) and all(st["fans_ok"])
              and all(st["failed_in_bin"]) and st["fan_in_bin"])
        return {"credit": 1.0 if ok else 0.0, "end": self.end, "steps": int(env.t), "unstable": unstable,
                "incident": st["lost"], "slots_ok": sum(st["slots_ok"]), "fans_ok": sum(st["fans_ok"]),
                "failed_in_bin": [bool(x) for x in st["failed_in_bin"]], "fan_in_bin": bool(st["fan_in_bin"]),
                "failed_slots": list(w.episode["failed_slots"]), "failed_fan": int(w.episode["failed_fan"])}


def hold_action():
    return {"l": [0.0] * 6 + [-1.0], "r": [0.0] * 6 + [-1.0], "cams": 0}

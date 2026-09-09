"""Analytic reference artifact. This file is never staged with public setup material."""

import json
import math
import sys


class BallSimulator:
    def reset(self, config, state):
        self.g = float(config["gravity"])
        self.mass = float(config["mass"])
        self.e = float(config["restitution"])
        self.floor = (None if config["plate_y"] is None else
                      float(config["plate_y"]) + float(config["radius"]))
        self.x, self.y = map(float, state["position"])
        self.vx, self.vy = map(float, state["velocity"])
        self.time = 0.0
        return self.state()

    def state(self):
        return {"position": [self.x, self.y], "velocity": [self.vx, self.vy],
                "time": self.time}

    def bounce_tail(self, duration):
        # At an impact, subsequent flight times form a geometric series.
        speed = self.vy
        period = 2.0 * speed / self.g
        if self.e == 1.0:
            residual = duration % period
        else:
            log_e = math.log(self.e)
            stop = max(1, math.ceil(math.log(1e-7 / speed) / log_e))
            limit = period / (1.0 - self.e)

            def elapsed(n):
                return -limit * math.expm1(n * log_e)

            if duration >= elapsed(stop):
                self.y, self.vy = self.floor, 0.0
                return
            n = max(0, min(stop-1, int(math.log1p(-duration / limit) / log_e)))
            while n > 0 and elapsed(n) > duration:
                n -= 1
            while n + 1 < stop and elapsed(n+1) <= duration:
                n += 1
            residual = max(0.0, duration - elapsed(n))
            speed *= self.e**n
        self.y = self.floor + speed*residual - 0.5*self.g*residual**2
        self.vy = speed - self.g*residual

    def step(self, dt, impulse):
        dt = float(dt)
        self.vx += float(impulse[0]) / self.mass
        self.vy += float(impulse[1]) / self.mass
        self.x += self.vx * dt
        self.time += dt
        if self.floor is None:
            self.y += self.vy * dt - 0.5 * self.g * dt * dt
            self.vy -= self.g * dt
            return self.state()

        remaining = dt
        for _ in range(10000):
            height = max(0.0, self.y - self.floor)
            if height <= 1e-12:
                height = 0.0
                self.y = self.floor
            if height == 0.0 and self.vy <= 0.0:
                self.y = self.floor
                if self.vy < 0.0:
                    self.vy = -self.e * self.vy
                    if self.g > 0.0 and self.vy <= 1e-7:
                        self.vy = 0.0
                if self.vy == 0.0:
                    break
            if remaining == 0.0:
                break

            if self.g == 0.0:
                hit = height / -self.vy if self.vy < 0.0 else math.inf
            else:
                speed = math.sqrt(self.vy * self.vy + 2.0 * self.g * height)
                # The alternate root avoids subtracting almost equal numbers on descent.
                hit = (2.0 * height / (speed - self.vy) if self.vy < 0.0
                       else (self.vy + speed) / self.g)
            if hit > remaining:
                self.y += self.vy * remaining - 0.5 * self.g * remaining**2
                self.vy -= self.g * remaining
                break
            self.vy = -self.e * (self.vy - self.g * hit)
            self.y = self.floor
            remaining = max(0.0, remaining - hit)
            if self.g > 0.0 and self.vy <= 1e-7:
                self.vy = 0.0
                break
            if self.g > 0.0 and remaining > 0.0:
                self.bounce_tail(remaining)
                break
        else:
            raise RuntimeError("unexpected collision count")
        return self.state()


def main():
    sim = BallSimulator()
    print("READY", flush=True)
    for line in sys.stdin:
        req = json.loads(line)
        if req["type"] == "close":
            print(json.dumps({"ok": True}), flush=True)
            return
        if req["type"] == "reset":
            state = sim.reset(req["config"], req["state"])
        elif req["type"] == "step":
            state = sim.step(req["dt"], req["impulse"])
        else:
            raise ValueError("unknown request")
        print(json.dumps({"state": state}, allow_nan=False), flush=True)


if __name__ == "__main__":
    main()

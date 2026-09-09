"""Independent 50-digit Decimal reference; does not import the submitted simulator."""

from decimal import Decimal, localcontext


def number(value):
    return Decimal(str(value))


class Reference:
    def reset(self, config, state):
        self.g = number(config["gravity"])
        self.mass = number(config["mass"])
        self.e = number(config["restitution"])
        self.floor = (None if config["plate_y"] is None else
                      number(config["plate_y"]) + number(config["radius"]))
        self.p = [number(x) for x in state["position"]]
        self.v = [number(x) for x in state["velocity"]]
        self.time = Decimal(0)
        return self.state()

    def state(self):
        return {"position": list(map(float, self.p)), "velocity": list(map(float, self.v)),
                "time": float(self.time)}

    def bounce_tail(self, duration):
        initial = self.v[1]
        period = 2*initial/self.g
        if self.e == 1:
            remainder = duration % period
            speed = initial
        else:
            limit = period/(1-self.e)
            if duration >= limit:
                self.p[1], self.v[1] = self.floor, Decimal(0)
                return

            def elapsed(count):
                return period*(1-self.e**count)/(1-self.e)

            # Integer bisection with Decimal arithmetic, independent of oracle logs.
            low, high = 0, 1
            while elapsed(high) <= duration and initial*self.e**high > number("1e-7"):
                low, high = high, high*2
            if elapsed(high) <= duration:
                self.p[1], self.v[1] = self.floor, Decimal(0)
                return
            while high-low > 1:
                mid = (low+high)//2
                if elapsed(mid) <= duration:
                    low = mid
                else:
                    high = mid
            speed = initial*self.e**low
            if speed <= number("1e-7"):
                self.p[1], self.v[1] = self.floor, Decimal(0)
                return
            remainder = duration-elapsed(low)
        self.p[1] = self.floor + speed*remainder - self.g*remainder**2/2
        self.v[1] = speed - self.g*remainder

    def step(self, dt, impulse):
        with localcontext() as ctx:
            ctx.prec = 50
            duration = number(dt)
            self.time += duration
            for k in range(2):
                self.v[k] += number(impulse[k]) / self.mass
            self.p[0] += self.v[0] * duration
            if self.floor is None:
                self.p[1] += self.v[1]*duration - self.g*duration**2/2
                self.v[1] -= self.g*duration
                return self.state()

            remaining = duration
            impacts = 0
            while True:
                gap = max(Decimal(0), self.p[1] - self.floor)
                if gap <= number("1e-12"):
                    gap = Decimal(0)
                    self.p[1] = self.floor
                if gap == 0 and self.v[1] < 0:
                    self.v[1] *= -self.e
                    if self.g > 0 and self.v[1] <= number("1e-7"):
                        self.v[1] = Decimal(0)
                if gap == 0 and self.v[1] == 0:
                    self.p[1] = self.floor
                    return self.state()
                if remaining == 0:
                    return self.state()
                if self.g:
                    collision = (self.v[1] + (self.v[1]**2 + 2*self.g*gap).sqrt())/self.g
                elif self.v[1] < 0:
                    collision = gap / -self.v[1]
                else:
                    collision = Decimal("Infinity")
                if collision > remaining:
                    self.p[1] += self.v[1]*remaining - self.g*remaining**2/2
                    self.v[1] -= self.g*remaining
                    return self.state()
                self.v[1] -= self.g*collision
                self.p[1] = self.floor
                self.v[1] *= -self.e
                remaining -= collision
                impacts += 1
                if self.g > 0 and self.v[1] <= number("1e-7"):
                    self.v[1] = Decimal(0)
                elif self.g > 0 and remaining > 0:
                    self.bounce_tail(remaining)
                    return self.state()
                if impacts > 10000:
                    raise RuntimeError("reference collision count exceeded")

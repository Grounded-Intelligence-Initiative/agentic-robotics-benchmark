#!/usr/bin/env python3
"""Observation-driven pick and place policy for NutAssemblySquare.

The simulator state is deliberately not accessed.  Only positions and quaternions
in the line protocol are used; the small object-pose bias cancels when following
the object from one step to the next.
"""
import json
import math
import sys


def clamp(x, lo=-1., hi=1.):
    return max(lo, min(hi, x))


def angle(x):
    return (x + math.pi) % (2 * math.pi) - math.pi


def yaw(q):
    # Robosuite object and end-effector quaternions are xyzw.
    x, y, z, w = map(float, q)
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


class Policy:
    def __init__(self):
        self.reset()

    def reset(self):
        self.stage = 'hover'
        self.stage_start = 0
        self.last_t = -1
        self.rest_z = None
        self.grasp_z = None
        self.grasp_eef_z = None
        self.attempt = 0
        self.yaw_sign = 1
        self.last_yaw = None
        self.last_rz = 0
        self.last_action = [0.] * 6 + [-1.]

    def set_stage(self, stage, t):
        self.stage = stage
        self.stage_start = t

    def step(self, t, obs):
        e = list(map(float, obs['eef_pos']))
        nut = list(map(float, obs['nut_pos']))
        handle = list(map(float, obs['handle_pos']))
        peg = list(map(float, obs['peg_pos']))
        if self.rest_z is None:
            self.rest_z = nut[2]
        age = t - self.stage_start

        # The handle's radial direction also gives a stable grasp orientation.
        dx, dy = handle[0] - nut[0], handle[1] - nut[1]
        handle_yaw = math.atan2(dy, dx)
        eef_yaw = yaw(obs['eef_quat'])
        if self.stage in ('hover', 'descend', 'close', 'lift', 'retry'):
            # The Panda fingers open along local y; its local x follows the
            # handle so that the fingers straddle the handle width.
            yaw_error = angle(handle_yaw - eef_yaw)
            # The gripper can approach from either direction along its x-axis.
            if yaw_error > math.pi / 2:
                yaw_error -= math.pi
            if yaw_error < -math.pi / 2:
                yaw_error += math.pi
        else:
            # The square hole is invariant under quarter turns.
            nut_yaw = yaw(obs['nut_quat'])
            yaw_error = angle(-nut_yaw)
            yaw_error = (yaw_error + math.pi / 4) % (math.pi / 2) - math.pi / 4
            # Determine the sign of the rotation controller from observed
            # object motion; the world frame of OSC yaw varies by controller.
            if self.last_yaw is not None and abs(self.last_rz) > .25:
                change = angle(nut_yaw - self.last_yaw)
                if abs(change) > .001 and change * self.last_rz * self.yaw_sign < -.0005:
                    self.yaw_sign *= -1
            self.last_yaw = nut_yaw

        # Expected command scale is 5 cm in translation and 0.5 rad in
        # rotation.  Moderate gains and clipping avoid overshoot at contact.
        def action_to(target, grip, rot=True, velocity=.95):
            a = [clamp((target[i] - e[i]) / .037, -velocity, velocity) for i in range(3)]
            a += [0., 0., clamp(self.yaw_sign * yaw_error / .34, -.65, .65) if rot else 0.]
            a += [grip]
            self.last_rz = a[5]
            self.last_action = a
            return a

        def follow_nut(target, grip=1., rot=True, velocity=.9):
            # Convert object error to end-effector target at every step; this
            # corrects any slip in the grasp as well as a fixed vision offset.
            return action_to([e[i] + target[i] - nut[i] for i in range(3)], grip, rot, velocity)

        if self.stage == 'hover':
            target = [handle[0], handle[1], handle[2] + .13]
            if abs(e[0] - target[0]) < .014 and abs(e[1] - target[1]) < .014 and abs(e[2] - target[2]) < .018 and abs(yaw_error) < .16:
                self.set_stage('descend', t)
            elif age > 60:
                self.set_stage('descend', t)
            return action_to(target, -1.)

        if self.stage == 'descend':
            # Start with the grasp site slightly above the handle center to
            # keep the fingers clear of the table.
            z_adjust = [0., -.012, .012][min(self.attempt, 2)]
            target = [handle[0], handle[1], handle[2] + .012 + z_adjust]
            if (abs(e[0] - target[0]) < .009 and abs(e[1] - target[1]) < .009 and abs(e[2] - target[2]) < .012) or age > 45:
                self.grasp_z = nut[2]
                self.grasp_eef_z = e[2]
                self.set_stage('close', t)
            return action_to(target, -1., velocity=.7)

        if self.stage == 'close':
            if age >= 9:
                self.set_stage('lift', t)
            return action_to(e, 1., rot=False, velocity=.5)

        if self.stage == 'lift':
            target = [e[0], e[1], self.grasp_eef_z + .16]
            if nut[2] > self.rest_z + .055 and e[2] > self.grasp_eef_z + .09:
                self.set_stage('transport', t)
            elif age > 38:
                if self.attempt < 2:
                    self.attempt += 1
                    self.set_stage('retry', t)
                else:
                    self.set_stage('transport', t)
            return action_to(target, 1., rot=False, velocity=.85)

        if self.stage == 'retry':
            target = [handle[0], handle[1], handle[2] + .13]
            if abs(e[2] - target[2]) < .02 or age > 25:
                self.set_stage('descend', t)
            return action_to(target, -1., rot=False)

        if self.stage == 'transport':
            target = [peg[0], peg[1], max(self.rest_z + .15, peg[2] + .15)]
            if math.hypot(nut[0]-target[0], nut[1]-target[1]) < .012 and abs(nut[2]-target[2]) < .02:
                self.set_stage('align', t)
            elif age > 60:
                self.set_stage('align', t)
            return follow_nut(target, rot=False)

        if self.stage == 'align':
            target = [peg[0], peg[1], max(self.rest_z + .15, peg[2] + .15)]
            if abs(yaw_error) < .075 and math.hypot(nut[0]-target[0], nut[1]-target[1]) < .012 and age > 5:
                self.set_stage('place', t)
            elif age > 30:
                self.set_stage('place', t)
            return follow_nut(target, velocity=.65)

        if self.stage == 'place':
            # A small positive height leaves the nut on the peg / table, and
            # lets the free object settle naturally after opening.
            target = [peg[0], peg[1], self.rest_z + .018]
            if (nut[2] < target[2] + .016 and math.hypot(nut[0]-peg[0], nut[1]-peg[1]) < .012 and age > 8) or age > 55:
                self.set_stage('release', t)
            return follow_nut(target, velocity=.55)

        if self.stage == 'release':
            if age >= 10:
                self.set_stage('retreat', t)
            return action_to(e, -1., rot=False, velocity=.3)

        # Leave the nut alone.  The native success check needs ten steps.
        target = [e[0], e[1], self.rest_z + .20]
        return action_to(target, -1., rot=False, velocity=.45)


def main():
    policy = Policy()
    print('READY', flush=True)
    for line in sys.stdin:
        try:
            request = json.loads(line)
            typ = request.get('type')
            if typ == 'reset':
                policy.reset()
                reply = {'ok': True}
            elif typ == 'close':
                reply = {'ok': True}
                print(json.dumps(reply), flush=True)
                break
            elif typ == 'act':
                reply = {'action': policy.step(int(request['t']), request['obs'])}
            else:
                reply = {'error': 'unknown request'}
            print(json.dumps(reply, separators=(',', ':')), flush=True)
        except Exception as exc:
            print('controller error: ' + repr(exc), file=sys.stderr, flush=True)
            print(json.dumps({'action': policy.last_action}), flush=True)


if __name__ == '__main__':
    main()

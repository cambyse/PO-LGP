import numpy as np
import random


def get_state(q, states):
    for i, k in enumerate(states):
        if q < k:
            return i
    return i+1


def sgn(x):
    if x < 0:
        return -1
    else:
        return 1


class Joint:
    def __init__(self, states, dampings, noise):
        self.vel = 0
        self.q = 0
        self.states = states
        self.dampings = dampings
        self.max_limit = None
        self.min_limit = None
        self.direction = 1
        self.locked = False
        self.noise = noise

    def add_force(self, f):
        if not self.locked:
            state = get_state(self.q, self.states)
            damping = self.dampings[state]
            self.vel += f/damping

    def lock(self):
        self.vel = 0
        self.locked = True

    def unlock(self):
        self.locked = False

    def is_locked(self):
        return self.locked

    def step(self, dt):
        if (self.locked):
            return (self.q, self.vel)

        self.q = self.q + self.vel * dt
        if ((self.max_limit and self.q > self.max_limit) or
                (self.min_limit and self.q < self.min_limit)):
            self.direction = -self.direction
        state = get_state(self.q, self.states)
        damping = self.dampings[state]
        tmp_sqr_vel = max(self.vel**2 - abs(1 * damping * self.vel * dt),
                          0)
        self.vel = self.direction * np.sqrt(tmp_sqr_vel)

    def get_vel(self):
        return random.gauss(self.vel, self.noise['vel'])

    def get_q(self):
        return random.gauss(self.q, self.noise['q'])


class World:
    def __init__(self, joints):
        self.joints = joints
        self.listeners = []

    def step(self, dt):
        for joint in self.joints:
            joint.step(dt)
        self._inform_listeners(dt)

    def register(self, listener):
        self.listeners.append(listener)

    def get_joint(self, num):
        return self.joints[num]

    def _inform_listeners(self, dt):
        for listener in self.listeners:
            listener.step(dt)


class Controller:
    def __init__(self, world):
        self.world = world
        self.world.register(self)
        self.goal_pos = None
        self.joint_num = None
        self.q_eps = 10e-3
        self.v_eps = 10e-3
        self.done = True
        self.q = 0
        self.v = 0
        self.kp = 1
        self.kd = .7
        self.ki = 0
        self.i = []
        self.applied_forces = []
        self.desired_forces = []
        self.max_force = 10
        for i in range(len(self.world.joints)):
            self.i.append(0)

    def move_to(self, joint_num, pos):
        self.goal_pos = pos
        self.joint_num = joint_num
        if not self._is_done():
            self.done = False
        else:
            self.done = True
            self.joint_num = None

    def step(self, dt):
        if self.joint_num is None:
            return

        joint = self.world.get_joint(self.joint_num)

        self.q = joint.get_q()
        self.v = joint.get_vel()

        if self._is_done():
            self.done = True
            self.joint_num = None
            return

        desired_force = self._pid_control()
        self.desired_forces.append(desired_force)

        sign = sgn(desired_force)
        applied_force = sign * min(abs(desired_force), self.max_force)
        joint.add_force(applied_force)
        self.applied_forces.append(applied_force)

    def _pid_control(self):
        self.i[self.joint_num] += (self.goal_pos - self.q)
        force = self.kp * (self.goal_pos - self.q)\
            + self.kd * (-self.v)\
            + self.ki * (self.i[self.joint_num])
        return force

    def _is_done(self):
        if self.joint_num is None:
            return True

        if (abs(self.q - self.goal_pos) < self.q_eps and
                abs(self.v) < self.v_eps):
            return True

        return False


class Recorder:
    def __init__(self, world):
        self.world = world
        self.world.register(self)
        self.qs = []
        self.vs = []
        self.accs = []

    def step(self, dt):
        qs = np.ndarray(len(self.world.joints))
        vs = np.ndarray(len(self.world.joints))
        accs = np.ndarray(len(self.world.joints))
        for i, joint in enumerate(self.world.joints):
            if len(vs) > 0:
                a = (joint.get_vel() - vs[-1]) * dt
            else:
                a = 0
            qs[i] = joint.get_q()
            vs[i] = joint.get_vel()
            accs[i] = a
        self.qs.append(qs)
        self.vs.append(vs)
        self.accs.append(accs)


class Locker:
    def __init__(self, world):
        self.world = world
        self.world.register(self)
        self.locker = locker
        self.locked = locked
        self.lower = lower
        self.upper = upper

    def step(self, dt):
        if self.locker.q > self.lower and self.locker.q < self.upper:
            if not self.locked.is_locked():
                self.locked.lock()
        else:
            if self.locked.is_locked():
                self.locked.unlock()

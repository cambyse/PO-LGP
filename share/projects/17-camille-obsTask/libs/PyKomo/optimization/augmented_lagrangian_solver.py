import numpy as np
from newton import NewtonFunction, Newton

class Lagrangian(NewtonFunction):
    def __init__(self, pb, mu=0.0, lambda_h=0.0, lambda_g=0.0):
        self.pb = pb
        self.mu = mu
        self.lambda_h = lambda_h
        self.lambda_g = lambda_g

    def value(self, x):
        f = self.pb.f.value(x)

        if self.pb.h:
            h = self.pb.h.value(x)
            f += self.mu * h ** 2 + self.lambda_h * h

        if self.pb.g:
            g = self.pb.g.value(x)
            activity = g >= 0 or self.lambda_g > 0
            if not activity:
                g = 0
            f += self.mu * g ** 2 + self.lambda_g * g

        return f

    def gradient(self, x):
        J = self.pb.f.gradient(x)

        if self.pb.h:
            h = self.pb.h.value(x)
            Jh = self.pb.h.gradient(x)
            Jb = 2 * np.dot(Jh.T, h)

            J += self.mu * Jb + self.lambda_h * Jh

        if self.pb.g:
            g = self.pb.g.value(x)
            Jg = self.pb.g.gradient(x)
            activity = g >= 0 or self.lambda_g > 0
            if not activity:
                g = 0
                Jg = np.zeros(Jg.shape)
            Jb = 2 * np.dot(Jg.T, g)
            J += self.mu * Jb + self.lambda_g * Jg

        return J

    def hessian(self, x):
        H = self.pb.f.hessian(x)  # hessian of f

        if self.pb.h:
            Jh = self.pb.h.gradient(x)
            _Jh = np.array([Jh])
            Hb = 2 * _Jh.T * _Jh  # pseudo hessian of the barrier
            Hh = self.pb.h.hessian(x)

            assert H.shape == Hb.shape, "wrong hessian shapes"
            assert H.shape == Hh.shape, "wrong hessian shapes"

            H += self.mu * Hb + self.lambda_h * Hh

        if self.pb.g:
            g = self.pb.g.value(x)
            Jg = self.pb.g.gradient(x)
            activity = g >= 0 or self.lambda_g > 0
            if not activity:
                g = 0
                Jg = np.zeros(Jg.shape)
            # Hb = 2 * np.dot(Jg.T, Jg) # pseudo hessian of the barrier
            _Jg = np.array([Jg])
            Hb = 2 * _Jg.T * _Jg  # pseudo hessian of the barrier
            Hg = self.pb.g.hessian(x)

            assert H.shape == Hb.shape, "wrong hessian shapes"
            assert H.shape == Hg.shape, "wrong hessian shapes"

            H += self.mu * Hb + self.lambda_g * Hg

        return H

class AugmentedLagrangianSolver:
    def __init__(self, pb):
        self.constrainedProblem = pb
        self.eps = 0.001 #max constraint violation
        self.mu = 1.0
        self.rho = 1.0 # how much we increase the square penalty at each cycle
        self.lambda_h = 0.0
        self.lambda_g = 0.0

    def run(self, x, observer=None):
        if observer:
            observer.new_aula_run(x)

        lagrangian = Lagrangian(self.constrainedProblem, mu=self.mu, lambda_h=self.lambda_h, lambda_g=self.lambda_g)
        gn = Newton(lagrangian)
        x = gn.run(x)
        h = self.constrainedProblem.h.value(x) if self.constrainedProblem.h else 0
        g = self.constrainedProblem.g.value(x) if self.constrainedProblem.g else 0

        i = 0
        while True:
            print("it={}, lambda_h={}, h={}, lambda_g={}, g={}".format(i, self.lambda_h, h, self.lambda_g, g))

            lagrangian = Lagrangian(self.constrainedProblem, mu=self.mu, lambda_h=self.lambda_h, lambda_g=self.lambda_g)
            solver = Newton(lagrangian)
            x = solver.run(x, observer=observer)
            h = self.constrainedProblem.h.value(x) if self.constrainedProblem.h else 0
            g = self.constrainedProblem.g.value(x) if self.constrainedProblem.g else 0

            self.lambda_h = self.lambda_h + 2 * self.mu * h
            self.lambda_g = self.lambda_g + 2 * self.mu * g
            self.mu *= self.rho

            if np.abs(h) < self.eps and g < self.eps:
                break

            i += 1

        return x
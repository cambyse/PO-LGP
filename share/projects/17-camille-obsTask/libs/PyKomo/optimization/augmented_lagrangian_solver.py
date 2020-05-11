import numpy as np
from newton import NewtonFunction, Newton

class AugmentedLagrangianSolver:
    def __init__(self, pb):
        self.constrainedProblem = pb
        self.eps = 0.001 #max constraint violation
        self.mu = 1.0
        self.rho = 2.0 # how much we increase the square penalty at each cycle
        self.lambda_h = 0.0
        self.lambda_g = 0.0

    @staticmethod
    def convert(pb, mu=0.0, lambda_h=0.0, lambda_g=0.0):
        class Augmented(NewtonFunction):
            def value(self, x):
                f = pb.f.value(x)

                if pb.h:
                    h = pb.h.value(x)
                    f += mu * h ** 2 + lambda_h * h

                if pb.g:
                    g = pb.g.value(x)
                    activity = g >= 0 or lambda_g > 0
                    if not activity:
                        g = 0
                    f += mu * g ** 2 + lambda_g * g

                return f

            def gradient(self, x):
                J = pb.f.gradient(x)

                if pb.h:
                    h = pb.h.value(x)
                    Jh = pb.h.gradient(x)
                    Jb = 2 * np.dot(Jh.T, h)

                    J += mu * Jb + lambda_h * Jh

                if pb.g:
                    g = pb.g.value(x)
                    Jg = pb.g.gradient(x)
                    activity = g >= 0 or lambda_g > 0
                    if not activity:
                        g = 0
                        Jg = np.zeros(Jg.shape)
                    Jb = 2 * np.dot(Jg.T, g)
                    J += mu * Jb + lambda_g * Jg

                return J

            def hessian(self, x):
                H = pb.f.hessian(x) # hessian of f

                if pb.h:
                    Jh = pb.h.gradient(x)
                    _Jh = np.array([Jh])
                    Hb = 2 * _Jh.T * _Jh # pseudo hessian of the barrier
                    Hh = pb.h.hessian(x)

                    assert H.shape == Hb.shape, "wrong hessian shapes"
                    assert H.shape == Hh.shape, "wrong hessian shapes"

                    H += mu * Hb + lambda_h * Hh

                if pb.g:
                    g = pb.g.value(x)
                    Jg = pb.g.gradient(x)
                    activity = g >= 0 or lambda_g > 0
                    if not activity:
                        g = 0
                        Jg = np.zeros(Jg.shape)
                    #Hb = 2 * np.dot(Jg.T, Jg) # pseudo hessian of the barrier
                    _Jg = np.array([Jg])
                    Hb = 2 * _Jg.T * _Jg # pseudo hessian of the barrier
                    Hg = pb.g.hessian(x)

                    assert H.shape == Hb.shape, "wrong hessian shapes"
                    assert H.shape == Hg.shape, "wrong hessian shapes"

                    H += mu * Hb + lambda_g * Hg

                return H

        return Augmented()

    def run(self, x):
        print("lambda_h={}".format(self.lambda_h))

        unconstrained = self.convert(self.constrainedProblem, mu=self.mu, lambda_h=self.lambda_h, lambda_g=self.lambda_g)
        gn = Newton(unconstrained)
        x = gn.run(x)
        h = self.constrainedProblem.h.value(x) if self.constrainedProblem.h else 0
        g = self.constrainedProblem.g.value(x) if self.constrainedProblem.g else 0

        i = 0
        while True:
            print("it={}, lambda_h={}, h={}, lambda_g={}, g={}".format(i, self.lambda_h, h, self.lambda_g, g))

            unconstrained = self.convert(self.constrainedProblem, mu=self.mu, lambda_h=self.lambda_h, lambda_g=self.lambda_g)
            solver = Newton(unconstrained)
            x = solver.run(x)
            h = self.constrainedProblem.h.value(x) if self.constrainedProblem.h else 0
            g = self.constrainedProblem.g.value(x) if self.constrainedProblem.g else 0

            self.lambda_h = self.lambda_h + 2 * self.mu * h
            self.lambda_g = self.lambda_g + 2 * self.mu * g
            self.mu *= self.rho

            if np.abs(h) < self.eps and g < self.eps:
                break

            i += 1

        return x
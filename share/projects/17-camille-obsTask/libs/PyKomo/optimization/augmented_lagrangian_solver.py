import numpy as np
from newton import NewtonFunction, Newton

class AugmentedLagrangianSolver:
    def __init__(self, pb):
        self.constrainedProblem = pb
        self.eps_h = 0.001 #max constraint violation
        self.mu = 1.0
        self.lambda_ = 0.0
        self.rho = 2.0 # how much we increase the sqzare penalty at each cycle

    @staticmethod
    def convert(pb, mu, lambda_):
        class Augmented(NewtonFunction):
            def value(self, x):
                f = pb.f.value(x)
                h = pb.h.value(x)
                return f + mu * h ** 2 + lambda_ * h

            def gradient(self, x):
                Jf = pb.f.gradient(x)
                h = pb.h.value(x)
                Jh = pb.h.gradient(x)
                Jb = 2 * np.dot(Jh.T, h)
                return Jf + mu * Jb + lambda_ * Jh

            def hessian(self, x):
                # NB: the hessian of the langrange term is assumed 0!
                Hf = pb.f.hessian(x) # hessian of f
                Jh = pb.h.gradient(x)
                Hb = 2 * np.dot(Jh.T, Jh) # pseudo hessian of the barrier
                return Hf + mu * Hb

        return Augmented()

    def run(self, x):
        print("lambda={}".format(self.lambda_))

        unconstrained = self.convert(self.constrainedProblem, self.mu, self.lambda_)
        gn = Newton(unconstrained)
        x = gn.run(x)
        h = self.constrainedProblem.h.value(x)

        while np.abs(h) > self.eps_h:
            self.lambda_ = self.lambda_ + 2 * self.mu * h
            self.mu *= self.rho

            print("lambda={}".format(self.lambda_))

            unconstrained = self.convert(self.constrainedProblem, self.mu, self.lambda_)
            gn = Newton(unconstrained)
            x = gn.run(x)
            h = self.constrainedProblem.h.value(x)

        return x
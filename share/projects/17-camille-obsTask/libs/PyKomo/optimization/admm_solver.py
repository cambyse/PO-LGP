import numpy as np
from augmented_lagrangian_solver import AugmentedLagrangianSolverEq
from newton import NewtonFunction, Newton
from optimization_problems import UnconstrainedQP, ConstrainedQP, ConstrainedProblem

class ADMMSolver:
    def __init__(self, pb):
        self.pb = pb
        self.y = 0
        self.rho = 1
        self.eps = 0.001 #max constraint violation

    @staticmethod
    def to_0(pb, xk, y, rho):
        class F(NewtonFunction):
            def value(self, x):
                delta = x - xk
                return pb.f0.value(x) + pb.f1.value(xk) + np.dot(y.T, delta) + 0.5 * rho * np.dot(delta.T, delta)

            def gradient(self, x):
                delta = x - xk
                return pb.f0.gradient(x) + y.T + rho * delta

            def hessian(self, x):
                h = pb.f0.hessian(x)
                Hb = np.identity(x.shape[0])
                return h + rho * Hb

        return F()

    @staticmethod
    def to_1(pb, x_fixed, y, rho):
        class F(NewtonFunction):
            def value(self, x):
                delta = x_fixed - x
                return pb.f0.value(x_fixed) + pb.f1.value(x) + np.dot(y.T, delta) + 0.5 * rho * np.dot(delta.T,
                                                                                                  delta)

            def gradient(self, x):
                delta = x_fixed - x
                return pb.f1.gradient(x) - y.T - rho * delta

            def hessian(self, x):
                h = pb.f1.hessian(x)
                Hb = np.identity(x.shape[0])
                return h + rho * Hb

        return F()

    def run(self, x):
        self.y = np.zeros(x.shape)

        while True:
            pb0 = self.to_0(self.pb, x, self.y, self.rho)
            assert pb0.checkGradients(x) and pb0.checkHessian(x)
            x0 = Newton(pb0).run(x)


            pb1 = self.to_1(self.pb, x0, self.y, self.rho)
            assert pb1.checkGradients(x0) and pb1.checkHessian(x0)
            x1 = Newton(pb1).run(x0)

            delta = x0 - x1
            self.y += self.rho * delta

            x = x1

            if np.abs(delta).max() < self.eps:
                break

        return x
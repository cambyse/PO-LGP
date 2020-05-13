import numpy as np
from newton import NewtonFunction, Newton
from optimization_problems import ConstrainedProblem
from admm_solver import ADMMLagrangian0, ADMMLagrangian1
from augmented_lagrangian_solver import Lagrangian

class DecentralizedAugmentedLagrangianSolver:
    def __init__(self, pb):
        self.pb = pb
        # common
        self.eps = 0.001 #max constraint violation
        self.mu = 1.0  # square penalty weight
        self.nu = 1.0  # how much we increase the square penalty at each cycle
        # admm
        self.y = 0     # lagrange term
        # aula
        self.lambda_h = 0.0
        self.lambda_g = 0.0

    def run(self, x, observer=None):
        self.y = np.zeros(x.shape)

        i = 0
        x0 = x
        x1 = x
        while True:
            unconstrained_0 = ADMMLagrangian0(Lagrangian(pb=self.pb.pb0, lambda_h=self.lambda_h, lambda_g=self.lambda_g, mu=self.mu), xk=x1, y=self.y, mu=self.mu)
            assert unconstrained_0.checkGradients(x0)
            assert unconstrained_0.checkHessian(x0)
            assert unconstrained_0.checkGradients(x1)
            assert unconstrained_0.checkHessian(x1)
            x0 = Newton(unconstrained_0).run(x1, observer=observer)

            unconstrained_1 = ADMMLagrangian1(Lagrangian(pb=self.pb.pb1, lambda_h=self.lambda_h, lambda_g=self.lambda_g, mu=self.mu), xk=x0, y=self.y, mu=self.mu)
            assert unconstrained_1.checkGradients(x0)
            assert unconstrained_1.checkHessian(x0)
            assert unconstrained_1.checkGradients(x1)
            assert unconstrained_1.checkHessian(x1)
            x1 = Newton(unconstrained_1).run(x0, observer=observer)

            # admm update
            delta = x0 - x1
            self.y += self.mu * delta

            # aula update
            h = self.pb.pb1.h.value(x1) if self.pb.pb1.h else 0
            g = self.pb.pb1.g.value(x1) if self.pb.pb1.g else 0

            self.lambda_h = self.lambda_h + 2 * self.mu * h
            self.lambda_g = self.lambda_g + 2 * self.mu * g

            self.mu *= self.nu

            print("IT={}, admm delta={}, h={}, lambda_h={}, g={}, lambda_g={}".format(i, delta, h, self.lambda_h, g, self.lambda_g))

            if np.abs(delta).max() < self.eps and np.abs(h) < self.eps and g < self.eps:
                break

            if i > 50:
                i = 0
                print("weird")

            i+=1

        return x1

class ADMMSolver_Newton:
    def __init__(self, pb, solver_class=Newton):
        self.pb = pb
        self.solver_class = solver_class
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

        x0 = x
        x1 = x
        i = 0
        while True:
            pb0 = self.to_0(self.pb, x1, self.y, self.rho)
            assert pb0.checkGradients(x1) and pb0.checkHessian(x1)
            x0 = self.solver_class(pb0).run(x1)

            pb1 = self.to_1(self.pb, x0, self.y, self.rho)
            assert pb1.checkGradients(x0) and pb1.checkHessian(x0)
            x1 = self.solver_class(pb1).run(x0)

            # admm update
            delta = x0 - x1
            self.y += self.rho * delta

            if np.abs(delta).max() < self.eps:
                break

        return x1
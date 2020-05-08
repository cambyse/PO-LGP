import numpy as np
from augmented_lagrangian_solver import AugmentedLagrangianSolver
from newton import NewtonFunction
from optimization_problems import UnconstrainedQP, ConstrainedQP, ConstrainedProblem

class UnconstrainedQPSolver:
    def __init__(self, qp):
        self.qp = qp

    def run(self):
        A = self.qp.Q
        B = -self.qp.c

        x = np.linalg.solve(A, B)

        return x

class ConstrainedQPSolver:
    def __init__(self, qp):
        self.qp = qp

    @staticmethod
    def to_f(qp):
        class F(NewtonFunction):
            def value(self, x):
                return 0.5 * np.dot(x.T, np.dot(qp.Q, x)) + np.dot(qp.c.T, x)

            def gradient(self, x):
                return np.dot(qp.Q, x) + qp.c

            def hessian(self, x):
                return qp.Q

        return F()

    @staticmethod
    def to_g(qp):
        class G(NewtonFunction):
            def value(self, x):
                return np.dot(qp.A, x) - qp.u

            def gradient(self, x):
                return qp.A

            def hessian(self, x):
                return np.zeros(qp.Q.shape)

        return G()

    def run(self, x):
        f = self.to_f(self.qp)
        g = self.to_g(self.qp)

        pb = ConstrainedProblem(f=f, g=g)

        al = AugmentedLagrangianSolver(pb)
        x =  al.run(x)

        return x
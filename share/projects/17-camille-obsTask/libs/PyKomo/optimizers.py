import numpy as np
import copy

def is_semi_pos_def(m):
    eigvals = np.linalg.eigvals(m)
    return np.all(eigvals >= 0)

class ConstrainedProblem:
    def __init__(self, f, h):
        self.f = f
        self.h = h

class SquarePenaltySolver:
    def __init__(self, pb):
        self.constrainedProblem = pb
        self.eps_h = 0.001 #max constraint violation
        self.mu = 1.0
        self.rho = 10 # how much we increase the sqzare penalty at each cycle (works also with one)

    @staticmethod
    def convert(pb, mu):
        class Augmented(NewtonFunction):
            def value(self, x):
                f = pb.f.value(x)
                b = pb.h.value(x) ** 2  # barrier
                return f + mu * b

            def gradient(self, x):
                jf = pb.f.gradient(x)
                h = pb.h.value(x)
                Jh = pb.h.gradient(x)
                jb = 2 * np.dot(Jh.T, h)
                return jf + mu * jb

            def hessian(self, x):
                Hf = pb.f.hessian(x)
                Jh = pb.h.gradient(x)
                Hb = 2 * np.dot(Jh.T, Jh)
                return Hf + mu * Hb

        return Augmented()

    def run(self, x):
        print("mu={}".format(self.mu))

        unconstrained = self.convert(self.constrainedProblem, self.mu)
        gn = Newton(unconstrained)
        x = gn.run(x)
        h = self.constrainedProblem.h.value(x)

        while np.abs(h) > self.eps_h:
            self.mu *= self.rho

            print("mu={}".format(self.mu))

            unconstrained = self.convert(self.constrainedProblem, self.mu)
            gn = Newton(unconstrained)
            x = gn.run(x)
            h = self.constrainedProblem.h.value(x)

        return x

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

class NewtonFunction:
    def value(self, x):
        pass

    def gradient(self, x):
        pass

    def hessian(self, x):
        pass

    def checkGradients(self, x):
        dx = 0.001
        j = self.gradient(x)
        y = self.value(x)

        close = True

        for i in range(0, x.shape[0]):
            x_ = copy.copy(x)
            x_[i] = x_[i] + dx
            y_ = self.value(x_)
            dy = y_ - y
            ji = dy / dx
            close = close and np.abs(j[i] -ji) < 0.01

        return close

class SquareCostFunction(NewtonFunction):
    def value(self, x):
        phi = self.phi(x)
        return np.dot(phi.T, phi)

    def gradient(self, x):
        phi = self.phi(x)
        Jphi = self.gradientPhi(x)
        return 2 * np.dot(Jphi.T, phi)

    def hessian(self, x): #pseudo hessian (neglects 2 phi.T * hessian(phi)
        Jphi = self.gradientPhi(x)
        return 2 * np.dot(Jphi.T, Jphi)

    def phi(self, x):
        pass

    def gradientPhi(self, x):
        pass

class Newton: # sum of square problems
    def __init__(self, function):
        self.function = function
        self.lambda_0 = 0.1
        self.rho = 0.01
        self.eps = 0.01  # update size

        assert issubclass(type(function), NewtonFunction), "wrong function type"

    def run(self, x):
        _lambda = self.lambda_0
        _alpha = 1

        I = np.identity(x.shape[0])
        while True:
            def matrix_preparation():
                hessian = self.function.hessian(x)
                A = hessian + _lambda * I # damping
                B = -self.function.gradient(x)
                return A, B

            A, B = matrix_preparation()
            d = np.linalg.solve(A, B)

            v = self.function.value(x)
            w = self.function.value(x + _alpha * d)   # line search
            while w > v + self.rho * np.matmul(np.transpose(B), _alpha * d):
                _alpha = _alpha * 0.5
                w = self.function.value(x + _alpha * d)

            x = x + _alpha * d
            _alpha = 1
            if np.linalg.norm(_alpha * d) < self.eps:
                break

        return x

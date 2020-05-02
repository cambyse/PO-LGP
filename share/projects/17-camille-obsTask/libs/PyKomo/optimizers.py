import numpy as np

def is_semi_pos_def(m):
    eigvals = np.linalg.eigvals(m)
    return np.all(eigvals >= 0)

class ConstrainedProblem:
    def __init__(self, f, g, h):
        self.f = f
        self.g = g
        self.h = h

def squarePenaltyConverter(pb, mu):
    def fuse(x):
        f, jf = pb.f(x)
        h, jh = pb.h(x)

        #gamma = f + mu * np.dot(h.T, h)
        #jgamma = jf + mu * 2 * h * jh

        gamma = np.vstack((f, mu * h))
        jgamma = np.vstack((jf, mu* jh))

        return gamma, jgamma

    return lambda x: fuse(x)

class SquarePenaltySolver:
    def __init__(self, pb):
        self.constrainedProblem = pb
        self.eps_h = 0.001 #max constraint violation
        self.mu = 1.0

    def run(self, x):
        print("mu={}".format(self.mu))

        unconstrained = squarePenaltyConverter(self.constrainedProblem, self.mu)
        gn = GaussNewton(unconstrained)
        x = gn.run(x)
        h, _ = self.constrainedProblem.h(x)

        while np.dot(h.T, h) > self.eps_h:
            self.mu *= 10

            print("mu={}".format(self.mu))

            unconstrained = squarePenaltyConverter(self.constrainedProblem, self.mu)
            gn = GaussNewton(unconstrained)
            x = gn.run(x)
            h, _ = self.constrainedProblem.h(x)

        return x

class GaussNewtonFunction:
    def value(self, x):
        pass

    def gradient(self, x):
        pass

class GaussNewton: # sum of square problems
    def __init__(self, function):
        self.function = function
        self.lambda_0 = 0.1
        self.rho = 0.01
        self.eps = 0.01  # update size

        assert issubclass(type(function), GaussNewtonFunction), "wrong function type"

    def run(self, x):
        _lambda = self.lambda_0
        _alpha = 1

        I = np.identity(x.shape[0])
        while True:
            gamma = self.function.value(x)
            Jgamma = self.function.gradient(x)

            def matrix_preparation():
                JgammaT = np.transpose(Jgamma)
                approxHessian = 2 * np.matmul(JgammaT, Jgamma)
                A = approxHessian + _lambda * I

                B = 2 * np.matmul(JgammaT, gamma)

                return A, B

            def solve():
                return np.linalg.solve(A, -B)

            A, B = matrix_preparation()
            d = solve()

            # import matplotlib.pyplot as plt
            # plt.matshow(Jgamma!=0)
            # plt.show()

            delta = self.function.value(x + _alpha * d)   # line search
            while np.dot(delta.T, delta) > np.dot(gamma.T, gamma) + self.rho * np.matmul(np.transpose(B), _alpha * d):
                _alpha = _alpha * 0.5
                delta = self.function.value(x + _alpha * d)  # line search

            x = x + _alpha * d
            _alpha = 1
            if np.linalg.norm(_alpha * d) < self.eps:
                break

        return x

class NewtonFunction:
    def value(self, x):
        pass

    def gradient(self, x):
        pass

    def hessian(self, x):
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
                A = hessian + _lambda * I
                B = 2 * self.function.gradient(x)
                return A, B

            def solve():
                return np.linalg.solve(A, -B)

            A, B = matrix_preparation()
            d = solve()

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

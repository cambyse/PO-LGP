import numpy as np
import copy

class NewtonFunction:
    def value(self, x):
        pass

    def gradient(self, x):
        pass

    def hessian(self, x):
        pass

    def checkGradients(self, x):
        dx = 0.001
        y = self.value(x)
        j = self.gradient(x)

        close = True

        for i in range(0, x.shape[0]):
            x_ = copy.copy(x)
            x_[i] += dx
            y_ = self.value(x_)
            dy = y_ - y
            ji = dy / dx
            close = close and np.abs(j[i] - ji) < 0.01

        return close

    def checkHessian(self, x):
        dx = 0.001
        h  = self.hessian(x)

        close = True

        y = self.value(x)
        for i in range(0, x.shape[0]):
            xp = copy.copy(x)
            xp[i] += dx
            xm = copy.copy(x)
            xm[i] -= dx
            yp = self.value(xp)
            ym = self.value(xm)
            hii = (ym - 2*y + yp) / (dx * dx)
            close = close and np.abs(h[i, i] - hii) < 0.01

        return close

class Newton: # sum of square problems
    def __init__(self, function):
        self.function = function
        self.lambda_0 = 0.1
        self.rho = 0.01
        self.eps = 0.01  # update size

        assert issubclass(type(function), NewtonFunction), "wrong function type"

    def run(self, x, observer=None):
        _lambda = self.lambda_0
        _alpha = 1

        if observer:
            observer.new_newton_run(x)

        I = np.identity(x.shape[0])
        while True:
            hessian = self.function.hessian(x)
            A = hessian + _lambda * I # damping
            B = -self.function.gradient(x)

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

        if observer:
            observer.add(x)

        return x

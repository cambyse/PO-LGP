import numpy as np

def is_semi_pos_def(m):
    eigvals = np.linalg.eigvals(m)
    return np.all( eigvals >= 0)


class GaussNewton:
    def __init__(self, function):
        self.function = function
        self.lambda_0 = 0.1
        self.rho = 0.01
        self.eps = 0.01  # update size

    def run(self, x):
        _lambda = self.lambda_0
        _alpha = 1

        I = np.identity(x.shape[0])
        while True:
            gamma, Jgamma = self.function(x)

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

            delta, _ = self.function(x + _alpha * d)   # line search
            while np.dot(delta.T, delta) > np.dot(gamma.T, gamma) + self.rho * np.matmul(np.transpose(B), _alpha * d):
                _alpha = _alpha * 0.5
                delta, _ = self.function(x + _alpha * d)

            x = x + _alpha * d
            _alpha = 1
            if np.linalg.norm(_alpha * d) < self.eps:
                break

        return x

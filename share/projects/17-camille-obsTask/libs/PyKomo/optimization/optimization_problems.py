import numpy as np

class ConstrainedProblem:
    def __init__(self, f, h):
        self.f = f
        self.h = h

class UnconstrainedQP:
    def __init__(self, Q, c):
        self.Q = Q  # hessian
        self.c = c

def is_semi_pos_def(m):
    eigvals = np.linalg.eigvals(m)
    return np.all(eigvals >= 0)
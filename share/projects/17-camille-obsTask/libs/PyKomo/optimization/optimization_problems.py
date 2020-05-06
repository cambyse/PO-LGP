import numpy as np

class ConstrainedProblem:
    def __init__(self, f, h=None, g=None):
        self.f = f
        self.h = h
        self.g = g

class UnconstrainedQP:
    def __init__(self, Q, c):
        self.Q = Q  # hessian
        self.c = c

class ConstrainedQP: # Ax <= u
    def __init__(self, Q, c, A, u):
        self.Q = Q  # hessian
        self.c = c
        self.A = A
        self.u = u

def is_semi_pos_def(m):
    eigvals = np.linalg.eigvals(m)
    return np.all(eigvals >= 0)
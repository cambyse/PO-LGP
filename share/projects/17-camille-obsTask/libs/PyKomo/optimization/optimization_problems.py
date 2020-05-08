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

class ADMMProblem:
    def __init__(self, f0, f1):
        self.f0 = f0
        self.f1 = f1

def is_semi_pos_def(m):
    eigvals = np.linalg.eigvals(m)
    return np.all(eigvals >= 0)


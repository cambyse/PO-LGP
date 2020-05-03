import numpy as np

class UnconstrainedQPSolver:
    def __init__(self, qp):
        self.qp = qp

    def run(self):
        A = self.qp.Q
        B = -self.qp.c

        x = np.linalg.solve(A, B)

        return x
import nose.tools as nt
import sys
import numpy.testing as npt
import numpy as np
from pathlib import Path
sys.path.append(str(Path('.').absolute().parent))
from optimization_problems import UnconstrainedQP, ConstrainedQP, ConstrainedProblem
from qp_solver import UnconstrainedQPSolver, ConstrainedQPSolver
from nose.tools import nottest

def test_unconstrained_qp():
    # min x^2 - x
    qp = UnconstrainedQP(Q=np.array([[2.0]]), c=np.array([-1.0]))
    solver = UnconstrainedQPSolver(qp)
    x = solver.run()

    nt.assert_almost_equals(x, 0.5, delta=0.001)

@nottest
def test_constrained_qp():
    # min x^2 - x
    # s.t. x <= 0
    qp = ConstrainedQP(Q=np.array([[2.0]]), c=np.array([-1.0]), A=np.array([[1.0]]), u=np.array([0.2]))
    solver = ConstrainedQPSolver(qp)
    x = solver.run(np.array([-1.0]))

    nt.assert_almost_equals(x, 0.2, delta=0.001)


if __name__ == "__main__":
     test_unconstrained_qp()
import nose.tools as nt
import sys
import numpy.testing as npt
import numpy as np
from pathlib import Path
sys.path.append(str(Path('.').absolute().parent))
from optimization_problems import UnconstrainedQP
from qp_solver import UnconstrainedQPSolver

def test_dummy_qp():
    qp = UnconstrainedQP(Q=np.array([[2.0]]), c=np.array([[1.0]]))
    solver = UnconstrainedQPSolver(qp)
    x = solver.run()

    nt.assert_almost_equals(x, -0.5, 0.0001)

if __name__ == "__main__":
     test_dummy_qp()
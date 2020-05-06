import nose.tools as nt
import sys
import numpy.testing as npt
import numpy as np
from pathlib import Path
sys.path.append(str(Path('.').absolute().parent))
from augmented_lagrangian_solver import AugmentedLagrangianSolverEq, AugmentedLagrangianSolverIneq
from optimization_problems import ConstrainedProblem
from functions import SquareDistance, ProjX, ProjY

def test_gradients_aula_eq():
    x0 = np.array([1.0, 1.0])

    pb = ConstrainedProblem(f=SquareDistance(), h=ProjX())
    al = AugmentedLagrangianSolverEq(pb)
    al.lambda_ = 1.0
    unconstrained = al.convert(al.constrainedProblem, al.mu, al.lambda_)

    nt.assert_true(unconstrained.checkGradients(x0))

def test_constrained_aula_eq():
    x0 = np.array([1.0, 1.0])

    pb = ConstrainedProblem(f=SquareDistance(), h=ProjX())
    al = AugmentedLagrangianSolverEq(pb)
    x = al.run(x0)

    npt.assert_almost_equal(x, np.array([0.0, 2.0]), decimal=1)
    nt.assert_almost_equals(x[0], 0, delta=0.001)

def test_gradients_aula_ineq():
    x0 = np.array([1.0, 1.0])

    pb = ConstrainedProblem(f=SquareDistance(), g=ProjY())
    al = AugmentedLagrangianSolverIneq(pb)
    al.lambda_ = 1.0
    unconstrained = al.convert(al.constrainedProblem, al.mu, al.lambda_)

    nt.assert_true(unconstrained.checkGradients(x0))

def test_constrained_aula_ineq_active_constraint():
    x0 = np.array([1.0, 1.0])

    pb = ConstrainedProblem(f=SquareDistance(), g=ProjY())
    al = AugmentedLagrangianSolverIneq(pb)
    x = al.run(x0)

    npt.assert_almost_equal(x, np.array([10.0, 0.0]), decimal=1)
    nt.assert_almost_equals(x[1], 0, delta=0.001)

def test_constrained_aula_ineq_inactive_constraint():
    x0 = np.array([1.0, -1.0])

    pb = ConstrainedProblem(f=SquareDistance(py=-1.0), g=ProjY())
    al = AugmentedLagrangianSolverIneq(pb)
    x = al.run(x0)

    npt.assert_almost_equal(x, np.array([10.0, -1.0]), decimal=1)
    nt.assert_almost_equals(x[1], -1.0, delta=0.001)

if __name__ == "__main__":
     test_constrained_squared_penalty()
     test_constrained_aula_eq()
     test_constrained_aula_ineq_inactive_constraint()

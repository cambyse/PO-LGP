import nose.tools as nt
import sys
import numpy.testing as npt
import numpy as np
from pathlib import Path
sys.path.append(str(Path('.').absolute().parent))
from augmented_lagrangian_solver import AugmentedLagrangianSolver, Lagrangian
from optimization_problems import ConstrainedProblem
from functions import SquareDistance, ProjX, ProjY, Plotter2D

def test_gradients_aula_eq():
    x0 = np.array([1.0, 1.0])

    pb = ConstrainedProblem(f=SquareDistance(), h=ProjX())
    al = AugmentedLagrangianSolver(pb)
    al.lambda_h = 1.0
    lagrangian = Lagrangian(al.constrainedProblem, mu=al.mu, lambda_h=al.lambda_h)

    nt.assert_true(lagrangian.checkGradients(x0))
    nt.assert_true(lagrangian.checkHessian(x0))

def test_constrained_aula_eq():
    x0 = np.array([1.0, 1.0])

    pb = ConstrainedProblem(f=SquareDistance(), h=ProjX())
    al = AugmentedLagrangianSolver(pb)
    x = al.run(x0)

    npt.assert_almost_equal(x, np.array([0.0, 2.0]), decimal=1)
    nt.assert_almost_equals(x[0], 0, delta=0.001)

def test_constrained_aula_eq_no_constraint():
    x0 = np.array([1.0, 1.0])

    pb = ConstrainedProblem(f=SquareDistance())
    al = AugmentedLagrangianSolver(pb)
    x = al.run(x0)

    npt.assert_almost_equal(x, np.array([10.0, 2.0]), decimal=1)

def test_gradients_aula_ineq_active_constraint():
    x0 = np.array([1.0, 1.0])

    pb = ConstrainedProblem(f=SquareDistance(), g=ProjY())
    al = AugmentedLagrangianSolver(pb)
    al.lambda_ = 1.0
    lagrangian = Lagrangian(al.constrainedProblem, mu=al.mu, lambda_g=al.lambda_g)

    nt.assert_true(lagrangian.checkGradients(x0))
    nt.assert_true(lagrangian.checkHessian(x0))

def test_constrained_aula_ineq_active_constraint():
    p = Plotter2D()

    x0 = np.array([1.0, 1.0])
    p.add(x0)

    pb = ConstrainedProblem(f=SquareDistance(), g=ProjY())
    al = AugmentedLagrangianSolver(pb)
    x = al.run(x0)

    npt.assert_almost_equal(x, np.array([10.0, 0.0]), decimal=1)
    nt.assert_almost_equals(x[1], 0, delta=0.001)

    #p.plot()

def test_gradients_aula_ineq_inactive_constraint():
    x0 = np.array([1.0, 1.0])

    pb = ConstrainedProblem(f=SquareDistance(cy=-1.0), g=ProjY())
    al = AugmentedLagrangianSolver(pb)
    lagrangian = Lagrangian(al.constrainedProblem, mu=al.mu, lambda_g=al.lambda_g)

    nt.assert_true(lagrangian.checkGradients(x0))

def test_gradients_aula_ineq_no_constraint():
    x0 = np.array([1.0, 1.0])

    pb = ConstrainedProblem(f=SquareDistance(cy=-1.0))
    al = AugmentedLagrangianSolver(pb)
    unconstrained = Lagrangian(al.constrainedProblem, mu=al.mu, lambda_g=al.lambda_g)

    nt.assert_true(unconstrained.checkGradients(x0))

def test_constrained_aula_ineq_inactive_constraint():
    x0 = np.array([1.0, -1.0])

    pb = ConstrainedProblem(f=SquareDistance(cy=-1.0), g=ProjY())
    al = AugmentedLagrangianSolver(pb)
    x = al.run(x0)

    npt.assert_almost_equal(x, np.array([10.0, -1.0]), decimal=1)
    nt.assert_almost_equals(x[1], -1.0, delta=0.001)

if __name__ == "__main__":
     test_constrained_squared_penalty()
     test_constrained_aula_eq()
     test_constrained_aula_ineq_inactive_constraint()

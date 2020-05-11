import nose.tools as nt
import sys
import numpy.testing as npt
import numpy as np
from pathlib import Path
sys.path.append(str(Path('.').absolute().parent))
from gauss_newton import SquareCostFunction
from optimization_problems import ADMMProblem_Newton, ADMMProblem, ConstrainedProblem
from admm_solver import ADMMSolver_Newton, ADMMSolver
from augmented_lagrangian_solver import AugmentedLagrangianSolver
from functions import ProjX

class SquareDistance3D(SquareCostFunction):
    def __init__(self, px=10, py=2, pz=1):
        self.px = px
        self.py = py
        self.pz = pz

    def phi(self, x):
        dx = x[0] - self.px
        dy = x[1] - self.py
        dz = x[2] - self.pz
        return np.asarray([dx, dy, dz])

    def gradientPhi(self, x):
        return np.asarray([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])

class SquareDistance3DDecomp0(SquareCostFunction):
    def __init__(self, px=10, py=2):
        self.px = px
        self.py = py
        self.x_factor = np.sqrt(0.5)

    def phi(self, x):
        dx = x[0] - self.px
        dy = x[1] - self.py
        return np.asarray([self.x_factor * dx, dy, 0])

    def gradientPhi(self, x):
        return np.asarray([[self.x_factor, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 0.0]])

class SquareDistance3DDecomp1(SquareCostFunction):
    def __init__(self, px=10, pz=1):
        self.px = px
        self.pz = pz
        self.x_factor = np.sqrt(0.5)

    def phi(self, x):
        dx = x[0] - self.px
        dz = x[2] - self.pz
        return np.asarray([self.x_factor * dx, 0, dz])

    def gradientPhi(self, x):
        return np.asarray([[self.x_factor, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 1.0]])

def test_distance_3d():
    f = SquareDistance3D(1, 1, 1)
    v = f.value(np.array([0, 0, 0]))
    nt.assert_almost_equals(v, 3, 0.0001)

def test_distance_3d_decomp():
    f0 = SquareDistance3DDecomp0(1, 1)
    f1 = SquareDistance3DDecomp1(1, 1)
    v0 = f0.value(np.array([0, 0, 0]))
    v1 = f1.value(np.array([0, 0, 0]))

    nt.assert_true(f0.checkGradients(np.array([0.0, 0.0, 0.0])))
    nt.assert_true(f1.checkGradients(np.array([0.0, 0.0, 0.0])))
    nt.assert_true(f0.checkHessian(np.array([0.0, 0.0, 0.0])))
    nt.assert_true(f1.checkHessian(np.array([0.0, 0.0, 0.0])))

    nt.assert_almost_equals(v0 + v1, 3, 0.0001)

def test_dist_3d_minimization():
    f0 = SquareDistance3DDecomp0(1, 1)
    f1 = SquareDistance3DDecomp1(1, 1)
    pb = ADMMProblem_Newton(f0=f0, f1=f1)
    solver = ADMMSolver_Newton(pb)
    x = solver.run(np.array([0.0, 0.0, 0.0]))
    npt.assert_almost_equal(x, np.array([1.0, 1.0, 1.0]), decimal=1)

def test_foo():
    pb0 = ConstrainedProblem(f=SquareDistance3DDecomp0(1, 1), h=ProjX())
    pb1 = ConstrainedProblem(f= SquareDistance3DDecomp1(1, 1))
    pb = ADMMProblem(pb0=pb0, pb1=pb1)
    solver = ADMMSolver(pb, solver_class=AugmentedLagrangianSolver)
    x = solver.run(np.array([0.0, 0.0, 0.0]))
    npt.assert_almost_equal(x, np.array([0.0, 1.0, 1.0]), decimal=1)
    nt.assert_almost_equals(x[0], 0, delta=0.001)

if __name__ == "__main__":
     test_distance_3d()
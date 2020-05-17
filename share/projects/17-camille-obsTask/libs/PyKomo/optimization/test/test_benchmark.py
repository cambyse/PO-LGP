import nose.tools as nt
import sys
import numpy.testing as npt
import numpy as np
from pathlib import Path
sys.path.append(str(Path('.').absolute().parent))
from optimization_problems import ADMMProblem_Newton, ADMMProblem, ConstrainedProblem
from decentralized_aula import DecentralizedAugmentedLagrangianSolver
from augmented_lagrangian_solver import AugmentedLagrangianSolver
from admm_solver import ADMMSolver
from functions import *
from observers import *

def test_dist_3d_planar_constraint():
    x0 = np.array([0.0, 0.0, 0.0])

    pb = ConstrainedProblem(f=SquareDistance3D(1, 1, 1), h=ProjX())

    pb0 = ConstrainedProblem(f=SquareDistance3DDecomp0(1, 1), h=ProjX())
    pb1 = ConstrainedProblem(f=SquareDistance3DDecomp1(1, 1), h=ProjX())
    admm_pb = ADMMProblem(pb0=pb0, pb1=pb1)

    pbs = {
        AugmentedLagrangianSolver: pb,
        ADMMSolver: admm_pb,
        DecentralizedAugmentedLagrangianSolver: admm_pb
    }

    for solver_class, pb in pbs.items():
        p = Plotter3D(solver_class.__name__)
        p.add_point(x0)

        solver = solver_class(pb)
        x = solver.run(x0, observer=p)

        npt.assert_almost_equal(x, np.array([0.0, 1.0, 1.0]), decimal=1)

        p.report()

def test_dist_5d_planar_constraint():
    x0 = np.array([0.0, 0.0, 0.0, 0.0, 0.0])

    pb = ConstrainedProblem(f=SquareDistanceND(np.array([1, 1, 1, 1, 1])), h=ProjX())

    pbs = {
        AugmentedLagrangianSolver: pb,
    }

    for solver_class, pb in pbs.items():
        p = Plotter3D(solver_class.__name__)
        p.add_point(x0)

        solver = solver_class(pb)
        x = solver.run(x0, observer=p)

        npt.assert_almost_equal(x, np.array([0.0, 1.0, 1.0, 1.0, 1.0]), decimal=1)

        p.report()
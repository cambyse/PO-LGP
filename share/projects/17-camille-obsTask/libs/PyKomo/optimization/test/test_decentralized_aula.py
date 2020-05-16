import nose.tools as nt
import sys
import numpy.testing as npt
import numpy as np
from pathlib import Path
sys.path.append(str(Path('.').absolute().parent))
from optimization_problems import ADMMProblem_Newton, ADMMProblem, ConstrainedProblem
from decentralized_aula import DecentralizedAugmentedLagrangianSolver
from functions import *
from observers import *

def test_unconstrained_dec_aula_3d():
    x0 = np.array([0.0, 0.0, 0.0])

    p = Plotter3D("dec aula (unconstrained)")
    p.add_point(x0)

    pb0 = ConstrainedProblem(f=SquareDistance3DDecomp0(1, 1))
    pb1 = ConstrainedProblem(f= SquareDistance3DDecomp1(1, 1))
    pb = ADMMProblem(pb0=pb0, pb1=pb1)
    solver = DecentralizedAugmentedLagrangianSolver(pb)
    x = solver.run(x0, observer=p)

    npt.assert_almost_equal(x, np.array([1.0, 1.0, 1.0]), decimal=1)

    p.report()


def test_constrained_dec_aula_3d():
    x0 = np.array([0.0, 0.0, 0.0])

    p = Plotter3D("decentralized aula_(h:x=0)")
    p.add_point(x0)

    pb0 = ConstrainedProblem(f=SquareDistance3DDecomp0(1, 1), h=ProjX())
    pb1 = ConstrainedProblem(f=SquareDistance3DDecomp1(1, 1), h=ProjX())
    pb = ADMMProblem(pb0=pb0, pb1=pb1)
    solver = DecentralizedAugmentedLagrangianSolver(pb)
    x = solver.run(x0, observer=p)

    npt.assert_almost_equal(x, np.array([0.0, 1.0, 1.0]), decimal=1)

    p.report(plot=True)


def test_constrained_dec_aula_3d_sphere():
    x0 = np.array([0.0, 0.0, 0.0])

    p = Plotter3D("decentralized aula_(h:sphere)")
    p.add_point(x0)

    h = SphereConstraint3D(cx=0, cy=0.5, cz=0.5, radius=0.5)
    pb0 = ConstrainedProblem(f=SquareDistance3DDecomp0(1, 1), h=h)
    pb1 = ConstrainedProblem(f=SquareDistance3DDecomp1(1, 1), h=h)
    pb = ADMMProblem(pb0=pb0, pb1=pb1)
    solver = DecentralizedAugmentedLagrangianSolver(pb)
    x = solver.run(x0, observer=p)

    nt.assert_almost_equals(h.value(x), 0, delta=0.001)

    p.report(plot=True)
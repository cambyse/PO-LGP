import nose.tools as nt
import sys
import numpy.testing as npt
import numpy as np
from pathlib import Path
sys.path.append(str(Path('.').absolute().parent))
from optimizers import GaussNewton, ConstrainedProblem, squarePenaltyConverter, SquarePenaltySolver

SHOW_PLOTS = True

def affine(x):
    return np.asarray([[x[0, 0]]]), np.asarray([[1.0, 0.0]])

def dist2d(x):
    # dist from center at (10, 0) in 2d
    dx = x[0, 0] - 10
    dy = x[1, 0]
    gamma = np.sqrt(dx*dx+ dy*dy)
    costheta = dx / gamma
    sintheta = dy / gamma
    return np.asarray([[gamma]]), np.asarray([[costheta, sintheta]])

def test_conversion_execution():
    # minimize dist from center in 2d
    # 2 input variable, one cost: 2->1
    pb = ConstrainedProblem(f=dist2d, h=affine, g=None)

    x = np.array([[1.0, 0]]).T

    f = squarePenaltyConverter(pb, 2.0)

    #print("f={}".format(pb.f(x)))
    #print("h={}".format(pb.h(x)))
    #print("f+={}".format(f(x)))


def test_constrained_opt():
    # minimize dist from center in 2d
    # 2 input variable, one cost: 2->1
    pb = ConstrainedProblem(f=dist2d, h=affine, g=None)

    x0 = np.array([[1.0, 1.0]]).T

    sq = SquarePenaltySolver(pb)
    x = sq.run(x0)

    npt.assert_almost_equal(x, np.array([[0.0, 0.0]]).T, 0.0001)

if __name__ == "__main__":
     test_constrained_opt()